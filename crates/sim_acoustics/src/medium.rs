//! The water shared by several boards: their projectors, heard by each other's hydrophones.
//!
//! A board's projector is fed from one of its output pins. The pin's level changes are recorded with their times;
//! the drive voltage (the pin through a driver stage of `supply_v`) is averaged over each step of the medium's
//! sample grid (1 MHz) and run through the transducer's transmitting response, a band-pass resonator when the TVR
//! is given as a resonance. The result is the pressure the projector radiates, in µPa at 1 m, kept for the last
//! [`HISTORY_S`] so hydrophones can read it back at whatever delay each propagation path has.
//!
//! Boards run in turns (see `sim_host::robot`), so a projector's waveform is only known up to the time its board
//! has reached. Each board reports its progress; samples are computed and cached only once the whole step is known,
//! and a hydrophone asking further ahead gets the last known value without it being cached. Any path longer than
//! the robot's slice (100 µs, 15 cm of water) therefore always reads exact values.
//!
//! Vehicles can move ([`Trajectory`]): the hydrophone recomputes its paths every millisecond and interpolates the
//! delays in between, so a closing or opening range shifts the received frequency (Doppler) and timing.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use serde::Deserialize;
use sim_core::time::Time;

use crate::env::Environment;
use crate::geometry::Vec3;
use crate::transducer::{Response, Transducer};

/// Projector pressure history kept for hydrophones to read back.
pub const HISTORY_S: f64 = 0.25;
/// Sample rate of the projector waveforms.
pub const RATE_HZ: f64 = 1_000_000.0;
const STEP_PS: u64 = 1_000_000;

pub type SharedMedium = Arc<Mutex<Medium>>;

/// Where a transducer is over time.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum Trajectory {
    Fixed(Vec3),
    /// Constant velocity from a starting point.
    Linear {
        start_m: Vec3,
        velocity_mps: Vec3,
    },
    /// Straight lines between `(time s, position m)` points; holds the first and last position outside them.
    Waypoints(Vec<(f64, Vec3)>),
}

impl Trajectory {
    pub fn at(&self, t: f64) -> Vec3 {
        match self {
            Trajectory::Fixed(p) => *p,
            Trajectory::Linear { start_m, velocity_mps } => {
                [start_m[0] + velocity_mps[0] * t, start_m[1] + velocity_mps[1] * t, start_m[2] + velocity_mps[2] * t]
            }
            Trajectory::Waypoints(points) => {
                let Some(first) = points.first() else { return [0.0; 3] };
                if t <= first.0 {
                    return first.1;
                }
                for w in points.windows(2) {
                    let ((t0, p0), (t1, p1)) = (w[0], w[1]);
                    if t <= t1 {
                        let x = if t1 > t0 { (t - t0) / (t1 - t0) } else { 1.0 };
                        return [p0[0] + (p1[0] - p0[0]) * x, p0[1] + (p1[1] - p0[1]) * x, p0[2] + (p1[2] - p0[2]) * x];
                    }
                }
                points.last().unwrap().1
            }
        }
    }

    pub fn moves(&self) -> bool {
        match self {
            Trajectory::Fixed(_) => false,
            Trajectory::Linear { velocity_mps, .. } => velocity_mps.iter().any(|v| *v != 0.0),
            Trajectory::Waypoints(p) => p.windows(2).any(|w| w[0].1 != w[1].1),
        }
    }
}

/// How a projector is driven, as written in a board file (`[transducer.drive]`).
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveSpec {
    /// The output pin.
    pub pin: u8,
    /// The driver stage's supply: the transducer sees 0 V for a low pin and this for a high one.
    pub supply_v: f64,
    /// Frequency used for the narrowband parts of the path (absorption, sensitivity, beams); the TVR's resonance
    /// by default.
    pub nominal_hz: Option<f64>,
}

/// Second-order band-pass with unit gain at its centre.
#[derive(Clone, Debug)]
struct Resonator {
    b0: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
}

impl Resonator {
    fn new(hz: f64, q: f64) -> Self {
        let w0 = 2.0 * std::f64::consts::PI * hz / RATE_HZ;
        let alpha = w0.sin() / (2.0 * q);
        let a0 = 1.0 + alpha;
        Resonator {
            b0: alpha / a0,
            a1: -2.0 * w0.cos() / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * (x - self.x2) - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// A board's projector in the medium.
#[derive(Clone, Debug)]
pub struct Projector {
    /// Which board drives it (its hydrophone doesn't hear it through the water).
    pub owner: usize,
    pub name: String,
    pub transducer: Transducer,
    pub trajectory: Trajectory,
    pub nominal_hz: f64,
    drive_v: f64,
    /// µPa at 1 m per volt of drive at the nominal frequency (the TVR's peak for a resonance).
    gain: f64,
    resonator: Resonator,
    /// Level changes not yet folded into samples, oldest first.
    edges: VecDeque<(Time, bool)>,
    level: bool,
    progress: Time,
    /// Grid index of `samples[0]`.
    base: u64,
    samples: VecDeque<f32>,
    /// Number of drive level changes seen.
    pub edge_count: u64,
}

impl Projector {
    fn computed(&self) -> u64 {
        self.base + self.samples.len() as u64
    }

    /// Computes grid steps up to and including `k`, as far as the drive is known.
    fn compute_to(&mut self, k: u64) {
        while self.computed() <= k {
            let n = self.computed();
            let start = Time(n * STEP_PS);
            let end = Time((n + 1) * STEP_PS);
            if end > self.progress {
                return;
            }
            // Fraction of the step the pin was high
            let mut high = 0u64;
            let mut t = start;
            while let Some(&(te, lv)) = self.edges.front() {
                if te >= end {
                    break;
                }
                let te = te.max(start);
                if self.level {
                    high += (te - t).as_ps();
                }
                t = te;
                self.level = lv;
                self.edges.pop_front();
            }
            if self.level {
                high += (end - t).as_ps();
            }
            let v = self.drive_v * high as f64 / STEP_PS as f64;
            let p = self.resonator.process(v) * self.gain;
            self.samples.push_back(p as f32);
            if self.samples.len() as f64 > HISTORY_S * RATE_HZ {
                self.samples.pop_front();
                self.base += 1;
            }
        }
    }

    fn sample(&mut self, k: u64) -> f64 {
        if k < self.base {
            return 0.0;
        }
        self.compute_to(k);
        let i = (k - self.base) as usize;
        match self.samples.get(i) {
            Some(v) => *v as f64,
            None => self.samples.back().copied().unwrap_or(0.0) as f64,
        }
    }

    /// Radiated pressure at `t` seconds (µPa at 1 m), interpolated between grid steps.
    pub fn pressure(&mut self, t: f64) -> f64 {
        if t < 0.0 {
            return 0.0;
        }
        let x = t * RATE_HZ;
        let k = x.floor() as u64;
        let f = x - k as f64;
        let a = self.sample(k);
        let b = self.sample(k + 1);
        a + (b - a) * f
    }
}

/// The shared water: an environment and the projectors of the boards in it.
#[derive(Clone, Debug)]
pub struct Medium {
    pub env: Arc<Environment>,
    pub projectors: Vec<Projector>,
}

impl Medium {
    pub fn new(env: Arc<Environment>) -> Medium {
        Medium { env, projectors: Vec::new() }
    }

    pub fn shared(env: Arc<Environment>) -> SharedMedium {
        Arc::new(Mutex::new(Medium::new(env)))
    }

    /// Adds a projector for board `owner`; returns its index.
    pub fn add_projector(
        &mut self,
        owner: usize,
        name: &str,
        transducer: Transducer,
        trajectory: Trajectory,
        drive: &DriveSpec,
    ) -> usize {
        let (resonator, gain, nominal) = match &transducer.tvr {
            Some(Response::Resonance { resonance_hz, q, peak_db }) => {
                (Resonator::new(*resonance_hz, *q), 10f64.powf(peak_db / 20.0), *resonance_hz)
            }
            other => {
                // A broadband response: a wide band-pass around the nominal frequency stops DC, at the TVR there
                let hz = drive.nominal_hz.unwrap_or(30_000.0);
                let db = other.as_ref().map_or(f64::NEG_INFINITY, |r| r.at(hz));
                (Resonator::new(hz, 0.5), 10f64.powf(db / 20.0), hz)
            }
        };
        let drive_v = match transducer.max_drive_v {
            Some(max) => drive.supply_v.min(max),
            None => drive.supply_v,
        };
        self.projectors.push(Projector {
            owner,
            name: name.to_string(),
            transducer,
            trajectory,
            nominal_hz: drive.nominal_hz.unwrap_or(nominal),
            drive_v,
            gain,
            resonator,
            edges: VecDeque::new(),
            level: false,
            progress: Time::ZERO,
            base: 0,
            samples: VecDeque::new(),
            edge_count: 0,
        });
        self.projectors.len() - 1
    }

    /// Projector `id`'s drive pin changed to `level` at `t`.
    pub fn edge(&mut self, id: usize, t: Time, level: bool) {
        let p = &mut self.projectors[id];
        p.edges.push_back((t, level));
        p.progress = p.progress.max(t);
        p.edge_count += 1;
    }

    /// Board `owner` has run up to `t`: its projectors' drive is known up to then.
    pub fn progress(&mut self, owner: usize, t: Time) {
        for p in self.projectors.iter_mut().filter(|p| p.owner == owner) {
            p.progress = p.progress.max(t);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::Water;

    fn medium() -> Medium {
        Medium::new(Arc::new(Environment::open_water(Water::default())))
    }

    fn projector(m: &mut Medium) -> usize {
        let t: Transducer = toml::from_str("tvr = { resonance_hz = 50_000, q = 8, peak_db = 140 }").unwrap();
        m.add_projector(0, "p", t, Trajectory::Fixed([0.0; 3]), &DriveSpec { pin: 7, supply_v: 12.0, nominal_hz: None })
    }

    /// Drives a square wave at `hz` from 0 to `until`.
    fn square(m: &mut Medium, id: usize, hz: f64, until: Time) {
        let half = Time::from_ps((1e12 / hz / 2.0) as u64);
        let mut t = Time::ZERO;
        let mut level = true;
        while t < until {
            m.edge(id, t, level);
            level = !level;
            t += half;
        }
        m.progress(0, until);
    }

    fn peak(m: &mut Medium, id: usize, from: f64, to: f64) -> f64 {
        let mut p = 0f64;
        let mut t = from;
        while t < to {
            p = p.max(m.projectors[id].pressure(t).abs());
            t += 1e-6;
        }
        p
    }

    #[test]
    fn square_wave_at_resonance_gives_the_tvr_level() {
        let mut m = medium();
        let id = projector(&mut m);
        square(&mut m, id, 50_000.0, Time::from_ms(5));
        // Fundamental of a 0-12 V square wave: 4/pi * 6 V peak; at resonance 140 dB re 1 uPa/V
        let expected = 4.0 / std::f64::consts::PI * 6.0 * 1e7;
        let p = peak(&mut m, id, 3e-3, 4e-3);
        assert!((p / expected - 1.0).abs() < 0.05, "{p:e} vs {expected:e}");
        // Off resonance it is much quieter
        let mut m2 = medium();
        let id2 = projector(&mut m2);
        square(&mut m2, id2, 30_000.0, Time::from_ms(5));
        assert!(peak(&mut m2, id2, 3e-3, 4e-3) < expected / 3.0);
    }

    #[test]
    fn samples_wait_for_the_board_to_get_there() {
        let mut m = medium();
        let id = projector(&mut m);
        m.edge(id, Time::from_us(100), true);
        // Known only to 100 us: asking for 200 us returns the last value without caching it
        let early = m.projectors[id].pressure(200e-6);
        assert_eq!(m.projectors[id].computed(), 100);
        m.progress(0, Time::from_us(300));
        let late = m.projectors[id].pressure(200e-6);
        assert!(early.abs() < 1.0 && late.abs() > 1e5, "{early} {late}");
        assert!(m.projectors[id].computed() >= 201);
    }

    #[test]
    fn trajectories() {
        let w = Trajectory::Waypoints(vec![(1.0, [0.0, 0.0, 1.0]), (3.0, [4.0, 0.0, 1.0])]);
        assert_eq!(w.at(0.0), [0.0, 0.0, 1.0]);
        assert_eq!(w.at(2.0), [2.0, 0.0, 1.0]);
        assert_eq!(w.at(9.0), [4.0, 0.0, 1.0]);
        assert!(w.moves());
        let l = Trajectory::Linear { start_m: [1.0, 2.0, 3.0], velocity_mps: [1.0, 0.0, 0.0] };
        assert_eq!(l.at(2.0), [3.0, 2.0, 3.0]);
        assert!(!Trajectory::Fixed([0.0; 3]).moves());
        let t: Trajectory =
            toml::from_str::<toml::Table>("t = [1.0, 2.0, 3.0]").unwrap()["t"].clone().try_into().unwrap();
        assert_eq!(t, Trajectory::Fixed([1.0, 2.0, 3.0]));
    }
}

#[cfg(test)]
mod link_tests {
    use super::*;
    use crate::chain::ChainSpec;
    use crate::hydrophone::Hydrophone;
    use crate::water::Water;

    fn setup(rx_traj: Trajectory) -> (SharedMedium, usize, Hydrophone) {
        let mut env = Environment::open_water(Water { sound_speed_mps: Some(1500.0), ..Water::default() });
        env.noise = crate::noise::Noise::None;
        let medium = Medium::shared(Arc::new(env));
        let t: Transducer =
            toml::from_str("tvr = { resonance_hz = 50_000, q = 8, peak_db = 140 }\nrvs_db = -190").unwrap();
        let id = medium.lock().unwrap().add_projector(
            0,
            "tx",
            t.clone(),
            Trajectory::Fixed([0.0, 0.0, 1.0]),
            &DriveSpec { pin: 7, supply_v: 12.0, nominal_hz: None },
        );
        let chain = ChainSpec { gain_db: 40.0, bandpass_hz: Some([40_000.0, 60_000.0]), ..Default::default() };
        let env = medium.lock().unwrap().env.clone();
        let h = Hydrophone::new(env, [0.0; 3], t, chain).in_medium(medium.clone(), 1).moving(rx_traj);
        (medium, id, h)
    }

    fn drive(m: &SharedMedium, id: usize, hz: f64, from: Time, until: Time) {
        let half = Time::from_ps((1e12 / hz / 2.0) as u64);
        let mut t = from;
        let mut level = true;
        let mut m = m.lock().unwrap();
        while t < until {
            m.edge(id, t, level);
            level = !level;
            t += half;
        }
        m.edge(id, until, false);
        m.progress(0, until + Time::from_ms(100));
    }

    /// Samples at 500 kS/s: (peak deviation from the bias, frequency from zero crossings).
    fn listen(h: &mut Hydrophone, from: Time, ms: u64) -> (f64, f64) {
        let n = ms * 500;
        let xs: Vec<f64> = (0..n).map(|i| h.sample(from + Time::from_us(2 * i), 0) - 1.65).collect();
        let peak = xs.iter().fold(0f64, |p, x| p.max(x.abs()));
        let crossings: Vec<usize> = (1..xs.len()).filter(|&i| xs[i - 1] < 0.0 && xs[i] >= 0.0).collect();
        let span = (crossings.last().unwrap() - crossings[0]) as f64 * 2e-6;
        (peak, (crossings.len() - 1) as f64 / span)
    }

    #[test]
    fn another_board_is_heard_with_the_link_budget_and_delay() {
        let (m, id, mut h) = setup(Trajectory::Fixed([10.0, 0.0, 1.0]));
        drive(&m, id, 50_000.0, Time::ZERO, Time::from_ms(20));
        // 4/pi * 6 V peak at 140 dB re uPa/V, 10 m (20 dB plus a little absorption), -190 dB, 40 dB of gain
        let env = m.lock().unwrap().env.clone();
        let loss = env.direct_loss_db([0.0, 0.0, 1.0], [10.0, 0.0, 1.0], 50_000.0);
        let spec = ChainSpec { gain_db: 40.0, bandpass_hz: Some([40_000.0, 60_000.0]), ..Default::default() };
        let chain = crate::chain::Chain::new(spec, 1e6).gain_at(50_000.0, 0);
        let expected = 4.0 / std::f64::consts::PI * 6.0 * 1e7 * 10f64.powf((-loss - 190.0) / 20.0) * chain;
        let (p, hz) = listen(&mut h, Time::from_ms(10), 5);
        assert!((p / expected - 1.0).abs() < 0.1, "{p} vs {expected}");
        assert!((hz - 50_000.0).abs() < 50.0, "{hz}");
        // Nothing arrives before the 6.7 ms the sound takes
        let (_, mut h2) = (0, setup(Trajectory::Fixed([10.0, 0.0, 1.0])).2);
        let quiet = (0..2000).map(|i| (h2.sample(Time::from_us(2 * i), 0) - 1.65).abs()).fold(0f64, f64::max);
        assert!(quiet < 1e-9, "{quiet}");
    }

    #[test]
    fn closing_range_raises_the_frequency() {
        // The receiver comes towards the projector at 15 m/s: +1 % (Doppler, v/c)
        let rx = Trajectory::Linear { start_m: [20.0, 0.0, 1.0], velocity_mps: [-15.0, 0.0, 0.0] };
        let (m, id, mut h) = setup(rx);
        drive(&m, id, 50_000.0, Time::ZERO, Time::from_ms(40));
        let (_, hz) = listen(&mut h, Time::from_ms(25), 10);
        assert!((hz - 50_500.0).abs() < 60.0, "{hz}");
        // Opening range lowers it
        let rx = Trajectory::Linear { start_m: [5.0, 0.0, 1.0], velocity_mps: [15.0, 0.0, 0.0] };
        let (m, id, mut h) = setup(rx);
        drive(&m, id, 50_000.0, Time::ZERO, Time::from_ms(40));
        let (_, hz) = listen(&mut h, Time::from_ms(25), 10);
        assert!((hz - 49_500.0).abs() < 60.0, "{hz}");
    }

    #[test]
    fn a_board_does_not_hear_its_own_projector_through_the_water() {
        let (m, id, h) = setup(Trajectory::Fixed([10.0, 0.0, 1.0]));
        let mut own = h.in_medium(m.clone(), 0);
        drive(&m, id, 50_000.0, Time::ZERO, Time::from_ms(20));
        let (p, _) = (
            (0..2500)
                .map(|i| (own.sample(Time::from_ms(10) + Time::from_us(2 * i), 0) - 1.65).abs())
                .fold(0f64, f64::max),
            0,
        );
        assert!(p < 1e-9, "{p}");
    }
}
