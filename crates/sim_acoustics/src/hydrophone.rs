//! A hydrophone in an environment, feeding an ADC input through its receive chain.
//!
//! The acoustic signal is computed on a fixed internal sample grid (1 MHz by default): every source's pressure
//! reaches the hydrophone over each image path (delayed, attenuated, reflected), is converted by the transducer's
//! sensitivity and beam pattern, gets ambient noise added, and runs through the receive chain's filters. The grid
//! is advanced lazily when the ADC samples; after a gap longer than [`MAX_GAP_STEPS`] the filters are restarted with
//! [`PREROLL_STEPS`] of history, which is far longer than they take to settle. Noise is a function of the grid step
//! and the environment's seed, so runs are reproducible however the ADC samples.
//!
//! Absorption and the transducer's sensitivity are applied at each source's nominal frequency (a narrowband
//! approximation); ambient noise is white at the level the noise model gives at the chain's centre frequency.
//!
//! Each step sums every path of every origin (a source, another board's projector). An origin that gives exactly
//! zero at every time its paths read over the steps being computed (a pinger between pulses, a projector whose
//! samples there are all zero: one not driven, its ringing died away below the smallest `f32`) is left out of the
//! sum, which changes nothing: the sum starts at +0 and is never -0, and adding a zero of either sign to it leaves
//! it as it is (`Hydrophone::skip_quiet`; in a pool of IVC boards, which transmit now and then, most of the time).
//!
//! In a shared [`Medium`](crate::medium::Medium) the hydrophone also hears the other boards' projectors. When
//! the hydrophone or a projector moves, every path is recomputed each [`BLOCK_STEPS`] and the delays and gains are
//! interpolated in between, which produces Doppler shifts. Both ends' positions are taken at the time the sound
//! arrives rather than when it left, which is exact to second order in v/c (a relative frequency error of 1e-7 at
//! 0.5 m/s). The tests check the interpolated paths against paths recomputed at every step, at the block
//! boundaries too, and the received frequency of a moving tone against v/c.

use std::sync::Arc;

use sim_core::analog::AnalogSource;
use sim_core::time::Time;

use crate::chain::{Chain, ChainSpec};
use crate::env::Environment;
use crate::geometry::Vec3;
use crate::medium::{Medium, ProjectorState, SharedMedium, Trajectory, WaveReader};
use crate::noise;
use crate::transducer::Transducer;
use crate::wav::Wav;

pub const DEFAULT_RATE_HZ: f64 = 1_000_000.0;
pub const MAX_GAP_STEPS: u64 = 2_000;
pub const PREROLL_STEPS: u64 = 1_000;
/// Paths are recomputed this often (grid steps) when something moves.
pub const BLOCK_STEPS: u64 = 1_000;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Origin {
    Source(usize),
    Projector(usize),
}

/// One propagation path, at the start and the end of the current block.
#[derive(Clone, Debug)]
struct Tap {
    from: Origin,
    /// Its origin's index in [`Hydrophone::spans`].
    slot: usize,
    delay_s: [f64; 2],
    /// Volts at the hydrophone per µPa at 1 m from the source.
    volts_per_upa: [f64; 2],
}

#[derive(Clone, Debug)]
pub struct Hydrophone {
    pub env: Arc<Environment>,
    /// Where it starts.
    pub position: Vec3,
    pub trajectory: Trajectory,
    pub transducer: Transducer,
    chain: Chain,
    rate: f64,
    taps: Vec<Tap>,
    /// The shared medium and the board this hydrophone belongs to.
    medium: Option<(SharedMedium, usize)>,
    /// Block the taps were computed for, and how many projectors there were.
    block: Option<(u64, usize)>,
    /// The state of each projector in the medium it hears (not its own board's), by index; refreshed from the
    /// medium at each [`Hydrophone::sample`].
    heard: Vec<Option<std::sync::Arc<ProjectorState>>>,
    /// Per origin (the environment's sources, then the medium's projectors): the shortest and longest delay of its
    /// taps in the block, and whether what it gives over the steps being computed is exactly zero (its taps are
    /// then left out: adding a zero of either sign to the sum, which is never -0, leaves it as it is).
    spans: Vec<(f64, f64)>,
    quiet: Vec<bool>,
    /// Leave out the taps of origins that give exactly zero (on by default; off only to check that it changes
    /// nothing).
    pub skip_quiet: bool,
    /// Steps computed with some origin left out (statistics).
    quiet_steps: u64,
    moves: bool,
    /// Centre of the receive chain's band, where the noise level is taken.
    centre_hz: f64,
    noise_rms_v: f64,
    seed: u64,
    step: Option<u64>,
    last: f64,
    /// ADC samples (time, volts at the pin), when recording.
    pub recording: Option<Vec<(Time, f32)>>,
    /// Samples taken, and how many of them the receive chain clipped at a rail.
    samples: u64,
    clipped: u64,
}

impl Hydrophone {
    pub fn new(env: Arc<Environment>, position: Vec3, transducer: Transducer, chain: ChainSpec) -> Self {
        Self::with_rate(env, position, transducer, chain, DEFAULT_RATE_HZ)
    }

    pub fn with_rate(
        env: Arc<Environment>,
        position: Vec3,
        transducer: Transducer,
        chain: ChainSpec,
        rate: f64,
    ) -> Self {
        let centre = match (chain.bandpass_hz, chain.filters.first()) {
            (Some([lo, hi]), _) => (lo * hi).sqrt(),
            (None, Some(f)) => match (f.highpass_hz, f.lowpass_hz) {
                (Some(lo), Some(hi)) => (lo * hi).sqrt(),
                (Some(f), None) | (None, Some(f)) => f,
                (None, None) => 30_000.0,
            },
            (None, None) => 30_000.0,
        };
        let chain = Chain::new(chain, rate);
        let level = env.noise.level_db(centre);
        let noise_rms_v = if level.is_finite() {
            10f64.powf(level / 20.0) * (rate / 2.0).sqrt() * transducer.sensitivity(centre)
        } else {
            0.0
        };
        let seed = env.seed
            ^ noise::hash(position[0].to_bits() ^ position[1].to_bits().rotate_left(21), position[2].to_bits());
        Hydrophone {
            env,
            position,
            trajectory: Trajectory::Fixed(position),
            transducer,
            chain,
            rate,
            taps: Vec::new(),
            medium: None,
            block: None,
            heard: Vec::new(),
            spans: Vec::new(),
            quiet: Vec::new(),
            skip_quiet: true,
            quiet_steps: 0,
            moves: false,
            centre_hz: centre,
            noise_rms_v,
            seed,
            step: None,
            last: 0.0,
            recording: None,
            samples: 0,
            clipped: 0,
        }
    }

    /// Puts the hydrophone in a shared medium as part of board `owner`: it hears the other boards' projectors (and
    /// the environment's sources, as before). The medium's environment replaces the one given to `new`.
    pub fn in_medium(mut self, medium: SharedMedium, owner: usize) -> Self {
        self.env = medium.lock().unwrap().env.clone();
        self.medium = Some((medium, owner));
        self.block = None;
        self
    }

    /// The shared medium the hydrophone listens in, if any.
    pub fn medium(&self) -> Option<&SharedMedium> {
        self.medium.as_ref().map(|m| &m.0)
    }

    /// Switches to another medium (a snapshot's copy of the one it was in), keeping its board.
    pub fn relink(&mut self, medium: SharedMedium) {
        if let Some(m) = &mut self.medium {
            m.0 = medium;
        }
        self.heard.clear();
    }

    /// Replaces the trajectory (a vehicle moved somewhere else between runs).
    pub fn set_trajectory(&mut self, trajectory: Trajectory) {
        self.position = trajectory.at(0.0);
        self.trajectory = trajectory;
        self.block = None;
    }

    /// Replaces the ambient noise with white noise of `level_db` re 1 µPa²/Hz (for noise sweeps).
    pub fn set_noise_db(&mut self, level_db: f64) {
        self.noise_rms_v =
            10f64.powf(level_db / 20.0) * (self.rate / 2.0).sqrt() * self.transducer.sensitivity(self.centre_hz);
    }

    /// Moves the hydrophone along `trajectory`.
    pub fn moving(mut self, trajectory: Trajectory) -> Self {
        self.position = trajectory.at(0.0);
        self.trajectory = trajectory;
        self.block = None;
        self
    }

    /// Paths from `src` at time `t` (s), with the hydrophone's beam and sensitivity at `hz` folded in.
    fn paths_at(&self, src: Vec3, hz: f64, t: f64, tx: Option<&Transducer>) -> Vec<(f64, f64)> {
        let rx = self.trajectory.at(t);
        let wavelength = self.env.sound_speed / hz;
        let sens = self.transducer.sensitivity(hz);
        self.env
            .paths(src, rx, hz)
            .into_iter()
            .map(|p| {
                let beam = self.transducer.beam_gain(p.arrival, wavelength);
                let tx_beam =
                    tx.map_or(1.0, |tx| tx.beam_gain([-p.arrival[0], -p.arrival[1], -p.arrival[2]], wavelength));
                (p.delay_s, p.gain * beam * tx_beam * sens)
            })
            .collect()
    }

    /// Recomputes the taps for the block holding grid step `n`, if needed.
    fn update_taps(&mut self, n: u64, med: Option<(&Medium, usize)>) {
        let (med, owner) = (med.map(|m| m.0), med.map(|m| m.1));
        let block = n / BLOCK_STEPS;
        let nproj = med.map_or(0, |m| m.projectors.len());
        match self.block {
            Some((b, np)) if np == nproj && (b == block || !self.moves) => return,
            _ => {}
        }
        self.moves = self.trajectory.moves()
            || med.is_some_and(|m| m.projectors.iter().any(|p| Some(p.owner) != owner && p.trajectory.moves()));
        let t0 = (block * BLOCK_STEPS) as f64 / self.rate;
        let t1 = ((block + 1) * BLOCK_STEPS) as f64 / self.rate;
        let mut taps = Vec::new();
        let nsrc = self.env.sources.len();
        let mut spans = vec![(f64::INFINITY, f64::NEG_INFINITY); nsrc + nproj];
        let mut add = |from: Origin, a: Vec<(f64, f64)>, b: Vec<(f64, f64)>| {
            let slot = match from {
                Origin::Source(i) => i,
                Origin::Projector(i) => nsrc + i,
            };
            for (pa, pb) in a.into_iter().zip(b) {
                let s = &mut spans[slot];
                *s = (s.0.min(pa.0).min(pb.0), s.1.max(pa.0).max(pb.0));
                taps.push(Tap { from, slot, delay_s: [pa.0, pb.0], volts_per_upa: [pa.1, pb.1] });
            }
        };
        for (i, s) in self.env.sources.iter().enumerate() {
            let hz = s.nominal_hz();
            add(Origin::Source(i), self.paths_at(s.position, hz, t0, None), self.paths_at(s.position, hz, t1, None));
        }
        if let Some(m) = med {
            for (i, p) in m.projectors.iter().enumerate().filter(|(_, p)| Some(p.owner) != owner) {
                let hz = p.nominal_hz;
                let tx = Some(&p.transducer);
                add(
                    Origin::Projector(i),
                    self.paths_at(p.trajectory.at(t0), hz, t0, tx),
                    self.paths_at(p.trajectory.at(t1), hz, t1, tx),
                );
            }
        }
        self.taps = taps;
        self.spans = spans;
        self.block = Some((block, nproj));
    }

    /// Starts keeping every ADC sample.
    pub fn record(mut self) -> Self {
        self.recording = Some(Vec::new());
        self
    }

    /// Which origins give exactly zero at every step from `n0` to `n1` (all in one block of taps), from the
    /// shortest and longest delays of their taps, with a margin for the rounding of the delays interpolated and of
    /// the times (see [`Hydrophone::spans`]). Only where every path is well within the second a projector keeps
    /// ([`crate::medium::HISTORY_S`]): reading a projector computes its samples, which may forget the oldest.
    fn find_quiet(&mut self, n0: u64, n1: u64, heard: &mut [Option<WaveReader<'_>>]) {
        const ROUNDING: f64 = 1e-9;
        let (t0, t1) = (n0 as f64 / self.rate, n1 as f64 / self.rate);
        let nsrc = self.env.sources.len();
        self.quiet.clear();
        if !self.skip_quiet {
            return;
        }
        for (slot, &(short, long)) in self.spans.iter().enumerate() {
            let quiet = short <= long && long < crate::medium::HISTORY_S - 0.01 && {
                let (from, to) = (t0 - long - ROUNDING, t1 - short + ROUNDING);
                match slot.checked_sub(nsrc) {
                    None => self.env.sources[slot].silent(from, to),
                    Some(i) => heard.get_mut(i).and_then(Option::as_mut).is_none_or(|w| w.silent(from, to)),
                }
            };
            self.quiet.push(quiet);
        }
    }

    /// Points [`Hydrophone::heard`] at the states of `med`'s projectors (other boards' ones).
    fn refresh_heard(&mut self, med: &Medium, owner: usize) {
        let owner = Some(owner);
        let same = self.heard.len() == med.projectors.len()
            && self.heard.iter().zip(&med.projectors).all(|(h, p)| match h {
                Some(h) => std::sync::Arc::ptr_eq(h, &p.state),
                None => Some(p.owner) == owner,
            });
        if !same {
            self.heard = med.projectors.iter().map(|p| (Some(p.owner) != owner).then(|| p.state.clone())).collect();
        }
    }

    /// Hydrophone output (before the chain) at grid step `n`, with the taps for its block, reading the projectors
    /// it hears through `heard` (by projector index).
    fn raw(&self, n: u64, heard: &mut [Option<WaveReader<'_>>]) -> f64 {
        let t = n as f64 / self.rate;
        let f = (n % BLOCK_STEPS) as f64 / BLOCK_STEPS as f64;
        let mut v = 0.0;
        for tap in &self.taps {
            if self.quiet.get(tap.slot).copied().unwrap_or(false) {
                continue;
            }
            let delay = tap.delay_s[0] + (tap.delay_s[1] - tap.delay_s[0]) * f;
            let gain = tap.volts_per_upa[0] + (tap.volts_per_upa[1] - tap.volts_per_upa[0]) * f;
            let p = match tap.from {
                Origin::Source(i) => self.env.sources[i].pressure(t - delay),
                Origin::Projector(i) => {
                    heard.get_mut(i).and_then(Option::as_mut).map_or(0.0, |w| w.pressure(t - delay))
                }
            };
            v += gain * p;
        }
        if self.noise_rms_v > 0.0 {
            v += self.noise_rms_v * noise::gaussian(self.seed, n);
        }
        if let Some(sh) = &self.env.shrimp {
            v += self.shrimp(sh, t);
        }
        v
    }

    fn shrimp(&self, sh: &noise::Shrimp, t: f64) -> f64 {
        const BIN: f64 = 1e-3;
        let sens = self.transducer.sensitivity(30_000.0);
        let peak = 10f64.powf(sh.peak_db_re_upa / 20.0) * sens;
        let decay = sh.decay_us * 1e-6;
        let m = (t / BIN).floor() as i64;
        let mut v = 0.0;
        for bin in [m - 1, m] {
            if bin < 0 {
                continue;
            }
            let b = bin as u64;
            if noise::uniform(self.seed ^ 0x5348_5249_4D50, b) > sh.rate_hz * BIN {
                continue;
            }
            let tc = bin as f64 * BIN + noise::uniform(self.seed ^ 0xC11C, b) * BIN;
            if t >= tc {
                let sign = if noise::hash(self.seed, b) & 1 == 0 { 1.0 } else { -1.0 };
                v += sign * peak * (-(t - tc) / decay).exp();
            }
        }
        v
    }

    /// The grid step holding time `t`. Computed in integer picoseconds: in floating point, a time exactly on a
    /// step (every ADC sample of a board sampling at 500 kS/s from a whole microsecond) can come out a hair short
    /// and read the step before, which jitters those samples by a whole step.
    fn step_at(&self, t: Time) -> u64 {
        if self.rate.fract() == 0.0 {
            (t.as_ps() as u128 * self.rate as u128 / 1_000_000_000_000) as u64
        } else {
            (t.as_secs_f64() * self.rate).floor() as u64
        }
    }

    /// Advances the chain to `t` and returns the voltage at the ADC pin.
    pub fn sample(&mut self, t: Time, pins: u32) -> f64 {
        let k = self.step_at(t);
        let from = match self.step {
            Some(s) if k <= s => return self.last,
            Some(s) if k - s <= MAX_GAP_STEPS => s + 1,
            _ => {
                self.chain.reset();
                k.saturating_sub(PREROLL_STEPS)
            }
        };
        // (Taken out rather than cloned: the count of a shared `Arc` is a cache line every board's thread writes.)
        let medium = self.medium.take();
        let mut n = from;
        while n <= k {
            // The taps for step n's block, holding the medium only for that; then the steps up to the next block
            // whose taps differ, holding only the projectors heard
            {
                let guard = medium.as_ref().map(|(m, owner)| (m.lock().unwrap_or_else(|e| e.into_inner()), *owner));
                self.update_taps(n, guard.as_ref().map(|(m, owner)| (&**m, *owner)));
                if let Some((m, owner)) = &guard {
                    self.refresh_heard(m, *owner);
                }
            }
            let last = if self.moves { k.min((n / BLOCK_STEPS + 1) * BLOCK_STEPS - 1) } else { k };
            let states = std::mem::take(&mut self.heard);
            // Readers on the stack for a few projectors (a pool of boards), else in a vector
            let mut few: [Option<WaveReader<'_>>; 4] = [None, None, None, None];
            let mut many = Vec::new();
            let heard: &mut [Option<WaveReader<'_>>] = if states.len() <= few.len() {
                for (r, s) in few.iter_mut().zip(&states) {
                    *r = s.as_ref().map(|s| s.read());
                }
                &mut few[..states.len()]
            } else {
                many.extend(states.iter().map(|s| s.as_ref().map(|s| s.read())));
                &mut many
            };
            self.find_quiet(n, last, heard);
            if self.quiet.contains(&true) {
                self.quiet_steps += last - n + 1;
            }
            for n in n..=last {
                let raw = self.raw(n, heard);
                self.last = self.chain.process(raw, pins);
            }
            drop(few);
            drop(many);
            self.heard = states;
            n = last + 1;
        }
        self.medium = medium;
        self.step = Some(k);
        self.samples += 1;
        if self.chain.at_rail(self.last) {
            self.clipped += 1;
        }
        if let Some(r) = &mut self.recording {
            r.push((t, self.last as f32));
        }
        self.last
    }

    /// Expected peak voltage swing at the pin from source `i` over the direct path alone, for steady state (no
    /// multipath interference), through the filter `pins` select.
    pub fn direct_peak_v(&self, i: usize, pins: u32) -> f64 {
        let s = &self.env.sources[i];
        let hz = s.nominal_hz();
        let p = self.env.paths(s.position, self.position, hz);
        let direct = p.iter().find(|p| p.reflections == 0).map_or(0.0, |p| p.gain);
        let beam = self.transducer.beam_gain(
            p.iter().find(|p| p.reflections == 0).map_or([0.0, 0.0, 1.0], |p| p.arrival),
            self.env.sound_speed / hz,
        );
        s.rms_upa
            * std::f64::consts::SQRT_2
            * direct
            * beam
            * self.transducer.sensitivity(hz)
            * self.chain.gain_at(hz, pins)
    }

    /// `(samples, clipped)`: how many samples the ADC took, and how many of them were at a rail of the receive
    /// chain (the signal clipped: too loud for the chain's gain).
    pub fn clipping(&self) -> (u64, u64) {
        (self.samples, self.clipped)
    }

    /// The recording as the ADC converted it, as a WAV file at `rate`: -1..1 is the ADC input range `range_v` (the
    /// pin voltage, offset included, as `calibrate acoustics` reads a board's own capture), quantised to `bits`
    /// (0: not quantised). Samples are placed at their times; gaps (between the firmware's captures) are
    /// mid-scale.
    pub fn recording_pin_wav(&self, rate: u32, range_v: [f64; 2], bits: u32) -> Option<Wav> {
        let r = self.recording.as_ref()?;
        let start = r.first()?.0;
        let end = r.last()?.0;
        let n = ((end - start).as_secs_f64() * rate as f64).round() as usize + 1;
        let mut samples = vec![0.0f32; n];
        let (mid, half) = ((range_v[0] + range_v[1]) / 2.0, (range_v[1] - range_v[0]) / 2.0);
        for (t, v) in r {
            // Rounded: ADC samples a hair under a sample period apart must not skip a slot
            let i = ((*t - start).as_secs_f64() * rate as f64).round() as usize;
            if let Some(s) = samples.get_mut(i) {
                let mut x = *v as f64;
                if bits > 0 {
                    let levels = ((1u64 << bits) - 1) as f64;
                    let code = ((x.clamp(range_v[0], range_v[1]) - range_v[0]) / (2.0 * half) * levels).round();
                    x = range_v[0] + code / levels * 2.0 * half;
                }
                *s = ((x - mid) / half).clamp(-1.0, 1.0) as f32;
            }
        }
        Some(Wav { rate, samples })
    }

    /// The recording as a WAV file at `rate` (samples are placed at their times; gaps are silent).
    pub fn recording_wav(&self, rate: u32) -> Option<Wav> {
        let r = self.recording.as_ref()?;
        let start = r.first()?.0;
        let end = r.last()?.0;
        let n = ((end - start).as_secs_f64() * rate as f64) as usize + 1;
        let mut samples = vec![0.0f32; n];
        let bias = self.chain.spec.bias_v.unwrap_or((self.chain.spec.rails_v[0] + self.chain.spec.rails_v[1]) / 2.0);
        let span = (self.chain.spec.rails_v[1] - self.chain.spec.rails_v[0]) as f32 / 2.0;
        for (t, v) in r {
            let i = ((*t - start).as_secs_f64() * rate as f64) as usize;
            if let Some(s) = samples.get_mut(i) {
                *s = (*v - bias as f32) / span;
            }
        }
        Some(Wav { rate, samples })
    }
}

impl AnalogSource for Hydrophone {
    fn voltage(&mut self, t: Time, pins: u32) -> f64 {
        self.sample(t, pins)
    }

    fn clone_box(&self) -> Box<dyn AnalogSource> {
        Box::new(self.clone())
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::{Source, Waveform};
    use crate::water::Water;

    fn tone_env(hz: f64, level_db: f64, at: Vec3) -> Arc<Environment> {
        let mut env = Environment::open_water(Water::default());
        env.sources.push(Source {
            position: at,
            waveform: Waveform::Tone { hz },
            rms_upa: 10f64.powf(level_db / 20.0),
        });
        Arc::new(env)
    }

    fn peak(h: &mut Hydrophone, from: Time, n: usize) -> f64 {
        // Sample at 500 kS/s like the IVC boards
        (0..n).map(|i| (h.sample(from + Time::from_us(2 * i as u64), 0) - 1.65).abs()).fold(0.0, f64::max)
    }

    #[test]
    fn tone_level_matches_the_link_budget() {
        // 160 dB re 1 uPa at 1 m, 10 m away: 140 dB = 1e7 uPa rms; -190 dB re 1 V/uPa: 3.16 mV rms; 20 dB gain
        let env = tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]);
        let chain = ChainSpec { gain_db: 20.0, bandpass_hz: Some([20_000.0, 50_000.0]), ..Default::default() };
        let mut h = Hydrophone::new(env, [0.0, 0.0, 1.0], Transducer::hydrophone(-190.0), chain);
        let expected = h.direct_peak_v(0, 0);
        let rms_upa = 1e7;
        let budget = rms_upa * 2f64.sqrt() * 10f64.powf(-9.5) * h.chain.gain_at(30_000.0, 0);
        assert!((expected / budget - 1.0).abs() < 1e-3, "{expected} {budget}");
        let p = peak(&mut h, Time::from_ms(10), 2000);
        assert!((p / expected - 1.0).abs() < 0.03, "{p} vs {expected}");
    }

    #[test]
    fn amplitude_halves_with_doubled_distance_and_gaps_are_handled() {
        let chain = ChainSpec { gain_db: 20.0, bandpass_hz: Some([20_000.0, 50_000.0]), ..Default::default() };
        let mut near = Hydrophone::new(
            tone_env(30_000.0, 160.0, [5.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            chain.clone(),
        );
        let mut far = Hydrophone::new(
            tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            chain,
        );
        let a = peak(&mut near, Time::from_ms(20), 1000);
        let b = peak(&mut far, Time::from_ms(20), 1000);
        assert!((a / b - 2.0).abs() < 0.05, "{a} {b}");
        // A long gap, then sampling again: the filter restarts with preroll and reads the same level
        let c = peak(&mut far, Time::from_secs(3), 1000);
        assert!((c / b - 1.0).abs() < 0.03, "{c} {b}");
    }

    /// A projector at 18 kHz (the first-generation IVC modem's band) and a hydrophone in `env`, either of them on a
    /// trajectory, the projector driven with a square wave for 400 ms.
    fn moving_link(env: Environment, tx: Trajectory, rx: Trajectory) -> Hydrophone {
        use crate::medium::{DriveSpec, Medium};
        let medium = Medium::shared(Arc::new(env));
        let t: Transducer =
            toml::from_str("tvr = { resonance_hz = 18_000, q = 4, peak_db = 140 }\nrvs_db = -190").unwrap();
        {
            let mut m = medium.lock().unwrap();
            let id = m.add_projector(0, "tx", t.clone(), tx, &DriveSpec { pin: 7, supply_v: 12.0, nominal_hz: None });
            let half = Time::from_ps(1_000_000_000_000 / 36_000);
            let mut at = Time::ZERO;
            let mut level = true;
            while at < Time::from_ms(400) {
                m.edge(id, at, level);
                level = !level;
                at += half;
            }
            m.progress(0, Time::from_ms(500));
        }
        let chain = ChainSpec { gain_db: 40.0, bandpass_hz: Some([12_000.0, 25_000.0]), ..Default::default() };
        let env = medium.lock().unwrap().env.clone();
        Hydrophone::new(env, [0.0; 3], t, chain).in_medium(medium, 1).moving(rx)
    }

    /// The hydrophone's output before the chain, summed over every path computed exactly for the positions at
    /// time `t` (what the per-block taps approximate by interpolation).
    fn exact_raw(h: &Hydrophone, n: u64) -> f64 {
        let t = n as f64 / h.rate;
        let medium = h.medium().unwrap().clone();
        let mut m = medium.lock().unwrap();
        let p = &mut m.projectors[0];
        let src = p.trajectory.at(t);
        let paths = h.env.paths(src, h.trajectory.at(t), p.nominal_hz);
        let sens = h.transducer.sensitivity(p.nominal_hz);
        paths.iter().map(|path| path.gain * sens * p.pressure(t - path.delay_s)).sum()
    }

    fn pool() -> Environment {
        let mut env = Environment::parse(
            "kind = \"tank\"\nsize_m = [25.0, 12.0, 3.0]\nwall_reflection = 0.6\nbottom_reflection = 0.5\n\
             multipath = { max_order = 3 }\n",
            std::path::Path::new("."),
        )
        .unwrap();
        env.noise = crate::noise::Noise::None;
        env
    }

    #[test]
    fn leaving_out_silent_origins_changes_no_sample() {
        // A pinger (4 ms on every 200 ms) and another board's projector driven for the first 400 ms, both silent
        // most of the time, heard over the pool's 63 paths, from a fixed and from a moving hydrophone, sampled at
        // 500 kS/s with gaps (where the chain restarts from its preroll): bit for bit the same with every tap
        let pinger = crate::source::Source {
            position: [20.0, 6.0, 2.0],
            waveform: crate::source::Waveform::Pinger { hz: 25_000.0, pulse: 4e-3, period: 0.2, start: 0.05 },
            rms_upa: 1e9,
        };
        let moving = Trajectory::Linear { start_m: [5.0, 6.0, 1.5], velocity_mps: [0.5, 0.0, 0.0] };
        for rx in [Trajectory::Fixed([5.0, 6.0, 1.5]), moving] {
            let mut env = pool();
            env.sources.push(pinger.clone());
            let mut on = moving_link(env.clone(), Trajectory::Fixed([10.0, 6.0, 1.5]), rx.clone());
            let mut off = moving_link(env, Trajectory::Fixed([10.0, 6.0, 1.5]), rx);
            off.skip_quiet = false;
            let mut t = Time::from_us(3);
            for k in 0..600_000u64 {
                let (a, b) = (on.sample(t, 0), off.sample(t, 0));
                assert_eq!(a.to_bits(), b.to_bits(), "at {t}");
                t += Time::from_us(if k % 50_000 == 49_999 { 3_001 } else { 2 });
            }
            // Most steps left some origin out: the projector after its drive, the pinger between pulses
            assert!(on.quiet_steps > on.samples, "{} of {}", on.quiet_steps, on.samples);
            assert_eq!(off.quiet_steps, 0);
        }
    }

    #[test]
    fn moving_taps_follow_the_exact_paths_across_block_boundaries() {
        // Talos's IVC board closing on the surface vehicle's at 0.5 m/s in the pool (63 paths), with either end the
        // one that moves: the interpolated taps match every path recomputed at every grid step, at the block
        // boundaries (where the taps are rebuilt) as much as anywhere else
        let moving = Trajectory::Linear { start_m: [5.0, 6.0, 1.5], velocity_mps: [0.5, 0.0, 0.0] };
        let fixed = Trajectory::Fixed([10.0, 6.0, 1.5]);
        for (tx, rx) in [(moving.clone(), fixed.clone()), (fixed, moving)] {
            let mut h = moving_link(pool(), tx, rx);
            let medium = h.medium().unwrap().clone();
            let (mut worst, mut peak, mut worst_at_boundary) = (0f64, 0f64, 0f64);
            for n in 50_000..150_000u64 {
                let got = {
                    let m = medium.lock().unwrap();
                    h.update_taps(n, Some((&m, 1)));
                    h.refresh_heard(&m, 1);
                    drop(m);
                    let states = h.heard.clone();
                    let mut heard: Vec<_> = states.iter().map(|s| s.as_ref().map(|s| s.read())).collect();
                    h.raw(n, &mut heard)
                };
                let want = exact_raw(&h, n);
                peak = peak.max(want.abs());
                worst = worst.max((got - want).abs());
                if n % BLOCK_STEPS == 0 {
                    worst_at_boundary = worst_at_boundary.max((got - want).abs());
                }
            }
            assert!(peak > 1e-3, "{peak}");
            assert!(worst < 1e-6 * peak, "{worst} of {peak}");
            assert!(worst_at_boundary <= worst);
        }
    }

    /// Zero crossings (upwards, linearly interpolated) of the hydrophone's output at the pin, sampled at 1 MS/s
    /// from `from` for `ms`.
    fn crossings(h: &mut Hydrophone, from: Time, ms: u64) -> Vec<f64> {
        let mut out = Vec::new();
        let mut prev: Option<(f64, f64)> = None;
        for i in 0..ms * 1000 {
            let t = from + Time::from_us(i);
            let v = h.sample(t, 0) - 1.65;
            if let Some((tp, vp)) = prev
                && vp < 0.0
                && v >= 0.0
            {
                out.push(tp + (t.as_secs_f64() - tp) * (-vp / (v - vp)));
            }
            prev = Some((t.as_secs_f64(), v));
        }
        out
    }

    #[test]
    fn walking_speed_doppler_is_smooth() {
        // 0.5 m/s in open water, either end moving, closing or opening: the received frequency is 18 kHz shifted by
        // v/c (6 Hz), and the phase follows it without a jump at any block boundary (zero crossings within a few
        // nanoseconds of a constant frequency over 200 ms, where one tap rebuild out of step would show as a
        // microsecond)
        let mut open = Environment::open_water(crate::water::Water {
            sound_speed_mps: Some(1500.0),
            ..crate::water::Water::default()
        });
        open.noise = crate::noise::Noise::None;
        let fixed = Trajectory::Fixed([0.0, 0.0, 1.0]);
        let mover = |v: f64| Trajectory::Linear { start_m: [5.0, 0.0, 1.0], velocity_mps: [v, 0.0, 0.0] };
        let f0 = 1e12 / (2 * (1_000_000_000_000u64 / 36_000)) as f64;
        for (tx_moves, v) in [(true, -0.5), (true, 0.5), (false, -0.5), (false, 0.5), (false, 0.0)] {
            let (tx, rx) = if tx_moves { (mover(v), fixed.clone()) } else { (fixed.clone(), mover(v)) };
            let mut h = moving_link(open.clone(), tx, rx);
            let xs = crossings(&mut h, Time::from_ms(100), 200);
            // Least-squares line through the crossing times: its slope is the period
            let n = xs.len() as f64;
            let (mi, mx) = ((n - 1.0) / 2.0, xs.iter().sum::<f64>() / n);
            let (mut sxy, mut sxx) = (0.0, 0.0);
            for (i, x) in xs.iter().enumerate() {
                sxy += (i as f64 - mi) * (x - mx);
                sxx += (i as f64 - mi) * (i as f64 - mi);
            }
            let period = sxy / sxx;
            let hz = 1.0 / period;
            // Range opening (v > 0, away from the other end at the origin) lowers the frequency, closing raises it
            let expected = if tx_moves { f0 / (1.0 + v / 1500.0) } else { f0 * (1.0 - v / 1500.0) };
            assert!((hz - expected).abs() < 0.05, "tx moves {tx_moves}, {v} m/s: {hz} Hz, expected {expected}");
            let worst =
                xs.iter().enumerate().map(|(i, x)| (x - mx - (i as f64 - mi) * period).abs()).fold(0.0, f64::max);
            assert!(worst < 5e-9, "tx moves {tx_moves}, {v} m/s: crossings off by {worst:e} s");
        }
    }

    #[test]
    fn a_time_on_a_grid_step_reads_that_step() {
        // 57 us is step 57, but in f64 seconds times 1e6 it comes out a hair short (as about one in eight whole
        // microseconds do)
        assert!((Time::from_us(57).as_secs_f64() * 1e6).floor() < 57.0);
        let h = Hydrophone::new(
            tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            ChainSpec::default(),
        );
        for us in [0, 1, 25, 57, 125_014, 1_234_567, 9_999_999] {
            assert_eq!(h.step_at(Time::from_us(us)), us);
            assert_eq!(h.step_at(Time::from_us(us) + Time::from_ps(999_999)), us);
        }
        let slow = Hydrophone::with_rate(
            tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            ChainSpec::default(),
            250_000.0,
        );
        assert_eq!(slow.step_at(Time::from_us(100_004)), 25_001);
        assert_eq!(slow.step_at(Time::from_us(100_003)), 25_000);
    }

    #[test]
    fn noise_is_reproducible_and_scaled() {
        let mut env = Environment::open_water(Water::default());
        env.noise = crate::noise::Noise::White { level_db_re_upa_per_hz: 60.0 };
        let env = Arc::new(env);
        let chain = ChainSpec { gain_db: 40.0, ..Default::default() };
        let mut a =
            Hydrophone::new(env.clone(), [0.0, 0.0, 1.0], Transducer::hydrophone(-180.0), chain.clone()).record();
        let mut b = Hydrophone::new(env.clone(), [0.0, 0.0, 1.0], Transducer::hydrophone(-180.0), chain.clone());
        let xs: Vec<f64> = (0..5000).map(|i| a.sample(Time::from_us(i), 0) - 1.65).collect();
        let ys: Vec<f64> = (0..5000).map(|i| b.sample(Time::from_us(i), 0) - 1.65).collect();
        assert_eq!(xs, ys);
        // 60 dB/Hz over 500 kHz: 1e3 uPa * 707 = 7.07e5 uPa rms; -180 dB: 7.07e-4 V; 40 dB: 70.7 mV rms
        let rms = (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt();
        assert!((rms - 0.0707).abs() < 0.005, "{rms}");
        let wav = a.recording_wav(1_000_000).unwrap();
        assert_eq!(wav.samples.len(), 5000);
    }
}
