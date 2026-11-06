//! The water shared by several boards: their projectors, heard by each other's hydrophones.
//!
//! A board's projector is fed from one of its output pins. The pin's level changes are recorded with their times;
//! the drive voltage (the pin through a driver stage of `supply_v`) is averaged over each step of the medium's
//! sample grid (1 MHz) and run through the transducer's transmitting response, a band-pass resonator when the TVR
//! is given as a resonance. The result is the pressure the projector radiates, in µPa at 1 m, kept for the last
//! [`HISTORY_S`] so hydrophones can read it back at whatever delay each propagation path has.
//!
//! A projector's waveform is only known up to the time its board has reached. Each board reports its progress;
//! samples are computed and cached only once the whole step is known, and a hydrophone asking further ahead gets
//! the last known value without it being cached. Boards taking turns (see `sim_host::robot`) are at most a slice
//! apart, so any path longer than the robot's slice (100 µs, 15 cm of water) always reads exact values; boards
//! running ahead of each other on threads do so only as far as the shortest path between them allows.
//!
//! What a board drives into its projector ([`Projector::edge`], [`Medium::progress`]) and the waveform computed
//! from it for the hydrophones each have a lock of their own ([`ProjectorState`]), so that a hydrophone reading
//! other boards' projectors and those boards driving theirs do not wait for each other.
//!
//! Vehicles can move ([`Trajectory`]): the hydrophone recomputes its paths every millisecond and interpolates the
//! delays in between, so a closing or opening range shifts the received frequency (Doppler) and timing. A
//! [`LivePath`] is a trajectory written as it happens, by a vehicle simulation running alongside the boards.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};

use serde::Deserialize;
use sim_core::time::Time;

use crate::env::Environment;
use crate::geometry::Vec3;
use crate::lanes::{self, LANES};
use crate::transducer::{Response, Transducer};

/// Projector pressure history kept for hydrophones to read back: paths up to about 1.5 km.
pub const HISTORY_S: f64 = 1.0;
/// Sample rate of the projector waveforms.
pub const RATE_HZ: f64 = 1_000_000.0;
const STEP_PS: u64 = 1_000_000;
/// Samples kept: [`HISTORY_S`] of them.
const HISTORY_LEN: u64 = (HISTORY_S * RATE_HZ) as u64;

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
    /// Recorded while the robot runs (a transducer on a simulated vehicle). Not in files: a robot file puts a
    /// board on a vehicle instead.
    #[serde(skip)]
    Live(LivePath),
}

/// A path being recorded: positions pushed as a vehicle simulation advances, read back by hydrophones and
/// projectors at any time up to about [`HISTORY_S`] ago. Beyond the newest point (boards run up to a slice ahead
/// of the vehicle) it continues at the last velocity for at most [`LivePath::AHEAD_S`].
///
/// Clones share the recording; [`LivePath::deep_copy`] makes an independent one (for snapshots).
#[derive(Clone, Debug)]
pub struct LivePath(Arc<Mutex<VecDeque<(f64, Vec3)>>>);

impl PartialEq for LivePath {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl LivePath {
    /// How far past the newest point the path is extrapolated.
    pub const AHEAD_S: f64 = 0.01;

    /// A path starting at `start` at time 0.
    pub fn new(start: Vec3) -> LivePath {
        LivePath(Arc::new(Mutex::new(VecDeque::from([(0.0, start)]))))
    }

    /// Adds the position at `t` seconds (later than every point so far; earlier ones are ignored).
    pub fn push(&self, t: f64, p: Vec3) {
        let mut pts = self.0.lock().unwrap();
        if pts.back().is_some_and(|b| t <= b.0) {
            return;
        }
        pts.push_back((t, p));
        while pts.len() > 2 && pts[1].0 < t - HISTORY_S {
            pts.pop_front();
        }
    }

    /// The recording so far (time travel keeps it with a checkpoint).
    pub fn points(&self) -> VecDeque<(f64, Vec3)> {
        self.0.lock().unwrap().clone()
    }

    /// Replaces the recording (time travel going back to a checkpoint), for every clone.
    pub fn set_points(&self, points: VecDeque<(f64, Vec3)>) {
        *self.0.lock().unwrap() = points;
    }

    /// An independent copy of the recording so far.
    pub fn deep_copy(&self) -> LivePath {
        LivePath(Arc::new(Mutex::new(self.0.lock().unwrap().clone())))
    }

    /// The newest point.
    pub fn last(&self) -> (f64, Vec3) {
        *self.0.lock().unwrap().back().expect("a live path always has a point")
    }

    pub fn at(&self, t: f64) -> Vec3 {
        let pts = self.0.lock().unwrap();
        let n = pts.len();
        let lerp = |(t0, p0): (f64, Vec3), (t1, p1): (f64, Vec3), t: f64| -> Vec3 {
            let x = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            [p0[0] + (p1[0] - p0[0]) * x, p0[1] + (p1[1] - p0[1]) * x, p0[2] + (p1[2] - p0[2]) * x]
        };
        if t <= pts[0].0 {
            return pts[0].1;
        }
        if t >= pts[n - 1].0 {
            if n < 2 {
                return pts[0].1;
            }
            let t = t.min(pts[n - 1].0 + Self::AHEAD_S);
            return lerp(pts[n - 2], pts[n - 1], t);
        }
        let i = pts.partition_point(|p| p.0 <= t);
        lerp(pts[i - 1], pts[i], t)
    }
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
            Trajectory::Live(path) => path.at(t),
        }
    }

    pub fn moves(&self) -> bool {
        match self {
            Trajectory::Fixed(_) => false,
            Trajectory::Linear { velocity_mps, .. } => velocity_mps.iter().any(|v| *v != 0.0),
            Trajectory::Waypoints(p) => p.windows(2).any(|w| w[0].1 != w[1].1),
            // Whatever the vehicle does next is not known yet
            Trajectory::Live(_) => true,
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
#[derive(Debug)]
pub struct Projector {
    /// Which board drives it (its hydrophone doesn't hear it through the water).
    pub owner: usize,
    pub name: String,
    pub transducer: Transducer,
    pub trajectory: Trajectory,
    pub nominal_hz: f64,
    /// What it is driven with and radiates, shared with the hydrophones that read it (see [`ProjectorState`]).
    pub state: Arc<ProjectorState>,
}

impl Clone for Projector {
    /// A copy with a state of its own (snapshots).
    fn clone(&self) -> Self {
        let drive = lock(&self.state.drive).clone();
        let wave = lock(&self.state.wave).clone();
        Projector {
            owner: self.owner,
            name: self.name.clone(),
            transducer: self.transducer.clone(),
            trajectory: self.trajectory.clone(),
            nominal_hz: self.nominal_hz,
            state: Arc::new(ProjectorState { drive: Apart(Mutex::new(drive)), wave: Apart(Mutex::new(wave)) }),
        }
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// A projector's drive and waveform. The board driving it adds level changes and progress to `drive`; reading
/// the waveform (`wave`) moves them over as it needs them, and computes the samples whose whole step is known.
/// Lock order: `wave` before `drive`.
#[derive(Debug)]
pub struct ProjectorState {
    drive: Apart<Mutex<Drive>>,
    wave: Apart<Mutex<Wave>>,
}

/// On a cache line of its own: the board driving a projector and the hydrophones reading it run on different
/// threads, each writing one of the locks.
#[derive(Debug)]
#[repr(align(128))]
struct Apart<T>(T);

impl<T> std::ops::Deref for Apart<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

/// What the board has driven and not yet handed to the waveform.
#[derive(Clone, Debug)]
struct Drive {
    /// Level changes, oldest first.
    edges: VecDeque<(Time, bool)>,
    progress: Time,
    /// Level changes ever added.
    pushed: u64,
}

/// The radiated waveform.
#[derive(Clone, Debug)]
struct Wave {
    drive_v: f64,
    /// µPa at 1 m per volt of drive at the nominal frequency (the TVR's peak for a resonance).
    gain: f64,
    resonator: Resonator,
    /// Level changes taken from the drive and not yet folded into samples, oldest first, and how many were taken.
    edges: VecDeque<(Time, bool)>,
    taken: u64,
    level: bool,
    /// The drive's progress when last taken.
    progress: Time,
    /// Grid index of `samples[0]`.
    base: u64,
    samples: VecDeque<f32>,
    /// The last sample computed that is not zero (either sign), by grid index.
    last_loud: Option<u64>,
    /// Sets of times `pressure_lanes` read in lanes, and one by one (tests).
    #[cfg(test)]
    lane_reads: [u64; 2],
}

/// Where a projector's drive was ([`ProjectorState::mark`]), to go back to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveMark {
    pushed: u64,
    progress: Time,
}

impl ProjectorState {
    fn add_edge(&self, t: Time, level: bool) {
        let mut d = lock(&self.drive);
        d.edges.push_back((t, level));
        d.progress = d.progress.max(t);
        d.pushed += 1;
    }

    fn add_progress(&self, t: Time) {
        let mut d = lock(&self.drive);
        d.progress = d.progress.max(t);
    }

    /// Level changes the drive has had.
    pub fn edge_count(&self) -> u64 {
        lock(&self.drive).pushed
    }

    /// Where the drive is now.
    pub fn mark(&self) -> DriveMark {
        let d = lock(&self.drive);
        DriveMark { pushed: d.pushed, progress: d.progress }
    }

    /// Takes the drive back to `mark` (its board is rolled back there): the level changes added since are
    /// forgotten, and the progress is as it was. None of them can have been folded into samples yet (hydrophones
    /// only read steps known before the board's rollback point); panics otherwise.
    pub fn back_to(&self, mark: DriveMark) {
        let mut w = lock(&self.wave);
        let mut d = lock(&self.drive);
        let mut drop = d.pushed - mark.pushed;
        let here = drop.min(d.edges.len() as u64);
        let keep = d.edges.len() - here as usize;
        d.edges.truncate(keep);
        drop -= here;
        assert!(drop <= w.edges.len() as u64, "level changes to forget were already in the waveform");
        let keep = w.edges.len() - drop as usize;
        w.edges.truncate(keep);
        w.taken -= drop;
        d.pushed = mark.pushed;
        d.progress = mark.progress;
        w.progress = w.progress.min(mark.progress);
    }

    /// Replaces the waveform's samples with `samples` from grid step `base` on, the deque's storage wrapping round
    /// after `wrap` of them (tests: `pressure_lanes` reads either half). The drive is never known further.
    #[cfg(test)]
    pub(crate) fn set_samples(&self, base: u64, samples: &[f32], wrap: usize) {
        let mut w = lock(&self.wave);
        let mut d = VecDeque::with_capacity(samples.len());
        // (Moving the deque's head along by its capacity less `wrap`)
        for _ in 0..d.capacity() - wrap.min(d.capacity()) {
            d.push_back(0.0);
            d.pop_front();
        }
        d.extend(samples.iter().copied());
        w.base = base;
        w.samples = d;
    }

    /// Sets of times read in lanes, and one by one ([`Wave::pressure_lanes`]; tests).
    #[cfg(test)]
    pub(crate) fn lane_reads(&self) -> [u64; 2] {
        lock(&self.wave).lane_reads
    }

    /// Whether the samples' storage wraps round (tests).
    #[cfg(test)]
    pub(crate) fn wraps(&self) -> bool {
        !lock(&self.wave).samples.as_slices().1.is_empty()
    }

    /// Reads the waveform (holding its lock until the reader is dropped).
    pub fn read(&self) -> WaveReader<'_> {
        WaveReader { wave: lock(&self.wave), drive: &self.drive }
    }
}

/// A projector's waveform being read ([`ProjectorState::read`]).
pub struct WaveReader<'a> {
    wave: MutexGuard<'a, Wave>,
    drive: &'a Mutex<Drive>,
}

impl WaveReader<'_> {
    /// Radiated pressure at `t` seconds (µPa at 1 m), interpolated between grid steps.
    #[inline]
    pub fn pressure(&mut self, t: f64) -> f64 {
        self.wave.pressure(t, self.drive)
    }

    /// [`WaveReader::pressure`] at each of the first `n` times of `t`, into `out`, the same to the bit, read as it
    /// reads them; the times after the `n`th are copies of one before it, and `k` and `b` are room for as many
    /// values (see [`crate::lanes`]).
    #[inline(always)]
    pub fn pressure_lanes(
        &mut self,
        t: &[[f64; LANES]],
        n: usize,
        out: &mut [[f64; LANES]],
        k: &mut [[f64; LANES]],
        b: &mut [[f64; LANES]],
    ) {
        self.wave.pressure_lanes(t, n, out, k, b, self.drive)
    }

    /// Whether [`WaveReader::pressure`] is exactly zero (either sign) at every time from `from` to `to` (s): every
    /// sample it would interpolate between is computed and zero. Computes the samples up to there first, as far as
    /// the drive is known (as reading them would).
    pub fn silent(&mut self, from: f64, to: f64) -> bool {
        if to < 0.0 {
            return true;
        }
        // The samples `pressure` reads at times from..=to (its `floor` does not shrink as the time grows)
        let first = if from < 0.0 { -1 } else { (from * RATE_HZ).floor() as i64 };
        let last = (to * RATE_HZ).floor() as u64 + 1;
        let w = &mut *self.wave;
        if last >= w.computed() {
            w.compute_to(last, self.drive);
        }
        last < w.computed() && w.last_loud.is_none_or(|l| (l as i64) < first)
    }
}

impl Wave {
    fn computed(&self) -> u64 {
        self.base + self.samples.len() as u64
    }

    /// Computes grid steps up to and including `k`, as far as the drive is known.
    fn compute_to(&mut self, k: u64, drive: &Mutex<Drive>) {
        while self.computed() <= k {
            let n = self.computed();
            let start = Time(n * STEP_PS);
            let end = Time((n + 1) * STEP_PS);
            if end > self.progress {
                // What the board has driven since
                let mut d = lock(drive);
                self.progress = d.progress;
                self.taken += d.edges.len() as u64;
                self.edges.extend(d.edges.drain(..));
                drop(d);
                if end > self.progress {
                    return;
                }
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
            let p = (self.resonator.process(v) * self.gain) as f32;
            if p.to_bits() & 0x7FFF_FFFF != 0 {
                self.last_loud = Some(n);
            }
            self.samples.push_back(p);
            if self.samples.len() as u64 > HISTORY_LEN {
                self.samples.pop_front();
                self.base += 1;
            }
        }
    }

    #[inline]
    fn sample(&mut self, k: u64, drive: &Mutex<Drive>) -> f64 {
        if k < self.base {
            return 0.0;
        }
        if k >= self.computed() {
            self.compute_to(k, drive);
        }
        let i = (k - self.base) as usize;
        match self.samples.get(i) {
            Some(v) => *v as f64,
            None => self.samples.back().copied().unwrap_or(0.0) as f64,
        }
    }

    #[inline]
    fn pressure(&mut self, t: f64, drive: &Mutex<Drive>) -> f64 {
        if t < 0.0 {
            return 0.0;
        }
        let x = t * RATE_HZ;
        let k = x.floor() as u64;
        let f = x - k as f64;
        let a = self.sample(k, drive);
        let b = self.sample(k + 1, drive);
        a + (b - a) * f
    }

    /// [`Wave::pressure`] at each of the first `n` times of `t`, into `out`, the same to the bit, reading the samples
    /// as it reads them (the times after the `n`th are copies of a time before it: computed too, and of no use; `k`
    /// and `b` are room for as many values): in loops over the lanes that vectorize (see [`crate::lanes`]) where all
    /// the samples read are computed and none is forgotten; else one time after the other.
    ///
    /// Computing the samples up to the latest of them first, rather than as each time comes, changes nothing: a
    /// sample is computed once and never changes, this computes no further than reading them in turn would, and
    /// forgets none of them (they are far less than the history apart). A caller reading several sets of times
    /// (the hydrophone's steps) keeps all of them that close together (`Hydrophone::lanes_ok`), so that none reads
    /// a sample another one's computing forgot, where reading them in turn would still have had it.
    #[inline(always)]
    fn pressure_lanes(
        &mut self,
        t: &[[f64; LANES]],
        n: usize,
        out: &mut [[f64; LANES]],
        k: &mut [[f64; LANES]],
        b: &mut [[f64; LANES]],
        drive: &Mutex<Drive>,
    ) {
        // `pressure`'s `x`, its `k` (`x.floor() as u64`, which is `floor_small(x)` for these, and exactly an `f64`)
        // and its `f` (into `out`); whether every time is one `floor_small` takes, and every sample read is there
        let (first, last) = (self.base as f64, self.computed() as f64 - 1.0);
        let (mut ok, mut there) = (true, true);
        for ((t, f), k) in t.iter().zip(out.iter_mut()).zip(k.iter_mut()) {
            let (o, h) = lanes::split(t, RATE_HZ, k, f, first, last);
            (ok, there) = (ok & o, there & h);
        }
        if !ok {
            return self.pressure_each(t, n, out, drive);
        }
        if !there {
            let (mut lo, mut hi) = (u64::MAX, 0);
            for &k in k.iter().flatten() {
                (lo, hi) = (lo.min(lanes::whole(k)), hi.max(lanes::whole(k)));
            }
            // Computing up to the latest forgets none of them (the earliest is far less than the history before)
            if hi + 1 >= self.computed() && hi + 2 <= lo + HISTORY_LEN {
                self.compute_to(hi + 1, drive);
            }
            if lo < self.base || hi + 1 >= self.computed() {
                return self.pressure_each(t, n, out, drive);
            }
        }
        // All of them there: each read gives what it would in turn. The two samples each time reads (the first
        // into `k`, which is done with then)
        let (front, back) = self.samples.as_slices();
        let base = self.base;
        for (k, b) in k.iter_mut().zip(b.iter_mut()) {
            for j in 0..LANES {
                let i = (lanes::whole(k[j]) - base) as usize;
                let (x, y) = if i + 1 < front.len() {
                    (front[i], front[i + 1])
                } else if i >= front.len() {
                    (back[i - front.len()], back[i + 1 - front.len()])
                } else {
                    (front[i], back[0])
                };
                (k[j], b[j]) = (x as f64, y as f64);
            }
        }
        for ((f, a), b) in out.iter_mut().zip(k.iter()).zip(b.iter()) {
            lanes::interpolate(f, a, b);
        }
        #[cfg(test)]
        {
            self.lane_reads[0] += 1;
        }
    }

    /// [`Wave::pressure`] at each of the first `n` times of `t` in turn.
    #[cold]
    fn pressure_each(&mut self, t: &[[f64; LANES]], n: usize, out: &mut [[f64; LANES]], drive: &Mutex<Drive>) {
        #[cfg(test)]
        {
            self.lane_reads[1] += 1;
        }
        for i in 0..n {
            out[i / LANES][i % LANES] = self.pressure(t[i / LANES][i % LANES], drive);
        }
    }
}

impl Projector {
    /// Radiated pressure at `t` seconds (µPa at 1 m), interpolated between grid steps.
    pub fn pressure(&self, t: f64) -> f64 {
        self.state.read().pressure(t)
    }

    /// Number of drive level changes seen.
    pub fn edge_count(&self) -> u64 {
        self.state.edge_count()
    }

    /// Grid steps computed so far.
    #[cfg(test)]
    fn computed(&self) -> u64 {
        lock(&self.state.wave).computed()
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
        let wave = Wave {
            drive_v,
            gain,
            resonator,
            edges: VecDeque::new(),
            taken: 0,
            level: false,
            progress: Time::ZERO,
            base: 0,
            samples: VecDeque::new(),
            last_loud: None,
            #[cfg(test)]
            lane_reads: [0; 2],
        };
        let drive_state = Drive { edges: VecDeque::new(), progress: Time::ZERO, pushed: 0 };
        self.projectors.push(Projector {
            owner,
            name: name.to_string(),
            transducer,
            trajectory,
            nominal_hz: drive.nominal_hz.unwrap_or(nominal),
            state: Arc::new(ProjectorState { drive: Apart(Mutex::new(drive_state)), wave: Apart(Mutex::new(wave)) }),
        });
        self.projectors.len() - 1
    }

    /// Projector `id`'s drive pin changed to `level` at `t`.
    pub fn edge(&mut self, id: usize, t: Time, level: bool) {
        self.projectors[id].state.add_edge(t, level);
    }

    /// Board `owner` has run up to `t`: its projectors' drive is known up to then.
    pub fn progress(&mut self, owner: usize, t: Time) {
        for p in self.projectors.iter().filter(|p| p.owner == owner) {
            p.state.add_progress(t);
        }
    }

    /// Where the drives of board `owner`'s projectors are, to take them back there ([`Medium::back_to`]).
    pub fn mark(&self, owner: usize) -> Vec<DriveMark> {
        self.projectors.iter().filter(|p| p.owner == owner).map(|p| p.state.mark()).collect()
    }

    /// Takes the drives of board `owner`'s projectors back to `marks` ([`Medium::mark`]): the board was rolled back
    /// to where they were taken.
    pub fn back_to(&self, owner: usize, marks: &[DriveMark]) {
        for (p, m) in self.projectors.iter().filter(|p| p.owner == owner).zip(marks) {
            p.state.back_to(*m);
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
    fn a_drive_goes_back_to_a_mark() {
        let mut m = medium();
        let id = projector(&mut m);
        square(&mut m, id, 50_000.0, Time::from_ms(2));
        let reference = m.clone();
        let mark = m.mark(0);
        assert_eq!(mark.len(), 1);
        // Driven on, and partly read (the reading takes over level changes it has not folded in yet)
        for k in 0..40u64 {
            m.edge(id, Time::from_ms(2) + Time::from_us(10 * k), k % 2 == 0);
        }
        m.progress(0, Time::from_ms(3));
        let _ = m.projectors[id].pressure(1.5e-3);
        let edges = m.projectors[id].edge_count();
        m.back_to(0, &mark);
        assert_eq!(m.projectors[id].edge_count(), edges - 40);
        // Driven again as before the mark: the waveform is the one the copy taken there computes
        square_from(&mut m, id, Time::from_ms(2), Time::from_ms(4));
        let mut copy = reference;
        square_from(&mut copy, id, Time::from_ms(2), Time::from_ms(4));
        for k in 0..3_000 {
            let t = k as f64 * 1.3e-6;
            assert_eq!(m.projectors[id].pressure(t).to_bits(), copy.projectors[id].pressure(t).to_bits(), "{t}");
        }
    }

    #[test]
    fn copies_drive_and_read_on_their_own() {
        let mut m = medium();
        let id = projector(&mut m);
        let mut copy = m.clone();
        square(&mut copy, id, 50_000.0, Time::from_ms(2));
        assert_eq!(m.projectors[id].edge_count(), 0);
        assert!(copy.projectors[id].pressure(1e-3).abs() > 1.0);
        assert_eq!(m.projectors[id].pressure(1e-3), 0.0);
    }

    /// Level changes every 10 us from `from` to `to`, starting high, and the progress to `to`.
    fn square_from(m: &mut Medium, id: usize, from: Time, to: Time) {
        let mut t = from;
        let mut level = true;
        while t < to {
            m.edge(id, t, level);
            level = !level;
            t += Time::from_us(10);
        }
        m.progress(0, to);
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

    #[test]
    fn live_paths_interpolate_extrapolate_and_forget() {
        let path = LivePath::new([0.0, 0.0, 1.0]);
        let t = Trajectory::Live(path.clone());
        assert!(t.moves());
        assert_eq!(t.at(5.0), [0.0, 0.0, 1.0]);
        for i in 1..=3000 {
            let s = i as f64 * 1e-3;
            path.push(s, [s, 0.0, 1.0]);
        }
        // Older points than the newest one are ignored
        path.push(1.0, [9.0, 9.0, 9.0]);
        assert!((t.at(2.5005)[0] - 2.5005).abs() < 1e-12);
        // Ahead of the recording: on at the last velocity, for at most 10 ms
        assert!((t.at(3.004)[0] - 3.004).abs() < 1e-9);
        assert!((t.at(4.0)[0] - 3.01).abs() < 1e-9);
        // Only the last second is kept: before it, the oldest kept point
        assert!(t.at(0.5)[0] >= 1.99 && t.at(0.5)[0] < 2.0, "{:?}", t.at(0.5));
        // Clones share the recording, deep copies don't
        let copy = path.deep_copy();
        assert_ne!(Trajectory::Live(copy.clone()), t);
        assert_eq!(Trajectory::Live(path.clone()), t);
        path.push(3.5, [100.0, 0.0, 1.0]);
        assert_eq!(copy.last().1[0], 3.0);
        assert_eq!(path.last(), (3.5, [100.0, 0.0, 1.0]));
        // Not something a robot file can say
        assert!(toml::from_str::<toml::Table>("t = 3").unwrap()["t"].clone().try_into::<Trajectory>().is_err());
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
