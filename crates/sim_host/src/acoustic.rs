//! Boards in a shared acoustic medium: the pin device that feeds a board's projector.

use std::sync::Arc;

use sim_acoustics::medium::{SharedMedium, Trajectory};
use sim_acoustics::{Environment, Hydrophone};
use sim_core::analog::AnalogSource;
use sim_core::time::Time;
use sim_devices::DeviceCtx;
use sim_devices::fault::FaultyAnalog;
use sim_devices::pin::PinDevice;

/// Watches a projector's drive pin and records its level changes in the medium.
#[derive(Clone)]
pub struct ProjectorPin {
    medium: SharedMedium,
    id: usize,
    pin: u8,
}

impl ProjectorPin {
    pub fn new(medium: SharedMedium, id: usize, pin: u8) -> Self {
        ProjectorPin { medium, id, pin }
    }
}

impl PinDevice for ProjectorPin {
    fn name(&self) -> String {
        format!("projector (GPIO {})", self.pin)
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        self.medium.lock().unwrap().edge(self.id, ctx.now(), level);
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

/// How far the boards in an acoustic medium can run ahead of each other, so that they can run on threads with the
/// same results as taking turns ([`crate::robot::Robot::run_until`]).
///
/// A board sees another only through the water: its hydrophone reads another board's projector at the time the
/// sound left it, the time it arrives less the delay of each path (the direct one and the reflections, which are
/// longer). A projector's waveform is computed only once its board has run past it (see
/// [`sim_acoustics::medium`]), so while every board stays less than the shortest delay between a hydrophone and
/// another board's projector ahead of where all of them were, every read gets exactly what it would taking turns.
/// The delays are bounded from below over a span of time: the shortest path at its start (less a margin for the
/// hydrophone's 1 ms blocks, whose delays are interpolated, and for its preroll), less how much both ends can close
/// in on each other at their greatest speeds (an image of a moving projector moves as fast as it does).
/// Trajectories recorded as a vehicle simulation runs ([`Trajectory::Live`]) are not known ahead: robots with
/// one take turns.
pub(crate) struct Lookahead {
    env: Arc<Environment>,
    pairs: Vec<Pair>,
}

/// A hydrophone and a projector of another board.
struct Pair {
    rx: Trajectory,
    tx: Trajectory,
    /// The greatest speeds of both, added (m/s).
    closing: f64,
    hz: f64,
}

/// What a lookahead leaves aside: the reads of a hydrophone at a time `t` reach its steps before `t` (an ADC
/// conversion, a sample of the grid and the next one to interpolate), which this covers many times over.
const MARGIN: Time = Time::from_us(50);
/// Before the start of a span, how far back paths may be interpolated from (a block of 1 ms, and the hydrophone's
/// preroll after a gap, 1 ms).
const BEFORE: f64 = 2.5e-3;
/// Paths longer than this do not run apart: the projectors keep 1 s of their waveforms
/// ([`sim_acoustics::medium::HISTORY_S`]), which boards apart could reach from either end.
const LONGEST_S: f64 = 0.4;

/// The greatest speed along `t` (m/s), if it is known ahead.
fn top_speed(t: &Trajectory) -> Option<f64> {
    match t {
        Trajectory::Fixed(_) => Some(0.0),
        Trajectory::Linear { velocity_mps, .. } => Some(sim_acoustics::geometry::norm(*velocity_mps)),
        Trajectory::Waypoints(points) => points.windows(2).try_fold(0f64, |top, w| {
            let ((t0, p0), (t1, p1)) = (w[0], w[1]);
            let d = sim_acoustics::geometry::norm(sim_acoustics::geometry::sub(p1, p0));
            match t1 - t0 {
                dt if dt > 0.0 => Some(top.max(d / dt)),
                _ if d == 0.0 => Some(top),
                // A jump
                _ => None,
            }
        }),
        Trajectory::Live(_) => None,
    }
}

impl Lookahead {
    /// The lookahead of `robot`'s boards in its medium: `None` without a medium; `Err` if a trajectory is not known
    /// ahead (they must take turns).
    pub(crate) fn of(robot: &mut crate::robot::Robot) -> Result<Option<Lookahead>, ()> {
        let Some(medium) = robot.medium.clone() else { return Ok(None) };
        let (env, projectors): (Arc<Environment>, Vec<(usize, Trajectory, f64)>) = {
            let m = medium.lock().unwrap();
            (m.env.clone(), m.projectors.iter().map(|p| (p.owner, p.trajectory.clone(), p.nominal_hz)).collect())
        };
        let mut pairs = Vec::new();
        for (b, board) in robot.boards.iter_mut().enumerate() {
            for h in hydrophones(&mut board.machine).filter(|h| h.medium().is_some_and(|m| Arc::ptr_eq(m, &medium))) {
                for (_, tx, hz) in projectors.iter().filter(|p| p.0 != b) {
                    let closing = top_speed(&h.trajectory).ok_or(())? + top_speed(tx).ok_or(())?;
                    pairs.push(Pair { rx: h.trajectory.clone(), tx: tx.clone(), closing, hz: *hz });
                }
            }
        }
        // Every trajectory must be known ahead, those no hydrophone of another board hears included
        if projectors.iter().any(|p| top_speed(&p.1).is_none()) {
            return Err(());
        }
        Ok(Some(Lookahead { env, pairs }))
    }

    /// How far ahead of `start` (where every board is) the boards can run, for a run of up to `span` from there:
    /// none of them reads another's projector at a time it has not reached by `start`.
    pub(crate) fn ahead(&self, start: Time, span: Time) -> Time {
        let a = (start.as_secs_f64() - BEFORE).max(0.0);
        let b = (start + span).as_secs_f64() + BEFORE;
        let c = self.env.sound_speed;
        let mut shortest = f64::INFINITY;
        for p in &self.pairs {
            let (tx, rx) = (p.tx.at(a), p.rx.at(a));
            let paths = self.env.paths(tx, rx, p.hz);
            let longest = paths.iter().map(|q| q.delay_s).fold(0.0, f64::max);
            if longest + p.closing * (b - a) / c > LONGEST_S {
                return Time::ZERO;
            }
            let direct = sim_acoustics::geometry::norm(sim_acoustics::geometry::sub(tx, rx)) / c;
            let first = paths.iter().map(|q| q.delay_s).fold(direct, f64::min);
            shortest = shortest.min(first - p.closing * (b - a) / c);
        }
        if shortest == f64::INFINITY {
            return Time::MAX;
        }
        Time::from_secs_f64(shortest.max(0.0)).saturating_sub(MARGIN)
    }
}

/// The hydrophone an ADC input's source is, if it is one, looking through a fault injector wrapped around it (an
/// ADC fault on the hydrophone's input).
fn hydrophone_in(src: &mut dyn AnalogSource) -> Option<&mut Hydrophone> {
    let any = src.as_any_mut()?;
    if any.is::<Hydrophone>() {
        return any.downcast_mut::<Hydrophone>();
    }
    hydrophone_in(&mut *any.downcast_mut::<FaultyAnalog>()?.inner)
}

/// The hydrophones attached to a machine's ADC inputs.
pub fn hydrophones(m: &mut rp2040::Machine) -> impl Iterator<Item = &mut Hydrophone> {
    m.sys.analog_inputs.iter_mut().flatten().filter_map(|h| hydrophone_in(&mut **h))
}

/// A line about each of a machine's hydrophones whose receive chain clipped some of the samples the ADC took (the
/// signal was too loud for the chain's gain: amplitudes the firmware measures then stop following the sound), for
/// the end of a run.
pub fn clipping_notes(m: &mut rp2040::Machine) -> Vec<String> {
    hydrophones(m)
        .filter_map(|h| {
            let (n, clipped) = h.clipping();
            (clipped > 0).then(|| {
                format!(
                    "hydrophone: {:.1} % of the {n} samples the ADC took were clipped at the receive chain's rails (too \
                     loud for its assumed gain; see docs/acoustics.md)",
                    100.0 * clipped as f64 / n as f64
                )
            })
        })
        .collect()
}

/// Starts keeping every ADC sample of a machine's hydrophones; returns how many there are.
pub fn record_hydrophones(m: &mut rp2040::Machine) -> usize {
    let mut n = 0;
    for h in hydrophones(m) {
        h.recording.get_or_insert_with(Vec::new);
        n += 1;
    }
    n
}

/// Writes what a machine's hydrophones recorded ([`record_hydrophones`]) as WAV files: the first to `path`, any
/// further one to `path` with `_2`, `_3`, ... before its extension. -1..1 is the ADC input range (0 to 3.3 V), the
/// samples quantised to the RP2040 ADC's 12 bits, `rate` the WAV's sample rate. Returns a line about each file.
pub fn write_hydrophone_wavs(
    m: &mut rp2040::Machine,
    path: &std::path::Path,
    rate: u32,
) -> std::io::Result<Vec<String>> {
    let mut out = Vec::new();
    for (k, h) in hydrophones(m).enumerate() {
        let p = if k == 0 {
            path.to_path_buf()
        } else {
            let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
            let ext = path.extension().map(|e| format!(".{}", e.to_string_lossy())).unwrap_or_default();
            path.with_file_name(format!("{stem}_{}{ext}", k + 1))
        };
        let n = h.recording.as_ref().map_or(0, Vec::len);
        let first = h.recording.as_ref().and_then(|r| r.first()).map(|f| f.0);
        match h.recording_pin_wav(rate, [0.0, 3.3], 12) {
            Some(w) => {
                w.save(&p)?;
                out.push(format!(
                    "hydrophone: {n} ADC sample(s) in {} ({:.3} s at {rate} Hz from {}, -1..1 is 0..3.3 V at the pin)",
                    p.display(),
                    w.duration(),
                    first.unwrap_or_default()
                ));
            }
            None => out.push(format!("hydrophone: no ADC samples to write to {}", p.display())),
        }
    }
    Ok(out)
}

/// Points a machine's hydrophones and projector pins that use medium `old` at `new` (a copy of it).
pub fn relink(m: &mut rp2040::Machine, old: &SharedMedium, new: &SharedMedium) {
    for h in hydrophones(m) {
        if h.medium().is_some_and(|m| std::sync::Arc::ptr_eq(m, old)) {
            h.relink(new.clone());
        }
    }
    for d in m.sys.pin_devices.iter_mut() {
        if let Some(p) = d.as_any_mut().and_then(|a| a.downcast_mut::<ProjectorPin>())
            && std::sync::Arc::ptr_eq(&p.medium, old)
        {
            p.medium = new.clone();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lookahead(rx: Trajectory, tx: Trajectory) -> Lookahead {
        let water = sim_acoustics::water::Water { sound_speed_mps: Some(1500.0), ..Default::default() };
        let env = Arc::new(Environment::open_water(water));
        let closing = top_speed(&rx).unwrap() + top_speed(&tx).unwrap();
        Lookahead { env, pairs: vec![Pair { rx, tx, closing, hz: 50_000.0 }] }
    }

    #[test]
    fn the_lookahead_is_the_shortest_delay_less_what_the_ends_can_close() {
        // 6 m of water at 1500 m/s: 4 ms, less the margin
        let fixed = lookahead(Trajectory::Fixed([0.0, 0.0, 1.0]), Trajectory::Fixed([6.0, 0.0, 1.0]));
        let ahead = fixed.ahead(Time::from_secs(2), Time::from_secs(1));
        assert_eq!(ahead, Time::from_us(4_000) - MARGIN);
        // Closing at 3 m/s: what they can close from 2.5 ms before the span to 2.5 ms after it comes off
        let rx = Trajectory::Linear { start_m: [0.0, 0.0, 1.0], velocity_mps: [1.0, 0.0, 0.0] };
        let tx = Trajectory::Linear { start_m: [6.0, 0.0, 1.0], velocity_mps: [-2.0, 0.0, 0.0] };
        let moving = lookahead(rx, tx);
        let ahead = moving.ahead(Time::ZERO, Time::from_ms(100));
        // (The span starts at 0, so from 0)
        let want = (6.0 - 3.0 * (0.1 + BEFORE)) / 1500.0;
        assert!((ahead.as_secs_f64() - (want - MARGIN.as_secs_f64())).abs() < 1e-9, "{ahead}");
        // Later they are closer, until there is nothing left
        assert!(moving.ahead(Time::from_secs(1), Time::from_ms(100)) < ahead);
        assert_eq!(moving.ahead(Time::from_secs(2), Time::from_ms(100)), Time::ZERO);
        // Nobody to hear: no limit
        assert_eq!(Lookahead { env: fixed.env.clone(), pairs: Vec::new() }.ahead(Time::ZERO, Time::MAX), Time::MAX);
    }

    #[test]
    fn paths_known_ahead_have_a_top_speed() {
        assert_eq!(top_speed(&Trajectory::Fixed([1.0; 3])), Some(0.0));
        let l = Trajectory::Linear { start_m: [0.0; 3], velocity_mps: [3.0, 4.0, 0.0] };
        assert_eq!(top_speed(&l), Some(5.0));
        let w = Trajectory::Waypoints(vec![
            (0.0, [0.0; 3]),
            (2.0, [2.0, 0.0, 0.0]),
            (3.0, [5.0, 0.0, 0.0]),
            (4.0, [5.0, 0.0, 0.0]),
        ]);
        assert_eq!(top_speed(&w), Some(3.0));
        let jump = Trajectory::Waypoints(vec![(1.0, [0.0; 3]), (1.0, [1.0, 0.0, 0.0])]);
        assert_eq!(top_speed(&jump), None);
        assert_eq!(top_speed(&Trajectory::Live(sim_acoustics::medium::LivePath::new([0.0; 3]))), None);
    }
    use sim_acoustics::chain::ChainSpec;
    use sim_acoustics::medium::Medium;
    use sim_acoustics::transducer::Transducer;
    use sim_acoustics::water::Water;

    #[test]
    fn hydrophone_recordings_are_written_as_wav() {
        use sim_acoustics::chain::ChainSpec;
        use sim_acoustics::transducer::Transducer;
        let env = std::sync::Arc::new(sim_acoustics::Environment::open_water(Default::default()));
        let chain = ChainSpec { bias_v: Some(2.475), ..Default::default() };
        let h = Hydrophone::new(env, [0.0; 3], Transducer::hydrophone(-190.0), chain);
        let mut m = rp2040::Machine::new(Default::default());
        m.sys.attach_analog(1, Box::new(h));
        assert_eq!(record_hydrophones(&mut m), 1);
        for k in 0..100u64 {
            m.sys.analog_inputs[1].as_mut().unwrap().voltage(sim_core::time::Time::from_us(2 * k), 0);
        }
        let dir = std::env::temp_dir().join(format!("hydro-wav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let lines = write_hydrophone_wavs(&mut m, &dir.join("h.wav"), 500_000).unwrap();
        assert!(lines[0].contains("100 ADC sample(s)"), "{lines:?}");
        let w = sim_acoustics::wav::Wav::load(&dir.join("h.wav")).unwrap();
        assert_eq!((w.rate, w.samples.len()), (500_000, 100));
        // A quiet pin at 2.475 V is half way up the upper half of 0..3.3 V (to the nearest 12-bit step)
        assert!(w.samples.iter().all(|s| (s - 0.5).abs() < 1e-3), "{:?}", w.samples);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn hydrophones_are_found_behind_a_fault_injector() {
        let medium = Medium::shared(std::sync::Arc::new(sim_acoustics::Environment::open_water(Water::default())));
        let env = medium.lock().unwrap().env.clone();
        let h = Hydrophone::new(env, [0.0; 3], Transducer::hydrophone(-190.0), ChainSpec::default())
            .in_medium(medium.clone(), 0);
        let mut m = rp2040::Machine::new(Default::default());
        m.sys.attach_analog(0, Box::new(FaultyAnalog::new(Box::new(h), 1)));
        assert_eq!(hydrophones(&mut m).count(), 1);
        // A snapshot's copy of the machine is moved to the copy of the medium
        let copy = std::sync::Arc::new(std::sync::Mutex::new(medium.lock().unwrap().clone()));
        relink(&mut m, &medium, &copy);
        assert!(std::sync::Arc::ptr_eq(hydrophones(&mut m).next().unwrap().medium().unwrap(), &copy));
    }

    #[test]
    fn clipped_samples_are_reported() {
        use sim_acoustics::source::{Source, Waveform};
        // A 30 kHz tone at 150 dB re 1 uPa, 2 m away: 1 V peak at the pin with 40 dB of gain, inside the rails;
        // 20 dB louder it clips
        let mut notes = Vec::new();
        for level in [150.0, 170.0] {
            let mut env = sim_acoustics::Environment::open_water(Water::default());
            env.sources.push(Source {
                position: [2.0, 0.0, 0.0],
                waveform: Waveform::Tone { hz: 30_000.0 },
                rms_upa: 10f64.powf(level / 20.0),
            });
            let chain = ChainSpec { gain_db: 40.0, ..Default::default() };
            let h = Hydrophone::new(std::sync::Arc::new(env), [0.0; 3], Transducer::hydrophone(-190.0), chain);
            let mut m = rp2040::Machine::new(Default::default());
            m.sys.attach_analog(0, Box::new(h));
            for k in 0..1000u64 {
                m.sys.analog_inputs[0].as_mut().unwrap().voltage(sim_core::time::Time::from_us(k * 2), 0);
            }
            notes.push(clipping_notes(&mut m));
        }
        assert!(notes[0].is_empty(), "{:?}", notes[0]);
        assert_eq!(notes[1].len(), 1);
        assert!(notes[1][0].contains("% of the 1000 samples the ADC took were clipped"), "{:?}", notes[1]);
    }
}
