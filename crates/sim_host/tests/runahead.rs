//! Boards of a robot run ahead of each other between the slices where they can see each other
//! (`sim_host::runahead`). The result must be exactly that of taking turns slice by slice: every CAN frame at the
//! same time, every log line at the same time and in the same order across boards, the same machine state, the
//! same statistics.

mod common;

use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::Robot;
use sim_host::timetravel::{Ran, TimeTravel, Travelled};

/// How a run is done.
#[derive(Clone, Default)]
struct How {
    threads: usize,
    run_ahead: bool,
    /// Boards on these threads (see `Robot::fix_threads`).
    fixed: Option<Vec<usize>>,
    /// `Robot::run_ahead_stress`.
    stress: Option<u64>,
    /// Pieces to run (`run_until` ends); one piece to `END` if empty.
    ends: Vec<Time>,
    /// Edits the robot (after loading it, before running).
    sink_vehicle: bool,
    /// Records the run for time travel, with a checkpoint this often (`sim_host::timetravel`)...
    record: Option<Time>,
    /// ...and once at the end, goes back to this time and runs to the end again (a replay).
    rewind: Option<Time>,
    /// Collects code coverage.
    coverage: bool,
}

const END: Time = Time::from_ms(700);

/// The ESC boards on threads of their own, the rest on the calling one.
const FIXED: [usize; 9] = [0, 1, 2, 0, 0, 0, 0, 0, 0];

/// Everything the run produced.
#[derive(PartialEq, Debug)]
struct Run {
    frames: Vec<String>,
    /// Every log and UART line from every board as the hooks saw them, in the order they saw them.
    hooks: Vec<String>,
    state: Vec<String>,
    bridge: String,
    /// Every board's code coverage (with `How::coverage`).
    coverage: Vec<Option<Box<rp2040::coverage::CoverageMap>>>,
}

/// Talos (one board switched on later, as in the robot file) with a mock bridge, run as `how` says. Returns the
/// run and what running ahead did.
fn talos(how: &How) -> Option<(Run, sim_host::runahead::Stats)> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    assert_eq!(r.names[1..3], ["esc0", "esc1"]);
    r.threads = how.threads;
    r.run_ahead = how.run_ahead;
    r.run_ahead_stress = how.stress;
    if let Some(f) = &how.fixed {
        r.fix_threads(f.clone());
    }
    if how.sink_vehicle {
        // Started below where it floats, the vehicle rises from the first slice: the camera cage's depth sensor
        // (its pressure input written by the vehicle) changes every millisecond
        let v = r.vehicle("talos").unwrap();
        let mut s = v.vehicle.state;
        s.position[2] += 1.5;
        v.vehicle.place(s);
        v.update_outputs();
    }
    for rb in &mut r.buses {
        rb.bus.record = Some(Vec::new());
    }
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    r.bus("internal").unwrap().endpoints.push(Box::new(bridge.clone()));
    let hooks = Arc::new(Mutex::new(Vec::new()));
    for (i, s) in r.boards.iter_mut().enumerate() {
        let h = hooks.clone();
        s.on_log = Some(Box::new(move |t, line| h.lock().unwrap().push(format!("{i} [{t}] {line}"))));
        let h = hooks.clone();
        s.on_uart_line = Some(Box::new(move |t, n, line| h.lock().unwrap().push(format!("{i} [{t}] uart{n} {line}"))));
    }
    if how.coverage {
        for s in &mut r.boards {
            s.machine.enable_coverage();
        }
    }
    let ends = if how.ends.is_empty() { vec![END] } else { how.ends.clone() };
    let inputs = r.inputs.clone();
    let mut tt = how.record.map(|every| {
        let mut tt = TimeTravel::new(&mut r, inputs);
        tt.every = every;
        tt
    });
    for &end in &ends {
        match &mut tt {
            Some(tt) => assert_eq!(tt.run_until(&mut r, end), Ran::Reached),
            None => assert!(r.run_until(end).is_none()),
        }
        assert_eq!(r.now, end);
    }
    if let (Some(tt), Some(every)) = (&tt, how.record) {
        // A checkpoint every `every`, or where the first window that ends after it ends (64 slices at most)
        let at_least = ends.last().unwrap().as_ps() / (every.as_ps() + r.slice.as_ps() * 64);
        assert!(tt.stats().taken >= at_least, "{:?}", tt.stats());
    }
    if let (Some(tt), Some(t)) = (&mut tt, how.rewind) {
        let before = r.run_ahead_stats;
        assert_eq!(tt.rewind_to(&mut r, t), Travelled::Arrived);
        assert!(r.now <= t && r.now + r.slice > t, "{} for {t}", r.now);
        let end = *ends.last().unwrap();
        assert_eq!(tt.run_until(&mut r, end), Ran::Reached);
        assert!(tt.at_frontier());
        // The replay ran ahead too (its bridge's frames come from the record)
        if how.run_ahead && how.threads != 1 {
            let replayed = r.run_ahead_stats.slices - before.slices;
            assert!(replayed * 10 >= (end - t).as_ps() / r.slice.as_ps() * 9, "{before:?} {:?}", r.run_ahead_stats);
        }
    }
    if let Some(fixed) = &how.fixed {
        assert_eq!(&r.thread_of_boards(), fixed);
    }
    let mut frames = Vec::new();
    for rb in &r.buses {
        for rec in rb.bus.record.as_ref().unwrap() {
            frames.push(format!("{} {} {:?} {} {:?}", rec.start, rec.end, rec.sender, rec.acked, rec.frame));
        }
    }
    let mut state = Vec::new();
    for (name, s) in r.names.iter().zip(&r.boards) {
        for (t, line) in &s.logs {
            state.push(format!("{name} [{t}] {line}"));
        }
        state.push(format!("{name} rtt {:?} uart {:?}", s.rtt_text, s.uart_text));
        let m = &s.machine;
        for (c, core) in m.cores.iter().enumerate() {
            let regs: Vec<u32> = (0..core.num_regs()).map(|n| core.read_reg(n)).collect();
            state.push(format!(
                "{name} core{c} {} {} {} {regs:x?}",
                m.core_time(c),
                core.cycles(),
                core.instructions()
            ));
        }
        let mut h = std::collections::hash_map::DefaultHasher::new();
        m.sys.sram.hash(&mut h);
        m.sys.flash[..].hash(&mut h);
        state.push(format!("{name} now {} memory {:x}", m.now, h.finish()));
        state.push(format!(
            "{name} skipped {} timer waits {} fused {}",
            m.spin_skipped, m.timer_waits_skipped, m.quanta_fused
        ));
        let devices: Vec<String> = m
            .sys
            .pin_devices
            .iter()
            .filter_map(|d| d.as_any().downcast_ref::<sim_devices::dshot::Esc>())
            .map(|e| format!("{} {:?} {}", e.name, e.motor(), e.frames.len()))
            .collect();
        state.push(format!("{name} escs {devices:?}"));
    }
    for v in &r.vehicles {
        state.push(format!("{} {:?} {:?}", v.vehicle.name, v.vehicle.state, v.rpms(&r.boards, r.now)));
    }
    let bridge = format!("{:?}", bridge.state().received);
    let hooks = std::mem::take(&mut *hooks.lock().unwrap());
    let coverage = r.boards.iter().map(|s| s.machine.coverage.clone()).collect();
    Some((Run { frames, hooks, state, bridge, coverage }, r.run_ahead_stats))
}

/// Asserts two runs are the same, showing where they first differ.
#[track_caller]
fn same(want: &Run, got: &Run, what: &str) {
    for (name, a, b) in
        [("frames", &want.frames, &got.frames), ("hooks", &want.hooks, &got.hooks), ("state", &want.state, &got.state)]
    {
        if let Some(i) = (0..a.len().max(b.len())).find(|&i| a.get(i) != b.get(i)) {
            panic!("{what}: {name} differ at {i}:\n  want {:?}\n  got  {:?}", a.get(i), b.get(i));
        }
    }
    assert_eq!(want.bridge, got.bridge, "{what}: what the bridge received");
    for (i, (a, b)) in want.coverage.iter().zip(&got.coverage).enumerate() {
        assert!(a == b, "{what}: board {i}'s coverage differs");
    }
}

fn turns() -> How {
    How { threads: 1, ..Default::default() }
}

fn ahead(threads: usize) -> How {
    How { threads, run_ahead: true, ..Default::default() }
}

#[test]
fn running_ahead_gives_the_same_run_as_turns() {
    let Some((want, none)) = talos(&turns()) else { return };
    assert_eq!(none, Default::default(), "one thread takes turns");
    assert!(want.frames.len() > 20, "{} frames", want.frames.len());
    assert!(want.hooks.len() > 20, "{} lines", want.hooks.len());
    assert!(want.bridge.len() > 100);
    // On threads slice by slice, as before
    let (lockstep, none) = talos(&How { threads: 3, fixed: Some(FIXED.to_vec()), ..Default::default() }).unwrap();
    assert_eq!(none, Default::default());
    same(&want, &lockstep, "slice by slice");
    for how in [ahead(3), ahead(2), ahead(0), How { fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&how).unwrap();
        same(&want, &got, &format!("{} threads", how.threads));
        // Most of the run was ahead: windows are many slices long
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 8, "{stats:?}");
    }
}

/// Every board ignoring where its window ends until it has been rolled back once, and every window cut at a random
/// boundary: boards are rolled back all the time, and the run stays the same.
#[test]
fn rolled_back_boards_rerun_the_same() {
    let Some((want, _)) = talos(&turns()) else { return };
    for seed in [1, 2] {
        let (got, stats) = talos(&How { stress: Some(seed), fixed: Some(FIXED.to_vec()), ..ahead(3) }).unwrap();
        same(&want, &got, &format!("seed {seed}"));
        assert!(stats.rollbacks > 2000, "{stats:?}");
    }
}

/// A rising vehicle changes the camera cage's depth input every millisecond; the camera cage follows the vehicle
/// in the commit rather than running ahead, and its frames cut windows short.
#[test]
fn boards_on_a_moving_vehicle_see_it_move_as_in_turns() {
    let sink = How { sink_vehicle: true, ..Default::default() };
    let Some((want, _)) = talos(&How { threads: 1, ..sink.clone() }) else { return };
    let (got, stats) =
        talos(&How { threads: 3, run_ahead: true, fixed: Some(FIXED.to_vec()), ..sink.clone() }).unwrap();
    same(&want, &got, "");
    assert!(stats.follower_cuts > 0, "{stats:?}");
    let (got, _) = talos(&How { threads: 3, run_ahead: true, stress: Some(7), ..sink }).unwrap();
    same(&want, &got, "");
    // It did move
    let (still, _) = talos(&turns()).unwrap();
    assert_ne!(still.state.last(), want.state.last());
}

/// Runs in pieces of any length (short ones take turns; a piece ends a window) give the same run.
#[test]
fn any_pieces_give_the_same_run() {
    let mut ends = Vec::new();
    let mut t = Time::ZERO;
    for d in [30, 1230, 700, 50_000, 200, 6_431, 777, 23_000].iter().cycle() {
        t = (t + Time::from_us(*d)).min(END);
        ends.push(t);
        if t == END {
            break;
        }
    }
    let Some((want, _)) = talos(&How { ends: ends.clone(), ..turns() }) else { return };
    for stress in [None, Some(3)] {
        let (got, _) = talos(&How { ends: ends.clone(), stress, fixed: Some(FIXED.to_vec()), ..ahead(3) }).unwrap();
        same(&want, &got, &format!("stress {stress:?}"));
    }
}

/// Which board stopped and why, the time every board got to and the robot's time.
type Stopped = (Option<(usize, rp2040::StopReason)>, Vec<Time>, Time);

/// Talos with breakpoints on two boards, run ahead on threads (`stress` as `Robot::run_ahead_stress`) or slice by
/// slice on threads.
fn stops(run_ahead: bool, stress: Option<u64>) -> Option<Stopped> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    r.threads = 3;
    r.run_ahead = run_ahead;
    r.run_ahead_stress = stress;
    r.fix_threads(vec![0, 0, 1, 0, 0, 2, 0, 0, 0]);
    assert!(r.run_until(Time::from_ms(300)).is_none());
    for name in ["led", "esc1"] {
        let m = &mut r.board(name).unwrap().machine;
        let image = m.image.clone().unwrap();
        m.breakpoints.insert((image.symbol("TIMER_IRQ_0").unwrap().addr & !1) + 2);
    }
    let stop = r.run_until(Time::from_ms(310));
    Some((stop, r.boards.iter().map(|b| b.machine.now).collect(), r.now))
}

/// A board that stops ends its window there and is reported as on threads slice by slice: lowest index first, the
/// others run to the end of that slice, the robot left at the slice's start.
#[test]
fn a_stop_ends_the_window_as_on_threads() {
    let Some(want) = stops(false, None) else { return };
    assert!(matches!(want.0, Some((_, rp2040::StopReason::Breakpoint { .. }))), "{want:?}");
    assert_eq!(stops(true, None).unwrap(), want);
    assert_eq!(stops(true, Some(5)).unwrap(), want);
}

/// An endpoint that cannot tell when it will send (as a bridge to real host tools).
struct Unknown;

impl sim_devices::can::CanEndpoint for Unknown {
    fn take_tx(&mut self, _until: Time) -> Vec<(Time, sim_devices::can::CanFrame)> {
        Vec::new()
    }
    fn receive(&mut self, _frame: &sim_devices::can::CanFrame, _at: Time) {}
}

/// A board watched as it runs (a trace sink), or an endpoint that cannot tell when it sends, keeps the boards in
/// step slice by slice on threads; the switch does too.
#[test]
fn what_cannot_be_run_ahead_runs_slice_by_slice() {
    if common::titan_elf("esc_board").is_none() {
        return;
    }
    let windows = |edit: &dyn Fn(&mut Robot)| {
        let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
        r.threads = 3;
        edit(&mut r);
        assert!(r.run_until(Time::from_ms(20)).is_none());
        r.run_ahead_stats.windows
    };
    assert!(windows(&|_| {}) > 0);
    assert_eq!(windows(&|r| r.run_ahead = false), 0);
    assert_eq!(windows(&|r| r.threads = 1), 0);
    assert_eq!(
        windows(&|r| {
            let sink = sim_core::trace::VecSink::new(&[sim_core::trace::Category::Gpio]);
            r.boards[4].machine.set_trace(Box::new(sink));
        }),
        0
    );
    assert_eq!(windows(&|r| r.buses[0].endpoints.push(Box::new(Unknown))), 0);
    assert!(windows(&|r| r.buses[0].endpoints.push(Box::new(sim_devices::can::Monitor::default()))) > 0);
}

/// Recording for time travel takes its checkpoints at boundaries the robot committed (a run is cut into pieces
/// that end where a checkpoint is due, and a window never goes past the end of a run), and the recorded bridge
/// still tells when it will send: the robot runs ahead while recording, and in replays, with the same result as
/// taking turns without recording.
#[test]
fn recording_for_time_travel_runs_ahead_the_same() {
    let Some((want, _)) = talos(&turns()) else { return };
    let record = |how: How| How { record: Some(Time::from_ms(10)), ..how };
    let (got, none) = talos(&record(turns())).unwrap();
    same(&want, &got, "recorded, in turns");
    assert_eq!(none, Default::default());
    for how in [ahead(3), How { stress: Some(4), fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&record(how.clone())).unwrap();
        same(&want, &got, &format!("recorded, ahead, stress {:?}", how.stress));
        assert!(stats.slices >= 6900 && stats.windows < stats.slices / 6, "{stats:?}");
    }
    // Back to a time and on to the end again: replayed ahead
    let (got, _) = talos(&How { rewind: Some(Time::from_us(237_450)), ..record(ahead(3)) }).unwrap();
    same(&want, &got, "recorded, ahead, replayed");
    let (got, _) = talos(&How { rewind: Some(Time::from_us(237_450)), ..record(turns()) }).unwrap();
    same(&want, &got, "recorded, in turns, replayed");
}

/// Code coverage counts what the committed run executed: what a board ran before it was rolled back does not
/// count, so the coverage is the same running ahead as taking turns (and with rollbacks forced).
#[test]
fn coverage_is_the_same_running_ahead() {
    let end = Time::from_ms(250);
    let cov = |how: How| How { coverage: true, ends: vec![end], ..how };
    let Some((want, _)) = talos(&cov(turns())) else { return };
    assert!(want.coverage.iter().all(Option::is_some));
    for how in [ahead(3), How { stress: Some(9), fixed: Some(FIXED.to_vec()), ..ahead(3) }] {
        let (got, stats) = talos(&cov(how.clone())).unwrap();
        same(&want, &got, &format!("coverage, ahead, stress {:?}", how.stress));
        assert!(stats.windows > 0 && stats.rollbacks > 0, "{stats:?}");
    }
}

/// A recorded robot takes turns while a debugger could stop it (GDB attached, so a `BKPT` stops; a breakpoint set),
/// so that stops are as a replay taking turns gives them; otherwise it runs ahead.
#[test]
fn a_recorded_robot_takes_turns_while_debugged() {
    if common::titan_elf("esc_board").is_none() {
        return;
    }
    let windows = |edit: &dyn Fn(&mut Robot)| {
        let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
        r.threads = 3;
        edit(&mut r);
        let inputs = r.inputs.clone();
        let mut tt = TimeTravel::new(&mut r, inputs);
        assert_eq!(tt.run_until(&mut r, Time::from_ms(20)), Ran::Reached);
        r.run_ahead_stats.windows
    };
    assert!(windows(&|_| {}) > 0);
    assert_eq!(windows(&|r| r.boards[2].machine.bkpt_policy = rp2040::machine::BkptPolicy::Stop), 0);
    assert_eq!(windows(&|r| _ = r.boards[4].machine.breakpoints.insert(0x2004_0000)), 0);
}
