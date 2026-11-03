//! Boards of a robot without an acoustic medium can run on several threads within a slice. The result must be
//! exactly that of running them in turn: every CAN frame at the same time, every log line at the same time, the
//! same machine state.

mod common;

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::Robot;

/// Talos's boards (with one switched on later, as in the robot file) for 600 ms, on `threads` threads; with
/// `profile`, every board's firmware profiled (its folded stacks are part of the state).
fn run(threads: usize, profile: bool) -> Option<Run> {
    run_in(threads, profile, &[Time::from_ms(600)], None)
}

/// CAN frames, the boards' state, and the mock bridge.
type Run = (Vec<String>, Vec<String>, MockBridge);

/// The thread each Talos board is put on by [`fixed`]: the ESC boards on threads of their own, the rest on the
/// calling one (as they are spread on an idle machine; how they are spread depends on the load).
const FIXED: [usize; 9] = [0, 1, 2, 0, 0, 0, 0, 0, 0];

/// Talos on the threads of [`FIXED`] for 600 ms.
fn fixed(profile: bool) -> Option<Run> {
    run_in(3, profile, &[Time::from_ms(600)], Some(FIXED.to_vec()))
}

/// Talos as [`run`], run to each of `ends` in turn, with the boards on the threads `fixed` if given.
fn run_in(threads: usize, profile: bool, ends: &[Time], fixed: Option<Vec<usize>>) -> Option<Run> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    if let Some(f) = &fixed {
        assert_eq!(r.names[1..3], ["esc0", "esc1"]);
        r.fix_threads(f.clone());
    }
    assert!(r.medium.is_none());
    r.threads = threads;
    if profile {
        for s in &mut r.boards {
            s.start_profile(Time::from_us(10));
        }
    }
    for rb in &mut r.buses {
        rb.bus.record = Some(Vec::new());
    }
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    r.bus("internal").unwrap().endpoints.push(Box::new(bridge.clone()));
    for &end in ends {
        assert!(r.run_until(end).is_none());
    }
    if let Some(fixed) = fixed {
        assert_eq!(r.thread_of_boards(), fixed);
    }
    let mut frames = Vec::new();
    for rb in &r.buses {
        for rec in rb.bus.record.as_ref().unwrap() {
            frames.push(format!("{} {} {} {:?}", rec.start, rec.end, rec.acked, rec.frame));
        }
    }
    let mut state = Vec::new();
    for (name, s) in r.names.iter().zip(&r.boards) {
        for (t, line) in &s.logs {
            state.push(format!("{name} [{t}] {line}"));
        }
        let m = &s.machine;
        for (c, core) in m.cores.iter().enumerate() {
            let regs: Vec<u32> = (0..core.num_regs()).map(|n| core.read_reg(n)).collect();
            state.push(format!("{name} core{c} {} {} {regs:x?}", m.core_time(c), core.cycles()));
        }
        state.push(format!("{name} sram {}", m.sys.sram.iter().map(|&b| b as u64).sum::<u64>()));
        if let Some(p) = &s.profiler {
            let report = p.report();
            assert!(report.cores[0].samples() > 50_000, "{name}: {} samples", report.cores[0].samples());
            state.push(format!("{name} profile {}", report.folded()));
        }
    }
    // Talos's vehicle is stepped after each slice on this thread, from the boards' state
    assert_eq!(r.vehicles.len(), 1);
    for v in &r.vehicles {
        state.push(format!("{} {:?} {:?}", v.vehicle.name, v.vehicle.state, v.rpms(&r.boards, r.now)));
    }
    Some((frames, state, bridge))
}

#[test]
fn threads_give_the_same_run_as_turns() {
    let Some((frames1, state1, bridge1)) = run(1, false) else { return };
    let received = |b: &MockBridge| format!("{:?}", b.state().received);
    assert!(frames1.len() > 20, "{} frames", frames1.len());
    assert!(!bridge1.state().received.is_empty());
    // Spread by cost (as the load on the machine allows), and on threads given
    for (frames, state, bridge) in [run(4, false).unwrap(), fixed(false).unwrap()] {
        assert_eq!(frames1, frames);
        assert_eq!(state1, state);
        assert_eq!(received(&bridge1), received(&bridge));
    }
}

/// Any thread count, and runs in pieces of any length (short ones take turns on the calling thread, longer ones
/// start their threads afresh and keep the grouping found before), give the same run.
#[test]
fn any_thread_count_and_any_pieces_give_the_same_run() {
    let Some((frames1, state1, bridge1)) = run(1, false) else { return };
    let received = |b: &MockBridge| format!("{:?}", b.state().received);
    for threads in [2, 3, 0] {
        let (frames, state, bridge) = run(threads, false).unwrap();
        assert_eq!(frames1, frames, "{threads} threads");
        assert_eq!(state1, state, "{threads} threads");
        assert_eq!(received(&bridge1), received(&bridge), "{threads} threads");
    }
    // Pieces: 30 µs (turns), 1.23 ms, 0.7 ms, 50 ms (regroups), 2 slices, then the rest
    let mut ends = Vec::new();
    let mut t = Time::ZERO;
    for d in [30, 1230, 700, 50_000, 200, 1230, 700, 50_000, 200].iter().cycle() {
        t = (t + Time::from_us(*d)).min(Time::from_ms(600));
        ends.push(t);
        if t == Time::from_ms(600) {
            break;
        }
    }
    // Pieces end slices where they end, so they are compared with the same pieces in turn
    let (frames1, state1, bridge1) = run_in(1, false, &ends, None).unwrap();
    let (frames, state, bridge) = run_in(3, false, &ends, Some(FIXED.to_vec())).unwrap();
    assert_eq!(frames1, frames);
    assert_eq!(state1, state);
    assert_eq!(received(&bridge1), received(&bridge));
}

/// Which board stopped and why, and the time every board got to.
type Stopped = (Option<(usize, rp2040::StopReason)>, Vec<Time>);

/// Talos's boards with a breakpoint on two of them, run on `threads` threads.
fn stops(threads: usize) -> Option<Stopped> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    r.threads = threads;
    if threads > 1 {
        // esc1 and led on workers
        r.fix_threads(vec![0, 0, 1, 0, 0, 2, 0, 0, 0]);
    }
    assert!(r.run_until(Time::from_ms(300)).is_none());
    // Somewhere every embassy image comes back to: the time driver's alarm interrupt (its second instruction:
    // the first runs in the step that enters the handler)
    for name in ["led", "esc1"] {
        let m = &mut r.board(name).unwrap().machine;
        let image = m.image.clone().unwrap();
        m.breakpoints.insert((image.symbol("TIMER_IRQ_0").unwrap().addr & !1) + 2);
    }
    let stop = r.run_until(Time::from_ms(310));
    Some((stop, r.boards.iter().map(|b| b.machine.now).collect()))
}

/// A board that stops (here at a breakpoint) on a worker thread is reported as on the calling thread, the lowest
/// index first.
#[test]
fn a_stop_on_a_worker_is_reported() {
    let Some((stop, _)) = stops(1) else { return };
    let (stop4, times) = stops(3).unwrap();
    let (led, esc1) = (5, 2);
    assert!(matches!(stop, Some((i, rp2040::StopReason::Breakpoint { .. })) if i == led || i == esc1), "{stop:?}");
    assert_eq!(stop, stop4);
    // With threads, every board has run to the end of the slice the stop came in (the stopped ones excepted;
    // a machine may end a little past the time asked for, by the instruction it was in)
    let slice = Time::from_us(100);
    let end = Time(times[0].as_ps() / slice.as_ps() * slice.as_ps());
    assert!(end > Time::from_ms(300) && end <= Time::from_ms(310), "{end}");
    for (i, t) in times.iter().enumerate() {
        if i != led && i != esc1 {
            assert!(*t >= end && *t < end + Time::from_us(1), "board {i}: {t}");
        }
    }
}

/// Profilers run on the worker threads with their boards: they cut the runs at the same sample times and
/// collect the same samples as when the boards take turns.
#[test]
fn profiled_boards_on_threads_give_the_same_run_and_profiles() {
    let Some((frames1, state1, _)) = run(1, true) else { return };
    let (frames4, state4, _) = fixed(true).unwrap();
    assert!(frames1.len() > 20, "{} frames", frames1.len());
    assert_eq!(frames1, frames4);
    assert!(state1.iter().any(|l| l.contains(" profile core0;")));
    assert_eq!(state1, state4);
}
