//! Time travel on titan_firmware_rs images: a board that panics (an injected fault) is gone back to its panic
//! handler from after the reboot, and one step further back, before the fault; a whole robot (boards, buses,
//! vehicle) goes back to a breakpoint hit on one board and to a time. Each state equals a fresh run stopped there.

use rp2040::StopReason;
use sim_core::cpu::Cpu;
use sim_host::Session;
use sim_host::fault::Fault;
use sim_host::timetravel::{Pos, Ran, TimeTravel, Travelled};
use sim_test::{Bench, RobotBench, Time};

fn state(s: &mut Session) -> Vec<String> {
    let next = s.machine.sys.sched.next_time();
    let m = &s.machine;
    let mut out = vec![format!("now {} resets {} next event {next}", m.now, m.resets)];
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        out.push(format!(
            "core{c} {} at {}: {} cycles, sleep {:?}, regs {regs:08x?}",
            m.core_enabled[c],
            m.core_time(c),
            core.cycles(),
            core.sleep_state()
        ));
    }
    let hash = |b: &[u8]| b.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    out.push(format!(
        "sram {:x} flash {:x} scratch {:x?}",
        hash(&m.sys.sram),
        hash(&m.sys.flash),
        m.sys.watchdog.scratch
    ));
    out.push(format!("logs {}", s.logs.len()));
    out.extend(s.logs.iter().rev().take(3).map(|(t, l)| format!("{t} {l}")));
    out
}

fn power_board() -> Option<Bench> {
    let elf = sim_test::titan_rs("power_board")?;
    Some(Bench::builder(&elf).board("mk2/power_board").bridge(1).boot().unwrap())
}

/// Runs a fresh `make()` until core `core` reaches `time` (its first step at or after it).
fn fresh_until(s: &mut Session, core: usize, time: Time) {
    s.machine.stop_at[core] = Some(time);
    loop {
        match s.run_until(time + Time::from_secs(1)) {
            StopReason::Reached { core: c, .. } if c as usize == core => return,
            StopReason::Reached { .. } => {}
            r => panic!("did not reach {time}: {r:?}"),
        }
    }
}

#[test]
fn a_panic_is_gone_back_to_from_after_the_reboot() {
    let Some(mut b) = power_board() else { return };
    let mut tt = TimeTravel::new(&mut b.session, vec![b.inputs.clone()]);
    tt.every = Time::from_ms(20);
    assert_eq!(tt.run_until(&mut b.session, Time::from_ms(500)), Ran::Reached);
    // The fault, a host change: kept as a checkpoint replays never go through
    b.inject(Fault::Panic).unwrap();
    tt.changed(&mut b.session);
    assert_eq!(tt.run_until(&mut b.session, Time::from_ms(900)), Ran::Reached);
    assert!(b.session.machine.resets >= 1, "the panic rebooted the board");
    assert!(b.logged("PANIC"), "{:?}", b.session.logs.iter().rev().take(5).collect::<Vec<_>>());
    let end = state(&mut b.session);

    // Back to the panic handler
    let handler = b.session.machine.image.as_ref().unwrap().symbol("__defmt_default_panic").unwrap().addr & !1;
    b.session.machine.breakpoints.insert(handler);
    let r = tt.reverse_continue(&mut b.session);
    let Travelled::Hit(hit) = r else { panic!("{r:?}") };
    assert_eq!(b.session.machine.cores[0].pc(), handler);
    let at_hit = state(&mut b.session);

    // A fresh run stopped there
    let mut f = power_board().unwrap();
    f.run_until(Time::from_ms(500)).unwrap();
    f.inject(Fault::Panic).unwrap();
    f.session.machine.breakpoints.insert(handler);
    let r = f.session.run_until(Time::from_ms(900));
    assert!(matches!(r, StopReason::Breakpoint { core: 0, .. }), "{r:?}");
    assert_eq!(hit.pos, Pos::At { board: 0, core: 0, time: f.session.machine.core_time(0) });
    assert_eq!(at_hit, state(&mut f.session));

    // One step back: before the fault was injected, where the run had got to
    b.session.machine.breakpoints.clear();
    assert_eq!(tt.reverse_step(&mut b.session, 0, 0), Travelled::Arrived);
    let Pos::At { time, .. } = tt.here() else { panic!("{:?}", tt.here()) };
    assert!(time < Time::from_ms(500), "{time}");
    let mut g = power_board().unwrap();
    fresh_until(&mut g.session, 0, time);
    assert_eq!(state(&mut b.session), state(&mut g.session));

    // And forward again over the change, to where the run had got
    assert_eq!(tt.run_until(&mut b.session, Time::from_ms(900)), Ran::Reached);
    assert!(tt.at_frontier());
    assert_eq!(state(&mut b.session), end);
}

/// Every board's state, the buses' error counters and the vehicle's pose.
fn robot_state(r: &mut sim_host::robot::Robot) -> Vec<String> {
    let mut out = vec![format!("now {}", r.now)];
    for (n, b) in r.names.clone().iter().zip(r.boards.iter_mut()) {
        out.push(n.clone());
        out.extend(state(b));
    }
    for v in &r.vehicles {
        out.push(format!("{:?}", v.vehicle.state));
    }
    for b in &r.buses {
        out.push(format!("bus free at {} dropped {}", b.bus.free_at, b.bus.endpoint_dropped));
    }
    out
}

#[test]
fn a_robot_goes_back_whole() {
    robot_goes_back_whole(1);
}

/// The same with the robot on threads, its boards run ahead of each other while it was recorded (and while going
/// forward again): stops for the debugger are taken in turns, from a history made running ahead, and every state
/// is the one fresh robots taking turns have.
#[test]
fn a_robot_that_ran_ahead_goes_back_whole() {
    robot_goes_back_whole(3);
}

fn robot_goes_back_whole(threads: usize) {
    if sim_test::titan_rs("esc_board").is_none() {
        return;
    }
    // The smart battery's bridge too: alone on the external bus with nothing to acknowledge its frames, it would
    // retry each without end (titan_firmware_rs c59474f on), and every retry ends a run-ahead window
    let load = || RobotBench::load("talos.toml", &[("internal", 1), ("external", 21)], |_| {}).unwrap();
    let mut rb = load();
    rb.robot.threads = threads;
    let inputs = rb.robot.inputs.clone();
    let mut tt = TimeTravel::new(&mut rb.robot, inputs);
    tt.every = Time::from_ms(50);
    assert_eq!(tt.run_until(&mut rb.robot, Time::from_ms(400)), Ran::Reached);
    let stats = rb.robot.run_ahead_stats;
    if threads > 1 {
        assert!(stats.slices >= 3900 && stats.windows < stats.slices / 6, "{stats:?}");
    } else {
        assert_eq!(stats, Default::default());
    }
    let end = robot_state(&mut rb.robot);

    // Back to a time, as a fresh robot has it there
    assert_eq!(tt.rewind_to(&mut rb.robot, Time::from_us(237_450)), Travelled::Arrived);
    assert_eq!(rb.robot.now, Time::from_us(237_400));
    let mut f = load();
    f.run_until(Time::from_us(237_400));
    assert_eq!(robot_state(&mut rb.robot), robot_state(&mut f.robot));

    // To the frontier again
    assert_eq!(tt.run_until(&mut rb.robot, Time::from_ms(400)), Ran::Reached);
    assert_eq!(robot_state(&mut rb.robot), end);

    // Back to the last breakpoint hit on one board: where its core 1 runs now
    let esc = rb.robot.index("esc0").unwrap();
    let pc = rb.robot.boards[esc].machine.cores[1].pc();
    rb.robot.boards[esc].machine.breakpoints.insert(pc);
    let r = tt.reverse_continue(&mut rb.robot);
    let Travelled::Hit(hit) = r else { panic!("{r:?}") };
    let Pos::At { board, core, time } = hit.pos else { panic!() };
    assert_eq!(board, esc);
    assert!(time < Time::from_ms(400));
    rb.robot.boards[esc].machine.breakpoints.clear();
    let mut g = load();
    g.robot.boards[esc].machine.stop_at[core] = Some(time);
    let r = g.robot.run_until(Time::from_ms(400));
    assert!(matches!(r, Some((b, StopReason::Reached { core: c, .. })) if b == esc && c as usize == core), "{r:?}");
    assert_eq!(robot_state(&mut rb.robot), robot_state(&mut g.robot));

    // Steps back on that board's core, the robot with it: as a fresh robot has it at each
    let mut times = Vec::new();
    for _ in 0..3 {
        assert_eq!(tt.reverse_step(&mut rb.robot, esc, core), Travelled::Arrived);
        let Pos::At { board: b, core: c, time: t } = tt.here() else { panic!("{:?}", tt.here()) };
        assert_eq!((b, c), (esc, core));
        assert!(times.last().is_none_or(|&l| t < l) && t < time, "{t} after {times:?}");
        times.push(t);
    }
    let mut h = load();
    h.robot.boards[esc].machine.stop_at[core] = Some(*times.last().unwrap());
    let r = h.robot.run_until(Time::from_ms(400));
    assert!(matches!(r, Some((b, StopReason::Reached { .. })) if b == esc), "{r:?}");
    assert_eq!(robot_state(&mut rb.robot), robot_state(&mut h.robot));
}
