//! Time travel ([`sim_host::timetravel`]): going back to a time, to the last breakpoint or watchpoint hit, or one
//! step of a core, gives exactly the state a fresh run has there (both cores, memory, peripherals, the host side),
//! on a board and across a robot; what host endpoints sent is replayed; changes in the past drop the recorded
//! future.

mod common;

use rp2040::StopReason;
use rp2040::machine::{WatchKind, Watchpoint};
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::Session;
use sim_host::timetravel::{Pos, Ran, Rewindable, TimeTravel, Travelled};

fn program(name: &str) -> std::path::PathBuf {
    common::repo().join("test_programs").join(name)
}

/// Everything a debugger or a test could look at on a board.
fn state(s: &mut Session) -> Vec<String> {
    let next = s.machine.sys.sched.next_time();
    let m = &s.machine;
    let mut out = vec![format!("now {} resets {}", m.now, m.resets)];
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        out.push(format!(
            "core{c} enabled {} at {}: {} cycles, {} instructions, sleep {:?}, regs {regs:08x?}",
            m.core_enabled[c],
            m.core_time(c),
            core.cycles(),
            core.instructions(),
            core.sleep_state()
        ));
    }
    let hash = |b: &[u8]| b.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    out.push(format!("sram {:x} flash {:x}", hash(&m.sys.sram), hash(&m.sys.flash)));
    out.push(format!("next event {next}"));
    for u in &m.sys.uart {
        out.push(format!("uart pending {:?}", u.output));
    }
    out.push(format!("uart text {:?}", s.uart_text));
    out.push(format!("logs {:?}", s.logs.iter().map(|(t, l)| format!("{t} {l}")).collect::<Vec<_>>()));
    out
}

/// A fresh run of `make()` to position `pos`, in the same 100 µs slices (stops change nothing).
fn fresh_at(make: &dyn Fn() -> Session, pos: Pos) -> Session {
    let mut s = make();
    match pos {
        Pos::Boundary(t) => {
            assert_eq!(s.run_until(t), StopReason::TimeLimit);
        }
        Pos::At { core, time, .. } => {
            s.machine.stop_at[core] = Some(time);
            loop {
                match s.run_until(time + Time::from_secs(1)) {
                    StopReason::Reached { core: c, .. } if c as usize == core => break,
                    StopReason::Reached { .. } => {}
                    r => panic!("did not reach {pos:?}: {r:?}"),
                }
            }
        }
    }
    s
}

fn multicore() -> Session {
    let mut s = common::boot(&program("pico/hello_multicore.elf"), None);
    s.keep_logs = true;
    s
}

#[test]
fn rewinding_to_a_time_gives_a_fresh_run_there() {
    let mut s = multicore();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_ms(2);
    assert_eq!(tt.run_until(&mut s, Time::from_ms(30)), Ran::Reached);
    assert!(tt.stats().checkpoints >= 14, "{:?}", tt.stats());
    let end_state = state(&mut s);
    for t in [Time::from_us(12_300), Time::from_ms(1), Time::from_us(29_900), Time::from_us(4_150)] {
        assert_eq!(tt.rewind_to(&mut s, t), Travelled::Arrived, "{t}");
        // The last slice boundary at or before it (slices start where the machine got to, a little past the last
        // slice's end at times)
        let Pos::Boundary(at) = tt.here() else { panic!("{:?}", tt.here()) };
        assert!(at <= t && at + Time::from_us(100) > t, "{at} for {t}");
        assert_eq!(s.now(), at);
        assert_eq!(state(&mut s), state(&mut fresh_at(&multicore, Pos::Boundary(at))), "at {at}");
        assert!(!tt.at_frontier());
    }
    // Forward again through the recorded history to the frontier: as it was
    assert_eq!(tt.run_until(&mut s, Time::from_ms(30)), Ran::Reached);
    assert_eq!(state(&mut s), end_state);
    assert!(tt.at_frontier());
    // And on live
    assert_eq!(tt.run_until(&mut s, Time::from_ms(32)), Ran::Reached);
    assert_eq!(state(&mut s), state(&mut fresh_at(&multicore, Pos::Boundary(Time::from_ms(32)))));
}

#[test]
fn reverse_continue_goes_to_the_last_breakpoint_hit() {
    let mut s = multicore();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_ms(3);
    tt.run_until(&mut s, Time::from_ms(20));
    // Somewhere core 1 runs: where it is now
    let pc = s.machine.cores[1].pc();
    s.machine.breakpoints.insert(pc);
    // The hits of a fresh run, in order, up to 20 ms
    let mut hits = Vec::new();
    let mut f = multicore();
    f.machine.breakpoints.insert(pc);
    loop {
        match f.run_until(Time::from_ms(20)) {
            StopReason::Breakpoint { core, .. } => hits.push((core as usize, f.machine.core_time(core as usize))),
            StopReason::TimeLimit => break,
            r => panic!("{r:?}"),
        }
    }
    assert!(hits.len() >= 3, "{hits:?}");
    // Back through the last three, each giving the fresh run's state there
    for &(core, time) in hits.iter().rev().take(3) {
        let r = tt.reverse_continue(&mut s);
        let Travelled::Hit(h) = r else { panic!("{r:?}") };
        assert_eq!(h.pos, Pos::At { board: 0, core, time });
        assert!(matches!(h.reason, StopReason::Breakpoint { .. }), "{:?}", h.reason);
        let mut fresh = fresh_at(&multicore, h.pos);
        assert_eq!(state(&mut s), state(&mut fresh));
    }
    // Forward from there: the next hit, as a fresh run has it
    let next = hits[hits.len() - 2];
    let r = tt.run_until(&mut s, Time::from_ms(20));
    assert!(matches!(r, Ran::Stopped { reason: StopReason::Breakpoint { .. }, .. }), "{r:?}");
    assert_eq!(tt.here(), Pos::At { board: 0, core: next.0, time: next.1 });
    s.machine.breakpoints.clear();
    // With no breakpoints left, back to the start of the history
    assert_eq!(tt.reverse_continue(&mut s), Travelled::Beginning);
    assert_eq!(s.machine.now, Time::ZERO);
}

#[test]
fn reverse_steps_go_back_one_step_at_a_time_through_interrupts() {
    // The embassy firmware takes timer interrupts and sleeps in WFE: steps include exception entries
    let make = || common::boot(&program("embassy_blinky.elf"), None);
    let mut s = make();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_ms(5);
    tt.run_until(&mut s, Time::from_secs(1));
    // Forward steps of core 0 in a fresh run from 10 ms, until a while after an exception entry
    let mut f = make();
    // (Past embassy's wait for the crystal)
    f.run_until(Time::from_ms(300));
    let mut positions = std::collections::VecDeque::new();
    let mut entered_at = None;
    for n in 0..20_000 {
        let t = f.machine.core_time(0);
        positions.push_back((t, f.machine.cores[0].pc(), f.machine.cores[0].ipsr()));
        if positions.len() > 400 {
            positions.pop_front();
        }
        let before = f.machine.cores[0].ipsr();
        f.machine.stop_at[0] = Some(t + Time(1));
        let r = f.run_until(Time::from_secs(1));
        assert!(matches!(r, StopReason::Reached { core: 0, .. }), "{r:?}");
        if entered_at.is_none() && before == 0 && f.machine.cores[0].ipsr() != 0 {
            entered_at = Some(n);
        }
        if entered_at.is_some_and(|e| n > e + 100) {
            break;
        }
    }
    assert!(entered_at.is_some(), "an interrupt was taken in the steps");
    // (The first is where the core went to sleep, not a step's start: its clock waited there)
    let positions: Vec<_> = positions.into_iter().skip(1).collect();
    let (last, _, _) = *positions.last().unwrap();
    // Go there in the recorded history, then step back over all of them
    assert_eq!(tt.rewind_to(&mut s, Time::from_ms(300)), Travelled::Arrived);
    s.machine.stop_at[0] = Some(last);
    assert!(matches!(tt.run_until(&mut s, Time::from_secs(1)), Ran::Stopped { .. }));
    assert_eq!(tt.here(), Pos::At { board: 0, core: 0, time: last });
    let mut through_entry = false;
    for w in positions.windows(2).rev() {
        let ((t, pc, ipsr), (_, _, next_ipsr)) = (w[0], w[1]);
        assert_eq!(tt.reverse_step(&mut s, 0, 0), Travelled::Arrived);
        assert_eq!(tt.here(), Pos::At { board: 0, core: 0, time: t });
        assert_eq!((s.machine.cores[0].pc(), s.machine.cores[0].ipsr()), (pc, ipsr), "at {t}");
        through_entry |= ipsr == 0 && next_ipsr != 0;
    }
    assert!(through_entry, "stepped back over the exception entry");
    let (t, _, _) = positions[0];
    assert_eq!(state(&mut s), state(&mut fresh_at(&make, Pos::At { board: 0, core: 0, time: t })));
    // Before the first: the WFE that put the core to sleep
    assert_eq!(tt.reverse_step(&mut s, 0, 0), Travelled::Arrived);
    let Pos::At { time, .. } = tt.here() else { panic!() };
    assert!(time < Time::from_ms(300), "{time}");
    assert_eq!(state(&mut s), state(&mut fresh_at(&make, tt.here())));
}

#[test]
fn reverse_continue_to_a_watchpoint_stops_before_the_write() {
    let mut s = multicore();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_ms(2);
    // A word of core 0's stack
    let addr = s.machine.cores[0].reg(13) - 8;
    tt.run_until(&mut s, Time::from_ms(10));
    let now_value = s.machine.sys.peek(addr, sim_core::bus::Size::Word).unwrap();
    s.machine.watchpoints.push(Watchpoint { addr, len: 4, kind: WatchKind::Write });
    let r = tt.reverse_continue(&mut s);
    let Travelled::Hit(h) = r else { panic!("{r:?}") };
    assert!(matches!(h.reason, StopReason::Watchpoint { kind: WatchKind::Write, .. }), "{:?}", h.reason);
    // Before the write: one step forward of that core writes the word
    let Pos::At { core, time, .. } = h.pos else { panic!() };
    let fresh = fresh_at(&multicore, h.pos);
    assert_eq!(s.machine.cores[core].pc(), fresh.machine.cores[core].pc());
    s.machine.watchpoints.clear();
    s.machine.stop_at[core] = Some(time + Time(1));
    assert!(matches!(tt.run_until(&mut s, Time::from_ms(10)), Ran::Stopped { .. }));
    let after = s.machine.sys.peek(addr, sim_core::bus::Size::Word).unwrap();
    let _ = now_value;
    let mut f2 = fresh_at(&multicore, Pos::At { board: 0, core, time: s.machine.core_time(core) });
    assert_eq!(after, f2.machine.sys.peek(addr, sim_core::bus::Size::Word).unwrap());
    assert_eq!(state(&mut s), state(&mut f2));
}

#[test]
fn a_change_in_the_past_drops_the_recorded_future() {
    let mut s = multicore();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_ms(2);
    tt.run_until(&mut s, Time::from_ms(10));
    tt.rewind_to(&mut s, Time::from_ms(5));
    // A word of SRAM nothing uses, and a register the program does not look at again
    tt.apply(
        &mut s,
        0,
        std::sync::Arc::new(|m| {
            m.sys.poke(0x2003_0000, sim_core::bus::Size::Word, 0x1234_5678);
        }),
    );
    assert!(tt.at_frontier(), "the future is gone: the run is live from here");
    tt.run_until(&mut s, Time::from_ms(6));
    let changed = state(&mut s);
    // Going back before the change and forward again makes it again
    tt.rewind_to(&mut s, Time::from_ms(4));
    tt.run_until(&mut s, Time::from_ms(6));
    assert_eq!(state(&mut s), changed);
}

#[test]
fn checkpoints_stay_within_the_budget() {
    let mut s = multicore();
    let mut tt = TimeTravel::new(&mut s, vec![]);
    tt.every = Time::from_us(500);
    tt.budget = 8 << 20;
    tt.run_until(&mut s, Time::from_ms(40));
    let st = tt.stats();
    assert!(st.bytes <= 8 << 20, "{st:?}");
    assert!(st.taken >= 70, "{st:?}");
    // Spacing grows with age, the first is kept
    let times: Vec<Time> = tt.checkpoints().iter().map(|c| c.0).collect();
    assert_eq!(times[0], Time::ZERO);
    assert!(times.len() >= 4, "{times:?}");
    // Any time can still be gone back to
    assert_eq!(tt.rewind_to(&mut s, Time::from_us(1_700)), Travelled::Arrived);
    assert_eq!(state(&mut s), state(&mut fresh_at(&multicore, tt.here())));
    let _ = s.now();
}
