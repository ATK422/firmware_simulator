//! Boards of a robot without an acoustic medium can run on several threads within a slice. The result must be
//! exactly that of running them in turn: every CAN frame at the same time, every log line at the same time, the
//! same machine state.

mod common;

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::Robot;

/// Talos's boards (with one switched on later, as in the robot file) for 600 ms, on `threads` threads.
fn run(threads: usize) -> Option<(Vec<String>, Vec<String>, MockBridge)> {
    common::titan_elf("esc_board")?;
    let mut r = Robot::load(&common::repo().join("robots/talos.toml")).unwrap();
    assert!(r.medium.is_none());
    r.threads = threads;
    for rb in &mut r.buses {
        rb.bus.record = Some(Vec::new());
    }
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    r.bus("internal").unwrap().endpoints.push(Box::new(bridge.clone()));
    assert!(r.run_until(Time::from_ms(600)).is_none());
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
    let Some((frames1, state1, bridge1)) = run(1) else { return };
    let (frames4, state4, bridge4) = run(4).unwrap();
    assert!(frames1.len() > 20, "{} frames", frames1.len());
    assert_eq!(frames1, frames4);
    assert_eq!(state1, state4);
    let received = |b: &MockBridge| format!("{:?}", b.state().received);
    assert_eq!(received(&bridge1), received(&bridge4));
    assert!(!bridge1.state().received.is_empty());
}
