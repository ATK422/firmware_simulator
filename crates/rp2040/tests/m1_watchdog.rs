//! M1: `test_programs/watchdog.S` arms the watchdog and sleeps; each timeout resets the chip, the boot count in
//! SCRATCH0 survives, REASON reads TIMER, and the sleeping core costs nothing (idle fast-forward).

use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
use sim_core::image::Image;
use sim_core::time::Time;

#[test]
fn watchdog_timeout_reboots_and_keeps_scratch() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/watchdog.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let mut m = Machine::new(MachineConfig::default());
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    assert_eq!(m.run_until(Time::from_ms(35)), StopReason::TimeLimit);
    assert_eq!(m.resets, 3, "timeouts at about 10, 20 and 30 ms");
    let wd = |m: &mut Machine, off: u32| m.debug_read(0, 0x4005_8000 + off, Size::Word).unwrap();
    assert_eq!(wd(&mut m, 0x0C), 4, "boot count survives watchdog resets");
    assert_eq!(wd(&mut m, 0x10), 1, "REASON = TIMER as seen by the last boot");
    // A core asleep in WFI does not execute: only a handful of instructions per boot
    assert!(sim_core::cpu::Cpu::cycles(&m.cores[0]) < 1_000_000);
}

/// Branch edge coverage leaves the run as it is (still batched), and what ran before a watchdog reset stays in the
/// map (`blink.S`: its delay loop's taken `BNE`, then the loop back to the toggle).
#[test]
fn edge_coverage_changes_nothing_and_survives_resets() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/blink.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let run = |edges: bool| {
        let mut m = Machine::new(MachineConfig::default());
        m.load(&image).unwrap();
        m.direct_boot().unwrap();
        if edges {
            m.enable_edges(&[]);
        }
        assert_eq!(m.run_until(Time::from_ms(5)), StopReason::TimeLimit);
        let before = m.edges().map(|e| e.count());
        m.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
        let kept = m.edges().map(|e| e.count());
        assert_eq!(m.run_until(Time::from_ms(10)), StopReason::TimeLimit);
        let core = &m.cores[0];
        let state = (m.resets, sim_core::cpu::Cpu::cycles(core), sim_core::cpu::Cpu::pc(core));
        (state, before, kept, m.edges())
    };
    let (plain, _, _, none) = run(false);
    let (with, before, kept, edges) = run(true);
    assert_eq!(plain, with, "the same run with edge coverage on");
    assert!(none.is_none());
    let before = before.unwrap();
    assert!(before >= 2, "the delay loop and the outer loop: {before}");
    assert_eq!(kept, Some(before), "the reset kept the map");
    assert_eq!(edges.unwrap().count(), before, "the same branches again after the reset");
}
