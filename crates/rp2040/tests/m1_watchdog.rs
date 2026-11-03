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
