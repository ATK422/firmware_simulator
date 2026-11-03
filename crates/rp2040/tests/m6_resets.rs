//! M6: the kinds of reset. `test_programs/watchdog.S` counts its boots in WATCHDOG.SCRATCH0 and lets the watchdog
//! bite every 10 ms. A watchdog reset keeps SRAM and the scratch registers; a RUN pin reset keeps SRAM only; a power
//! cycle keeps neither. VREG_AND_CHIP_RESET.CHIP_RESET says which chip-level reset happened last. The flash keeps
//! its contents through all of them, and across processes through a flash file.

use rp2040::{Machine, MachineConfig};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;

const SCRATCH0: u32 = 0x4005_800C;
const SCRATCH1: u32 = 0x4005_8010;
const CHIP_RESET: u32 = 0x4006_4008;
const HAD_POR: u32 = 1 << 8;
const HAD_RUN: u32 = 1 << 16;

fn machine() -> Machine {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/watchdog.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let mut m = Machine::new(MachineConfig::default());
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    m
}

fn rd(m: &mut Machine, addr: u32) -> u32 {
    m.debug_read(0, addr, Size::Word).unwrap()
}

const MARK: u32 = 0x2003_0000;

fn mark_sram(m: &mut Machine) {
    let o = (MARK - 0x2000_0000) as usize;
    m.sys.sram[o..o + 4].copy_from_slice(&0xC0FF_EE00u32.to_le_bytes());
}

fn sram_marked(m: &Machine) -> bool {
    let o = (MARK - 0x2000_0000) as usize;
    m.sys.sram[o..o + 4] == 0xC0FF_EE00u32.to_le_bytes()
}

#[test]
fn watchdog_reset_keeps_sram_and_scratch() {
    let mut m = machine();
    m.run_until(Time::from_ms(5));
    assert_eq!(rd(&mut m, SCRATCH0), 1);
    assert_eq!(rd(&mut m, CHIP_RESET), HAD_POR, "came up from power-on");
    mark_sram(&mut m);
    m.run_until(Time::from_ms(15));
    assert_eq!(m.resets, 1);
    assert_eq!(rd(&mut m, SCRATCH0), 2, "scratch survives");
    assert_eq!(rd(&mut m, SCRATCH1), 1, "REASON = TIMER");
    assert!(sram_marked(&m), "SRAM survives");
    assert_eq!(rd(&mut m, CHIP_RESET), HAD_POR, "a watchdog reset is not a chip reset");
}

#[test]
fn run_pin_reset_clears_scratch_but_keeps_sram() {
    let mut m = machine();
    m.run_until(Time::from_ms(15));
    assert_eq!(rd(&mut m, SCRATCH0), 2);
    mark_sram(&mut m);
    m.run_pin_reset();
    let t = m.now;
    m.run_until(t + Time::from_ms(1));
    assert_eq!(rd(&mut m, SCRATCH0), 1, "scratch cleared, one boot since");
    assert_eq!(rd(&mut m, SCRATCH1), 0, "REASON cleared");
    assert_eq!(rd(&mut m, CHIP_RESET), HAD_RUN);
    assert!(sram_marked(&m), "SRAM keeps its contents while powered");
}

#[test]
fn power_cycle_clears_everything_but_flash() {
    let mut m = machine();
    m.run_until(Time::from_ms(15));
    mark_sram(&mut m);
    m.sys.flash[0x10_0000] = 0x5A;
    m.run_pin_reset();
    m.power_cycle();
    let t = m.now;
    m.run_until(t + Time::from_ms(1));
    assert_eq!(rd(&mut m, SCRATCH0), 1);
    assert_eq!(rd(&mut m, SCRATCH1), 0);
    assert_eq!(rd(&mut m, CHIP_RESET), HAD_POR);
    assert!(!sram_marked(&m), "SRAM comes up with arbitrary contents");
    assert_eq!(m.sys.flash[0x10_0000], 0x5A, "flash is non-volatile");
    // And the program keeps running from flash: the watchdog bites again
    let r = m.resets;
    m.run_until(t + Time::from_ms(25));
    assert_eq!(m.resets, r + 2);
}

#[test]
fn flash_file_round_trip() {
    let dir = std::env::temp_dir().join(format!("fwsim-flash-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("flash.bin");
    let mut m = machine();
    assert!(!m.load_flash_file(&path).unwrap(), "no file yet");
    m.sys.flash[0x1F_F000..0x1F_F004].copy_from_slice(b"uwrt");
    m.save_flash_file(&path).unwrap();
    assert_eq!(std::fs::metadata(&path).unwrap().len(), m.sys.flash.len() as u64);

    let mut m2 = Machine::new(MachineConfig::default());
    assert!(m2.load_flash_file(&path).unwrap());
    assert_eq!(&m2.sys.flash[0x1F_F000..0x1F_F004], b"uwrt");
    assert_eq!(m2.sys.flash[..0x3C], m.sys.flash[..0x3C], "the program too");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn sysresetreq_resets_only_the_core() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/sysreset.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let mut m = Machine::new(MachineConfig::default());
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    m.run_until(Time::from_ms(1));
    assert_eq!(rd(&mut m, SCRATCH0), 10, "ten boots, counted in scratch");
    let o = (MARK - 0x2000_0000) as usize;
    assert_eq!(u32::from_le_bytes(m.sys.sram[o..o + 4].try_into().unwrap()), 10, "and in SRAM");
    assert_eq!(m.resets, 0, "no chip or watchdog reset");
    assert_eq!(rd(&mut m, 0x4005_8008), 0, "watchdog REASON untouched");
    assert_eq!(rd(&mut m, CHIP_RESET), HAD_POR);
}

/// A snapshot holds how the machine boots (directly or through the bootrom), the USB boot request and the reset
/// count: restored after the machine was switched to booting through the bootrom, the next watchdog reset boots
/// the program directly again, as it does in a copy that was never switched.
#[test]
fn snapshot_restores_the_boot_settings() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/watchdog.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let mut m = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    m.run_until(Time::from_ms(15));
    assert_eq!((m.resets, rd(&mut m, SCRATCH0)), (1, 2));
    let snap = m.snapshot();
    let mut original = m.fork();
    // Rebooted through the bootrom (which finds no boot2 and goes into USB boot), with a USB boot request
    m.usb_boot_requested = true;
    m.rom_boot().unwrap();
    m.run_until(Time::from_ms(18));
    m.restore(&snap);
    assert!(!m.boots_from_rom() && !m.usb_boot_requested);
    assert_eq!(m.resets, 1);
    original.run_until(Time::from_ms(25));
    m.run_until(Time::from_ms(25));
    assert_eq!((original.resets, rd(&mut original, SCRATCH0)), (2, 3), "the watchdog bit again at 20 ms");
    assert_eq!((m.resets, rd(&mut m, SCRATCH0)), (2, 3), "and the restored machine booted the program again");
    assert_eq!(m.cores[0].pc(), original.cores[0].pc());
    // And the other way: a snapshot of a machine that boots through the bootrom restores that
    let mut rom = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    rom.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    rom.rom_boot().unwrap();
    let snap = rom.snapshot();
    rom.load(&image).unwrap();
    rom.direct_boot().unwrap();
    rom.restore(&snap);
    assert!(rom.boots_from_rom());
    rom.power_cycle();
    assert!(rom.cores[0].pc() < 0x4000, "the power-on reset starts in the bootrom");
}
