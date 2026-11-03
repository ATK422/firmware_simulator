//! `--hle-flash`: the trapped bootrom flash functions leave the SSI, the QSPI pins and the flash chip as the real
//! ones do, so pico-sdk code that talks to the chip itself afterwards (`flash_do_cmd`, behind `pico_unique_id`)
//! and boot2 keep working.

mod common;

use rp2040::Machine;
use rp2040::hle::RomFn;
use rp2040::periph::ssi::CsOverride;
use sim_core::Size;
use sim_core::cpu::Cpu;
use sim_core::time::Time;

fn pico(name: &str) -> std::path::PathBuf {
    common::repo().join("test_programs/pico").join(format!("{name}.elf"))
}

/// What the flash functions change: SSI registers (CTRLR0, CTRLR1, SSIENR, SER, BAUDR, SPI_CTRLR0), the QSPI pin
/// controls and chip select override, the chip's mode, and whether memory-mapped reads work.
#[derive(Debug, PartialEq)]
struct FlashState {
    ssi: Vec<u32>,
    pins: Vec<u32>,
    cs: CsOverride,
    continuous: bool,
    status: [u8; 3],
    xip: Result<(), &'static str>,
}

fn state(m: &Machine) -> FlashState {
    let s = &m.sys;
    FlashState {
        ssi: [0x00, 0x04, 0x08, 0x10, 0x14, 0xF4].iter().map(|&o| s.ssi.peek(o).unwrap()).collect(),
        pins: (0..6).map(|i| s.io_qspi.read(4 + 8 * i).unwrap()).collect(),
        cs: s.io_qspi.cs_override(),
        continuous: s.ssi.chip.continuous,
        status: [s.ssi.chip.sr1, s.ssi.chip.sr2, s.ssi.chip.sr3],
        xip: s.ssi.xip_config(),
    }
}

/// `flash_program` booted and idle, with core 0 parked in a loop in SRAM with interrupts masked (so a function
/// that leaves memory-mapped flash off can return without the core faulting).
fn parked() -> Machine {
    let mut s = common::boot(&pico("flash_program"), None);
    s.run_until(Time::from_ms(400));
    let mut m = s.machine;
    const PARK: u32 = 0x2003_0000;
    // cpsid i; b .
    assert!(m.sys.poke(PARK, Size::Half, 0xB672) && m.sys.poke(PARK + 2, Size::Half, 0xE7FE));
    m.cores[0].set_pc(PARK);
    m.cores[0].wake();
    m.run_for(Time::from_us(10));
    assert_eq!(m.cores[0].pc(), PARK + 2);
    // A page of data to program from
    for i in 0..256 {
        m.sys.poke(0x2003_0100 + i, Size::Byte, (i * 7) & 0xFF);
    }
    m
}

/// Calls ROM function `f` on core 0 of `m` and lets it return.
fn call(m: &mut Machine, f: RomFn) {
    let addr = m.hle.iter().find(|(_, g)| **g == f).map(|(a, _)| *a).unwrap();
    let args: &[u32] = match f {
        // 4 KiB at 256 KiB, with pico-sdk's block size and command
        RomFn::FlashRangeErase => &[0x40000, 4096, 1 << 16, 0xD8],
        RomFn::FlashRangeProgram => &[0x40000, 0x2003_0100, 256],
        _ => &[],
    };
    let back = m.cores[0].pc();
    m.inject_call(0, addr | 1, args);
    m.run_for(Time::from_ms(60));
    assert_eq!(m.cores[0].pc(), back, "{f:?} returned");
}

/// Forces the flash chip select high through IO_QSPI, as `flash_do_cmd` leaves it.
fn force_cs_high(m: &mut Machine) {
    let ctrl = m.sys.io_qspi.read(0x0C).unwrap();
    let t = m.now;
    m.sys.write(&mut sim_core::trace::NullSink, 0, t, 0x4001_800C, Size::Word, ctrl | 3 << 8).unwrap();
    assert_eq!(m.sys.io_qspi.cs_override(), CsOverride::High);
}

#[test]
fn trapped_flash_functions_leave_the_flash_as_the_rom_does() {
    let base = parked();
    let sequences: &[&[RomFn]] = &[
        &[RomFn::ConnectInternalFlash],
        &[RomFn::FlashExitXip],
        &[RomFn::FlashFlushCache],
        &[RomFn::FlashEnterCmdXip],
        // What pico-sdk's flash_range_erase and flash_range_program do, boot2 aside
        &[RomFn::ConnectInternalFlash, RomFn::FlashExitXip, RomFn::FlashRangeErase, RomFn::FlashFlushCache],
        &[RomFn::ConnectInternalFlash, RomFn::FlashExitXip, RomFn::FlashRangeErase],
        &[RomFn::ConnectInternalFlash, RomFn::FlashExitXip, RomFn::FlashRangeErase, RomFn::FlashRangeProgram],
        &[RomFn::ConnectInternalFlash, RomFn::FlashExitXip, RomFn::FlashEnterCmdXip],
    ];
    for seq in sequences {
        // From XIP as boot2 left it, and with the chip select forced high
        for forced in [false, true] {
            let run = |hle: bool| {
                let mut m = base.fork();
                m.hle_enabled = hle;
                if forced {
                    force_cs_high(&mut m);
                }
                for f in seq.iter() {
                    call(&mut m, *f);
                }
                m
            };
            let (real, hle) = (run(false), run(true));
            assert_eq!(state(&hle), state(&real), "{seq:?}, chip select forced high first: {forced}");
            assert!(hle.sys.flash == real.sys.flash, "{seq:?}: the flash contents differ");
        }
    }
    // The ones that write did write
    let mut m = base.fork();
    m.hle_enabled = true;
    for f in [RomFn::ConnectInternalFlash, RomFn::FlashExitXip, RomFn::FlashRangeErase, RomFn::FlashRangeProgram] {
        call(&mut m, f);
    }
    let page: Vec<u8> = (0..256).map(|i| ((i * 7) & 0xFF) as u8).collect();
    assert_eq!(&m.sys.flash[0x40000..0x40100], &page[..]);
    assert!(m.sys.flash[0x40100..0x41000].iter().all(|&b| b == 0xFF));
}

fn run(name: &str, hle: bool, rom_boot: bool, t: Time) -> String {
    let image = sim_core::image::Image::load(&pico(name)).unwrap();
    let mut m =
        Machine::new(rp2040::MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    m.hle_enabled = hle;
    m.load(&image).unwrap();
    if rom_boot {
        m.rom_boot().unwrap();
    } else {
        m.direct_boot().unwrap();
    }
    let mut s = sim_host::Session::new(m, &image);
    s.run_until(t);
    s.poll_uarts();
    s.uart_text[0].clone()
}

#[test]
fn the_unique_id_reads_under_hle() {
    // pico_unique_id reads the ID before main through flash_do_cmd, which ends with the chip select forced high
    // and counts on flash_flush_cache to release it before boot2 runs again; with the trap leaving it forced,
    // boot2 polled a deselected chip forever. Through the bootrom (--rom-boot) the trapped functions are the
    // bootrom's own as well, and boot2 never started.
    let real = run("unique_id", false, false, Time::from_ms(50));
    let id = format!("unique id {}", hex(&rp2040::periph::qspi_flash::UNIQUE_ID));
    assert!(real.contains(&id) && real.contains("still reading flash"), "{real}");
    for rom_boot in [false, true] {
        let out = run("unique_id", true, rom_boot, Time::from_ms(50));
        assert!(out.contains(&id), "rom boot {rom_boot}: {out:?}");
        assert!(out.contains(&format!("again {}", hex(&rp2040::periph::qspi_flash::UNIQUE_ID))), "{out}");
        assert!(out.contains("still reading flash"), "{out}");
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|b| format!("{b:02X}")).collect()
}

#[test]
fn flash_program_under_hle() {
    // Erase and program trapped, with the same result as running the ROM's routines on the chip model
    for rom_boot in [false, true] {
        let out = run("flash_program", true, rom_boot, Time::from_ms(400));
        assert!(out.contains("Programming successful!"), "rom boot {rom_boot}: {out}");
    }
}
