//! The run loop executes instructions in batches when nothing needs to see each one (see
//! `CortexM0Plus::run_batch`). A batch must be indistinguishable from stepping: these tests run programs both ways
//! (a breakpoint set at an address that is never reached turns batching off) and compare everything observable:
//! GPIO and unmodelled-register traces with their times, UART output with its times, and the final CPU, clock and
//! memory state.

use rp2040::{BOOTROM_B1, Machine, MachineConfig, StopReason};
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::trace::{Category, VecSink};

struct Run {
    stop: StopReason,
    trace: Vec<String>,
    uart: Vec<(Time, u8)>,
    state: Vec<String>,
}

fn run(elf: &str, until: Time, batched: bool, rom_boot: bool) -> Run {
    let path = format!("{}/../../test_programs/{elf}", env!("CARGO_MANIFEST_DIR"));
    let image = Image::parse(&std::fs::read(&path).unwrap()).unwrap();
    let mut m = Machine::new(MachineConfig { bootrom: Some(BOOTROM_B1.to_vec()), ..Default::default() });
    let sink = VecSink::new(&[Category::Gpio, Category::Unmodelled, Category::Device]);
    m.set_trace(Box::new(sink.clone()));
    m.load(&image).unwrap();
    if rom_boot {
        m.rom_boot().unwrap();
    } else {
        m.direct_boot().unwrap();
    }
    if !batched {
        // Never reached (instructions are halfword aligned), but makes the loop check every instruction
        m.breakpoints.insert(0xFFFF_FFF1);
    }
    let stop = m.run_until(until);
    let mut uart = Vec::new();
    for u in &m.sys.uart {
        uart.extend(u.output.iter().copied());
    }
    let mut state = vec![format!("now {}", m.now)];
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        state.push(format!(
            "core{c} at {}: {} cycles, {} instructions, sleep {:?}, regs {regs:08x?}",
            m.core_time(c),
            core.cycles(),
            core.instructions(),
            core.sleep_state()
        ));
    }
    state
        .push(format!("sram {:x}", m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))));
    let trace = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
    Run { stop, trace, uart, state }
}

/// Runs `elf` batched and stepped, checks they agree and returns the batched run.
fn same(elf: &str, until: Time, rom_boot: bool) -> Run {
    let a = run(elf, until, true, rom_boot);
    let b = run(elf, until, false, rom_boot);
    assert_eq!(a.stop, b.stop, "{elf}");
    assert_eq!(a.state, b.state, "{elf}");
    assert_eq!(a.uart, b.uart, "{elf}");
    assert_eq!(a.trace.len(), b.trace.len(), "{elf}");
    for (x, y) in a.trace.iter().zip(&b.trace) {
        assert_eq!(x, y, "{elf}");
    }
    a
}

#[test]
fn embassy_firmware_batched_as_stepped() {
    // Timer alarms, interrupts, WFE sleeps and GPIO toggles
    let r = same("embassy_blinky.elf", Time::from_ms(600), false);
    assert!(r.trace.len() >= 4, "{:?}", r.trace);
}

#[test]
fn dual_core_batched_as_stepped() {
    // Core 1 launch over the SIO FIFO, both cores running, UART output
    let r = same("pico/hello_multicore.elf", Time::from_ms(30), false);
    assert!(r.uart.len() > 100);
}

#[test]
fn flash_programming_batched_as_stepped() {
    // The bootrom's flash routines against the QSPI chip: XIP faults while the SSI is in command mode, and the chip
    // is busy for its program and erase times
    let r = same("pico/flash_program.elf", Time::from_ms(300), false);
    assert!(r.uart.len() > 2000, "the program reports the flash contents before and after");
}

#[test]
fn rom_boot_batched_as_stepped() {
    // Both cores start in the bootrom, boot2 sets up XIP
    let r = same("pico/hello_uart.elf", Time::from_ms(30), true);
    assert!(!r.uart.is_empty());
}

#[test]
fn watchdog_resets_batched_as_stepped() {
    let r = same("watchdog.elf", Time::from_ms(35), false);
    assert_eq!(r.stop, StopReason::TimeLimit);
}
