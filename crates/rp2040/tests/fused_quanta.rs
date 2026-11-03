//! Quantum fusing: while only one core can run and it touches nothing but memory and the timer, one batch runs
//! through several quanta (see `Machine::run_until`). It must change nothing: these programs run with fusing on and
//! off (`Machine::fuse_quanta`), in slices as sessions run machines, and everything observable is compared: GPIO,
//! unmodelled-register and device traces with their times, UART output with its times, the stop reasons and the
//! final CPU, clock and memory state.

use rp2040::{BOOTROM_B1, Machine, MachineConfig};
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::trace::{Category, VecSink};

struct Run {
    out: Vec<String>,
    trace: Vec<String>,
    fused: u64,
}

fn run(elf: &str, until: Time, slice: Time, fuse: bool, rom_boot: bool) -> Run {
    let path = format!("{}/../../test_programs/{elf}", env!("CARGO_MANIFEST_DIR"));
    let image = Image::parse(&std::fs::read(&path).unwrap()).unwrap();
    let mut m = Machine::new(MachineConfig { bootrom: Some(BOOTROM_B1.to_vec()), ..Default::default() });
    m.fuse_quanta = fuse;
    let sink = VecSink::new(&[Category::Gpio, Category::Unmodelled, Category::Device]);
    m.set_trace(Box::new(sink.clone()));
    m.load(&image).unwrap();
    if rom_boot {
        m.rom_boot().unwrap();
    } else {
        m.direct_boot().unwrap();
    }
    let mut out = Vec::new();
    let mut t = Time::ZERO;
    while t < until {
        t = (t + slice).min(until);
        out.push(format!("{:?} now {}", m.run_until(t), m.now));
    }
    for u in &m.sys.uart {
        out.push(format!("uart {:?}", u.output));
    }
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        out.push(format!(
            "core{c} at {}: {} cycles, {} instructions, sleep {:?}, regs {regs:08x?}",
            m.core_time(c),
            core.cycles(),
            core.instructions(),
            core.sleep_state()
        ));
    }
    out.push(format!("resets {} skipped {}", m.resets, m.spin_skipped > 0));
    out.push(format!("sram {:x}", m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))));
    let trace = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
    Run { out, trace, fused: m.quanta_fused }
}

/// Runs `elf` with and without fusing, checks they agree and returns the number of quantum boundaries fused.
fn same(elf: &str, until: Time, slice: Time, rom_boot: bool) -> u64 {
    let a = run(elf, until, slice, true, rom_boot);
    let b = run(elf, until, slice, false, rom_boot);
    assert_eq!(b.fused, 0);
    assert_eq!(a.out, b.out, "{elf}");
    assert_eq!(a.trace.len(), b.trace.len(), "{elf}");
    for (x, y) in a.trace.iter().zip(&b.trace) {
        assert_eq!(x, y, "{elf}");
    }
    a.fused
}

#[test]
fn embassy_firmware_fused_as_unfused() {
    // Timer alarms, interrupts, WFE sleeps and GPIO toggles; slices both on and off quantum boundaries
    for slice in [Time::from_us(100), Time::from_us(37)] {
        let fused = same("embassy_blinky.elf", Time::from_ms(600), slice, false);
        assert!(fused > 1000, "{fused} quanta fused");
    }
}

#[test]
fn dual_core_fused_as_unfused() {
    // Core 1 asleep in the bootrom until core 0 launches it over the SIO FIFO (an access that ends a fused batch),
    // then both cores running, UART output
    let fused = same("pico/hello_multicore.elf", Time::from_ms(30), Time::from_us(100), false);
    assert!(fused > 0, "{fused} quanta fused");
}

#[test]
fn flash_programming_fused_as_unfused() {
    // XIP faults while the SSI is in command mode (a slow fetch path), the flash chip busy meanwhile
    same("pico/flash_program.elf", Time::from_ms(300), Time::from_us(100), false);
}

#[test]
fn rom_boot_fused_as_unfused() {
    // Both cores start in the bootrom, boot2 sets up XIP
    same("pico/hello_uart.elf", Time::from_ms(30), Time::from_us(100), true);
}

#[test]
fn watchdog_resets_fused_as_unfused() {
    same("watchdog.elf", Time::from_ms(35), Time::from_us(100), false);
}

/// A core computes (writing SRAM, which a fused batch may run through) and now and then arms a timer alarm a few
/// ticks ahead: the write schedules an event earlier than the end of the fused batch would be, so the batch must
/// end there and the interrupt (which toggles a GPIO) come at the same time as without fusing.
#[test]
fn event_scheduled_inside_a_fused_batch() {
    use rp2040::StopReason;
    use rp2040::memmap::{IO_BANK0_BASE, RESETS_BASE, SIO_BASE, SRAM_BASE, TIMER_BASE, WATCHDOG_BASE};
    use sim_core::bus::Size;
    use sim_core::trace::NullSink;

    let run = |fuse: bool| {
        let mut m = Machine::new(MachineConfig::default());
        m.fuse_quanta = fuse;
        let sink = VecSink::new(&[Category::Gpio]);
        m.set_trace(Box::new(sink.clone()));
        let words = [
            (0x00, 0x2000_2000),
            (0x04, SRAM_BASE + 0x101),
            (0x40, SRAM_BASE + 0x141), // TIMER_IRQ_0
            (0x160, TIMER_BASE),
            (0x164, SIO_BASE + 0x1C), // GPIO_OUT_XOR
            (0x168, 1 << 25),
            (0x16C, SRAM_BASE + 0x400),
        ];
        for (off, v) in words {
            assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
        }
        let code: &[(u32, u16)] = &[
            (0x100, 0x4E17), // ldr r6, =TIMER_BASE
            (0x102, 0x4D18), // ldr r5, =GPIO_OUT_XOR
            (0x104, 0x4F18), // ldr r7, =1 << 25
            (0x106, 0x4B19), // ldr r3, =scratch
            (0x108, 0x21C8), // loop: movs r1, #200
            (0x10A, 0x6019), // inner: str r1, [r3]
            (0x10C, 0x3901), // subs r1, #1
            (0x10E, 0xD1FC), // bne inner
            (0x110, 0x6AB4), // ldr r4, [r6, #0x28] (TIMERAWL)
            (0x112, 0x3402), // adds r4, #2
            (0x114, 0x6134), // str r4, [r6, #0x10] (ALARM0: arm two ticks ahead)
            (0x116, 0xE7F7), // b loop
            (0x140, 0x2001), // alarm: movs r0, #1
            (0x142, 0x4907), // ldr r1, =TIMER_BASE
            (0x144, 0x6348), // str r0, [r1, #0x34] (INTR: clear)
            (0x146, 0x4907), // ldr r1, =GPIO_OUT_XOR
            (0x148, 0x4807), // ldr r0, =1 << 25
            (0x14A, 0x6008), // str r0, [r1]: toggle GPIO 25
            (0x14C, 0x4770), // bx lr
        ];
        for &(off, hw) in code {
            assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
        }
        m.direct_boot().unwrap();
        let mut w = |addr: u32, v: u32| m.sys.write(&mut NullSink, 0, Time::ZERO, addr, Size::Word, v).unwrap();
        w(WATCHDOG_BASE + 0x2C, 1 << 9 | 12); // a timer tick every 12 reference clock cycles
        w(RESETS_BASE + 0x3000, 1 << 5 | 1 << 8 | 1 << 21); // IO_BANK0, PADS_BANK0 and TIMER out of reset
        w(IO_BANK0_BASE + 25 * 8 + 4, 5); // GPIO 25: SIO
        w(SIO_BASE + 0x24, 1 << 25); // GPIO_OE_SET
        w(TIMER_BASE + 0x38, 1); // INTE: alarm 0
        m.cores[0].nvic.enabled = 1;
        for i in 1..=50 {
            assert_eq!(m.run_until(Time::from_us(100 * i)), StopReason::TimeLimit);
        }
        let c = &m.cores[0];
        let regs: Vec<u32> = (0..c.num_regs()).map(|r| c.read_reg(r)).collect();
        let state = format!("{} {} {} {} {regs:08x?}", m.now, m.core_time(0), c.cycles(), c.instructions());
        let trace: Vec<String> = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
        (trace, state, m.quanta_fused)
    };
    let (trace, state, fused) = run(true);
    let (trace_ref, state_ref, _) = run(false);
    assert!(fused > 1000, "{fused} quanta fused");
    assert!(trace.len() > 20, "{} GPIO toggles", trace.len());
    assert_eq!(state, state_ref);
    assert_eq!(trace, trace_ref);
}
