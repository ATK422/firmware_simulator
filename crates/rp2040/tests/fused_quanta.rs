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
        assert!(fused > 100, "{fused} quanta fused");
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

/// Core 0 computes while polling peripherals: each time round its inner loop it writes SRAM, reads UART0's flags and
/// the timer twice; each time round the outer loop it writes to core 1's SIO FIFO, which wakes core 1 from WFI (its
/// handler drains the FIFO and toggles GPIO 24: when core 1 wakes depends on where core 0's quantum ends), and feeds
/// the watchdog (moving its timeout, at times the next event). Every other time it also arms a timer alarm a tick
/// ahead (its handler toggles GPIO 25), then either busy-waits on the timer until three ticks past it, or moves the alarm two
/// ticks later (the next event moves later) and polls on. A fused batch runs on through the polling and the
/// feeding, and must end where an access moves the next event or wakes core 1, and at the busy-wait, for it to be
/// found. Assembled from:
///
/// ```text
/// .org 0x000; .word 0x20002000, 0x20000101   .org 0x040; .word 0x20000181 (TIMER_IRQ_0)
/// .org 0x100                                 .org 0x180
///     ldr r3, =0x20000400 (scratch)          timer_irq:
///     ldr r4, =0x40034000 (UART0)                ldr r1, =0x40054000
///     ldr r5, =0x40058000 (WATCHDOG)             movs r0, #1
///     ldr r6, =0x40054000 (TIMER)                str r0, [r1, #0x34] (INTR)
///     ldr r7, =0xD0000000 (SIO)                  ldr r1, =0xD000001C (GPIO_OUT_XOR)
/// outer: movs r1, #40                            ldr r0, =1 << 25
/// inner: str r1, [r3]                            str r0, [r1]
///     ldr r0, [r4, #0x18] (UARTFR)               bx lr
///     ldr r0, [r6, #0x28] (TIMERAWL)         .org 0x200 (core 1)
///     ldr r0, [r6, #0x28]                    reset1: wfi
///     subs r1, #1                                b reset1
///     bne inner                              .org 0x220
///     str r1, [r7, #0x54] (FIFO_WR)          sio_irq1:
///     ldr r2, =16                                ldr r1, =0xD0000000
///     str r2, [r5, #4] (LOAD: feed)          pop: ldr r0, [r1, #0x58] (FIFO_RD)
///     ldr r2, [r3, #4]                           ldr r0, [r1, #0x50] (FIFO_ST)
///     adds r2, #1                                lsls r0, r0, #31 (VLD)
///     str r2, [r3, #4]                           bne pop
///     lsls r0, r2, #31                           ldr r0, =1 << 24
///     bne outer                                  str r0, [r1, #0x1C] (GPIO_OUT_XOR)
///     ldr r0, [r6, #0x28]                        bx lr
///     adds r0, #1                            .org 0x800 (core 1's vectors)
///     str r0, [r6, #0x10] (ALARM0)           .word 0x20003000, 0x20000201
///     lsls r1, r2, #30                       .org 0x880; .word 0x20000221 (SIO_IRQ_PROC1)
///     bpl wait_setup
///     adds r0, #2
///     str r0, [r6, #0x10]
///     b outer
/// wait_setup: adds r0, #3
/// wait: ldr r2, [r6, #0x28]
///     cmp r2, r0
///     bcc wait
///     b outer
/// ```
#[test]
fn fused_batches_run_through_polling() {
    use rp2040::StopReason;
    use rp2040::memmap::{IO_BANK0_BASE, RESETS_BASE, SIO_BASE, SRAM_BASE, TIMER_BASE, WATCHDOG_BASE};
    use sim_core::bus::Size;
    use sim_core::trace::NullSink;

    const PROGRAM: &[(u32, u32)] = &[
        (0x000, 0x20002000),
        (0x004, 0x20000101),
        (0x040, 0x20000181),
        (0x100, 0x4C114B10),
        (0x104, 0x4E124D11),
        (0x108, 0x21284F12),
        (0x10C, 0x69A06019),
        (0x110, 0x6AB06AB0),
        (0x114, 0xD1F93901),
        (0x118, 0x4A0F6579),
        (0x11C, 0x685A606A),
        (0x120, 0x605A3201),
        (0x124, 0xD1F007D0),
        (0x128, 0x30016AB0),
        (0x12C, 0x07916130),
        (0x130, 0x3002D502),
        (0x134, 0xE7E86130),
        (0x138, 0x6AB23003),
        (0x13C, 0xD3FC4282),
        (0x140, 0x0000E7E3),
        (0x144, 0x20000400),
        (0x148, 0x40034000),
        (0x14C, 0x40058000),
        (0x150, 0x40054000),
        (0x154, 0xD0000000),
        (0x158, 0x00000010),
        (0x180, 0x20014903),
        (0x184, 0x49036348),
        (0x188, 0x60084803),
        (0x18C, 0x00004770),
        (0x190, 0x40054000),
        (0x194, 0xD000001C),
        (0x198, 0x02000000),
        (0x200, 0xE7FDBF30),
        (0x220, 0x6D884903),
        (0x224, 0x07C06D08),
        (0x228, 0x4802D1FB),
        (0x22C, 0x477061C8),
        (0x230, 0xD0000000),
        (0x234, 0x01000000),
        (0x800, 0x20003000),
        (0x804, 0x20000201),
        (0x880, 0x20000221),
    ];
    let run = |fuse: bool, slice: Time| {
        let mut m = Machine::new(MachineConfig::default());
        m.fuse_quanta = fuse;
        let sink = VecSink::new(&[Category::Gpio]);
        m.set_trace(Box::new(sink.clone()));
        for &(off, v) in PROGRAM {
            assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
        }
        m.direct_boot().unwrap();
        let mut trace = NullSink;
        let mut bus = rp2040::system::CoreBus { sys: &mut m.sys, trace: &mut trace, core: 1, now: Time::ZERO };
        m.cores[1].reset(&mut bus, SRAM_BASE + 0x800);
        m.core_enabled[1] = true;
        let mut w = |addr: u32, v: u32| m.sys.write(&mut NullSink, 0, Time::ZERO, addr, Size::Word, v).unwrap();
        w(WATCHDOG_BASE + 0x2C, 1 << 9 | 480); // a timer tick every 480 reference clock cycles (40 µs)
        // IO_BANK0, PADS_BANK0, TIMER and UART0 out of reset
        w(RESETS_BASE + 0x3000, 1 << 5 | 1 << 8 | 1 << 21 | 1 << 22);
        for pin in [24, 25] {
            w(IO_BANK0_BASE + pin * 8 + 4, 5); // SIO
        }
        w(SIO_BASE + 0x24, 3 << 24); // GPIO_OE_SET
        w(TIMER_BASE + 0x38, 1); // INTE: alarm 0
        w(WATCHDOG_BASE + 0x04, 16);
        w(WATCHDOG_BASE, 1 << 30); // CTRL: ENABLE (eight ticks, fed in time: it never fires)
        m.cores[0].nvic.enabled = 1;
        m.cores[1].nvic.enabled = 1 << 16;
        let end = Time::from_ms(12);
        let mut t = Time::ZERO;
        while t < end {
            t = (t + slice).min(end);
            assert_eq!(m.run_until(t), StopReason::TimeLimit);
        }
        let mut out = vec![format!("now {} resets {}", m.now, m.resets)];
        for (c, core) in m.cores.iter().enumerate() {
            let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
            out.push(format!(
                "core{c} at {}: {} cycles, {} instructions, regs {regs:08x?}",
                m.core_time(c),
                core.cycles(),
                core.instructions()
            ));
        }
        out.push(format!(
            "sram {:x}",
            m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
        ));
        let trace: Vec<String> = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
        (out, trace, m.quanta_fused, m.spin_skipped)
    };
    // (Slices of a millisecond leave the watchdog's timeout the next event, before the end of the run, at times)
    for slice in [Time::from_us(100), Time::from_us(37), Time::from_ms(1)] {
        let (out, trace, fused, skipped) = run(true, slice);
        let (out_ref, trace_ref, _, _) = run(false, slice);
        assert_eq!(out, out_ref);
        assert_eq!(trace, trace_ref);
        // Both handlers ran, many times
        for pin in [24, 25] {
            let n = trace.iter().filter(|l| l.contains(&format!("GPIO{pin} "))).count();
            assert!(n > 20, "GPIO {pin} toggled {n} times: {trace:?}");
        }
        assert!(out[0].ends_with("resets 0"), "{out:?}");
        // Fused through the polling, and the busy-waits were still found
        assert!(fused > 500, "{fused} quanta fused (slice {slice})");
        assert!(skipped > 0, "{skipped} skipped (slice {slice})");
    }
}
