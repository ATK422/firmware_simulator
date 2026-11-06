//! SysTick with CLKSOURCE clear counts the reference clock, which on the RP2040 is the watchdog's tick (1 µs once
//! `WATCHDOG.TICK` is set up, as pico-sdk does): it decrements once per tick of virtual time, wraps every RVR + 1
//! ticks and pends its exception, waking a core asleep in WFI.

use rp2040::memmap::{SRAM_BASE, WATCHDOG_BASE};
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_core::trace::NullSink;

const COUNTER: u32 = SRAM_BASE + 0x200;
const SYST_CSR: u32 = 0xE000_E010;
const SYST_RVR: u32 = 0xE000_E014;
const SYST_CVR: u32 = 0xE000_E018;

/// A machine running, from SRAM, `wfi` in a loop with a SysTick handler that counts its calls at `COUNTER`, and
/// the watchdog tick dividing `clk_ref` by `cycles`.
fn machine(cycles: u32) -> Machine {
    let mut m = Machine::new(MachineConfig::default());
    let words = [(0x00, 0x2000_2000), (0x04, SRAM_BASE + 0x101), (0x3C, SRAM_BASE + 0x111)];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    let code: [(u32, u16); 9] = [
        (0x100, 0xBF30), // wfi
        (0x102, 0xE7FD), // b 0x100
        (0x110, 0x4802), // ldr r0, [pc, #8] (COUNTER)
        (0x112, 0x6801), // ldr r1, [r0]
        (0x114, 0x3101), // adds r1, #1
        (0x116, 0x6001), // str r1, [r0]
        (0x118, 0x4770), // bx lr
        (0x11A, 0xBF00), // nop
        (0x11C, COUNTER as u16),
    ];
    for (off, hw) in code {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    assert!(m.sys.poke(SRAM_BASE + 0x11E, Size::Half, COUNTER >> 16));
    m.direct_boot().unwrap();
    m.sys.write(&mut NullSink, 0, Time::ZERO, WATCHDOG_BASE + 0x2C, Size::Word, 1 << 9 | cycles).unwrap();
    m
}

/// Sets a SysTick register of core 0 (as a debugger would: they are plain fields).
fn ppb_write(m: &mut Machine, addr: u32, v: u32) {
    let st = &mut m.cores[0].systick;
    match addr {
        SYST_CSR => st.csr = v,
        SYST_RVR => st.rvr = v,
        SYST_CVR => st.cvr = v,
        _ => unreachable!(),
    }
}

#[test]
fn systick_on_the_reference_clock_counts_watchdog_ticks() {
    let clk_ref = {
        let m = Machine::new(MachineConfig::default());
        m.sys.freqs.clk_ref
    };
    // Whole cycles per tick: a microsecond tick when clk_ref is a whole number of MHz
    let cycles = (clk_ref / 1_000_000).max(1) as u32;
    let mut m = machine(cycles);
    ppb_write(&mut m, SYST_RVR, 999);
    ppb_write(&mut m, SYST_CVR, 0);
    ppb_write(&mut m, SYST_CSR, 0b011); // ENABLE, TICKINT, CLKSOURCE = reference
    let end = Time::from_us(10_500);
    assert_eq!(m.run_until(end), StopReason::TimeLimit);
    let ticks = m.sys.watchdog.tick.ticks(end);
    assert!(ticks > 5_000, "{ticks} ticks");
    // A wrap every 1000 ticks, each one handled
    let calls = m.sys.peek(COUNTER, Size::Word).unwrap() as u64;
    assert_eq!(calls, ticks / 1000, "{ticks} ticks");
    // The counter as SysTick fed tick by tick would have it
    let mut model = cpu_armv6m::SysTick { csr: 0b011, rvr: 999, cvr: 0 };
    model.tick_reference(ticks);
    assert_eq!(m.debug_read(0, SYST_CVR, Size::Word), Some(model.cvr));
    // The core slept between interrupts (it was woken by SysTick only)
    assert!(m.cores[0].instructions() < 20 * (calls + 2), "{} instructions", m.cores[0].instructions());
}

#[test]
fn systick_on_the_core_clock_is_not_fed_reference_ticks() {
    let mut m = machine(12);
    ppb_write(&mut m, SYST_RVR, 0xFF_FFFF);
    ppb_write(&mut m, SYST_CVR, 0);
    ppb_write(&mut m, SYST_CSR, 0b101); // ENABLE, CLKSOURCE = core clock, no interrupt
    m.run_until(Time::from_ms(1));
    let cvr = m.cores[0].systick.cvr;
    // About 1 ms of core cycles (asleep or not), not a thousand reference ticks
    let core_cycles = 0xFF_FFFF - cvr as u64;
    let expect = m.clk_sys().cycles_in(Time::from_ms(1));
    assert!(core_cycles.abs_diff(expect) < 100, "{core_cycles} vs {expect}");
}

const COUNTER1: u32 = SRAM_BASE + 0x204;
const BUF: u32 = SRAM_BASE + 0x1000;

/// Core 0 computing into SRAM with SysTick on the reference clock (`rvr`, interrupting), polling SYST_CVR and
/// SYST_CSR between rounds into a ring at `BUF`; its handler counts at `COUNTER` and writes a new RVR each time.
/// Core 0's code is in XIP SRAM with `xip`; with `quiet` its rounds only read memory (a quantum that only reads
/// is fused after while SysTick counts: there is no busy-wait to skip). Core 1 sleeps in WFI with SysTick on the reference clock too
/// (`rvr1`), its handler counting at `COUNTER1`. The watchdog tick divides `clk_ref` by `cycles`.
fn computing(cycles: u32, rvr: u32, rvr1: u32, xip: bool, quiet: bool) -> Machine {
    use rp2040::memmap::XIP_SRAM_BASE;
    let mut m = Machine::new(MachineConfig::default());
    let base = if xip { XIP_SRAM_BASE + 0x100 } else { SRAM_BASE + 0x400 };
    let (code, handler) = (base, base + 0x40);
    let words = [
        (0x00, 0x2000_2000),
        // A stub that goes there: ldr r0, [pc, #0]; bx r0
        (0x04, SRAM_BASE + 0x301),
        (0x300, 0x4700_4800),
        (0x304, code + 1),
        (0x3C, handler + 1),
        (0x800, 0x2000_3000),
        (0x804, SRAM_BASE + 0x101),
        (0x83C, SRAM_BASE + 0x111),
    ];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    // Core 1: wfi in a loop, its handler counting
    let core1: [(u32, u16); 9] = [
        (0x100, 0xBF30), // wfi
        (0x102, 0xE7FD), // b 0x100
        (0x110, 0x4802), // ldr r0, [pc, #8] (COUNTER1)
        (0x112, 0x6801), // ldr r1, [r0]
        (0x114, 0x3101), // adds r1, #1
        (0x116, 0x6001), // str r1, [r0]
        (0x118, 0x4770), // bx lr
        (0x11A, 0xBF00), // nop
        (0x11C, COUNTER1 as u16),
    ];
    for (off, hw) in core1 {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    assert!(m.sys.poke(SRAM_BASE + 0x11E, Size::Half, COUNTER1 >> 16));
    let core0: [u16; 24] = [
        0x4C09,                              // ldr r4, [pc, #36] (BUF)
        0x4D0A,                              // ldr r5, [pc, #40] (SYST_CSR)
        0x2600,                              // movs r6, #0
        0x23C8,                              // outer: movs r3, #200
        if quiet { 0x6822 } else { 0x6022 }, // inner: ldr r2, [r4, #0] or str r2, [r4, #0]
        0x3203,                              // adds r2, #3
        0x4357,                              // muls r7, r2
        0x3B01,                              // subs r3, #1
        0xD1FA,                              // bne inner
        0x68A8,                              // ldr r0, [r5, #8] (SYST_CVR)
        0x6829,                              // ldr r1, [r5, #0] (SYST_CSR: COUNTFLAG, cleared by the read)
        0x00F7,                              // lsls r7, r6, #3
        0x193F,                              // adds r7, r7, r4
        0x60B8,                              // str r0, [r7, #8]
        0x60F9,                              // str r1, [r7, #12]
        0x3601,                              // adds r6, #1
        0x273F,                              // movs r7, #63
        0x403E,                              // ands r6, r7
        0xE7EF,                              // b outer
        0xBF00,                              // nop
        BUF as u16,
        (BUF >> 16) as u16,
        SYST_CSR as u16,
        (SYST_CSR >> 16) as u16,
    ];
    let handler0: [u16; 14] = [
        0x4804,                       // ldr r0, [pc, #16] (COUNTER)
        0x6801,                       // ldr r1, [r0]
        0x3101,                       // adds r1, #1
        0x6001,                       // str r1, [r0]
        0x4A03,                       // ldr r2, [pc, #12] (SYST_RVR)
        0x2307,                       // movs r3, #7
        0x400B,                       // ands r3, r1
        0x3300 | (rvr & 0xFF) as u16, // adds r3, #rvr: the next period varies with the count
        0x6013,                       // str r3, [r2]
        0x4770,                       // bx lr
        COUNTER as u16,
        (COUNTER >> 16) as u16,
        SYST_RVR as u16,
        (SYST_RVR >> 16) as u16,
    ];
    let mut put = |addr: u32, hws: &[u16]| {
        for (k, hw) in hws.iter().enumerate() {
            let a = addr + 2 * k as u32;
            if xip {
                let off = (a - XIP_SRAM_BASE) as usize;
                m.sys.xip_sram[off..off + 2].copy_from_slice(&hw.to_le_bytes());
            } else {
                assert!(m.sys.poke(a, Size::Half, *hw as u32));
            }
        }
    };
    put(code, &core0);
    put(handler, &handler0);
    m.direct_boot().unwrap();
    m.sys.write(&mut NullSink, 0, Time::ZERO, WATCHDOG_BASE + 0x2C, Size::Word, 1 << 9 | cycles).unwrap();
    let st = &mut m.cores[0].systick;
    (st.rvr, st.cvr, st.csr) = (rvr, 0, 0b011); // ENABLE, TICKINT, CLKSOURCE = reference
    let mut trace = NullSink;
    let mut bus = rp2040::system::CoreBus { sys: &mut m.sys, trace: &mut trace, core: 1, now: Time::ZERO };
    m.cores[1].reset(&mut bus, SRAM_BASE + 0x800);
    m.core_enabled[1] = true;
    let st = &mut m.cores[1].systick;
    (st.rvr, st.cvr, st.csr) = (rvr1, 0, 0b011);
    m
}

/// Everything observable of a run of [`computing`] to `end` in `pieces` runs, batched or one instruction at a time.
fn computed(m: &mut Machine, end: Time, pieces: u64, batched: bool) -> Vec<String> {
    if !batched {
        m.breakpoints.insert(0xFFFF_FFF1);
    }
    for k in 1..=pieces {
        assert_eq!(m.run_until(Time(end.as_ps() * k / pieces)), StopReason::TimeLimit);
    }
    let mut out = vec![format!("now {}", m.now)];
    for c in 0..2 {
        let core = &m.cores[c];
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        let st = &core.systick;
        out.push(format!(
            "core{c} at {}: {} cycles, {} instructions, regs {regs:08x?}, systick {:x} {} {}",
            m.core_time(c),
            core.cycles(),
            core.instructions(),
            st.csr,
            st.rvr,
            st.cvr
        ));
    }
    for a in [COUNTER, COUNTER1] {
        out.push(format!("{a:x}: {}", m.sys.peek(a, Size::Word).unwrap()));
    }
    out.push(format!("{:?}", (0..130).map(|k| m.sys.peek(BUF + 4 * k, Size::Word).unwrap()).collect::<Vec<_>>()));
    out
}

#[test]
fn batches_run_across_reference_ticks_as_steps_do() {
    // Ticks of 1 us and of one clk_ref cycle; code in SRAM and in XIP SRAM; in one run and in uneven pieces
    for (cycles, xip, pieces, quiet) in [
        (12, false, 1, false),
        (12, true, 7, false),
        (1, false, 3, false),
        (1, true, 1, false),
        (12, false, 5, true),
        (1, true, 1, true),
    ] {
        let end = Time::from_us(12_345);
        let mut batched = computing(cycles, 200, 150, xip, quiet);
        let a = computed(&mut batched, end, pieces, true);
        let b = computed(&mut computing(cycles, 200, 150, xip, quiet), end, pieces, false);
        assert_eq!(a, b, "cycles {cycles}, xip {xip}, pieces {pieces}, quiet {quiet}");
        // It ran: both handlers were called many times, core 0 read SysTick as it counted
        let calls: Vec<u32> = [COUNTER, COUNTER1].iter().map(|&a| batched.sys.peek(a, Size::Word).unwrap()).collect();
        assert!(calls.iter().all(|&n| n > 20), "{calls:?}");
        // ...and its quanta were fused across ticks
        assert!(batched.quanta_fused > 100, "{} quanta fused: {a:?}", batched.quanta_fused);
    }
}
