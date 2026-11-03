//! SysTick with CLKSOURCE clear counts the reference clock, which on the RP2040 is the watchdog's tick (1 µs once
//! `WATCHDOG.TICK` is set up, as pico-sdk does): it decrements once per tick of virtual time, wraps every RVR + 1
//! ticks and pends its exception, waking a core asleep in WFI.

use rp2040::memmap::{SRAM_BASE, WATCHDOG_BASE};
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
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
