//! The run loop with both cores in play: what one core's state does over time must not depend on whether the other
//! core happens to be running or asleep.

use rp2040::memmap::SRAM_BASE;
use rp2040::system::CoreBus;
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_core::trace::NullSink;

/// SysTick handler calls on core 1.
const COUNTER: u32 = SRAM_BASE + 0x200;

/// Core 0 runs `b .` (`spin`) or sleeps in a `wfi` loop; core 1 sleeps in a `wfi` loop with SysTick on the core
/// clock waking it every `rvr + 1` cycles, its handler counting at `COUNTER`.
fn machine(spin: bool, rvr: u32) -> Machine {
    let mut m = Machine::new(MachineConfig::default());
    let words = [
        // Core 0's vector table
        (0x00, 0x2000_2000),
        (0x04, SRAM_BASE + if spin { 0x121 } else { 0x101 }),
        // Core 1's
        (0x400, 0x2000_3000),
        (0x404, SRAM_BASE + 0x101),
        (0x43C, SRAM_BASE + 0x111),
    ];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    let code: [(u32, u16); 10] = [
        (0x100, 0xBF30), // wfi
        (0x102, 0xE7FD), // b 0x100
        (0x110, 0x4802), // ldr r0, [pc, #8] (COUNTER)
        (0x112, 0x6801), // ldr r1, [r0]
        (0x114, 0x3101), // adds r1, #1
        (0x116, 0x6001), // str r1, [r0]
        (0x118, 0x4770), // bx lr
        (0x11A, 0xBF00), // nop
        (0x11C, COUNTER as u16),
        (0x120, 0xE7FE), // b .
    ];
    for (off, hw) in code {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    assert!(m.sys.poke(SRAM_BASE + 0x11E, Size::Half, COUNTER >> 16));
    m.direct_boot().unwrap();
    // Core 1 started as if launched with its own vector table
    let mut trace = NullSink;
    let mut bus = CoreBus { sys: &mut m.sys, trace: &mut trace, core: 1, now: Time::ZERO };
    m.cores[1].reset(&mut bus, SRAM_BASE + 0x400);
    m.core_enabled[1] = true;
    let st = &mut m.cores[1].systick;
    st.rvr = rvr;
    st.cvr = 0;
    st.csr = 0b111; // ENABLE, TICKINT, CLKSOURCE = core clock
    m
}

/// SysTick handler calls on core 1 in the first `end` of virtual time, and how many wraps that is.
fn systick_calls(spin: bool, rvr: u32, end: Time) -> (u64, u64) {
    let mut m = machine(spin, rvr);
    assert_eq!(m.run_until(end), StopReason::TimeLimit);
    let expect = m.clk_sys().cycles_in(end) / (rvr as u64 + 1);
    (m.sys.peek(COUNTER, Size::Word).unwrap() as u64, expect)
}

/// With both cores asleep, time jumps to core 1's next SysTick wrap. At the reset clock (ROSC, whose period is not
/// a whole number of picoseconds) the jump must still count the cycles up to the wrap, so that it happens.
#[test]
fn idle_fast_forward_reaches_the_systick_wrap() {
    let (n, expect) = systick_calls(false, 99, Time::from_ms(10));
    assert!(expect > 100, "{expect} wraps expected");
    assert!(n.abs_diff(expect) <= 1, "{n} SysTick calls on core 1, expected {expect}");
}

/// A core asleep in WFI keeps counting SysTick on the core clock while the other core runs (quantum by quantum),
/// and its exception wakes it on time, as when the other core sleeps too.
#[test]
fn sleeping_core_systick_counts_while_the_other_core_runs() {
    let (n, expect) = systick_calls(true, 99, Time::from_ms(10));
    let (asleep, _) = systick_calls(false, 99, Time::from_ms(10));
    assert!(n.abs_diff(expect) <= 1, "{n} SysTick calls on core 1, expected {expect}");
    assert!(n.abs_diff(asleep) <= 1, "{n} vs {asleep} with core 0 asleep");
}

/// AIRCR.SYSRESETREQ resets only the requesting core's PSM domain. Core 1 then comes out of reset in the bootrom,
/// as after its PSM force-off is released, and waits there for the launch sequence on the FIFO (so that core 0
/// can launch it again); it is not switched off for good.
#[test]
fn core1_sysresetreq_restarts_it_in_the_bootrom() {
    let mut m = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    let words = [
        (0x00, 0x2000_2000),
        (0x04, SRAM_BASE + 0x121), // core 0: b .
        (0x400, 0x2000_3000),
        (0x404, SRAM_BASE + 0x131), // core 1: SYSRESETREQ
        (0x138, 0xE000_ED0C),       // AIRCR
        (0x13C, 0x05FA_0004),       // VECTKEY | SYSRESETREQ
    ];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    let code: [(u32, u16); 5] = [
        (0x120, 0xE7FE), // b .
        (0x130, 0x4801), // ldr r0, =AIRCR
        (0x132, 0x4902), // ldr r1, =0x05FA0004
        (0x134, 0x6001), // str r1, [r0]
        (0x136, 0xE7FE), // b .
    ];
    for (off, hw) in code {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    m.direct_boot().unwrap();
    // Core 1 launched at its vector table in SRAM
    let mut trace = NullSink;
    let mut bus = CoreBus { sys: &mut m.sys, trace: &mut trace, core: 1, now: Time::ZERO };
    m.cores[1].reset(&mut bus, SRAM_BASE + 0x400);
    m.core_enabled[1] = true;
    assert_eq!(m.run_until(Time::from_us(500)), StopReason::TimeLimit);
    assert!(m.core_enabled[1], "core 1 runs again after its reset");
    let pc = m.cores[1].pc();
    assert!(pc < 0x4000, "core 1 is in the bootrom, at {pc:#010x}");
    assert!(m.cores[1].sleep_state().is_some(), "waiting for the launch sequence");
    assert_eq!(m.resets, 0, "no chip reset");
    assert_eq!(m.cores[0].pc(), SRAM_BASE + 0x120, "core 0 runs on undisturbed");
}
