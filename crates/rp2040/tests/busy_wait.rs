//! Busy-waits on the timer (the ESC board's control loop spends most of its time in one) are skipped quantum by
//! quantum instead of interpreted, as long as nothing else can happen meanwhile. Runs with and without skipping
//! must be identical: here a core polls the timer in a loop of the same shape as embassy's `Instant::now`
//! comparison, toggles a GPIO each time the wait ends, and a timer alarm interrupts it every few ticks. The
//! machine runs on its reset clocks (ROSC); the timer ticks every 511 of them so that a tick spans many loops.

use rp2040::memmap::{IO_BANK0_BASE, RESETS_BASE, SIO_BASE, SRAM_BASE, TIMER_BASE, WATCHDOG_BASE};
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_core::trace::{Category, NullSink, VecSink};

const COUNTER: u32 = SRAM_BASE + 0x200;

fn machine(alarm: bool, skip: bool) -> (Machine, VecSink) {
    let mut m = Machine::new(MachineConfig::default());
    m.skip_spins = skip;
    let sink = VecSink::new(&[Category::Gpio]);
    m.set_trace(Box::new(sink.clone()));
    let words = [
        (0x00, 0x2000_2000),
        (0x04, SRAM_BASE + 0x101),
        (0x40, SRAM_BASE + 0x141), // TIMER_IRQ_0
        (0x120, TIMER_BASE + 0x24),
        (0x124, SIO_BASE + 0x1C),
        (0x128, 1 << 25),
        (0x160, TIMER_BASE),
        (0x164, COUNTER),
    ];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    let code: &[(u32, u16)] = &[
        (0x100, 0x4E07), // ldr r6, =TIMERAWH
        (0x102, 0x4D08), // ldr r5, =GPIO_OUT_XOR
        (0x104, 0x4F08), // ldr r7, =1 << 25
        (0x106, 0x2300), // movs r3, #0
        (0x108, 0x6874), // loop: ldr r4, [r6, #4] (TIMERAWL)
        (0x10A, 0x3402), // adds r4, #2: wait two ticks
        (0x10C, 0x6830), // wait: ldr r0, [r6] (high word)
        (0x10E, 0x6871), // ldr r1, [r6, #4] (low word)
        (0x110, 0x6832), // ldr r2, [r6] (high again)
        (0x112, 0x4290), // cmp r0, r2
        (0x114, 0xD1FA), // bne wait
        (0x116, 0x1B09), // subs r1, r1, r4
        (0x118, 0x4198), // sbcs r0, r3
        (0x11A, 0xD3F7), // bcc wait
        (0x11C, 0x602F), // str r7, [r5]: toggle GPIO 25
        (0x11E, 0xE7F3), // b loop
        (0x140, 0x4807), // alarm: ldr r0, =TIMER_BASE
        (0x142, 0x2101), // movs r1, #1
        (0x144, 0x6341), // str r1, [r0, #0x34] (INTR: clear)
        (0x146, 0x6A81), // ldr r1, [r0, #0x28] (TIMERAWL)
        (0x148, 0x3103), // adds r1, #3
        (0x14A, 0x6101), // str r1, [r0, #0x10] (ALARM0: re-arm)
        (0x14C, 0x4805), // ldr r0, =COUNTER
        (0x14E, 0x6801), // ldr r1, [r0]
        (0x150, 0x3101), // adds r1, #1
        (0x152, 0x6001), // str r1, [r0]
        (0x154, 0x4770), // bx lr
    ];
    for &(off, hw) in code {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    m.direct_boot().unwrap();
    let mut w = |addr: u32, v: u32| m.sys.write(&mut NullSink, 0, Time::ZERO, addr, Size::Word, v).unwrap();
    w(WATCHDOG_BASE + 0x2C, 1 << 9 | 511);
    w(RESETS_BASE + 0x3000, 1 << 5 | 1 << 8 | 1 << 21); // IO_BANK0, PADS_BANK0 and TIMER out of reset
    w(IO_BANK0_BASE + 25 * 8 + 4, 5); // GPIO 25: SIO
    w(SIO_BASE + 0x24, 1 << 25); // GPIO_OE_SET
    if alarm {
        w(TIMER_BASE + 0x38, 1); // INTE: alarm 0
        w(TIMER_BASE + 0x10, 3); // ALARM0 (arms it)
        m.cores[0].nvic.enabled = 1;
    }
    (m, sink)
}

fn run(alarm: bool, skip: bool) -> (Vec<String>, String, u64) {
    let (mut m, sink) = machine(alarm, skip);
    // In slices, as sessions run machines
    for i in 1..=200 {
        assert_eq!(m.run_until(Time::from_us(100 * i)), StopReason::TimeLimit);
    }
    let trace = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
    let c = &m.cores[0];
    let regs: Vec<u32> = (0..c.num_regs()).map(|r| c.read_reg(r)).collect();
    let state = format!(
        "{} {} {} {} {regs:08x?} counter {}",
        m.now,
        m.core_time(0),
        c.cycles(),
        c.instructions(),
        m.sys.peek(COUNTER, Size::Word).unwrap()
    );
    (trace, state, m.spin_skipped)
}

#[test]
fn skipped_busy_waits_change_nothing() {
    for alarm in [false, true] {
        let (trace, state, skipped) = run(alarm, true);
        let (trace_ref, state_ref, none) = run(alarm, false);
        assert_eq!(none, 0);
        assert!(skipped > 10_000, "alarm {alarm}: {skipped} instructions skipped");
        assert!(trace.len() > 50, "alarm {alarm}: {} GPIO toggles", trace.len());
        assert_eq!(state, state_ref, "alarm {alarm}");
        assert_eq!(trace, trace_ref, "alarm {alarm}");
        if alarm {
            assert!(!state.ends_with("counter 0"), "{state}");
        }
    }
}
