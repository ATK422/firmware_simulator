//! Busy-waits on the timer that last many ticks are skipped up to their deadline (`Machine::skip_timer_waits`, see
//! `CortexM0Plus::timer_wait`) rather than tick by tick. Runs with and without must be identical. A core waits
//! twenty timer ticks in a loop, toggles a GPIO and waits again; a timer alarm interrupts it every few ticks. The
//! waits are written the ways firmware writes them (embassy's 64-bit `Instant` comparison, 32-bit comparisons) and
//! some ways that must not be skipped (a signed comparison, a counter, flags or a register left from before a
//! decision), which must still give the same run.

use rp2040::memmap::{IO_BANK0_BASE, RESETS_BASE, SIO_BASE, SRAM_BASE, TIMER_BASE, WATCHDOG_BASE};
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_core::trace::{Category, NullSink, VecSink};

const COUNTER: u32 = SRAM_BASE + 0x1F0;
/// Where the wait loop starts.
const WAIT: u32 = 0x10C;

/// `ldr rd, [pc, #..]` at `at` loading the word at `lit`.
fn ldr_lit(rd: u16, at: u32, lit: u32) -> u16 {
    0x4800 | rd << 8 | ((lit - ((at + 4) & !3)) / 4) as u16
}

/// `b<cond> to` at `at`.
fn bcond(cond: u16, at: u32, to: u32) -> u16 {
    0xD000 | cond << 8 | ((to.wrapping_sub(at + 4) as i32 >> 1) as u16 & 0xFF)
}

/// `b to` at `at`.
fn b(at: u32, to: u32) -> u16 {
    0xE000 | ((to.wrapping_sub(at + 4) as i32 >> 1) as u16 & 0x7FF)
}

const EQ: u16 = 0;
const NE: u16 = 1;
const CS: u16 = 2;
const CC: u16 = 3;
const MI: u16 = 4;
const HI: u16 = 8;

/// The wait loops, as code from [`WAIT`] (branches back to it; falling through ends the wait), and whether it is
/// skipped across ticks.
fn waits() -> Vec<(&'static str, Vec<u16>, bool)> {
    let w = WAIT;
    vec![
        (
            // embassy-time's `while Instant::now() < deadline {}` (the ESC board's control loop)
            "embassy",
            vec![
                0x6830, // ldr r0, [r6] (TIMERAWH)
                0x6871, // ldr r1, [r6, #4] (TIMERAWL)
                0x6832, // ldr r2, [r6] (TIMERAWH)
                0x4290, // cmp r0, r2
                bcond(NE, w + 8, w),
                0x1B09, // subs r1, r1, r4
                0x4198, // sbcs r0, r3
                bcond(CC, w + 14, w),
            ],
            true,
        ),
        ("lo_below", vec![0x6870, 0x42A0, bcond(CC, w + 4, w)], true), // ldr r0, [r6, #4]; cmp r0, r4; bcc
        ("lo_not_equal", vec![0x6870, 0x42A0, bcond(NE, w + 4, w)], true), // ...; bne
        ("deadline_above", vec![0x6871, 0x428C, bcond(HI, w + 4, w)], true), // ldr r1, [r6, #4]; cmp r4, r1; bhi
        // (int32_t)(now - deadline) < 0: the sign of a difference is not understood
        ("signed", vec![0x6870, 0x1B00, bcond(MI, w + 4, w)], false), // ldr r0, [r6, #4]; subs r0, r0, r4; bmi
        // A count of times round: the state never comes back
        ("counting", vec![0x3201, 0x6870, 0x42A0, bcond(CC, w + 6, w)], false), // adds r2, #1; ldr; cmp; bcc
        // Flags from the read before: they are read before they are written at every address of the loop
        ("flags_from_before", vec![0x42A1, 0x6871, bcond(CC, w + 4, w)], false), // cmp r1, r4; ldr r1, [r6, #4]; bcc
        // r1 is written after the decision, so landing would leave it stale on the way out
        (
            "written_after_decision",
            vec![0x6870, 0x42A0, bcond(CS, w + 4, w + 10), 0x0001, b(w + 8, w), 0xBF00], // ...; bcs out; movs r1, r0
            false,
        ),
        // Equality the other way round: wait while the low word is the deadline (it only is for one tick)
        ("lo_equal", vec![0x6870, 0x42A0, bcond(EQ, w + 4, w)], false),
    ]
}

fn machine(wait: &[u16], alarm: bool, timer_waits: bool) -> (Machine, VecSink) {
    let mut m = Machine::new(MachineConfig::default());
    m.skip_timer_waits = timer_waits;
    let sink = VecSink::new(&[Category::Gpio]);
    m.set_trace(Box::new(sink.clone()));
    let words = [
        (0x00, 0x2000_2000),
        (0x04, SRAM_BASE + 0x101),
        (0x40, SRAM_BASE + 0x181), // TIMER_IRQ_0
        (0x1C0, TIMER_BASE + 0x24),
        (0x1C4, SIO_BASE + 0x1C),
        (0x1C8, 1 << 25),
        (0x1CC, TIMER_BASE),
        (0x1D0, COUNTER),
    ];
    for (off, v) in words {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Word, v));
    }
    let mut code: Vec<(u32, u16)> = vec![
        (0x100, ldr_lit(6, 0x100, 0x1C0)), // ldr r6, =TIMERAWH
        (0x102, ldr_lit(5, 0x102, 0x1C4)), // ldr r5, =GPIO_OUT_XOR
        (0x104, ldr_lit(7, 0x104, 0x1C8)), // ldr r7, =1 << 25
        (0x106, 0x2300),                   // movs r3, #0
        (0x108, 0x6874),                   // loop: ldr r4, [r6, #4] (TIMERAWL)
        (0x10A, 0x3414),                   // adds r4, #20: wait twenty ticks
    ];
    let mut at = WAIT;
    for &hw in wait {
        code.push((at, hw));
        at += 2;
    }
    code.push((at, 0x602F)); // str r7, [r5]: toggle GPIO 25
    code.push((at + 2, b(at + 2, 0x108)));
    let alarm_code = [
        ldr_lit(0, 0x180, 0x1CC), // ldr r0, =TIMER_BASE
        0x2101,                   // movs r1, #1
        0x6341,                   // str r1, [r0, #0x34] (INTR: clear)
        0x6A81,                   // ldr r1, [r0, #0x28] (TIMERAWL)
        0x3107,                   // adds r1, #7
        0x6101,                   // str r1, [r0, #0x10] (ALARM0: re-arm)
        0,                        // ldr r0, =COUNTER
        0x6801,                   // ldr r1, [r0]
        0x3101,                   // adds r1, #1
        0x6001,                   // str r1, [r0]
        0x4770,                   // bx lr
    ];
    for (i, &hw) in alarm_code.iter().enumerate() {
        let a = 0x180 + 2 * i as u32;
        code.push((a, if i == 6 { ldr_lit(0, a, 0x1D0) } else { hw }));
    }
    for (off, hw) in code {
        assert!(m.sys.poke(SRAM_BASE + off, Size::Half, hw as u32));
    }
    m.direct_boot().unwrap();
    let mut w = |addr: u32, v: u32| m.sys.write(&mut NullSink, 0, Time::ZERO, addr, Size::Word, v).unwrap();
    // The timer ticks every 511 cycles of the reset clocks (ROSC), so a tick spans many times round a loop
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

/// The GPIO trace, the final state and the waits skipped across ticks; `slices` cuts the run as sessions do (100
/// µs), or finer.
fn run(wait: &[u16], alarm: bool, timer_waits: bool, slice: Time) -> (Vec<String>, String, u64) {
    let (mut m, sink) = machine(wait, alarm, timer_waits);
    let end = Time::from_ms(40);
    let mut t = Time::ZERO;
    while t < end {
        t = (t + slice).min(end);
        assert_eq!(m.run_until(t), StopReason::TimeLimit);
    }
    let trace = sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")).collect();
    let c = &m.cores[0];
    let regs: Vec<u32> = (0..c.num_regs()).map(|r| c.read_reg(r)).collect();
    let state = format!(
        "{} {} {} {} {regs:08x?} counter {} sram {:x}",
        m.now,
        m.core_time(0),
        c.cycles(),
        c.instructions(),
        m.sys.peek(COUNTER, Size::Word).unwrap(),
        m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
    );
    (trace, state, m.timer_waits_skipped)
}

#[test]
fn timer_waits_skipped_as_ticked() {
    for (name, wait, skippable) in waits() {
        for alarm in [false, true] {
            for slice in [Time::from_ms(40), Time::from_ms(1), Time::from_us(100), Time::from_ns(3_700)] {
                let (trace, state, skipped) = run(&wait, alarm, true, slice);
                let (trace_ref, state_ref, none) = run(&wait, alarm, false, slice);
                let what = format!("{name}, alarm {alarm}, slices of {slice}");
                assert_eq!(none, 0, "{what}");
                // (Finer slices end a wait's skip before the next tick: then it is left to the plain skip)
                if slice >= Time::from_ms(1) {
                    assert_eq!(skipped > 0, skippable, "{what}: {skipped} waits skipped");
                }
                assert!(trace.len() > 10, "{what}: {} GPIO toggles", trace.len());
                assert_eq!(state, state_ref, "{what}");
                assert_eq!(trace, trace_ref, "{what}");
                if alarm {
                    assert!(!state.contains("counter 0 "), "{what}: {state}");
                }
            }
        }
    }
}
