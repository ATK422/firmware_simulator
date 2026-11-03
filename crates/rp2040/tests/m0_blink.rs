//! M0 acceptance: the hand-written Thumb program `test_programs/blink.S` toggles GPIO 25 through SIO and the trace
//! shows each toggle at the cycle-exact interval (3005 cycles of `clk_sys`, which the program leaves on the ROSC).

use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, VecSink};

fn blink_machine() -> (Machine, VecSink) {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_programs/blink.elf")).unwrap();
    let image = Image::parse(&bytes).unwrap();
    let mut m = Machine::new(MachineConfig::default());
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    let sink = VecSink::new(&[Category::Gpio]);
    m.set_trace(Box::new(sink.clone()));
    m.load(&image).unwrap();
    assert_eq!(m.direct_boot().unwrap(), 0x1000_0000);
    (m, sink)
}

#[test]
fn blink_toggles_gpio25_every_3005_cycles() {
    let (mut m, sink) = blink_machine();
    assert_eq!(m.run_until(Time::from_ms(20)), StopReason::TimeLimit);
    let events = sink.take();

    let toggles: Vec<(Time, bool)> = events
        .iter()
        .filter_map(|(t, e)| match e {
            TraceEvent::Gpio { pin: 25, level } => Some((*t, *level)),
            _ => None,
        })
        .collect();
    assert!(toggles.len() >= 20, "{toggles:?}");
    // Alternating levels
    for w in toggles.windows(2) {
        assert_ne!(w[0].1, w[1].1);
    }
    // STR + LDR + 1000 x SUBS + 999 taken BNE + 1 untaken BNE + B = 3005 cycles of 8 ns
    assert!(toggles[0].1, "first change drives the pin high");
    assert_eq!(m.clk_sys().hz(), rp2040::periph::clocks::ROSC_HZ);
    let period = m.clk_sys().cycles(3005);
    for w in toggles.windows(2) {
        // 6.5 MHz cycles are not a whole number of picoseconds; intervals may differ by the rounding
        let interval = w[1].0 - w[0].0;
        assert!(interval.as_ps().abs_diff(period.as_ps()) <= 1, "toggle interval {interval:?} != {period:?}");
    }
}

#[test]
fn snapshot_restore_replays_identically() {
    let (mut m, sink) = blink_machine();
    m.run_until(Time::from_ms(2));
    let snap = m.snapshot();
    sink.take();
    m.run_until(Time::from_ms(6));
    let first = sink.take();
    m.restore(&snap);
    m.run_until(Time::from_ms(6));
    let second = sink.take();
    assert!(!first.is_empty());
    assert_eq!(first, second);
}
