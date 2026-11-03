//! M1: our own embassy firmware (`test_programs/embassy_blinky`) toggles GPIO 25 every 250 ms through
//! `embassy_time::Timer`, so the toggles measure the whole TIMER/alarm/NVIC/WFE path against virtual time.

mod common;

use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, VecSink};

#[test]
fn toggles_every_250_ms_of_virtual_time() {
    let sink = VecSink::new(&[Category::Gpio]);
    let mut s = common::boot(&common::repo().join("test_programs/embassy_blinky.elf"), Some(Box::new(sink.clone())));
    s.run_until(Time::from_ms(2_200));

    assert!(s.logged("blinky up, clk_sys 125000000 Hz"), "{:?}", s.logs);
    // Floats go through the ROM routines, integer division through the SIO divider
    assert!(s.logged("toggle 4 at 1000 ms (1.3333334 571)"), "{:?}", s.logs);
    assert!(s.logged("toggle 8 at 2000 ms (2.6666667 1142)"), "{:?}", s.logs);

    let toggles: Vec<Time> = sink
        .take()
        .into_iter()
        .filter_map(|(t, e)| matches!(e, TraceEvent::Gpio { pin: 25, .. }).then_some(t))
        .collect();
    assert_eq!(toggles.len(), 8);
    for w in toggles.windows(2) {
        let d = (w[1] - w[0]).as_us();
        // 250 ms plus the few microseconds the executor needs to wake and re-arm (`Timer::after` drifts by that)
        assert!((250_000..250_100).contains(&d), "interval {d} us");
    }
}
