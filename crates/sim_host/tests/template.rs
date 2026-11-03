//! M1 acceptance on `titan_firmware_rs/firmware/template`: it boots through `embassy_rp::init`, prints its defmt
//! logs, and its status LED PWM runs at the driver's 1 kHz. Without an MCP2517FD (M3) the CAN bring-up fails three
//! times, the firmware panics and resets through the watchdog, and the next boot reports the panic.

mod common;

use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, VecSink};

#[test]
fn template_boots_logs_and_reboots_after_can_failure() {
    let Some(elf) = common::titan_elf("template") else { return };
    let sink = VecSink::new(&[Category::Gpio]);
    let mut s = common::boot(&elf, Some(Box::new(sink.clone())));
    s.run_until(Time::from_ms(1200));

    let msgs: Vec<&str> = s.logs.iter().map(|(_, l)| l.message.as_str()).collect();
    let pos =
        |text: &str| msgs.iter().position(|m| m.contains(text)).unwrap_or_else(|| panic!("no {text:?} in {msgs:#?}"));
    let boot = pos("mk2/power_board v1.0");
    let attempt1 = pos("CAN controller init failed (OscillatorTimeout), attempt 1");
    let panic = pos("Failed to initialize CAN bus hardware: OscillatorTimeout");
    let reported = pos("Reset by panic after");
    let fault = pos("Fault FAULT_WATCHDOG_RESET raised");
    assert!(boot < attempt1 && attempt1 < panic && panic < reported && reported < fault);
    assert!(msgs.iter().any(|m| m.contains("panics: 1")), "crash count survives the reset in scratch");
    assert!(s.machine.resets >= 2, "panic -> watchdog reset, twice in 1.2 s");

    // Firmware time: the second CAN attempt follows the first by the driver's 10 ms poll timeout + 1 ms reset delay
    // + 10 ms retry wait (plus SPI time)
    let ts = |text: &str| -> f64 {
        let (_, l) = s.logs.iter().find(|(_, l)| l.message.contains(text)).unwrap();
        l.timestamp.as_ref().unwrap().parse().unwrap()
    };
    let gap = ts("attempt 2") - ts("attempt 1");
    assert!((0.0205..0.0220).contains(&gap), "retry gap {gap}");

    // The red LED (GPIO 13, PWM 6B, inverted) is set "fully on" with CC = TOP, which leaves a one-count pulse per
    // period: the pin toggles at the LED driver's 1 kHz PWM rate.
    let rises: Vec<Time> = sink
        .take()
        .into_iter()
        .filter_map(|(t, e)| matches!(e, TraceEvent::Gpio { pin: 13, level: true }).then_some(t))
        .filter(|t| (Time::from_ms(140)..Time::from_ms(180)).contains(t))
        .collect();
    assert!(rises.len() > 30, "{rises:?}");
    for w in rises.windows(2) {
        let period = (w[1] - w[0]).as_ns();
        // divider 122.0625 (8.4 fixed point of 125 MHz / 1.024 MHz), 1024 counts: 999.96 us
        assert!((999_900..1_000_000).contains(&period), "PWM period {period} ns");
    }
}
