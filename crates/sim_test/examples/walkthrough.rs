//! The walkthrough of `docs/testing.md`: a firmware test of the power board, step by step. The guide quotes this
//! file; `tests/guide.rs` runs it as a test and checks that the guide's snippets are still in it.
//!
//! ```text
//! cargo run --release -p sim_test --example walkthrough
//! ```
//!
//! Needs the titan_firmware_rs images (`cd external/titan_firmware_rs/firmware && cargo build --workspace
//! --release`, or `TITAN_RS_RELEASE_DIR`); without them it says so and does nothing.

use sim_test::titan_link::{DeviceToHost, HostToDevice, power};
use sim_test::{Bench, Fault, Time};

/// The power board's `state/electrical` readings the bridge received.
fn electrical(b: &Bench) -> Vec<(Time, power::ElectricalReadings)> {
    b.picked(|m| match m {
        DeviceToHost::Power(power::DeviceToHost::Electrical(r)) => Some(*r),
        _ => None,
    })
}

pub fn main() {
    // 1. Boot the image on its board, next to a mock titan_bridge for client 1
    let Some(elf) = sim_test::titan_rs("power_board") else { return };
    let mut b = Bench::builder(&elf).board("mk2/power_board").bridge(1).record_bus().boot().unwrap();
    let connected = b.wait_for_log("ROS connected", Time::from_secs(1)).unwrap();
    println!("ROS connected at {connected}");

    // 2. What the bridge received: the firmware status every second
    b.run_until(Time::from_ms(1500)).unwrap();
    let (_, status) = b.statuses().last().cloned().expect("a firmware status");
    assert_eq!(status.board_name.as_str(), "mk2/power_board");
    assert_eq!(status.client_id, 1);
    assert_eq!(b.faults(), Some(0));
    assert!(b.link_errors().is_empty());

    // 3. Board messages: the battery voltages from the board file's starting values
    let (_, r) = *electrical(&b).last().expect("an electrical reading");
    assert!((r.port_voltage - 16.2).abs() < 0.05, "{r:?}");

    // 4. Change a physical input while the board runs
    b.set("port_battery", 14.0);
    b.run_for(Time::from_ms(1100));
    let (_, r) = *electrical(&b).last().unwrap();
    assert!((r.port_voltage - 14.0).abs() < 0.05, "{r:?}");

    // 5. Send a message as the bridge would: command/electrical DISABLE_FANS (6)
    let fan_pin = 28;
    assert!(b.session.machine.sys.gpio.input(fan_pin), "the fan is on at boot");
    let sent = b.now();
    b.send(HostToDevice::Power(power::HostToDevice::ElectricalCommand(6)));
    b.wait_for_log_after("Fans disabled", sent, Time::from_ms(500)).unwrap();
    assert!(!b.session.machine.sys.gpio.input(fan_pin));

    // 6. The raw CAN frames (with `record_bus`)
    let frames = b.frames();
    assert!(frames.iter().all(|f| f.error.is_none()));
    println!("{} frames on the bus, the last at {}", frames.len(), frames.last().unwrap().end);

    // 7. Snapshot once, then run scenarios from the snapshot: titan_bridge goes away
    let base = b.snapshot();
    let mut lost = base.snapshot();
    let t0 = lost.now();
    lost.inject(Fault::Bridge(false)).unwrap();
    let t = lost.wait_for_log_after("Lost connection to ROS", t0, Time::from_secs(2)).unwrap();
    let after = t - t0;
    assert!(after >= Time::from_ms(600) && after <= Time::from_ms(800), "{after}");
    println!("lost ROS {after} after the bridge stopped");

    // ... or the same fault as the command line writes it, at a set time
    let mut cut = base.snapshot();
    let at = cut.now() + Time::from_ms(100);
    cut.inject_at(at, "can-unplug".parse().unwrap());
    cut.inject_at(at + Time::from_secs(1), "can-plug".parse().unwrap());
    cut.run_for(Time::from_secs(2));
    assert!(cut.log_times("Lost connection to ROS").into_iter().any(|t| t > at));
}
