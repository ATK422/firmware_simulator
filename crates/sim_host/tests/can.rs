//! M3: boards on the CAN bus. A Rust board (titan_firmware_rs `template`, client 1) and the C Titan bootloader talk
//! through the MCP2517FD model on an in-process CAN FD bus; the scripted mock bridge plays `titan_bridge`.
//!
//! Real `vcan0`/`candump`/`titan_bridge` runs are not possible in the cloud environment (no CAN in the kernel); the
//! bus record stands in for `candump vcan0` and the mock bridge (built on the real `canmore` and `titan_link` crates)
//! for `titan_bridge`.

mod common;

use canmore::id::{Direction, Fields, FrameType};
use sim_core::bus::Size;
use sim_core::time::Time;
use sim_devices::can::{CanBus, Sender, candump};
use sim_host::Session;
use sim_host::board::Board;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::Robot;

fn board(name: &str) -> Board {
    Board::load(&common::repo().join("boards").join(format!("{name}.toml"))).unwrap()
}

/// Boots `elf` on `board` with its CAN bus recorded and, optionally, a mock bridge for `client`.
fn on_bus(elf: &std::path::Path, board_name: &str, bridge: Option<MockBridge>) -> Session {
    let b = board(board_name);
    let mut s = common::boot(elf, None);
    b.apply(&mut s.machine).unwrap();
    let mut bus = CanBus::new("can0", b.can_rates().unwrap());
    bus.record = Some(Vec::new());
    s.can_bus = Some(bus);
    if let Some(br) = bridge {
        s.can_endpoints.push(Box::new(br));
    }
    s
}

fn records(s: &Session) -> &[sim_devices::can::BusRecord] {
    s.can_bus.as_ref().unwrap().record.as_ref().unwrap()
}

/// CANmore heartbeats (utility channel 15) sent by `client`.
fn heartbeats(recs: &[sim_devices::can::BusRecord], client: u8) -> Vec<(Time, u8)> {
    recs.iter()
        .filter(|r| {
            let f = Fields::decode(r.frame.id, r.frame.extended);
            f.frame_type == FrameType::Util
                && f.noc == canmore::CHAN_HEARTBEAT
                && f.client_id == client
                && f.direction == Direction::ClientToAgent
        })
        .map(|r| (r.end, r.frame.data[0]))
        .collect()
}

#[test]
fn rust_board_heartbeat_on_the_bus() {
    let Some(elf) = common::titan_elf("template") else { return };
    // With the bridge listening, frames are acknowledged (what a vcan interface does too)
    let bridge = MockBridge::new(1, true, Time::from_ms(1000));
    let mut s = on_bus(&elf, "mk2/power_board", Some(bridge));
    s.run_until(Time::from_ms(2200));
    assert!(s.logged("CAN controller online as client 1"), "{:?}", s.logs);
    let hb = heartbeats(records(&s), 1);
    assert!(hb.len() >= 4, "{hb:?}");
    for w in hb.windows(2) {
        let d = (w[1].0 - w[0].0).as_us();
        assert!((499_000..502_000).contains(&d), "heartbeat interval {d} us");
    }
    // candump vcan0 would show e.g. `06F#C0`: client 1, utility frame, channel 15, one byte
    let first = records(&s).iter().find(|r| matches!(r.sender, Sender::Node(_))).unwrap();
    assert_eq!(candump(&first.frame)[..4], *"06F#");
    assert!(first.acked);
}

#[test]
fn rust_board_alone_gets_no_ack() {
    let Some(elf) = common::titan_elf("template") else { return };
    let mut s = on_bus(&elf, "mk2/power_board", None);
    s.run_until(Time::from_ms(600));
    let recs = records(&s);
    assert!(!recs.is_empty());
    assert!(recs.iter().all(|r| !r.acked), "nobody else on the bus acknowledges");
}

#[test]
fn bridge_connects_heartbeat_and_firmware_status() {
    let Some(elf) = common::titan_elf("template") else { return };
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    let mut s = on_bus(&elf, "mk2/power_board", Some(bridge.clone()));
    s.run_until(Time::from_ms(3500));
    assert!(s.logged("ROS connected"));
    assert!(bridge.state().errors.is_empty(), "{:?}", bridge.state().errors);
    // titan_link heartbeat every 100 ms
    let hb = bridge.link_heartbeats();
    assert!(hb.len() > 25, "{}", hb.len());
    for w in hb.windows(2) {
        let d = (w[1] - w[0]).as_us();
        assert!((99_000..101_500).contains(&d), "{d}");
    }
    // Firmware status every second, with the board identity and the ROS kill switch enabled and not killing
    let st = bridge.statuses();
    assert!(st.len() >= 3, "{st:?}");
    for w in st.windows(2) {
        let d = (w[1].0 - w[0].0).as_ms();
        assert!((999..=1001).contains(&d), "{d}");
    }
    let last = &st.last().unwrap().1;
    assert_eq!(last.board_name.as_str(), "mk2/power_board");
    assert_eq!((last.client_id, last.bus_id, last.version_major), (1, 1, 1));
    assert_eq!(last.kill_switches_enabled, 1);
    assert_eq!(last.kill_switches_asserting_kill, 0);
    assert_eq!(last.faults, 0);
    // CANmore heartbeats report no error and a valid termination reading
    let hbs = bridge.state().heartbeats.clone();
    let h = hbs.last().unwrap().1;
    assert!(!h.error && h.term_valid && h.term_enabled);
}

/// The status LED's blue channel (GPIO 15, PWM 7B) is on while the vehicle is enabled.
fn blue_on(s: &Session) -> bool {
    let slice = &s.machine.sys.pwm.slices[7];
    slice.cc >> 16 >= slice.top && slice.top > 0
}

/// Watchdog counter (CTRL.TIME), which the safety task reloads every 10 ms.
fn watchdog_counter(s: &mut Session) -> u32 {
    s.machine.debug_read(0, 0x4005_8000, Size::Word).unwrap() & 0xFF_FFFF
}

#[test]
fn stopping_the_bridge_kills_the_vehicle() {
    let Some(elf) = common::titan_elf("template") else { return };
    let bridge = MockBridge::new(1, true, Time::ZERO);
    let mut s = on_bus(&elf, "mk2/power_board", Some(bridge.clone()));

    // Before ROS connects the watchdog runs with its 3 s setup timeout (counter loaded with 2 x 3e6)
    s.run_until(Time::from_ms(150));
    assert!(!s.logged("ROS connected"));
    let c = watchdog_counter(&mut s);
    assert!(c > 5_000_000, "setup watchdog counter {c}");

    // Connected and enabled: watchdog tightened to 250 ms, LED blue
    bridge.set_kill(Some(false));
    s.run_until(Time::from_ms(1000));
    assert!(s.logged("ROS connected"));
    let c = watchdog_counter(&mut s);
    assert!(c <= 500_000 && c > 400_000, "active watchdog counter {c}");
    assert!(blue_on(&s), "vehicle enabled");

    // Stop the bridge: no more pings or kill switch updates
    let stop = s.machine.now;
    bridge.stop();
    s.run_until(stop + Time::from_ms(400));
    assert!(blue_on(&s), "still enabled 400 ms after the bridge stopped");
    // The ROS kill switch needs an update every 500 ms; without one it times out and kills
    s.run_until(stop + Time::from_ms(600));
    assert!(!blue_on(&s), "killed by the kill switch timeout");
    assert!(!s.logged("Lost connection to ROS"));
    // 700 ms without pings: ROS counts as disconnected, safety de-initialized, watchdog relaxed again
    s.run_until(stop + Time::from_ms(900));
    let lost = s.logs.iter().find(|(_, l)| l.message.contains("Lost connection to ROS")).expect("disconnect");
    let last_ping = bridge.state().last_ping.unwrap();
    let after = (lost.0 - last_ping).as_ms();
    assert!((700..=760).contains(&after), "disconnect {after} ms after the last ping");
    s.run_until(stop + Time::from_ms(1000));
    assert!(watchdog_counter(&mut s) > 5_000_000, "back to the setup timeout");
}

#[test]
fn hung_executor_trips_the_watchdog() {
    let Some(elf) = common::titan_elf("template") else { return };
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    let mut s = on_bus(&elf, "mk2/power_board", Some(bridge));
    s.run_until(Time::from_ms(1000));
    assert!(s.logged("ROS connected"));
    // Starve the executor: mask every interrupt at the NVIC, so no task is woken again (masking only the TIMER
    // interrupt is not enough: CAN traffic wakes the CAN tasks, and rescheduling their timeouts makes the time driver
    // run expired timers, the safety task's included). The 250 ms watchdog must reset the board.
    s.machine.cores[0].nvic.enabled = 0;
    let resets = s.machine.resets;
    s.run_until(Time::from_ms(1240));
    assert_eq!(s.machine.resets, resets, "no reset before 250 ms");
    s.run_until(Time::from_ms(1400));
    assert_eq!(s.machine.resets, resets + 1, "watchdog reset");
    s.run_until(Time::from_ms(1700));
    assert!(s.logged("Watchdog timeout after"), "{:?}", s.logs.iter().rev().take(5).collect::<Vec<_>>());
    assert!(s.logged("Fault FAULT_WATCHDOG_RESET raised"));
}

#[test]
fn c_bootloader_on_the_bus() {
    let Some(elf) =
        common::c_elf("PowerBoard/titan_lib/titan_bootloader/src/titan_bootloader-build/titan_bootloader.elf")
    else {
        return;
    };
    let bridge = MockBridge::new(1, true, Time::from_ms(10_000));
    let mut s = on_bus(&elf, "mk2/power_board", Some(bridge));
    s.run_until(Time::from_ms(1200));
    s.poll_uarts();
    assert!(s.uart_text[0].contains("Running BL"), "{}", s.uart_text[0]);
    // The C library (with CRC and safe writes) brought the controller up and the bootloader heartbeats
    let hb = heartbeats(records(&s), 1);
    assert!(hb.len() >= 2, "{hb:?}");
    let mode = canmore::Heartbeat::from_byte(hb.last().unwrap().1).mode;
    assert_eq!(mode, canmore::ControlInterfaceMode::Bootloader as u8);
}

#[test]
fn c_and_rust_boards_share_the_bus() {
    let (Some(rust), Some(c)) = (
        common::titan_elf("template"),
        common::c_elf("LEDBoard/titan_lib/titan_bootloader/src/titan_bootloader-build/titan_bootloader.elf"),
    ) else {
        return;
    };
    let mut bus = CanBus::new("can0", board("mk2/power_board").can_rates().unwrap());
    bus.record = Some(Vec::new());
    let mut robot = Robot::new(bus);
    for (elf, name) in [(&rust, "mk2/power_board"), (&c, "talos/led_board")] {
        let mut s = common::boot(elf, None);
        board(name).apply(&mut s.machine).unwrap();
        robot.add(name, s, Some(0));
    }
    let bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    robot.buses[0].endpoints.push(Box::new(bridge.clone()));
    assert!(robot.run_until(Time::from_ms(2500)).is_none());

    let recs = robot.buses[0].bus.record.as_ref().unwrap();
    let rust_hb = heartbeats(recs, 1);
    let c_hb = heartbeats(recs, 6);
    assert!(rust_hb.len() >= 4 && c_hb.len() >= 3, "{rust_hb:?} {c_hb:?}");
    // Every frame after both are up is acknowledged: each board acknowledges the other
    let up = rust_hb[0].0.max(c_hb[0].0);
    assert!(recs.iter().filter(|r| r.start > up).all(|r| r.acked));
    // The Rust board's link works with the C board on the bus
    assert!(robot.boards[0].logged("ROS connected"));
    assert!(bridge.statuses().len() >= 2);
    assert!(bridge.state().errors.is_empty());
    robot.boards[1].poll_uarts();
    assert!(robot.boards[1].uart_text[0].contains("Running BL"));
}
