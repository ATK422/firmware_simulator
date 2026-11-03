//! M5: boards whose peripherals hang off PIO state machines.

mod common;

use sim_core::time::Time;
use sim_devices::can::CanBus;
use sim_devices::ws2812::{Rgb, Ws2812};
use sim_host::Session;
use sim_host::board::{Board, Inputs};
use sim_host::mock_bridge::MockBridge;
use titan_link::HostToDevice;

fn board(name: &str) -> Board {
    Board::load(&common::repo().join("boards").join(format!("{name}.toml"))).unwrap()
}

fn connected(elf: &std::path::Path, board_name: &str, client: u8) -> (Session, Inputs, MockBridge) {
    let b = board(board_name);
    let mut s = common::boot(elf, None);
    let inputs = b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    let bridge = MockBridge::new(client, b.can_rates().unwrap().data.is_some(), Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    (s, inputs, bridge)
}

#[test]
fn camera_cage_status_strip_colours() {
    use titan_link::camera_cage::{HostToDevice as Cage, LedCommand};
    let Some(elf) = common::titan_elf("camera_cage_bb") else { return };
    let trace = sim_core::trace::VecSink::new(&[sim_core::trace::Category::Device]);
    let events = trace.events.clone();
    let (mut s, _inputs, bridge) = connected(&elf, "mk2/camera_cage_bb", 4);
    s.machine.set_trace(Box::new(trace));
    s.run_until(Time::from_ms(1500));
    let strip = s.machine.sys.pin_device::<Ws2812>().unwrap();
    assert_eq!(strip.timing_errors, 0);
    // Refreshed every 50 ms with 12 pixels; the startup animation is orange (255, 10, 0)
    let frames = &strip.frames;
    assert!(frames.len() >= 20, "{} frames", frames.len());
    assert!(frames.iter().all(|f| f.1.len() == 12));
    let orange = Rgb { r: 255, g: 10, b: 0 };
    assert!(frames.iter().any(|f| f.1.contains(&orange)));
    let gaps: Vec<u64> = frames.windows(2).map(|w| (w[1].0 - w[0].0).as_ms()).collect();
    assert!(gaps.iter().all(|g| (49..=51).contains(g)), "{gaps:?}");
    // Solid green from ROS
    bridge.send(HostToDevice::CameraCage(Cage::LedCommand(LedCommand {
        red: 0,
        green: 200,
        blue: 0,
        mode: 0,
        target: 3,
    })));
    s.run_until(Time::from_ms(4000));
    let strip = s.machine.sys.pin_device::<Ws2812>().unwrap();
    let last = strip.last_frame().unwrap();
    assert!(last.iter().all(|&c| c == Rgb { r: 0, g: 200, b: 0 }), "{last:?}");
    // The colours are in the trace
    let ev = events.lock().unwrap();
    assert!(ev.iter().any(|(_, e)| matches!(e, sim_core::trace::TraceEvent::Device { message, .. } if message.contains("status strip (GPIO 9): 12 LEDs: 00c800"))));
}

fn esc_messages(bridge: &MockBridge) -> Vec<(Time, titan_link::esc::DeviceToHost)> {
    bridge
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            titan_link::DeviceToHost::Esc(e) => Some((*t, e.clone())),
            _ => None,
        })
        .collect()
}

#[test]
fn esc_board_dshot_frames_and_erpm_feedback() {
    use sim_devices::dshot::Esc;
    use titan_link::esc::{DeviceToHost as EscMsg, HostToDevice as EscCmd};
    let Some(elf) = common::titan_elf("esc_board") else { return };
    let (mut s, _inputs, bridge) = connected(&elf, "mk2/esc_board", 2);
    // Raw mode: targets are DShot throttle -999..=999, so the frames can be checked against the commands
    s.run_until(Time::from_ms(300));
    assert!(s.logged("ROS connected"));
    bridge.send(HostToDevice::Esc(EscCmd::SetRawMode(true)));
    // The ESCs get neutral for 5 s after power-up; keep commanding (targets go stale after 500 ms)
    let targets = [200i16, -300, 0, 500, 0, 0, 0, 0];
    let mut t = 400;
    while t < 6800 {
        bridge.send(HostToDevice::Esc(EscCmd::ThrusterRpm(targets)));
        t += 100;
        s.run_until(Time::from_ms(t));
    }
    let escs: Vec<&Esc> = s.machine.sys.pin_devices.iter().filter_map(|d| d.as_any().downcast_ref::<Esc>()).collect();
    assert_eq!(escs.len(), 4);
    for (i, e) in escs.iter().enumerate() {
        assert_eq!(e.bad_frames, 0, "{}", e.name);
        let f = *e.frames.last().unwrap();
        assert!(f.checksum_ok && f.bidirectional, "{f:?}");
        assert_eq!(f.rate_kbps, 600);
        // DShot values: forward 1..999 -> 49..1047, reverse -> 1049..2047, 0 neutral
        let expected = match targets[i] {
            0 => 0,
            v if v > 0 => v as u16 + 48,
            v => (-v) as u16 + 1048,
        };
        assert_eq!(f.throttle, expected, "{}", e.name);
        // A frame every 400 us
        let n = e.frames.len();
        let gap = (e.frames[n - 1].at - e.frames[n - 2].at).as_us();
        assert!((395..=420).contains(&gap), "{gap} us");
        assert!(e.replies as usize >= n - 2, "every frame answered with eRPM");
    }
    // eRPM feedback reaches ROS as signed mechanical RPM matching the motors
    let rpm: Vec<_> = esc_messages(&bridge)
        .into_iter()
        .filter_map(|(_, m)| match m {
            EscMsg::Rpm(r) => Some(r),
            _ => None,
        })
        .collect();
    let last = rpm.last().expect("RPM feedback");
    assert_eq!(last.valid_mask, 0xF);
    assert_eq!(last.cmd, [200, -300, 0, 500]);
    for (i, e) in escs.iter().enumerate() {
        let model = e.rpm;
        let reported = last.rpm[i] as f64;
        assert!((reported - model).abs() <= model.abs() * 0.02 + 15.0, "thruster {i}: {reported} vs {model}");
    }
    assert!(last.rpm[0] > 500 && last.rpm[1] < -800 && last.rpm[3] > 1500, "{:?}", last.rpm);
    // UART telemetry: requested in turn, answered on the telemetry pins, reported every 500 ms
    let telem: Vec<_> = esc_messages(&bridge)
        .into_iter()
        .filter_map(|(_, m)| match m {
            EscMsg::Telemetry(t) => Some(t),
            _ => None,
        })
        .collect();
    assert!(escs.iter().all(|e| e.telemetry_sent > 0));
    let t = telem.last().expect("telemetry");
    assert!(t.escs_powered && t.thrusters_moving && t.disabled_flags == 0);
    assert!((15_950..=16_050).contains(&t.vcc_mv), "{}", t.vcc_mv);
    // A control loop tick per 400 us frame
    assert!((1200..=1260).contains(&t.tick_cnt), "{}", t.tick_cnt);
    for (i, e) in t.escs.iter().enumerate() {
        assert!(e.esc_online && e.thruster_ready, "{e:?}");
        assert_eq!((e.temperature_c, e.voltage_cv), (35, 1600));
        assert!(e.uart_success_cnt > 0 && e.uart_missed_cnt == 0 && e.dshot_missed_cnt == 0, "{e:?}");
        // Telemetry RPM is signed by the commanded direction and agrees with the eRPM feedback
        assert!((e.rpm as f64 - escs[i].rpm).abs() < escs[i].rpm.abs() * 0.05 + 100.0, "{e:?}");
    }
}

#[test]
fn actuator_board_dynamixel_round_trip() {
    use sim_devices::dynamixel::Dynamixel;
    use titan_link::actuator::{Command, DeviceToHost as Act, HostToDevice as ActCmd};
    let Some(elf) = common::titan_elf("actuator_board") else { return };
    let (mut s, _inputs, bridge) = connected(&elf, "mk2/actuator_board", 5);
    s.run_until(Time::from_ms(3000));
    assert!(s.logged("ROS connected"));
    let statuses = |bridge: &MockBridge| -> Vec<titan_link::actuator::DynamixelStatus> {
        bridge
            .state()
            .received
            .iter()
            .filter_map(|(_, m)| match m {
                titan_link::DeviceToHost::Actuator(Act::Dynamixel(d)) => Some(*d),
                _ => None,
            })
            .collect()
    };
    // Both servos were pinged, identified and read back: state/actuator/dynamixel carries their registers
    let st = statuses(&bridge);
    for id in [2, 3] {
        let d = st.iter().rev().find(|d| d.id == id).unwrap_or_else(|| panic!("no status for ID {id}: {st:?}"));
        assert_eq!((d.model, d.fw_version, d.voltage, d.temperature), (1060, 46, 120, 30));
        assert!(!d.torque_enable, "torque disabled on connect");
        assert_eq!(d.position, if id == 2 { 2048 } else { 6144 });
    }
    let servos: Vec<&Dynamixel> = s.machine.sys.pin_devices.iter().filter_map(|d| d.as_any().downcast_ref()).collect();
    assert!(servos.iter().all(|d| d.crc_errors == 0 && d.received.contains(&0x01) && d.received.contains(&0x02)));
    // Arm, then open the claw: the board writes torque and a goal position to ID 3, which moves there
    bridge.send(HostToDevice::Actuator(ActCmd::Command { seq: 6, command: Command::Arm }));
    s.run_until(Time::from_ms(3300));
    bridge.send(HostToDevice::Actuator(ActCmd::Command { seq: 7, command: Command::Claw { open: true } }));
    s.run_until(Time::from_ms(6000));
    let result = bridge.state().received.iter().find_map(|(_, m)| match m {
        titan_link::DeviceToHost::Actuator(Act::CommandResult { seq: 7, result }) => Some(*result),
        _ => None,
    });
    assert_eq!(result, Some(Ok(())));
    let claw = s
        .machine
        .sys
        .pin_devices
        .iter()
        .filter_map(|d| d.as_any().downcast_ref::<Dynamixel>())
        .find(|d| d.id() == 3)
        .unwrap();
    assert!(claw.torque_enabled());
    assert_eq!(claw.goal_position(), 12288, "CLAW_OPEN_POSITION");
    assert_eq!(claw.present_position(), claw.goal_position(), "reached the goal");
    let st = statuses(&bridge);
    let last = st.iter().rev().find(|d| d.id == 3).unwrap();
    assert!(last.torque_enable);
    assert_eq!(last.position, claw.goal_position(), "the reported position comes from the servo");
}

#[test]
fn smart_battery_gauge_topics() {
    use sim_devices::bq40z80::Bq40z80;
    use sim_devices::i2c_pins::I2cPinTarget;
    use titan_link::smart_battery::{Detect, DeviceToHost as Sb};
    let Some(elf) = common::titan_elf("smart_battery") else { return };
    // Client ID = serial number 21 from flash; the external (classic CAN) bus
    let (mut s, inputs, bridge) = connected(&elf, "mk2/sbh_mcu", 21);
    s.run_until(Time::from_ms(3500));
    assert!(s.logged("Connected to gauge, serial 21"));
    assert!(s.logged("ROS connected"));
    let status = |bridge: &MockBridge| {
        bridge
            .state()
            .received
            .iter()
            .filter_map(|(_, m)| match m {
                titan_link::DeviceToHost::SmartBattery(Sb::BatteryStatus(b)) => Some(b.clone()),
                _ => None,
            })
            .collect::<Vec<_>>()
    };
    let st = status(&bridge);
    assert!(st.len() >= 2, "{st:?}");
    let b = st.last().unwrap();
    assert_eq!(
        (b.serial, b.detect, b.pack_voltage_mv, b.pack_current_ma, b.soc),
        (21, Detect::Port, 16_400, -2_500, 80)
    );
    // 8000 mAh left at 2.5 A: 192 minutes
    assert_eq!(b.time_to_dischg, 192);
    let th = bridge.state().received.iter().rev().find_map(|(_, m)| match m {
        titan_link::DeviceToHost::SmartBattery(Sb::TempHumidity { temperature_c, .. }) => Some(*temperature_c),
        _ => None,
    });
    assert!((th.expect("temperature") - 24.0).abs() < 0.01);
    // Scripted discharge: the pack sags and the readings follow
    inputs.set("gauge.voltage_mv", 15_100.0);
    inputs.set("gauge.soc_percent", 35.0);
    s.run_until(Time::from_ms(6000));
    let b = status(&bridge).last().unwrap().clone();
    assert_eq!((b.pack_voltage_mv, b.soc), (15_100, 35));
    let target = s.machine.sys.pin_device::<I2cPinTarget>().unwrap();
    let gauge = target.dev.as_any().downcast_ref::<Bq40z80>().unwrap();
    assert_eq!(gauge.pec_errors, 0);
    assert!(gauge.reads > 50, "{}", gauge.reads);
}

#[test]
fn poac_board_torpedo_servo() {
    use sim_devices::hiwonder::Hiwonder;
    use titan_link::poac::{DeviceToHost as Poac, HostToDevice as PoacCmd};
    let Some(elf) = common::titan_elf("poac_board") else { return };
    let (mut s, _inputs, bridge) = connected(&elf, "talos/poac_board", 1);
    s.run_until(Time::from_ms(2500));
    assert!(s.logged("ROS connected"));
    let statuses = |bridge: &MockBridge| -> Vec<titan_link::poac::ActuatorStatus> {
        bridge
            .state()
            .received
            .iter()
            .filter_map(|(_, m)| match m {
                titan_link::DeviceToHost::Poac(Poac::Status { status, .. }) => Some(*status),
                _ => None,
            })
            .collect()
    };
    let before = statuses(&bridge).last().copied().expect("actuator status every 500 ms");
    let servo = s.machine.sys.pin_device::<Hiwonder>().unwrap();
    assert!(servo.commands.len() > 2, "pinged and homed: {:?}", servo.commands);
    assert_eq!(servo.checksum_errors, 0);
    let home = servo.target();
    // Arm and fire a torpedo: the servo is sent to a firing position and the count goes down
    bridge.send(HostToDevice::Poac(PoacCmd::Arm(true)));
    s.run_until(Time::from_ms(2700));
    bridge.send(HostToDevice::Poac(PoacCmd::FireTorpedo));
    s.run_until(Time::from_ms(3200));
    let servo = s.machine.sys.pin_device::<Hiwonder>().unwrap();
    assert_ne!(servo.target(), home, "{:?}", servo.commands);
    assert!(servo.commands.contains(&1), "MOVE_TIME_WRITE");
    s.run_until(Time::from_ms(6000));
    let after = statuses(&bridge).last().copied().unwrap();
    assert_eq!(after.torpedo_available_count + 1, before.torpedo_available_count, "{before:?} -> {after:?}");
    let feedback = bridge.state().received.iter().rev().find_map(|(_, m)| match m {
        titan_link::DeviceToHost::Poac(Poac::CommandFeedback(f)) => Some(*f),
        _ => None,
    });
    assert_eq!(feedback, Some(None), "the last command succeeded");
}
