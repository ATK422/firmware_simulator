//! M4: boards with their sensors. Each test boots a titan_firmware_rs board with the devices from its board file,
//! sets the physical inputs, and checks what reaches the (mock) bridge: the values `titan_bridge` would publish on
//! the board's ROS topics.

mod common;

use sim_core::time::Time;
use sim_devices::can::CanBus;
use sim_host::Session;
use sim_host::board::{Board, Inputs};
use sim_host::mock_bridge::MockBridge;
use titan_link::{DeviceToHost, power};

fn board(name: &str) -> Board {
    Board::load(&common::repo().join("boards").join(format!("{name}.toml"))).unwrap()
}

/// Boots `elf` on `board_name` with a connected, enabling mock bridge for `client`.
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

fn power_messages(bridge: &MockBridge) -> Vec<(Time, power::DeviceToHost)> {
    bridge
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::Power(p) => Some((*t, p.clone())),
            _ => None,
        })
        .collect()
}

fn close(a: f32, b: f64, tol: f64) -> bool {
    (a as f64 - b).abs() <= tol
}

#[test]
fn power_board_electrical_and_environment() {
    let Some(elf) = common::titan_elf("power_board") else { return };
    let (mut s, inputs, bridge) = connected(&elf, "mk2/power_board", 1);
    s.run_until(Time::from_ms(3200));
    assert!(s.logged("ROS connected"));
    assert!(!s.logged("failed"), "{:?}", s.logs.iter().filter(|l| l.1.message.contains("failed")).collect::<Vec<_>>());
    let msgs = power_messages(&bridge);

    // state/temp/powerboard and state/humidity/powerboard: every SHT41 reading (500 ms)
    let th: Vec<_> = msgs
        .iter()
        .filter_map(|(t, m)| match m {
            power::DeviceToHost::TempHumidity { temperature_c, humidity_rh } => {
                Some((*t, *temperature_c, *humidity_rh))
            }
            _ => None,
        })
        .collect();
    assert!(th.len() >= 5, "{th:?}");
    let (_, t, h) = th.last().unwrap();
    assert!(close(*t, 30.0, 0.01) && close(*h, 35.0, 0.01), "{t} {h}");

    // state/electrical every second: batteries from the RP2040 ADC, rails from the MCP3426
    let el: Vec<_> = msgs
        .iter()
        .filter_map(|(t, m)| match m {
            power::DeviceToHost::Electrical(r) => Some((*t, *r)),
            _ => None,
        })
        .collect();
    assert!(el.len() >= 2, "{el:?}");
    let r = &el.last().unwrap().1;
    // 12-bit ADC behind 10k/1k: one code is 8.9 mV at the battery
    assert!(close(r.port_voltage, 16.2, 0.02) && close(r.stbd_voltage, 16.4, 0.02), "{r:?}");
    assert!(close(r.three_volt_voltage, 3.3, 0.001), "{r:?}");
    assert!(close(r.balanced_voltage, 16.0, 0.002), "{r:?}");
    // The MCP3426 has two channels; selections 3 and 4 convert channels 1 and 2 again, so the "12 V" and "5 V"
    // rails read the 3.3 V and balanced taps through the 10k/1k scaling (see the M4 PR)
    assert!(close(r.twelve_volt_voltage, 1.65 * 11.0, 0.01), "{r:?}");
    assert!(close(r.five_volt_voltage, 16.0, 0.01), "{r:?}");

    // Scripted change: the port battery sags and the board temperature rises
    inputs.set("port_battery", 14.0);
    inputs.set("sht41.temperature_c", 45.5);
    s.run_until(Time::from_ms(4500));
    let msgs = power_messages(&bridge);
    let last_el = msgs.iter().rev().find_map(|(_, m)| match m {
        power::DeviceToHost::Electrical(r) => Some(*r),
        _ => None,
    });
    assert!(close(last_el.unwrap().port_voltage, 14.0, 0.02));
    let last_t = msgs.iter().rev().find_map(|(_, m)| match m {
        power::DeviceToHost::TempHumidity { temperature_c, .. } => Some(*temperature_c),
        _ => None,
    });
    assert!(close(last_t.unwrap(), 45.5, 0.01));
    // state/kill: not killed (the bridge enables the vehicle), every 150 ms
    let kills: Vec<_> =
        msgs.iter().filter_map(|(t, m)| matches!(m, power::DeviceToHost::KillState(_)).then_some(*t)).collect();
    assert!(kills.len() > 20);
    assert!(matches!(
        msgs.iter().rev().find(|m| matches!(m.1, power::DeviceToHost::KillState(_))).unwrap().1,
        power::DeviceToHost::KillState(false)
    ));
}

/// Board messages the bridge received, picked out by `f`.
fn picked<T>(bridge: &MockBridge, f: impl Fn(&DeviceToHost) -> Option<T>) -> Vec<(Time, T)> {
    bridge.state().received.iter().filter_map(|(t, m)| f(m).map(|v| (*t, v))).collect()
}

#[test]
fn camera_cage_depth_and_environment() {
    use titan_link::camera_cage::DeviceToHost as Cage;
    let Some(elf) = common::titan_elf("camera_cage_bb") else { return };
    let (mut s, inputs, bridge) = connected(&elf, "mk2/camera_cage_bb", 4);
    // Zeroing takes about 2 s of back-to-back conversions at the surface
    s.run_until(Time::from_ms(3000));
    assert!(s.logged("ROS connected"));
    let surface = inputs.get("depth.pressure_pa");
    // Dive to 2.5 m in fresh water (the firmware's density)
    inputs.set("depth.pressure_pa", surface + 997.0 * 9.80665 * 2.5);
    inputs.set("depth.temperature_c", 12.5);
    s.run_until(Time::from_ms(4500));
    let depth = picked(&bridge, |m| match m {
        DeviceToHost::CameraCage(Cage::Depth(d)) => Some(*d),
        _ => None,
    });
    // A reading every 50 ms once zeroed; at the surface first, then -2.5 m (negative below the surface)
    let at_surface: Vec<_> = depth.iter().filter(|(t, _)| *t < Time::from_ms(3000)).collect();
    assert!(!at_surface.is_empty(), "{depth:?}");
    assert!(at_surface.iter().all(|(_, d)| d.abs() < 0.01), "{at_surface:?}");
    let (_, last) = depth.last().unwrap();
    assert!(close(*last, -2.5, 0.01), "{last}");
    let n_after = depth.iter().filter(|(t, _)| *t > Time::from_ms(3500)).count();
    assert!((18..=21).contains(&n_after), "50 ms polling: {n_after} in 1 s");
    let water = picked(&bridge, |m| match m {
        DeviceToHost::CameraCage(Cage::WaterTemp(t)) => Some(*t),
        _ => None,
    });
    assert!(close(water.last().unwrap().1, 12.5, 0.01), "{water:?}");
    let th = picked(&bridge, |m| match m {
        DeviceToHost::CameraCage(Cage::TempHumidity { temperature_c, humidity_rh }) => {
            Some((*temperature_c, *humidity_rh))
        }
        _ => None,
    });
    let (t, h) = th.last().unwrap().1;
    assert!(close(t, 35.0, 0.01) && close(h, 20.0, 0.01));
    assert_eq!(bridge.statuses().last().unwrap().1.faults, 0);
}

#[test]
fn poac_board_power_stage() {
    use titan_link::poac::DeviceToHost as Poac;
    let Some(elf) = common::titan_elf("poac_board") else { return };
    let (mut s, inputs, bridge) = connected(&elf, "talos/poac_board", 1);
    s.run_until(Time::from_ms(2500));
    inputs.set("rail_balanced", 14.8);
    s.run_until(Time::from_ms(5000));
    assert!(s.logged("ROS connected"));
    let el = picked(&bridge, |m| match m {
        DeviceToHost::Poac(Poac::Power(power::DeviceToHost::Electrical(r))) => Some(*r),
        _ => None,
    });
    let r = &el.last().unwrap().1;
    assert!(close(r.three_volt_voltage, 3.3, 0.001) && close(r.balanced_voltage, 14.8, 0.002), "{r:?}");
    let th = picked(&bridge, |m| match m {
        DeviceToHost::Poac(Poac::Power(power::DeviceToHost::TempHumidity { temperature_c, .. })) => {
            Some(*temperature_c)
        }
        _ => None,
    });
    assert!(close(th.last().unwrap().1, 28.0, 0.01));
}

#[test]
fn sbc_board_connects_with_its_bus_configured() {
    let Some(elf) = common::titan_elf("sbc_board") else { return };
    let (mut s, _inputs, bridge) = connected(&elf, "talos/sbc_board", 7);
    s.run_until(Time::from_ms(2500));
    assert!(s.logged("ROS connected"));
    let st = bridge.statuses();
    let last = &st.last().unwrap().1;
    assert_eq!((last.board_name.as_str(), last.client_id, last.faults), ("talos/sbc_board", 7, 0));
    // The board I2C bus is set up (GPIO 28 SDA, 29 SCL on I2C0) though nothing is on it yet
    let gpio = &s.machine.sys.gpio;
    assert_eq!((gpio.funcsel(28), gpio.funcsel(29)), (3, 3));
    // 200 kHz asked for; the DW controller adds SPKLEN + 7 cycles to the high time and 1 to the low time
    let khz = s.machine.sys.i2c[0].scl_hz(s.machine.sys.freqs.clk_sys) / 1000;
    assert!((185..=200).contains(&khz), "{khz} kHz");
}

#[test]
fn smart_battery_environment_and_display() {
    use sim_core::bus::Size;
    use sim_devices::sht41::Sht41;
    use sim_devices::ssd1306::Ssd1306;
    let Some(elf) = common::titan_elf("smart_battery") else { return };
    let b = board("mk2/sbh_mcu");
    let mut s = common::boot(&elf, None);
    b.apply(&mut s.machine).unwrap();
    // The serial number (and CANmore client ID) lives in the last flash sector
    assert!(s.machine.sys.poke(0x101F_F000, Size::Byte, 42));
    s.run_until(Time::from_ms(3000));
    assert!(s.logged("Board serial number 42"), "{:?}", s.logs.iter().take(10).collect::<Vec<_>>());
    // Without the gauge (on the PIO SMBus, M5) the board publishes nothing on ROS, but its I2C side runs: the SHT41
    // is polled every 500 ms and the OLED is set up as a 128x32 panel and drawn
    let sht = s.machine.sys.i2c_device::<Sht41>().unwrap();
    assert!((5..=7).contains(&sht.measurements), "{}", sht.measurements);
    let oled = s.machine.sys.i2c_device::<Ssd1306>().unwrap();
    assert!(oled.display_on && oled.charge_pump);
    assert_eq!((oled.mux_ratio, oled.addressing_mode), (32, 0));
    assert!(oled.data_bytes >= 512, "{}", oled.data_bytes);
    assert!(oled.frame().iter().flatten().filter(|&&p| p).count() > 200, "{}", oled.render_text());
    // Firmware observation (see the M4 PR): the driver selects horizontal addressing, then positions each page with
    // the page-mode commands B0h/00h/10h, which the datasheet says only work in page mode. Each update is 512 bytes
    // (4 pages), so the write pointer alternates between pages 0 and 4 and every other frame lands in RAM the 32-line
    // panel does not show.
    let updates = oled.data_bytes / 512;
    assert_eq!(oled.ignored_page_commands, 3 + 12 * updates, "3 in init, 3 per page per update");
    assert_eq!(oled.page, if updates.is_multiple_of(2) { 0 } else { 4 });
}
