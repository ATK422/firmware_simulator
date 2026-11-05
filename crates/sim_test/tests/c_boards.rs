//! titan_firmware's C board apps (your build: `$TITAN_FIRMWARE_DIR`, each project's `build/NAME_with_bl.uf2` as
//! `scripts/build_c_boards.sh` writes them) on the robot files that run them. Skipped, passing, while
//! `TITAN_FIRMWARE_DIR` is unset or an app is not built.
//!
//! Each board boots through the bootrom and the CAN bootloader (whose own heartbeat goes out first) into its app,
//! which sends its CANmore heartbeat every 500 ms whether or not a micro-ROS agent answers; there is no agent here,
//! so nothing past "waiting for the agent" is checked. Checked with titan_firmware 058df2a.

use sim_core::bus::Size;
use sim_core::image::Image;
use sim_devices::can::{CanEndpoint, CanFrame};
use sim_test::{RobotBench, Time};

/// The CANmore heartbeat's standard identifier for a client (`client << 6 | 0x2F`).
fn heartbeat_id(client: u32) -> u32 {
    (client << 6) | 0x2F
}

/// Times of the frames with identifier `id` on bus `bus` (every attempt, acknowledged or not).
fn sent(r: &RobotBench, bus: usize, id: u32) -> Vec<f64> {
    let rec = r.robot.buses[bus].bus.record.as_ref().expect("recording");
    rec.iter().filter(|f| f.frame.id == id && !f.frame.extended).map(|f| f.start.as_secs_f64()).collect()
}

/// The images `robots/<file>` takes from `$TITAN_FIRMWARE_DIR`, if all are built.
fn c_images(projects: &[&str]) -> bool {
    projects.iter().all(|p| sim_test::titan_firmware(p).is_some())
}

/// `robots/talos_c.toml`: every board's bootloader and app start, and each app's heartbeat is on its bus from
/// then on, with no reset. The SBC board is the exception (below).
#[test]
fn talos_c_boards_boot_through_their_bootloader_and_send_heartbeats() {
    if !c_images(&[
        "talos/PoacBoard/build/poac_board_with_bl.uf2",
        "talos/ESCBoard/build/esc_board_with_bl.uf2",
        "talos/CameraCageBB/build/camera_cage_bb_with_bl.uf2",
        "talos/Actuator/build/actuator_mk2_with_bl.uf2",
        "talos/LEDBoard/build/led_board_with_bl.uf2",
        "SBCBoard/build/sbc_board_with_bl.uf2",
        "talos/SmartBattery/build/smart_battery_with_bl.uf2",
    ]) {
        return;
    }
    let mut r = RobotBench::load("talos_c.toml", &[], |_| {}).unwrap();
    for b in &mut r.robot.buses {
        b.bus.record = Some(Vec::new());
    }
    r.run_until(Time::from_ms(3000));
    // (board, its board header's name in the build tag, client, bus)
    let boards = [
        ("poac", "talos/poac_board", 1, 0),
        ("esc0", "mk2/esc_board", 2, 0),
        ("esc1", "mk2/esc_board", 3, 0),
        ("camera_cage", "mk2/camera_cage_bb", 4, 0),
        ("actuator", "mk2/actuator_board", 5, 0),
        ("led", "talos/led_board", 6, 0),
        ("sbc", "talos/sbc_board", 7, 0),
        ("battery", "mk2/sbh_mcu", 21, 1),
    ];
    for (name, header, client, bus) in boards {
        let s = r.board(name);
        let log = &s.uart_text[0];
        assert!(log.contains(&format!("titan_bootloader 1.0-talos/{header}")), "{name}: {log}");
        assert!(log.contains(&format!("1.0-talos/{header} ")) && log.contains("Clean Boot: Power-on Reset"), "{log}");
        assert_eq!(s.machine.resets, 0, "{name}");
        let beats = sent(&r, bus, heartbeat_id(client));
        // The bootloader's heartbeat first, ~80 ms after power-on
        assert!(beats.first().is_some_and(|t| *t < 0.2), "{name}: {beats:?}");
        let app: Vec<f64> = beats.iter().copied().filter(|t| (1.0..3.0).contains(t)).collect();
        if name == "sbc" {
            // Firmware finding: SBCBoard's app (058df2a, work in progress) calls canbus_init but never canbus_tick,
            // which sends the heartbeat, so the board falls silent once its bootloader hands over
            assert!(app.is_empty(), "sbc: {beats:?}");
            assert!(log.contains("Smart Battery Charger Board FW"), "{log}");
        } else if bus == 1 {
            // Alone on the external bus (the Orin's agent is not there to ACK), the smart battery's controller
            // retransmits each heartbeat until its attempts run out: three tries every 500 ms
            assert!(app.len() >= 4 * 3, "{name}: {app:?}");
        } else {
            assert!(app.len() >= 4, "{name}: {app:?}");
        }
    }
    // Every heartbeat on the internal bus but the first few (while the others' controllers come up) is acknowledged
    let rec = r.robot.buses[0].bus.record.as_ref().unwrap();
    assert!(rec.iter().filter(|f| f.start.as_secs_f64() > 1.0).all(|f| f.acked && f.error.is_none()));
}

/// `robots/mercury.toml`'s boards: the raw CANmore thruster command (channel 0, four big-endian RPMs, what
/// riptide's controller writes to can0) reaches the ESC board's `target_req.rpm` (core 1's target speeds) only with
/// each RPM's high byte twice on Mercury: `mercury/ESCBoard/src/main.c` builds each value as
/// `(buf[idx] << 8) | buf[idx]`, a bug titan_firmware PR #26 fixed in `talos/ESCBoard` only (`buf[idx + 1]`). The
/// Talos board decodes the same frame right. (What core 1 then sends as DShot needs ROS: without an agent the
/// boards stay killed and unconfigured.)
#[test]
fn firmware_finding_mercury_esc_board_reads_each_rpm_high_byte_twice() {
    let rpm: [i16; 4] = [1200, -1200, 300, 0x1234];
    for (robot, project, elf) in [
        ("mercury.toml", "mercury/ESCBoard/build/esc_board_with_bl.uf2", "mercury/ESCBoard/build/esc_board.elf"),
        ("talos_c.toml", "talos/ESCBoard/build/esc_board_with_bl.uf2", "talos/ESCBoard/build/esc_board.elf"),
    ] {
        let (Some(_), Some(elf)) = (sim_test::titan_firmware(project), sim_test::titan_firmware(elf)) else {
            return;
        };
        let target_req = Image::load(&elf).unwrap().symbol("target_req").expect("target_req").addr;
        let mut r = RobotBench::load(robot, &[], |spec| {
            spec.board.retain(|b| b.name == "esc0");
            spec.vehicle.clear();
        })
        .unwrap();
        r.robot.buses[0].endpoints.push(Box::new(Controller { next: Time::from_ms(1000), client: 2, rpm }));
        r.run_until(Time::from_ms(1500));
        let s = r.board("esc0");
        // struct core1_cmd_shared_mem { spin_lock_t *lock; int16_t rpm[4]; absolute_time_t rpm_expiration; }
        let got: Vec<i16> =
            (0..4).map(|k| s.machine.sys.peek(target_req + 4 + 2 * k, Size::Half).unwrap() as u16 as i16).collect();
        let want: Vec<i16> = if robot == "mercury.toml" {
            rpm.iter().map(|r| ((r.to_be_bytes()[0] as u16) << 8 | r.to_be_bytes()[0] as u16) as i16).collect()
        } else {
            rpm.to_vec()
        };
        assert_eq!(got, want, "{robot}");
        if robot == "mercury.toml" {
            assert_eq!(got, [0x0404, 0xfbfb_u16 as i16, 0x0101, 0x1212]);
        }
    }
}

/// riptide's controller writing raw CANmore thruster commands to one ESC board every 20 ms
/// (`send_thruster_cmd_canbus.c`: utility frame, agent to client, channel 0).
struct Controller {
    next: Time,
    client: u32,
    rpm: [i16; 4],
}

impl CanEndpoint for Controller {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut out = Vec::new();
        while self.next <= until {
            let data: Vec<u8> = self.rpm.iter().flat_map(|r| r.to_be_bytes()).collect();
            let id = canmore::id::std_id(
                self.client as u8,
                canmore::FrameType::Util,
                canmore::Direction::AgentToClient,
                canmore::CHAN_THRUSTER_CMDS,
            );
            out.push((self.next, CanFrame::classic(id, false, &data)));
            self.next += Time::from_ms(20);
        }
        out
    }

    fn receive(&mut self, _: &CanFrame, _: Time) {}
}
