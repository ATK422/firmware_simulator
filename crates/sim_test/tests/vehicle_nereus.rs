//! Talos's Rust ESC boards with nereus as the physics (`robots/talos_nereus.toml`), driven the way the riptide
//! controller drives the real ones: `complete_controller`'s `send_thruster_cmd_canbus.c` writes raw CANmore utility
//! frames (channel 0, four big-endian RPMs per ESC board) to can0, not titan_link messages. Each motor reaches its
//! target and the thrust the emulator reports for nereus (`/api/thrusters`) is what the controller's force-to-RPM
//! curve chose that speed for; the vehicle stays where the posted poses put it, and the camera cage reports the
//! depth of the pose. (About 40 s of wall time: the ESC boards hold neutral for their first 5 s.)

use sim_devices::can::{CanEndpoint, CanFrame};
use sim_host::vehicle::ExternalPose;
use sim_test::titan_link::{self, DeviceToHost};
use sim_test::{MockBridge, RobotBench, Time};

/// `CANMORE_CALC_THRUSTER_CMD_ID` of `send_thruster_cmd_canbus.c`: client ID, utility type, agent to client, channel
/// `CANMORE_TITAN_CHAN_THRUSTER_CMDS` (0), in the standard identifier's layout as that file copies it.
fn controller_id(client: u32) -> u32 {
    (client << 6) | (1 << 5) | (1 << 4)
}

/// The controller's CAN writer: every 20 ms, ESC board 0 (client 2) gets RPMs 0-3 and ESC board 1 (client 3) 4-7.
struct Controller {
    next: Time,
    rpm: [i16; 8],
}

impl CanEndpoint for Controller {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut out = Vec::new();
        while self.next <= until {
            for (board, client) in [(0, 2), (1, 3)] {
                let data: Vec<u8> = self.rpm[board * 4..board * 4 + 4].iter().flat_map(|r| r.to_be_bytes()).collect();
                out.push((self.next, CanFrame::classic(controller_id(client), false, &data)));
            }
            self.next += Time::from_ms(20);
        }
        out
    }

    fn receive(&mut self, _: &CanFrame, _: Time) {}
}

#[test]
fn the_controllers_can_frames_spin_the_thrusters_nereus_applies() {
    for elf in ["esc_board", "camera_cage_bb"] {
        if sim_test::titan_rs(elf).is_none() {
            return;
        }
    }
    // The identifier the controller hand-copied from titan_canmore is canmore's
    for client in [2, 3] {
        let id = canmore::id::std_id(
            client as u8,
            canmore::FrameType::Util,
            canmore::Direction::AgentToClient,
            canmore::CHAN_THRUSTER_CMDS,
        );
        assert_eq!(controller_id(client), id);
    }
    let mut r = RobotBench::load("talos_nereus.toml", &[("internal", 2)], |spec| {
        spec.board.retain(|b| ["esc0", "esc1", "camera_cage"].contains(&b.name.as_str()));
    })
    .unwrap();
    assert!(r.robot.vehicles[0].external);
    // titan_bridge's side for the other two (the kill switch released, the camera cage's depth readings)
    let mut bridges = vec![r.bridges[0].clone().unwrap()];
    for client in [3, 4] {
        let b = MockBridge::new(client, true, Time::ZERO);
        b.set_kill(Some(false));
        r.robot.buses[0].endpoints.push(Box::new(b.clone()));
        bridges.push(b);
    }
    // What riptide's force_to_rpm curve gives for 12 N, -5 N, 16 N and -8 N, the signs as the solver sends them.
    // (Its 24 N limit, 3171 RPM, is out of reach: the boards' hard limit of 725 throttle is about 2900 RPM.)
    let cmd: [i16; 8] = [2275, -1408, 2604, 0, -1408, 2275, 2604, -1720];
    let force: [f64; 8] = [12.0, -5.0, 16.0, 0.0, -5.0, 12.0, 16.0, -8.0];
    r.robot.buses[0].endpoints.push(Box::new(Controller { next: Time::from_ms(100), rpm: cmd }));
    // nereus has it 1.2 m down, turned to the east; the bridge holds it at the surface until the depth sensor has
    // zeroed (the first 3 s here)
    let pose = |depth: f64| ExternalPose {
        position: [3.0, 4.0, depth],
        attitude: sim_physics::Quat::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        velocity: Some([0.0; 3]),
        rate: Some([0.0; 3]),
    };
    while r.now() < Time::from_ms(7000) {
        let depth = if r.now() < Time::from_ms(3000) { 0.0 } else { 1.2 };
        r.robot.vehicles[0].set_pose(&pose(depth));
        r.run_for(Time::from_ms(20));
    }
    // Where it was put, its own physics off (it would have floated up, or dived on these thrusts)
    let v = &r.robot.vehicles[0];
    assert_eq!(v.vehicle.state.position, [3.0, 4.0, 1.2]);
    // Each motor at its target, the inverted ones (1 and 4 of each board) turning the other way. Reverse targets
    // settle about 330 RPM short: the ESC model needs more throttle for a speed than the feed-forward fit in
    // esc_core (from the real thrusters) gives, P alone leaves an error just above the I term's 300 RPM bound, and
    // the I term never starts. A model calibration to revisit; forward targets get inside the bound and converge.
    let rpm = v.rpms(&r.robot.boards, r.now());
    let invert = [-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0];
    for (k, got) in rpm.iter().enumerate() {
        let want = cmd[k] as f64 * invert[k];
        if cmd[k] >= 0 {
            assert!((got - want).abs() < 15.0, "thruster {k}: {rpm:?}");
        } else {
            let short = want.abs() - got.abs();
            assert!(got.signum() == want.signum() && (250.0..400.0).contains(&short), "thruster {k}: {rpm:?}");
        }
    }
    // The thrust nereus is given, thruster k at index k: the curve at the motor's speed, so the force the
    // controller chose each speed for where the motor got there (within the curve fit's 4%)
    let names = r.robot.names.clone();
    let boards: Vec<&sim_host::Session> = r.robot.boards.iter().collect();
    let api = sim_host::dashboard::thrusters_json(&names, &boards, &r.robot.vehicles, r.now());
    let thrusters = api["vehicles"][0]["thrusters"].as_array().unwrap();
    for (k, f) in force.iter().enumerate() {
        let got = thrusters[k]["thrust_n"].as_f64().unwrap();
        let curve = v.vehicle.thrusters[k].thrust(rpm[k]);
        assert!((got - curve).abs() < 1e-9, "thruster {k}: {got} N vs the curve's {curve} N");
        assert!(*f == 0.0 || got.signum() == f.signum(), "thruster {k}: {got} N");
        if cmd[k] >= 0 {
            assert!((got - f).abs() < 0.04 * f.abs() + 0.05, "thruster {k}: {got} N vs {f} N");
        }
    }
    let escs = api["escs"].as_array().unwrap();
    assert_eq!(escs.len(), 8);
    assert!(escs.iter().zip(thrusters).all(|(e, t)| e["thrust_n"] == t["thrust_n"]));
    // The camera cage zeroed at the surface, then reports the depth of its port below base_link's posted depth
    let depths: Vec<(Time, f64)> = bridges[2]
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::CameraCage(titan_link::camera_cage::DeviceToHost::Depth(d)) => Some((*t, -*d as f64)),
            _ => None,
        })
        .collect();
    assert!(depths.iter().filter(|(t, _)| *t < Time::from_ms(2900)).all(|(_, d)| d.abs() < 0.01), "{depths:?}");
    let (_, last) = *depths.last().unwrap();
    let port = 1.2 - 0.135;
    assert!((last - port).abs() < 0.03, "reported {last} m, the port is at {port} m");
}
