//! Talos with its thrusters in the loop: the two ESC boards' RPM controllers spin the motors to the commanded
//! speeds (from their eRPM feedback, turning the counter-rotating thrusters backwards), the vehicle dives and turns,
//! and the camera cage's depth sensor reports the dive. (About 40 s of wall time: the ESC boards
//! hold neutral for their first 5 s, and each runs slower than real time.)

use sim_test::titan_link::{self, DeviceToHost, HostToDevice};
use sim_test::{MockBridge, RobotBench, Time};

#[test]
fn talos_dives_and_turns_on_esc_board_rpm_commands() {
    for elf in ["esc_board", "camera_cage_bb"] {
        if sim_test::titan_rs(elf).is_none() {
            return;
        }
    }
    // Only the boards the vehicle needs: the two ESC boards (clients 2 and 3) and the camera cage (client 4)
    let mut r = RobotBench::load("talos.toml", &[("internal", 2)], |spec| {
        spec.board.retain(|b| ["esc0", "esc1", "camera_cage"].contains(&b.name.as_str()));
    })
    .unwrap();
    let mut bridges = vec![r.bridges[0].clone().unwrap()];
    for client in [3, 4] {
        let b = MockBridge::new(client, true, Time::ZERO);
        b.set_kill(Some(false));
        r.robot.buses[0].endpoints.push(Box::new(b.clone()));
        bridges.push(b);
    }
    let port = [0.30, 0.0, 0.10];
    let zero = r.robot.vehicle("talos").unwrap().vehicle.point(port)[2];
    // RPM targets, as titan_bridge sends them: the same command to both boards (each takes its four). The
    // starboard pair ahead and the port pair astern (turning to port), the vertical ones down. The boards hold
    // neutral for 5 s after power-up, so the motion starts then.
    let cmd = [800, -800, 800, -800, 1200, 1200, 1200, 1200];
    let mut still = None;
    while r.now() < Time::from_ms(7500) {
        for b in &bridges[..2] {
            b.send(HostToDevice::Esc(titan_link::esc::HostToDevice::ThrusterRpm(cmd)));
        }
        r.run_for(Time::from_ms(100));
        if r.now() == Time::from_ms(4900) {
            still = Some(r.robot.vehicle("talos").unwrap().vehicle.state);
        }
    }
    let v = r.robot.vehicle("talos").unwrap().vehicle.clone();
    // Afloat and still until the ESC boards let the thrusters go
    let still = still.unwrap();
    assert!((still.position[2] - zero + 0.10).abs() < 1e-6 && still.attitude.euler()[2] == 0.0, "{still:?}");
    // Each motor at its target (thrusters 1 and 4 of each board turning the other way), so each thruster makes the
    // thrust of its target speed, ahead or astern
    let rpm = r.robot.vehicles[0].rpms(&r.robot.boards, r.now());
    let want = [-800.0, -800.0, 800.0, 800.0, -1200.0, 1200.0, 1200.0, -1200.0];
    for (got, want) in rpm.iter().zip(want) {
        assert!((got - want).abs() < 10.0, "{rpm:?}");
    }
    let t200 = |rpm: f64, k: f64| k * (rpm / 3800.0).powi(2);
    let ahead = t200(800.0, 51.5);
    let astern = -t200(800.0, 40.2);
    let down = t200(1200.0, 51.5);
    let expected = [ahead, astern, ahead, astern, down, down, down, down];
    for (got, want) in v.thrust_n.iter().zip(expected) {
        assert!((got - want).abs() < 0.03 * want.abs(), "{:?} vs {expected:?}", v.thrust_n);
    }
    // Turning to port at the rate where the yaw drag (3 r + 5 r^2) takes the couple, 0.403 m per newton
    let couple = 0.403 * (2.0 * ahead - 2.0 * astern);
    let rate = (-3.0 + (9.0f64 + 20.0 * couple).sqrt()) / 10.0;
    assert!((v.state.rate[2] + rate).abs() < 0.02, "{} vs {}", v.state.rate[2], -rate);
    let [roll, pitch, yaw] = v.state.attitude.euler().map(f64::to_degrees);
    assert!(yaw < -40.0, "{yaw}");
    assert!(roll.abs() < 1.0 && pitch.abs() < 1.0, "level: {roll} {pitch}");
    // Diving: the camera cage reports the depth of its sensor port below where it zeroed, as the vehicle sees it
    let depths: Vec<(Time, f64)> = bridges[2]
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::CameraCage(titan_link::camera_cage::DeviceToHost::Depth(d)) => Some((*t, -*d as f64)),
            _ => None,
        })
        .collect();
    assert!(depths.iter().filter(|(t, _)| *t < Time::from_secs(5)).all(|(_, d)| d.abs() < 0.01), "{depths:?}");
    let (t, last) = *depths.last().unwrap();
    assert!(r.now() - t <= Time::from_ms(60), "a reading every 50 ms");
    let truth = v.point(port)[2] - zero;
    assert!(last > 0.3 && (last - truth).abs() < 0.03, "reported {last} vs {truth}");
}
