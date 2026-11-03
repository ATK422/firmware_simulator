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

/// Snapshots of a robot whose vehicle drives a sensor input each get their own input, so a sweep runs its points
/// side by side: three copies of Talos, each held at another depth, and each camera cage reports its own.
#[test]
fn a_parallel_sweep_over_vehicle_depths() {
    if sim_test::titan_rs("camera_cage_bb").is_none() {
        return;
    }
    let mut base = RobotBench::load("talos.toml", &[("internal", 4)], |spec| {
        spec.board.retain(|b| b.name == "camera_cage");
        spec.vehicle[0].thrusters.clear();
    })
    .unwrap();
    // Trimmed neutral, so it stays where it is put
    let v = &mut base.robot.vehicle("talos").unwrap().vehicle;
    v.params.volume_m3 = v.params.mass_kg / v.water.density_kg_m3;
    base.run_until(Time::from_ms(3000));
    let depths = |b: &RobotBench| -> Vec<(Time, f64)> {
        b.picked("internal", |m| match m {
            DeviceToHost::CameraCage(titan_link::camera_cage::DeviceToHost::Depth(d)) => Some(-*d as f64),
            _ => None,
        })
    };
    assert!(depths(&base).last().is_some_and(|d| d.1.abs() < 0.01), "zeroed at the surface");
    let points = [0.5, 1.0, 2.0];
    // The points go in lockstep, 10 ms at a time, so they really run side by side (one at a time, they would
    // never all get to the same step)
    let lockstep = Lockstep::new(points.len());
    let reported = sim_test::sweep(&base, &points, points.len(), |mut b, dive| {
        let v = &mut b.robot.vehicle("talos").unwrap().vehicle;
        v.state.position[2] += dive;
        v.state.velocity = [0.0; 3];
        for _ in 0..100 {
            b.run_for(Time::from_ms(10));
            lockstep.wait(std::time::Duration::from_secs(60));
        }
        let truth = b.robot.vehicle("talos").unwrap().vehicle.state.position[2];
        (depths(&b).last().unwrap().1, truth)
    });
    let start = base.robot.vehicle("talos").unwrap().vehicle.state.position[2];
    for (dive, (got, truth)) in points.iter().zip(reported) {
        assert!((truth - start - dive).abs() < 0.02, "{dive}: the vehicle is at {truth}");
        assert!((got - dive).abs() < 0.03, "{dive} m down, the camera cage reports {got}");
    }
}

/// A barrier that gives up: `n` threads meet at each step, or the one waiting panics after `timeout`.
struct Lockstep {
    n: usize,
    /// Step number and how many have arrived at it.
    state: std::sync::Mutex<(u64, usize)>,
    cv: std::sync::Condvar,
}

impl Lockstep {
    fn new(n: usize) -> Self {
        Lockstep { n, state: std::sync::Mutex::new((0, 0)), cv: std::sync::Condvar::new() }
    }

    fn wait(&self, timeout: std::time::Duration) {
        let mut s = self.state.lock().unwrap();
        let step = s.0;
        s.1 += 1;
        if s.1 == self.n {
            *s = (step + 1, 0);
            self.cv.notify_all();
            return;
        }
        let (s, t) = self.cv.wait_timeout_while(s, timeout, |s| s.0 == step).unwrap();
        assert!(!t.timed_out() || s.0 != step, "the sweep's points do not run side by side");
    }
}
