//! Vehicle physics with the real firmware in the loop: thruster commands go through the boards' DShot to the ESC
//! and motor models, the thrust moves the simulated vehicle, and the boards' own depth sensors report the motion.

use sim_host::ethernet::PuddlesBridge;
use sim_test::titan_link::puddles_backplane as pb;
use sim_test::titan_link::{DeviceToHost, HostToDevice};
use sim_test::{RobotBench, Time};

/// Puddles from its robot file, with a scripted bridge on the backplane's Ethernet.
fn puddles() -> Option<(RobotBench, PuddlesBridge)> {
    sim_test::titan_rs("puddles_backplane")?;
    let mut r = RobotBench::load("puddles.toml", &[], |_| {}).unwrap();
    let bridge = r.eth_bridge("backplane");
    Some((r, bridge))
}

/// The depths (positive down, m) the backplane reported, with when.
fn reported_depths(bridge: &PuddlesBridge) -> Vec<(Time, f64)> {
    bridge
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::PuddlesBackplane(pb::DeviceToHost::Depth(d)) => Some((*t, -*d as f64)),
            _ => None,
        })
        .collect()
}

/// How deep the vehicle's depth sensor port is now, below where the firmware zeroed it (m).
fn sensor_depth(r: &mut RobotBench, zero: f64) -> f64 {
    let v = &r.robot.vehicle("puddles").unwrap().vehicle;
    v.point([-0.2, 0.0, 0.0])[2] - zero
}

#[test]
fn puddles_dives_on_thruster_commands_and_its_depth_sensor_sees_it() {
    let Some((mut r, bridge)) = puddles() else { return };
    let zero = r.robot.vehicle("puddles").unwrap().vehicle.point([-0.2, 0.0, 0.0])[2];
    // Afloat until the commands take effect, 5 s after boot; the sensor zeroes meanwhile
    r.run_until(Time::from_ms(5100));
    assert!(r.board("backplane").logged("Depth zeroed"));
    let before = reported_depths(&bridge);
    assert!(before.len() > 20, "{}", before.len());
    assert!(before.iter().all(|(_, d)| d.abs() < 0.01), "{before:?}");
    // The vertical thrusters (5-8) at 30 % push it down
    let dive = [0, 0, 0, 0, 300, 300, 300, 300];
    for _ in 0..150 {
        bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot(dive)));
        r.run_for(Time::from_ms(20));
    }
    let v = r.robot.vehicle("puddles").unwrap().vehicle.clone();
    eprintln!("after 3 s diving: {:?}, thrust {:?}", v.state, v.thrust_n);
    // Throttle 300 is DShot 348, 301/1000 of the ESC's 4000 RPM: 5.90 N from each T200 (50.82 N at 3533 RPM)
    let rpm = 301.0 / 1000.0 * 4000.0;
    let thrust = 50.82 * (rpm / 3533.0f64).powi(2);
    assert!(v.thrust_n[4..].iter().all(|t| (*t - thrust).abs() < 0.01), "{:?}", v.thrust_n);
    assert!(v.thrust_n[..4].iter().all(|t| *t == 0.0));
    // Sinking at the speed where the heave drag (33 w + 190 w^2) takes the 4 x 5.90 N less 1 N of buoyancy
    let force = 4.0 * thrust - 1.0;
    let terminal = (-33.0 + (33.0f64 * 33.0 + 4.0 * 190.0 * force).sqrt()) / (2.0 * 190.0);
    let w = v.state.world_velocity()[2];
    assert!((w - terminal).abs() < 0.002, "{w} vs {terminal}");
    let depth = sensor_depth(&mut r, zero);
    assert!(depth > 0.3, "{depth}");
    // The firmware's own reading (state/depth) agrees with where the sensor is, to within a poll's motion
    let (t, last) = *reported_depths(&bridge).last().unwrap();
    assert!(r.now() - t < Time::from_ms(100));
    assert!((last - depth).abs() < 0.03, "reported {last} vs {depth}");
    // Going down monotonically, level, without turning
    let during: Vec<f64> =
        reported_depths(&bridge).iter().filter(|(t, _)| *t > Time::from_ms(5600)).map(|d| d.1).collect();
    assert!(during.windows(2).all(|w| w[1] >= w[0] - 1e-3), "{during:?}");
    let [roll, pitch, yaw] = v.state.attitude.euler();
    assert!(roll.abs() < 0.01 && pitch.abs() < 0.01 && yaw.abs() < 1e-6, "{roll} {pitch} {yaw}");
    // The physical kill switch pulled (GPIO 12 left to its pull-up): the backplane stops the thrusters although the
    // commands keep coming, the drag stops the vehicle and the 1 N of buoyancy slowly brings it back up
    r.robot.boards[0].machine.sys.gpio.external[12] = sim_core::wire::External::Floating;
    for _ in 0..200 {
        bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot(dive)));
        r.run_for(Time::from_ms(20));
    }
    let v = &r.robot.vehicle("puddles").unwrap().vehicle;
    assert!(v.thrust_n.iter().all(|t| t.abs() < 1e-6), "spun down: {:?}", v.thrust_n);
    let w = r.robot.vehicle("puddles").unwrap().vehicle.state.world_velocity()[2];
    assert!((-0.03..-0.01).contains(&w), "rising: {w}");
}

#[test]
fn a_depth_controller_through_the_firmware_reaches_and_holds_the_target() {
    let Some((mut r, bridge)) = puddles() else { return };
    let zero = r.robot.vehicle("puddles").unwrap().vehicle.point([-0.2, 0.0, 0.0])[2];
    r.run_until(Time::from_ms(5100));
    // A PD controller on the backplane's depth readings, as the robot computer would run it: thrust force from
    // the error, plus the buoyancy to cancel, turned into throttle through the T200's square law
    let target = 1.0;
    let (kp, kd, buoyancy) = (40.0, 40.0, 1.0);
    // Full throttle is 4000 RPM, where a T200 makes 50.82 * (4000/3533)^2 N forwards (40.53 * ... in reverse)
    let full = |reverse: bool| if reverse { 40.53 } else { 50.82 } * (4000.0f64 / 3533.0).powi(2) * 4.0;
    let mut est = (0.0, 0.0, Time::ZERO);
    let mut trace = Vec::new();
    while r.now() < Time::from_secs(25) {
        if let Some(&(t, depth)) = reported_depths(&bridge).last()
            && t > est.2
        {
            // Speed from successive readings, smoothed
            let dt = (t - est.2).as_secs_f64();
            let speed = if est.2 == Time::ZERO { 0.0 } else { (depth - est.0) / dt };
            est = (depth, est.1 * 0.7 + speed * 0.3, t);
        }
        let force: f64 = kp * (target - est.0) - kd * est.1 + buoyancy;
        let throttle = ((force.abs() / full(force < 0.0)).sqrt() * 1000.0).min(999.0) * force.signum();
        let t = throttle.round() as i16;
        bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot([0, 0, 0, 0, t, t, t, t])));
        r.run_for(Time::from_ms(20));
        trace.push((r.now(), est.0));
    }
    let late: Vec<f64> = trace.iter().filter(|(t, _)| *t > Time::from_secs(15)).map(|x| x.1).collect();
    eprintln!("last readings {:?}", &late[late.len() - 5..]);
    assert!(late.iter().all(|d| (d - target).abs() < 0.05), "{late:?}");
    // It got there without much overshoot, and the truth agrees with what the firmware reported
    let peak = trace.iter().map(|x| x.1).fold(0f64, f64::max);
    assert!(peak < target + 0.15, "{peak}");
    let v = r.robot.vehicle("puddles").unwrap().vehicle.clone();
    let truth = v.point([-0.2, 0.0, 0.0])[2] - zero;
    assert!((truth - target).abs() < 0.03, "{truth}");
    assert!(v.state.world_velocity()[2].abs() < 0.02);
}

#[test]
fn puddles_runs_reproduce_exactly() {
    if sim_test::titan_rs("puddles_backplane").is_none() {
        return;
    }
    // Firmware, DShot, motors and vehicle: the same commands give the same motion, to the last bit
    let go = || {
        let (mut r, bridge) = puddles().unwrap();
        r.run_until(Time::from_ms(5000));
        for i in 0..40 {
            let t = 100 * (i % 7) as i16 - 300;
            bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot([t, -t, 50, 0, 200, 210, 190, t])));
            r.run_for(Time::from_ms(20));
        }
        let v = r.robot.vehicle("puddles").unwrap().vehicle.clone();
        (v.state, v.thrust_n, reported_depths(&bridge))
    };
    let (a, b) = (go(), go());
    assert!(a.0.position[2] > 0.2, "{:?}", a.0);
    assert!(a.0.attitude.euler()[2].abs() > 1e-3, "turning: {:?}", a.0);
    assert_eq!(a, b);
}

/// Puddles robots sweep like CAN robots: each snapshot gets its own copy of the Ethernet bridge (and vehicle), so
/// points with different thruster commands run side by side, each reporting its own dive to its own bridge, and
/// exactly as a lone copy would.
#[test]
fn puddles_dives_sweep_in_parallel() {
    let Some((mut base, bridge)) = puddles() else { return };
    base.run_until(Time::from_ms(5100));
    let seen = bridge.state().received.len();
    let dive = |mut b: RobotBench, throttle: &i16| {
        // The copy's own bridge, not the original's
        let bridge = b.eth_bridge("backplane");
        for _ in 0..75 {
            let t = *throttle;
            bridge.send(HostToDevice::PuddlesBackplane(pb::HostToDevice::Dshot([0, 0, 0, 0, t, t, t, t])));
            b.run_for(Time::from_ms(20));
        }
        let truth = b.robot.vehicle("puddles").unwrap().vehicle.point([-0.2, 0.0, 0.0])[2];
        (reported_depths(&bridge).last().unwrap().1, truth)
    };
    let throttles = [200i16, 400, 600];
    let results = sim_test::sweep(&base, &throttles, throttles.len(), dive);
    let zero = base.robot.vehicle("puddles").unwrap().vehicle.point([-0.2, 0.0, 0.0])[2];
    for w in results.windows(2) {
        assert!(w[1].1 > w[0].1 + 0.05, "more thrust, deeper: {results:?}");
    }
    // Each reading within a poll's motion of the truth (it sinks at up to 0.4 m/s)
    for (got, truth) in &results {
        assert!((got - (truth - zero)).abs() < 0.06, "reported {got} vs {}", truth - zero);
    }
    assert_eq!(dive(base.snapshot(), &throttles[1]), results[1], "the same as a copy run on its own");
    // The original's bridge heard nothing of it, and the original did not move
    assert_eq!(bridge.state().received.len(), seen);
    assert!(sensor_depth(&mut base, zero).abs() < 1e-3);
}
