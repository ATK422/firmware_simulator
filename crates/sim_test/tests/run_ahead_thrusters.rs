//! Talos's ESC boards spinning its thrusters while the boards run ahead of each other (`sim_host::runahead`): the
//! vehicle steps with the motors as they were at each slice boundary, and the camera cage's depth sensor sees the
//! vehicle move, exactly as when the boards take turns.

use sim_host::robot::Robot;
use sim_test::titan_link::{self, HostToDevice};
use sim_test::{MockBridge, RobotBench, Time};

/// RPM targets for both boards (each takes its four), as in `vehicle_talos.rs`.
const CMD: [i16; 8] = [800, -800, 800, -800, 1200, 1200, 1200, 1200];

/// The bridges of clients 2, 3 and 4 (the ESC boards and the camera cage) on `r`'s bus.
fn bridges(r: &mut Robot) -> Vec<MockBridge> {
    [2, 3, 4]
        .iter()
        .map(|&client| {
            let b = MockBridge::new(client, true, Time::ZERO);
            b.set_kill(Some(false));
            r.buses[0].endpoints.push(Box::new(b.clone()));
            b
        })
        .collect()
}

/// Runs `r` to `end` in 100 ms pieces, commanding the thrusters at the start of each.
fn run(r: &mut Robot, bridges: &[MockBridge], end: Time) {
    while r.now < end {
        for b in &bridges[..2] {
            b.send(HostToDevice::Esc(titan_link::esc::HostToDevice::ThrusterRpm(CMD)));
        }
        let t = (r.now + Time::from_ms(100)).min(end);
        assert!(r.run_until(t).is_none());
    }
}

#[test]
fn thrusters_move_the_vehicle_as_in_turns_when_running_ahead() {
    for elf in ["esc_board", "camera_cage_bb"] {
        if sim_test::titan_rs(elf).is_none() {
            return;
        }
    }
    let mut start = RobotBench::load("talos.toml", &[], |spec| {
        spec.board.retain(|b| ["esc0", "esc1", "camera_cage"].contains(&b.name.as_str()));
    })
    .unwrap()
    .robot;
    // The ESC boards hold neutral for their first 5 s
    start.threads = 3;
    let b = bridges(&mut start);
    run(&mut start, &b, Time::from_ms(4900));
    let outcome = |threads: usize, stress: Option<u64>| {
        let mut r = start.snapshot();
        let forks: Vec<MockBridge> = b.iter().map(MockBridge::fork).collect();
        for f in &forks {
            r.buses[0].endpoints.push(Box::new(f.clone()));
        }
        r.buses[0].bus.record = Some(Vec::new());
        r.threads = threads;
        r.run_ahead_stress = stress;
        if threads > 1 {
            r.fix_threads(vec![1, 2, 0]);
        }
        let before = r.run_ahead_stats;
        run(&mut r, &forks, Time::from_ms(5600));
        let v = &r.vehicles[0];
        let frames: Vec<String> =
            r.buses[0].bus.record.as_ref().unwrap().iter().map(|f| format!("{} {:?}", f.end, f.frame)).collect();
        let logs: Vec<String> =
            r.boards.iter().flat_map(|s| s.logs.iter().map(|(t, l)| format!("[{t}] {l}"))).collect();
        let received: Vec<String> = forks.iter().map(|f| format!("{:?}", f.state().received)).collect();
        let rpm = v.rpms(&r.boards, r.now);
        assert!(rpm.iter().all(|x| x.abs() > 500.0), "the thrusters turn: {rpm:?}");
        let state = format!("{:?} {:?} {rpm:?}", v.vehicle.state, v.vehicle.thrust_n);
        let stats = r.run_ahead_stats;
        ((frames, logs, received, state), stats.slices - before.slices, stats.rollbacks - before.rollbacks)
    };
    let (want, none, _) = outcome(1, None);
    assert_eq!(none, 0, "one thread takes turns");
    let (got, ahead, _) = outcome(3, None);
    assert_eq!(want, got);
    assert!(ahead > 6000, "{ahead} slices run ahead");
    let (got, _, rollbacks) = outcome(3, Some(11));
    assert_eq!(want, got);
    assert!(rollbacks > 100, "{rollbacks} rollbacks");
}
