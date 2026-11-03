//! Robot files with a vehicle: the wiring of thrusters, pressure sensors and transducers to the boards, and the
//! sensors and transducers following the vehicle. The boards run a small embassy program (their firmware doesn't
//! matter here); the firmware-in-the-loop tests are in `sim_test/tests/vehicle*.rs`.

mod common;

use sim_acoustics::medium::Trajectory;
use sim_core::time::Time;
use sim_devices::dshot::Esc;
use sim_host::robot::{Robot, RobotSpec};

const ROBOT: &str = r#"
[environment]
file = "../environments/pool.toml"

[[board]]
name = "esc"
board = "../boards/mk2/esc_board.toml"
elf = "../test_programs/embassy_blinky.elf"

[[board]]
name = "cage"
board = "../boards/mk2/camera_cage_bb.toml"
elf = "../test_programs/embassy_blinky.elf"

[[board]]
name = "ivc"
board = "../boards/talos/ivc_board.toml"
elf = "../test_programs/embassy_blinky.elf"

[[vehicle]]
name = "talos"
file = "../vehicles/talos.toml"
position_m = [5.0, 6.0, 1.5]
rpy_deg = [0.0, 0.0, 90.0]
thrusters = [
    { thruster = "fwd_stbd", board = "esc", esc = "esc1" },
    { thruster = "heave_aft_port", board = "esc", esc = "esc4" },
]

[[vehicle.sensor]]
board = "cage"
input = "depth.pressure_pa"
mount_m = [0.3, 0.0, 0.1]

[[vehicle.transducer]]
board = "ivc"
mount_m = [-0.2, 0.0, 0.15]
"#;

fn spec(edit: impl FnOnce(&mut String)) -> RobotSpec {
    let mut text = ROBOT.to_string();
    edit(&mut text);
    toml::from_str(&text).unwrap()
}

fn load(spec: &RobotSpec) -> Result<Robot, String> {
    Robot::from_spec(spec, &common::repo().join("robots"))
}

const MOUNT: [f64; 3] = [-0.2, 0.0, 0.15];

#[test]
fn sensors_and_transducers_follow_the_vehicle() {
    let mut r = load(&spec(|_| {})).unwrap();
    let v = &r.vehicles[0].vehicle;
    // In the pool: it rests on its floor if it sinks
    assert_eq!(v.water.bottom_m, Some(3.0));
    // Pointing east (yaw 90): the transducer 0.2 m aft is 0.2 m west of the centre of gravity, 0.15 m deeper
    let p = v.point(MOUNT);
    assert!((p[0] - 5.0).abs() < 1e-9 && (p[1] - 5.8).abs() < 1e-9 && (p[2] - 1.65).abs() < 1e-9, "{p:?}");
    let pressure = 101_325.0 + 997.0 * 9.80665 * (1.5 + 0.1);
    assert!((r.inputs[1].get("depth.pressure_pa") - pressure).abs() < 1e-6);
    // Give it way: 0.5 m/s ahead and turning to starboard
    r.vehicles[0].vehicle.state.velocity = [0.5, 0.0, 0.0];
    r.vehicles[0].vehicle.state.rate = [0.0, 0.0, 0.2];
    r.vehicles[0].keep_log(Time::from_ms(100));
    assert!(r.run_until(Time::from_ms(500)).is_none());
    let v = r.vehicles[0].vehicle.clone();
    assert!((v.time - 0.5).abs() < 1e-9);
    assert!(v.state.position[1] > 6.15, "moved east: {:?}", v.state.position);
    assert!(v.state.attitude.euler()[2] > std::f64::consts::FRAC_PI_2);
    // The depth sensor input is the pressure at its port now
    assert!((r.inputs[1].get("depth.pressure_pa") - v.pressure_at([0.3, 0.0, 0.1])).abs() < 1e-6);
    // The IVC board's hydrophone and projector follow the transducer's mount, at every time of the run
    let now = v.point(MOUNT);
    let hydrophones: Vec<Trajectory> =
        sim_host::acoustic::hydrophones(&mut r.boards[2].machine).map(|h| h.trajectory.clone()).collect();
    assert_eq!(hydrophones.len(), 1);
    let Trajectory::Live(path) = &hydrophones[0] else { panic!("{:?}", hydrophones[0]) };
    let at = path.at(0.5);
    assert!(sim_physics::math::norm(sim_physics::math::sub(at, now)) < 1e-9, "{at:?} vs {now:?}");
    assert!(sim_physics::math::norm(sim_physics::math::sub(path.at(0.0), p)) < 1e-9);
    let medium = r.medium.as_ref().unwrap().lock().unwrap();
    assert_eq!(medium.projectors[0].owner, 2);
    assert_eq!(medium.projectors[0].trajectory, hydrophones[0]);
    drop(medium);
    // The log has a row every 100 ms
    let log = r.vehicles[0].log.as_ref().unwrap();
    assert_eq!(log.rows.len(), 6);
    assert!(log.to_csv().lines().nth(1).unwrap().starts_with("0.0000,5.00000,6.00000,1.50000,"));
}

#[test]
fn thrusters_read_their_esc_motor() {
    let mut r = load(&spec(|_| {})).unwrap();
    let escs: Vec<&mut Esc> = r.boards[0]
        .machine
        .sys
        .pin_devices
        .iter_mut()
        .filter_map(|d| d.as_any_mut().and_then(|a| a.downcast_mut::<Esc>()))
        .collect();
    assert_eq!(escs.len(), 4);
    for (i, e) in escs.into_iter().enumerate() {
        e.rpm = 1000.0 * (i + 1) as f64;
    }
    let rpm = r.vehicles[0].rpms(&r.boards, Time::ZERO);
    // fwd_stbd on esc1, heave_aft_port on esc4; the rest aren't wired
    assert_eq!(rpm, [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4000.0]);
}

#[test]
fn snapshots_carry_their_own_vehicle() {
    let mut r = load(&spec(|_| {})).unwrap();
    assert!(r.run_until(Time::from_ms(100)).is_none());
    let mut copy = r.snapshot();
    copy.vehicles[0].vehicle.state.velocity = [0.0, 0.0, 1.0];
    assert!(copy.run_until(Time::from_ms(300)).is_none());
    // The depth sensor input is shared, so the copy's run left its own pressure there; the original's next run
    // starts by putting its own back
    let port = [0.3, 0.0, 0.1];
    assert_eq!(r.inputs[1].get("depth.pressure_pa"), copy.vehicles[0].vehicle.pressure_at(port));
    assert!(r.run_until(r.now).is_none());
    assert_eq!(r.inputs[1].get("depth.pressure_pa"), r.vehicles[0].vehicle.pressure_at(port));
    assert!(r.run_until(Time::from_ms(300)).is_none());
    // The copy dived 20 cm; the original, trimmed 2 N positive, barely moved
    let (a, b) = (r.vehicles[0].vehicle.state.position[2], copy.vehicles[0].vehicle.state.position[2]);
    assert!(b - a > 0.1, "{a} {b}");
    // Each one's transducer follows its own vehicle
    for robot in [&mut r, &mut copy] {
        let want = robot.vehicles[0].vehicle.point(MOUNT);
        let h = sim_host::acoustic::hydrophones(&mut robot.boards[2].machine).next().unwrap();
        assert!((h.trajectory.at(0.3)[2] - want[2]).abs() < 1e-9);
        let medium = robot.medium.as_ref().unwrap().lock().unwrap();
        assert!((medium.projectors[0].trajectory.at(0.3)[2] - want[2]).abs() < 1e-9);
    }
}

#[test]
fn wiring_mistakes_are_reported() {
    let err = |from: &str, to: &str| {
        let s = spec(|t| *t = t.replacen(from, to, 1));
        load(&s).err().unwrap_or_else(|| panic!("{from} -> {to} loaded"))
    };
    assert!(err("thruster = \"fwd_stbd\"", "thruster = \"nope\"").contains("no thruster \"nope\""));
    assert!(err("board = \"esc\", esc = \"esc1\"", "board = \"motor\", esc = \"esc1\"").contains("no board \"motor\""));
    assert!(err("esc = \"esc1\"", "esc = \"esc9\"").contains("no ESC \"esc9\""));
    assert!(
        err("esc = \"esc4\"", "esc = \"esc4\" },\n{ thruster = \"fwd_stbd\", board = \"esc\", esc = \"esc2\"")
            .contains("wired twice")
    );
    assert!(err("input = \"depth.pressure_pa\"", "input = \"depth.pressure\"").contains("no input"));
    assert!(
        err("name = \"ivc\"\nboard", "name = \"ivc\"\nposition_m = [1.0, 1.0, 1.0]\nboard").contains("on a vehicle")
    );
    assert!(err("[environment]\nfile = \"../environments/pool.toml\"", "").contains("no transducer in an acoustic"));
    assert!(err("file = \"../vehicles/talos.toml\"", "file = \"../vehicles/none.toml\"").contains("none.toml"));
    let mut two = spec(|_| {});
    two.vehicle.push(two.vehicle[0].clone());
    assert!(load(&two).err().unwrap().contains("two vehicles"));
}

#[test]
fn shipped_robot_files_load_their_vehicles() {
    for (file, name, thrusters) in [("talos.toml", "talos", 8), ("puddles.toml", "puddles", 8)] {
        let path = common::repo().join("robots").join(file);
        let mut spec: RobotSpec = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        // The vehicle wiring only needs the boards' files: boot them with the small program instead
        for b in &mut spec.board {
            b.elf = "../test_programs/embassy_blinky.elf".into();
        }
        let r = Robot::from_spec(&spec, path.parent().unwrap()).unwrap();
        let v = &r.vehicles[0];
        assert_eq!(v.vehicle.name, name);
        assert_eq!(v.vehicle.thrusters.len(), thrusters);
        assert_eq!(v.rpms(&r.boards, Time::ZERO).len(), thrusters);
        // Afloat: positively buoyant, at the depth where it floats
        assert!(v.vehicle.params.net_buoyancy_n(&v.vehicle.water) > 0.0);
        assert_eq!(v.vehicle.floating_depth(), Some(v.vehicle.state.position[2]));
    }
}
