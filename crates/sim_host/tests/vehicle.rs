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

/// The first-generation IVC boards of `robots/ivc_pool.toml` still talk when Talos's board rides on a vehicle that
/// a current carries towards the surface vehicle: its projector and hydrophone move with the hull. (At 0.5 m/s
/// the first byte fails its CRC, exactly as with a scripted `Trajectory::Linear` at that speed.)
#[test]
fn ivc_boards_talk_from_a_drifting_vehicle() {
    use titan_link::{DeviceToHost, HostToDevice};
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let path = common::repo().join("robots/ivc_pool.toml");
    let mut spec: RobotSpec = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    spec.board[0].position_m = None;
    spec.vehicle.push(
        toml::from_str(
            r#"
name = "talos"
file = "../vehicles/talos.toml"
position_m = [5.0, 6.0, 1.5]
water = { current_mps = [0.2, 0.0, 0.0] }
transducer = [{ board = "talos_ivc", mount_m = [0.0, 0.0, 0.0] }]
"#,
        )
        .unwrap(),
    );
    let mut r = Robot::from_spec(&spec, path.parent().unwrap()).unwrap();
    // Trimmed neutral, and already moving with the water
    let v = &mut r.vehicles[0].vehicle;
    v.params.volume_m3 = v.params.mass_kg / v.water.density_kg_m3;
    v.state.velocity = [v.water.current_mps[0], 0.0, 0.0];
    let mut bridges = Vec::new();
    for b in r.buses.iter_mut() {
        let br = sim_host::mock_bridge::MockBridge::new(8, true, Time::ZERO);
        br.set_kill(Some(false));
        b.endpoints.push(Box::new(br.clone()));
        bridges.push(br);
    }
    r.run_until(Time::from_ms(600));
    for b in [0x5A, 0xC3] {
        bridges[0].send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(b)));
    }
    r.run_until(Time::from_ms(4000));
    let got: Vec<u8> = bridges[1]
        .state()
        .received
        .iter()
        .filter_map(|(_, m)| match m {
            DeviceToHost::Ivc(titan_link::ivc::DeviceToHost::Rx(b)) => Some(*b),
            _ => None,
        })
        .collect();
    assert_eq!(got, [0x5A, 0xC3]);
    // Talos's transducer went 0.8 m closer, and that is where the surface board hears it from
    let p = r.vehicles[0].vehicle.state.position;
    assert!((p[0] - 5.8).abs() < 0.01 && (p[2] - 1.5).abs() < 0.01, "{p:?}");
    let medium = r.medium.as_ref().unwrap().lock().unwrap();
    let at = medium.projectors[0].trajectory.at(4.0);
    assert!((at[0] - p[0]).abs() < 1e-6, "{at:?}");
}

/// A robot with a vehicle but no acoustic medium runs its boards side by side with `threads` > 1 (only a medium
/// couples boards within a slice); the vehicle is stepped after each slice on the calling thread, from the boards'
/// ESC motors and into their pressure inputs. Each thread count must give exactly the run of boards in turn.
#[test]
fn a_vehicle_without_a_medium_runs_the_same_on_threads() {
    use sim_core::cpu::Cpu;
    let run = |threads: usize| {
        let mut r = load(&spec(|t| {
            *t = t.replacen("[environment]\nfile = \"../environments/pool.toml\"\n", "", 1);
            let i = t.find("[[vehicle.transducer]]").unwrap();
            t.truncate(i);
        }))
        .unwrap();
        assert!(r.medium.is_none());
        r.threads = threads;
        // Both wired motors spinning (and running down, as nothing commands them): thrust ahead and a dive
        for e in r.boards[0]
            .machine
            .sys
            .pin_devices
            .iter_mut()
            .filter_map(|d| d.as_any_mut().and_then(|a| a.downcast_mut::<Esc>()))
        {
            e.rpm = 3000.0;
        }
        r.vehicles[0].vehicle.state.rate = [0.0, 0.0, 0.2];
        r.vehicles[0].keep_log(Time::from_ms(10));
        assert!(r.run_until(Time::from_ms(300)).is_none());
        let v = &r.vehicles[0];
        let mut out = vec![format!("{:?}", v.vehicle.state), v.log.as_ref().unwrap().to_csv()];
        out.push(format!("{:?}", r.inputs[1].get("depth.pressure_pa").to_bits()));
        for (name, s) in r.names.iter().zip(&r.boards) {
            out.extend(s.logs.iter().map(|(t, l)| format!("{name} [{t}] {l}")));
            for (c, core) in s.machine.cores.iter().enumerate() {
                let regs: Vec<u32> = (0..core.num_regs()).map(|n| core.read_reg(n)).collect();
                out.push(format!("{name} core{c} {} {} {regs:x?}", s.machine.core_time(c), core.cycles()));
            }
        }
        out
    };
    let turns = run(1);
    // It moved: ahead from the starboard thruster, and down from the aft heave one
    assert!(turns[1].lines().count() > 25, "{}", turns[1]);
    assert_ne!(turns[1].lines().nth(1), turns[1].lines().last());
    assert_eq!(turns, run(3));
}
