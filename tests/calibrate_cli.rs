//! The calibration commands end to end: `calibrate thrusters` and `calibrate vehicle` on data the emulator's models
//! generate with known parameters, `calibrate acoustics` on a hydrophone capture the IVCBoard2 firmware itself took
//! in a pool that differs from the assumed one (`run --hydrophone-wav`), and `replay-pool-test` on a robot's own
//! log.

use std::path::{Path, PathBuf};
use std::process::Command;

use sim_calib::table::Table;

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("calib-cli-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn sim(args: &[&str]) -> (i32, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator")).args(args).current_dir(repo()).output().unwrap();
    let text = String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr);
    (out.status.code().unwrap_or(-1), text)
}

fn path(p: &Path) -> &str {
    p.to_str().unwrap()
}

fn toml_of(p: &Path) -> toml::Table {
    toml::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
}

fn num(v: &toml::Value) -> f64 {
    v.as_float().or(v.as_integer().map(|i| i as f64)).unwrap()
}

#[test]
fn calibrate_thrusters_from_a_test_stand_csv() {
    let mut esc = sim_devices::dshot::Esc::new("truth", 0, None);
    esc.curve = sim_devices::dshot::SpeedCurve::Fit { forward: [10.0, 0.11, 0.00004], reverse: [16.0, 0.095, 0.00005] };
    esc.tau = sim_core::time::Time::from_ms(45);
    let curve = sim_physics::ThrustCurve { forward_n: 47.0, reverse_n: 36.0, at_rpm: 3800.0 };
    let csv = temp("stand.csv");
    std::fs::write(&csv, sim_calib::thrusters::synthetic_stand(&esc, &curve, 10.0, 0.2, 16.0, 5).to_csv(4)).unwrap();
    let out = temp("thrusters");
    let (code, text) = sim(&[
        "calibrate",
        "thrusters",
        path(&csv),
        "--vehicle",
        "vehicles/talos.toml",
        "--board",
        "boards/mk2/esc_board.toml",
        "-o",
        path(&out),
    ]);
    assert_eq!(code, 0, "{text}");
    // The board file's ESCs have a throttle fit (the APD 80F3 bench fit): the same form is fitted
    assert!(text.contains("thrust curve:") && text.contains("forward_c1"), "{text}");
    let fitted = std::fs::read_to_string(out.join("thrusters.toml")).unwrap();
    // The two halves of the file are TOML: the vehicle's line and the ESC's keys
    let t: toml::Table = toml::from_str(&fitted).unwrap();
    let c = t["thruster_model"].as_table().unwrap();
    // At the vehicle file's at_rpm: compare the thrust at the truth's 3800 RPM
    let k = (3800.0 / num(&c["at_rpm"])).powi(2);
    assert!((num(&c["forward_n"]) * k - 47.0).abs() < 0.3 && (num(&c["reverse_n"]) * k - 36.0).abs() < 0.3, "{fitted}");
    assert!((num(&t["tau_ms"]) - 45.0).abs() < 2.0, "{fitted}");
    for f in ["report.md", "thrust.svg", "rpm.svg", "rpm_time.svg"] {
        assert!(out.join(f).exists(), "{f}");
    }
    // The ESC keys load in a board file, and its ESC turns as fast as the truth for each throttle
    let keys: String = fitted
        .lines()
        .filter(|l| l.starts_with("throttle_fit") || l.starts_with("tau_ms"))
        .map(|l| format!("{l}\n"))
        .collect();
    let board = sim_host::board::Board::parse(&format!("name = \"x\"\n[[dshot_esc]]\nsignal_pin = 1\n{keys}")).unwrap();
    let mut m = rp2040::Machine::new(Default::default());
    board.apply(&mut m).unwrap();
    let got = m.sys.pin_devices[0].as_any().downcast_ref::<sim_devices::dshot::Esc>().unwrap().curve;
    for d in [148u16, 400, 700, 1047, 1150, 1500, 2047] {
        assert!((got.rpm(d) - esc.curve.rpm(d)).abs() < 15.0, "{d}: {} vs {}\n{fitted}", got.rpm(d), esc.curve.rpm(d));
    }
    // A linear curve on request
    let (code, text) = sim(&["calibrate", "thrusters", path(&csv), "--curve", "linear", "-o", path(&out)]);
    assert_eq!(code, 0, "{text}");
    assert!(std::fs::read_to_string(out.join("thrusters.toml")).unwrap().contains("max_rpm = "));
    let (code, text) = sim(&["calibrate", "thrusters", path(&csv), "--fit", "nope", "-o", path(&out)]);
    assert_ne!(code, 0);
    assert!(text.contains("no ESC parameter \"nope\""), "{text}");
}

#[test]
fn calibrate_vehicle_from_a_pool_test_log() {
    let vf = sim_physics::VehicleFile::load(&repo().join("vehicles/talos.toml")).unwrap();
    let mut truth = vf.params.clone();
    truth.quadratic_drag[2] *= 0.8;
    truth.added_mass[2] *= 1.25;
    truth.volume_m3 *= 1.001;
    let data = sim_calib::vehicle::synthetic_pool_test(&vf, &truth, &Default::default(), 0.5, 2);
    // Two logs, as two pool runs would be
    let half = data.rows.len() / 2;
    let (a, b) = (temp("pool_a.csv"), temp("pool_b.csv"));
    std::fs::write(&a, Table { columns: data.columns.clone(), rows: data.rows[..half].to_vec() }.to_csv(5)).unwrap();
    std::fs::write(&b, Table { columns: data.columns.clone(), rows: data.rows[half..].to_vec() }.to_csv(5)).unwrap();
    let out = temp("vehicle");
    let (code, text) = sim(&[
        "calibrate",
        "vehicle",
        path(&a),
        path(&b),
        "--vehicle",
        "vehicles/talos.toml",
        "--fit",
        "volume_m3,quadratic_drag.heave,added_mass.heave,linear_drag.heave",
        "-o",
        path(&out),
    ]);
    assert_eq!(code, 0, "{text}");
    let f = sim_physics::VehicleFile::load(&out.join("vehicle.toml")).unwrap();
    assert!((f.params.quadratic_drag[2] / truth.quadratic_drag[2] - 1.0).abs() < 0.1, "{text}");
    assert!((f.params.added_mass[2] / truth.added_mass[2] - 1.0).abs() < 0.1, "{text}");
    // (The file's value is 1e-3 off: 0.3 N of buoyancy)
    assert!((f.params.volume_m3 / truth.volume_m3 - 1.0).abs() < 4e-4, "{text}");
    assert!(text.contains("depth_m") && out.join("depth_m.svg").exists() && out.join("report.md").exists());
}

/// The pool as it "really" is (noisier, its walls less reflective than environments/pool.toml assumes), with a
/// pinger that pings every 30 ms, timed so that a ping reaches the board early in its first capture (which then
/// holds the reverberation after it); the firmware's own capture of it is the recording.
const TRUE_POOL: &str = r#"
kind = "tank"
size_m = [25.0, 12.0, 3.0]
wall_reflection = 0.35
bottom_reflection = 0.5
water = { temperature_c = 26.0, salinity_ppt = 0.0 }
noise = { kind = "white", level_db_re_upa_per_hz = 62.0 }
multipath = { max_order = 3 }

[[source]]
kind = "pinger"
position_m = [20.0, 6.0, 2.0]
frequency_hz = 25_000
pulse_ms = 2
period_ms = 30
source_level_db = 160
start_ms = 25.5
"#;

#[test]
fn calibrate_acoustics_from_the_firmwares_own_capture() {
    let Some(elf) = sim_test::titan_rs("ivc_board2") else { return };
    let env = temp("true_pool.toml");
    std::fs::write(&env, TRUE_POOL).unwrap();
    let wav = temp("capture.wav");
    let (code, text) = sim(&[
        "run",
        path(&elf),
        "--board",
        "boards/talos/ivc_board.toml",
        "--environment",
        path(&env),
        "--position",
        "12,6,1.5",
        "--time",
        "600ms",
        "--hydrophone-wav",
        path(&wav),
    ]);
    assert_eq!(code, 0, "{text}");
    // When the capture starts, from the run's line about it
    let line = text.lines().find(|l| l.starts_with("hydrophone:")).unwrap_or_else(|| panic!("{text}"));
    let from = line.split(" from ").nth(1).unwrap().split('s').next().unwrap().parse::<f64>().unwrap();
    // The first of the firmware's captures (4096 samples at 500 kS/s) is the recording
    let manifest = temp("acoustics.toml");
    let source = TRUE_POOL.split("[[source]]").nth(1).unwrap();
    std::fs::write(
        &manifest,
        format!(
            "board = \"{}\"\nenvironment = \"{}\"\nfit = [\"noise_db\", \"wall_reflection\"]\n\n[source]{source}\n\
             [[recording]]\nfile = \"capture.wav\"\nreceiver_m = [12.0, 6.0, 1.5]\nstart_s = {from}\nduration_s = 0.008\n",
            repo().join("boards/talos/ivc_board.toml").display(),
            repo().join("environments/pool.toml").display(),
        ),
    )
    .unwrap();
    let out = temp("acoustics");
    let (code, text) = sim(&["calibrate", "acoustics", path(&manifest), "-o", path(&out)]);
    assert_eq!(code, 0, "{text}");
    let env = toml_of(&out.join("environment.toml"));
    let noise = num(&env["noise"]["level_db_re_upa_per_hz"]);
    let wall = num(&env["wall_reflection"]);
    assert!((noise - 62.0).abs() < 0.5, "noise {noise}\n{text}");
    assert!((wall - 0.35).abs() < 0.05, "wall {wall}\n{text}");
    assert!(out.join("sim_capture.wav").exists() && out.join("overlay.toml").exists());
    let report = std::fs::read_to_string(out.join("report.md")).unwrap();
    assert!(report.contains("## capture"), "{report}");
}

#[test]
fn replay_pool_test_of_a_robots_own_log_finds_nothing() {
    let Some(elf) = sim_test::titan_rs("power_board") else { return };
    let robot = temp("power_robot.toml");
    std::fs::write(
        &robot,
        format!(
            "[[bus]]\nname = \"internal\"\n\n[[board]]\nname = \"power\"\nboard = \"{}\"\nelf = \"{}\"\nbus = \"internal\"\n",
            repo().join("boards/mk2/power_board.toml").display(),
            elf.display()
        ),
    )
    .unwrap();
    let log = temp("power.log");
    let (code, text) =
        sim(&["robot", path(&robot), "--mock-bridge", "internal:1", "--time", "4s", "--candump-log", path(&log)]);
    assert_eq!(code, 0, "{text}");
    let report = temp("replay.md");
    let emulated = temp("emulated.log");
    let (code, text) = sim(&[
        "replay-pool-test",
        path(&log),
        "--robot",
        path(&robot),
        "--report",
        path(&report),
        "--candump-log",
        path(&emulated),
    ]);
    assert_eq!(code, 0, "{text}");
    assert!(text.contains("**SAME**") && text.contains("power's power-on"), "{text}");
    assert!(std::fs::read_to_string(&report).unwrap().contains("# Pool test replay"));
    assert!(!sim_host::canlog::read_file(&emulated).unwrap().is_empty());
    // A bus the robot does not have: an error, exit 2
    let (code, text) = sim(&["replay-pool-test", path(&log), "--robot", path(&robot), "--iface", "internal=nope"]);
    assert_eq!(code, 2, "{text}");
    assert!(text.contains("no bus nope"), "{text}");
}

/// scripts/bag_to_calib.py's conversion tests (no ROS needed), and its --help without ROS.
#[test]
fn bag_to_calib_python() {
    let out = Command::new("python3")
        .args(["-I", "-B", "-m", "unittest", "discover", "-s"])
        .arg(repo().join("scripts"))
        .args(["-p", "test_bag_to_calib.py"])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success() && err.contains("OK"), "{err}");
    let out = Command::new("python3")
        .args(["-I", "-B"])
        .arg(repo().join("scripts/bag_to_calib.py"))
        .arg("--help")
        .output()
        .unwrap();
    assert!(out.status.success() && String::from_utf8_lossy(&out.stdout).contains("--rpm-topic"));
}
