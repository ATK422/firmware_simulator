//! The host tools through the command line: `robot --candump-log` / `--replay`, `can-diff` and `bridges`.

use std::path::{Path, PathBuf};
use std::process::Command;

fn exe() -> Command {
    Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
}

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// A fresh directory for one test's files.
fn scratch(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("fwsim-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn run(cmd: &mut Command) -> (i32, String, String) {
    let out = cmd.output().unwrap();
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

fn talos_built() -> bool {
    let dir = sim_host::titan_rs_release_dir(&repo());
    let ok = ["poac_board", "esc_board", "camera_cage_bb", "actuator_board", "led_board", "sbc_board", "ivc_board2"]
        .iter()
        .chain(&["smart_battery"])
        .all(|e| dir.join(e).exists());
    if !ok {
        eprintln!("skipping: titan_firmware_rs images not built in {}", dir.display());
    }
    ok
}

/// Talos with a bridge per client, logged; the bridges' side replayed into a second run; the logs compared.
#[test]
fn robot_replay_and_diff() {
    if !talos_built() {
        return;
    }
    let d = scratch("robot");
    let (a, b) = (d.join("a.log"), d.join("b.log"));
    let mut first = exe();
    first.args(["robot", "robots/talos.toml", "--time", "1.2s", "--candump-log"]).arg(&a).current_dir(repo());
    for c in ["internal:1", "internal:2", "internal:3", "internal:4", "internal:5", "internal:8", "external:21"] {
        first.args(["--mock-bridge", c]);
    }
    let (code, _, err) = run(&mut first);
    assert_eq!(code, 0, "{err}");
    assert!(err.contains("candump log: "), "{err}");
    let log = std::fs::read_to_string(&a).unwrap();
    assert!(log.lines().any(|l| l.contains(" internal ")) && log.lines().any(|l| l.contains(" external ")));

    let (code, _, err) = run(exe()
        .args(["robot", "robots/talos.toml", "--time", "1.2s", "--replay-filter", "sender=host", "--replay"])
        .arg(&a)
        .arg("--candump-log")
        .arg(&b)
        .current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    assert!(err.contains("(internal -> internal)") && err.contains("(external -> external)"), "{err}");

    let (code, out, err) = run(exe().arg("can-diff").arg(&a).arg(&b));
    assert_eq!(code, 0, "{out}{err}");
    assert!(out.contains("identical"), "{out}");
    assert!(out.contains("external client 21 board->host SmartBattery::BatteryStatus"), "{out}");
    assert!(out.contains("internal client 8 board->host Heartbeat"), "{out}");
    assert!(out.contains("aligned on timestamps as logged"), "{out}");

    // Without the bridges' traffic the boards never connect: their link messages are missing
    let c = d.join("c.log");
    let (code, _, err) =
        run(exe().args(["robot", "robots/talos.toml", "--time", "1.2s", "--candump-log"]).arg(&c).current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    let (code, out, _) = run(exe().arg("can-diff").arg(&a).arg(&c).args(["--details", "1"]));
    assert_eq!(code, 1, "{out}");
    assert!(out.contains("DIFFERENT") && out.contains("missing in B"), "{out}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn run_replay_needs_a_bus_and_bad_logs_fail() {
    let d = scratch("run");
    let bad = d.join("bad.log");
    std::fs::write(&bad, "(1.0) can0 06F#C\n").unwrap();
    let (code, _, err) = run(exe().arg("can-diff").arg(&bad).arg(&bad));
    assert_eq!(code, 2, "{err}");
    assert!(err.contains("line 1"), "{err}");
    let (code, _, err) = run(exe()
        .args(["run", "test_programs/embassy_blinky.elf", "--time", "1ms", "--replay"])
        .arg(&bad)
        .current_dir(repo()));
    assert_ne!(code, 0);
    assert!(err.contains("need a --board with a CAN bus"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
}

fn check_bash(path: &Path) {
    let (code, _, err) = run(Command::new("bash").arg("-n").arg(path));
    assert_eq!(code, 0, "bash -n {}: {err}", path.display());
}

#[test]
fn bridges_lines_script_and_launch_file() {
    let (code, out, err) = run(exe().args(["bridges", "robots/talos.toml"]).current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    assert!(out.contains("titan_bridge --board esc1 --interface vcan0 --namespace talos --client-id 3\n"), "{out}");
    assert!(out.contains("titan_bridge --board smart_battery --interface vcan1 --namespace talos --client-id 21\n"));

    let d = scratch("bridges");
    let sh = d.join("bridges.sh");
    let (code, _, err) = run(exe()
        .args(["bridges", "robots/talos.toml", "--format", "script", "--can", "internal=can0", "--output"])
        .arg(&sh)
        .current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    check_bash(&sh);
    use std::os::unix::fs::PermissionsExt;
    assert_eq!(std::fs::metadata(&sh).unwrap().permissions().mode() & 0o111, 0o111, "executable");
    let text = std::fs::read_to_string(&sh).unwrap();
    assert!(text.contains("start poac --board poac --interface can0"), "{text}");
    // Without the interfaces the script stops before starting anything
    let (code, _, err) = run(Command::new("bash").arg(&sh).env("PATH", "/usr/bin:/bin"));
    assert_eq!(code, 1, "{err}");
    assert!(err.contains("no interface can0"), "{err}");

    let py = d.join("bridges.launch.py");
    let (code, _, err) =
        run(exe().args(["bridges", "robots/talos.toml", "--format", "launch", "-o"]).arg(&py).current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    let (code, _, err) = run(Command::new("python3").args(["-I", "-m", "py_compile"]).arg(&py));
    assert_eq!(code, 0, "{err}");
    let (code, _, _) = run(exe().args(["bridges", "robots/talos.toml", "--can", "nosuch=vcan5"]).current_dir(repo()));
    assert_ne!(code, 0);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn bridges_probe_finds_client_ids() {
    if !talos_built() {
        return;
    }
    let (code, out, err) = run(exe().args(["bridges", "robots/talos.toml", "--probe"]).current_dir(repo()));
    assert_eq!(code, 0, "{err}");
    let commands: Vec<&str> = out.lines().filter(|l| l.starts_with("titan_bridge")).collect();
    assert_eq!(commands.len(), 9, "{out}");
    assert!(!out.contains("no CANmore heartbeat") && !out.contains("suggests client"), "{out}");
}
