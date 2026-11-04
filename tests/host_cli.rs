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
    ["poac_board", "esc_board", "camera_cage_bb", "actuator_board", "led_board", "sbc_board", "ivc_board2"]
        .iter()
        .chain(&["smart_battery"])
        .all(|e| sim_host::titan_rs_image(&repo(), e).is_some())
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

    let (code, out, err) =
        run(exe().arg("can-diff").arg(&a).arg(&b).arg("--robot").arg(repo().join("robots/talos.toml")));
    assert_eq!(code, 0, "{out}{err}");
    assert!(out.contains("identical"), "{out}");
    assert!(out.contains("external client 21 (battery) board->host SmartBattery::BatteryStatus"), "{out}");
    assert!(out.contains("internal client 8 (ivc) board->host Heartbeat"), "{out}");
    assert!(out.contains("aligned on their timestamps as logged"), "{out}");

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

/// Stand-ins for `ip`, `modprobe` and `sudo` that log their calls and keep the interfaces in a file, so the vcan
/// script's logic runs without kernel CAN support.
fn stub_tools(dir: &Path, modprobe_ok: bool) {
    let ip = r#"#!/bin/bash
echo "ip $*" >> "$STUB_DIR/log"
state="$STUB_DIR/state"
touch "$state"
case "$1 $2" in
    "link show")
        line=$(grep "^$3 " "$state") || exit 1
        echo "1: $3: <NOARP,UP> mtu ${line#* } qdisc noqueue"
        ;;
    "link add") echo "$4 16" >> "$state" ;;
    "link set") if [ "$5" = mtu ]; then sed -i "s/^$4 .*/$4 $6/" "$state"; fi ;;
    "link delete") sed -i "/^$4 /d" "$state" ;;
esac
"#;
    let modprobe =
        format!("#!/bin/bash\necho \"modprobe $*\" >> \"$STUB_DIR/log\"\nexit {}\n", if modprobe_ok { 0 } else { 1 });
    let sudo = "#!/bin/bash\n\"$@\"\n";
    for (name, text) in [("ip", ip), ("modprobe", modprobe.as_str()), ("sudo", sudo)] {
        let p = dir.join(name);
        std::fs::write(&p, text).unwrap();
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
    }
}

fn vcan_up(stubs: &Path, args: &[&str]) -> (i32, String, String) {
    let path = format!("{}:{}", stubs.display(), std::env::var("PATH").unwrap());
    run(Command::new("bash").arg(repo().join("scripts/vcan_up.sh")).args(args).env("PATH", path).env("STUB_DIR", stubs))
}

#[test]
fn vcan_script() {
    check_bash(&repo().join("scripts/vcan_up.sh"));
    let d = scratch("vcan");
    stub_tools(&d, true);
    let (code, out, err) = vcan_up(&d, &[]);
    assert_eq!(code, 0, "{err}");
    assert_eq!(out, "vcan0 up (CAN FD MTU 72)\nvcan1 up (CAN FD MTU 72)\n");
    let log = std::fs::read_to_string(d.join("log")).unwrap();
    for call in
        ["modprobe vcan", "ip link add dev vcan0 type vcan", "ip link set dev vcan0 mtu 72", "ip link set dev vcan1 up"]
    {
        assert!(log.contains(call), "{call}: {log}");
    }
    assert!(log.contains("ip link set dev vcan1 txqueuelen 1000"));
    assert_eq!(std::fs::read_to_string(d.join("state")).unwrap(), "vcan0 72\nvcan1 72\n");
    // Again: nothing is re-created, and an interface that already has the FD MTU is not taken down
    std::fs::remove_file(d.join("log")).unwrap();
    let (code, _, _) = vcan_up(&d, &["vcan0"]);
    assert_eq!(code, 0);
    let log = std::fs::read_to_string(d.join("log")).unwrap();
    assert!(!log.contains("link add") && !log.contains("down"), "{log}");
    let (code, out, _) = vcan_up(&d, &["--down"]);
    assert_eq!((code, out.as_str()), (0, "vcan0 removed\nvcan1 removed\n"));
    assert_eq!(std::fs::read_to_string(d.join("state")).unwrap(), "");
    // Without the kernel module it says what to do instead
    stub_tools(&d, false);
    let (code, _, err) = vcan_up(&d, &[]);
    assert_eq!(code, 1);
    assert!(err.contains("no virtual CAN support"), "{err}");
    assert_eq!(vcan_up(&d, &["--bogus"]).0, 2);
    let _ = std::fs::remove_dir_all(&d);
}

/// The Docker files can't be built here; check what can be checked: the compose file parses (with PyYAML, if
/// present), the services use the host's network, and every file the Dockerfiles copy exists.
#[test]
fn docker_files_are_consistent() {
    let dir = repo().join("docker");
    for df in ["robot_computer/Dockerfile", "emulator/Dockerfile"] {
        let text = std::fs::read_to_string(dir.join(df)).unwrap();
        for line in text.lines().filter(|l| l.starts_with("COPY ") && !l.contains("--from")) {
            let words: Vec<&str> = line.split_whitespace().collect();
            for src in &words[1..words.len() - 1] {
                if *src != "." {
                    assert!(repo().join(src).exists(), "{df}: {src} does not exist");
                }
            }
        }
    }
    check_bash(&dir.join("robot_computer/entrypoint.sh"));
    let compose = dir.join("robot_computer/compose.yaml");
    let script = "import sys, yaml\nd = yaml.safe_load(open(sys.argv[1]))\nfor n, s in d['services'].items():\n    \
                  print(n, s['network_mode'], s.get('build', {}).get('dockerfile', '-'))\n";
    let (code, out, err) = run(Command::new("python3").args(["-I", "-c", script]).arg(&compose));
    if err.contains("No module named 'yaml'") {
        eprintln!("skipping the YAML check: no PyYAML");
        return;
    }
    assert_eq!(code, 0, "{err}");
    assert_eq!(
        out,
        "emulator host docker/emulator/Dockerfile\nrobot_computer host docker/robot_computer/Dockerfile\ntools host -\n"
    );
}

/// A hardware log's interfaces have to be mapped to the robot's buses.
#[test]
fn robot_replay_maps_interfaces() {
    if !talos_built() {
        return;
    }
    let d = scratch("map");
    let log = d.join("hw.log");
    std::fs::write(&log, "(1700000000.000000) can0 070#01\n(1700000000.000100) can1 070#02\n").unwrap();
    let robot = |extra: &[&str]| {
        let mut c = exe();
        c.args(["robot", "robots/talos.toml", "--time", "1ms", "--replay"]).arg(&log).args(extra).current_dir(repo());
        run(&mut c)
    };
    let (code, _, err) = robot(&[]);
    assert_ne!(code, 0);
    assert!(err.contains("no bus for interface \"can0\"; map it with --replay-iface can0=BUS"), "{err}");
    let (code, _, err) = robot(&["--replay-iface", "can0=internal", "--replay-iface", "can1=external"]);
    assert_eq!(code, 0, "{err}");
    assert!(err.contains("(can0 -> internal): 1 of 1 frame(s) sent"), "{err}");
    assert!(err.contains("(can1 -> external): 1 of 1 frame(s) sent"), "{err}");
    // An interface left out on purpose is skipped with a warning
    let (code, _, err) = robot(&["--replay-iface", "can0=internal"]);
    assert_eq!(code, 0, "{err}");
    assert!(err.contains("frames on can1 are not replayed"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
}

/// The emulator <-> nereus bridge's Python: its logic and pack generation tests (no ROS needed), and the files that
/// need ROS compile.
#[test]
fn nereus_bridge_python() {
    let dir = repo().join("integrations/nereus");
    let (code, out, err) = run(Command::new("python3")
        .args(["-I", "-B", "-m", "unittest", "discover", "-s"])
        .arg(&dir)
        .args(["-p", "test_*.py"]));
    assert_eq!(code, 0, "{out}{err}");
    assert!(err.contains("OK"), "{err}");
    for f in ["firmware_bridge.py", "nereus_firmware.launch.py", "make_packs.py"] {
        let src = std::fs::read_to_string(dir.join(f)).unwrap();
        let parse = "import ast, sys; ast.parse(open(sys.argv[1]).read())";
        let (code, _, err) = run(Command::new("python3").args(["-I", "-B", "-c", parse]).arg(dir.join(f)));
        assert_eq!(code, 0, "{f}: {err}");
        assert!(src.starts_with("#!/usr/bin/env python3") || f.ends_with(".launch.py"), "{f}");
    }
}

/// An `ip` stand-in for the scripts' interface checks: `STUB_DIR/links` lists `NAME KIND` per interface.
fn stub_ip(dir: &Path) {
    let ip = r#"#!/bin/bash
echo "ip $*" >> "$STUB_DIR/log"
[ "$1" = -details ] && shift
if [ "$1 $2" = "link show" ]; then
    kind=$(grep "^$3 " "$STUB_DIR/links" 2>/dev/null | cut -d' ' -f2) || exit 1
    [ -n "$kind" ] || exit 1
    echo "1: $3: <NOARP,UP> mtu 72"; echo "    link/can  $kind"
fi
"#;
    let p = dir.join("ip");
    std::fs::write(&p, ip).unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
}

fn script(stubs: &Path, name: &str, args: &[&str], env: &[(&str, &str)]) -> (i32, String, String) {
    let path = format!("{}:{}", stubs.display(), std::env::var("PATH").unwrap());
    let mut cmd = Command::new("bash");
    cmd.arg(repo().join("scripts").join(name)).args(args).env("PATH", path).env("STUB_DIR", stubs);
    cmd.env_remove("ROS_DISTRO").env("FIRMWARE_SIMULATOR", env!("CARGO_BIN_EXE_firmware_simulator"));
    for (k, v) in env {
        cmd.env(k, v);
    }
    run(&mut cmd)
}

/// `scripts/robot_in_a_box.sh` and `scripts/nereus_firmware_sim.sh` (which need vcan, ROS 2 and nereus to really run)
/// in `--dry-run`: the commands they would run, and the checks that stop them.
#[test]
fn robot_in_a_box_and_nereus_scripts() {
    for s in ["robot_in_a_box.sh", "nereus_firmware_sim.sh", "build_c_boards.sh"] {
        check_bash(&repo().join("scripts").join(s));
        if Command::new("shellcheck").arg("--version").output().is_ok() {
            let (code, out, _) = run(Command::new("shellcheck").arg(repo().join("scripts").join(s)));
            assert_eq!(code, 0, "shellcheck {s}:\n{out}");
        }
    }
    let d = scratch("box");
    stub_ip(&d);
    let tf = d.join("tf");
    let tf = tf.to_str().unwrap();
    // C images: the emulator on can0 and can1, the Orin's micro-ROS agents (no ROS here, so printed)
    let (code, out, err) = script(&d, "robot_in_a_box.sh", &["--dry-run"], &[("TITAN_FIRMWARE_DIR", tf)]);
    assert_eq!(code, 0, "{err}");
    assert!(out.contains("scripts/vcan_up.sh can0 can1"), "{out}");
    assert!(
        out.contains("robots/talos_c.toml --can internal=can0 --realtime") && out.contains("--can external=can1"),
        "{out}"
    );
    assert!(out.contains("ros2 launch riptide_hardware2 copro_agent.launch.py robot:=talos"), "{out}");
    assert!(out.contains("No sourced ROS 2 with the riptide workspace found"), "{out}");
    // Rust images: titan_bridge per board, from the bridges command
    let (code, out, err) = script(&d, "robot_in_a_box.sh", &["--dry-run", "--rust", "--no-vcan"], &[]);
    assert_eq!(code, 0, "{err}");
    assert!(!out.contains("vcan_up") && out.contains("robots/talos.toml --can internal=can0"), "{out}");
    assert!(out.contains("bridges") && out.contains("--format launch"), "{out}");
    // A real CAN adapter called can0 is never used
    std::fs::write(d.join("links"), "can0 mttcan\n").unwrap();
    let (code, _, err) = script(&d, "robot_in_a_box.sh", &["--dry-run"], &[("TITAN_FIRMWARE_DIR", tf)]);
    assert_eq!(code, 1);
    assert!(err.contains("can0 exists and is not a vcan interface"), "{err}");
    std::fs::write(d.join("links"), "can0 vcan\ncan1 vcan\n").unwrap();
    let (code, _, err) = script(&d, "robot_in_a_box.sh", &["--dry-run", "--no-vcan"], &[("TITAN_FIRMWARE_DIR", tf)]);
    assert_eq!(code, 0, "{err}");
    // A robot file without the external bus leaves can1 alone
    let (code, out, err) =
        script(&d, "robot_in_a_box.sh", &["--dry-run", "--no-vcan", "--robot", "robots/talos_c_nereus.toml"], &[]);
    assert_eq!(code, 0, "{err}");
    assert!(out.contains("--dashboard 127.0.0.1:8080") && !out.contains("external=can1"), "{out}");
    let (code, _, err) = script(&d, "robot_in_a_box.sh", &["--stack", "maybe"], &[]);
    assert_eq!(code, 2, "{err}");
    let (code, _, err) = script(&d, "robot_in_a_box.sh", &[], &[]);
    assert!(code == 1 && err.contains("TITAN_FIRMWARE_DIR is not set"), "{err}");

    // nereus: a stand-in checkout with the launch file it includes
    let nereus = d.join("nereus");
    std::fs::create_dir_all(nereus.join("integrations/uwrt/launch")).unwrap();
    std::fs::write(nereus.join("integrations/uwrt/launch/sim.launch.py"), "").unwrap();
    let n = nereus.to_str().unwrap();
    let (code, out, err) = script(
        &d,
        "nereus_firmware_sim.sh",
        &["--dry-run", "--nereus", n, "--pool", "rpac"],
        &[("TITAN_FIRMWARE_DIR", tf)],
    );
    assert_eq!(code, 0, "{err}");
    // What's missing is listed, and the commands are printed anyway
    assert!(err.contains("nereus: not built") && err.contains("ROS 2: not sourced"), "{err}");
    assert!(out.contains("make_packs.py") && out.contains("--pool rpac"), "{out}");
    assert!(out.contains("robots/talos_c_nereus.toml --can internal=can0 --realtime"), "{out}");
    assert!(out.contains("--dashboard 127.0.0.1:8080"), "{out}");
    assert!(out.contains("nereus_firmware.launch.py") && out.contains("scenarios/talos_firmware_rpac"), "{out}");
    assert!(!out.contains("firmware:=rust"), "{out}");
    let (code, out, err) =
        script(&d, "nereus_firmware_sim.sh", &["--dry-run", "--nereus", n, "--rust", "--no-vcan"], &[]);
    assert_eq!(code, 0, "{err}");
    assert!(out.contains("robots/talos_nereus.toml") && out.contains("firmware:=rust"), "{out}");
    assert!(out.contains("bridges_launch:="), "{out}");
    // Without --dry-run the missing pieces stop it before anything starts
    let (code, out, err) = script(&d, "nereus_firmware_sim.sh", &["--nereus", n], &[]);
    assert_eq!(code, 1, "{out}");
    assert!(err.contains("TITAN_FIRMWARE_DIR is not set") && !out.contains("+ "), "{err}{out}");
    let (code, _, _) = script(&d, "nereus_firmware_sim.sh", &["--bogus"], &[]);
    assert_eq!(code, 2);
    let _ = std::fs::remove_dir_all(&d);
}
