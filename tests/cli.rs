//! The command line's developer tools end to end: `run` and `robot` with `--profile`, `--vcd`, `--log-json`,
//! `--trace` for chosen robot boards, on the hand-written test programs (no firmware build needed).

use std::path::{Path, PathBuf};
use std::process::Command;

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn sim(args: &[&str]) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator")).args(args).output().unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    assert!(out.status.success(), "{args:?} failed:\n{stderr}");
    String::from_utf8_lossy(&out.stdout).into_owned() + &stderr
}

fn path(p: &Path) -> &str {
    p.to_str().unwrap()
}

#[test]
fn run_writes_a_profile_a_waveform_and_a_json_log() {
    let prof = temp("run.txt");
    let vcd = temp("run.vcd");
    let json = temp("run.jsonl");
    let elf = repo().join("test_programs/pico/hello_uart.elf");
    let out = sim(&[
        "run",
        path(&elf),
        "--time",
        "20ms",
        "--profile",
        path(&prof),
        "--profile-interval",
        "5us",
        "--vcd",
        path(&vcd),
        "--vcd-pins",
        "0,1,25",
        "--vcd-events",
        "--log-json",
        path(&json),
    ]);
    assert!(out.contains("uart0: AB Hello, UART!"), "{out}");
    // The profile: both cores, the pico-sdk program's functions
    let table = std::fs::read_to_string(&prof).unwrap();
    assert!(table.contains("## core 0:") && table.contains("## core 1:"), "{table}");
    assert!(table.contains("main") || table.contains("runtime_init"), "{table}");
    let folded = std::fs::read_to_string(prof.with_extension("folded")).unwrap();
    assert!(folded.lines().all(|l| l.starts_with("core") && l.rsplit_once(' ').unwrap().1.parse::<u64>().is_ok()));
    // The waveform: the UART TX pin and the line it sent
    let v = sim_host::vcd::parse(&std::fs::read_to_string(&vcd).unwrap()).unwrap();
    assert!(v.signal("board", "gpio0").is_some() && v.signal("board", "gpio25").is_some());
    assert!(v.signal("board", "gpio2").is_none(), "only the chosen pins");
    let uart = v.signal("board", "uart0").unwrap();
    assert!(uart.iter().any(|(_, l)| l == "AB_Hello,_UART!"), "{uart:?}");
    // The JSON log: the UART line and the end of the run
    let lines: Vec<serde_json::Value> =
        std::fs::read_to_string(&json).unwrap().lines().map(|l| serde_json::from_str(l).unwrap()).collect();
    assert!(lines.iter().any(|l| l["kind"] == "uart" && l["line"] == "AB Hello, UART!" && l["uart"] == 0), "{lines:?}");
    let end = lines.last().unwrap();
    assert_eq!((&end["kind"], &end["reason"]), (&"end".into(), &"TimeLimit".into()));
}

/// `picoboot load FILE --profile`: the machine booted a blank flash, so the profile names what was flashed.
#[test]
fn a_profile_after_picoboot_load_names_the_flashed_image() {
    let prof = temp("picoboot.txt");
    let elf = repo().join("test_programs/pico/hello_uart.elf");
    let out = sim(&["picoboot", "load", path(&elf), "--time", "100ms", "--profile", path(&prof)]);
    assert!(out.contains("uart0: AB Hello, UART!"), "{out}");
    let table = std::fs::read_to_string(&prof).unwrap();
    assert!(table.starts_with("# hello_uart.elf:"), "{table}");
    assert!(table.contains(" _reset_handler\n") && table.contains(" xosc_init\n"), "{table}");
}

#[test]
fn robot_traces_chosen_boards_and_profiles_and_records_every_board() {
    let blink = repo().join("test_programs/blink.elf");
    let board = repo().join("boards/mk2/power_board.toml");
    let robot = temp("two.toml");
    std::fs::write(
        &robot,
        format!(
            "[[board]]\nname = \"a\"\nboard = {b:?}\nelf = {e:?}\n\n[[board]]\nname = \"b\"\nboard = {b:?}\nelf = {e:?}\npower_on_ms = 2\n",
            b = path(&board),
            e = path(&blink)
        ),
    )
    .unwrap();
    let trace = temp("two.trace");
    let dir = temp("prof");
    let vcd = temp("two.vcd");
    let out = sim(&[
        "robot",
        path(&robot),
        "--time",
        "10ms",
        "--trace",
        "gpio",
        "--trace-board",
        "b",
        "--trace-file",
        path(&trace),
        "--profile-dir",
        path(&dir),
        "--vcd",
        path(&vcd),
        "--vcd-pins",
        "25",
    ]);
    assert!(out.contains("profile of a") && out.contains("profile of b"), "{out}");
    let t = std::fs::read_to_string(&trace).unwrap();
    assert!(!t.is_empty() && t.lines().all(|l| l.starts_with("b: ")), "only board b is traced:\n{t}");
    assert!(t.contains("GPIO25 = 1"));
    for b in ["a", "b"] {
        let p = std::fs::read_to_string(dir.join(format!("{b}.txt"))).unwrap();
        // blink.S has neither symbols nor CFI: the samples land on its delay loop's addresses
        assert!(p.contains("## core 0:") && p.contains("0x10000028"), "{p}");
        assert!(!p.contains("could not be walked"), "{p}");
        assert!(dir.join(format!("{b}.folded")).exists());
    }
    let v = sim_host::vcd::parse(&std::fs::read_to_string(&vcd).unwrap()).unwrap();
    let a = v.signal("a", "gpio25").unwrap();
    let b = v.signal("b", "gpio25").unwrap();
    assert!(a.len() > 10 && b.len() > 10);
    // Board b was switched on 2 ms later
    let first = |s: &[(u64, String)]| s.iter().find(|c| c.1 == "1").unwrap().0;
    assert_eq!(first(b) - first(a), 2_000_000);
}

#[test]
fn bad_pin_names_are_reported() {
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["run", path(&repo().join("test_programs/blink.elf")), "--time", "1ms", "--vcd"])
        .arg(temp("bad.vcd"))
        .args(["--board", path(&repo().join("boards/mk2/esc_board.toml")), "--vcd-pins", "esc9_pwm"])
        .output()
        .unwrap();
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("no pin \"esc9_pwm\"") && err.contains("esc1_pwm_pin"), "{err}");
}

/// A robot with two boards, the second `c_dual_usb` (titan_dual_stdio_usb) with `extra` robot file keys.
fn dual_usb_robot(name: &str, extra: &str) -> PathBuf {
    let robot = temp(name);
    std::fs::write(
        &robot,
        format!(
            "[[board]]\nname = \"blinky\"\nboard = {pb:?}\nelf = {blink:?}\n\n\
             [[board]]\nname = \"c_ivc\"\nboard = {ivc:?}\nelf = {dual:?}\n{extra}\n",
            pb = path(&repo().join("boards/mk2/power_board.toml")),
            blink = path(&repo().join("test_programs/blink.elf")),
            ivc = path(&repo().join("boards/talos/ivc_board.toml")),
            dual = path(&repo().join("test_programs/c_dual_usb/c_dual_usb.elf")),
        ),
    )
    .unwrap();
    robot
}

/// `robot --usb-cdc BOARD=stdout` prints the board's USB text log with its name; `--usb-cdc2 BOARD=pty:PATH` puts
/// its second port on a terminal linked from PATH, and replaces the robot file's setting.
#[test]
fn robot_usb_cdc_ports_from_the_command_line() {
    let link = temp("c_ivc-ros");
    let robot = dual_usb_robot("usb.toml", "usb_cdc2 = \"pty\"");
    let out = sim(&[
        "robot",
        path(&robot),
        "--time",
        "300ms",
        "--usb-cdc",
        "c_ivc=stdout",
        "--usb-cdc2",
        &format!("c_ivc=pty:{}", path(&link)),
    ]);
    assert!(out.contains("] c_ivc usb: titan dual stdio up"), "{out}");
    assert!(out.contains("c_ivc: USB CDC port 1 on /dev/") && out.contains(&format!("(linked from {})", path(&link))));
    assert_eq!(out.matches("USB CDC port").count(), 1, "the command line replaces the file's pty: {out}");
    // Mistakes are reported
    for (args, want) in [
        (&["--usb-cdc2", "c_ivc=stdout"][..], "raw bytes"),
        (&["--usb-cdc", "nope=stdout"], "--usb-cdc: no board \"nope\""),
        (&["--usb-cdc", "c_ivc"], "expected BOARD=SINK"),
        (&["--ivc-send", "1s@blinky:0x5a"], "not an IVC image"),
        (&["--ivc-send", "1s@blinky:300"], "0-255 or 0x00-0xff"),
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
            .args(["robot", path(&robot), "--time", "1ms"])
            .args(args)
            .output()
            .unwrap();
        let err = String::from_utf8_lossy(&out.stderr);
        assert!(!out.status.success() && err.contains(want), "{args:?}: {err}");
    }
}

/// `robot --ivc-send`: Talos's IVCBoard2 transmits a byte through its mock bridge, and the surface vehicle's mock
/// bridge prints it as it arrives across the pool.
#[test]
fn robot_ivc_send_reaches_the_other_vehicle() {
    let fw = sim_host::titan_rs_release_dir(&repo()).join("ivc_board2");
    if !fw.exists() {
        eprintln!("skipping: {} not built", fw.display());
        return;
    }
    let robot = repo().join("robots/ivc2_pool.toml");
    let out = sim(&[
        "robot",
        path(&robot),
        "--mock-bridge",
        "talos:8",
        "--mock-bridge",
        "surface:8",
        "--ivc-send",
        "3.5s@talos_ivc:0x5a",
        "--time",
        "12.5s",
    ]);
    assert!(out.contains("talos_ivc: ") && out.contains("Transmitting 0x5a"), "{out}");
    let got: Vec<&str> = out.lines().filter(|l| l.contains("IVC received")).collect();
    assert_eq!(got.len(), 1, "{out}");
    assert!(got[0].starts_with('[') && got[0].ends_with("] surface_ivc: IVC received 0x5a"), "{got:?}");
    // Sent at 3.5 s, nine seconds of packet: it arrives as the packet's closing sync starts
    let t: f64 = got[0][1..got[0].find("s]").unwrap()].parse().unwrap();
    assert!((11.5..12.5).contains(&t), "{t}");
    // Without a mock bridge on the board's bus the command says which to add
    let err = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["robot", path(&robot), "--ivc-send", "1s@surface_ivc:1", "--time", "1ms"])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&err.stderr);
    assert!(err.contains("surface_ivc has no mock bridge (add --mock-bridge surface:8)"), "{err}");
}

/// `robots/ivc2_interop.toml` loads with the C image from `$TITAN_FIRMWARE_DIR` (a stand-in here: `c_dual_usb`,
/// titan_dual_stdio_usb like the C IVCBoard2, at the C build's path), boots it through the bootrom and opens its
/// two USB ports; the command line moves the ptys away from the file's fixed paths.
#[test]
fn ivc2_interop_robot_takes_the_c_image_from_titan_firmware_dir() {
    if !sim_host::titan_rs_release_dir(&repo()).join("ivc_board2").exists() {
        eprintln!("skipping: ivc_board2 not built");
        return;
    }
    let tf = temp("titan_firmware");
    let build = tf.join("talos/IVCBoard2/build");
    std::fs::create_dir_all(&build).unwrap();
    std::fs::copy(repo().join("test_programs/c_dual_usb/c_dual_usb.elf"), build.join("IVCBoard2_with_bl.uf2")).unwrap();
    let ros = temp("interop-ros");
    let robot = repo().join("robots/ivc2_interop.toml");
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["robot", path(&robot), "--mock-bridge", "talos:8", "--time", "800ms", "--usb-cdc", "c_ivc=stdout"])
        .arg("--usb-cdc2")
        .arg(format!("c_ivc=pty:{}", path(&ros)))
        .env("TITAN_FIRMWARE_DIR", &tf)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "{text}");
    assert!(text.contains("] c_ivc usb: titan dual stdio up"), "{text}");
    assert!(text.contains("c_ivc: USB CDC port 1 on /dev/") && text.contains(path(&ros)), "{text}");
    assert!(text.contains("talos_ivc: ") && text.contains("ROS connected"), "{text}");
    // Without the variable the error says what to set
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["robot", path(&robot), "--time", "1ms"])
        .env_remove("TITAN_FIRMWARE_DIR")
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success() && err.contains("TITAN_FIRMWARE_DIR is not set"), "{err}");
}
