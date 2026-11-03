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
