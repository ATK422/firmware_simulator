//! `firmware_simulator scenario` and `firmware_simulator compare` end to end: exit statuses, the text, JUnit XML,
//! JSON and Markdown they write, `--check`, image overrides and `--fail-on-regression`.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("scenario-cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn sim(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_firmware_simulator")).args(args).output().unwrap()
}

fn text(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned() + &String::from_utf8_lossy(&o.stderr)
}

fn path(p: &Path) -> &str {
    p.to_str().unwrap()
}

/// A scenario file in a temporary directory, with the repository's board files reachable by absolute path.
fn scenario_file(name: &str, body: &str) -> PathBuf {
    let p = temp(name);
    let board = repo().join("boards/mk2/power_board.toml");
    std::fs::write(&p, body.replace("BOARD_FILE", path(&board))).unwrap();
    p
}

#[test]
fn check_lists_every_shipped_scenario_without_running_it() {
    let o = sim(&["scenario", "--check", path(&repo().join("scenarios"))]);
    let t = text(&o);
    assert!(o.status.success(), "{t}");
    assert!(t.contains("Talos boots and every board connects"), "{t}");
    assert!(t.contains("case \"sensor unplugged\":"), "{t}");
    assert!(t.contains("expect camera_cage log \"Lost connection to ROS\" within 1s"), "{t}");
    assert!(t.contains("scenario(s) in 8 file(s): OK"), "{t}");
}

#[test]
fn a_bad_file_is_an_error_that_names_the_step() {
    let p = scenario_file(
        "bad.yaml",
        "name: bad\nboard: {image: power_board, file: BOARD_FILE}\ntime: 1s\nsteps:\n  - expect: {log: x, withn: 1s}\n",
    );
    let o = sim(&["scenario", path(&p)]);
    assert_eq!(o.status.code(), Some(2), "{}", text(&o));
    assert!(text(&o).contains("step 1: expect: the condition: unknown key `withn`"), "{}", text(&o));
}

#[test]
fn a_failing_scenario_exits_with_1_and_writes_junit_and_json() {
    if sim_host::titan_rs_image(&repo(), "power_board").is_none() {
        return;
    }
    let p = scenario_file(
        "fails.yaml",
        "name: one passes, one fails
board: {image: power_board, file: BOARD_FILE}
bridges: [1]
time: 1s
steps:
  - expect: {log: ROS connected, within: 500ms}
  - expect: {log: nothing like this, within: 200ms}
---
name: passes
board: {image: power_board, file: BOARD_FILE}
bridges: [1]
time: 1.5s
steps:
  - expect: {status: {faults: 0, max_age: 2s}, at: 1.5s}
",
    );
    let (junit, json) = (temp("fails.xml"), temp("fails.json"));
    let o = sim(&["scenario", path(&p), "--junit", path(&junit), "--json", path(&json), "-j", "2"]);
    let t = text(&o);
    assert_eq!(o.status.code(), Some(1), "{t}");
    assert!(t.contains("✓      0.200400s  expect log \"ROS connected\" within 500ms"), "{t}");
    assert!(t.contains("✗      0.400400s  expect log \"nothing like this\" within 200ms"), "{t}");
    assert!(t.contains("2 scenario(s): 1 passed, 1 failed, 0 skipped (one passes, one fails)"), "{t}");
    let xml = std::fs::read_to_string(&junit).unwrap();
    assert!(xml.contains("tests=\"2\" failures=\"1\""), "{xml}");
    assert!(
        xml.contains("<failure message=\"step 2: expect log &quot;nothing like this&quot; within 200ms\"/>"),
        "{xml}"
    );
    let j: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&json).unwrap()).unwrap();
    assert_eq!((j["passed"].as_u64(), j["failed"].as_u64()), (Some(1), Some(1)));
    assert_eq!(j["scenarios"][0]["steps"][0]["time"], 0.2004);
    // JSON on stdout instead of the text; --quiet leaves out what passed
    let o = sim(&["scenario", path(&p), "--json", "-"]);
    let j: serde_json::Value = serde_json::from_slice(&o.stdout).unwrap();
    assert_eq!(j["scenarios"].as_array().unwrap().len(), 2);
    let o = sim(&["scenario", path(&p), "-q"]);
    assert!(!text(&o).contains("● passes"), "{}", text(&o));
}

#[test]
fn compare_reports_two_builds_and_fails_on_a_regression() {
    let (Some(_), Some(template)) =
        (sim_host::titan_rs_image(&repo(), "power_board"), sim_host::titan_rs_image(&repo(), "template"))
    else {
        return;
    };
    // The power board publishes its electrical readings; the template image, on the same board, does not
    let p = scenario_file(
        "ab.yaml",
        "name: electrical readings
board: {image: power_board, file: BOARD_FILE, name: power}
bridges: [1]
time: 2.5s
steps:
  - expect: {log: ROS connected, within: 1s}
  - expect: {message: Power.Electrical.port_voltage, value: [15, 17], within: 2s}
",
    );
    let (md, json) = (temp("ab.md"), temp("ab.json"));
    let b = format!("power={}", path(&template));
    let o = sim(&["compare", "--b", &b, "--b-label", "template", path(&p), "-o", path(&md), "--json", path(&json)]);
    let t = text(&o);
    assert!(o.status.success(), "{t}");
    assert!(t.contains("1 comparison(s), 1 regression(s)"), "{t}");
    let report = std::fs::read_to_string(&md).unwrap();
    assert!(report.contains("## Firmware A/B: electrical readings"), "{report}");
    assert!(report.contains("**A**: the scenario's images · **B**: template"), "{report}");
    assert!(report.contains("| 2 | expect message Power.Electrical.port_voltage in 15..17 within 2s | ✓"), "{report}");
    assert!(report.contains("Power::Electrical"), "{report}");
    let j: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&json).unwrap()).unwrap();
    assert_eq!(j["comparisons"][0]["differs"], true);
    assert_eq!(j["comparisons"][0]["steps"][1]["b"]["status"], "fail");
    let o = sim(&["compare", "--b", &b, path(&p), "--fail-on-regression", "-o", path(&md)]);
    assert_eq!(o.status.code(), Some(1), "{}", text(&o));
}
