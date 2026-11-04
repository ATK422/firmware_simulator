//! The YAML scenarios in `scenarios/` (docs/scenarios.md), run as part of `cargo test`; the runner's verdicts,
//! evidence and outputs on scenarios written here; determinism; and firmware A/B comparisons of real runs.

use std::path::PathBuf;

use sim_test::compare::{CompareOptions, compare};
use sim_test::scenario::{self, ImageOverride, Options, Outcome, Status};

fn scenarios_dir() -> PathBuf {
    sim_test::repo().join("scenarios")
}

/// Every shipped scenario passes with the current images (each skips when its images are not built).
#[test]
fn the_shipped_scenarios_pass() {
    scenario::assert_pass(&[scenarios_dir()], 0);
}

fn run_text(text: &str) -> Vec<Outcome> {
    let s = scenario::parse_str(text, &scenarios_dir().join("inline.yaml")).unwrap();
    scenario::run_all(&s, &Options { record: true, ..Default::default() }, 2)
}

const POWER: &str = "board: {image: power_board, file: mk2/power_board}\nbridges: [1]\n";

/// A scenario that fails in each way a step can, and what the runner says about it.
#[test]
fn failures_name_the_step_and_show_the_evidence() {
    if sim_test::titan_rs("power_board").is_none() {
        return;
    }
    let o = &run_text(&format!(
        "name: failing on purpose
{POWER}time: 3s
steps:
  - expect: {{log: ROS connected, within: 1s}}
  - expect: {{log: this line never comes, within: 500ms}}
  - fault: can-unplug
  - expect: {{status: {{faults: 0}}, after: 1s}}
  - expect: {{link: up, within: 3s}}
  - expect: {{log: never reached}}
  - never: {{log: Lost connection to ROS}}
  - always: {{link: up, at: 0.5s, until: 2s}}
  - fault: {{board: nope, fault: can-stuck}}
"
    ))[0];
    assert!(!o.passed());
    let st = |n: usize| o.steps.iter().find(|s| s.number == n).unwrap();
    assert_eq!(st(1).status, Status::Pass);
    // No such line: the window, then the board's last lines
    assert_eq!(st(2).status, Status::Fail);
    assert!(st(2).evidence[0].contains("no matching line from 0.200400s to 0.700400s"), "{:?}", st(2).evidence);
    assert!(st(2).evidence.iter().any(|e| e.contains("ROS connected")), "{:?}", st(2).evidence);
    // The fault goes in at the cursor (the end of step 2's window, on the 100 us slice grid)
    assert_eq!((st(3).status, st(3).time), (Status::Pass, Some(sim_test::Time::from_us(700_400))));
    // Unplugged before its first status went out: a second later the bridge has none to look at
    assert_eq!(st(4).status, Status::Fail);
    assert!(st(4).evidence[0].contains("no FirmwareStatus at can:1"), "{:?}", st(4).evidence);
    // The link never comes back within the run (the stalled transmit queue): the run ends first
    assert_eq!(st(5).status, Status::Fail);
    assert!(st(5).evidence[0].contains("the run ended at 3.000000s before the window closed"), "{:?}", st(5).evidence);
    assert_eq!(st(6).status, Status::NotReached);
    // Watched from the start, whatever holds the timeline up
    assert_eq!(st(7).status, Status::Fail);
    assert!(st(7).evidence[0].contains("Lost connection to ROS"));
    assert_eq!(st(8).status, Status::Fail);
    assert!(st(8).evidence[0].contains("down at 0.9"), "{:?}", st(8).evidence);
    assert_eq!(st(9).status, Status::NotReached);
    let text = scenario::human(o);
    assert!(text.contains("✗") && text.contains("·") && text.contains("FAIL (7 of 9 steps)"), "{text}");
    let xml = scenario::junit(std::slice::from_ref(o));
    assert!(xml.contains("failures=\"1\"") && xml.contains("<failure message=\"step 2: expect log"), "{xml}");
    let j = scenario::json(std::slice::from_ref(o));
    assert_eq!(j["failed"], 1);
    assert_eq!(j["scenarios"][0]["steps"][1]["status"], "fail");
}

/// A setup that cannot work is an error of the scenario, not a panic.
#[test]
fn setup_errors_are_reported() {
    let o = &run_text(&format!("name: x\n{POWER}inputs: {{no_such_input: 1}}\ntime: 1s\n"))[0];
    if o.skipped.is_some() {
        return;
    }
    assert!(o.error.as_deref().is_some_and(|e| e.contains("no input \"no_such_input\"")), "{:?}", o.error);
    let o = &run_text("name: x\nboard: {image: no_such_image, file: mk2/power_board}\ntime: 1s\n")[0];
    assert!(o.error.as_deref().is_some_and(|e| e.contains("is not built")), "{:?}", o.error);
    let s = scenario::parse_str(
        "name: x\nboard: {image: no_such_image, file: mk2/power_board}\ntime: 1s\n",
        &scenarios_dir().join("inline.yaml"),
    )
    .unwrap();
    let skipped = &scenario::run(&s[0], &Options { skip_missing: true, ..Default::default() })[0];
    if !sim_host::titan_rs_required() {
        assert!(skipped.skipped.is_some());
    }
}

/// The same files give the same results, step for step and time for time, on any number of threads.
#[test]
fn runs_are_deterministic() {
    if sim_test::titan_rs("poac_board").is_none() || sim_test::titan_rs("power_board").is_none() {
        return;
    }
    let files = [scenarios_dir().join("crash_record.yaml"), scenarios_dir().join("kill_switch.yaml")];
    let all = scenario::load(&files).unwrap();
    let strip = |o: &[Outcome]| {
        let mut j = scenario::json(o);
        for s in j["scenarios"].as_array_mut().unwrap() {
            s["wall_s"] = serde_json::Value::Null;
        }
        j
    };
    let opts = Options::default();
    let one = scenario::run_all(&all, &opts, 1);
    let four = scenario::run_all(&all, &opts, 4);
    assert_eq!(strip(&one), strip(&four));
    assert_eq!(strip(&one), strip(&scenario::run_all(&all, &opts, 2)));
    assert!(one.iter().all(Outcome::passed));
}

fn power_board_run(images: Vec<ImageOverride>) -> Outcome {
    let s = scenario::parse_str(
        &format!(
            "name: power board
{POWER}time: 4s
steps:
  - expect: {{log: ROS connected, within: 1s}}
  - at: 2s
    fault: bridge-stop
  - expect: {{log: Lost connection to ROS, within: 1s}}
  - at: 3s
    fault: bridge-start
  - expect: {{log: ROS connected, within: 500ms}}
"
        ),
        &scenarios_dir().join("inline.yaml"),
    )
    .unwrap();
    scenario::run(&s[0], &Options { images, record: true, ..Default::default() }).remove(0)
}

/// The same build against itself: nothing differs, and the report says so.
#[test]
fn a_build_compared_with_itself_has_no_differences() {
    if sim_test::titan_rs("power_board").is_none() {
        return;
    }
    let (a, b) = (power_board_run(Vec::new()), power_board_run(Vec::new()));
    let c = compare(&a, &b, ("A", "B"), &CompareOptions::default());
    assert!(!c.differs(), "{}", c.markdown());
    assert!(c.markdown().contains("Every board logged the same lines"));
}

/// Two builds that differ for real: the power board image against the template image on the same board (another
/// firmware with the same board name and client). The report sees the differences in lines, faults and traffic.
#[test]
fn two_firmware_builds_compared() {
    let (Some(_), Some(template)) = (sim_test::titan_rs("power_board"), sim_test::titan_rs("template")) else {
        return;
    };
    let a = power_board_run(Vec::new());
    let b = power_board_run(vec![ImageOverride::Board("power_board".into(), template.clone())]);
    assert_eq!(b.images[0].1, template);
    assert!(a.passed() && b.passed(), "{}{}", scenario::human(&a), scenario::human(&b));
    let c = compare(&a, &b, ("power_board", "template"), &CompareOptions::default());
    assert!(c.differs());
    assert_ne!(c.boards[0].hashes.0, c.boards[0].hashes.1);
    let md = c.markdown();
    assert!(md.contains("**Images that differ:** `power_board`"), "{md}");
    // The power board publishes its electrical readings; the template does not
    let bus = &c.buses[0];
    let s =
        bus.report.streams.iter().find(|s| s.name.contains("Power::Electrical")).expect("the power board's readings");
    assert!(s.a_count > 0 && s.b_count == 0, "{s:?}");
    assert!(md.contains("Power::Electrical"), "{md}");
    let j = c.json();
    assert_eq!(j["differs"], true);
}
