//! Scenarios with mock agents (`agents`) and topic steps (`send: {topic, msg}`, `expect: {topic, msg}`): the
//! nano-ROS test board (test_programs/nanoros_led, committed) through its agent, and a titan_firmware_rs board's
//! titan_link messages through the same syntax on its mock bridge.

use std::path::PathBuf;

use serde_json::json;
use sim_test::scenario::{self, Options, Outcome, Status, msg_mismatch};

fn dir() -> PathBuf {
    sim_test::repo().join("scenarios")
}

fn run_text(text: &str) -> Vec<Outcome> {
    let s = scenario::parse_str(text, &dir().join("inline.yaml")).unwrap();
    scenario::run_all(&s, &Options::default(), 2)
}

fn parse_err(text: &str) -> String {
    scenario::parse_str(text, &dir().join("inline.yaml")).unwrap_err()
}

const LED: &str = "robot: robots/nanoros_led.toml\nagents: [nanoros:led]\ntime: 1.5s\n";

#[test]
fn agents_and_topic_steps_parse() {
    let s = &scenario::parse_str(
        "name: x\nrobot: robots/nanoros_led.toml\n\
         agents: [nanoros:internal:led=../test_programs/nanoros_led/ros.toml, titan_link:led]\ntime: 1s\nsteps:\n\
         \x20 - send: {board: led, topic: bit_sub, msg: {data: 1}, every: 10ms, within: 50ms}\n\
         \x20 - expect: {topic: imu_data, msg: {orientation: {w: {min: 0.9}}}, count: {min: 2}, within: 1s}\n",
        &dir().join("inline.yaml"),
    )
    .unwrap()[0];
    assert_eq!(s.agents.len(), 2);
    let ros = s.agents[0].config.as_ref().unwrap();
    assert!(ros.is_absolute() && ros.ends_with("test_programs/nanoros_led/ros.toml"), "{}", ros.display());
    assert_eq!(scenario::describe(&s.steps[0]), "send led bit_sub {\"data\":1} every 10ms within 50ms");
    assert!(scenario::describe(&s.steps[1]).starts_with("expect topic imu_data {\"orientation\""));
    let head = "name: x\nrobot: robots/nanoros_led.toml\ntime: 1s\n";
    assert!(parse_err(&format!("{head}agents: [mqtt:led]\n")).contains("unknown protocol"));
    assert!(parse_err(&format!("{head}agents: [nanoros:led=nope.toml]\n")).contains("no file nope.toml"));
    assert!(parse_err(&format!("{head}agents: nanoros:led\n")).contains("expected a list"));
    let step = |s: &str| parse_err(&format!("{head}steps:\n  - {s}\n"));
    assert!(step("send: {topic: bit_sub, every: 10ms}").contains("`every` needs"));
    assert!(step("send: {topic: bit_sub, within: 1s}").contains("only go with `every`"));
    assert!(step("send: {topic: bit_sub, message: {}}").contains("message"));
    assert!(step("expect: {log: x, msg: {}}").contains("`msg` goes with `topic`"));
}

#[test]
fn message_patterns() {
    let m = json!({"a": 1.5, "b": {"c": "x", "d": [1, 2]}, "f": 9.8100004196167});
    assert_eq!(msg_mismatch(&json!({}), &m), None);
    assert_eq!(msg_mismatch(&json!({"a": 1.5, "b": {"d": [1, 2]}}), &m), None);
    assert_eq!(msg_mismatch(&json!({"f": 9.81}), &m), None, "a float32's rounding");
    assert_eq!(msg_mismatch(&json!({"a": {"min": 1, "max": 2}}), &m), None);
    assert_eq!(msg_mismatch(&json!({"a": {"max": 1}}), &m).unwrap(), "a is 1.5, not in [-inf, 1]");
    assert_eq!(msg_mismatch(&json!({"b": {"c": "y"}}), &m).unwrap(), "b.c is \"x\", not \"y\"");
    assert_eq!(msg_mismatch(&json!({"b": {"d": [1, 3]}}), &m).unwrap(), "b.d[1] is 2, not 3");
    assert!(msg_mismatch(&json!({"b": {"d": [1]}}), &m).unwrap().contains("b.d is [1,2]"));
    assert_eq!(msg_mismatch(&json!({"z": 1}), &m).unwrap(), "no field z");
    assert!(msg_mismatch(&json!({"b": {"min": 0}}), &m).unwrap().contains("not a number"));
}

/// What the scenario runner says when a topic check fails, and a send it refuses.
#[test]
fn topic_failures_show_the_messages() {
    let o = &run_text(&format!(
        "name: failing on purpose\n{LED}steps:\n\
         \x20 - expect: {{topic: firmware_status, msg: {{board_name: other}}, within: 1.2s}}\n\
         \x20 - expect: {{topic: no_such_topic, within: 10ms}}\n\
         \x20 - send: {{topic: firmware_status}}\n\
         \x20 - send: {{topic: bit_sub, msg: {{data: 300}}}}\n\
         \x20 - expect: {{topic: imu_data, msg: {{header: {{frame_id: imu_link}}}}, count: {{min: 5}}, within: 50ms}}\n"
    ))[0];
    let st = |n: usize| o.steps.iter().find(|s| s.number == n).unwrap();
    assert_eq!(st(1).status, Status::Fail);
    assert!(st(1).evidence[0].contains("board_name is \"nanoros_led\", not \"other\""), "{:?}", st(1).evidence);
    assert_eq!(st(2).status, Status::Fail);
    assert!(st(2).evidence[0].contains("no topic \"no_such_topic\""), "{:?}", st(2).evidence);
    assert!(st(2).evidence[0].contains("bit_sub"), "{:?}", st(2).evidence);
    assert_eq!(st(3).status, Status::Fail);
    assert!(st(3).evidence[0].contains("publishers"), "{:?}", st(3).evidence);
    assert_eq!(st(4).status, Status::Fail);
    assert!(st(4).evidence[0].contains("out of range"), "{:?}", st(4).evidence);
    assert_eq!(st(5).status, Status::Fail, "an Imu every 100 ms: not five in 50 ms");
}

/// The same steps on a titan_firmware_rs board: its mock bridge as the agent, titan_link variants as topics.
#[test]
fn titan_link_through_topic_steps() {
    if sim_test::titan_rs("power_board").is_none() {
        return;
    }
    let o = &run_text(
        "name: titan_link as topics
board: {image: power_board, file: mk2/power_board}
agents: [titan_link:1]
time: 2.5s
steps:
  - expect: {log: ROS connected, within: 1s}
  - expect: {topic: FirmwareStatus, msg: {client_id: 1, faults: 0, uptime_ms: {min: 900}}, within: 1.5s}
  - send: {topic: KillSwitch, msg: {asserting_kill: false}, every: 100ms, until: 2s}
  - send: {topic: NoSuchVariant}
  - expect: {topic: Heartbeat, count: {min: 5}, at: 0.5s, until: 2.5s}
  - never: {topic: FirmwareStatus, msg: {faults: {min: 1}}}
",
    )[0];
    let st = |n: usize| o.steps.iter().find(|s| s.number == n).unwrap();
    for n in [1, 2, 3, 5, 6] {
        assert_eq!(st(n).status, Status::Pass, "{}", scenario::human(o));
    }
    assert!(st(3).evidence[0].contains("message(s) from can:1"), "{:?}", st(3).evidence);
    assert_eq!(st(4).status, Status::Fail);
    assert!(st(4).evidence[0].contains("titan_link HostToDevice NoSuchVariant"), "{:?}", st(4).evidence);
}
