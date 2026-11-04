//! The nano-ROS mock agent against real libnanoros firmware: `test_programs/nanoros_led` (osu-uwrt/nano_ros's
//! library and nano_ros_codegen's code for its ros.toml, unmodified, on titan_firmware's CAN driver) on the Talos
//! LED board. The board publishes, the agent decodes; the agent publishes, the board reacts (and says so on UART0).

mod common;

use std::sync::{Arc, Mutex};

use serde_json::{Value, json};
use sim_core::time::Time;
use sim_host::host_protocol::{AgentSpec, HostAgent, attach};
use sim_host::robot::Robot;

/// UART lines with their times.
type Lines = Arc<Mutex<Vec<(Time, String)>>>;

/// The robot file's one board with a nano-ROS agent from `nanoros:led` (the robot file's `ros_toml`), and the
/// board's UART lines.
fn led_robot() -> (Robot, Box<dyn HostAgent>, Lines) {
    let mut r = Robot::load(&common::repo().join("robots/nanoros_led.toml")).unwrap();
    let spec: AgentSpec = "nanoros:led".parse().unwrap();
    let (p, agent) = attach(&mut r, &spec, &common::repo()).unwrap();
    assert_eq!((p.board, p.client), (Some(0), 6));
    let lines = Arc::new(Mutex::new(Vec::new()));
    let l = lines.clone();
    r.boards[0].on_uart_line = Some(Box::new(move |t, _, line| l.lock().unwrap().push((t, line.to_string()))));
    (r, agent, lines)
}

fn on(agent: &dyn HostAgent, topic: &str) -> Vec<(Time, Value)> {
    agent.messages(0).into_iter().filter(|m| m.topic == topic).map(|m| (m.t, m.msg)).collect()
}

fn line_after(lines: &Mutex<Vec<(Time, String)>>, text: &str, after: Time) -> Option<(Time, String)> {
    lines.lock().unwrap().iter().find(|(t, l)| *t >= after && l.contains(text)).cloned()
}

#[test]
fn the_board_registers_and_publishes() {
    let (mut r, agent, lines) = led_robot();
    r.run_until(Time::from_ms(1200));
    assert!(agent.connected(r.now), "{:?}", agent.errors());
    assert!(agent.errors().is_empty(), "{:?}", agent.errors());
    let st = agent.status(r.now);
    assert_eq!(st["handshakes"], 1);
    let topics = st["topics"].as_array().unwrap();
    assert_eq!(topics.len(), 5);
    assert!(topics.iter().all(|t| t["registered"] == true && t["in_ros_toml"] == true), "{topics:?}");
    // The first DISCOVER (at 0) came before the board was up; it registered on the next
    let (reg, _) = line_after(&lines, "starting registration", Time::ZERO).unwrap();
    assert!(reg > Time::from_ms(230) && reg < Time::from_ms(240), "{reg}");
    assert!(line_after(&lines, "done reg", reg).is_some());
    let statuses = on(&*agent, "firmware_status");
    assert_eq!(statuses.len(), 1, "one a second");
    let s = &statuses[0].1;
    assert_eq!(
        (s["board_name"].as_str(), s["client_id"].as_u64(), s["bus_id"].as_u64()),
        (Some("nanoros_led"), Some(6), Some(1))
    );
    assert!((1000..1100).contains(&s["uptime_ms"].as_u64().unwrap()), "{s}");
    let imu = on(&*agent, "imu_data");
    assert!(imu.len() >= 8, "every 100 ms from registration: {}", imu.len());
    let (t, m) = imu.last().unwrap();
    assert_eq!(m["header"]["frame_id"], "imu_link");
    assert_eq!(m["orientation"], json!({"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}));
    assert_eq!(m["linear_acceleration"]["z"], 9.81);
    assert_eq!(m["orientation_covariance"][4], 0.01);
    // The stamp is the board's uptime, a little before the agent got it
    let stamp = m["header"]["stamp"]["sec"].as_f64().unwrap() + m["header"]["stamp"]["nanosec"].as_f64().unwrap() / 1e9;
    assert!(t.as_secs_f64() - stamp > 0.0 && t.as_secs_f64() - stamp < 0.1, "{t} {stamp}");
}

#[test]
fn the_agent_publishes_and_the_board_reacts() {
    let (mut r, agent, lines) = led_robot();
    // Sent before the board has registered: it goes once the board has
    agent.send("bit_sub", &json!({"data": 41}), None).unwrap();
    agent.send("bit_sub", &json!({"data": 9}), Some(Time::from_ms(700))).unwrap();
    let cmd = json!({
        "header": {"stamp": {"sec": 12, "nanosec": 345}, "frame_id": "cmd_frame"},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.6, "w": 0.8},
    });
    agent.send("imu_cmd", &cmd, Some(Time::from_ms(800))).unwrap();
    assert!(agent.send("imu_data", &Value::Null, None).unwrap_err().contains("publishers"));
    r.run_until(Time::from_ms(1100));
    assert!(agent.errors().is_empty(), "{:?}", agent.errors());
    let (t41, _) = line_after(&lines, "bit_sub 41", Time::ZERO).expect("the board got 41");
    // Soon after registering (libnanoros prints each payload it publishes in hex on UART0 first, ~60 ms for an Imu)
    assert!(t41 > Time::from_ms(230) && t41 < Time::from_ms(400), "{t41}");
    let (t9, _) = line_after(&lines, "bit_sub 9", Time::from_ms(700)).expect("the board got 9");
    assert!(t9 < Time::from_ms(720), "{t9}");
    let echoes: Vec<u64> = on(&*agent, "bit_echo").iter().map(|(_, m)| m["data"].as_u64().unwrap()).collect();
    assert_eq!(echoes, [42, 10]);
    // An Imu to the board: float64s after a string, aligned from the body's start as the board deserializes it
    let (t, l) = line_after(&lines, "imu_cmd", Time::from_ms(800)).expect("the board got the Imu");
    assert!(l.contains("frame_id cmd_frame stamp 12.000000345 orientation 0.000 0.000 0.600 0.800"), "{l}");
    let next = on(&*agent, "imu_data").into_iter().find(|(at, _)| *at > t).expect("an imu_data after it").1;
    assert_eq!(next["orientation"], json!({"x": 0.0, "y": 0.0, "z": 0.6, "w": 0.8}));
    assert_eq!(next["header"]["frame_id"], "cmd_frame");
    let st = agent.status(r.now);
    let sent: u64 = st["topics"].as_array().unwrap().iter().map(|t| t["sent"].as_u64().unwrap()).sum();
    assert_eq!(sent, 3);
}

/// Restarting the agent is nano_ros_agent starting again: it discovers and the board registers its topics again,
/// but (libnanoros as of 9c289c6) never finishes:
/// * a DISCOVER that comes while a publish is still going out is dropped (`register_topics_start` needs the
///   transmitter idle, and nothing retries): the one at 800 ms is; nano_ros_agent sends only one, the mock tries
///   again 230 ms later;
/// * `registration_complete` stays set from the first handshake, so the board goes on publishing while it
///   registers; a publish clears `tx.is_registration`, and the ACK of the last topic then sends no REGISTER_DONE.
///
/// The agent goes on decoding what the board publishes, but never sees the handshake finish (nano_ros_agent would
/// never create the topics in ROS). See docs/nanoros.md.
#[test]
fn a_restarted_agent_sees_the_board_register_again_but_never_finish() {
    let (mut r, agent, lines) = led_robot();
    r.run_until(Time::from_ms(500));
    agent.stop();
    r.run_until(Time::from_ms(800));
    let quiet = agent.message_count();
    agent.start(Time::from_ms(800));
    r.run_until(Time::from_ms(1300));
    let (first, _) = line_after(&lines, "starting registration", Time::from_ms(800)).unwrap();
    let (load, _) = line_after(&lines, "loading reg frame", Time::from_ms(800)).unwrap();
    assert!(first < Time::from_ms(810) && load > Time::from_ms(1030), "dropped at {first}, registered at {load}");
    // Every topic registered again (and acknowledged), a publish in the middle, and no REGISTER_DONE
    assert!(line_after(&lines, "host ack: topic id[129]", load).is_some());
    let (publ, _) = line_after(&lines, "serialized bytes", load).unwrap();
    assert!(publ < line_after(&lines, "host ack: topic id[129]", load).unwrap().0, "a publish before the last ACK");
    assert!(line_after(&lines, "done reg", load).is_none());
    let st = agent.status(r.now);
    assert_eq!((st["handshakes"].as_u64(), st["connected"].as_bool()), (Some(1), Some(false)));
    assert!(st["topics"].as_array().unwrap().iter().all(|t| t["registered"] == true));
    assert!(on(&*agent, "imu_data").iter().any(|(t, _)| *t > Time::from_ms(1100)), "still decoded");
    assert!(agent.message_count() > quiet);
    assert!(agent.errors().is_empty(), "{:?}", agent.errors());
}
