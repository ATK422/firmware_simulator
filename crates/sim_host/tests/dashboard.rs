//! The live dashboard: its HTTP server over loopback, the JSON state, and controls that change the running boards.
//! (The page itself is not browser-tested here; these tests cover everything it talks to.)

mod common;

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::{Arc, Mutex};

use serde_json::Value;
use sim_core::time::Time;
use sim_host::Session;
use sim_host::board::Inputs;
use sim_host::dashboard::{BoardInfo, Dashboard, Link};

/// One HTTP request; returns the status code and the body.
fn request(addr: SocketAddr, method: &str, path: &str, body: &str) -> (u16, String) {
    let mut s = TcpStream::connect(addr).unwrap();
    write!(
        s,
        "{method} {path} HTTP/1.1\r\nHost: test\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{body}",
        body.len()
    )
    .unwrap();
    let mut out = String::new();
    s.read_to_string(&mut out).unwrap();
    let (head, body) = out.split_once("\r\n\r\n").unwrap();
    let code = head.split_whitespace().nth(1).unwrap().parse().unwrap();
    assert!(head.contains(&format!("Content-Length: {}", body.len())), "{head}");
    (code, body.to_string())
}

fn json(body: &str) -> Value {
    serde_json::from_str(body).unwrap_or_else(|e| panic!("{e}: {body}"))
}

fn blink() -> Session {
    common::boot(&common::repo().join("test_programs/blink.elf"), None)
}

/// Runs `s` in short pieces, servicing the dashboard, until `client` (run on another thread) is done.
fn run_with_client<T: Send + 'static>(
    s: &mut Session,
    d: &mut Dashboard,
    client: impl FnOnce(SocketAddr) -> T + Send + 'static,
) -> T {
    let addr = d.addr;
    let h = std::thread::spawn(move || client(addr));
    while !h.is_finished() {
        if !d.paused() {
            s.run_for(Time::from_us(500));
        }
        d.service_session(s);
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    h.join().unwrap()
}

#[test]
fn serves_the_page_and_the_state() {
    let mut s = blink();
    let mut d = Dashboard::start("127.0.0.1:0", vec![BoardInfo { name: "blink".into(), ..Default::default() }], vec![])
        .unwrap();
    d.publish_every = std::time::Duration::ZERO;
    d.tap_session(0, &mut s);
    let (page, states, missing, wrong_method) = run_with_client(&mut s, &mut d, |addr| {
        let page = request(addr, "GET", "/", "");
        // The LED toggles every ~0.46 ms of virtual time: watch the state for a while
        let mut states = Vec::new();
        for _ in 0..40 {
            states.push(request(addr, "GET", "/api/state", ""));
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        (page, states, request(addr, "GET", "/nope", ""), request(addr, "DELETE", "/api/state", ""))
    });
    assert_eq!(page.0, 200);
    assert!(page.1.contains("<title>firmware_simulator</title>") && page.1.contains("/api/state"));
    assert!(!page.1.contains("http://") && !page.1.contains("https://"), "the page loads nothing from elsewhere");
    assert_eq!(missing.0, 404);
    assert_eq!(wrong_method.0, 405);
    let states: Vec<Value> = states
        .iter()
        .map(|(c, b)| {
            assert_eq!(*c, 200);
            json(b)
        })
        .collect();
    let last = states.last().unwrap();
    assert!(last["time_s"].as_f64().unwrap() > 0.0);
    let b = &last["boards"][0];
    assert_eq!(b["name"], "blink");
    assert_eq!(b["cores"][0]["state"], "running");
    assert_eq!(b["cores"][1]["state"], "sleeping", "core 1 waits in the bootrom");
    assert_eq!(b["gpio"]["labels"][25], "gpio25");
    assert_eq!(b["gpio"]["funcs"][25], "SIO");
    // GPIO 25 seen both high and low
    let levels: std::collections::BTreeSet<bool> =
        states.iter().filter_map(|s| s["boards"][0]["gpio"]["level"].as_u64()).map(|l| l & (1 << 25) != 0).collect();
    assert_eq!(levels.len(), 2, "GPIO 25 toggles in the published state");
    // Virtual time moves forward between states
    let times: Vec<f64> = states.iter().filter_map(|s| s["time_s"].as_f64()).collect();
    assert!(times.windows(2).all(|w| w[1] >= w[0]) && times.last() > times.first());
    assert!(last["fault_help"].as_array().unwrap().iter().any(|f| f["fault"] == "can-stuck"));
}

#[test]
fn posted_faults_and_inputs_take_effect() {
    let mut s = blink();
    let vbat = Arc::new(Mutex::new(12.0));
    let inputs = Inputs([("vbat".to_string(), vbat.clone())].into());
    let info = BoardInfo { name: "blink".into(), image: "blink.elf".into(), inputs, ..Default::default() };
    let mut d = Dashboard::start("127.0.0.1:0", vec![info], vec![]).unwrap();
    d.publish_every = std::time::Duration::ZERO;
    d.tap_session(0, &mut s);
    let replies = run_with_client(&mut s, &mut d, |addr| {
        let mut r = vec![
            // GPIO 3 is not driven by the program: drive it high from outside
            request(addr, "POST", "/api/fault", r#"{"board": "blink", "fault": "gpio:3:high"}"#),
            request(addr, "POST", "/api/fault", r#"{"board": "blink", "fault": "warp-drive"}"#),
            request(addr, "POST", "/api/fault", r#"{"board": "nobody", "fault": "panic"}"#),
            // Not on a CAN bus: the fault is understood but cannot be applied
            request(addr, "POST", "/api/fault", r#"{"board": "blink", "fault": "can-stuck"}"#),
            request(addr, "POST", "/api/input", r#"{"board": "blink", "name": "vbat", "value": 9.5}"#),
            request(addr, "POST", "/api/input", r#"{"board": "blink", "name": "vbus", "value": 5}"#),
            request(addr, "POST", "/api/input", r#"{"board": "blink", "name": "vbat", "value": "x"}"#),
            request(addr, "POST", "/api/fault", "not json"),
        ];
        // Answers come once the state shows what was done
        r.push(request(addr, "GET", "/api/state", ""));
        r
    });
    let codes: Vec<u16> = replies.iter().map(|r| r.0).collect();
    assert_eq!(codes, [200, 400, 422, 422, 200, 422, 400, 400, 200], "{replies:#?}");
    assert!(json(&replies[1].1)["error"].as_str().unwrap().contains("unknown fault"));
    assert!(json(&replies[3].1)["error"].as_str().unwrap().contains("not on a CAN bus"));
    assert!(json(&replies[5].1)["error"].as_str().unwrap().contains("vbat"), "lists the inputs there are");
    // The fault took effect on the machine, and shows in the state
    assert_ne!(s.machine.sys.gpio.level & (1 << 3), 0, "GPIO 3 driven high");
    let state = json(&replies.last().unwrap().1);
    assert_ne!(state["boards"][0]["gpio"]["level"].as_u64().unwrap() & (1 << 3), 0);
    let history = state["faults"].as_array().unwrap();
    // Faults that parse are recorded, applied or not
    assert_eq!(history.len(), 3, "{history:?}");
    assert_eq!(history[0]["fault"], "gpio:3:high");
    assert_eq!(history[0]["ok"], true);
    assert_eq!((&history[1]["board"], &history[1]["ok"]), (&Value::from("nobody"), &Value::from(false)));
    assert_eq!(history[2]["ok"], false);
    // The input changed
    assert_eq!(*vbat.lock().unwrap(), 9.5);
    assert_eq!(state["boards"][0]["inputs"][0]["value"], 9.5);
}

#[test]
fn pausing_holds_virtual_time() {
    let mut s = blink();
    let mut d = Dashboard::start("127.0.0.1:0", vec![BoardInfo { name: "blink".into(), ..Default::default() }], vec![])
        .unwrap();
    d.publish_every = std::time::Duration::ZERO;
    let (paused, held, resumed) = run_with_client(&mut s, &mut d, |addr| {
        assert_eq!(request(addr, "POST", "/api/pause", r#"{"paused": true}"#).0, 200);
        let a = json(&request(addr, "GET", "/api/state", "").1);
        std::thread::sleep(std::time::Duration::from_millis(50));
        let b = json(&request(addr, "GET", "/api/state", "").1);
        assert_eq!(request(addr, "POST", "/api/pause", r#"{"paused": false}"#).0, 200);
        std::thread::sleep(std::time::Duration::from_millis(50));
        let c = json(&request(addr, "GET", "/api/state", "").1);
        (a, b, c)
    });
    assert_eq!(paused["paused"], true);
    assert_eq!(paused["time_s"], held["time_s"], "no virtual time passes while paused");
    assert_eq!(held["speed"], 0.0);
    assert_eq!(resumed["paused"], false);
    assert!(resumed["time_s"].as_f64() > held["time_s"].as_f64());
}

#[test]
fn a_robot_state_has_every_board_and_its_power() {
    let mut r = sim_host::robot::Robot::default();
    for n in ["a", "b"] {
        r.add(n, blink(), None);
    }
    r.power_on[1] = (Time::from_secs(10), false);
    let infos = r.names.iter().map(|n| BoardInfo { name: n.clone(), ..Default::default() }).collect();
    let mut d = Dashboard::start("127.0.0.1:0", infos, vec![]).unwrap();
    d.publish_every = std::time::Duration::ZERO;
    r.run_until(Time::from_ms(5));
    let addr = d.addr;
    let h = std::thread::spawn(move || {
        let f = request(addr, "POST", "/api/fault", r#"{"board": "a", "fault": "gpio:4:low"}"#);
        (f, request(addr, "GET", "/api/state", ""))
    });
    while !h.is_finished() {
        r.run_until(r.now + Time::from_us(500));
        d.service_robot(&mut r, &[]);
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    let (fault, state) = h.join().unwrap();
    assert_eq!(fault.0, 200, "{}", fault.1);
    let state = json(&state.1);
    let boards = state["boards"].as_array().unwrap();
    assert_eq!(boards.len(), 2);
    assert_eq!(boards[0]["powered"], true);
    assert_eq!(boards[1]["powered"], false);
    assert_eq!(boards[1]["cores"][0]["state"], "off");
    assert_eq!(state["vehicles"], serde_json::json!([]));
    assert_eq!(state["faults"][0]["board"], "a");
}

/// A Rust board on its bus with a mock bridge: firmware status decoded from titan_link, CAN traffic counted, and
/// the kill switch thrown from the dashboard.
#[test]
fn firmware_status_can_traffic_and_the_kill_switch() {
    let Some(elf) = common::titan_elf("power_board") else { return };
    let board = sim_host::board::Board::load(&common::repo().join("boards/mk2/power_board.toml")).unwrap();
    let mut s = common::boot(&elf, None);
    let inputs = board.apply(&mut s.machine).unwrap();
    s.can_bus = Some(sim_devices::can::CanBus::new("can0", board.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    let info = BoardInfo {
        name: "power".into(),
        image: "power_board".into(),
        inputs,
        pins: board.pins.clone(),
        ..Default::default()
    };
    let mut d = Dashboard::start("127.0.0.1:0", vec![info], vec!["can0".into()]).unwrap();
    d.publish_every = std::time::Duration::ZERO;
    d.tap_session(0, &mut s);
    d.add_link(Some(0), None, Link::Can(bridge.clone()));
    s.run_for(Time::from_ms(2200));
    d.service_session(&mut s);
    let addr = d.addr;
    let state = json(&std::thread::spawn(move || request(addr, "GET", "/api/state", "").1).join().unwrap());
    let b = &state["boards"][0];
    let st = &b["link"]["status"];
    assert!(st["board_name"].as_str().unwrap().contains("power_board"), "{st}");
    assert_eq!(st["client_id"], 1);
    assert!(st["uptime_s"].as_f64().unwrap() > 1.0);
    assert_eq!(st["faults"], serde_json::json!([]), "a healthy board reports no faults: {st}");
    assert!(b["link"]["heartbeats"].as_u64().unwrap() > 5);
    // The board's own fault names, read from its FAULT_NAMES table
    let names = b["fault_names"].as_array().unwrap();
    assert_eq!(names[0], "FAULT_WATCHDOG_RESET");
    assert!(names.len() > 5, "{names:?}");
    assert!(
        b["logs"].as_array().unwrap().iter().any(|l| l["text"].as_str().unwrap().contains("CAN controller online"))
    );
    assert!(b["inputs"].as_array().unwrap().len() >= 2, "the board's sensor inputs");
    let bus = &state["buses"][0];
    assert!(bus["frames"].as_u64().unwrap() > 20 && bus["rate_fps"].as_f64().unwrap() > 5.0, "{bus}");
    assert!(bus["load"].as_f64().unwrap() > 0.0);
    assert!(!bus["recent"].as_array().unwrap().is_empty());
    // Throw the kill switch from the page
    let reply = run_with_client(&mut s, &mut d, |addr| {
        request(addr, "POST", "/api/fault", r#"{"board": "power", "fault": "kill:on"}"#)
    });
    assert_eq!(reply.0, 200, "{}", reply.1);
    assert_eq!(bridge.state().kill, Some(true));
}

/// A fault posted to the dashboard shows up as the firmware reports it: the camera cage's depth sensor stops
/// answering, the firmware raises its board-specific FAULT_DEPTH_INIT_ERROR, and the decoded status names it.
#[test]
fn a_posted_sensor_fault_comes_back_as_a_named_firmware_fault() {
    let Some(elf) = common::titan_elf("camera_cage_bb") else { return };
    let board = sim_host::board::Board::load(&common::repo().join("boards/mk2/camera_cage_bb.toml")).unwrap();
    let mut s = common::boot(&elf, None);
    let inputs = board.apply(&mut s.machine).unwrap();
    s.can_bus = Some(sim_devices::can::CanBus::new("can0", board.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(4, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    let info = BoardInfo { name: "cage".into(), inputs, pins: board.pins.clone(), ..Default::default() };
    let mut d = Dashboard::start("127.0.0.1:0", vec![info], vec!["can0".into()]).unwrap();
    d.publish_every = std::time::Duration::ZERO;
    d.tap_session(0, &mut s);
    d.add_link(Some(0), None, Link::Can(bridge));
    let reply = run_with_client(&mut s, &mut d, |addr| {
        request(addr, "POST", "/api/fault", r#"{"board": "cage", "fault": "i2c-absent:0x76"}"#)
    });
    assert_eq!(reply.0, 200, "{}", reply.1);
    s.run_for(Time::from_ms(2500));
    d.service_session(&mut s);
    let state = json(&d.published());
    let st = &state["boards"][0]["link"]["status"];
    let faults: Vec<&str> = st["faults"].as_array().unwrap().iter().map(|f| f.as_str().unwrap()).collect();
    assert!(faults.contains(&"FAULT_DEPTH_INIT_ERROR"), "{st}");
}

/// `GET /api/logs`: lines after a cursor, from every board in order or from one, a page at a time or the newest,
/// read live (not from the published state), and a warning when lines after the cursor were dropped.
#[test]
fn logs_are_read_after_a_cursor() {
    let (mut a, mut b) = (blink(), blink());
    let infos = vec![
        BoardInfo { name: "a".into(), ..Default::default() },
        BoardInfo { name: "b".into(), ..Default::default() },
    ];
    let mut d = Dashboard::start("127.0.0.1:0", infos, vec![]).unwrap();
    d.tap_session(0, &mut a);
    d.tap_session(1, &mut b);
    // Lines as the sessions report them: a's UART, b's defmt log
    for k in 0..3u64 {
        (a.on_uart_line.as_mut().unwrap())(Time::from_ms(k), 0, &format!("a{k}"));
        let line = sim_host::defmt::LogLine { level: Some("info".into()), timestamp: None, message: format!("b{k}") };
        (b.on_log.as_mut().unwrap())(Time::from_ms(k), &line);
    }
    let addr = d.addr;
    let get = move |q: &str| {
        let (code, body) = request(addr, "GET", &format!("/api/logs{q}"), "");
        (code, json(&body))
    };
    let (code, all) = get("");
    assert_eq!(code, 200, "{all}");
    let texts = |v: &Value| -> Vec<String> {
        v["lines"].as_array().unwrap().iter().map(|l| l["text"].as_str().unwrap().to_string()).collect()
    };
    assert_eq!(texts(&all), ["a0", "INFO  b0", "a1", "INFO  b1", "a2", "INFO  b2"], "every board, in order");
    assert_eq!(all["lines"][0]["board"], "a");
    assert_eq!(all["lines"][0]["kind"], "uart0");
    assert_eq!(all["lines"][1]["kind"], "info");
    assert_eq!(all["lines"][5]["t"], 0.002);
    assert_eq!(
        (&all["next"], &all["more"], &all["truncated"]),
        (&Value::from(6), &Value::from(false), &Value::from(false))
    );
    // Not published: the live read sees the lines before any state is
    assert_eq!(json(&d.published()), serde_json::json!({}));
    // A page at a time, and on from the cursor
    let (_, page) = get("?max=4");
    assert_eq!((texts(&page).len(), &page["more"], &page["next"]), (4, &Value::from(true), &Value::from(4)));
    let (_, rest) = get("?since=4");
    assert_eq!(texts(&rest), ["a2", "INFO  b2"]);
    // One board; the newest lines
    let (_, only_b) = get("?board=b&since=1");
    assert_eq!(texts(&only_b), ["INFO  b0", "INFO  b1", "INFO  b2"]);
    let (_, newest) = get("?board=a&max=1&tail=1");
    assert_eq!((texts(&newest), &newest["more"]), (vec!["a2".to_string()], &Value::from(true)));
    let (_, none) = get("?since=6");
    assert_eq!((texts(&none).len(), &none["next"]), (0, &Value::from(6)));
    // Bad requests
    let (code, e) = get("?board=c");
    assert_eq!(code, 400);
    assert!(e["error"].as_str().unwrap().contains("there are: a, b"), "{e}");
    assert_eq!(get("?since=x").0, 400);
    // Lines beyond what is kept are dropped, oldest first, and a reader behind them is told
    for k in 0..2100u64 {
        (a.on_uart_line.as_mut().unwrap())(Time::from_ms(10 + k), 0, &format!("more {k}"));
    }
    let (_, behind) = get("?board=a&since=2&max=5");
    assert_eq!(behind["truncated"], true);
    let (_, caught_up) = get("?board=a&since=2000");
    assert_eq!(caught_up["truncated"], false);
    assert_eq!(get("?board=a&max=100000").1["lines"].as_array().unwrap().len(), 2000, "at most 2000 a read");
    // The published state still carries the last 120 lines a board
    d.service_session(&mut a);
    assert_eq!(json(&d.published())["boards"][0]["logs"].as_array().unwrap().len(), 120);
}

/// `GET /api/frames`: a bus's frames after a cursor, as structured records, filtered by ID and mask.
#[test]
fn frames_are_read_after_a_cursor_and_filtered_by_id() {
    use sim_devices::can::{BitRates, BusRecord, CanBus, CanFrame, Sender};
    let rates = BitRates { nominal: 1_000_000, data: None };
    let mut buses = [
        sim_host::robot::RobotBus::new(CanBus::new("x", rates)),
        sim_host::robot::RobotBus::new(CanBus::new("y", rates)),
    ];
    let d = Dashboard::start("127.0.0.1:0", vec![], vec!["x".into(), "y".into()]).unwrap();
    for (j, rb) in buses.iter_mut().enumerate() {
        d.tap_bus(j, rb);
    }
    let frame = |id: u32, k: u64, acked: bool| BusRecord {
        start: Time::from_us(100 * k),
        end: Time::from_us(100 * k + 50),
        sender: Sender::Node(0),
        frame: CanFrame::classic(id, false, &[k as u8]),
        acked,
        error: None,
    };
    for k in 0..6u64 {
        let id = if k % 2 == 0 { 0x100 } else { 0x201 };
        (buses[0].on_can.as_mut().unwrap())(&frame(id, k, k != 5));
    }
    let addr = d.addr;
    let get = move |q: &str| {
        let (code, body) = request(addr, "GET", &format!("/api/frames{q}"), "");
        (code, json(&body))
    };
    let (code, all) = get("?bus=x");
    assert_eq!(code, 200, "{all}");
    let frames = all["frames"].as_array().unwrap();
    assert_eq!(frames.len(), 6);
    assert_eq!(frames[0]["id"], "0x100");
    assert_eq!(frames[0]["seq"], 1);
    assert_eq!(frames[0]["data"], "00");
    assert_eq!(frames[0]["candump"], "100#00");
    assert_eq!(frames[5]["acked"], false);
    assert_eq!((&all["next"], &all["total"], &all["bus"]), (&Value::from(6), &Value::from(6), &Value::from("x")));
    let ids = |v: &Value| -> Vec<u64> {
        v["frames"].as_array().unwrap().iter().map(|f| f["seq"].as_u64().unwrap()).collect()
    };
    assert_eq!(ids(&get("?bus=x&id=0x201").1), [2, 4, 6]);
    assert_eq!(ids(&get("?bus=x&id=0x200&mask=0x700").1), [2, 4, 6], "0x201 under mask 0x700");
    assert_eq!(ids(&get("?bus=x&id=256&since=1").1), [3, 5]);
    let (_, page) = get("?bus=x&max=2");
    assert_eq!((ids(&page), &page["next"], &page["more"]), (vec![1, 2], &Value::from(2), &Value::from(true)));
    assert_eq!(ids(&get("?bus=x&max=2&tail=1").1), [5, 6]);
    // A filter that matches nothing still moves the cursor on
    let (_, nothing) = get("?bus=x&id=0x7ff");
    assert_eq!((ids(&nothing).len(), &nothing["next"]), (0, &Value::from(6)));
    assert_eq!(get("?bus=y").1["frames"], serde_json::json!([]));
    // Two buses: which one must be said
    let (code, e) = get("");
    assert_eq!(code, 400);
    assert!(e["error"].as_str().unwrap().contains("x, y"), "{e}");
    assert_eq!(get("?bus=z").0, 400);
    assert_eq!(get("?bus=x&id=zz").0, 400);
}

/// `POST /api/pause` with `at_s`: pause when virtual time gets there, and run on to a time and pause again.
#[test]
fn pausing_at_a_virtual_time() {
    let mut s = blink();
    let mut d = Dashboard::start("127.0.0.1:0", vec![BoardInfo { name: "blink".into(), ..Default::default() }], vec![])
        .unwrap();
    d.publish_every = std::time::Duration::ZERO;
    let wait_paused = |addr: SocketAddr| loop {
        let st = json(&request(addr, "GET", "/api/state", "").1);
        if st["paused"] == true {
            break st;
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    };
    let (first, second, past, bad, live) = run_with_client(&mut s, &mut d, move |addr| {
        assert_eq!(request(addr, "POST", "/api/pause", r#"{"paused": true}"#).0, 200);
        let now = json(&request(addr, "GET", "/api/state", "").1)["time_s"].as_f64().unwrap();
        let at = now + 0.02;
        let r = request(addr, "POST", "/api/pause", &format!(r#"{{"paused": false, "at_s": {at}}}"#));
        assert_eq!(r.0, 200, "{}", r.1);
        assert!(json(&r.1)["message"].as_str().unwrap().starts_with("running until"));
        let first = (at, wait_paused(addr));
        // Pause at a later time while running
        assert_eq!(request(addr, "POST", "/api/pause", r#"{"paused": false}"#).0, 200);
        let at = first.1["time_s"].as_f64().unwrap() + 0.01;
        let r = request(addr, "POST", "/api/pause", &format!(r#"{{"paused": true, "at_s": {at}}}"#));
        assert!(json(&r.1)["message"].as_str().unwrap().starts_with("will pause at"), "{}", r.1);
        let second = (at, wait_paused(addr));
        let past = request(addr, "POST", "/api/pause", r#"{"paused": false, "at_s": 0.0}"#);
        let bad = request(addr, "POST", "/api/pause", r#"{"paused": true, "at_s": "soon"}"#);
        let live = json(&request(addr, "GET", "/api/logs", "").1);
        (first, second, past, bad, live)
    });
    for (at, st) in [first, second] {
        let t = st["time_s"].as_f64().unwrap();
        // The run loop pauses at the first piece that ends at or after the time (500 µs pieces here)
        assert!(t >= at - 1e-9 && t < at + 0.001, "paused at {t}, asked for {at}");
        assert_eq!(st["pause_at_s"], Value::Null);
    }
    assert_eq!(past.0, 422, "{}", past.1);
    assert_eq!(bad.0, 400);
    assert_eq!(live["paused"], true, "the live reads say where the run is: {live}");
    assert!(live["time_s"].as_f64().unwrap() > 0.0);
}

#[test]
fn query_strings_are_decoded() {
    let q = sim_host::dashboard::query_params("board=camera%20cage&since=5&x=a+b&flag&bad=%zz");
    assert_eq!(q["board"], "camera cage");
    assert_eq!(q["since"], "5");
    assert_eq!(q["x"], "a b");
    assert_eq!(q["flag"], "");
    assert_eq!(q["bad"], "%zz");
}

/// `--dashboard-pause-at`: a run told before it starts to pause at a time does, however fast it runs; zero pauses
/// it from the start.
#[test]
fn a_run_can_be_set_to_pause_before_it_starts() {
    let info = || vec![BoardInfo { name: "blink".into(), ..Default::default() }];
    let mut d = Dashboard::start("127.0.0.1:0", info(), vec![]).unwrap();
    d.pause_at(Some(Time::ZERO));
    assert!(d.paused());
    let mut s = blink();
    let mut d = Dashboard::start("127.0.0.1:0", info(), vec![]).unwrap();
    d.pause_at(Some(Time::from_ms(3)));
    while !d.paused() {
        s.run_for(Time::from_us(500));
        d.service_session(&mut s);
        assert!(s.machine.now < Time::from_ms(10), "did not pause");
    }
    assert!(s.machine.now >= Time::from_ms(3) && s.machine.now < Time::from_us(3500), "{}", s.machine.now);
    assert_eq!(json(&d.published())["pause_at_s"], Value::Null, "done with");
}

/// `GET /api/state?fresh=1` has the run loop build the state now, rather than return the last one published.
#[test]
fn a_fresh_state_is_built_on_request() {
    let mut s = blink();
    let mut d = Dashboard::start("127.0.0.1:0", vec![BoardInfo { name: "blink".into(), ..Default::default() }], vec![])
        .unwrap();
    d.publish_every = std::time::Duration::from_secs(3600);
    d.tap_session(0, &mut s);
    s.run_for(Time::from_ms(1));
    d.service_session(&mut s);
    let (stale, fresh) = run_with_client(&mut s, &mut d, |addr| {
        std::thread::sleep(std::time::Duration::from_millis(30));
        (json(&request(addr, "GET", "/api/state", "").1), json(&request(addr, "GET", "/api/state?fresh=1", "").1))
    });
    let first = stale["time_s"].as_f64().unwrap();
    assert!(first < 0.0011, "not due again for an hour: {first}");
    assert!(fresh["time_s"].as_f64().unwrap() > first + 0.001, "{}", fresh["time_s"]);
}
