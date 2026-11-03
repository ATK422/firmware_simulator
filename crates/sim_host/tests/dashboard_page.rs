//! The dashboard page's script, run under Node (when it is installed) against a minimal stand-in for the DOM and
//! fed real states from the emulator: it must render every board, bus, bridge and vehicle without throwing, and
//! send the right requests for its controls. This is no substitute for a browser, but it catches script errors
//! and mismatches between the page and the JSON it reads.

mod common;

use std::process::Command;

use serde_json::{Value, json};
use sim_core::time::Time;
use sim_host::dashboard::{BoardInfo, Dashboard};

const HARNESS: &str = r##"
const vm = require("vm");
const fs = require("fs");
const [script, statesFile] = process.argv.slice(2);
const states = JSON.parse(fs.readFileSync(statesFile, "utf8"));

class El {
  constructor(tag) {
    this.tagName = tag; this.children = []; this.attrs = {}; this.style = {}; this.textContent = "";
    this.className = ""; this.hidden = false; this.value = ""; this.scrollTop = 0; this.clientHeight = 100;
    this.scrollHeight = 0;
  }
  append(...kids) {
    for (const k of kids) {
      if (k === undefined || k === null) throw new Error("append(undefined)");
      if (typeof k === "string") { const t = new El("#text"); t.textContent = k; this.children.push(t); }
      else if (k instanceof El) this.children.push(k);
      else throw new Error("append of " + typeof k);
    }
  }
  replaceChildren(...kids) { this.children = []; this.append(...kids); }
  setAttribute(k, v) { this.attrs[k] = String(v); if (k === "value") this.value = String(v); }
  get childNodes() { return this.children; }
  get firstChild() { const c = this.children[0]; return c ? { remove: () => this.children.shift() } : null; }
  get options() { return this.children; }
  text() { return this.textContent + this.children.map((c) => c.text()).join(" "); }
}
const ids = {};
const document = { getElementById: (id) => ids[id] || (ids[id] = new El("div")), createElement: (t) => new El(t) };
let current = states[0];
const posts = [];
async function fetch(path, opts) {
  if (opts && opts.method === "POST") posts.push([path, JSON.parse(opts.body)]);
  return { json: async () => (path === "/api/state" ? current : { ok: true, message: "done" }) };
}
// The page reports a failed render in its connection line; make the error visible here too
const ctx = vm.createContext({ document, fetch, setInterval: () => 0, console, Number, Math, JSON, String, Error, Object });
vm.runInContext(fs.readFileSync(script, "utf8"), ctx);
const tick = () => new Promise((r) => setTimeout(r, 5));
(async () => {
  // The page polls once as it loads
  await tick();
  for (const s of states) {
    current = s;
    await vm.runInContext("poll()", ctx);
    await tick();
    if (!ids.conn || ids.conn.textContent) throw new Error("render failed: " + (ids.conn && ids.conn.textContent));
  }
  // Polling the same state again updates the cards in place: the same nodes, no log line added twice
  const card = ids.boards.children[0], firstRow = card.children[1].children[0], before = ids.boards.text();
  await vm.runInContext("poll()", ctx);
  await tick();
  const inPlace = ids.boards.children[0] === card && card.children[1].children[0] === firstRow && ids.boards.text() === before;
  // The controls
  const $ = document.getElementById;
  $("fboard").value = states.at(-1).boards[0].name;
  $("fault").value = "can-stuck";
  $("faultform").onsubmit({ preventDefault() {} });
  $("ivalue").value = "12.5";
  $("inputform").onsubmit({ preventDefault() {} });
  $("pause").onclick();
  await new Promise((r) => setTimeout(r, 10));
  const out = {
    boards: ids.boards.children.length,
    vehicles: ids.vehicles.children.length,
    buses: ids.buses.children.length,
    systemHidden: ids["system-section"].hidden,
    systemTitle: ids["system-title"].textContent,
    time: ids.time.textContent,
    mode: ids.mode.textContent,
    boardText: ids.boards.text(),
    busText: ids.buses.text(),
    vehicleText: ids.vehicles.text(),
    posts,
    inPlace,
  };
  console.log(JSON.stringify(out));
})().catch((e) => { console.error(e.stack || e); process.exit(1); });
"##;

fn node() -> Option<String> {
    for n in ["node", "nodejs", "/opt/node22/bin/node"] {
        if Command::new(n).arg("--version").output().is_ok_and(|o| o.status.success()) {
            return Some(n.to_string());
        }
    }
    eprintln!("skipping: no node to run the dashboard page's script");
    None
}

/// The script of the page.
fn script() -> String {
    let page = std::fs::read_to_string(common::repo().join("crates/sim_host/src/dashboard.html")).unwrap();
    let start = page.find("<script>").unwrap() + "<script>".len();
    let end = page.rfind("</script>").unwrap();
    page[start..end].to_string()
}

#[test]
fn the_page_renders_real_states_and_posts_its_controls() {
    let Some(node) = node() else { return };
    // Real states: two blinking boards in a robot, one switched on later
    let mut r = sim_host::robot::Robot::default();
    for n in ["a", "b"] {
        r.add(n, common::boot(&common::repo().join("test_programs/blink.elf"), None), None);
    }
    r.power_on[1] = (Time::from_ms(3), false);
    let inputs =
        sim_host::board::Inputs([("vbat".to_string(), std::sync::Arc::new(std::sync::Mutex::new(12.0)))].into());
    let infos = vec![
        BoardInfo { name: "a".into(), image: "blink.elf".into(), inputs, ..Default::default() },
        BoardInfo { name: "b".into(), image: "blink.elf".into(), ..Default::default() },
    ];
    let mut d = Dashboard::start("127.0.0.1:0", infos, vec![]).unwrap();
    for (i, s) in r.boards.iter_mut().enumerate() {
        d.tap_session(i, s);
    }
    let mut states = Vec::new();
    for ms in [1, 5] {
        r.run_until(Time::from_ms(ms));
        states.push(d.robot_state(&mut r));
    }
    // and one with everything a robot can have: a bus, a bridge with a status and faults, a vehicle, LEDs, a
    // strip, ESCs and log lines (shaped as the emulator produces them)
    let mut full = states[1].clone();
    full["buses"] = json!([{"name": "internal", "frames": 120, "errors": 1, "unacked": 2, "rate_fps": 40.0, "load": 0.012,
        "recent": [{"t": 0.004, "frame": "06F#C0"}, {"t": 0.003, "frame": "02420802##10100  (no ACK)"}]}]);
    let link = json!({"kind": "can", "client": 1, "running": false, "kill": true, "pings": 30, "heartbeats": 25,
        "last_heartbeat_age_s": 0.2, "decode_errors": 0, "board": "a", "bus": "internal",
        "status": {"age_s": 0.4, "board_name": "mk2/power_board", "bus_id": 0, "client_id": 1, "version": "1.0 (clean)",
            "uptime_s": 3.2, "fault_bits": 0x41, "faults": ["FAULT_WATCHDOG_RESET", "FAULT_DEPTH_INIT_ERROR"],
            "kill_switches": {"enabled": 1, "asserting_kill": 1, "needs_update": 0, "timed_out": 0}}});
    full["links"] = json!([link.clone()]);
    full["boards"][0]["link"] = link;
    full["boards"][0]["leds"] = json!([{"name": "status_ledr", "pin": 13, "on": true}]);
    full["boards"][0]["strips"] = json!([{"name": "status strip", "pin": 9, "colors": ["ff0a00", "000000"]}]);
    full["boards"][0]["cores"][0]["function"] =
        json!("<embassy_executor::platform::thread::Executor>::run::<sbc_board::__cortex_m_rt_main::{closure#0}>+0xba");
    full["boards"][0]["cores"][1]["function"] = json!(
        "<embassy_executor::raw::TaskStorage<ivc_board2::modem::__rx_task_task::__rx_task_task_inner_function::{closure#0}>>::poll+0x5c8"
    );
    full["boards"][0]["escs"] = json!([{"name": "esc1", "rpm": 1234.5, "pin": 6}]);
    full["boards"][0]["logs"] = json!([{"seq": 1, "t": 0.1, "kind": "info", "text": "0.001 INFO hello"},
        {"seq": 2, "t": 0.2, "kind": "uart0", "text": "AB Hello"}]);
    full["vehicles"] = json!([{"name": "talos", "position_m": [5.0, 6.0, 0.25], "depth_m": 0.25, "rpy_deg": [0.0, 1.0, 90.0],
        "velocity_mps": [0.1, 0.0, 0.0], "speed_mps": 0.1, "rate_dps": [0.0, 0.0, 0.0],
        "thrusters": [{"name": "fwd_stbd", "rpm": 1500.0}, {"name": "fwd_port", "rpm": -300.0}]}]);
    full["faults"] =
        json!([{"t": 1.0, "board": "a", "fault": "kill:on", "ok": true, "error": null, "from": "dashboard"}]);
    full["finished"] = json!("TimeLimit");
    states.push(full);

    let dir = std::env::temp_dir().join(format!("dashboard-page-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("page.js"), script()).unwrap();
    std::fs::write(dir.join("states.json"), serde_json::to_string(&states).unwrap()).unwrap();
    std::fs::write(dir.join("harness.js"), HARNESS).unwrap();
    let out = Command::new(node)
        .arg(dir.join("harness.js"))
        .arg(dir.join("page.js"))
        .arg(dir.join("states.json"))
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "the page's script failed:\n{stderr}");
    let o: Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(o["boards"], 2, "{o}");
    assert_eq!(o["vehicles"], 1);
    assert_eq!(o["systemHidden"], false);
    assert_eq!(o["systemTitle"], "Vehicles, CAN buses and bridges");
    assert_eq!(o["buses"], 2, "a bus card and a bridge card: {o}");
    assert_eq!(o["mode"], "finished: TimeLimit");
    let boards = o["boardText"].as_str().unwrap();
    for want in
        ["core0 running", "FAULT_DEPTH_INIT_ERROR", "status_ledr", "esc1", "1235 rpm", "AB Hello", "vbat", "12.000"]
    {
        assert!(boards.contains(want), "{want:?} not shown: {boards}");
    }
    // A long demangled symbol is shown short, with its offset apart (the full one is in the tooltip)
    assert!(boards.contains("Executor::run<main> +0xba"), "{boards}");
    assert!(boards.contains("TaskStorage<rx_task>::poll +0x5c8"), "{boards}");
    let buses = o["busText"].as_str().unwrap();
    for want in ["120", "40.0", "1.2%", "06F#C0", "(no ACK)", "mk2/power_board 1.0 (clean)", "asserting kill"] {
        assert!(buses.contains(want), "{want:?} not shown: {buses}");
    }
    assert!(o["vehicleText"].as_str().unwrap().contains("fwd_port"));
    assert_eq!(o["inPlace"], true, "a second poll rebuilt the cards or repeated log lines");
    let posts = o["posts"].as_array().unwrap();
    assert_eq!(posts[0], json!(["/api/fault", {"board": "a", "fault": "can-stuck"}]));
    assert_eq!(posts[1][0], "/api/input");
    assert_eq!(posts[1][1]["value"], 12.5);
    assert_eq!(posts[2], json!(["/api/pause", {"paused": true}]));
}
