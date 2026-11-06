//! `firmware_simulator mcp` end to end: a client speaking JSON-RPC lines to the server's stdin and stdout, as
//! Claude Code and other MCP clients do, running the emulator through its tools.

use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

struct Client {
    server: Child,
    stdin: Option<ChildStdin>,
    stdout: BufReader<ChildStdout>,
    next_id: u64,
}

impl Client {
    fn start() -> Client {
        let mut server = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
            .arg("mcp")
            .current_dir(repo())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let stdin = server.stdin.take();
        let stdout = BufReader::new(server.stdout.take().unwrap());
        Client { server, stdin, stdout, next_id: 1 }
    }

    fn send_line(&mut self, line: &str) {
        let stdin = self.stdin.as_mut().unwrap();
        writeln!(stdin, "{line}").unwrap();
        stdin.flush().unwrap();
    }

    fn read(&mut self) -> Value {
        let mut line = String::new();
        assert!(self.stdout.read_line(&mut line).unwrap() > 0, "the server closed stdout");
        serde_json::from_str(&line).unwrap_or_else(|e| panic!("{e}: not a JSON-RPC line: {line:?}"))
    }

    /// A request and its answer.
    fn request(&mut self, method: &str, params: Value) -> Value {
        let id = self.next_id;
        self.next_id += 1;
        self.send_line(&json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}).to_string());
        let reply = self.read();
        assert_eq!(reply["jsonrpc"], "2.0");
        assert_eq!(reply["id"], id, "{reply}");
        reply
    }

    /// A tool call that must succeed; returns its structured content.
    fn call(&mut self, name: &str, args: Value) -> Value {
        let r = self.call_result(name, args.clone());
        assert_eq!(r["isError"], false, "{name} {args}: {}", r["content"][0]["text"]);
        assert_eq!(r["content"][0]["type"], "text");
        let text = r["content"][0]["text"].as_str().unwrap();
        let s = r["structuredContent"].clone();
        if name != "run_once" {
            assert_eq!(serde_json::from_str::<Value>(text).unwrap(), s, "the text is the structured content");
        }
        s
    }

    fn call_result(&mut self, name: &str, args: Value) -> Value {
        let reply = self.request("tools/call", json!({"name": name, "arguments": args}));
        reply["result"].clone()
    }

    fn initialize(&mut self) {
        let r = self.request(
            "initialize",
            json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "1"}}),
        );
        assert_eq!(r["result"]["protocolVersion"], "2025-06-18");
        assert!(r["result"]["capabilities"]["tools"].is_object());
        self.send_line(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#);
    }

    /// Closes stdin and waits for the server to exit.
    fn close(mut self) -> std::process::ExitStatus {
        drop(self.stdin.take());
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            if let Some(s) = self.server.try_wait().unwrap() {
                return s;
            }
            assert!(Instant::now() < deadline, "the server did not exit when stdin closed");
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        let _ = self.server.kill();
        let _ = self.server.wait();
    }
}

/// Whether a process is gone (reaped, or never there).
fn gone(pid: u64) -> bool {
    // A zombie is gone for our purposes too: it no longer runs
    if Path::new("/proc/self/stat").exists() {
        return match std::fs::read_to_string(format!("/proc/{pid}/stat")) {
            Err(_) => true,
            Ok(stat) => stat.rsplit(')').next().is_some_and(|rest| rest.trim_start().starts_with('Z')),
        };
    }
    // No /proc (macOS): ps prints nothing for a process that is gone, and Z for a zombie
    match Command::new("ps").args(["-o", "stat=", "-p", &pid.to_string()]).output() {
        Ok(o) => {
            let state = String::from_utf8_lossy(&o.stdout);
            state.trim().is_empty() || state.trim_start().starts_with('Z')
        }
        Err(_) => true,
    }
}

fn wait_gone(pid: u64) -> bool {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !gone(pid) {
        if Instant::now() > deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    true
}

#[test]
fn protocol_errors() {
    let mut c = Client::start();
    c.initialize();
    assert_eq!(c.request("ping", json!({}))["result"], json!({}));
    assert_eq!(c.request("nope/method", json!({}))["error"]["code"], -32601);
    assert_eq!(c.request("tools/call", json!({"name": "no_such_tool"}))["error"]["code"], -32602);
    assert_eq!(c.request("tools/call", json!({"arguments": {}}))["error"]["code"], -32602);
    c.send_line("{this is not json");
    let r = c.read();
    assert_eq!((&r["error"]["code"], &r["id"]), (&json!(-32700), &Value::Null));
    c.send_line(r#"[{"jsonrpc":"2.0","id":99,"method":"ping"}]"#);
    assert_eq!(c.read()["error"]["code"], -32600);
    // Bad arguments are the tool's error, for the model to correct
    let r = c.call_result("state", json!({"bogus": 1}));
    assert_eq!(r["isError"], true);
    assert!(r["content"][0]["text"].as_str().unwrap().contains("no argument `bogus`"));
    let r = c.call_result("run_once", json!({"args": ["run", "no/such/image.elf"], "time": "1ms"}));
    assert_eq!(r["isError"], false, "the command ran; it failed");
    assert_eq!(r["structuredContent"]["exit_code"], 1, "{r}");
    assert!(c.close().success());
}

#[test]
fn tools_drive_a_run() {
    let mut c = Client::start();
    c.initialize();
    let tools = c.request("tools/list", json!({}));
    let names: Vec<&str> =
        tools["result"]["tools"].as_array().unwrap().iter().map(|t| t["name"].as_str().unwrap()).collect();
    for t in ["run_once", "start", "attach", "list", "stop", "state", "logs", "can_frames", "inject_fault"] {
        assert!(names.contains(&t), "{t} in {names:?}");
    }
    for t in ["set_input", "pause", "resume", "rewind", "wait_for", "console"] {
        assert!(names.contains(&t), "{t} in {names:?}");
    }

    // One run to completion
    let r = c.call("run_once", json!({"args": ["run", "test_programs/blink.elf"], "time": "10ms"}));
    assert_eq!(r["exit_code"], 0, "{r}");
    assert!(r["stopped"].as_str().unwrap().starts_with("stopped at 0.010000s (TimeLimit)"), "{r}");
    let r = c.call(
        "run_once",
        json!({"args": ["run", "test_programs/pico/hello_uart.elf"], "time": "20ms", "grep": "uart0"}),
    );
    assert_eq!(r["output"], "[0.006337s] uart0: AB Hello, UART!", "{r}");

    // A session: the UART program, paused at 0.5 s
    let s = c.call(
        "start",
        json!({"args": ["run", "test_programs/pico/hello_uart.elf"], "time": "600s", "pause_at_s": 0.5}),
    );
    let (id, pid) = (s["session"].as_str().unwrap().to_string(), s["pid"].as_u64().unwrap());
    assert!(s["url"].as_str().unwrap().starts_with("http://127.0.0.1:"));
    assert!(!gone(pid));
    let w = c.call("wait_for", json!({"session": id, "time_s": 0.5, "timeout_s": 30}));
    assert_eq!(w["matched"], "time", "{w}");
    assert_eq!(w["paused"], true, "pause_at_s paused it");
    let t = w["time_s"].as_f64().unwrap();
    assert!((0.5..0.52).contains(&t), "paused at {t}");
    // Its log, after a cursor
    let l = c.call("logs", json!({}));
    let lines = l["lines"].as_array().unwrap();
    assert_eq!(lines.len(), 1, "{l}");
    assert_eq!(lines[0]["text"], "AB Hello, UART!");
    assert_eq!(lines[0]["kind"], "uart0");
    let next = l["next"].as_u64().unwrap();
    assert_eq!(c.call("logs", json!({"since": next}))["lines"], json!([]), "nothing new after the cursor");
    let w = c.call("wait_for", json!({"pattern": "Hello, UART", "timeout_s": 5}));
    assert_eq!((&w["matched"], &w["line"]["text"]), (&json!("log"), &json!("AB Hello, UART!")));
    let w = c.call("wait_for", json!({"pattern": "Hello", "since": next, "timeout_s": 5}));
    assert_eq!(w["matched"], Value::Null);
    assert!(w["reason"].as_str().unwrap().contains("paused"), "{w}");
    // A fault, seen in the state
    let f = c.call("inject_fault", json!({"fault": "gpio:3:high"}));
    assert_eq!(f["ok"], true, "{f}");
    let r = c.call_result("inject_fault", json!({"fault": "warp-drive"}));
    assert_eq!(r["isError"], true);
    assert!(r["content"][0]["text"].as_str().unwrap().contains("unknown fault"), "{r}");
    let st = c.call("state", json!({"sections": ["gpio", "cores", "faults"]}));
    let b = &st["boards"][0];
    assert!(b["gpio"]["high"].as_array().unwrap().contains(&json!("gpio3")), "{st}");
    assert_eq!(b["cores"].as_array().unwrap().len(), 2);
    assert_eq!(st["faults"][0]["fault"], "gpio:3:high");
    assert_eq!(st["paused"], true);
    assert!(st["boards"][0].get("logs").is_none(), "only the sections asked for");
    // Run on to 0.6 s
    let r = c.call("resume", json!({"until_s": 0.6}));
    assert!(r["message"].as_str().unwrap().contains("running until 0.600000s"), "{r}");
    let w = c.call("wait_for", json!({"time_s": 0.6, "timeout_s": 30}));
    assert_eq!(w["matched"], "time", "{w}");
    // Back to 0.25 s (paused there: nothing runs on), then forward again through what was run
    c.call("pause", json!({}));
    let r = c.call("rewind", json!({"time_s": 0.25}));
    assert_eq!(r["ok"], true, "{r}");
    assert!(r["message"].as_str().unwrap().starts_with("at 0.25"), "{r}");
    let st = c.call("state", json!({"sections": ["cores"]}));
    assert!((0.249..0.2501).contains(&st["time_s"].as_f64().unwrap()), "{st}");
    let r = c.call_result("rewind", json!({}));
    assert_eq!(r["isError"], true);
    let r = c.call("rewind", json!({"save": "early"}));
    assert!(r["message"].as_str().unwrap().contains("saved"), "{r}");
    c.call("resume", json!({"until_s": 0.4}));
    c.call("wait_for", json!({"time_s": 0.4, "timeout_s": 30}));
    let r = c.call("rewind", json!({"checkpoint": "early"}));
    assert!(r["message"].as_str().unwrap().starts_with("at 0.25"), "{r}");
    let r = c.call_result("rewind", json!({"checkpoint": "nope"}));
    assert_eq!(r["isError"], true);
    c.call("resume", json!({"until_s": 0.6}));
    let w = c.call("wait_for", json!({"time_s": 0.6, "timeout_s": 30}));
    assert_eq!(w["matched"], "time", "{w}");
    let listed = c.call("list", json!({}));
    assert_eq!(listed["sessions"][0]["session"], id.as_str());
    assert_eq!(listed["sessions"][0]["process"], "running");
    let console = c.call("console", json!({"grep": "dashboard on"}));
    assert_eq!(console["lines"].as_array().unwrap().len(), 1, "{console}");
    // Stopped: the child is gone
    let stopped = c.call("stop", json!({"session": id}));
    assert_eq!(stopped["stopped"], "killed by signal 9", "{stopped}");
    assert!(wait_gone(pid), "the emulator {pid} still runs after stop");
    assert_eq!(c.call("list", json!({}))["sessions"], json!([]));

    // The run being over: the state says so, and faults are refused
    let s = c.call("start", json!({"args": ["run", "test_programs/blink.elf"], "time": "5ms"}));
    let pid2 = s["pid"].as_u64().unwrap();
    let w = c.call("wait_for", json!({"pattern": "never logged", "timeout_s": 30}));
    assert!(w["reason"].as_str().unwrap().contains("finished"), "{w}");
    let st = c.call("state", json!({}));
    assert_eq!(st["finished"], "TimeLimit");
    assert_eq!(c.call_result("inject_fault", json!({"fault": "panic"}))["isError"], true);
    // Attached to the same dashboard as another session; stopping that only detaches
    let a = c.call("attach", json!({"url": s["url"]}));
    let r = c.call_result("state", json!({}));
    assert!(r["content"][0]["text"].as_str().unwrap().contains("several sessions"), "{r}");
    let d = c.call("stop", json!({"session": a["session"]}));
    assert!(d["stopped"].as_str().unwrap().contains("detached"));
    assert!(!gone(pid2));

    // Closing stdin stops the server, and it takes its emulators with it
    assert!(c.close().success());
    assert!(wait_gone(pid2), "the emulator {pid2} outlived the server");
}

/// A titan_firmware_rs image on its board with a mock bridge: firmware logs, CANmore traffic, decoded status.
#[test]
fn a_rust_board_with_a_mock_bridge() {
    let Some(image) = sim_host::titan_rs_image(&repo(), "template") else {
        return;
    };
    let image = image.to_str().unwrap();
    let mut c = Client::start();
    c.initialize();
    let args = json!(["run", image, "--board", "boards/mk2/power_board.toml", "--mock-bridge", "1"]);
    let r = c.call("run_once", json!({"args": args, "time": "1s"}));
    assert_eq!(r["exit_code"], 0, "{r}");
    assert!(r["output"].as_str().unwrap().contains("CAN controller online as client 1"), "{r}");

    let s = c.call("start", json!({"args": args, "time": "600s", "realtime": true}));
    let pid = s["pid"].as_u64().unwrap();
    let w = c.call("wait_for", json!({"pattern": "ROS connected", "timeout_s": 30}));
    assert_eq!(w["matched"], "log", "{w}");
    let w = c.call("wait_for", json!({"time_s": 2.5, "timeout_s": 30, "pause": true}));
    assert_eq!((&w["matched"], &w["paused"]), (&json!("time"), &json!(true)), "{w}");
    let l = c.call("logs", json!({"kinds": ["info"], "grep": "online"}));
    assert_eq!(l["lines"].as_array().unwrap().len(), 1, "{l}");
    let st = c.call("state", json!({}));
    let b = &st["boards"][0];
    assert_eq!(b["bridge"]["status"]["client_id"], 1, "{st}");
    assert!(b["bridge"]["heartbeats"].as_u64().unwrap() > 0, "{st}");
    assert!(b["inputs"]["stbd_battery"].is_number());
    assert!(st.get("other_bridges").is_none(), "the board's bridge is shown once");
    assert!(st["buses"][0]["frames"].as_u64().unwrap() > 5);
    let f = c.call("can_frames", json!({"max": 5}));
    assert_eq!(f["frames"].as_array().unwrap().len(), 5);
    let next = f["next"].as_u64().unwrap();
    let f2 = c.call("can_frames", json!({"since": next, "max": 5}));
    assert_eq!(f2["frames"][0]["seq"], next + 1);
    let id = f["frames"][0]["id"].as_str().unwrap().to_string();
    let only = c.call("can_frames", json!({"id": id, "max": 2000}));
    assert!(only["frames"].as_array().unwrap().iter().all(|x| x["id"] == id.as_str()));
    let r = c.call("set_input", json!({"name": "stbd_battery", "value": 14.0}));
    assert_eq!(r["ok"], true, "{r}");
    assert_eq!(c.call("state", json!({"sections": ["inputs"]}))["boards"][0]["inputs"]["stbd_battery"], 14.0);
    let r = c.call("inject_fault", json!({"fault": "bridge-stop"}));
    assert_eq!(r["ok"], true, "{r}");
    c.call("stop", json!({}));
    assert!(wait_gone(pid));
    assert!(c.close().success());
}

/// A nano-ROS board with its mock agent: the agent's state, and sending on a topic through it.
#[test]
fn a_nanoros_agent_through_the_tools() {
    let mut c = Client::start();
    c.initialize();
    let args = json!(["robot", "robots/nanoros_led.toml", "--mock-agent", "nanoros:led"]);
    let s = c.call("start", json!({"args": args, "time": "600s", "realtime": true}));
    let pid = s["pid"].as_u64().unwrap();
    let w = c.call("wait_for", json!({"pattern": "done reg", "timeout_s": 30}));
    assert_eq!(w["matched"], "log", "{w}");
    let r = c.call("agent_send", json!({"board": "led", "topic": "bit_sub", "msg": {"data": 7}}));
    assert_eq!(r["ok"], true, "{r}");
    let w = c.call("wait_for", json!({"pattern": "bit_sub 7", "timeout_s": 30}));
    assert_eq!(w["matched"], "log", "{w}");
    let r = c.call_result("agent_send", json!({"topic": "imu_data"}));
    assert_eq!(r["isError"], true);
    assert!(r["content"][0]["text"].as_str().unwrap().contains("publishers"), "{r}");
    let w = c.call("wait_for", json!({"time_s": 1.5, "timeout_s": 30, "pause": true}));
    assert_eq!(w["paused"], true, "{w}");
    let st = c.call("state", json!({"sections": ["agent"]}));
    let a = &st["boards"][0]["agent"];
    assert_eq!((&a["protocol"], &a["connected"]), (&json!("nanoros"), &json!(true)), "{st}");
    let topics = a["topics"].as_array().unwrap();
    let echo = topics.iter().find(|t| t["name"] == "bit_echo").unwrap();
    assert_eq!(echo["last"]["data"], 8, "{st}");
    let fw = topics.iter().find(|t| t["name"] == "firmware_status").unwrap();
    assert_eq!(fw["last"]["board_name"], "nanoros_led", "{st}");
    c.call("stop", json!({}));
    assert!(wait_gone(pid));
    assert!(c.close().success());
}

#[test]
fn the_project_mcp_config_runs_this_server() {
    let cfg: Value = serde_json::from_str(&std::fs::read_to_string(repo().join(".mcp.json")).unwrap()).unwrap();
    let server = &cfg["mcpServers"]["firmware_simulator"];
    assert_eq!(server["args"], json!(["mcp"]));
    let cmd = server["command"].as_str().unwrap();
    assert!(Path::new(cmd).ends_with("target/release/firmware_simulator"), "{cmd}");
}
