//! `firmware_simulator mcp`: a Model Context Protocol server over stdio, so that coding agents can run the
//! emulator, read a run's state, logs and CAN traffic as JSON, and drive it (faults, inputs, pause), through tools
//! instead of the dashboard page.
//!
//! The transport is the MCP stdio one: JSON-RPC 2.0 messages, one per line, on stdin and stdout; logs go to
//! stderr. Tool calls run on their own threads, so a long `wait_for` or `run_once` does not hold up `ping` or other
//! calls. The emulators it starts are its children, run with `--dashboard 127.0.0.1:0`; the tools talk to their
//! dashboards' HTTP API ([`sim_host::dashboard`]). When stdin closes the server kills them and exits (on Linux
//! they are also killed if the server dies some other way).

use std::io::{BufRead, Write};
use std::sync::{Arc, Mutex};

use serde_json::{Map, Value, json};

mod http;
mod procs;
mod rpc;
mod tools;

/// Protocol revisions this server speaks, newest first.
const PROTOCOL_VERSIONS: [&str; 4] = ["2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05"];

const INSTRUCTIONS: &str = "Runs the firmware_simulator RP2040 emulator (UWRT robot boards). For a quick check use \
run_once (\"does it boot, what does it log\"). To interact: start a session (run or robot arguments), then \
wait_for a log line or a virtual time, read state / logs / can_frames (logs and can_frames take a `since` cursor: \
pass back `next` to read only what is new), drive it with inject_fault, set_input, pause and resume, and stop it \
when done. Paths in arguments are relative to the server's working directory unless `cwd` is given.";

#[derive(clap::Args)]
pub struct McpArgs {}

/// Writes messages to stdout, one per line, whole.
#[derive(Clone)]
struct Out(Arc<Mutex<std::io::Stdout>>);

impl Out {
    fn send(&self, v: &Value) {
        let mut line = v.to_string();
        line.push('\n');
        let mut out = self.0.lock().unwrap();
        // If the client has gone, stdin closes too and the server stops there
        let _ = out.write_all(line.as_bytes()).and_then(|_| out.flush());
    }
}

/// The answer to `initialize`: the client's protocol revision if this server speaks it, else the newest.
fn initialize(params: &Value) -> Value {
    let asked = params["protocolVersion"].as_str().unwrap_or("");
    let version = PROTOCOL_VERSIONS.iter().find(|v| **v == asked).unwrap_or(&PROTOCOL_VERSIONS[0]);
    json!({
        "protocolVersion": version,
        "capabilities": {"tools": {"listChanged": false}},
        "serverInfo": {"name": "firmware_simulator", "title": "firmware_simulator RP2040 emulator", "version": env!("CARGO_PKG_VERSION")},
        "instructions": INSTRUCTIONS,
    })
}

/// A tool's outcome as a `tools/call` result.
fn tool_result(r: Result<tools::ToolOutput, String>) -> Value {
    match r {
        Ok(o) => {
            let mut v = json!({"content": [{"type": "text", "text": o.text}], "isError": false});
            if let Some(s) = o.structured.filter(Value::is_object) {
                v["structuredContent"] = s;
            }
            v
        }
        Err(e) => json!({"content": [{"type": "text", "text": e}], "isError": true}),
    }
}

/// Answers one request, or returns `None` for a `tools/call`, which the caller runs on its own thread.
fn answer(id: &Value, method: &str, params: &Value) -> Option<Value> {
    Some(match method {
        "initialize" => rpc::result(id, initialize(params)),
        "ping" => rpc::result(id, json!({})),
        "tools/list" => rpc::result(id, json!({"tools": tools::definitions()})),
        "tools/call" => {
            let name = params["name"].as_str();
            let args = match &params["arguments"] {
                Value::Null => Some(Map::new()),
                Value::Object(m) => Some(m.clone()),
                _ => None,
            };
            match (name, args) {
                (None, _) => rpc::error(id, rpc::INVALID_PARAMS, "tools/call needs the tool's `name`"),
                (_, None) => rpc::error(id, rpc::INVALID_PARAMS, "`arguments` must be an object"),
                (Some(n), Some(_)) if !tools::definitions().iter().any(|d| d["name"] == n) => {
                    rpc::error(id, rpc::INVALID_PARAMS, &format!("unknown tool: {n}"))
                }
                _ => return None,
            }
        }
        _ => rpc::error(id, rpc::METHOD_NOT_FOUND, &format!("method not found: {method}")),
    })
}

/// Serves MCP on stdin and stdout until stdin closes.
pub fn serve(_args: McpArgs) -> anyhow::Result<()> {
    let tools = Arc::new(tools::Tools::new()?);
    let out = Out(Arc::new(Mutex::new(std::io::stdout())));
    log::info!("MCP server ready on stdio");
    let mut stdin = std::io::stdin().lock();
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match stdin.read_until(b'\n', &mut buf) {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) => {
                log::error!("reading stdin: {e}");
                break;
            }
        }
        let line = String::from_utf8_lossy(&buf);
        if line.trim().is_empty() {
            continue;
        }
        match rpc::parse(line.trim()) {
            rpc::Incoming::Request { id, method, params } => match answer(&id, &method, &params) {
                Some(reply) => out.send(&reply),
                None => {
                    let (tools, out) = (tools.clone(), out.clone());
                    std::thread::spawn(move || {
                        let name = params["name"].as_str().unwrap_or("");
                        let args = params["arguments"].as_object().cloned().unwrap_or_default();
                        log::debug!("tools/call {name} {}", Value::Object(args.clone()));
                        let r = tools.call(name, &args).expect("checked before");
                        out.send(&rpc::result(&id, tool_result(r)));
                    });
                }
            },
            // notifications/initialized, notifications/cancelled (calls run to their end; the client ignores
            // the late answer), and anything else: nothing to answer
            rpc::Incoming::Notification { method, .. } => log::debug!("notification {method}"),
            rpc::Incoming::Response => {}
            rpc::Incoming::Invalid { id, code, message } => out.send(&rpc::error(&id, code, &message)),
        }
    }
    log::info!("stdin closed: stopping the emulators and exiting");
    tools.shutdown();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_versions_are_negotiated() {
        assert_eq!(initialize(&json!({"protocolVersion": "2025-06-18"}))["protocolVersion"], "2025-06-18");
        assert_eq!(initialize(&json!({"protocolVersion": "2024-11-05"}))["protocolVersion"], "2024-11-05");
        assert_eq!(initialize(&json!({"protocolVersion": "1999-01-01"}))["protocolVersion"], PROTOCOL_VERSIONS[0]);
        let r = initialize(&json!({}));
        assert_eq!(r["capabilities"]["tools"]["listChanged"], false);
        assert_eq!(r["serverInfo"]["name"], "firmware_simulator");
    }

    #[test]
    fn requests_are_answered_or_refused() {
        let id = json!(1);
        let code = |method: &str, params: Value| answer(&id, method, &params).map(|r| r["error"]["code"].clone());
        assert_eq!(answer(&id, "ping", &Value::Null).unwrap()["result"], json!({}));
        assert!(answer(&id, "tools/list", &Value::Null).unwrap()["result"]["tools"].as_array().unwrap().len() > 10);
        assert_eq!(code("resources/list", Value::Null), Some(json!(rpc::METHOD_NOT_FOUND)));
        assert_eq!(code("tools/call", json!({})), Some(json!(rpc::INVALID_PARAMS)), "no name");
        assert_eq!(code("tools/call", json!({"name": "nope"})), Some(json!(rpc::INVALID_PARAMS)), "unknown tool");
        assert_eq!(code("tools/call", json!({"name": "list", "arguments": [1]})), Some(json!(rpc::INVALID_PARAMS)));
        assert_eq!(code("tools/call", json!({"name": "list"})), None, "runs on its own thread");
    }

    #[test]
    fn tool_results() {
        let ok = tool_result(Ok(tools::ToolOutput { text: "{}".into(), structured: Some(json!({"a": 1})) }));
        assert_eq!(ok["isError"], false);
        assert_eq!(ok["content"][0]["type"], "text");
        assert_eq!(ok["structuredContent"]["a"], 1);
        let plain = tool_result(Ok(tools::ToolOutput { text: "x".into(), structured: Some(json!([1])) }));
        assert!(plain.get("structuredContent").is_none(), "structured content must be an object");
        let err = tool_result(Err("no board \"x\"".into()));
        assert_eq!((&err["isError"], &err["content"][0]["text"]), (&json!(true), &json!("no board \"x\"")));
    }
}
