//! The MCP tools: running the emulator, and reading and driving a run through its dashboard API.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{Map, Value, json};

use super::http;
use super::procs::{self, Proc};

/// Characters kept of one console or log line.
const LINE_CHARS: usize = 500;

/// What a tool returns: text for the model, and the same as JSON where there is a structure.
pub struct ToolOutput {
    pub text: String,
    pub structured: Option<Value>,
}

impl ToolOutput {
    fn json(v: Value) -> ToolOutput {
        ToolOutput { text: v.to_string(), structured: Some(v) }
    }
}

/// A run the tools read and drive: one this server started, or one attached to.
pub struct Session {
    pub id: String,
    pub addr: SocketAddr,
    pub proc_: Option<Arc<Proc>>,
    pub command: String,
}

pub struct Tools {
    /// The emulator binary children run (this one).
    exe: PathBuf,
    sessions: Mutex<Vec<Arc<Session>>>,
    next_id: AtomicU64,
}

/// The subcommands `run_once` takes, and those `start` takes.
const ONCE_SUBCOMMANDS: [&str; 6] = ["run", "robot", "picoboot", "can-diff", "bridges", "can-tunnel"];
const START_SUBCOMMANDS: [&str; 3] = ["run", "robot", "picoboot"];

/// State sections, and those `state` shows by default.
const SECTIONS: [&str; 11] =
    ["cores", "leds", "gpio", "inputs", "bridge", "logs", "strips", "escs", "buses", "vehicles", "faults"];
const DEFAULT_SECTIONS: [&str; 10] =
    ["cores", "leds", "inputs", "bridge", "logs", "strips", "escs", "buses", "vehicles", "faults"];

/// The tools as `tools/list` describes them: name, title, description, input schema, read only.
pub fn definitions() -> Vec<Value> {
    let session =
        json!({"type": "string", "description": "Session id from start or attach (optional when there is only one)."});
    let board = json!({"type": "string", "description": "Board name as in the state (a robot file's board names; optional for a single-board run)."});
    let cwd = json!({"type": "string", "description": "Working directory for relative paths (default: the server's, normally the repository root)."});
    let args = json!({"type": "array", "items": {"type": "string"}, "description": "Command line after `firmware_simulator`, one argument per item, e.g. [\"run\", \"test_programs/blink.elf\"] or [\"robot\", \"robots/talos.toml\", \"--mock-bridge\", \"internal:1\"]."});
    let time =
        json!({"type": "string", "description": "Virtual time to run for, e.g. \"500ms\", \"2s\" (adds --time)."});
    let faults: Vec<String> = sim_host::dashboard::FAULT_HELP.iter().map(|(f, h)| format!("`{f}` ({h})")).collect();
    let tool = |name: &str, title: &str, description: &str, properties: Value, required: &[&str], read_only: bool| {
        json!({
            "name": name,
            "title": title,
            "description": description,
            "inputSchema": {"type": "object", "properties": properties, "required": required, "additionalProperties": false},
            "annotations": {"title": title, "readOnlyHint": read_only, "openWorldHint": false},
        })
    };
    vec![
        tool(
            "run_once",
            "Run the emulator once",
            "Runs one firmware_simulator command to completion and returns its console output: firmware log lines \
             (`[virtual time] fw-time LEVEL message`), UART lines, the final `stopped at ...` line and the table of \
             unmodelled registers touched. Use it for \"does this image boot, what does it log, does it crash\". \
             Long output keeps its start and end; narrow it with `grep`. Paths are relative to `cwd`.",
            json!({
                "args": args,
                "time": time,
                "timeout_s": {"type": "number", "description": "Wall-clock limit in seconds (default 120, at most 1800); the run is killed after it."},
                "max_lines": {"type": "integer", "description": "Most output lines returned (default 200)."},
                "grep": {"type": "string", "description": "Only return lines matching this regular expression."},
                "cwd": cwd,
            }),
            &["args"],
            false,
        ),
        tool(
            "start",
            "Start an emulator session",
            "Starts a long-running `run` or `robot` (or `picoboot load`) in the background with its dashboard on a free \
             local port, waits until it answers, and returns a session id for the other tools. The run keeps \
             serving its final state after its `time` is up, until `stop`. Without `realtime` virtual time runs as \
             fast as the host allows (a light board does seconds per wall second), so pass a long `time`, \
             `realtime: true`, or `pause_at_s` to have something to interact with.",
            json!({
                "args": args,
                "time": time,
                "realtime": {"type": "boolean", "description": "Pace virtual time to the wall clock (adds --realtime)."},
                "pause_at_s": {"type": "number", "description": "Pause when virtual time reaches this many seconds (0: as soon as it starts)."},
                "timeout_s": {"type": "number", "description": "How long to wait for it to come up (default 60)."},
                "cwd": cwd,
            }),
            &["args"],
            false,
        ),
        tool(
            "attach",
            "Attach to a running emulator",
            "Attaches to an emulator someone else started with `--dashboard ADDR`, by its URL (e.g. \
             `http://127.0.0.1:8080/`), and returns a session id. `stop` on it only detaches.",
            json!({"url": {"type": "string", "description": "The dashboard's URL or address."}}),
            &["url"],
            false,
        ),
        tool(
            "list",
            "List sessions",
            "Lists the sessions: id, URL, command, whether the emulator process still runs, and its virtual time.",
            json!({}),
            &[],
            true,
        ),
        tool(
            "stop",
            "Stop a session",
            "Stops a session: kills the emulator this server started (or detaches from an attached one) and returns \
             the end of its console output.",
            json!({"session": session}),
            &[],
            false,
        ),
        tool(
            "state",
            "Read the run's state",
            "The run's state as JSON: virtual time, paused or finished, applied faults; per board its cores (state \
             and function), LEDs, physical inputs, the mock bridge's decoded firmware status (fault bits by name, \
             kill switch, heartbeats), WS2812 strips, ESC RPMs and its last log lines; per bus frame count, rate, \
             load and errors; vehicles. Compact by default: pick `board` and `sections` to narrow it, add \
             \"gpio\" for pin levels, directions and functions, or `full` for the dashboard's raw JSON.",
            json!({
                "session": session,
                "board": board,
                "sections": {"type": "array", "items": {"type": "string", "enum": SECTIONS}, "description": "Sections to include (default all but gpio)."},
                "log_lines": {"type": "integer", "description": "Log lines per board (default 5; use `logs` to read more)."},
                "full": {"type": "boolean", "description": "Return the dashboard's whole state (large)."},
            }),
            &[],
            true,
        ),
        tool(
            "logs",
            "Read log lines",
            "Log and UART lines of one board or all boards, after a cursor: pass the `next` of the previous call as \
             `since` to read only what is new. Each line has `seq`, `board`, virtual time `t` (s), `kind` (defmt level \
             such as info/warn/error, or uart0/uart1) and `text`. `truncated` means lines after `since` were already \
             dropped (2000 are kept per board).",
            json!({
                "session": session,
                "board": board,
                "since": {"type": "integer", "description": "Cursor: return lines after this seq (default 0, the start)."},
                "max": {"type": "integer", "description": "Most lines returned (default 100)."},
                "tail": {"type": "boolean", "description": "Return the newest `max` lines after `since` instead of the oldest."},
                "grep": {"type": "string", "description": "Only lines whose text matches this regular expression."},
                "kinds": {"type": "array", "items": {"type": "string"}, "description": "Only these kinds, e.g. [\"warn\", \"error\"]."},
            }),
            &[],
            true,
        ),
        tool(
            "can_frames",
            "Read CAN frames",
            "Frames that went over a CAN bus, after a cursor (`since`: the bus's frame count; pass the previous \
             `next`), optionally only IDs matching `id` under `mask`. Each frame has `seq`, `t`, `id` (hex), \
             `data` (hex), `candump`, `acked`, `error` (an injected bit error) and `sender`. The last 2000 frames per \
             bus are kept.",
            json!({
                "session": session,
                "bus": {"type": "string", "description": "Bus name (optional with one bus; a robot's bus names are in the state's buses)."},
                "since": {"type": "integer", "description": "Cursor (default 0)."},
                "max": {"type": "integer", "description": "Most frames returned (default 50)."},
                "tail": {"type": "boolean", "description": "Return the newest `max` frames instead of the oldest."},
                "id": {"type": "string", "description": "Only frames with this ID (hex `0x...` or decimal), compared under `mask`."},
                "mask": {"type": "string", "description": "ID bits that must match `id` (default all)."},
            }),
            &[],
            true,
        ),
        tool(
            "inject_fault",
            "Inject a fault",
            &format!(
                "Injects a hardware fault on a board now, in the CLI's `--fault` spelling. Some: {}. Bridge and kill \
                 faults act on the mock bridge (`--mock-bridge`). The outcome is recorded in the state's faults.",
                faults.join(", ")
            ),
            json!({"session": session, "board": board, "fault": {"type": "string", "description": "The fault, e.g. \"can-unplug\" or \"i2c-absent:0x76\"."}}),
            &["fault"],
            false,
        ),
        tool(
            "set_input",
            "Set a physical input",
            "Sets one of a board's physical inputs (sensor values and supply voltages, e.g. `stbd_battery`, \
             `sht41.temperature_c`, `depth.pressure_pa`); the state's inputs list a board's names and values.",
            json!({"session": session, "board": board, "name": {"type": "string"}, "value": {"type": "number"}}),
            &["name", "value"],
            false,
        ),
        tool(
            "pause",
            "Pause virtual time",
            "Pauses virtual time now, or with `at_s` when it reaches that many seconds (within the run loop's 10 ms \
             pieces). While paused nothing runs, so state reads stay put.",
            json!({"session": session, "at_s": {"type": "number", "description": "Pause when virtual time gets here instead of now."}}),
            &[],
            false,
        ),
        tool(
            "rewind",
            "Go back in virtual time",
            "Takes the whole run (every board, bus, vehicle and the water) back to virtual time `time_s` (the last \
             100 µs slice boundary at or before it), or to a checkpoint a scenario's `pause` saved by name, exactly \
             as it was then: runs are deterministic, so the past is replayed from the nearest checkpoint. Going on \
             (resume, wait_for) re-executes the recorded history (log lines already read are not repeated) and then \
             runs live; setting an input or injecting a fault in the past starts a new future from there. A time \
             past what has run stops where the run has got. Pausing first keeps the run from going on at once. \
             Needs time travel, which runs started with a dashboard (as `start` does) have.",
            json!({
                "session": session,
                "time_s": {"type": "number", "description": "Virtual time to go to, in seconds."},
                "checkpoint": {"type": "string", "description": "A checkpoint saved by name (a scenario's pause step)."},
            }),
            &[],
            false,
        ),
        tool(
            "resume",
            "Resume virtual time",
            "Resumes a paused run, optionally only for `for_s` virtual seconds or until `until_s`, then pauses again \
             (use wait_for with `time_s` to wait for that).",
            json!({
                "session": session,
                "for_s": {"type": "number", "description": "Run this many virtual seconds, then pause."},
                "until_s": {"type": "number", "description": "Run until this virtual time, then pause."},
            }),
            &[],
            false,
        ),
        tool(
            "wait_for",
            "Wait for a log line or a time",
            "Blocks until a board logs a line matching `pattern` (after the cursor `since`), or virtual time reaches \
             `time_s`, whichever comes first, or until `timeout_s` of wall time. Returns what happened (`matched`: \
             \"log\", \"time\", or null with the reason: timed out, the run is paused, the run finished), the line, \
             and the cursor to continue from. With `pause` the run is paused when it happens (exactly at `time_s`, \
             give or take 10 ms).",
            json!({
                "session": session,
                "board": board,
                "pattern": {"type": "string", "description": "Substring of the line's text (or a regular expression with `regex`)."},
                "regex": {"type": "boolean", "description": "Treat `pattern` as a regular expression."},
                "time_s": {"type": "number", "description": "Virtual time to wait for, in seconds."},
                "since": {"type": "integer", "description": "Log cursor to search after (default 0: lines already logged count)."},
                "timeout_s": {"type": "number", "description": "Wall-clock limit in seconds (default 30, at most 600)."},
                "pause": {"type": "boolean", "description": "Pause the run when the condition is met."},
            }),
            &[],
            false,
        ),
        tool(
            "console",
            "Read the emulator's console",
            "The console output (stdout and stderr) of an emulator this server started: setup messages, log lines, \
             `--candump` lines, and once the run is over its `stopped at ...` line and unmodelled-register table.",
            json!({
                "session": session,
                "lines": {"type": "integer", "description": "How many of the last lines to return (default 40)."},
                "grep": {"type": "string", "description": "Only lines matching this regular expression."},
            }),
            &[],
            true,
        ),
    ]
}

/// A tool's arguments, checked against what it takes.
struct Args<'a>(&'a Map<String, Value>);

impl Args<'_> {
    fn str(&self, k: &str) -> Result<Option<&str>, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::String(s)) => Ok(Some(s)),
            Some(v) => Err(format!("`{k}` must be a string, not {v}")),
        }
    }

    fn req_str(&self, k: &str) -> Result<&str, String> {
        self.str(k)?.ok_or_else(|| format!("`{k}` is required"))
    }

    fn f64(&self, k: &str) -> Result<Option<f64>, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(v) => match v.as_f64().or_else(|| v.as_str()?.trim().parse().ok()) {
                Some(x) if x.is_finite() => Ok(Some(x)),
                _ => Err(format!("`{k}` must be a number, not {v}")),
            },
        }
    }

    fn u64(&self, k: &str) -> Result<Option<u64>, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(v) => v
                .as_u64()
                .or_else(|| v.as_f64().filter(|x| *x >= 0.0 && x.fract() == 0.0).map(|x| x as u64))
                .map(Some)
                .ok_or_else(|| format!("`{k}` must be a whole number, 0 or more, not {v}")),
        }
    }

    fn bool(&self, k: &str) -> Result<bool, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(false),
            Some(Value::Bool(b)) => Ok(*b),
            Some(v) => Err(format!("`{k}` must be true or false, not {v}")),
        }
    }

    fn strings(&self, k: &str) -> Result<Option<Vec<String>>, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::Array(a)) => a
                .iter()
                .map(|v| v.as_str().map(str::to_string).ok_or_else(|| format!("`{k}` must be a list of strings")))
                .collect::<Result<Vec<_>, _>>()
                .map(Some),
            Some(_) => Err(format!("`{k}` must be a list of strings")),
        }
    }

    /// An ID or mask: a JSON number, or a string in hex (`0x...`) or decimal.
    fn id(&self, k: &str) -> Result<Option<u64>, String> {
        match self.0.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::Number(n)) => n.as_u64().map(Some).ok_or_else(|| format!("`{k}` must be 0 or more")),
            Some(Value::String(s)) => {
                let s = s.trim();
                let v = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
                    Some(h) => u64::from_str_radix(h, 16),
                    None => s.parse(),
                };
                v.map(Some).map_err(|_| format!("`{k}`: not a number: {s:?}"))
            }
            Some(v) => Err(format!("`{k}` must be a number or a string, not {v}")),
        }
    }

    fn regex(&self, k: &str) -> Result<Option<regex::Regex>, String> {
        self.str(k)?.map(|p| regex::Regex::new(p).map_err(|e| format!("`{k}`: {e}"))).transpose()
    }
}

/// Cuts a line to [`LINE_CHARS`].
fn cut(line: &str) -> String {
    match line.char_indices().nth(LINE_CHARS) {
        Some((at, _)) => format!("{}… ({} more bytes)", &line[..at], line.len() - at),
        None => line.to_string(),
    }
}

/// Keeps the first quarter and the rest from the end of `lines` when there are more than `max`, with a note
/// where lines were left out.
pub fn shorten(lines: Vec<String>, max: usize) -> (Vec<String>, usize) {
    let max = max.max(2);
    if lines.len() <= max {
        return (lines, 0);
    }
    let head = max / 4;
    let tail = max - head - 1;
    let omitted = lines.len() - head - tail;
    let mut out: Vec<String> = lines[..head].to_vec();
    out.push(format!("… {omitted} lines left out (narrow with grep or raise max_lines) …"));
    out.extend_from_slice(&lines[lines.len() - tail..]);
    (out, omitted)
}

/// Rounds the virtual times in a live read (`time_s`, and `t` of its lines or frames) to the nanosecond.
fn round_times(v: &mut Value) {
    if let Some(t) = v["time_s"].as_f64() {
        v["time_s"] = round(Some(t), 9);
    }
    for k in ["lines", "frames"] {
        for x in v.get_mut(k).and_then(Value::as_array_mut).into_iter().flatten() {
            if let Some(t) = x["t"].as_f64() {
                x["t"] = round(Some(t), 9);
            }
        }
    }
}

fn round(x: Option<f64>, places: i32) -> Value {
    match x {
        Some(x) => {
            let f = 10f64.powi(places);
            json!((x * f).round() / f)
        }
        None => Value::Null,
    }
}

/// The run's state as `state` shows it: one board or all, only the sections asked for, without the page's
/// layout data.
pub fn compact_state(st: &Value, board: Option<&str>, sections: &[String], log_lines: usize) -> Result<Value, String> {
    let has = |s: &str| sections.iter().any(|x| x == s);
    let boards = st["boards"].as_array().cloned().unwrap_or_default();
    let names: Vec<&str> = boards.iter().filter_map(|b| b["name"].as_str()).collect();
    if let Some(b) = board
        && !names.contains(&b)
    {
        return Err(format!("no board {b:?}; there are: {}", names.join(", ")));
    }
    let mut out = Map::new();
    out.insert("time_s".into(), round(st["time_s"].as_f64(), 6));
    out.insert("wall_s".into(), round(st["wall_s"].as_f64(), 2));
    out.insert("speed".into(), round(st["speed"].as_f64(), 3));
    for k in ["paused", "pause_at_s", "realtime", "finished"] {
        out.insert(k.into(), st[k].clone());
    }
    if has("faults") {
        out.insert("faults".into(), st["faults"].clone());
    }
    let mut bs = Vec::new();
    for b in boards.iter().filter(|b| board.is_none_or(|n| b["name"] == n)) {
        let mut o = Map::new();
        for k in ["name", "image", "powered", "resets", "bus"] {
            if !b[k].is_null() {
                o.insert(k.into(), b[k].clone());
            }
        }
        if has("cores") {
            let cores: Vec<Value> = b["cores"]
                .as_array()
                .into_iter()
                .flatten()
                .map(|c| {
                    let f = c["function"].as_str().map(cut);
                    json!({"state": c["state"], "function": f, "pc": c["pc"]})
                })
                .collect();
            o.insert("cores".into(), cores.into());
        }
        if has("leds") {
            let leds: Map<String, Value> = b["leds"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|l| Some((l["name"].as_str()?.to_string(), l["on"].clone())))
                .collect();
            o.insert("leds".into(), leds.into());
        }
        if has("gpio") {
            let g = &b["gpio"];
            let (level, oe) = (g["level"].as_u64().unwrap_or(0), g["oe"].as_u64().unwrap_or(0));
            let label = |p: usize| g["labels"][p].as_str().map_or_else(|| format!("gpio{p}"), str::to_string);
            let n = g["labels"].as_array().map_or(30, Vec::len);
            let high: Vec<String> = (0..n).filter(|p| level & (1 << p) != 0).map(label).collect();
            let outputs: Vec<String> = (0..n).filter(|p| oe & (1 << p) != 0).map(label).collect();
            let funcs: Map<String, Value> = (0..n)
                .filter_map(|p| {
                    let f = g["funcs"][p].as_str().filter(|f| !f.is_empty())?;
                    Some((label(p), f.into()))
                })
                .collect();
            o.insert(
                "gpio".into(),
                json!({"level": format!("{level:#010x}"), "high": high, "outputs": outputs, "funcs": funcs}),
            );
        }
        if has("inputs") {
            let inputs: Map<String, Value> = b["inputs"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|i| Some((i["name"].as_str()?.to_string(), i["value"].clone())))
                .collect();
            o.insert("inputs".into(), inputs.into());
        }
        for k in ["strips", "escs"] {
            if has(k) && b[k].as_array().is_some_and(|a| !a.is_empty()) {
                o.insert(k.into(), b[k].clone());
            }
        }
        if has("bridge") && !b["link"].is_null() {
            o.insert("bridge".into(), b["link"].clone());
        }
        if has("logs") && log_lines > 0 {
            let logs = b["logs"].as_array().cloned().unwrap_or_default();
            let tail: Vec<Value> = logs[logs.len().saturating_sub(log_lines)..]
                .iter()
                .map(|l| json!({"seq": l["seq"], "t": round(l["t"].as_f64(), 9), "kind": l["kind"], "text": l["text"].as_str().map(cut)}))
                .collect();
            o.insert("logs".into(), tail.into());
        }
        bs.push(Value::Object(o));
    }
    out.insert("boards".into(), bs.into());
    if has("bridge") {
        // A robot's bridges that are not any one board's (a single board's run has only its own)
        let other: Vec<Value> = st["links"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|l| l.get("board").is_some_and(Value::is_null))
            .cloned()
            .collect();
        if !other.is_empty() {
            out.insert("other_bridges".into(), other.into());
        }
    }
    if has("buses") {
        let buses: Vec<Value> = st["buses"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|b| {
                json!({
                    "name": b["name"], "frames": b["frames"], "rate_fps": round(b["rate_fps"].as_f64(), 1),
                    "load": round(b["load"].as_f64(), 4), "errors": b["errors"], "unacked": b["unacked"],
                })
            })
            .collect();
        out.insert("buses".into(), buses.into());
    }
    if has("vehicles") && st["vehicles"].as_array().is_some_and(|v| !v.is_empty()) {
        out.insert("vehicles".into(), st["vehicles"].clone());
    }
    Ok(Value::Object(out))
}

impl Tools {
    pub fn new() -> anyhow::Result<Tools> {
        Ok(Tools { exe: std::env::current_exe()?, sessions: Mutex::default(), next_id: AtomicU64::new(1) })
    }

    /// Calls tool `name`; `None` if there is no such tool.
    pub fn call(&self, name: &str, args: &Map<String, Value>) -> Option<Result<ToolOutput, String>> {
        let def = definitions().into_iter().find(|d| d["name"] == name)?;
        let known = def["inputSchema"]["properties"].as_object().cloned().unwrap_or_default();
        if let Some(k) = args.keys().find(|k| !known.contains_key(*k)) {
            let names: Vec<&String> = known.keys().collect();
            return Some(Err(format!(
                "{name} takes no argument `{k}`; it takes: {}",
                names.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ")
            )));
        }
        let a = Args(args);
        Some(match name {
            "run_once" => self.run_once(&a),
            "start" => self.start(&a),
            "attach" => self.attach(&a),
            "list" => Ok(self.list()),
            "stop" => self.stop(&a),
            "state" => self.state(&a),
            "logs" => self.logs(&a),
            "can_frames" => self.can_frames(&a),
            "inject_fault" => self.inject_fault(&a),
            "set_input" => self.set_input(&a),
            "pause" => self.pause(&a),
            "resume" => self.resume(&a),
            "rewind" => self.rewind(&a),
            "wait_for" => self.wait_for(&a),
            "console" => self.console(&a),
            _ => unreachable!("{name} is defined but not handled"),
        })
    }

    /// Kills every emulator this server started.
    pub fn shutdown(&self) {
        self.sessions.lock().unwrap().clear();
        procs::kill_all();
    }

    fn command_args(a: &Args, allowed: &[&str]) -> Result<Vec<String>, String> {
        let mut args = a.strings("args")?.ok_or("`args` is required")?;
        let sub = args.first().ok_or("`args` is empty: give the subcommand first, e.g. [\"run\", \"IMAGE\"]")?;
        if !allowed.contains(&sub.as_str()) {
            return Err(format!("the subcommand must be one of {}, not {sub:?}", allowed.join(", ")));
        }
        if let Some(t) = a.str("time")? {
            if args.iter().any(|x| x == "--time" || x == "-t" || x.starts_with("--time=")) {
                return Err("give the time once: in `time` or as --time in `args`".into());
            }
            args.push("--time".into());
            args.push(t.into());
        }
        Ok(args)
    }

    fn cwd(a: &Args) -> Result<Option<PathBuf>, String> {
        match a.str("cwd")? {
            Some(d) if !Path::new(d).is_dir() => Err(format!("cwd {d:?} is not a directory")),
            d => Ok(d.map(PathBuf::from)),
        }
    }

    fn run_once(&self, a: &Args) -> Result<ToolOutput, String> {
        let args = Self::command_args(a, &ONCE_SUBCOMMANDS)?;
        let timeout = a.f64("timeout_s")?.unwrap_or(120.0).clamp(1.0, 1800.0);
        let max_lines = a.u64("max_lines")?.unwrap_or(200) as usize;
        let grep = a.regex("grep")?;
        let cwd = Self::cwd(a)?;
        let p = Proc::spawn(&self.exe, &args, cwd.as_deref())?;
        let status = p.wait_timeout(Duration::from_secs_f64(timeout));
        let timed_out = status.is_none();
        let status = if timed_out { p.kill() } else { status };
        p.drain();
        let wall = p.started.elapsed().as_secs_f64();
        let (lines, total) = p.lines();
        let stopped = lines.iter().rev().find(|l| l.starts_with("stopped at ")).cloned();
        let lines: Vec<String> =
            lines.iter().filter(|l| grep.as_ref().is_none_or(|g| g.is_match(l))).map(|l| cut(l)).collect();
        let matching = lines.len();
        let (lines, omitted) = shorten(lines, max_lines);
        let output = lines.join("\n");
        let status_text = procs::describe(status);
        let mut head = format!("$ {}\n{status_text} after {wall:.2} s wall", p.command);
        if timed_out {
            head.push_str(&format!(" (killed: still running after the {timeout} s timeout)"));
        }
        if grep.is_some() {
            head.push_str(&format!("; {matching} of {total} lines match"));
        }
        let structured = json!({
            "command": p.command,
            "status": status_text,
            "exit_code": status.and_then(|s| s.code()),
            "timed_out": timed_out,
            "wall_s": round(Some(wall), 2),
            "lines_total": total,
            "lines_omitted": omitted,
            "stopped": stopped,
            "output": output,
        });
        Ok(ToolOutput { text: format!("{head}\n\n{output}"), structured: Some(structured) })
    }

    fn add_session(&self, addr: SocketAddr, proc_: Option<Arc<Proc>>, command: String) -> Arc<Session> {
        let id = format!("s{}", self.next_id.fetch_add(1, Ordering::Relaxed));
        let s = Arc::new(Session { id, addr, proc_, command });
        self.sessions.lock().unwrap().push(s.clone());
        s
    }

    fn start(&self, a: &Args) -> Result<ToolOutput, String> {
        let mut args = Self::command_args(a, &START_SUBCOMMANDS)?;
        if args.iter().any(|x| x.starts_with("--dashboard")) {
            return Err("leave out --dashboard: start adds its own".into());
        }
        if a.bool("realtime")? && !args.iter().any(|x| x == "--realtime") {
            args.push("--realtime".into());
        }
        let pause_at = a.f64("pause_at_s")?;
        if pause_at.is_some_and(|t| t < 0.0) {
            return Err("`pause_at_s` must be 0 or more".into());
        }
        let timeout = Duration::from_secs_f64(a.f64("timeout_s")?.unwrap_or(60.0).clamp(1.0, 600.0));
        args.extend(["--dashboard".into(), "127.0.0.1:0".into(), "--dashboard-hold".into()]);
        // Known to the run loop from the start: a fast run would be well past it before a posted pause arrived
        if let Some(t) = pause_at {
            args.extend(["--dashboard-pause-at".into(), format!("{t}s")]);
        }
        let cwd = Self::cwd(a)?;
        let p = Proc::spawn(&self.exe, &args, cwd.as_deref())?;
        let deadline = Instant::now() + timeout;
        let fail = |p: &Proc, why: String| {
            let status = p.kill();
            p.drain();
            let (lines, _) = p.lines();
            let (tail, _) = shorten(lines.iter().map(|l| cut(l)).collect(), 40);
            format!("{why} ({}). Its output:\n{}", procs::describe(status), tail.join("\n"))
        };
        // Up once its dashboard has said where it is and published a first state
        let (addr, state) = loop {
            if let Some(addr) = p.dashboard()
                && let Ok((200, st)) = http::get(addr, "/api/state")
                && st.get("time_s").is_some()
            {
                break (addr, st);
            }
            if p.status().is_some() {
                return Err(fail(&p, "the emulator exited before its dashboard came up".into()));
            }
            if Instant::now() >= deadline {
                return Err(fail(&p, format!("its dashboard did not come up within {} s", timeout.as_secs_f64())));
            }
            std::thread::sleep(Duration::from_millis(20));
        };
        let s = self.add_session(addr, Some(p.clone()), p.command.clone());
        let (lines, _) = p.lines();
        let console: Vec<String> = lines.iter().take(30).map(|l| cut(l)).collect();
        Ok(ToolOutput::json(json!({
            "session": s.id,
            "url": format!("http://{addr}/"),
            "pid": p.pid,
            "command": p.command,
            "time_s": round(state["time_s"].as_f64(), 9),
            "paused": state["paused"],
            "pause_at_s": state["pause_at_s"],
            "boards": state["boards"].as_array().into_iter().flatten().map(|b| b["name"].clone()).collect::<Vec<_>>(),
            "buses": state["buses"].as_array().into_iter().flatten().map(|b| b["name"].clone()).collect::<Vec<_>>(),
            "console": console,
        })))
    }

    fn attach(&self, a: &Args) -> Result<ToolOutput, String> {
        let url = a.req_str("url")?;
        let addr = http::parse_addr(url)?;
        let (code, st) = http::get(addr, "/api/state")?;
        if code != 200 || st.get("time_s").is_none() {
            return Err(format!("{url} does not look like a firmware_simulator dashboard (HTTP {code}: {st})"));
        }
        let s = self.add_session(addr, None, format!("attached to http://{addr}/"));
        Ok(ToolOutput::json(json!({
            "session": s.id,
            "url": format!("http://{addr}/"),
            "time_s": round(st["time_s"].as_f64(), 9),
            "finished": st["finished"],
            "boards": st["boards"].as_array().into_iter().flatten().map(|b| b["name"].clone()).collect::<Vec<_>>(),
            "buses": st["buses"].as_array().into_iter().flatten().map(|b| b["name"].clone()).collect::<Vec<_>>(),
        })))
    }

    fn list(&self) -> ToolOutput {
        let sessions = self.sessions.lock().unwrap().clone();
        let list: Vec<Value> = sessions
            .iter()
            .map(|s| {
                let status = s.proc_.as_ref().map(|p| procs::describe(p.status()));
                let live = http::get(s.addr, "/api/logs?since=18446744073709551615").ok().map(|r| r.1);
                json!({
                    "session": s.id,
                    "url": format!("http://{}/", s.addr),
                    "command": s.command,
                    "pid": s.proc_.as_ref().map(|p| p.pid),
                    "process": status.unwrap_or_else(|| "attached".into()),
                    "time_s": live.as_ref().map(|l| round(l["time_s"].as_f64(), 9)),
                    "paused": live.as_ref().map(|l| l["paused"].clone()),
                    "finished": live.as_ref().map(|l| l["finished"].clone()),
                })
            })
            .collect();
        ToolOutput::json(json!({"sessions": list}))
    }

    fn session(&self, a: &Args) -> Result<Arc<Session>, String> {
        let sessions = self.sessions.lock().unwrap();
        let ids = || sessions.iter().map(|s| s.id.as_str()).collect::<Vec<_>>().join(", ");
        match a.str("session")? {
            Some(id) => sessions.iter().find(|s| s.id == id).cloned().ok_or_else(|| {
                format!("no session {id:?}; there are: {}", if sessions.is_empty() { "none".into() } else { ids() })
            }),
            None => match sessions.as_slice() {
                [s] => Ok(s.clone()),
                [] => Err("no session: start one with `start` or `attach`".into()),
                _ => Err(format!("several sessions: say which with `session` ({})", ids())),
            },
        }
    }

    /// What went wrong reaching a session, with why if its emulator has exited.
    fn unreachable(&self, s: &Session, e: String) -> String {
        match &s.proc_ {
            Some(p) if p.status().is_some() => {
                p.drain();
                let (lines, _) = p.lines();
                let tail: Vec<String> = lines.iter().rev().take(20).rev().map(|l| cut(l)).collect();
                format!(
                    "session {}'s emulator has ended ({}). Its last output:\n{}",
                    s.id,
                    procs::describe(p.status()),
                    tail.join("\n")
                )
            }
            _ => format!("session {}: {e}", s.id),
        }
    }

    fn get(&self, s: &Session, path: &str) -> Result<Value, String> {
        match http::get(s.addr, path) {
            Ok((200, v)) => Ok(v),
            Ok((code, v)) => Err(v["error"].as_str().map_or_else(|| format!("HTTP {code}: {v}"), str::to_string)),
            Err(e) => Err(self.unreachable(s, e)),
        }
    }

    fn post(&self, s: &Session, path: &str, body: &Value) -> Result<Value, String> {
        match http::post(s.addr, path, body) {
            Ok((200, v)) => Ok(v),
            Ok((code, v)) => Err(v["error"].as_str().map_or_else(|| format!("HTTP {code}: {v}"), str::to_string)),
            Err(e) => Err(self.unreachable(s, e)),
        }
    }

    fn stop(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        self.sessions.lock().unwrap().retain(|x| x.id != s.id);
        let Some(p) = &s.proc_ else {
            return Ok(ToolOutput::json(json!({"session": s.id, "stopped": "detached (the emulator keeps running)"})));
        };
        let status = p.kill();
        p.drain();
        let (lines, total) = p.lines();
        let tail: Vec<String> = lines.iter().rev().take(30).rev().map(|l| cut(l)).collect();
        Ok(ToolOutput::json(json!({
            "session": s.id,
            "pid": p.pid,
            "stopped": procs::describe(status),
            "lines_total": total,
            "console_tail": tail,
        })))
    }

    fn state(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let board = a.str("board")?;
        let sections = match a.strings("sections")? {
            Some(v) => {
                if let Some(bad) = v.iter().find(|x| !SECTIONS.contains(&x.as_str())) {
                    return Err(format!("no section {bad:?}; there are: {}", SECTIONS.join(", ")));
                }
                v
            }
            None => DEFAULT_SECTIONS.iter().map(|x| x.to_string()).collect(),
        };
        let st = self.get(&s, "/api/state?fresh=1")?;
        if st.get("time_s").is_none() {
            return Err("the run has not published a state yet; try again in a moment".into());
        }
        if a.bool("full")? {
            let mut st = st;
            if let Some(b) = board
                && let Some(boards) = st["boards"].as_array_mut()
            {
                boards.retain(|x| x["name"] == b);
            }
            return Ok(ToolOutput::json(st));
        }
        let log_lines = a.u64("log_lines")?.unwrap_or(5) as usize;
        Ok(ToolOutput::json(compact_state(&st, board, &sections, log_lines)?))
    }

    fn logs(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let since = a.u64("since")?.unwrap_or(0);
        let max = a.u64("max")?.unwrap_or(100).clamp(1, 2000) as usize;
        let tail = a.bool("tail")?;
        let grep = a.regex("grep")?;
        let kinds = a.strings("kinds")?;
        let filtered = grep.is_some() || kinds.is_some();
        let mut path = format!("/api/logs?since={since}&max={}", if filtered { 2000 } else { max });
        if tail {
            path.push_str("&tail=1");
        }
        if let Some(b) = a.str("board")? {
            path.push_str(&format!("&board={}", http::encode(b)));
        }
        let mut v = self.get(&s, &path)?;
        let lines = v["lines"].as_array().cloned().unwrap_or_default();
        let mut keep: Vec<Value> = lines
            .into_iter()
            .filter(|l| {
                let text = l["text"].as_str().unwrap_or("");
                let kind = l["kind"].as_str().unwrap_or("");
                grep.as_ref().is_none_or(|g| g.is_match(text))
                    && kinds.as_ref().is_none_or(|k| k.iter().any(|x| x.eq_ignore_ascii_case(kind)))
            })
            .collect();
        if keep.len() > max {
            v["more"] = true.into();
            if tail {
                keep.drain(..keep.len() - max);
            } else {
                keep.truncate(max);
                v["next"] = keep.last().map_or(Value::Null, |l| l["seq"].clone());
            }
        }
        for l in &mut keep {
            if let Some(t) = l["text"].as_str() {
                l["text"] = cut(t).into();
            }
        }
        v["lines"] = keep.into();
        round_times(&mut v);
        Ok(ToolOutput::json(v))
    }

    fn can_frames(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let since = a.u64("since")?.unwrap_or(0);
        let max = a.u64("max")?.unwrap_or(50).clamp(1, 2000);
        let mut path = format!("/api/frames?since={since}&max={max}");
        if a.bool("tail")? {
            path.push_str("&tail=1");
        }
        if let Some(b) = a.str("bus")? {
            path.push_str(&format!("&bus={}", http::encode(b)));
        }
        if let Some(id) = a.id("id")? {
            path.push_str(&format!("&id={id}"));
        }
        if let Some(m) = a.id("mask")? {
            path.push_str(&format!("&mask={m}"));
        }
        let mut v = self.get(&s, &path)?;
        if let Some(frames) = v["frames"].as_array_mut() {
            for f in frames {
                if let Some(o) = f.as_object_mut() {
                    for k in ["kind", "bus", "t_ns", "start_ns", "brs", "rtr"] {
                        o.remove(k);
                    }
                    if o.get("error").is_some_and(Value::is_null) {
                        o.remove("error");
                    }
                }
            }
        }
        round_times(&mut v);
        Ok(ToolOutput::json(v))
    }

    fn inject_fault(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let body = json!({"board": a.str("board")?.unwrap_or(""), "fault": a.req_str("fault")?});
        Ok(ToolOutput::json(self.post(&s, "/api/fault", &body)?))
    }

    fn set_input(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let value = a.f64("value")?.ok_or("`value` is required")?;
        let body = json!({"board": a.str("board")?.unwrap_or(""), "name": a.req_str("name")?, "value": value});
        Ok(ToolOutput::json(self.post(&s, "/api/input", &body)?))
    }

    fn pause(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let mut body = json!({"paused": true});
        if let Some(t) = a.f64("at_s")? {
            body["at_s"] = t.into();
        }
        Ok(ToolOutput::json(self.post(&s, "/api/pause", &body)?))
    }

    fn rewind(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let body = match (a.f64("time_s")?, a.str("checkpoint")?) {
            (Some(t), None) => json!({"time_s": t}),
            (None, Some(name)) => json!({"checkpoint": name}),
            _ => return Err("give `time_s` or `checkpoint`".into()),
        };
        Ok(ToolOutput::json(self.post(&s, "/api/rewind", &body)?))
    }

    /// Where the run is now (not as of the last published state).
    fn live(&self, s: &Session) -> Result<Value, String> {
        self.get(s, &format!("/api/logs?since={}", u64::MAX))
    }

    fn resume(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let mut body = json!({"paused": false});
        match (a.f64("for_s")?, a.f64("until_s")?) {
            (Some(_), Some(_)) => return Err("give `for_s` or `until_s`, not both".into()),
            (Some(d), None) => {
                let now = self.live(&s)?["time_s"].as_f64().unwrap_or(0.0);
                body["at_s"] = (now + d.max(0.0)).into();
            }
            (None, Some(t)) => body["at_s"] = t.into(),
            (None, None) => {}
        }
        Ok(ToolOutput::json(self.post(&s, "/api/pause", &body)?))
    }

    fn wait_for(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let pattern = a.str("pattern")?;
        let time_s = a.f64("time_s")?;
        if pattern.is_none() && time_s.is_none() {
            return Err("give `pattern`, `time_s` or both".into());
        }
        let matcher = match (pattern, a.bool("regex")?) {
            (Some(p), true) => Some(regex::Regex::new(p).map_err(|e| format!("`pattern`: {e}"))?),
            (Some(p), false) => Some(regex::Regex::new(&regex::escape(p)).unwrap()),
            (None, _) => None,
        };
        let timeout = a.f64("timeout_s")?.unwrap_or(30.0).clamp(0.0, 600.0);
        let pause = a.bool("pause")?;
        let mut since = a.u64("since")?.unwrap_or(0);
        let board = a.str("board")?.map(|b| format!("&board={}", http::encode(b))).unwrap_or_default();
        // Pausing at a time is best left to the run loop, which does it exactly
        if pause && let Some(t) = time_s {
            self.post(&s, "/api/pause", &json!({"paused": true, "at_s": t}))?;
        }
        let started = Instant::now();
        let deadline = started + Duration::from_secs_f64(timeout);
        let answer = |matched: Value, mut line: Value, reason: Value, next: u64, live: &Value| {
            if let Some(t) = line["t"].as_f64() {
                line["t"] = round(Some(t), 9);
            }
            ToolOutput::json(json!({
                "matched": matched,
                "line": line,
                "reason": reason,
                "next": next,
                "time_s": round(live["time_s"].as_f64(), 9),
                "paused": live["paused"],
                "finished": live["finished"],
                "waited_s": round(Some(started.elapsed().as_secs_f64()), 2),
            }))
        };
        loop {
            let v = self.get(&s, &format!("/api/logs?since={since}&max=2000{board}"))?;
            let lines = v["lines"].as_array().cloned().unwrap_or_default();
            if let Some(m) = &matcher
                && let Some(l) = lines.iter().find(|l| m.is_match(l["text"].as_str().unwrap_or("")))
            {
                let mut v = v.clone();
                if pause {
                    v = self.post(&s, "/api/pause", &json!({"paused": true})).map(|_| self.live(&s))??;
                }
                let next = l["seq"].as_u64().unwrap_or(since);
                return Ok(answer("log".into(), l.clone(), Value::Null, next, &v));
            }
            since = v["next"].as_u64().unwrap_or(since);
            if v["more"] == true {
                continue;
            }
            let now = v["time_s"].as_f64().unwrap_or(0.0);
            if let Some(t) = time_s
                && now >= t - 1e-12
            {
                return Ok(answer("time".into(), Value::Null, Value::Null, since, &v));
            }
            let why = if !v["finished"].is_null() {
                Some(format!("the run finished ({})", v["finished"].as_str().unwrap_or("")))
            } else if v["paused"] == true {
                Some("the run is paused (resume it first)".into())
            } else if Instant::now() >= deadline {
                Some(format!("timed out after {timeout} s of wall time"))
            } else {
                None
            };
            if let Some(why) = why {
                return Ok(answer(Value::Null, Value::Null, why.into(), since, &v));
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    fn console(&self, a: &Args) -> Result<ToolOutput, String> {
        let s = self.session(a)?;
        let p = s.proc_.as_ref().ok_or("an attached session's console is wherever it was started")?;
        let n = a.u64("lines")?.unwrap_or(40) as usize;
        let grep = a.regex("grep")?;
        let (lines, total) = p.lines();
        let matching: Vec<&String> = lines.iter().filter(|l| grep.as_ref().is_none_or(|g| g.is_match(l))).collect();
        let shown: Vec<String> = matching[matching.len().saturating_sub(n)..].iter().map(|l| cut(l)).collect();
        Ok(ToolOutput::json(json!({
            "session": s.id,
            "process": procs::describe(p.status()),
            "lines_total": total,
            "lines": shown,
        })))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: Value) -> Map<String, Value> {
        v.as_object().unwrap().clone()
    }

    #[test]
    fn every_tool_is_defined_and_handled() {
        let t = Tools::new().unwrap();
        let defs = definitions();
        assert_eq!(defs.len(), 15);
        for d in &defs {
            let name = d["name"].as_str().unwrap();
            assert!(d["description"].as_str().unwrap().len() > 40, "{name} needs a description");
            assert_eq!(d["inputSchema"]["type"], "object");
            for r in d["inputSchema"]["required"].as_array().unwrap() {
                assert!(d["inputSchema"]["properties"].get(r.as_str().unwrap()).is_some(), "{name}: {r}");
            }
            // Handled: no session yet, or bad arguments, but never "no such tool"
            assert!(t.call(name, &Map::new()).is_some(), "{name}");
        }
        assert!(t.call("nope", &Map::new()).is_none());
    }

    #[test]
    fn arguments_are_checked() {
        let t = Tools::new().unwrap();
        let err = |name: &str, v: Value| match t.call(name, &args(v)).unwrap() {
            Err(e) => e,
            Ok(o) => panic!("{name} succeeded: {}", o.text),
        };
        assert!(err("state", json!({"bord": "x"})).contains("no argument `bord`"));
        assert!(err("run_once", json!({})).contains("`args` is required"));
        assert!(err("run_once", json!({"args": "run x"})).contains("list of strings"));
        assert!(err("run_once", json!({"args": []})).contains("empty"));
        assert!(err("run_once", json!({"args": ["mcp"]})).contains("must be one of"));
        assert!(err("start", json!({"args": ["bridges", "robots/talos.toml"]})).contains("must be one of run, robot"));
        assert!(err("run_once", json!({"args": ["run", "x", "--time", "1s"], "time": "2s"})).contains("once"));
        assert!(err("start", json!({"args": ["run", "x", "--dashboard", "127.0.0.1:1"]})).contains("--dashboard"));
        assert!(err("run_once", json!({"args": ["run", "x"], "grep": "("})).contains("grep"));
        assert!(err("run_once", json!({"args": ["run", "x"], "timeout_s": "soon"})).contains("number"));
        assert!(err("state", json!({})).contains("no session"));
        assert!(err("state", json!({"session": "s9"})).contains("no session \"s9\""));
        assert!(err("attach", json!({"url": "https://x:1"})).contains("plain http"));
    }

    #[test]
    fn long_output_keeps_its_start_and_end() {
        let lines: Vec<String> = (0..1000).map(|i| format!("line {i}")).collect();
        let (short, omitted) = shorten(lines.clone(), 100);
        assert_eq!(short.len(), 100);
        assert_eq!(omitted, 901);
        assert_eq!(short[0], "line 0");
        assert!(short[25].contains("901 lines left out"));
        assert_eq!(short.last().unwrap(), "line 999");
        assert_eq!(shorten(lines[..10].to_vec(), 100), (lines[..10].to_vec(), 0));
        assert!(cut(&"x".repeat(2000)).ends_with("(1500 more bytes)"));
    }

    #[test]
    fn times_are_rounded_without_adding_fields() {
        let mut v = json!({"time_s": 0.21000030769199998, "lines": [{"t": 0.200400331692}]});
        round_times(&mut v);
        assert_eq!(v, json!({"time_s": 0.210000308, "lines": [{"t": 0.200400332}]}));
    }

    fn sample_state() -> Value {
        json!({
            "time_s": 1.234567891, "wall_s": 2.71, "speed": 0.98765, "paused": false, "pause_at_s": null,
            "realtime": false, "finished": null, "faults": [], "fault_help": [{"fault": "can-stuck", "help": "..."}],
            "boards": [
                {"name": "a", "image": "a.elf", "powered": true, "resets": 0, "bus": "internal", "fault_names": ["F0"],
                 "cores": [{"state": "running", "pc": "0x10000100", "function": "main+0x4", "exception": 0}],
                 "gpio": {"level": 0x0200_0001u32, "oe": 0x0200_0000u32, "labels": ["gpio0", "gpio1", "gpio2"],
                          "funcs": ["UART", "", "SIO"]},
                 "leds": [{"name": "status_led", "pin": 25, "on": true}], "strips": [], "escs": [],
                 "inputs": [{"name": "vbat", "value": 12.0}],
                 "logs": (0..8).map(|i| json!({"seq": i, "t": 0.1, "kind": "info", "text": format!("l{i}")})).collect::<Vec<_>>(),
                 "link": {"kind": "can", "heartbeats": 3}},
                {"name": "b", "powered": false, "cores": [], "logs": [], "link": null},
            ],
            "buses": [{"name": "internal", "frames": 10, "rate_fps": 12.345, "load": 0.0123456, "errors": 0, "unacked": 1, "recent": [1, 2]}],
            "links": [{"board": null, "bus": "internal", "kind": "can"}],
            "vehicles": [],
        })
    }

    #[test]
    fn the_state_is_compacted() {
        let st = sample_state();
        let all: Vec<String> = DEFAULT_SECTIONS.iter().map(|s| s.to_string()).collect();
        let c = compact_state(&st, None, &all, 5).unwrap();
        assert_eq!(c["time_s"], 1.234568);
        assert!(c.get("fault_help").is_none());
        let a = &c["boards"][0];
        assert_eq!(a["leds"], json!({"status_led": true}));
        assert_eq!(a["inputs"], json!({"vbat": 12.0}));
        assert_eq!(a["logs"].as_array().unwrap().len(), 5);
        assert_eq!(a["logs"][4]["text"], "l7");
        assert_eq!(a["bridge"]["heartbeats"], 3);
        assert!(a.get("gpio").is_none() && a.get("strips").is_none() && a.get("fault_names").is_none());
        assert_eq!(c["buses"][0]["load"], 0.0123);
        assert!(c["buses"][0].get("recent").is_none());
        assert_eq!(c["other_bridges"][0]["bus"], "internal");
        // A single board's run has no other bridges: its links name no board
        let mut single = st.clone();
        single["links"] = json!([{"kind": "can"}]);
        assert!(compact_state(&single, None, &all, 5).unwrap().get("other_bridges").is_none());
        assert!(c.get("vehicles").is_none(), "no vehicles, no section");
        // One board, chosen sections
        let c = compact_state(&st, Some("a"), &["gpio".to_string()], 5).unwrap();
        assert_eq!(c["boards"].as_array().unwrap().len(), 1);
        let g = &c["boards"][0]["gpio"];
        assert_eq!(g["high"], json!(["gpio0"]), "only labelled pins in range");
        assert_eq!(g["funcs"], json!({"gpio0": "UART", "gpio2": "SIO"}));
        assert!(c["boards"][0].get("logs").is_none() && c.get("buses").is_none());
        assert!(compact_state(&st, Some("z"), &all, 5).unwrap_err().contains("there are: a, b"));
    }
}
