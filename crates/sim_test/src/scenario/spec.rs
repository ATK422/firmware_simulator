//! The scenario file format: parsing YAML into [`Scenario`]s, with errors that name the step and the key.
//!
//! The reference is `docs/scenarios.md`; this module follows it key for key.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde_yaml::{Mapping, Value};
use sim_core::time::Time;
use sim_host::fault::Fault;
use sim_host::host_protocol::AgentSpec;
use titan_link::HostToDevice;

/// One scenario: what to run, for how long, and the steps.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,
    /// The file it came from, and its place among the file's YAML documents (0 for the first).
    pub file: PathBuf,
    pub document: usize,
    pub setup: Setup,
    pub bridges: Bridges,
    /// Mock agents in a board's own protocol (`nanoros:BOARD`), as `--mock-agent`; their `=CONFIG` paths found.
    pub agents: Vec<AgentSpec>,
    /// Physical inputs set before the boards start: `BOARD.INPUT` (or `INPUT` with one board) and the value.
    pub inputs: Vec<(String, f64)>,
    /// Virtual time to run.
    pub time: Time,
    /// Threads the boards may run on (results are the same on any number).
    pub threads: usize,
    pub steps: Vec<Step>,
    /// Branches run from a snapshot taken once `steps` are done, each with steps of its own.
    pub cases: Vec<Case>,
}

/// What the scenario runs.
#[derive(Clone, Debug)]
pub enum Setup {
    /// A robot file, optionally with only some of its boards.
    Robot { file: PathBuf, boards: Option<Vec<String>> },
    /// One board: an image on a board file.
    Board { name: String, image: PathBuf, file: Option<PathBuf>, rom_boot: bool },
}

/// The mock `titan_bridge`s.
#[derive(Clone, Debug, PartialEq)]
pub enum Bridges {
    /// One for every board's CANmore client on its bus, and one on every Ethernet board's cable.
    All,
    List(Vec<BridgeSpec>),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BridgeSpec {
    /// `BUS:CLIENT` (or `CLIENT` when there is one bus).
    Can { bus: Option<String>, client: u8 },
    /// `eth:BOARD`: on an Ethernet board's cable (Puddles).
    Eth { board: String },
}

impl std::fmt::Display for BridgeSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BridgeSpec::Can { bus: Some(b), client } => write!(f, "{b}:{client}"),
            BridgeSpec::Can { bus: None, client } => write!(f, "{client}"),
            BridgeSpec::Eth { board } => write!(f, "eth:{board}"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Case {
    pub name: String,
    pub steps: Vec<Step>,
}

/// When a step starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Start {
    /// When the step before it finished (the *cursor*).
    Cursor,
    /// At this virtual time.
    At(Time),
    /// This long after the cursor.
    After(Time),
}

/// When a step's window ends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    /// This long after the step's start.
    Within(Time),
    /// At this virtual time.
    Until(Time),
}

#[derive(Clone, Debug)]
pub struct Step {
    /// Its place in the file (1 for the first step), for messages.
    pub number: usize,
    /// The step's `name`, shown instead of the generated description.
    pub label: Option<String>,
    pub start: Start,
    pub end: Option<End>,
    pub kind: StepKind,
}

#[derive(Clone, Debug)]
pub enum StepKind {
    Fault {
        board: Option<String>,
        spec: String,
        fault: Fault,
    },
    Input(Vec<(String, f64)>),
    /// A titan_link message from a board's bridge, once or `every` so often until the window ends.
    Send {
        target: Target,
        message: Box<HostToDevice>,
        every: Option<Time>,
    },
    /// A message on a topic from the board's mock agent (or its bridge's titan_link message, by variant), once or
    /// `every` so often until the window ends.
    SendTopic {
        target: Target,
        topic: String,
        msg: serde_json::Value,
        every: Option<Time>,
    },
    IvcSend {
        board: String,
        byte: u8,
    },
    /// A point to stop at: the dashboard pauses there (`scenario --dashboard`); otherwise nothing happens.
    Pause(String),
    Expect(Condition),
    Never(Condition),
    Always(Condition),
}

impl StepKind {
    pub fn word(&self) -> &'static str {
        match self {
            StepKind::Fault { .. } => "fault",
            StepKind::Input(_) => "input",
            StepKind::Send { .. } | StepKind::SendTopic { .. } => "send",
            StepKind::IvcSend { .. } => "ivc_send",
            StepKind::Pause(_) => "pause",
            StepKind::Expect(_) => "expect",
            StepKind::Never(_) => "never",
            StepKind::Always(_) => "always",
        }
    }

    pub fn is_action(&self) -> bool {
        !matches!(self, StepKind::Expect(_) | StepKind::Never(_) | StepKind::Always(_))
    }

    /// For a `send`: its `every`.
    pub fn send_every(&self) -> Option<Option<Time>> {
        match self {
            StepKind::Send { every, .. } | StepKind::SendTopic { every, .. } => Some(*every),
            _ => None,
        }
    }
}

/// A board, or a bridge given as `BUS:CLIENT` / `eth:BOARD`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Target {
    pub board: Option<String>,
    pub bridge: Option<BridgeSpec>,
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match (&self.board, &self.bridge) {
            (_, Some(b)) => write!(f, "{b}"),
            (Some(b), None) => write!(f, "{b}"),
            (None, None) => Ok(()),
        }
    }
}

/// What an `expect`, `never` or `always` looks at.
#[derive(Clone, Debug)]
pub struct Condition {
    pub target: Target,
    /// The bus, for frames (default: the board's, or the only one).
    pub bus: Option<String>,
    pub what: What,
    /// How many matching occurrences the window must hold (occurrence conditions only).
    pub count: Option<Range>,
    /// Matching occurrences per second over the window.
    pub rate: Option<Range>,
}

#[derive(Clone, Debug)]
pub enum TextMatch {
    Substring(String),
    Regex(regex::Regex),
}

impl TextMatch {
    pub fn matches(&self, s: &str) -> bool {
        match self {
            TextMatch::Substring(t) => s.contains(t.as_str()),
            TextMatch::Regex(r) => r.is_match(s),
        }
    }
}

impl std::fmt::Display for TextMatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TextMatch::Substring(t) => write!(f, "{t:?}"),
            TextMatch::Regex(r) => write!(f, "/{}/", r.as_str()),
        }
    }
}

#[derive(Clone, Debug)]
pub enum What {
    /// A line the board printed: defmt (optionally of one `level`), UART or USB serial.
    Log {
        text: TextMatch,
        level: Option<String>,
    },
    /// Each `FirmwareStatus` the bridge decodes.
    Status(StatusCheck),
    /// The board's titan_link heartbeats reach the bridge (one in the last 300 ms) or not.
    Link(bool),
    Frame(FrameMatch),
    Gpio {
        pin: PinRef,
        high: bool,
    },
    Esc {
        name: Option<String>,
        rpm: Range,
    },
    Vehicle {
        name: Option<String>,
        checks: Vec<(VehicleQty, Range)>,
    },
    /// Chip resets since the start.
    Resets(Range),
    /// Core lockups since the start.
    Lockups(Range),
    /// A byte the board's IVC modem received (any byte if `None`).
    Ivc(Option<u8>),
    /// Each titan_link message from the board whose JSON form has `path` (`CameraCage.Depth`), with its value there
    /// checked.
    Message {
        path: Vec<String>,
        value: Option<ValueCheck>,
    },
    /// Each message on `topic` the board's mock agent gets (or its bridge's titan_link messages of that variant),
    /// with the fields `msg` gives matching ([`msg_mismatch`]).
    Topic {
        topic: String,
        msg: Option<serde_json::Value>,
    },
}

impl What {
    /// Occurrences with a time (lines, frames, messages) rather than a state sampled every millisecond.
    pub fn is_occurrence(&self) -> bool {
        matches!(
            self,
            What::Log { .. }
                | What::Status(_)
                | What::Frame(_)
                | What::Ivc(_)
                | What::Message { .. }
                | What::Topic { .. }
        )
    }

    /// Value streams: at an instant, the latest one counts.
    pub fn is_stream(&self) -> bool {
        matches!(self, What::Status(_) | What::Message { .. } | What::Topic { .. })
    }
}

/// Whether message `v` has what `pattern` asks for, or the first thing it does not: an object's fields each match
/// (others are not looked at); a list item by item, the same length; `{min, max}` (either) a number in that range;
/// a number equal to within a millionth (a float32 decoded keeps its rounding); anything else equal.
pub fn msg_mismatch(pattern: &serde_json::Value, v: &serde_json::Value) -> Option<String> {
    use serde_json::Value as J;
    fn at(path: &str) -> String {
        if path.is_empty() { "the message".into() } else { path.to_string() }
    }
    fn walk(p: &J, v: &J, path: &str) -> Option<String> {
        match (p, v) {
            (J::Object(pm), _) if !pm.is_empty() && pm.keys().all(|k| k == "min" || k == "max") => {
                let Some(x) = v.as_f64() else { return Some(format!("{} is {v}, not a number", at(path))) };
                let lo = pm.get("min").and_then(J::as_f64).unwrap_or(f64::NEG_INFINITY);
                let hi = pm.get("max").and_then(J::as_f64).unwrap_or(f64::INFINITY);
                (x < lo || x > hi).then(|| format!("{} is {x}, not in [{lo}, {hi}]", at(path)))
            }
            (J::Object(pm), J::Object(vm)) => pm.iter().find_map(|(k, pv)| {
                let sub = if path.is_empty() { k.clone() } else { format!("{path}.{k}") };
                match vm.get(k) {
                    Some(x) => walk(pv, x, &sub),
                    None => Some(format!("no field {sub}")),
                }
            }),
            (J::Array(pa), J::Array(va)) if pa.len() == va.len() => {
                pa.iter().zip(va).enumerate().find_map(|(i, (a, b))| walk(a, b, &format!("{path}[{i}]")))
            }
            (J::Number(a), J::Number(b)) => {
                let (a, b) = (a.as_f64().unwrap_or(f64::NAN), b.as_f64().unwrap_or(f64::NAN));
                ((a - b).abs() > 1e-6 * a.abs().max(b.abs()).max(1.0)).then(|| format!("{} is {b}, not {a}", at(path)))
            }
            (p, v) if p == v => None,
            (p, v) => Some(format!("{} is {v}, not {p}", at(path))),
        }
    }
    walk(pattern, v, "")
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PinRef {
    Number(u8),
    Name(String),
}

impl std::fmt::Display for PinRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PinRef::Number(n) => write!(f, "GPIO {n}"),
            PinRef::Name(n) => write!(f, "{n}"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleQty {
    Depth,
    North,
    East,
    Speed,
    Roll,
    Pitch,
    Yaw,
}

impl VehicleQty {
    pub fn name(self) -> &'static str {
        match self {
            VehicleQty::Depth => "depth",
            VehicleQty::North => "north",
            VehicleQty::East => "east",
            VehicleQty::Speed => "speed",
            VehicleQty::Roll => "roll",
            VehicleQty::Pitch => "pitch",
            VehicleQty::Yaw => "yaw",
        }
    }
}

/// An inclusive range of numbers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Range {
    pub min: Option<f64>,
    pub max: Option<f64>,
}

impl Range {
    pub fn exactly(v: f64) -> Range {
        Range { min: Some(v), max: Some(v) }
    }

    pub fn contains(&self, v: f64) -> bool {
        self.min.is_none_or(|m| v >= m) && self.max.is_none_or(|m| v <= m)
    }
}

impl std::fmt::Display for Range {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match (self.min, self.max) {
            (Some(a), Some(b)) if a == b => write!(f, "{a}"),
            (Some(a), Some(b)) => write!(f, "{a}..{b}"),
            (Some(a), None) => write!(f, ">= {a}"),
            (None, Some(b)) => write!(f, "<= {b}"),
            (None, None) => write!(f, "any"),
        }
    }
}

/// A check of a value in a decoded message.
#[derive(Clone, Debug)]
pub enum ValueCheck {
    Range(Range),
    /// Equal to this (a string, a boolean, a list, ...), compared as JSON.
    Equals(serde_json::Value),
}

/// Checks on a `FirmwareStatus`; all given must hold.
#[derive(Clone, Debug, Default)]
pub struct StatusCheck {
    /// The fault bitmask, exactly.
    pub faults: Option<u32>,
    /// Faults that must be raised, by name (`FAULT_DEPTH_INIT_ERROR`, or without `FAULT_`) or bit number.
    pub raised: Vec<FaultRef>,
    /// Faults that must not be raised.
    pub clear: Vec<FaultRef>,
    /// The vehicle is killed: some kill switch asserts kill.
    pub killed: Option<bool>,
    /// Some kill switch has timed out (no `state/kill` forwarded for too long).
    pub kill_timed_out: Option<bool>,
    /// Uptime, seconds.
    pub uptime: Option<Range>,
    pub board_name: Option<String>,
    pub client_id: Option<u8>,
    /// At an instant, the latest status must be at most this old (2.5 s by default: statuses come every second).
    pub max_age: Option<Time>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FaultRef {
    Bit(u8),
    Name(String),
}

impl std::fmt::Display for FaultRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FaultRef::Bit(b) => write!(f, "fault {b}"),
            FaultRef::Name(n) => write!(f, "{n}"),
        }
    }
}

/// Which CAN frames: identifier under a mask, optionally the frame format and the start of the data.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FrameMatch {
    pub id: Option<u32>,
    pub mask: Option<u32>,
    pub extended: Option<bool>,
    /// The data must start with these bytes.
    pub data: Vec<u8>,
}

impl FrameMatch {
    pub fn matches(&self, f: &sim_devices::can::CanFrame) -> bool {
        let mask = self.mask.unwrap_or(if f.extended { 0x1FFF_FFFF } else { 0x7FF });
        self.id.is_none_or(|id| f.id & mask == id & mask)
            && self.extended.is_none_or(|e| e == f.extended)
            && f.data.starts_with(&self.data)
    }
}

impl std::fmt::Display for FrameMatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.id {
            Some(id) => write!(f, "frame {id:#x}")?,
            None => write!(f, "frame")?,
        }
        if let Some(m) = self.mask {
            write!(f, "/{m:#x}")?;
        }
        if !self.data.is_empty() {
            let hex: String = self.data.iter().map(|b| format!("{b:02X}")).collect();
            write!(f, " data {hex}..")?;
        }
        Ok(())
    }
}

// ---- Parsing

/// Parses every scenario in a file (one per YAML document, separated by `---`).
pub fn load_file(path: &Path) -> Result<Vec<Scenario>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse_str(&text, path)
}

/// Parses scenarios from text; `path` is where it came from (paths in it are found from its directory).
pub fn parse_str(text: &str, path: &Path) -> Result<Vec<Scenario>, String> {
    use serde::Deserialize;
    let mut out = Vec::new();
    for (k, doc) in serde_yaml::Deserializer::from_str(text).enumerate() {
        let v = Value::deserialize(doc).map_err(|e| format!("{}: {e}", path.display()))?;
        if v.is_null() {
            continue;
        }
        let s = parse_scenario(&v, path, k).map_err(|e| {
            let doc = if k > 0 { format!(" (document {})", k + 1) } else { String::new() };
            format!("{}{doc}: {e}", path.display())
        })?;
        out.push(s);
    }
    if out.is_empty() {
        return Err(format!("{}: no scenario in it", path.display()));
    }
    Ok(out)
}

type R<T> = Result<T, String>;

fn map<'a>(v: &'a Value, what: &str) -> R<&'a Mapping> {
    v.as_mapping().ok_or_else(|| format!("{what}: expected a map, found {}", show(v)))
}

fn show(v: &Value) -> String {
    let s = serde_yaml::to_string(v).unwrap_or_default();
    let s = s.trim().trim_start_matches("---").trim();
    if s.len() > 60 { format!("{}...", &s[..60]) } else { s.to_string() }
}

fn key_str(k: &Value) -> R<&str> {
    k.as_str().ok_or_else(|| format!("keys must be names, found {}", show(k)))
}

fn check_keys(m: &Mapping, allowed: &[&str], what: &str) -> R<()> {
    for k in m.keys() {
        let k = key_str(k)?;
        if !allowed.contains(&k) {
            return Err(format!("{what}: unknown key `{k}` (expected one of: {})", allowed.join(", ")));
        }
    }
    Ok(())
}

fn get<'a>(m: &'a Mapping, k: &str) -> Option<&'a Value> {
    m.get(Value::String(k.to_string()))
}

fn string(v: &Value, what: &str) -> R<String> {
    match v {
        Value::String(s) => Ok(s.clone()),
        Value::Number(n) => Ok(n.to_string()),
        Value::Bool(b) => Ok(b.to_string()),
        _ => Err(format!("{what}: expected text, found {}", show(v))),
    }
}

fn number(v: &Value, what: &str) -> R<f64> {
    match v {
        Value::Number(n) => n.as_f64().ok_or_else(|| format!("{what}: bad number {n}")),
        Value::String(s) => parse_number(s).ok_or_else(|| format!("{what}: expected a number, found {s:?}")),
        _ => Err(format!("{what}: expected a number, found {}", show(v))),
    }
}

fn parse_number(s: &str) -> Option<f64> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u64::from_str_radix(&h.replace('_', ""), 16).ok().map(|v| v as f64),
        None => s.replace('_', "").parse().ok(),
    }
}

fn integer<T: TryFrom<u64>>(v: &Value, what: &str) -> R<T> {
    let f = number(v, what)?;
    if f < 0.0 || f.fract() != 0.0 {
        return Err(format!("{what}: expected a whole number, found {f}"));
    }
    T::try_from(f as u64).map_err(|_| format!("{what}: {f} is out of range"))
}

fn boolean(v: &Value, what: &str) -> R<bool> {
    v.as_bool().ok_or_else(|| format!("{what}: expected true or false, found {}", show(v)))
}

/// A virtual time or duration: `1.5s`, `200ms`, `50us`, `10ns`, or a bare number of seconds.
pub fn parse_time(s: &str) -> R<Time> {
    let s = s.trim();
    let (num, mult) = if let Some(n) = s.strip_suffix("ns") {
        (n, 1e-9)
    } else if let Some(n) = s.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1.0)
    };
    let v: f64 = num.trim().parse().map_err(|_| format!("bad time {s:?} (write e.g. 1.5s, 200ms)"))?;
    if !(v >= 0.0 && v.is_finite()) {
        return Err(format!("bad time {s:?}: must be positive"));
    }
    // Rounded to the nanosecond, so 0.1 + 0.2 style sums of decimal times are exact
    Ok(Time::from_ns((v * mult * 1e9).round() as u64))
}

fn time(v: &Value, what: &str) -> R<Time> {
    match v {
        Value::Number(n) => parse_time(&n.as_f64().ok_or_else(|| format!("{what}: bad number"))?.to_string())
            .map_err(|e| format!("{what}: {e}")),
        Value::String(s) => parse_time(s).map_err(|e| format!("{what}: {e}")),
        _ => Err(format!("{what}: expected a time (1.5s, 200ms), found {}", show(v))),
    }
}

/// `5`, `[1, 3]`, `{min: 1}`, `{max: 3}`, `{min: 1, max: 3}`.
fn range(v: &Value, what: &str) -> R<Range> {
    match v {
        Value::Sequence(s) if s.len() == 2 => {
            let r = Range { min: Some(number(&s[0], what)?), max: Some(number(&s[1], what)?) };
            if r.min > r.max {
                return Err(format!("{what}: the range {r} is empty"));
            }
            Ok(r)
        }
        Value::Mapping(m) => {
            check_keys(m, &["min", "max"], what)?;
            let r = Range {
                min: get(m, "min").map(|v| number(v, what)).transpose()?,
                max: get(m, "max").map(|v| number(v, what)).transpose()?,
            };
            if r.min.is_none() && r.max.is_none() {
                return Err(format!("{what}: give min, max or both"));
            }
            Ok(r)
        }
        _ => number(v, what)
            .map(Range::exactly)
            .map_err(|_| format!("{what}: expected a number, [min, max], or {{min: .., max: ..}}, found {}", show(v))),
    }
}

fn parse_scenario(v: &Value, path: &Path, document: usize) -> R<Scenario> {
    let m = map(v, "the scenario")?;
    check_keys(
        m,
        &["name", "description", "robot", "board", "bridges", "agents", "inputs", "time", "threads", "steps", "cases"],
        "the scenario",
    )?;
    let name = string(get(m, "name").ok_or("a scenario needs a `name`")?, "name")?;
    let description = get(m, "description").map(|v| string(v, "description")).transpose()?;
    // Paths are searched for from the scenario's directory upward, on its absolute path
    let parent = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    let dir_buf = parent.canonicalize().unwrap_or_else(|_| parent.to_path_buf());
    let dir = dir_buf.as_path();
    let setup = match (get(m, "robot"), get(m, "board")) {
        (Some(r), None) => parse_robot(r, dir)?,
        (None, Some(b)) => parse_board(b, dir)?,
        (Some(_), Some(_)) => return Err("give either `robot` or `board`, not both".into()),
        (None, None) => {
            return Err("a scenario needs a `robot` (a robot file) or a `board` (an image on a board)".into());
        }
    };
    let bridges = match get(m, "bridges") {
        None => Bridges::List(Vec::new()),
        Some(Value::String(s)) if s == "all" => Bridges::All,
        Some(Value::Sequence(list)) => {
            Bridges::List(list.iter().map(|b| parse_bridge(&string(b, "bridges")?)).collect::<R<Vec<_>>>()?)
        }
        Some(other) => {
            return Err(format!(
                "bridges: expected `all` or a list like [internal:1, eth:backplane], found {}",
                show(other)
            ));
        }
    };
    let agents = match get(m, "agents") {
        None => Vec::new(),
        Some(Value::Sequence(list)) => {
            list.iter().map(|a| parse_agent(&string(a, "agents")?, dir)).collect::<R<_>>()?
        }
        Some(other) => return Err(format!("agents: expected a list like [nanoros:led], found {}", show(other))),
    };
    let inputs = match get(m, "inputs") {
        Some(v) => parse_inputs(v, "inputs")?,
        None => Vec::new(),
    };
    let time = time(get(m, "time").ok_or("a scenario needs a `time` (how long to run, e.g. 15s)")?, "time")?;
    if time == Time::ZERO {
        return Err("time: must be more than 0".into());
    }
    let threads = get(m, "threads").map(|v| integer(v, "threads")).transpose()?.unwrap_or(1);
    let steps = match get(m, "steps") {
        Some(v) => parse_steps(v, 0)?,
        None => Vec::new(),
    };
    let mut cases = Vec::new();
    if let Some(v) = get(m, "cases") {
        let list = v.as_sequence().ok_or("cases: expected a list of {name, steps}")?;
        for (k, c) in list.iter().enumerate() {
            let cm = map(c, &format!("case {}", k + 1))?;
            check_keys(cm, &["name", "steps"], &format!("case {}", k + 1))?;
            let name = string(get(cm, "name").ok_or(format!("case {}: needs a `name`", k + 1))?, "name")?;
            if cases.iter().any(|c: &Case| c.name == name) {
                return Err(format!("two cases called {name:?}"));
            }
            let steps = match get(cm, "steps") {
                Some(v) => parse_steps(v, steps.len()).map_err(|e| format!("case {name:?}: {e}"))?,
                None => Vec::new(),
            };
            cases.push(Case { name, steps });
        }
    }
    let s = Scenario {
        name,
        description,
        file: path.to_path_buf(),
        document,
        setup,
        bridges,
        agents,
        inputs,
        time,
        threads,
        steps,
        cases,
    };
    for st in s.steps.iter().chain(s.cases.iter().flat_map(|c| &c.steps)) {
        check_times(st, s.time)?;
    }
    Ok(s)
}

/// Times in steps must fall within the run.
fn check_times(st: &Step, end: Time) -> R<()> {
    let past = |t: Time, key: &str| {
        if t > end {
            Err(format!("step {}: `{key}: {t}` is after the end of the run ({end})", st.number))
        } else {
            Ok(())
        }
    };
    if let Start::At(t) = st.start {
        past(t, "at")?;
    }
    if let Some(End::Until(t)) = st.end {
        past(t, "until")?;
    }
    Ok(())
}

/// Finds `rel` from `dir` or the closest directory above it that has it (so `robots/talos.toml` works from
/// `scenarios/`, and from any directory under the repository).
pub fn find_upward(dir: &Path, rel: &Path) -> Option<PathBuf> {
    if rel.is_absolute() {
        return rel.exists().then(|| rel.to_path_buf());
    }
    let mut d = Some(dir);
    while let Some(here) = d {
        let p = here.join(rel);
        if p.exists() {
            return Some(p);
        }
        d = here.parent();
    }
    None
}

fn find_file(dir: &Path, s: &str, what: &str) -> R<PathBuf> {
    let p = sim_host::expand_path(s)?;
    find_upward(dir, &p)
        .ok_or_else(|| format!("{what}: {s} not found (from {} or a directory above it)", dir.display()))
}

fn parse_robot(v: &Value, dir: &Path) -> R<Setup> {
    let (file, boards) = match v {
        Value::String(s) => (s.clone(), None),
        Value::Mapping(m) => {
            check_keys(m, &["file", "boards"], "robot")?;
            let file = string(get(m, "file").ok_or("robot: needs a `file`")?, "robot.file")?;
            let boards = match get(m, "boards") {
                Some(Value::Sequence(l)) => Some(l.iter().map(|b| string(b, "robot.boards")).collect::<R<Vec<_>>>()?),
                Some(other) => {
                    return Err(format!("robot.boards: expected a list of board names, found {}", show(other)));
                }
                None => None,
            };
            (file, boards)
        }
        _ => return Err(format!("robot: expected a robot file or {{file, boards}}, found {}", show(v))),
    };
    let path = match find_upward(dir, Path::new(&file)) {
        Some(p) => p,
        None if !file.contains('/') && !file.ends_with(".toml") => {
            find_file(dir, &format!("robots/{file}.toml"), "robot")?
        }
        None => find_file(dir, &file, "robot")?,
    };
    Ok(Setup::Robot { file: path, boards })
}

fn parse_board(v: &Value, dir: &Path) -> R<Setup> {
    let m = map(v, "board")?;
    check_keys(m, &["image", "file", "name", "rom_boot"], "board")?;
    let image = string(get(m, "image").ok_or("board: needs an `image`")?, "board.image")?;
    let image_path = resolve_board_image(dir, &image)?;
    let file = match get(m, "file") {
        Some(v) => {
            let f = string(v, "board.file")?;
            Some(match find_upward(dir, Path::new(&f)) {
                Some(p) if f.ends_with(".toml") => p,
                _ => find_file(dir, &format!("boards/{f}.toml"), "board.file")?,
            })
        }
        None => return Err("board: needs a `file` (its board file, e.g. mk2/esc_board)".into()),
    };
    let name = match get(m, "name") {
        Some(v) => string(v, "board.name")?,
        None => image_path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "board".into()),
    };
    let rom_boot = get(m, "rom_boot").map(|v| boolean(v, "board.rom_boot")).transpose()?.unwrap_or(false);
    Ok(Setup::Board { name, image: image_path, file, rom_boot })
}

/// A board image: a path (from the scenario's directory up), or the name of a titan_firmware_rs release image
/// (`esc_board`), found in `$TITAN_RS_RELEASE_DIR` or the repository's build. A missing image is not an error here
/// (the run reports it, or skips).
fn resolve_board_image(dir: &Path, image: &str) -> R<PathBuf> {
    let p = sim_host::expand_path(image)?;
    if let Some(found) = find_upward(dir, &p) {
        return Ok(found);
    }
    if !image.contains('/') {
        if let Some(d) = std::env::var_os("TITAN_RS_RELEASE_DIR") {
            return Ok(PathBuf::from(d).join(image));
        }
        let suffix = Path::new(RS_RELEASE).join(image);
        if let Some(found) = find_upward(dir, &suffix) {
            return Ok(found);
        }
        // Where it would be in the repository the scenario is in
        let repo = find_upward(dir, Path::new("external/titan_firmware_rs"))
            .and_then(|p| p.parent().and_then(Path::parent).map(Path::to_path_buf))
            .unwrap_or_else(|| dir.to_path_buf());
        return Ok(repo.join(suffix));
    }
    Ok(dir.join(p))
}

const RS_RELEASE: &str = "external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release";

fn parse_bridge(s: &str) -> R<BridgeSpec> {
    let s = s.trim();
    if let Some(board) = s.strip_prefix("eth:") {
        return Ok(BridgeSpec::Eth { board: board.to_string() });
    }
    let (bus, client) = match s.rsplit_once(':') {
        Some((b, c)) => (Some(b.to_string()), c),
        None => (None, s),
    };
    let client = parse_number(client)
        .filter(|c| c.fract() == 0.0 && (0.0..=255.0).contains(c))
        .ok_or_else(|| format!("bridge {s:?}: expected BUS:CLIENT (internal:1), CLIENT or eth:BOARD"))?;
    Ok(BridgeSpec::Can { bus, client: client as u8 })
}

/// An agent as `--mock-agent` takes it; its `=CONFIG` path is looked for from the scenario's directory up.
fn parse_agent(s: &str, dir: &Path) -> R<AgentSpec> {
    let mut a: AgentSpec = s.parse().map_err(|e| format!("agents: {e}"))?;
    if let Some(c) = &a.config {
        let p = sim_host::expand_path(&c.to_string_lossy())?;
        a.config = Some(find_upward(dir, &p).ok_or_else(|| format!("agents: {s}: no file {}", p.display()))?);
    }
    Ok(a)
}

fn parse_inputs(v: &Value, what: &str) -> R<Vec<(String, f64)>> {
    let m = map(v, what)?;
    m.iter().map(|(k, v)| Ok((key_str(k)?.to_string(), number(v, &format!("{what}.{}", key_str(k)?))?))).collect()
}

const TIMING: [&str; 4] = ["at", "after", "within", "until"];
const KINDS: [&str; 8] = ["fault", "input", "send", "ivc_send", "pause", "expect", "never", "always"];

fn parse_steps(v: &Value, first: usize) -> R<Vec<Step>> {
    let list = v.as_sequence().ok_or("steps: expected a list")?;
    list.iter().enumerate().map(|(k, s)| parse_step(s, first + k + 1)).collect()
}

fn parse_step(v: &Value, number: usize) -> R<Step> {
    let ctx = |e: String| format!("step {number}: {e}");
    let m = map(v, "a step").map_err(ctx)?;
    let mut allowed: Vec<&str> = KINDS.to_vec();
    allowed.extend(TIMING);
    allowed.push("name");
    check_keys(m, &allowed, "a step").map_err(ctx)?;
    let kinds: Vec<&str> = KINDS.iter().copied().filter(|k| get(m, k).is_some()).collect();
    let kind = match kinds[..] {
        [k] => k,
        [] => return Err(ctx(format!("needs one of: {}", KINDS.join(", ")))),
        _ => return Err(ctx(format!("has {}: one per step", kinds.join(" and ")))),
    };
    let body = get(m, kind).unwrap();
    // Timing keys may sit next to the kind or inside its map, but not both
    let mut timing: BTreeMap<&str, &Value> = BTreeMap::new();
    for k in TIMING {
        if let Some(v) = get(m, k) {
            timing.insert(k, v);
        }
        if let Some(v) = body.as_mapping().and_then(|b| get(b, k))
            && timing.insert(k, v).is_some()
        {
            return Err(ctx(format!("`{k}` is given twice")));
        }
    }
    let t = |k: &str| timing.get(k).map(|v| time(v, k)).transpose().map_err(ctx);
    let start = match (t("at")?, t("after")?) {
        (Some(_), Some(_)) => return Err(ctx("give `at` or `after`, not both".into())),
        (Some(a), None) => Start::At(a),
        (None, Some(d)) => Start::After(d),
        (None, None) => Start::Cursor,
    };
    let end = match (t("within")?, t("until")?) {
        (Some(_), Some(_)) => return Err(ctx("give `within` or `until`, not both".into())),
        (Some(d), None) => Some(End::Within(d)),
        (None, Some(u)) => Some(End::Until(u)),
        (None, None) => None,
    };
    if let (Start::At(a), Some(End::Until(u))) = (start, end)
        && u < a
    {
        return Err(ctx(format!("`until: {u}` is before `at: {a}`")));
    }
    let label = get(m, "name").map(|v| string(v, "name")).transpose().map_err(ctx)?;
    let kind = parse_kind(kind, body).map_err(|e| ctx(format!("{kind}: {e}")))?;
    match &kind {
        k if k.send_every() == Some(None) && end.is_some() => {
            return Err(ctx("send: `within`/`until` only go with `every`".into()));
        }
        k if k.send_every().is_some_and(|e| e.is_some()) && end.is_none() => {
            return Err(ctx("send: `every` needs `within` or `until` to say when to stop".into()));
        }
        k if k.is_action() && k.send_every().is_none() && end.is_some() => {
            return Err(ctx(format!("{}: actions take `at` or `after`, not `within` or `until`", k.word())));
        }
        StepKind::Always(c) | StepKind::Never(c) if c.count.is_some() || c.rate.is_some() => {
            return Err(ctx(format!("{}: `count` and `rate` go with `expect`", kind.word())));
        }
        StepKind::Always(c) if c.what.is_occurrence() && !c.what.is_stream() => {
            return Err(ctx(
                "always: needs a state (link, gpio, esc, vehicle, resets, lockups) or a status or message check; \
                 for lines, frames and IVC bytes that must not happen, use `never`"
                    .into(),
            ));
        }
        StepKind::Expect(c) | StepKind::Never(c) | StepKind::Always(c)
            if !c.what.is_occurrence() && (c.count.is_some() || c.rate.is_some()) =>
        {
            return Err(ctx("`count` and `rate` count lines, frames, statuses, messages or IVC bytes".into()));
        }
        _ => {}
    }
    Ok(Step { number, label, start, end, kind })
}

/// Splits `BOARD:REST` when `BOARD` is not itself part of a fault spec.
fn split_board(s: &str) -> (Option<String>, String) {
    match s.split_once(':') {
        Some((b, rest)) if s.parse::<Fault>().is_err() => (Some(b.to_string()), rest.to_string()),
        _ => (None, s.to_string()),
    }
}

fn without_timing(m: &Mapping) -> Mapping {
    m.iter()
        .filter(|(k, _)| !k.as_str().is_some_and(|k| TIMING.contains(&k)))
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

fn parse_target(m: &Mapping) -> R<Target> {
    Ok(Target {
        board: get(m, "board").map(|v| string(v, "board")).transpose()?,
        bridge: get(m, "bridge").map(|v| parse_bridge(&string(v, "bridge")?)).transpose()?,
    })
}

fn parse_kind(kind: &str, body: &Value) -> R<StepKind> {
    Ok(match kind {
        "fault" => {
            let (board, spec) = match body {
                Value::String(s) => split_board(s),
                Value::Mapping(m) => {
                    let m = without_timing(m);
                    check_keys(&m, &["board", "fault"], "fault")?;
                    let board = get(&m, "board").map(|v| string(v, "board")).transpose()?;
                    (board, string(get(&m, "fault").ok_or("needs `fault` (e.g. can-unplug)")?, "fault")?)
                }
                _ => return Err(format!("expected BOARD:FAULT or {{board, fault}}, found {}", show(body))),
            };
            let fault = spec.parse::<Fault>().map_err(|e| format!("{spec:?}: {e} (faults: see docs/faults.md)"))?;
            StepKind::Fault { board, spec, fault }
        }
        "input" => {
            let m = without_timing(map(body, "input")?);
            StepKind::Input(parse_inputs(&Value::Mapping(m), "input")?)
        }
        "send" if map(body, "send")?.contains_key("topic") => {
            let m = without_timing(map(body, "send")?);
            check_keys(&m, &["board", "bridge", "topic", "msg", "every"], "send")?;
            let target = parse_target(&m)?;
            let topic = string(get(&m, "topic").unwrap(), "topic")?;
            // Through JSON: a message is its fields as the agent's protocol has them
            let msg = match get(&m, "msg") {
                Some(v) => serde_json::to_value(v).map_err(|e| format!("msg: {e}"))?,
                None => serde_json::Value::Null,
            };
            let every = get(&m, "every").map(|v| time(v, "every")).transpose()?;
            if every == Some(Time::ZERO) {
                return Err("every: must be more than 0".into());
            }
            StepKind::SendTopic { target, topic, msg, every }
        }
        "send" => {
            let m = without_timing(map(body, "send")?);
            check_keys(&m, &["board", "bridge", "message", "every"], "send")?;
            let target = parse_target(&m)?;
            if target.board.is_none() && target.bridge.is_none() {
                return Err("needs `board` or `bridge`: whose bridge sends it".into());
            }
            let msg =
                get(&m, "message").ok_or("needs a `message`, e.g. {Esc: {ThrusterRpm: [0, 0, 0, 0, 0, 0, 0, 0]}}")?;
            // Through JSON, whose externally tagged enums are YAML's plain maps (serde_yaml wants `!Tags`)
            let json = serde_json::to_value(msg).map_err(|e| format!("message: {e}"))?;
            let message: HostToDevice = serde_json::from_value(json).map_err(|e| {
                format!("message {}: {e} (titan_link's HostToDevice, e.g. {{Esc: {{ThrusterRpm: [...]}}}})", show(msg))
            })?;
            let every = get(&m, "every").map(|v| time(v, "every")).transpose()?;
            if every == Some(Time::ZERO) {
                return Err("every: must be more than 0".into());
            }
            StepKind::Send { target, message: Box::new(message), every }
        }
        "ivc_send" => {
            let (board, byte) = match body {
                Value::String(s) => {
                    let (b, byte) = s.rsplit_once(':').ok_or("expected BOARD:BYTE (talos_ivc:0x5a)")?;
                    (b.to_string(), Value::String(byte.to_string()))
                }
                Value::Mapping(m) => {
                    let m = without_timing(m);
                    check_keys(&m, &["board", "byte"], "ivc_send")?;
                    let b = string(get(&m, "board").ok_or("needs `board`")?, "board")?;
                    (b, get(&m, "byte").ok_or("needs `byte`")?.clone())
                }
                _ => return Err(format!("expected BOARD:BYTE or {{board, byte}}, found {}", show(body))),
            };
            StepKind::IvcSend { board, byte: integer(&byte, "byte")? }
        }
        "pause" => StepKind::Pause(match body {
            Value::Null => String::new(),
            Value::Mapping(m) if without_timing(m).is_empty() => String::new(),
            v => string(v, "pause")?,
        }),
        "expect" => StepKind::Expect(parse_condition(body)?),
        "never" => StepKind::Never(parse_condition(body)?),
        "always" => StepKind::Always(parse_condition(body)?),
        _ => unreachable!(),
    })
}

const WHATS: [&str; 12] =
    ["log", "status", "link", "frame", "gpio", "esc", "vehicle", "resets", "lockups", "ivc", "message", "topic"];

fn parse_condition(body: &Value) -> R<Condition> {
    let m = without_timing(map(body, "the condition")?);
    let mut allowed: Vec<&str> = WHATS.to_vec();
    allowed.extend(["board", "bridge", "bus", "level", "count", "rate", "value", "msg"]);
    check_keys(&m, &allowed, "the condition")?;
    let whats: Vec<&str> = WHATS.iter().copied().filter(|k| get(&m, k).is_some()).collect();
    let w = match whats[..] {
        [w] => w,
        [] => return Err(format!("needs one of: {}", WHATS.join(", "))),
        _ => return Err(format!("has {}: one per step", whats.join(" and "))),
    };
    let v = get(&m, w).unwrap();
    let only_with = |k: &str, with: &str| -> R<()> {
        if get(&m, k).is_some() && w != with {
            return Err(format!("`{k}` goes with `{with}`"));
        }
        Ok(())
    };
    only_with("level", "log")?;
    only_with("value", "message")?;
    only_with("msg", "topic")?;
    let what = match w {
        "log" => {
            let text = match v {
                Value::Mapping(lm) => {
                    check_keys(lm, &["regex"], "log")?;
                    let re = string(get(lm, "regex").ok_or("log: a text, or {regex: PATTERN}")?, "log.regex")?;
                    TextMatch::Regex(regex::Regex::new(&re).map_err(|e| format!("log.regex: {e}"))?)
                }
                v => TextMatch::Substring(string(v, "log")?),
            };
            let level = get(&m, "level").map(|v| string(v, "level").map(|s| s.to_lowercase())).transpose()?;
            if let Some(l) = &level
                && !["trace", "debug", "info", "warn", "error"].contains(&l.as_str())
            {
                return Err(format!("level: {l:?}: trace, debug, info, warn or error"));
            }
            What::Log { text, level }
        }
        "status" => What::Status(parse_status(v)?),
        "link" => What::Link(match string(v, "link")?.as_str() {
            "up" | "true" => true,
            "down" | "false" => false,
            other => return Err(format!("link: {other:?}: up or down")),
        }),
        "frame" => What::Frame(parse_frame(v)?),
        "gpio" => {
            let gm = map(v, "gpio")?;
            check_keys(gm, &["pin", "level"], "gpio")?;
            let pin = match get(gm, "pin").ok_or("gpio: needs `pin` (a number or a name from the board file)")? {
                Value::Number(_) => PinRef::Number(integer(get(gm, "pin").unwrap(), "gpio.pin")?),
                other => {
                    let s = string(other, "gpio.pin")?;
                    match s.parse::<u8>() {
                        Ok(n) => PinRef::Number(n),
                        Err(_) => PinRef::Name(s),
                    }
                }
            };
            let high = match get(gm, "level").map(|v| string(v, "gpio.level")).transpose()?.as_deref() {
                Some("high") | Some("1") | Some("true") => true,
                Some("low") | Some("0") | Some("false") => false,
                other => return Err(format!("gpio.level: {other:?}: high or low")),
            };
            What::Gpio { pin, high }
        }
        "esc" => {
            let em = map(v, "esc")?;
            check_keys(em, &["name", "rpm"], "esc")?;
            What::Esc {
                name: get(em, "name").map(|v| string(v, "esc.name")).transpose()?,
                rpm: range(get(em, "rpm").ok_or("esc: needs `rpm` (a range)")?, "esc.rpm")?,
            }
        }
        "vehicle" => {
            let vm = map(v, "vehicle")?;
            let qs = ["depth", "north", "east", "speed", "roll", "pitch", "yaw"];
            let mut allowed = vec!["name"];
            allowed.extend(qs);
            check_keys(vm, &allowed, "vehicle")?;
            let mut checks = Vec::new();
            for (q, qty) in qs.iter().zip([
                VehicleQty::Depth,
                VehicleQty::North,
                VehicleQty::East,
                VehicleQty::Speed,
                VehicleQty::Roll,
                VehicleQty::Pitch,
                VehicleQty::Yaw,
            ]) {
                if let Some(r) = get(vm, q) {
                    checks.push((qty, range(r, &format!("vehicle.{q}"))?));
                }
            }
            if checks.is_empty() {
                return Err(format!("vehicle: check at least one of: {}", qs.join(", ")));
            }
            What::Vehicle { name: get(vm, "name").map(|v| string(v, "vehicle.name")).transpose()?, checks }
        }
        "resets" => What::Resets(range(v, "resets")?),
        "lockups" => What::Lockups(range(v, "lockups")?),
        "ivc" => What::Ivc(match v {
            Value::String(s) if s == "any" => None,
            v => Some(integer(v, "ivc")?),
        }),
        "message" => {
            let path = string(v, "message")?;
            let path: Vec<String> = path.split(['.', ':']).filter(|s| !s.is_empty()).map(str::to_string).collect();
            if path.is_empty() {
                return Err("message: a path like CameraCage.Depth".into());
            }
            let value = match get(&m, "value") {
                None => None,
                Some(v @ Value::Sequence(s)) if s.len() == 2 && s.iter().all(|x| x.is_number()) => {
                    Some(ValueCheck::Range(range(v, "value")?))
                }
                Some(Value::Mapping(vm)) if vm.keys().all(|k| matches!(k.as_str(), Some("min" | "max"))) => {
                    Some(ValueCheck::Range(range(get(&m, "value").unwrap(), "value")?))
                }
                Some(Value::Number(n)) => Some(ValueCheck::Range(Range::exactly(n.as_f64().unwrap_or(f64::NAN)))),
                Some(other) => {
                    Some(ValueCheck::Equals(serde_json::to_value(other).map_err(|e| format!("value: {e}"))?))
                }
            };
            What::Message { path, value }
        }
        "topic" => What::Topic {
            topic: string(v, "topic")?,
            msg: get(&m, "msg").map(|x| serde_json::to_value(x).map_err(|e| format!("msg: {e}"))).transpose()?,
        },
        _ => unreachable!(),
    };
    let target = parse_target(&m)?;
    let bus = get(&m, "bus").map(|v| string(v, "bus")).transpose()?;
    let count = get(&m, "count").map(|v| range(v, "count")).transpose()?;
    let rate = get(&m, "rate").map(|v| range(v, "rate")).transpose()?;
    Ok(Condition { target, bus, what, count, rate })
}

fn fault_refs(v: &Value, what: &str) -> R<Vec<FaultRef>> {
    let one = |x: &Value| -> R<FaultRef> {
        match x {
            Value::Number(_) => Ok(FaultRef::Bit(integer(x, what)?)),
            x => {
                let s = string(x, what)?;
                Ok(match s.parse::<u8>() {
                    Ok(b) => FaultRef::Bit(b),
                    Err(_) => FaultRef::Name(s),
                })
            }
        }
    };
    match v {
        Value::Sequence(l) => l.iter().map(one).collect(),
        x => Ok(vec![one(x)?]),
    }
}

fn parse_status(v: &Value) -> R<StatusCheck> {
    let m = match v {
        Value::Mapping(m) => m.clone(),
        Value::String(s) if s == "any" => Mapping::new(),
        _ => {
            return Err(format!(
                "status: expected a map of checks (faults, raised, clear, killed, ...) or `any`, found {}",
                show(v)
            ));
        }
    };
    check_keys(
        &m,
        &["faults", "raised", "clear", "killed", "kill_timed_out", "uptime", "board_name", "client_id", "max_age"],
        "status",
    )?;
    Ok(StatusCheck {
        faults: get(&m, "faults").map(|v| integer(v, "status.faults")).transpose()?,
        raised: get(&m, "raised").map(|v| fault_refs(v, "status.raised")).transpose()?.unwrap_or_default(),
        clear: get(&m, "clear").map(|v| fault_refs(v, "status.clear")).transpose()?.unwrap_or_default(),
        killed: get(&m, "killed").map(|v| boolean(v, "status.killed")).transpose()?,
        kill_timed_out: get(&m, "kill_timed_out").map(|v| boolean(v, "status.kill_timed_out")).transpose()?,
        uptime: get(&m, "uptime").map(|v| range(v, "status.uptime")).transpose()?,
        board_name: get(&m, "board_name").map(|v| string(v, "status.board_name")).transpose()?,
        client_id: get(&m, "client_id").map(|v| integer(v, "status.client_id")).transpose()?,
        max_age: get(&m, "max_age").map(|v| time(v, "status.max_age")).transpose()?,
    })
}

fn parse_frame(v: &Value) -> R<FrameMatch> {
    let m = match v {
        Value::Mapping(m) => m.clone(),
        Value::String(s) if s == "any" => Mapping::new(),
        Value::Number(_) | Value::String(_) => {
            let mut m = Mapping::new();
            m.insert("id".into(), v.clone());
            m
        }
        _ => return Err(format!("frame: expected {{id, mask, extended, data}}, an id, or `any`, found {}", show(v))),
    };
    check_keys(&m, &["id", "mask", "extended", "data"], "frame")?;
    let data = match get(&m, "data") {
        Some(d) => {
            let s = string(d, "frame.data")?.replace([' ', '_'], "");
            if s.len() % 2 != 0 {
                return Err(format!("frame.data: {s:?}: an even number of hex digits"));
            }
            (0..s.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| format!("frame.data: {s:?}: not hex")))
                .collect::<R<Vec<u8>>>()?
        }
        None => Vec::new(),
    };
    Ok(FrameMatch {
        id: get(&m, "id").map(|v| integer(v, "frame.id")).transpose()?,
        mask: get(&m, "mask").map(|v| integer(v, "frame.mask")).transpose()?,
        extended: get(&m, "extended").map(|v| boolean(v, "frame.extended")).transpose()?,
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> R<Vec<Scenario>> {
        parse_str(text, &crate::repo().join("scenarios/x.yaml"))
    }

    const HEAD: &str = "name: t\nrobot: robots/talos.toml\ntime: 15s\n";

    #[test]
    fn parses_the_example() {
        let s = parse(
            "name: ESC board recovers after its CAN cable is pulled
robot: robots/talos.toml
bridges: [internal:1]
inputs: {camera_cage.depth.pressure_pa: 101325}
time: 15s
steps:
  - at: 5s
    fault: {board: esc0, fault: can-unplug}
  - expect: {board: esc0, log: \"CAN offline\", within: 2s}
  - at: 8s
    fault: {board: esc0, fault: can-replug}
",
        );
        // can-replug is not a fault: the error names the step and lists where the faults are
        let e = s.unwrap_err();
        assert!(e.contains("step 3") && e.contains("can-replug") && e.contains("faults.md"), "{e}");
        let s = &parse(&format!(
            "{HEAD}bridges: [internal:1]
inputs: {{camera_cage.depth.pressure_pa: 101325}}
steps:
  - at: 5s
    fault: {{board: esc0, fault: can-unplug}}
  - expect: {{board: esc0, log: \"CAN offline\", within: 2s}}
  - expect: {{bridge: internal:1, status: {{faults: 0}}, at: 14s}}
  - never: {{board: esc0, log: {{regex: \"panick?ed\"}}}}
"
        ))
        .unwrap()[0];
        assert_eq!(s.time, Time::from_secs(15));
        assert_eq!(s.bridges, Bridges::List(vec![BridgeSpec::Can { bus: Some("internal".into()), client: 1 }]));
        assert_eq!(s.inputs, vec![("camera_cage.depth.pressure_pa".to_string(), 101325.0)]);
        assert_eq!(s.steps.len(), 4);
        assert_eq!(s.steps[0].start, Start::At(Time::from_secs(5)));
        assert!(
            matches!(&s.steps[0].kind, StepKind::Fault { board: Some(b), fault: Fault::CanUnplugged(true), .. } if b == "esc0")
        );
        assert_eq!(s.steps[1].start, Start::Cursor);
        assert_eq!(s.steps[1].end, Some(End::Within(Time::from_secs(2))));
        assert!(
            matches!(&s.steps[2].kind, StepKind::Expect(c) if matches!(&c.what, What::Status(st) if st.faults == Some(0)))
        );
        assert_eq!(s.steps[2].start, Start::At(Time::from_secs(14)));
        let StepKind::Never(c) = &s.steps[3].kind else { panic!() };
        let What::Log { text, .. } = &c.what else { panic!() };
        assert!(text.matches("it panicked") && text.matches("it paniced") && !text.matches("fine"));
        let Setup::Robot { file, boards: None } = &s.setup else { panic!() };
        assert!(file.ends_with("robots/talos.toml"));
    }

    #[test]
    fn timing_keys_go_inside_or_next_to_the_kind() {
        let s = &parse(&format!(
            "{HEAD}steps:
  - fault: esc0:can-unplug
    after: 1.5s
  - expect: {{board: esc0, log: x}}
    within: 250ms
  - never: {{board: esc0, log: y, at: 1s, until: 2s}}
"
        ))
        .unwrap()[0];
        assert_eq!(s.steps[0].start, Start::After(Time::from_ms(1500)));
        assert_eq!(s.steps[1].end, Some(End::Within(Time::from_ms(250))));
        assert_eq!(
            (s.steps[2].start, s.steps[2].end),
            (Start::At(Time::from_secs(1)), Some(End::Until(Time::from_secs(2))))
        );
        let e = parse(&format!("{HEAD}steps:\n  - at: 1s\n    expect: {{board: a, log: x, at: 2s}}\n")).unwrap_err();
        assert!(e.contains("`at` is given twice"), "{e}");
        let e = parse(&format!("{HEAD}steps:\n  - fault: can-stuck\n    within: 1s\n")).unwrap_err();
        assert!(e.contains("actions take"), "{e}");
        let e = parse(&format!("{HEAD}steps:\n  - expect: {{log: x, at: 20s}}\n")).unwrap_err();
        assert!(e.contains("after the end of the run"), "{e}");
    }

    #[test]
    fn faults_with_and_without_a_board() {
        let s = &parse(&format!(
            "{HEAD}steps:\n  - fault: can-flood:0x10:0.5\n  - fault: camera_cage:i2c-absent:0x76\n  - fault: {{fault: brownout:50ms}}\n"
        ))
        .unwrap()[0];
        let f: Vec<(Option<String>, Fault)> = s
            .steps
            .iter()
            .map(|st| match &st.kind {
                StepKind::Fault { board, fault, .. } => (board.clone(), fault.clone()),
                _ => panic!(),
            })
            .collect();
        assert_eq!(f[0], (None, Fault::CanFlood { id: 0x10, extended: false, len: 8, load: 0.5 }));
        assert_eq!(f[1].0.as_deref(), Some("camera_cage"));
        assert_eq!(f[2], (None, Fault::Brownout { off: Time::from_ms(50) }));
    }

    #[test]
    fn conditions() {
        let s = &parse(&format!(
            "{HEAD}steps:
  - expect: {{board: esc0, status: {{raised: [FAULT_DEPTH_INIT_ERROR, 1], clear: WATCHDOG_RESET, killed: false, uptime: {{min: 5}}}}, within: 2s}}
  - expect: {{bus: internal, frame: {{id: 0x1020800, mask: 0x1ffff00, data: \"01 00\"}}, rate: [9, 11], within: 1s}}
  - always: {{board: esc0, gpio: {{pin: status_ledg, level: high}}, until: 3s}}
  - always: {{board: esc1, esc: {{name: esc1, rpm: [700, 900]}}, within: 1s}}
  - expect: {{vehicle: {{depth: {{min: 0.3}}, yaw: [-90, -40]}}}}
  - expect: {{board: battery, resets: 1}}
  - expect: {{board: surface_ivc, ivc: 0x5a, within: 10s}}
  - expect: {{board: camera_cage, message: CameraCage.Depth, value: [-0.01, 0.01], count: {{min: 3}}, within: 1s}}
  - never: {{board: x, message: Heartbeat}}
"
        ))
        .unwrap()[0];
        let w: Vec<&What> = s
            .steps
            .iter()
            .map(|st| match &st.kind {
                StepKind::Expect(c) | StepKind::Never(c) | StepKind::Always(c) => &c.what,
                _ => panic!(),
            })
            .collect();
        let What::Status(st) = w[0] else { panic!() };
        assert_eq!(st.raised, vec![FaultRef::Name("FAULT_DEPTH_INIT_ERROR".into()), FaultRef::Bit(1)]);
        assert_eq!(st.clear, vec![FaultRef::Name("WATCHDOG_RESET".into())]);
        assert_eq!((st.killed, st.uptime), (Some(false), Some(Range { min: Some(5.0), max: None })));
        let What::Frame(f) = w[1] else { panic!() };
        assert_eq!((f.id, f.mask, &f.data[..]), (Some(0x1020800), Some(0x1ffff00), &[1u8, 0][..]));
        assert!(matches!(w[2], What::Gpio { pin: PinRef::Name(n), high: true } if n == "status_ledg"));
        assert!(matches!(w[3], What::Esc { rpm, .. } if rpm.contains(800.0) && !rpm.contains(950.0)));
        assert!(matches!(w[4], What::Vehicle { checks, .. } if checks.len() == 2));
        assert!(matches!(w[5], What::Resets(r) if *r == Range::exactly(1.0)));
        assert!(matches!(w[6], What::Ivc(Some(0x5a))));
        assert!(
            matches!(w[7], What::Message { path, value: Some(ValueCheck::Range(_)) } if path == &["CameraCage", "Depth"])
        );
        let StepKind::Expect(c) = &s.steps[7].kind else { panic!() };
        assert_eq!(c.count, Some(Range { min: Some(3.0), max: None }));
    }

    #[test]
    fn mistakes_are_named() {
        let err = |steps: &str| parse(&format!("{HEAD}steps:\n{steps}")).unwrap_err();
        assert!(err("  - expect: {board: a, log: x, within: 1s, withn: 2s}\n").contains("unknown key `withn`"));
        assert!(err("  - expect: {board: a, log: x, gpio: {pin: 1, level: high}}\n").contains("one per step"));
        assert!(err("  - always: {board: a, log: x}\n").contains("use `never`"));
        assert!(err("  - never: {board: a, log: x, count: 1}\n").contains("go with `expect`"));
        assert!(err("  - expect: {board: a, resets: 1, count: 1}\n").contains("count"));
        assert!(err("  - expect: {board: a, log: x, level: loud}\n").contains("level"));
        assert!(err("  - send: {board: a, message: {Nope: 1}}\n").contains("HostToDevice"));
        assert!(
            err("  - send: {board: a, message: {KillSwitch: {asserting_kill: true}}, every: 100ms}\n")
                .contains("every")
        );
        assert!(err("  - expect: {board: a, frame: {id: 1, data: 123}}\n").contains("even number"));
        assert!(err("  - teleport: x\n").contains("unknown key `teleport`"));
        let e = parse("name: t\ntime: 1s\n").unwrap_err();
        assert!(e.contains("`robot`"), "{e}");
        let e = parse("name: t\nrobot: robots/nope.toml\ntime: 1s\n").unwrap_err();
        assert!(e.contains("not found"), "{e}");
    }

    #[test]
    fn messages_parse_into_titan_link() {
        let s = &parse(&format!(
            "{HEAD}steps:
  - send: {{board: esc0, message: {{Esc: {{ThrusterRpm: [800, -800, 800, -800, 0, 0, 0, 0]}}}}, every: 100ms, until: 7s}}
  - send: {{bridge: internal:1, message: {{KillSwitch: {{asserting_kill: true}}}}}}
  - ivc_send: talos_ivc:0x5a
  - ivc_send: {{board: surface_ivc, byte: 195, at: 13s}}
"
        ))
        .unwrap()[0];
        let StepKind::Send { message, every, .. } = &s.steps[0].kind else { panic!() };
        assert_eq!(
            **message,
            HostToDevice::Esc(titan_link::esc::HostToDevice::ThrusterRpm([800, -800, 800, -800, 0, 0, 0, 0]))
        );
        assert_eq!(*every, Some(Time::from_ms(100)));
        assert!(
            matches!(&s.steps[1].kind, StepKind::Send { message, .. } if **message == HostToDevice::KillSwitch { asserting_kill: true })
        );
        assert!(matches!(&s.steps[2].kind, StepKind::IvcSend { board, byte: 0x5a } if board == "talos_ivc"));
        assert!(matches!(&s.steps[3].kind, StepKind::IvcSend { byte: 0xc3, .. }));
    }

    #[test]
    fn several_documents_boards_and_cases() {
        let all = parse(
            "name: one
board: {image: power_board, file: mk2/power_board}
bridges: [1]
time: 2s
---
name: two
robot: {file: talos, boards: [poac, esc0]}
bridges: all
time: 3s
steps:
  - expect: {board: poac, log: ROS connected, within: 1s}
cases:
  - name: unplugged
    steps:
      - fault: poac:can-unplug
  - name: stuck
    steps:
      - fault: can-stuck
",
        )
        .unwrap();
        assert_eq!(all.len(), 2);
        let Setup::Board { name, image, file: Some(f), rom_boot: false } = &all[0].setup else { panic!() };
        assert_eq!(name, "power_board");
        assert!(image.ends_with("power_board") && f.ends_with("boards/mk2/power_board.toml"));
        assert_eq!(all[0].bridges, Bridges::List(vec![BridgeSpec::Can { bus: None, client: 1 }]));
        assert_eq!(all[1].document, 1);
        assert_eq!(all[1].bridges, Bridges::All);
        assert!(matches!(&all[1].setup, Setup::Robot { boards: Some(b), .. } if b == &["poac", "esc0"]));
        assert_eq!(all[1].cases.len(), 2);
        // Steps are numbered on from the common ones
        assert_eq!(all[1].cases[1].steps[0].number, 2);
    }

    #[test]
    fn times() {
        assert_eq!(parse_time("1.5s"), Ok(Time::from_ms(1500)));
        assert_eq!(parse_time("200ms"), Ok(Time::from_ms(200)));
        assert_eq!(parse_time("7"), Ok(Time::from_secs(7)));
        assert_eq!(parse_time("0.3"), Ok(Time::from_ms(300)));
        assert_eq!(parse_time("50us"), Ok(Time::from_us(50)));
        assert!(parse_time("-1s").is_err() && parse_time("soon").is_err());
        assert!(Range { min: Some(1.0), max: None }.contains(1.0));
        assert!(!Range { min: None, max: Some(1.0) }.contains(1.5));
    }
}
