//! A live dashboard: a small HTTP server (std::net, one thread per connection) serving a self-contained page and a
//! JSON view of the running boards, and accepting faults and input changes.
//!
//! The run loop owns the boards; the server never touches them. The loop calls [`Dashboard::service_session`] or
//! [`Dashboard::service_robot`] between slices: commands posted to the server are applied there (and answered with
//! the outcome), and every [`Dashboard::publish_every`] of wall time a fresh state is built for the server to hand
//! out. Building it reads only what is already there (registers, pin levels, device models, the mock bridges'
//! records), so a dashboard costs nothing per instruction.
//!
//! Endpoints:
//!
//! | Request | Answer |
//! |---|---|
//! | `GET /` | the page |
//! | `GET /api/state` | the state (JSON, see [`Dashboard::session_state`]) |
//! | `POST /api/fault` `{"board": "esc0", "fault": "can-stuck"}` | applies a fault (`sim_host::fault::Fault` syntax) |
//! | `POST /api/input` `{"board": "esc0", "name": "esc_vcc", "value": 12.0}` | sets a physical input |
//! | `POST /api/pause` `{"paused": true}` | pauses or resumes virtual time |

use std::collections::{BTreeMap, VecDeque};
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_devices::can::BusRecord;

use crate::Session;
use crate::board::Inputs;
use crate::ethernet::PuddlesBridge;
use crate::fault::{BridgeControl, Fault};
use crate::mock_bridge::MockBridge;
use crate::robot::{Robot, RobotBus};

const PAGE: &str = include_str!("dashboard.html");
/// Log lines kept per board.
const LOG_LINES: usize = 300;
/// Log lines per board in each state.
const LOG_LINES_SENT: usize = 120;
/// Recent frames kept per bus.
const RECENT_FRAMES: usize = 40;
/// How long a POST waits for the run loop to apply it.
const REPLY_TIMEOUT: Duration = Duration::from_secs(10);

/// The faults the page offers, with what they do.
pub const FAULT_HELP: [(&str, &str); 21] = [
    ("can-stuck", "hold the CAN bus dominant"),
    ("can-release", "release a stuck bus"),
    ("can-unplug", "unplug the board from its bus"),
    ("can-plug", "plug it back in"),
    ("can-error-rate:0.3", "bit errors on 30% of frames (0 ends it)"),
    ("can-corrupt:5", "bit errors on the next 5 frames"),
    ("can-flood:0x100:0.5", "a node flooding ID 0x100 at 50% load"),
    ("can-flood-stop", "stop the floods"),
    ("bridge-stop", "titan_bridge stops"),
    ("bridge-start", "titan_bridge starts again"),
    ("kill:on", "assert the kill switch"),
    ("kill:off", "enable the vehicle"),
    ("kill:none", "stop forwarding the kill switch"),
    ("i2c-absent:0x76", "an I2C device stops answering (i2c-ok:ADDR repairs)"),
    ("i2c-stuck:0x76", "an I2C device freezes"),
    ("adc-short:0:0", "ADC input 0 shorted to 0 V (adc-ok:IN repairs)"),
    ("gpio:1:low", "drive GPIO 1 low from outside (gpio:N:release)"),
    ("brownout:50ms", "the supply drops out for 50 ms"),
    ("hang:0", "core 0 hangs (hang-masked:0 with interrupts off)"),
    ("panic", "core 0 panics"),
    ("watchdog", "the watchdog fires"),
];

/// The standard titan fault bits (`titan_rp2040::safety`); boards add their own from bit 5.
const FAULT_NAMES: [&str; 5] = [
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",
];

/// A stand-in for `titan_bridge` the dashboard reads firmware status from and stops, starts and uses for the kill
/// switch.
#[derive(Clone)]
pub enum Link {
    Can(MockBridge),
    Eth(PuddlesBridge),
}

impl Link {
    pub fn control(&self) -> &dyn BridgeControl {
        match self {
            Link::Can(b) => b,
            Link::Eth(b) => b,
        }
    }

    pub fn client(&self) -> Option<u8> {
        match self {
            Link::Can(b) => Some(b.state().client_id),
            Link::Eth(_) => None,
        }
    }

    fn json(&self, now: Time, names: &[String]) -> Value {
        let (running, kill, pings, heartbeats, last, errors, status) = match self {
            Link::Can(b) => {
                let s = b.state();
                let status = s.received.iter().rev().find_map(|(t, m)| match m {
                    titan_link::DeviceToHost::FirmwareStatus(st) => Some((*t, st.clone())),
                    _ => None,
                });
                let hb = s
                    .received
                    .iter()
                    .filter(|(_, m)| matches!(m, titan_link::DeviceToHost::Heartbeat))
                    .map(|(t, _)| *t)
                    .collect::<Vec<_>>();
                (s.running, s.kill, s.pings, hb.len(), hb.last().copied(), s.errors.len(), status)
            }
            Link::Eth(b) => {
                let s = b.state();
                let status = s.received.iter().rev().find_map(|(t, m)| match m {
                    titan_link::DeviceToHost::FirmwareStatus(st) => Some((*t, st.clone())),
                    _ => None,
                });
                let hb = s
                    .received
                    .iter()
                    .filter(|(_, m)| matches!(m, titan_link::DeviceToHost::Heartbeat))
                    .map(|(t, _)| *t)
                    .collect::<Vec<_>>();
                (s.running, s.kill, s.pings, hb.len(), hb.last().copied(), s.errors.len(), status)
            }
        };
        let status = status.map(|(t, st)| {
            let bits = |v: u32| -> Vec<String> {
                (0..32).filter(|b| v & (1 << b) != 0).map(|b| fault_name(names, b as usize)).collect()
            };
            json!({
                "age_s": (now.saturating_sub(t)).as_secs_f64(),
                "board_name": st.board_name.as_str(),
                "bus_id": st.bus_id,
                "client_id": st.client_id,
                "version": format!("{}.{} ({})", st.version_major, st.version_minor, release_type(st.version_release_type)),
                "uptime_s": st.uptime_ms as f64 / 1000.0,
                "fault_bits": st.faults,
                "faults": bits(st.faults),
                "kill_switches": {
                    "enabled": st.kill_switches_enabled,
                    "asserting_kill": st.kill_switches_asserting_kill,
                    "needs_update": st.kill_switches_needs_update,
                    "timed_out": st.kill_switches_timed_out,
                },
            })
        });
        json!({
            "kind": match self { Link::Can(_) => "can", Link::Eth(_) => "ethernet" },
            "client": self.client(),
            "running": running,
            "kill": kill,
            "pings": pings,
            "heartbeats": heartbeats,
            "last_heartbeat_age_s": last.map(|t| now.saturating_sub(t).as_secs_f64()),
            "decode_errors": errors,
            "status": status,
        })
    }
}

fn fault_name(names: &[String], bit: usize) -> String {
    match names.get(bit) {
        Some(n) => n.clone(),
        None => FAULT_NAMES.get(bit).map_or_else(|| format!("fault {bit}"), |n| n.to_string()),
    }
}

/// The board's fault names, from the `FAULT_NAMES: [&str; N]` table titan_firmware_rs boards keep in flash (empty
/// if the image has none).
pub fn fault_names(image: &sim_core::image::Image) -> Vec<String> {
    let word = |a: u32| -> Option<u32> {
        image.segments.iter().find_map(|s| {
            let o = a.checked_sub(s.addr)? as usize;
            Some(u32::from_le_bytes(s.data.get(o..o + 4)?.try_into().ok()?))
        })
    };
    let bytes = |a: u32, n: u32| -> Option<Vec<u8>> {
        image.segments.iter().find_map(|s| {
            let o = a.checked_sub(s.addr)? as usize;
            s.data.get(o..o + n as usize).map(<[u8]>::to_vec)
        })
    };
    let Some(table) = image.symbols.iter().find(|s| {
        !s.is_function
            && (0x1000_0000..0x1100_0000).contains(&s.addr)
            && crate::demangle::demangle(&s.name).ends_with("::FAULT_NAMES")
    }) else {
        return Vec::new();
    };
    (0..table.size / 8)
        .map_while(|k| {
            let (ptr, len) = (word(table.addr + 8 * k)?, word(table.addr + 8 * k + 4)?);
            (len < 64).then_some(())?;
            String::from_utf8(bytes(ptr, len)?).ok()
        })
        .collect()
}

fn release_type(t: u8) -> &'static str {
    match t {
        0 => "debug",
        1 => "dev",
        2 => "clean",
        3 => "tagged",
        _ => "?",
    }
}

/// What the dashboard knows about a board besides its session.
#[derive(Clone, Default)]
pub struct BoardInfo {
    pub name: String,
    /// The firmware image's file name.
    pub image: String,
    pub inputs: Inputs,
    /// The board file's `[pins]` table.
    pub pins: BTreeMap<String, u8>,
    /// Names of the firmware's fault bits (filled from the image by [`Dashboard::tap_session`] if left empty).
    pub fault_names: Vec<String>,
}

#[derive(Default)]
struct Feed {
    next_seq: u64,
    lines: VecDeque<(u64, Time, String, String)>,
}

impl Feed {
    fn push(&mut self, t: Time, kind: &str, text: String) {
        self.next_seq += 1;
        self.lines.push_back((self.next_seq, t, kind.to_string(), text));
        if self.lines.len() > LOG_LINES {
            self.lines.pop_front();
        }
    }
}

#[derive(Default)]
struct BusFeed {
    frames: u64,
    errors: u64,
    unacked: u64,
    recent: VecDeque<(Time, String)>,
    /// (start, end) of the frames in the last second.
    window: VecDeque<(Time, Time)>,
}

impl BusFeed {
    fn record(&mut self, r: &BusRecord) {
        self.frames += 1;
        if r.error.is_some() {
            self.errors += 1;
        } else if !r.acked {
            self.unacked += 1;
        }
        let mut text = sim_devices::can::candump(&r.frame);
        if let Some(e) = &r.error {
            text.push_str(&format!("  (destroyed: {e:?})"));
        } else if !r.acked {
            text.push_str("  (no ACK)");
        }
        self.recent.push_back((r.end, text));
        if self.recent.len() > RECENT_FRAMES {
            self.recent.pop_front();
        }
        self.window.push_back((r.start, r.end));
    }

    fn json(&mut self, name: &str, now: Time) -> Value {
        let from = now.saturating_sub(Time::from_secs(1));
        while self.window.front().is_some_and(|w| w.1 < from) {
            self.window.pop_front();
        }
        let busy: u64 = self.window.iter().map(|(s, e)| (*e - (*s).max(from)).as_ps()).sum();
        let span = (now - from).as_ps().max(1);
        json!({
            "name": name,
            "frames": self.frames,
            "errors": self.errors,
            "unacked": self.unacked,
            "rate_fps": self.window.len() as f64 * 1e12 / span as f64,
            "load": busy as f64 / span as f64,
            "recent": self.recent.iter().rev().map(|(t, f)| json!({"t": t.as_secs_f64(), "frame": f})).collect::<Vec<_>>(),
        })
    }
}

enum Command {
    Fault { board: String, fault: Fault, spec: String, reply: mpsc::Sender<Result<String, String>> },
    Input { board: String, name: String, value: f64, reply: mpsc::Sender<Result<String, String>> },
    Pause { paused: bool, reply: mpsc::Sender<Result<String, String>> },
}

struct Shared {
    state: String,
}

/// The dashboard of a run.
pub struct Dashboard {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    commands: mpsc::Receiver<Command>,
    /// Where the server listens.
    pub addr: SocketAddr,
    boards: Vec<BoardInfo>,
    links: Vec<(Option<usize>, Option<usize>, Link)>,
    feeds: Vec<Arc<Mutex<Feed>>>,
    bus_feeds: Vec<Arc<Mutex<BusFeed>>>,
    bus_names: Vec<String>,
    faults: Vec<Value>,
    paused: bool,
    /// Wall time between published states.
    pub publish_every: Duration,
    last_publish: Option<Instant>,
    speed: (Instant, Time, f64),
    started: Instant,
    /// Shown on the page once the run is over.
    pub finished: Option<String>,
}

impl Dashboard {
    /// Starts the server on `addr` (e.g. `127.0.0.1:8080`; port 0 picks a free one) for these boards and buses.
    pub fn start(addr: &str, boards: Vec<BoardInfo>, bus_names: Vec<String>) -> std::io::Result<Dashboard> {
        let listener = TcpListener::bind(addr)?;
        let addr = listener.local_addr()?;
        let shared = Arc::new(Mutex::new(Shared { state: "{}".into() }));
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();
        {
            let shared = shared.clone();
            let stop = stop.clone();
            std::thread::Builder::new().name("dashboard".into()).spawn(move || {
                for stream in listener.incoming() {
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    let Ok(stream) = stream else { continue };
                    let shared = shared.clone();
                    let tx = tx.clone();
                    let _ = std::thread::Builder::new().name("dashboard-conn".into()).spawn(move || {
                        let _ = handle(stream, &shared, &tx);
                    });
                }
            })?;
        }
        let started = Instant::now();
        let feeds = boards.iter().map(|_| Arc::default()).collect();
        let bus_feeds = bus_names.iter().map(|_| Arc::default()).collect();
        Ok(Dashboard {
            shared,
            stop,
            commands: rx,
            addr,
            boards,
            links: Vec::new(),
            feeds,
            bus_feeds,
            bus_names,
            faults: Vec::new(),
            paused: false,
            publish_every: Duration::from_millis(200),
            last_publish: None,
            speed: (started, Time::ZERO, 0.0),
            started,
            finished: None,
        })
    }

    /// A mock bridge (or the Puddles one) attached to `board` or on bus `bus`.
    pub fn add_link(&mut self, board: Option<usize>, bus: Option<usize>, link: Link) {
        self.links.push((board, bus, link));
    }

    /// Whether the page has paused virtual time.
    pub fn paused(&self) -> bool {
        self.paused
    }

    /// Collects board `i`'s log and UART lines (keeping whatever the session already does with them) and, if it
    /// has its own bus (a single board), that bus's frames as bus 0.
    pub fn tap_session(&mut self, i: usize, s: &mut Session) {
        if self.boards[i].fault_names.is_empty()
            && let Some(image) = &s.machine.image
        {
            self.boards[i].fault_names = fault_names(image);
        }
        let feed = self.feeds[i].clone();
        let mut prev = s.on_log.take();
        s.on_log = Some(Box::new(move |t, line| {
            if let Some(p) = &mut prev {
                p(t, line);
            }
            let level = line.level.clone().unwrap_or_default().to_lowercase();
            feed.lock().unwrap().push(t, &level, line.to_string());
        }));
        let feed = self.feeds[i].clone();
        let mut prev = s.on_uart_line.take();
        s.on_uart_line = Some(Box::new(move |t, n, line| {
            if let Some(p) = &mut prev {
                p(t, n, line);
            }
            feed.lock().unwrap().push(t, &format!("uart{n}"), line.to_string());
        }));
        if let (Some(bus), Some(feed)) = (&mut s.can_bus, self.bus_feeds.first()) {
            bus.record.get_or_insert_with(Vec::new);
            let feed = feed.clone();
            let mut prev = s.on_can.take();
            s.on_can = Some(Box::new(move |r| {
                if let Some(p) = &mut prev {
                    p(r);
                }
                feed.lock().unwrap().record(r);
            }));
        }
    }

    /// Collects the frames of robot bus `j`.
    pub fn tap_bus(&self, j: usize, rb: &mut RobotBus) {
        rb.bus.record.get_or_insert_with(Vec::new);
        let feed = self.bus_feeds[j].clone();
        let mut prev = rb.on_can.take();
        rb.on_can = Some(Box::new(move |r| {
            if let Some(p) = &mut prev {
                p(r);
            }
            feed.lock().unwrap().record(r);
        }));
    }

    /// Notes a fault applied by someone else (the command line's `--fault`), for the page's history.
    pub fn note_fault(&mut self, t: Time, board: &str, spec: &str, result: &Result<(), String>) {
        self.faults.push(json!({
            "t": t.as_secs_f64(),
            "board": board,
            "fault": spec,
            "ok": result.is_ok(),
            "error": result.as_ref().err(),
            "from": "command line",
        }));
    }

    fn board_index(&self, name: &str) -> Result<usize, String> {
        if name.is_empty() && self.boards.len() == 1 {
            return Ok(0);
        }
        self.boards.iter().position(|b| b.name == name).ok_or_else(|| format!("no board {name:?}"))
    }

    fn set_input(&self, board: &str, name: &str, value: f64) -> Result<String, String> {
        let i = self.board_index(board)?;
        let inputs = &self.boards[i].inputs;
        if !inputs.0.contains_key(name) {
            let names: Vec<&str> = inputs.0.keys().map(String::as_str).collect();
            return Err(format!("{} has no input {name:?}; it has: {}", self.boards[i].name, names.join(", ")));
        }
        if !value.is_finite() {
            return Err("the value must be a number".into());
        }
        inputs.set(name, value);
        Ok(format!("{}: {name} = {value}", self.boards[i].name))
    }

    /// Between slices of a single board's run: applies what was posted and publishes the state when due.
    pub fn service_session(&mut self, s: &mut Session) {
        while let Ok(cmd) = self.commands.try_recv() {
            match cmd {
                Command::Fault { board, fault, spec, reply } => {
                    let r = self.board_index(&board).and_then(|_| {
                        let ctl = self.links.iter().find(|l| l.0 == Some(0) || l.0.is_none()).map(|l| l.2.control());
                        crate::fault::apply_to_session(s, ctl, &fault)
                    });
                    let name = if board.is_empty() {
                        self.boards.first().map_or(String::new(), |b| b.name.clone())
                    } else {
                        board.clone()
                    };
                    self.record_fault(s.machine.now, &name, &spec, &r);
                    let _ = reply.send(r.map(|_| format!("{spec} applied to {name}")));
                }
                Command::Input { board, name, value, reply } => {
                    let r = self.set_input(&board, &name, value);
                    let _ = reply.send(r);
                }
                Command::Pause { paused, reply } => {
                    self.paused = paused;
                    if !paused {
                        s.reset_pacing();
                    }
                    let _ = reply.send(Ok(if paused { "paused" } else { "running" }.into()));
                }
            }
            self.last_publish = None;
        }
        if self.due() {
            let state = self.session_state(s);
            self.shared.lock().unwrap().state = state.to_string();
        }
    }

    /// Between slices of a robot's run: applies what was posted and publishes the state when due. `bridges` are the
    /// mock bridges fault injection uses per bus (the first on each, as for `--fault`).
    pub fn service_robot(&mut self, r: &mut Robot, bridges: &[Option<MockBridge>]) {
        while let Ok(cmd) = self.commands.try_recv() {
            match cmd {
                Command::Fault { board, fault, spec, reply } => {
                    let res = self.board_index(&board).and_then(|i| {
                        let eth = self.links.iter().find(|l| l.0 == Some(i)).map(|l| l.2.clone());
                        let can = r.board_bus[i].and_then(|j| bridges.get(j).cloned().flatten());
                        let ctl: Option<&dyn BridgeControl> = match (&eth, &can) {
                            (Some(l), _) => Some(l.control()),
                            (None, Some(b)) => Some(b),
                            _ => None,
                        };
                        crate::fault::robot_target(r, i, ctl).apply(&fault)
                    });
                    self.record_fault(r.now, &board, &spec, &res);
                    let _ = reply.send(res.map(|_| format!("{spec} applied to {board}")));
                }
                Command::Input { board, name, value, reply } => {
                    let _ = reply.send(self.set_input(&board, &name, value));
                }
                Command::Pause { paused, reply } => {
                    self.paused = paused;
                    if !paused {
                        r.reset_pacing();
                    }
                    let _ = reply.send(Ok(if paused { "paused" } else { "running" }.into()));
                }
            }
            self.last_publish = None;
        }
        if self.due() {
            let state = self.robot_state(r);
            self.shared.lock().unwrap().state = state.to_string();
        }
    }

    /// Publishes the final state (the run is over, or stopped with `why`).
    pub fn finish_session(&mut self, s: &mut Session, why: &str) {
        self.finished = Some(why.to_string());
        let state = self.session_state(s);
        self.shared.lock().unwrap().state = state.to_string();
    }

    /// Publishes the final state of a robot.
    pub fn finish_robot(&mut self, r: &mut Robot, why: &str) {
        self.finished = Some(why.to_string());
        let state = self.robot_state(r);
        self.shared.lock().unwrap().state = state.to_string();
    }

    /// Keeps serving the final state (until the process is stopped), answering commands that the run is over.
    pub fn hold(&mut self) -> ! {
        loop {
            match self.commands.recv() {
                Ok(Command::Fault { reply, .. })
                | Ok(Command::Input { reply, .. })
                | Ok(Command::Pause { reply, .. }) => {
                    let _ = reply.send(Err("the run is over".into()));
                }
                Err(_) => std::thread::sleep(Duration::from_secs(3600)),
            }
        }
    }

    fn record_fault(&mut self, t: Time, board: &str, spec: &str, r: &Result<(), String>) {
        self.faults.push(json!({
            "t": t.as_secs_f64(),
            "board": board,
            "fault": spec,
            "ok": r.is_ok(),
            "error": r.as_ref().err(),
            "from": "dashboard",
        }));
    }

    fn due(&mut self) -> bool {
        let now = Instant::now();
        if self.last_publish.is_some_and(|t| now.duration_since(t) < self.publish_every) {
            return false;
        }
        self.last_publish = Some(now);
        true
    }

    fn header(&mut self, now: Time, realtime: bool) -> Value {
        // Virtual seconds per wall second, over the last half second or more
        let wall = Instant::now();
        let (w0, v0, _) = self.speed;
        let dt = wall.duration_since(w0).as_secs_f64();
        let rate = if self.paused || dt <= 0.0 { 0.0 } else { now.saturating_sub(v0).as_secs_f64() / dt };
        if dt >= 0.5 {
            self.speed = (wall, now, rate);
        } else if w0 == self.started {
            self.speed.2 = rate;
        }
        json!({
            "time_s": now.as_secs_f64(),
            "wall_s": self.started.elapsed().as_secs_f64(),
            "speed": self.speed.2,
            "paused": self.paused,
            "realtime": realtime,
            "finished": self.finished,
            "faults": self.faults,
            "fault_help": FAULT_HELP.iter().map(|(f, h)| json!({"fault": f, "help": h})).collect::<Vec<_>>(),
        })
    }

    /// The state of a single board's run.
    pub fn session_state(&mut self, s: &mut Session) -> Value {
        let mut v = self.header(s.machine.now, s.realtime);
        let board = self.board_json(0, s, true, s.machine.now);
        v["boards"] = json!([board]);
        let name = self.bus_names.first().cloned().unwrap_or_default();
        v["buses"] = match self.bus_feeds.first() {
            Some(f) if s.can_bus.is_some() => json!([f.lock().unwrap().json(&name, s.machine.now)]),
            _ => json!([]),
        };
        let names = &self.boards[0].fault_names;
        v["links"] = json!(self.links.iter().map(|l| l.2.json(s.machine.now, names)).collect::<Vec<_>>());
        v["vehicles"] = json!([]);
        v
    }

    /// The state of a robot's run.
    pub fn robot_state(&mut self, r: &mut Robot) -> Value {
        let now = r.now;
        let mut v = self.header(now, r.realtime);
        let boards: Vec<Value> = (0..r.boards.len())
            .map(|i| {
                let powered = r.power_on.get(i).is_none_or(|p| p.1);
                let mut b = self.board_json(i, &mut r.boards[i], powered, now);
                b["bus"] = json!(r.board_bus[i].map(|j| r.bus_names[j].clone()));
                b
            })
            .collect();
        v["boards"] = json!(boards);
        v["buses"] = json!(
            self.bus_feeds.iter().zip(&self.bus_names).map(|(f, n)| f.lock().unwrap().json(n, now)).collect::<Vec<_>>()
        );
        v["links"] = json!(
            self.links
                .iter()
                .map(|(b, j, l)| {
                    let names = b.map_or(&[][..], |b| &self.boards[b].fault_names[..]);
                    let mut x = l.json(now, names);
                    x["board"] = json!(b.map(|b| self.boards[b].name.clone()));
                    x["bus"] = json!(j.map(|j| self.bus_names[j].clone()));
                    x
                })
                .collect::<Vec<_>>()
        );
        v["vehicles"] = json!(
            r.vehicles
                .iter()
                .map(|vs| {
                    let s = &vs.vehicle.state;
                    let [roll, pitch, yaw] = s.attitude.euler().map(f64::to_degrees);
                    let rpms = vs.rpms(&r.boards, now);
                    json!({
                        "name": vs.vehicle.name,
                        "position_m": s.position,
                        "depth_m": s.position[2],
                        "rpy_deg": [roll, pitch, yaw],
                        "velocity_mps": s.velocity,
                        "speed_mps": sim_physics::math::norm(s.velocity),
                        "rate_dps": s.rate.map(f64::to_degrees),
                        "thrusters": vs.vehicle.thrusters.iter().zip(rpms).map(|(t, rpm)| json!({"name": t.name, "rpm": rpm})).collect::<Vec<_>>(),
                    })
                })
                .collect::<Vec<_>>()
        );
        v
    }

    fn board_json(&self, i: usize, s: &mut Session, powered: bool, now: Time) -> Value {
        let info = &self.boards[i];
        let m = &s.machine;
        let cores: Vec<Value> = (0..2)
            .map(|c| {
                let cpu = &m.cores[c];
                let state = if !powered || !m.core_enabled[c] {
                    "off"
                } else if cpu.is_locked_up() {
                    "locked up"
                } else if cpu.is_sleeping() {
                    "sleeping"
                } else {
                    "running"
                };
                let pc = cpu.reg(15);
                let function = m.symbolize(pc).map(|f| {
                    let (name, off) = f.rsplit_once('+').filter(|(_, o)| o.starts_with("0x")).unwrap_or((&f, ""));
                    let name = crate::demangle::demangle(name);
                    if off.is_empty() { name } else { format!("{name}+{off}") }
                });
                json!({"state": state, "pc": format!("{pc:#010x}"), "function": function, "exception": cpu.ipsr()})
            })
            .collect();
        let labels = crate::vcd::pin_labels(&info.pins);
        let funcs: Vec<&str> = (0..crate::vcd::PINS)
            .map(|p| match m.sys.gpio.funcsel(p) {
                0 => "XIP",
                1 => "SPI",
                2 => "UART",
                3 => "I2C",
                4 => "PWM",
                5 => "SIO",
                6 => "PIO0",
                7 => "PIO1",
                8 => "CLOCK",
                9 => "USB",
                _ => "",
            })
            .collect();
        let leds: Vec<Value> = info
            .pins
            .iter()
            .filter(|(n, _)| n.contains("led"))
            .map(|(n, &p)| {
                json!({"name": n.strip_suffix("_pin").unwrap_or(n), "pin": p, "on": m.sys.gpio.level & (1 << p) != 0})
            })
            .collect();
        let mut strips = Vec::new();
        let mut escs = Vec::new();
        for d in &m.sys.pin_devices {
            let any = d.as_any();
            if let Some(w) = any.downcast_ref::<sim_devices::ws2812::Ws2812>() {
                let colors: Vec<String> =
                    w.shown.iter().map(|c| format!("{:02x}{:02x}{:02x}", c.r, c.g, c.b)).collect();
                strips.push(json!({"name": w.name, "pin": w.pin, "colors": colors}));
            } else if let Some(e) = any.downcast_ref::<sim_devices::dshot::Esc>() {
                escs.push(json!({"name": e.name, "rpm": e.rpm_at(now), "pin": e.signal_pin}));
            }
        }
        let inputs: Vec<Value> =
            info.inputs.0.iter().map(|(n, h)| json!({"name": n, "value": *h.lock().unwrap()})).collect();
        let feed = self.feeds[i].lock().unwrap();
        let skip = feed.lines.len().saturating_sub(LOG_LINES_SENT);
        let logs: Vec<Value> = feed
            .lines
            .iter()
            .skip(skip)
            .map(|(seq, t, kind, text)| json!({"seq": seq, "t": t.as_secs_f64(), "kind": kind, "text": text}))
            .collect();
        let link = self.links.iter().find(|l| l.0 == Some(i)).map(|l| l.2.json(now, &info.fault_names));
        json!({
            "name": info.name,
            "image": info.image,
            "fault_names": info.fault_names,
            "powered": powered,
            "time_s": m.now.as_secs_f64(),
            "resets": m.resets,
            "cores": cores,
            "gpio": {
                "level": m.sys.gpio.level,
                "oe": m.sys.gpio.oe_to_pad,
                "labels": labels,
                "funcs": funcs,
            },
            "leds": leds,
            "strips": strips,
            "escs": escs,
            "inputs": inputs,
            "logs": logs,
            "link": link,
        })
    }

    /// The page's view of the state as last published (what `GET /api/state` returns).
    pub fn published(&self) -> String {
        self.shared.lock().unwrap().state.clone()
    }
}

impl Drop for Dashboard {
    fn drop(&mut self) {
        // Wake the accept loop so its thread ends
        self.stop.store(true, Ordering::Relaxed);
        let _ = TcpStream::connect_timeout(&self.addr, Duration::from_millis(200));
    }
}

/// One HTTP request.
struct Request {
    method: String,
    path: String,
    body: Vec<u8>,
}

fn read_request(stream: &mut TcpStream) -> std::io::Result<Option<Request>> {
    let mut buf = Vec::new();
    let mut chunk = [0u8; 4096];
    let head_end = loop {
        if let Some(i) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            break i;
        }
        if buf.len() > 16 * 1024 {
            return Ok(None);
        }
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            return Ok(None);
        }
        buf.extend_from_slice(&chunk[..n]);
    };
    let head = String::from_utf8_lossy(&buf[..head_end]).into_owned();
    let mut lines = head.split("\r\n");
    let mut first = lines.next().unwrap_or("").split_whitespace();
    let method = first.next().unwrap_or("").to_string();
    let path = first.next().unwrap_or("/").to_string();
    let mut len = 0usize;
    for l in lines {
        if let Some((k, v)) = l.split_once(':')
            && k.trim().eq_ignore_ascii_case("content-length")
        {
            len = v.trim().parse().unwrap_or(0);
        }
    }
    if len > 64 * 1024 {
        return Ok(None);
    }
    let mut body = buf[head_end + 4..].to_vec();
    while body.len() < len {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        body.extend_from_slice(&chunk[..n]);
    }
    body.truncate(len);
    Ok(Some(Request { method, path, body }))
}

fn respond(stream: &mut TcpStream, code: u16, content_type: &str, body: &[u8]) -> std::io::Result<()> {
    let reason = match code {
        200 => "OK",
        400 => "Bad Request",
        404 => "Not Found",
        405 => "Method Not Allowed",
        422 => "Unprocessable Entity",
        503 => "Service Unavailable",
        _ => "",
    };
    write!(
        stream,
        "HTTP/1.1 {code} {reason}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    stream.write_all(body)?;
    stream.flush()
}

fn json_reply(stream: &mut TcpStream, code: u16, v: Value) -> std::io::Result<()> {
    respond(stream, code, "application/json", v.to_string().as_bytes())
}

fn handle(mut stream: TcpStream, shared: &Mutex<Shared>, tx: &mpsc::Sender<Command>) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let Some(req) = read_request(&mut stream)? else { return Ok(()) };
    let path = req.path.split('?').next().unwrap_or("/");
    match (req.method.as_str(), path) {
        ("GET", "/") | ("GET", "/index.html") => respond(&mut stream, 200, "text/html; charset=utf-8", PAGE.as_bytes()),
        ("GET", "/api/state") => {
            let state = shared.lock().unwrap().state.clone();
            respond(&mut stream, 200, "application/json", state.as_bytes())
        }
        ("POST", "/api/fault") | ("POST", "/api/input") | ("POST", "/api/pause") => {
            let body: Value = match serde_json::from_slice(&req.body) {
                Ok(v) => v,
                Err(e) => return json_reply(&mut stream, 400, json!({"ok": false, "error": format!("bad JSON: {e}")})),
            };
            let board = body["board"].as_str().unwrap_or("").to_string();
            let (reply, answer) = mpsc::channel();
            let cmd = match path {
                "/api/fault" => {
                    let spec = body["fault"].as_str().unwrap_or("").trim().to_string();
                    match spec.parse::<Fault>() {
                        Ok(fault) => Command::Fault { board, fault, spec, reply },
                        Err(e) => return json_reply(&mut stream, 400, json!({"ok": false, "error": e})),
                    }
                }
                "/api/input" => {
                    let name = body["name"].as_str().unwrap_or("").to_string();
                    let Some(value) = body["value"].as_f64().or_else(|| body["value"].as_str()?.trim().parse().ok())
                    else {
                        return json_reply(&mut stream, 400, json!({"ok": false, "error": "value must be a number"}));
                    };
                    Command::Input { board, name, value, reply }
                }
                _ => Command::Pause { paused: body["paused"].as_bool().unwrap_or(true), reply },
            };
            if tx.send(cmd).is_err() {
                return json_reply(&mut stream, 503, json!({"ok": false, "error": "the run is over"}));
            }
            match answer.recv_timeout(REPLY_TIMEOUT) {
                Ok(Ok(msg)) => json_reply(&mut stream, 200, json!({"ok": true, "message": msg})),
                Ok(Err(e)) => json_reply(&mut stream, 422, json!({"ok": false, "error": e})),
                Err(_) => json_reply(&mut stream, 503, json!({"ok": false, "error": "the run loop did not answer"})),
            }
        }
        (_, "/") | (_, "/api/state") | (_, "/api/fault") | (_, "/api/input") | (_, "/api/pause") => {
            json_reply(&mut stream, 405, json!({"ok": false, "error": "method not allowed"}))
        }
        _ => json_reply(&mut stream, 404, json!({"ok": false, "error": "not found"})),
    }
}
