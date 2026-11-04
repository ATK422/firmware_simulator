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
//! | `GET /api/state` (`?fresh=1`: built now) | the state (JSON, see [`Dashboard::session_state`]) |
//! | `GET /api/logs?board=esc0&since=SEQ&max=N&tail=1` | log and UART lines after a cursor, read live |
//! | `GET /api/frames?bus=internal&since=SEQ&max=N&id=0x100&mask=0x7ff&tail=1` | CAN frames after a cursor, read live |
//! | `POST /api/fault` `{"board": "esc0", "fault": "can-stuck"}` | applies a fault (`sim_host::fault::Fault` syntax) |
//! | `POST /api/input` `{"board": "esc0", "name": "esc_vcc", "value": 12.0}` | sets a physical input |
//! | `POST /api/pause` `{"paused": true, "at_s": 2.0}` | pauses or resumes virtual time (`at_s`: pause when it gets there) |
//! | `GET /api/thrusters` | every DShot ESC's propeller speed and each vehicle's thrust, built live (see [`thrusters_json`]) |
//! | `POST /api/vehicle` `{"vehicle": "talos", "position_m": [n, e, d], "rpy_deg": [r, p, y]}` | puts a vehicle at a pose (an external one: its only motion), answered with `/api/thrusters` |
//! | `POST /api/rewind` `{"time_s": 1.25}`, `{"checkpoint": "booted"}` or `{"save": "booted"}` | goes back (or forward, within what was run) to a time or a saved checkpoint (or saves one), the whole run at once ([`crate::timetravel`]; needs [`Dashboard::travel`]) |
//!
//! `POST /api/vehicle` takes the attitude as `rpy_deg` or `quat_wxyz` (body to world), and optionally
//! `velocity_mps` (body frame) and `rate_dps`; `vehicle` may be left out when there is one. Frames are the
//! emulator's: world north-east-down, body forward-starboard-down ([`crate::vehicle::ExternalPose`]).

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
use crate::vehicle::ExternalPose;

const PAGE: &str = include_str!("dashboard.html");
/// Log lines kept per board (for `GET /api/logs`).
const LOG_LINES: usize = 2000;
/// Log lines per board in each state.
const LOG_LINES_SENT: usize = 120;
/// Frames kept per bus (for `GET /api/frames`).
const FRAMES_KEPT: usize = 2000;
/// Recent frames per bus in each state.
const RECENT_FRAMES: usize = 40;
/// Most lines or frames one `GET /api/logs` or `/api/frames` returns.
const MAX_READ: usize = 2000;
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

/// One log or UART line.
struct LogLine {
    seq: u64,
    t: Time,
    kind: String,
    text: String,
}

/// Every board's lines. Sequence numbers count across all boards, so one cursor reads them all in order.
#[derive(Default)]
struct Logs {
    next_seq: u64,
    boards: Vec<BoardLog>,
}

#[derive(Default)]
struct BoardLog {
    lines: VecDeque<LogLine>,
    /// The sequence number of the last line dropped to keep [`LOG_LINES`].
    dropped_to: u64,
}

impl Logs {
    fn push(&mut self, board: usize, t: Time, kind: &str, text: String) {
        self.next_seq += 1;
        let b = &mut self.boards[board];
        b.lines.push_back(LogLine { seq: self.next_seq, t, kind: kind.to_string(), text });
        if b.lines.len() > LOG_LINES {
            b.dropped_to = b.lines.pop_front().map_or(0, |l| l.seq);
        }
    }
}

#[derive(Default)]
struct BusFeed {
    frames: u64,
    errors: u64,
    unacked: u64,
    /// The last [`FRAMES_KEPT`] frames with their sequence numbers (1 for the bus's first frame).
    kept: VecDeque<(u64, BusRecord)>,
    /// (start, end) of the frames in the last second.
    window: VecDeque<(Time, Time)>,
}

/// How a frame shows in the page's list: candump notation, and what happened to it.
fn frame_text(r: &BusRecord) -> String {
    let mut text = sim_devices::can::candump(&r.frame);
    if let Some(e) = &r.error {
        text.push_str(&format!("  (destroyed: {e:?})"));
    } else if !r.acked {
        text.push_str("  (no ACK)");
    }
    text
}

impl BusFeed {
    fn record(&mut self, r: &BusRecord) {
        self.frames += 1;
        if r.error.is_some() {
            self.errors += 1;
        } else if !r.acked {
            self.unacked += 1;
        }
        self.kept.push_back((self.frames, r.clone()));
        if self.kept.len() > FRAMES_KEPT {
            self.kept.pop_front();
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
            "recent": self.kept.iter().rev().take(RECENT_FRAMES).map(|(_, r)| json!({"t": r.end.as_secs_f64(), "frame": frame_text(r)})).collect::<Vec<_>>(),
        })
    }
}

enum Command {
    Fault {
        board: String,
        fault: Fault,
        spec: String,
        reply: mpsc::Sender<Result<String, String>>,
    },
    Input {
        board: String,
        name: String,
        value: f64,
        reply: mpsc::Sender<Result<String, String>>,
    },
    Pause {
        paused: bool,
        at: Option<Time>,
        reply: mpsc::Sender<Result<String, String>>,
    },
    /// Publish a state now (`GET /api/state?fresh=1`).
    Refresh {
        reply: mpsc::Sender<Result<String, String>>,
    },
    /// `GET /api/thrusters`, built by the run loop.
    Thrusters {
        reply: mpsc::Sender<Result<Value, String>>,
    },
    /// `POST /api/vehicle`.
    Vehicle {
        name: String,
        pose: ExternalPose,
        reply: mpsc::Sender<Result<Value, String>>,
    },
    /// `POST /api/rewind`.
    Rewind {
        to: RewindTo,
        reply: mpsc::Sender<Result<String, String>>,
    },
    /// `POST /api/agent`: a board's mock agent sends a message on a topic.
    AgentSend {
        board: String,
        topic: String,
        msg: Value,
        reply: mpsc::Sender<Result<String, String>>,
    },
}

/// Where `POST /api/rewind` goes.
#[derive(Clone, Debug, PartialEq)]
pub enum RewindTo {
    Time(Time),
    Checkpoint(String),
    /// Not a move: keeps a checkpoint here under this name.
    Save(String),
}

/// Goes back (or forward) with `tt` on `target`: the message for the answer.
pub fn rewind(
    tt: Option<&mut crate::timetravel::TimeTravel>,
    target: &mut dyn crate::timetravel::Rewindable,
    to: &RewindTo,
) -> Result<String, String> {
    use crate::timetravel::Travelled;
    let tt = tt.ok_or("time travel is off for this run (it records with --checkpoint-every above 0)")?;
    let r = match to {
        RewindTo::Time(t) => tt.rewind_to(target, *t),
        RewindTo::Save(name) => {
            tt.save(target, name)?;
            return Ok(format!("checkpoint {name:?} saved at {}", target.now()));
        }
        RewindTo::Checkpoint(name) => tt.rewind_to_saved(target, name).ok_or_else(|| {
            format!(
                "no checkpoint {name:?}; saved: {:?}",
                tt.checkpoints().iter().filter_map(|c| c.1).collect::<Vec<_>>()
            )
        })?,
    };
    let now = target.now();
    Ok(match r {
        Travelled::Beginning => format!("at {now}, the start of the recorded history"),
        Travelled::Frontier => format!("at {now}, as far as the run has got"),
        _ => format!("at {now}"),
    })
}

struct Shared {
    state: String,
    /// Where the run is, as of the last time the run loop serviced the dashboard.
    now: Time,
    paused: bool,
    finished: Option<String>,
}

/// What the server threads read besides the published state.
struct Server {
    shared: Arc<Mutex<Shared>>,
    logs: Arc<Mutex<Logs>>,
    board_names: Vec<String>,
    bus_feeds: Vec<Arc<Mutex<BusFeed>>>,
    bus_names: Vec<String>,
    tx: mpsc::Sender<Command>,
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
    /// Protocol-neutral mock agents ([`crate::host_protocol`]), with the board each serves.
    agents: Vec<(Option<usize>, Box<dyn crate::host_protocol::HostAgent>)>,
    logs: Arc<Mutex<Logs>>,
    bus_feeds: Vec<Arc<Mutex<BusFeed>>>,
    bus_names: Vec<String>,
    faults: Vec<Value>,
    paused: bool,
    /// Pause when virtual time gets here.
    pause_at: Option<Time>,
    /// Wall time between published states.
    pub publish_every: Duration,
    last_publish: Option<Instant>,
    speed: (Instant, Time, f64),
    started: Instant,
    /// Shown on the page once the run is over.
    pub finished: Option<String>,
    /// The run's recorder, for `POST /api/rewind` (see [`crate::timetravel`]); runs through
    /// [`Dashboard::run_session`] and [`Dashboard::run_robot`] go through it.
    pub travel: Option<crate::timetravel::TimeTravel>,
}

impl Dashboard {
    /// Starts the server on `addr` (e.g. `127.0.0.1:8080`; port 0 picks a free one) for these boards and buses.
    pub fn start(addr: &str, boards: Vec<BoardInfo>, bus_names: Vec<String>) -> std::io::Result<Dashboard> {
        let listener = TcpListener::bind(addr)?;
        let addr = listener.local_addr()?;
        let shared =
            Arc::new(Mutex::new(Shared { state: "{}".into(), now: Time::ZERO, paused: false, finished: None }));
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();
        let logs =
            Arc::new(Mutex::new(Logs { next_seq: 0, boards: boards.iter().map(|_| BoardLog::default()).collect() }));
        let bus_feeds: Vec<Arc<Mutex<BusFeed>>> = bus_names.iter().map(|_| Arc::default()).collect();
        let server = Arc::new(Server {
            shared: shared.clone(),
            logs: logs.clone(),
            board_names: boards.iter().map(|b| b.name.clone()).collect(),
            bus_feeds: bus_feeds.clone(),
            bus_names: bus_names.clone(),
            tx,
        });
        {
            let stop = stop.clone();
            std::thread::Builder::new().name("dashboard".into()).spawn(move || {
                for stream in listener.incoming() {
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    let Ok(stream) = stream else { continue };
                    let server = server.clone();
                    let _ = std::thread::Builder::new().name("dashboard-conn".into()).spawn(move || {
                        let _ = handle(stream, &server);
                    });
                }
            })?;
        }
        let started = Instant::now();
        Ok(Dashboard {
            shared,
            stop,
            commands: rx,
            addr,
            boards,
            links: Vec::new(),
            agents: Vec::new(),
            logs,
            bus_feeds,
            bus_names,
            faults: Vec::new(),
            paused: false,
            pause_at: None,
            publish_every: Duration::from_millis(200),
            last_publish: None,
            speed: (started, Time::ZERO, 0.0),
            started,
            finished: None,
            travel: None,
        })
    }

    /// Runs `s` to `end`, through [`Dashboard::travel`] when there is one.
    pub fn run_session(&mut self, s: &mut Session, end: Time) -> rp2040::StopReason {
        use crate::timetravel::Ran;
        match &mut self.travel {
            None => s.run_until(end),
            Some(tt) => match tt.run_until(s, end) {
                Ran::Reached => rp2040::StopReason::TimeLimit,
                Ran::Interrupted => rp2040::StopReason::Requested,
                Ran::Stopped { reason, .. } => reason,
            },
        }
    }

    /// Runs `r` to `end`, through [`Dashboard::travel`] when there is one (as [`Robot::run_until`]).
    pub fn run_robot(&mut self, r: &mut Robot, end: Time) -> Option<(usize, rp2040::StopReason)> {
        use crate::timetravel::Ran;
        match &mut self.travel {
            None => r.run_until(end),
            Some(tt) => match tt.run_until(r, end) {
                Ran::Reached | Ran::Interrupted => None,
                Ran::Stopped { board, reason } => Some((board, reason)),
            },
        }
    }

    /// A mock bridge (or the Puddles one) attached to `board` or on bus `bus`.
    pub fn add_link(&mut self, board: Option<usize>, bus: Option<usize>, link: Link) {
        self.links.push((board, bus, link));
    }

    /// A mock agent ([`crate::host_protocol::HostAgent`]: a nano-ROS agent) serving `board`: its status goes in the
    /// board's `agent` and `POST /api/agent` sends through it.
    pub fn add_agent(&mut self, board: Option<usize>, agent: Box<dyn crate::host_protocol::HostAgent>) {
        self.agents.push((board, agent));
    }

    /// Sends `msg` on `topic` through `board`'s agent (the only agent when `board` is empty).
    fn agent_send(&self, board: &str, topic: &str, msg: &Value) -> Result<String, String> {
        let agent = if board.is_empty() {
            match &self.agents[..] {
                [(_, a)] => a,
                [] => return Err("no mock agent (--mock-agent)".into()),
                _ => return Err("which board's agent? (give board)".into()),
            }
        } else {
            let i = self.board_index(board)?;
            &self.agents.iter().find(|a| a.0 == Some(i)).ok_or_else(|| format!("{board} has no mock agent"))?.1
        };
        agent.send(topic, msg, None)?;
        Ok(format!("{} agent sends {topic}", agent.protocol()))
    }

    /// Pauses virtual time when it reaches `at` (at once for zero: the run then stops after its first piece).
    pub fn pause_at(&mut self, at: Option<Time>) {
        match at {
            Some(Time::ZERO) => self.paused = true,
            at => self.pause_at = at,
        }
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
        let logs = self.logs.clone();
        let mut prev = s.on_log.take();
        s.on_log = Some(Box::new(move |t, line| {
            if let Some(p) = &mut prev {
                p(t, line);
            }
            let level = line.level.clone().unwrap_or_default().to_lowercase();
            logs.lock().unwrap().push(i, t, &level, line.to_string());
        }));
        let logs = self.logs.clone();
        let mut prev = s.on_uart_line.take();
        s.on_uart_line = Some(Box::new(move |t, n, line| {
            if let Some(p) = &mut prev {
                p(t, n, line);
            }
            logs.lock().unwrap().push(i, t, &format!("uart{n}"), line.to_string());
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
        let mut replies = Vec::new();
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
                    if let Some(tt) = &mut self.travel {
                        tt.changed(s);
                    }
                    replies.push((reply, r.map(|_| format!("{spec} applied to {name}"))));
                }
                Command::Rewind { to, reply } => {
                    let r = rewind(self.travel.as_mut(), s, &to);
                    s.reset_pacing();
                    replies.push((reply, r));
                }
                Command::Input { board, name, value, reply } => {
                    let r = self.set_input(&board, &name, value);
                    replies.push((reply, r));
                }
                Command::Pause { paused, at, reply } => {
                    let r = self.pause(paused, at, s.machine.now);
                    if !self.paused {
                        s.reset_pacing();
                    }
                    replies.push((reply, r));
                }
                Command::Refresh { reply } => replies.push((reply, Ok(String::new()))),
                Command::Thrusters { reply } => {
                    let v = thrusters_json(&self.board_names(), &[&*s], &[], s.machine.now);
                    let _ = reply.send(Ok(self.with_status(v, s.machine.now)));
                }
                Command::Vehicle { reply, .. } => {
                    let _ = reply.send(Err("a single board's run has no vehicle".into()));
                }
                Command::AgentSend { board, topic, msg, reply } => {
                    replies.push((reply, self.agent_send(&board, &topic, &msg)));
                }
            }
        }
        let paused_now = self.pause_if_due(s.machine.now);
        self.note_status(s.machine.now);
        // Publish what the commands did before answering them, so a client that reads the state after its
        // answer sees their effect (and a pause as it happens)
        if !replies.is_empty() || paused_now || self.due() {
            self.last_publish = Some(Instant::now());
            let state = self.session_state(s);
            self.shared.lock().unwrap().state = state.to_string();
        }
        for (reply, r) in replies {
            let _ = reply.send(r);
        }
    }

    /// Between slices of a robot's run: applies what was posted and publishes the state when due. `bridges` are the
    /// mock bridges fault injection uses per bus (the first on each, as for `--fault`).
    pub fn service_robot(&mut self, r: &mut Robot, bridges: &[Option<MockBridge>]) {
        let mut replies = Vec::new();
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
                    if let Some(tt) = &mut self.travel {
                        tt.changed(r);
                    }
                    replies.push((reply, res.map(|_| format!("{spec} applied to {board}"))));
                }
                Command::Rewind { to, reply } => {
                    let res = rewind(self.travel.as_mut(), r, &to);
                    r.reset_pacing();
                    replies.push((reply, res));
                }
                Command::Input { board, name, value, reply } => {
                    replies.push((reply, self.set_input(&board, &name, value)));
                }
                Command::Pause { paused, at, reply } => {
                    let res = self.pause(paused, at, r.now);
                    if !self.paused {
                        r.reset_pacing();
                    }
                    replies.push((reply, res));
                }
                Command::Refresh { reply } => replies.push((reply, Ok(String::new()))),
                Command::Thrusters { reply } => {
                    let _ = reply.send(Ok(self.robot_thrusters(r)));
                }
                Command::Vehicle { name, pose, reply } => {
                    let res = set_vehicle_pose(r, &name, &pose);
                    if let (Ok(_), Some(tt)) = (&res, &mut self.travel) {
                        tt.changed(r);
                    }
                    let res = res.map(|msg| {
                        let mut v = self.robot_thrusters(r);
                        v["message"] = msg.into();
                        v
                    });
                    let _ = reply.send(res);
                }
                Command::AgentSend { board, topic, msg, reply } => {
                    replies.push((reply, self.agent_send(&board, &topic, &msg)));
                }
            }
        }
        let paused_now = self.pause_if_due(r.now);
        self.note_status(r.now);
        if !replies.is_empty() || paused_now || self.due() {
            self.last_publish = Some(Instant::now());
            let state = self.robot_state(r);
            self.shared.lock().unwrap().state = state.to_string();
        }
        for (reply, res) in replies {
            let _ = reply.send(res);
        }
    }

    /// Publishes the final state (the run is over, or stopped with `why`).
    pub fn finish_session(&mut self, s: &mut Session, why: &str) {
        self.finished = Some(why.to_string());
        self.note_status(s.machine.now);
        let state = self.session_state(s);
        self.shared.lock().unwrap().state = state.to_string();
    }

    /// Publishes the final state of a robot.
    pub fn finish_robot(&mut self, r: &mut Robot, why: &str) {
        self.finished = Some(why.to_string());
        self.note_status(r.now);
        let state = self.robot_state(r);
        self.shared.lock().unwrap().state = state.to_string();
    }

    /// A posted pause or resume: `at` pauses when virtual time gets there (now, if it is already past); resuming
    /// with `at` pauses again there.
    fn pause(&mut self, paused: bool, at: Option<Time>, now: Time) -> Result<String, String> {
        match (paused, at) {
            (true, None) => {
                self.paused = true;
                self.pause_at = None;
                Ok("paused".into())
            }
            (true, Some(t)) if t <= now => {
                self.paused = true;
                self.pause_at = None;
                Ok(format!("paused (at {now}, already past {t})"))
            }
            (true, Some(t)) => {
                self.pause_at = Some(t);
                Ok(format!("will pause at {t}"))
            }
            (false, Some(t)) if t <= now => Err(format!("{t} is already past (the time is {now})")),
            (false, at) => {
                self.paused = false;
                self.pause_at = at;
                Ok(match at {
                    Some(t) => format!("running until {t}"),
                    None => "running".into(),
                })
            }
        }
    }

    /// Pauses if the time to has come; says whether it did.
    fn pause_if_due(&mut self, now: Time) -> bool {
        let due = self.pause_at.is_some_and(|t| now >= t);
        if due {
            self.paused = true;
            self.pause_at = None;
        }
        due
    }

    /// Tells the server where the run is, for the live reads.
    fn note_status(&self, now: Time) {
        let mut sh = self.shared.lock().unwrap();
        sh.now = now;
        sh.paused = self.paused;
        sh.finished.clone_from(&self.finished);
    }

    /// Keeps serving the final state (until the process is stopped), answering commands that the run is over.
    pub fn hold(&mut self) -> ! {
        loop {
            match self.commands.recv() {
                Ok(Command::Fault { reply, .. })
                | Ok(Command::Input { reply, .. })
                | Ok(Command::Pause { reply, .. })
                | Ok(Command::Rewind { reply, .. })
                | Ok(Command::AgentSend { reply, .. })
                | Ok(Command::Refresh { reply }) => {
                    let _ = reply.send(Err("the run is over".into()));
                }
                Ok(Command::Thrusters { reply }) | Ok(Command::Vehicle { reply, .. }) => {
                    let _ = reply.send(Err("the run is over".into()));
                }
                Err(_) => std::thread::sleep(Duration::from_secs(3600)),
            }
        }
    }

    fn board_names(&self) -> Vec<String> {
        self.boards.iter().map(|b| b.name.clone()).collect()
    }

    /// `/api/thrusters` of a robot, with where the run is.
    fn robot_thrusters(&self, r: &Robot) -> Value {
        let boards: Vec<&Session> = r.boards.iter().collect();
        let v = thrusters_json(&self.board_names(), &boards, &r.vehicles, r.now);
        self.with_status(v, r.now)
    }

    /// Adds the run's time and state to a live answer.
    fn with_status(&self, mut v: Value, now: Time) -> Value {
        v["time_s"] = now.as_secs_f64().into();
        v["paused"] = self.paused.into();
        v["finished"] = json!(self.finished);
        v
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
            "pause_at_s": self.pause_at.map(Time::as_secs_f64),
            "realtime": realtime,
            "finished": self.finished,
            "faults": self.faults,
            "travel": self.travel.as_ref().map(|tt| {
                let st = tt.stats();
                json!({
                    "live": tt.at_frontier(),
                    "oldest_s": st.oldest.as_secs_f64(),
                    "frontier_s": st.frontier.as_secs_f64(),
                    "checkpoints": st.checkpoints,
                    "memory_mb": st.bytes as f64 / (1 << 20) as f64,
                    "saved": tt.checkpoints().iter().filter_map(|c| c.1.map(|n| json!({"name": n, "time_s": c.0.as_secs_f64()}))).collect::<Vec<_>>(),
                })
            }),
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
                        "external": vs.external,
                        "thrusters": vs.vehicle.thrusters.iter().zip(rpms).map(|(t, rpm)| json!({"name": t.name, "rpm": rpm, "thrust_n": t.thrust(rpm)})).collect::<Vec<_>>(),
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
        let all = self.logs.lock().unwrap();
        let lines = &all.boards[i].lines;
        let logs: Vec<Value> = lines
            .iter()
            .skip(lines.len().saturating_sub(LOG_LINES_SENT))
            .map(|l| json!({"seq": l.seq, "t": l.t.as_secs_f64(), "kind": l.kind, "text": l.text}))
            .collect();
        drop(all);
        let link = self.links.iter().find(|l| l.0 == Some(i)).map(|l| l.2.json(now, &info.fault_names));
        let agent = self.agents.iter().find(|a| a.0 == Some(i)).map(|a| a.1.status(now));
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
            "agent": agent,
        })
    }

    /// The page's view of the state as last published (what `GET /api/state` returns).
    pub fn published(&self) -> String {
        self.shared.lock().unwrap().state.clone()
    }
}

/// `GET /api/thrusters`: every DShot ESC of every board (`escs`: board, ESC name, propeller speed in signed
/// mechanical RPM, the last throttle value it got, whether its signal is lost, and the vehicle thruster it spins
/// with that thruster's thrust in newtons along its direction), and each vehicle's thrusters in the vehicle file's
/// order with its pose. `board_names` are the robot's board names, in order; the speeds are those at `now`.
pub fn thrusters_json(
    board_names: &[String],
    boards: &[&Session],
    vehicles: &[crate::vehicle::VehicleSim],
    now: Time,
) -> Value {
    use sim_devices::dshot::Esc;
    let mut escs = Vec::new();
    for (i, s) in boards.iter().enumerate() {
        for (k, d) in s.machine.sys.pin_devices.iter().enumerate() {
            let Some(e) = d.as_any().downcast_ref::<Esc>() else { continue };
            let mut x = json!({
                "board": board_names.get(i).cloned().unwrap_or_default(),
                "esc": e.name,
                "rpm": e.rpm_at(now),
                "throttle": e.last_throttle(),
                "signal_lost": e.signal_lost(now),
                "thruster": Value::Null,
                "thrust_n": Value::Null,
            });
            for v in vehicles {
                if let Some(t) = v.wiring().iter().position(|w| *w == Some((i, k))) {
                    x["vehicle"] = v.vehicle.name.clone().into();
                    x["thruster"] = v.vehicle.thrusters[t].name.clone().into();
                    x["thrust_n"] = v.vehicle.thrusters[t].thrust(e.rpm_at(now)).into();
                }
            }
            escs.push(x);
        }
    }
    let vehicles: Vec<Value> = vehicles
        .iter()
        .map(|v| {
            let s = &v.vehicle.state;
            let rpms = v.rpms(boards, now);
            let thrusters: Vec<Value> = v
                .vehicle
                .thrusters
                .iter()
                .zip(v.wiring())
                .zip(rpms)
                .map(|((t, w), rpm)| {
                    let (board, esc) = match w {
                        Some((b, k)) => (
                            board_names.get(*b).cloned(),
                            boards[*b].machine.sys.pin_devices[*k]
                                .as_any()
                                .downcast_ref::<Esc>()
                                .map(|e| e.name.clone()),
                        ),
                        None => (None, None),
                    };
                    json!({"name": t.name, "board": board, "esc": esc, "rpm": rpm, "thrust_n": t.thrust(rpm)})
                })
                .collect();
            json!({
                "name": v.vehicle.name,
                "external": v.external,
                "poses_set": v.poses_set,
                "position_m": s.position,
                "rpy_deg": s.attitude.euler().map(f64::to_degrees),
                "thrusters": thrusters,
            })
        })
        .collect();
    json!({"escs": escs, "vehicles": vehicles})
}

/// Puts the vehicle called `name` (or the only one, for an empty name) at `pose`.
fn set_vehicle_pose(r: &mut Robot, name: &str, pose: &ExternalPose) -> Result<String, String> {
    let names: Vec<String> = r.vehicles.iter().map(|v| v.vehicle.name.clone()).collect();
    let v = match name {
        "" if r.vehicles.len() == 1 => &mut r.vehicles[0],
        "" if r.vehicles.is_empty() => return Err("the robot has no vehicle".into()),
        "" => return Err(format!("which vehicle? there are: {}", names.join(", "))),
        n => r
            .vehicles
            .iter_mut()
            .find(|v| v.vehicle.name == n)
            .ok_or_else(|| format!("no vehicle {n:?}; there are: {}", names.join(", ")))?,
    };
    v.set_pose(pose);
    let p = pose.position;
    Ok(format!("{} at [{:.3}, {:.3}, {:.3}] m", v.vehicle.name, p[0], p[1], p[2]))
}

/// The pose of a `POST /api/vehicle` body: `position_m`, `rpy_deg` or `quat_wxyz`, optional `velocity_mps` and
/// `rate_dps`; also returns its `vehicle`.
pub fn parse_pose(body: &Value) -> Result<(String, ExternalPose), String> {
    fn vec<const N: usize>(body: &Value, key: &str) -> Result<Option<[f64; N]>, String> {
        let v = &body[key];
        if v.is_null() {
            return Ok(None);
        }
        let a = v.as_array().filter(|a| a.len() == N).ok_or_else(|| format!("{key}: expected {N} numbers"))?;
        let mut out = [0.0; N];
        for (o, x) in out.iter_mut().zip(a) {
            *o = x.as_f64().filter(|x| x.is_finite()).ok_or_else(|| format!("{key}: expected {N} numbers"))?;
        }
        Ok(Some(out))
    }
    let name = match &body["vehicle"] {
        Value::Null => String::new(),
        Value::String(s) => s.clone(),
        _ => return Err("vehicle: expected a name".into()),
    };
    let position = vec::<3>(body, "position_m")?.ok_or("position_m: missing")?;
    let attitude = match (vec::<3>(body, "rpy_deg")?, vec::<4>(body, "quat_wxyz")?) {
        (Some(_), Some(_)) => return Err("give rpy_deg or quat_wxyz, not both".into()),
        (Some([r, p, y]), None) => sim_physics::Quat::from_euler(r.to_radians(), p.to_radians(), y.to_radians()),
        (None, Some([w, x, y, z])) => {
            let n = (w * w + x * x + y * y + z * z).sqrt();
            if n <= 1e-6 {
                return Err("quat_wxyz: not a rotation".into());
            }
            sim_physics::Quat { w, x, y, z }.normalized()
        }
        (None, None) => return Err("rpy_deg or quat_wxyz: missing".into()),
    };
    let velocity = vec::<3>(body, "velocity_mps")?;
    let rate = vec::<3>(body, "rate_dps")?.map(|r| r.map(f64::to_radians));
    Ok((name, ExternalPose { position, attitude, velocity, rate }))
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

/// The parameters of a query string (`a=1&b=x%20y`), percent-decoded.
pub fn query_params(query: &str) -> BTreeMap<String, String> {
    fn decode(s: &str) -> String {
        let b = s.as_bytes();
        let mut out = Vec::with_capacity(b.len());
        let mut i = 0;
        while i < b.len() {
            let hex = b.get(i + 1..i + 3).and_then(|h| u8::from_str_radix(std::str::from_utf8(h).ok()?, 16).ok());
            match (b[i], hex) {
                (b'+', _) => out.push(b' '),
                (b'%', Some(byte)) => {
                    out.push(byte);
                    i += 2;
                }
                (c, _) => out.push(c),
            }
            i += 1;
        }
        String::from_utf8_lossy(&out).into_owned()
    }
    query
        .split('&')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let (k, v) = p.split_once('=').unwrap_or((p, ""));
            (decode(k), decode(v))
        })
        .collect()
}

/// An unsigned number in decimal or `0x` hex.
fn parse_u64(s: &str) -> Option<u64> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u64::from_str_radix(h, 16).ok(),
        None => s.parse().ok(),
    }
}

/// A read's parameters common to `/api/logs` and `/api/frames`.
struct Cursor {
    since: u64,
    max: usize,
    tail: bool,
}

impl Cursor {
    fn parse(q: &BTreeMap<String, String>) -> Result<Cursor, String> {
        let since = match q.get("since") {
            Some(v) => parse_u64(v).ok_or_else(|| format!("since: not a number: {v:?}"))?,
            None => 0,
        };
        let max = match q.get("max") {
            Some(v) => parse_u64(v).ok_or_else(|| format!("max: not a number: {v:?}"))? as usize,
            None => 200,
        };
        let tail = q.get("tail").is_some_and(|v| v == "1" || v == "true");
        Ok(Cursor { since, max: max.clamp(1, MAX_READ), tail })
    }

    /// Takes up to `max` items after the cursor: the first ones, or with `tail` the last ones. Says whether more
    /// were left out.
    fn take<T>(&self, mut items: Vec<T>) -> (Vec<T>, bool) {
        let more = items.len() > self.max;
        if more {
            if self.tail {
                items.drain(..items.len() - self.max);
            } else {
                items.truncate(self.max);
            }
        }
        (items, more)
    }
}

impl Server {
    fn status(&self) -> serde_json::Map<String, Value> {
        let sh = self.shared.lock().unwrap();
        let mut m = serde_json::Map::new();
        m.insert("time_s".into(), sh.now.as_secs_f64().into());
        m.insert("paused".into(), sh.paused.into());
        m.insert("finished".into(), json!(sh.finished));
        m
    }

    /// `GET /api/logs`: the lines after `since` (of one board, or all of them in order), with the cursor to read
    /// on from (`next`) and whether lines after `since` were already dropped (`truncated`).
    fn logs(&self, q: &BTreeMap<String, String>) -> Result<Value, String> {
        let c = Cursor::parse(q)?;
        let boards: Vec<usize> = match q.get("board").map(String::as_str) {
            None | Some("") => (0..self.board_names.len()).collect(),
            Some(name) => vec![
                self.board_names
                    .iter()
                    .position(|b| b == name)
                    .ok_or_else(|| format!("no board {name:?}; there are: {}", self.board_names.join(", ")))?,
            ],
        };
        let mut m = self.status();
        let logs = self.logs.lock().unwrap();
        let mut lines: Vec<(usize, &LogLine)> = boards
            .iter()
            .flat_map(|&i| logs.boards[i].lines.iter().filter(|l| l.seq > c.since).map(move |l| (i, l)))
            .collect();
        lines.sort_by_key(|(_, l)| l.seq);
        let truncated = boards.iter().any(|&i| logs.boards[i].dropped_to > c.since);
        let (lines, more) = c.take(lines);
        // Reading on from `next` with the same board neither repeats nor skips a line
        let next = match lines.last() {
            Some((_, l)) if more => l.seq,
            _ => logs.next_seq,
        };
        m.insert("next".into(), next.into());
        m.insert("more".into(), more.into());
        m.insert("truncated".into(), truncated.into());
        m.insert(
            "lines".into(),
            lines
                .iter()
                .map(|(i, l)| {
                    json!({"seq": l.seq, "board": self.board_names[*i], "t": l.t.as_secs_f64(), "kind": l.kind, "text": l.text})
                })
                .collect(),
        );
        Ok(Value::Object(m))
    }

    /// `GET /api/frames`: one bus's frames after `since` (the bus's frame count), optionally only those whose ID
    /// matches `id` under `mask`.
    fn frames(&self, q: &BTreeMap<String, String>) -> Result<Value, String> {
        let c = Cursor::parse(q)?;
        let j = match q.get("bus").map(String::as_str) {
            None | Some("") if self.bus_names.len() == 1 => 0,
            None | Some("") if self.bus_names.is_empty() => return Err("there is no CAN bus".into()),
            None | Some("") => return Err(format!("which bus? there are: {}", self.bus_names.join(", "))),
            Some(name) => self
                .bus_names
                .iter()
                .position(|b| b == name)
                .ok_or_else(|| format!("no bus {name:?}; there are: {}", self.bus_names.join(", ")))?,
        };
        let id = match q.get("id") {
            Some(v) => Some(parse_u64(v).ok_or_else(|| format!("id: not a number: {v:?}"))? as u32),
            None => None,
        };
        let mask = match q.get("mask") {
            Some(v) => parse_u64(v).ok_or_else(|| format!("mask: not a number: {v:?}"))? as u32,
            None => u32::MAX,
        };
        let mut m = self.status();
        let feed = self.bus_feeds[j].lock().unwrap();
        let frames: Vec<&(u64, BusRecord)> = feed
            .kept
            .iter()
            .filter(|(seq, r)| *seq > c.since && id.is_none_or(|id| r.frame.id & mask == id & mask))
            .collect();
        let truncated = feed.kept.front().is_some_and(|(first, _)| *first > c.since + 1);
        let (frames, more) = c.take(frames);
        let next = match frames.last() {
            Some((seq, _)) if more => *seq,
            _ => feed.frames,
        };
        m.insert("bus".into(), self.bus_names[j].clone().into());
        m.insert("total".into(), feed.frames.into());
        m.insert("next".into(), next.into());
        m.insert("more".into(), more.into());
        m.insert("truncated".into(), truncated.into());
        m.insert(
            "frames".into(),
            frames
                .iter()
                .map(|(seq, r)| {
                    let mut f = crate::jsonlog::can_json(&self.bus_names[j], r);
                    f["seq"] = (*seq).into();
                    f
                })
                .collect(),
        );
        Ok(Value::Object(m))
    }
}

/// Sends `cmd` to the run loop and answers with what it built (`{"ok": true, ...}`), or the error.
fn live(
    stream: &mut TcpStream,
    server: &Server,
    cmd: Command,
    answer: mpsc::Receiver<Result<Value, String>>,
) -> std::io::Result<()> {
    if server.tx.send(cmd).is_err() {
        return json_reply(stream, 503, json!({"ok": false, "error": "the run is over"}));
    }
    match answer.recv_timeout(REPLY_TIMEOUT) {
        Ok(Ok(mut v)) => {
            v["ok"] = true.into();
            json_reply(stream, 200, v)
        }
        Ok(Err(e)) => json_reply(stream, 422, json!({"ok": false, "error": e})),
        Err(_) => json_reply(stream, 503, json!({"ok": false, "error": "the run loop did not answer"})),
    }
}

fn handle(mut stream: TcpStream, server: &Server) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let Some(req) = read_request(&mut stream)? else { return Ok(()) };
    let (path, query) = req.path.split_once('?').unwrap_or((&req.path, ""));
    match (req.method.as_str(), path) {
        ("GET", "/") | ("GET", "/index.html") => respond(&mut stream, 200, "text/html; charset=utf-8", PAGE.as_bytes()),
        ("GET", "/api/state") => {
            // Fresh: have the run loop publish first (once the run is over, the last state is the final one)
            if query_params(query).get("fresh").is_some_and(|v| v == "1" || v == "true") {
                let (reply, answer) = mpsc::channel();
                if server.tx.send(Command::Refresh { reply }).is_ok() {
                    let _ = answer.recv_timeout(REPLY_TIMEOUT);
                }
            }
            let state = server.shared.lock().unwrap().state.clone();
            respond(&mut stream, 200, "application/json", state.as_bytes())
        }
        ("GET", "/api/logs") | ("GET", "/api/frames") => {
            let q = query_params(query);
            let r = if path == "/api/logs" { server.logs(&q) } else { server.frames(&q) };
            match r {
                Ok(v) => json_reply(&mut stream, 200, v),
                Err(e) => json_reply(&mut stream, 400, json!({"ok": false, "error": e})),
            }
        }
        ("GET", "/api/thrusters") => {
            let (reply, answer) = mpsc::channel();
            live(&mut stream, server, Command::Thrusters { reply }, answer)
        }
        ("POST", "/api/vehicle") => {
            let parsed = serde_json::from_slice::<Value>(&req.body)
                .map_err(|e| format!("bad JSON: {e}"))
                .and_then(|b| parse_pose(&b));
            let (name, pose) = match parsed {
                Ok(p) => p,
                Err(e) => return json_reply(&mut stream, 400, json!({"ok": false, "error": e})),
            };
            let (reply, answer) = mpsc::channel();
            live(&mut stream, server, Command::Vehicle { name, pose, reply }, answer)
        }
        ("POST", "/api/rewind") => {
            let body: Value = match serde_json::from_slice(&req.body) {
                Ok(v) => v,
                Err(e) => return json_reply(&mut stream, 400, json!({"ok": false, "error": format!("bad JSON: {e}")})),
            };
            let to = match (&body["time_s"], body["checkpoint"].as_str()) {
                _ if body["save"].is_string() => RewindTo::Save(body["save"].as_str().unwrap_or("").to_string()),
                (_, Some(name)) => RewindTo::Checkpoint(name.to_string()),
                (v, None) => match v.as_f64() {
                    Some(t) if t.is_finite() && t >= 0.0 => RewindTo::Time(Time::from_secs_f64(t)),
                    _ => {
                        let e = "give time_s (seconds, 0 or more), checkpoint (a name) or save (a name)";
                        return json_reply(&mut stream, 400, json!({"ok": false, "error": e}));
                    }
                },
            };
            let (reply, answer) = mpsc::channel();
            if server.tx.send(Command::Rewind { to, reply }).is_err() {
                return json_reply(&mut stream, 503, json!({"ok": false, "error": "the run is over"}));
            }
            match answer.recv_timeout(REPLY_TIMEOUT) {
                Ok(Ok(msg)) => json_reply(&mut stream, 200, json!({"ok": true, "message": msg})),
                Ok(Err(e)) => json_reply(&mut stream, 422, json!({"ok": false, "error": e})),
                Err(_) => json_reply(&mut stream, 503, json!({"ok": false, "error": "the run loop did not answer"})),
            }
        }
        ("POST", "/api/fault") | ("POST", "/api/input") | ("POST", "/api/pause") | ("POST", "/api/agent") => {
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
                "/api/agent" => {
                    let Some(topic) = body["topic"].as_str().map(str::to_string) else {
                        return json_reply(&mut stream, 400, json!({"ok": false, "error": "needs a topic"}));
                    };
                    Command::AgentSend { board, topic, msg: body["msg"].clone(), reply }
                }
                _ => {
                    let at = match &body["at_s"] {
                        Value::Null => None,
                        v => match v.as_f64() {
                            Some(t) if t.is_finite() && t >= 0.0 => Some(Time::from_secs_f64(t)),
                            _ => {
                                let e = "at_s must be a number of seconds, 0 or more";
                                return json_reply(&mut stream, 400, json!({"ok": false, "error": e}));
                            }
                        },
                    };
                    Command::Pause { paused: body["paused"].as_bool().unwrap_or(true), at, reply }
                }
            };
            if server.tx.send(cmd).is_err() {
                return json_reply(&mut stream, 503, json!({"ok": false, "error": "the run is over"}));
            }
            match answer.recv_timeout(REPLY_TIMEOUT) {
                Ok(Ok(msg)) => json_reply(&mut stream, 200, json!({"ok": true, "message": msg})),
                Ok(Err(e)) => json_reply(&mut stream, 422, json!({"ok": false, "error": e})),
                Err(_) => json_reply(&mut stream, 503, json!({"ok": false, "error": "the run loop did not answer"})),
            }
        }
        (_, "/")
        | (_, "/api/state")
        | (_, "/api/logs")
        | (_, "/api/frames")
        | (_, "/api/fault")
        | (_, "/api/input")
        | (_, "/api/agent")
        | (_, "/api/pause")
        | (_, "/api/thrusters")
        | (_, "/api/rewind")
        | (_, "/api/vehicle") => json_reply(&mut stream, 405, json!({"ok": false, "error": "method not allowed"})),
        _ => json_reply(&mut stream, 404, json!({"ok": false, "error": "not found"})),
    }
}
