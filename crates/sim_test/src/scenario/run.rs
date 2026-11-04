//! Running a scenario: building the robot (or board), the bridges, the hooks that collect what the boards print and
//! send, and the timeline of steps.
//!
//! Virtual time advances in pieces, each a whole number of the robot's 100 µs slices, so stopping to apply an action
//! or to look at a state never changes the run (a robot runs the same in one call or many). Actions take effect at
//! the start of the slice at or after their time. Occurrences (log lines, frames, messages) are judged by their own
//! times; states (link, GPIO, ESC, vehicle, resets) are sampled every millisecond while a condition looks at one.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_devices::can::{BusRecord, Sender};
use sim_host::ethernet::PuddlesBridge;
use sim_host::fault::{BridgeControl, Fault};
use sim_host::host_protocol::{HostAgent, Placement};
use sim_host::mock_bridge::{IvcModem, MockBridge};
use sim_host::robot::{Robot, RobotSpec};
use titan_link::{DeviceToHost, FirmwareStatus, HostToDevice};

use super::spec::*;

type R<T> = Result<T, String>;

/// A firmware image to use instead of the scenario's own: a directory of builds (every board whose image file name
/// is in it), one image (every board running an image of that file name) or one board's (`BOARD=PATH`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ImageOverride {
    Dir(PathBuf),
    File(PathBuf),
    Board(String, PathBuf),
}

impl std::str::FromStr for ImageOverride {
    type Err = String;

    /// `DIR`, `IMAGE` or `BOARD=IMAGE`.
    fn from_str(s: &str) -> R<ImageOverride> {
        if let Some((board, path)) = s.split_once('=') {
            return Ok(ImageOverride::Board(board.to_string(), sim_host::expand_path(path)?));
        }
        let p = sim_host::expand_path(s)?;
        if p.is_dir() {
            Ok(ImageOverride::Dir(p))
        } else if p.is_file() {
            Ok(ImageOverride::File(p))
        } else {
            Err(format!("{s}: no such directory or image"))
        }
    }
}

/// The image board `board` runs instead of `current`, if an override applies (the most specific one wins).
pub fn override_image(overrides: &[ImageOverride], board: &str, current: &Path) -> Option<PathBuf> {
    let name = current.file_name()?;
    overrides
        .iter()
        .find_map(|o| match o {
            ImageOverride::Board(b, p) if b == board => Some(p.clone()),
            _ => None,
        })
        .or_else(|| {
            overrides.iter().find_map(|o| match o {
                ImageOverride::File(p) if p.file_name() == Some(name) => Some(p.clone()),
                _ => None,
            })
        })
        .or_else(|| {
            overrides.iter().find_map(|o| match o {
                ImageOverride::Dir(d) if d.join(name).is_file() => Some(d.join(name)),
                _ => None,
            })
        })
}

/// How to run scenarios.
#[derive(Clone, Debug, Default)]
pub struct Options {
    /// Firmware images in place of the scenario's (A/B comparisons).
    pub images: Vec<ImageOverride>,
    /// Keep what the run produced ([`Outcome::record`]) for comparing runs.
    pub record: bool,
    /// Profile every board's firmware with this sampling interval (recorded with `record`).
    pub profile: Option<Time>,
    /// A scenario whose firmware images are not built is skipped (tests) instead of failing, unless
    /// `SIM_REQUIRE_TITAN_RS` is set.
    pub skip_missing: bool,
    /// Scenarios that reach the `pause` step of this name the same way (the same setup, inputs, bridges, images,
    /// time and steps up to it) share one run up to there: the first to get there keeps a checkpoint of everything
    /// (as `cases` do), the others go on from copies of it (`scenario --from-checkpoint NAME`).
    pub from_checkpoint: Option<String>,
    /// Collect firmware code coverage ([`Outcome::coverage`]; the boards run one instruction at a time).
    pub coverage: bool,
}

/// Checkpoints kept by `pause` steps for [`Options::from_checkpoint`], by what led to them.
type Checkpoints = std::collections::HashMap<String, Arc<Mutex<Option<Engine>>>>;

fn checkpoints() -> &'static Mutex<Checkpoints> {
    static CHECKPOINTS: std::sync::OnceLock<Mutex<Checkpoints>> = std::sync::OnceLock::new();
    CHECKPOINTS.get_or_init(Default::default)
}

/// How many checkpoints `pause` steps keep for [`Options::from_checkpoint`] in this process.
pub fn shared_checkpoints() -> usize {
    checkpoints().lock().unwrap().values().filter(|s| s.lock().unwrap().is_some()).count()
}

/// Where the shared prefix ends: the index of the `pause` step named `name`, if the scenario can share the run up
/// to it (no step after it is watched from the start of the run, which a copy taken later could not do).
fn shared_prefix(s: &Scenario, name: &str) -> Option<usize> {
    let p = s.steps.iter().position(|st| matches!(&st.kind, StepKind::Pause(n) if n == name))?;
    let early = |st: &Step| {
        matches!(st.kind, StepKind::Never(_) | StepKind::Always(_))
            && matches!(
                (st.start, st.end),
                (Start::At(_), None | Some(End::Within(_)) | Some(End::Until(_))) | (Start::Cursor, None)
            )
    };
    let later = s.steps[p + 1..].iter().chain(s.cases.iter().flat_map(|c| c.steps.iter()));
    if later.clone().any(early) {
        return None;
    }
    Some(p)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Pass,
    Fail,
    /// The run ended (or stopped) before the step's turn.
    NotReached,
}

/// What happened to one step.
#[derive(Clone, Debug, serde::Serialize)]
pub struct StepResult {
    pub number: usize,
    pub kind: &'static str,
    pub description: String,
    pub status: Status,
    /// When it passed, failed or was applied.
    #[serde(serialize_with = "ser_time")]
    pub time: Option<Time>,
    /// The line, frame, status or state that decided it; on a failure, what was there instead.
    pub evidence: Vec<String>,
}

fn ser_time<S: serde::Serializer>(t: &Option<Time>, s: S) -> Result<S::Ok, S::Error> {
    match t {
        Some(t) => s.serialize_f64(t.as_secs_f64()),
        None => s.serialize_none(),
    }
}

/// The result of running one scenario (or one of its cases).
#[derive(Clone, Debug)]
pub struct Outcome {
    pub name: String,
    pub file: PathBuf,
    pub document: usize,
    pub case: Option<String>,
    pub steps: Vec<StepResult>,
    /// The scenario could not be set up or run to the end (a missing image, a board that stopped, a panic).
    pub error: Option<String>,
    /// Why it was skipped (an image not built, with [`Options::skip_missing`]).
    pub skipped: Option<String>,
    /// Virtual time reached.
    pub end: Time,
    pub wall: std::time::Duration,
    /// Each board and the image it ran.
    pub images: Vec<(String, PathBuf)>,
    pub record: Option<Record>,
    /// What ran of each image, with [`Options::coverage`] (boards running the same image merged).
    pub coverage: Option<sim_host::coverage::Coverage>,
}

impl Outcome {
    pub fn passed(&self) -> bool {
        self.error.is_none() && self.steps.iter().all(|s| s.status == Status::Pass)
    }

    /// `name` or `name / case`.
    pub fn title(&self) -> String {
        match &self.case {
            Some(c) => format!("{} / {c}", self.name),
            None => self.name.clone(),
        }
    }

    fn empty(s: &Scenario) -> Outcome {
        Outcome {
            name: s.name.clone(),
            file: s.file.clone(),
            document: s.document,
            case: None,
            steps: Vec::new(),
            error: None,
            skipped: None,
            end: Time::ZERO,
            wall: Default::default(),
            images: Vec::new(),
            record: None,
            coverage: None,
        }
    }
}

/// A line a board printed.
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub t: Time,
    /// The defmt level (`info`, ...), or `uart0`, `uart1`, `usb`.
    pub kind: String,
    /// The message (without the firmware's timestamp and level), or the serial line.
    pub text: String,
    /// As the CLI prints it after the board's name: `0.068333 INFO  ROS connected`.
    pub display: String,
}

/// Everything a run produced that two runs can be compared on.
#[derive(Clone, Debug, Default)]
pub struct Record {
    pub boards: Vec<BoardRecord>,
    /// Each bus's acknowledged frames, as a CAN log.
    pub buses: Vec<(String, Vec<sim_host::canlog::LogFrame>)>,
    pub end: Time,
}

#[derive(Clone, Debug, Default)]
pub struct BoardRecord {
    pub name: String,
    pub image: PathBuf,
    /// FNV-1a of the image file, to tell builds apart.
    pub image_hash: u64,
    pub lines: Vec<Line>,
    /// Chip resets (watchdog and power-on after a brownout) and when they were seen.
    pub resets: Vec<Time>,
    pub lockups: u64,
    pub fault_names: Vec<String>,
    /// Every `FirmwareStatus` the board's bridge decoded: when, and its fault bits.
    pub statuses: Vec<(Time, u32)>,
    /// Each ESC's speed every 10 ms.
    pub escs: Vec<(String, Vec<f64>)>,
    pub profile: Option<sim_host::profile::Profile>,
}

/// FNV-1a 64 of a file's bytes (0 if unreadable).
pub fn file_hash(p: &Path) -> u64 {
    std::fs::read(p)
        .map(|d| d.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ *b as u64).wrapping_mul(0x100_0000_01b3)))
        .unwrap_or(0)
}

/// What the boards printed and the buses carried, collected by hooks.
#[derive(Clone, Default)]
struct Store {
    lines: Vec<Vec<Line>>,
    frames: Vec<Vec<BusRecord>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BridgeRef {
    Can(usize),
    Eth(usize),
}

/// A protocol-neutral host side: a mock agent (`agents`), or a CAN mock bridge seen as one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AgentRef {
    Agent(usize),
    Can(usize),
}

/// The robot and everything around it.
struct World {
    robot: Robot,
    /// Mock bridges on buses: bus, client, bridge.
    can: Vec<(usize, u8, MockBridge)>,
    /// Bridges on Ethernet cables: board, bridge.
    eth: Vec<(usize, PuddlesBridge)>,
    /// Mock agents in the boards' own protocols (`agents`): where each is, and the agent.
    agents: Vec<(Placement, Box<dyn HostAgent>)>,
    store: Arc<Mutex<Store>>,
    /// Per bus, the board each CAN node belongs to.
    nodes: Vec<Vec<usize>>,
    fault_names: Vec<Vec<String>>,
    /// Resets counted before the machine's own count (which a power-on clears).
    reset_base: Vec<u32>,
    reset_times: Vec<Vec<Time>>,
    last_resets: Vec<u32>,
    /// Boards that stopped for good, with why.
    stopped: Vec<(Time, usize, String)>,
    escs: Vec<(usize, usize, String, Vec<f64>)>,
    next_esc_sample: Time,
}

/// The standard titan fault bits (`titan_rp2040::safety`); boards add their own from bit 5.
const STANDARD_FAULTS: [&str; 5] = [
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",
];

impl World {
    fn install_hooks(&mut self) {
        let n = self.robot.boards.len();
        {
            let mut st = self.store.lock().unwrap();
            st.lines.resize(n, Vec::new());
            st.frames.resize(self.robot.buses.len(), Vec::new());
        }
        for (i, s) in self.robot.boards.iter_mut().enumerate() {
            s.keep_logs = false;
            s.stop_on_lockup = false;
            let st = self.store.clone();
            s.on_log = Some(Box::new(move |t, l| {
                let line = Line {
                    t,
                    kind: l.level.clone().unwrap_or_default().to_lowercase(),
                    text: l.message.clone(),
                    display: l.to_string(),
                };
                st.lock().unwrap().lines[i].push(line);
            }));
            let st = self.store.clone();
            s.on_uart_line = Some(Box::new(move |t, u, l| {
                let line = Line { t, kind: format!("uart{u}"), text: l.to_string(), display: format!("uart{u}: {l}") };
                st.lock().unwrap().lines[i].push(line);
            }));
            let st = self.store.clone();
            s.on_usb_cdc_line = Some(Box::new(move |t, l| {
                let line = Line { t, kind: "usb".into(), text: l.to_string(), display: format!("usb: {l}") };
                st.lock().unwrap().lines[i].push(line);
            }));
        }
        for (j, rb) in self.robot.buses.iter_mut().enumerate() {
            rb.bus.record.get_or_insert_with(Vec::new);
            let st = self.store.clone();
            rb.on_can = Some(Box::new(move |r| st.lock().unwrap().frames[j].push(r.clone())));
        }
    }

    /// A copy that runs on independently: robot, bridges and what was collected so far.
    fn snapshot(&self) -> World {
        let mut robot = self.robot.snapshot();
        // A session's snapshot starts without a profiler: the copy carries on with the samples so far
        for (copy, orig) in robot.boards.iter_mut().zip(&self.robot.boards) {
            copy.profiler = orig.profiler.clone();
        }
        let can: Vec<(usize, u8, MockBridge)> = self.can.iter().map(|(j, c, b)| (*j, *c, b.fork())).collect();
        for (j, _, b) in &can {
            robot.buses[*j].endpoints.push(Box::new(b.clone()));
        }
        let eth: Vec<(usize, PuddlesBridge)> = self.eth.iter().map(|(i, b)| (*i, b.fork())).collect();
        for (i, b) in &eth {
            robot.boards[*i].eth_endpoints.push(Box::new(b.clone()));
        }
        let agents: Vec<(Placement, Box<dyn HostAgent>)> = self.agents.iter().map(|(p, a)| (*p, a.fork())).collect();
        for (p, a) in &agents {
            robot.buses[p.bus].endpoints.push(a.endpoint());
        }
        let store = self.store.lock().unwrap().clone();
        let mut w = World {
            robot,
            can,
            eth,
            agents,
            store: Arc::new(Mutex::new(store)),
            nodes: self.nodes.clone(),
            fault_names: self.fault_names.clone(),
            reset_base: self.reset_base.clone(),
            reset_times: self.reset_times.clone(),
            last_resets: self.last_resets.clone(),
            stopped: self.stopped.clone(),
            escs: self.escs.clone(),
            next_esc_sample: self.next_esc_sample,
        };
        w.install_hooks();
        w
    }

    fn now(&self) -> Time {
        self.robot.now
    }

    fn slice(&self) -> Time {
        self.robot.slice
    }

    /// `t` rounded up to the slice grid.
    fn grid(&self, t: Time) -> Time {
        let s = self.slice().0.max(1);
        Time(t.0.div_ceil(s) * s)
    }

    fn board(&self, name: &str) -> R<usize> {
        self.robot.index(name).ok_or_else(|| format!("no board {name:?} (the boards: {})", self.robot.names.join(", ")))
    }

    fn bus(&self, name: &str) -> R<usize> {
        self.robot
            .bus_names
            .iter()
            .position(|n| n == name)
            .ok_or_else(|| format!("no bus {name:?} (the buses: {})", self.robot.bus_names.join(", ")))
    }

    fn only_bus(&self) -> R<usize> {
        match self.robot.buses.len() {
            1 => Ok(0),
            0 => Err("there is no CAN bus".into()),
            _ => Err(format!("which bus? ({})", self.robot.bus_names.join(", "))),
        }
    }

    fn bridge_spec(&self, b: &BridgeSpec) -> R<BridgeRef> {
        match b {
            BridgeSpec::Can { bus, client } => {
                let j = match bus {
                    Some(n) => self.bus(n)?,
                    None => self.only_bus()?,
                };
                self.can
                    .iter()
                    .position(|(jj, c, _)| *jj == j && c == client)
                    .map(BridgeRef::Can)
                    .ok_or_else(|| format!("no mock bridge {b} (add it to `bridges`)"))
            }
            BridgeSpec::Eth { board } => {
                let i = self.board(board)?;
                self.eth
                    .iter()
                    .position(|(ii, _)| *ii == i)
                    .map(BridgeRef::Eth)
                    .ok_or_else(|| format!("no mock bridge on {board}'s cable (add eth:{board} to `bridges`)"))
            }
        }
    }

    /// The bridge serving board `i`: its CANmore client on its bus, or its Ethernet cable's.
    fn board_bridge(&self, i: usize) -> Option<BridgeRef> {
        if let (Some(j), Some(c)) = (self.robot.board_bus[i], self.robot.client_ids[i])
            && let Some(k) = self.can.iter().position(|(jj, cc, _)| *jj == j && *cc == c)
        {
            return Some(BridgeRef::Can(k));
        }
        self.eth.iter().position(|(ii, _)| *ii == i).map(BridgeRef::Eth)
    }

    fn target_bridge(&self, t: &Target) -> R<BridgeRef> {
        if let Some(b) = &t.bridge {
            return self.bridge_spec(b);
        }
        if let Some(b) = &t.board {
            let i = self.board(b)?;
            return self.board_bridge(i).ok_or_else(|| {
                let hint = match (self.robot.board_bus[i], self.robot.client_ids[i]) {
                    (Some(j), Some(c)) => {
                        format!(" (add {}:{c} to `bridges`, or `bridges: all`)", self.robot.bus_names[j])
                    }
                    _ => " (add it to `bridges`)".into(),
                };
                format!("{b} has no mock bridge{hint}")
            });
        }
        match (self.can.len(), self.eth.len()) {
            (1, 0) => Ok(BridgeRef::Can(0)),
            (0, 1) => Ok(BridgeRef::Eth(0)),
            (0, 0) => Err("there is no mock bridge (add `bridges`)".into()),
            _ => Err("which bridge? (give `board` or `bridge`)".into()),
        }
    }

    /// The board a bridge serves.
    fn bridge_board(&self, b: BridgeRef) -> Option<usize> {
        match b {
            BridgeRef::Can(k) => {
                let (j, c, _) = &self.can[k];
                (0..self.robot.boards.len())
                    .find(|&i| self.robot.board_bus[i] == Some(*j) && self.robot.client_ids[i] == Some(*c))
            }
            BridgeRef::Eth(k) => Some(self.eth[k].0),
        }
    }

    fn target_board(&self, t: &Target) -> R<usize> {
        if let Some(b) = &t.board {
            return self.board(b);
        }
        if let Some(b) = &t.bridge {
            let r = self.bridge_spec(b)?;
            return self.bridge_board(r).ok_or_else(|| format!("no board is client of bridge {b}"));
        }
        match self.robot.boards.len() {
            1 => Ok(0),
            _ => Err(format!("which board? (give `board`: {})", self.robot.names.join(", "))),
        }
    }

    fn bridge_name(&self, b: BridgeRef) -> String {
        match b {
            BridgeRef::Can(k) => format!("{}:{}", self.robot.bus_names[self.can[k].0], self.can[k].1),
            BridgeRef::Eth(k) => format!("eth:{}", self.robot.names[self.eth[k].0]),
        }
    }

    /// The agent a topic step uses: `bridge` (a mock bridge), or the board's agent (`agents`), else its mock bridge.
    fn target_agent(&self, t: &Target) -> R<AgentRef> {
        if let Some(b) = &t.bridge {
            return match self.bridge_spec(b)? {
                BridgeRef::Can(k) => Ok(AgentRef::Can(k)),
                BridgeRef::Eth(_) => Err(format!("{b}: topics go through a CAN agent or bridge")),
            };
        }
        if let Some(b) = &t.board {
            let i = self.board(b)?;
            if let Some(k) = self.agents.iter().position(|(p, _)| p.board == Some(i)) {
                return Ok(AgentRef::Agent(k));
            }
            return match self.board_bridge(i) {
                Some(BridgeRef::Can(k)) => Ok(AgentRef::Can(k)),
                _ => Err(format!("{b} has no mock agent (add `agents: [nanoros:{b}]`) or CAN mock bridge")),
            };
        }
        match (self.agents.len(), self.can.len()) {
            (1, _) => Ok(AgentRef::Agent(0)),
            (0, 1) => Ok(AgentRef::Can(0)),
            (0, 0) => Err("there is no mock agent (add `agents`)".into()),
            _ => Err("which agent? (give `board`)".into()),
        }
    }

    fn host(&self, a: AgentRef) -> &dyn HostAgent {
        match a {
            AgentRef::Agent(k) => &*self.agents[k].1,
            AgentRef::Can(k) => &self.can[k].2,
        }
    }

    fn agent_name(&self, a: AgentRef) -> String {
        match a {
            AgentRef::Agent(k) => {
                let p = self.agents[k].0;
                p.board.map_or_else(
                    || format!("{}:{}", self.robot.bus_names[p.bus], p.client),
                    |i| self.robot.names[i].clone(),
                )
            }
            AgentRef::Can(k) => self.bridge_name(BridgeRef::Can(k)),
        }
    }

    fn with_received<T>(&self, b: BridgeRef, f: impl FnOnce(&[(Time, DeviceToHost)]) -> T) -> T {
        match b {
            BridgeRef::Can(k) => f(&self.can[k].2.state().received),
            BridgeRef::Eth(k) => f(&self.eth[k].1.state().received),
        }
    }

    fn send(&self, b: BridgeRef, msg: HostToDevice) {
        match b {
            BridgeRef::Can(k) => self.can[k].2.send(msg),
            BridgeRef::Eth(k) => self.eth[k].1.send(msg),
        }
    }

    /// Chip resets of board `i` so far. A brownout's power-on reset counts once the board is powered again (the
    /// machine's own count starts again from 0 there).
    fn resets(&self, i: usize) -> u32 {
        if self.robot.power_on[i].1 {
            self.reset_base[i] + self.robot.boards[i].machine.resets
        } else {
            self.reset_base[i].saturating_sub(1)
        }
    }

    /// Notes resets and ESC speeds (for the record) after a piece.
    fn sample(&mut self, record: bool) {
        let now = self.now();
        for i in 0..self.robot.boards.len() {
            let r = self.resets(i);
            while self.last_resets[i] < r {
                self.reset_times[i].push(now);
                self.last_resets[i] += 1;
            }
        }
        if record && now >= self.next_esc_sample {
            for (i, k, _, samples) in &mut self.escs {
                let d = &self.robot.boards[*i].machine.sys.pin_devices[*k];
                if let Some(e) = d.as_any().downcast_ref::<sim_devices::dshot::Esc>() {
                    samples.push(e.rpm_at(now));
                }
            }
            self.next_esc_sample = now + Time::from_ms(10);
        }
    }

    /// Runs to `t` (on the slice grid). A board that stops for good is noted and the others go on.
    fn run_to(&mut self, t: Time) {
        let mut stops = 0;
        while self.robot.now < t {
            match self.robot.run_until(t) {
                None => {}
                Some((i, why)) => {
                    let at = self.robot.boards[i].machine.now;
                    self.stopped.push((at, i, format!("{why:?}")));
                    stops += 1;
                    if stops > 4 {
                        // Not a lockup the board stays in quietly: give up on the run
                        self.robot.now = t;
                        return;
                    }
                }
            }
        }
    }
}

// ---- Resolved conditions

#[derive(Clone, Debug)]
enum RWhat {
    Log { board: usize, text: TextMatch, level: Option<String> },
    Status { bridge: BridgeRef, check: StatusCheck, names: Vec<String>, raised: u32, clear: u32 },
    Link { bridge: BridgeRef, up: bool },
    Frame { bus: usize, sender: Option<usize>, m: FrameMatch },
    Gpio { board: usize, pin: u8, high: bool },
    Esc { board: usize, dev: usize, rpm: Range },
    Vehicle { v: usize, checks: Vec<(VehicleQty, Range)> },
    Resets { board: usize, r: Range },
    Lockups { board: usize, r: Range },
    Ivc { bridge: BridgeRef, byte: Option<u8> },
    Message { bridge: BridgeRef, path: Vec<String>, value: Option<ValueCheck> },
    Topic { agent: AgentRef, topic: String, msg: Option<serde_json::Value> },
}

impl RWhat {
    fn is_state(&self) -> bool {
        matches!(
            self,
            RWhat::Link { .. }
                | RWhat::Gpio { .. }
                | RWhat::Esc { .. }
                | RWhat::Vehicle { .. }
                | RWhat::Resets { .. }
                | RWhat::Lockups { .. }
        )
    }

    fn is_stream(&self) -> bool {
        matches!(self, RWhat::Status { .. } | RWhat::Message { .. } | RWhat::Topic { .. })
    }
}

fn fault_bit(names: &[String], f: &FaultRef) -> R<u32> {
    match f {
        FaultRef::Bit(b) if *b < 32 => Ok(1 << b),
        FaultRef::Bit(b) => Err(format!("fault bit {b}: 0 to 31")),
        FaultRef::Name(n) => {
            let up = n.to_uppercase();
            let full = if up.starts_with("FAULT_") { up } else { format!("FAULT_{up}") };
            names
                .iter()
                .position(|x| *x == full)
                .map(|b| 1 << b)
                .ok_or_else(|| format!("no fault {n} on this board (its faults: {})", names.join(", ")))
        }
    }
}

fn fault_list(names: &[String], bits: u32) -> String {
    let v: Vec<String> = (0..32)
        .filter(|b| bits & (1 << b) != 0)
        .map(|b| names.get(b).cloned().unwrap_or_else(|| format!("fault {b}")))
        .collect();
    if v.is_empty() { "none".into() } else { v.join(", ") }
}

fn resolve(w: &World, c: &Condition) -> R<RWhat> {
    Ok(match &c.what {
        What::Log { text, level } => {
            RWhat::Log { board: w.target_board(&c.target)?, text: text.clone(), level: level.clone() }
        }
        What::Status(check) => {
            let bridge = w.target_bridge(&c.target)?;
            let names = w
                .bridge_board(bridge)
                .map(|i| w.fault_names[i].clone())
                .unwrap_or_else(|| STANDARD_FAULTS.iter().map(|s| s.to_string()).collect());
            let bits = |l: &[FaultRef]| l.iter().try_fold(0u32, |a, f| Ok::<u32, String>(a | fault_bit(&names, f)?));
            let raised = bits(&check.raised)?;
            let clear = bits(&check.clear)?;
            RWhat::Status { bridge, check: check.clone(), names, raised, clear }
        }
        What::Link(up) => RWhat::Link { bridge: w.target_bridge(&c.target)?, up: *up },
        What::Frame(m) => {
            let bus = match (&c.bus, &c.target.board) {
                (Some(b), _) => w.bus(b)?,
                (None, Some(b)) => {
                    let i = w.board(b)?;
                    w.robot.board_bus[i].ok_or_else(|| format!("{b} is not on a CAN bus"))?
                }
                (None, None) => w.only_bus()?,
            };
            let sender = c.target.board.as_deref().map(|b| w.board(b)).transpose()?;
            RWhat::Frame { bus, sender, m: m.clone() }
        }
        What::Gpio { pin, high } => {
            let board = w.target_board(&c.target)?;
            let pins = &w.robot.pins[board];
            let pin = match pin {
                PinRef::Number(n) if *n < 30 => *n,
                PinRef::Number(n) => return Err(format!("GPIO {n}: 0 to 29")),
                PinRef::Name(n) => *pins.get(n).or_else(|| pins.get(&format!("{n}_pin"))).ok_or_else(|| {
                    let names: Vec<&str> = pins.keys().map(|k| k.strip_suffix("_pin").unwrap_or(k)).collect();
                    format!("no pin {n:?} in {}'s board file (its pins: {})", w.robot.names[board], names.join(", "))
                })?,
            };
            RWhat::Gpio { board, pin, high: *high }
        }
        What::Esc { name, rpm } => {
            let board = w.target_board(&c.target)?;
            let escs: Vec<(usize, String)> = w.robot.boards[board]
                .machine
                .sys
                .pin_devices
                .iter()
                .enumerate()
                .filter_map(|(k, d)| d.as_any().downcast_ref::<sim_devices::dshot::Esc>().map(|e| (k, e.name.clone())))
                .collect();
            let dev = match name {
                Some(n) => escs.iter().find(|e| e.1 == *n).map(|e| e.0),
                None if escs.len() == 1 => Some(escs[0].0),
                None => None,
            }
            .ok_or_else(|| {
                let list: Vec<&str> = escs.iter().map(|e| e.1.as_str()).collect();
                format!("{}: which ESC? (give esc.name: {})", w.robot.names[board], list.join(", "))
            })?;
            RWhat::Esc { board, dev, rpm: *rpm }
        }
        What::Vehicle { name, checks } => {
            let names: Vec<String> = w.robot.vehicles.iter().map(|v| v.vehicle.name.clone()).collect();
            let v = match name {
                Some(n) => names.iter().position(|x| x == n),
                None if names.len() == 1 => Some(0),
                None => None,
            }
            .ok_or_else(|| {
                format!(
                    "which vehicle? (the vehicles: {})",
                    if names.is_empty() { "none".into() } else { names.join(", ") }
                )
            })?;
            RWhat::Vehicle { v, checks: checks.clone() }
        }
        What::Resets(r) => RWhat::Resets { board: w.target_board(&c.target)?, r: *r },
        What::Lockups(r) => RWhat::Lockups { board: w.target_board(&c.target)?, r: *r },
        What::Ivc(b) => RWhat::Ivc { bridge: w.target_bridge(&c.target)?, byte: *b },
        What::Message { path, value } => {
            RWhat::Message { bridge: w.target_bridge(&c.target)?, path: path.clone(), value: value.clone() }
        }
        What::Topic { topic, msg } => {
            let agent = w.target_agent(&c.target)?;
            let known = w.host(agent).topics();
            // A topic the agent knows the board does not have (from its ros.toml) is a typo
            if w.host(agent).protocol() == "nanoros" && !known.is_empty() && !known.iter().any(|(t, _)| t == topic) {
                let names: Vec<&str> = known.iter().map(|(t, _)| t.as_str()).collect();
                return Err(format!("no topic {topic:?} (the board's topics: {})", names.join(", ")));
            }
            RWhat::Topic { agent, topic: topic.clone(), msg: msg.clone() }
        }
    })
}

/// A status summary: `faults 0x2 (FAULT_WATCHDOG_WARNING), uptime 13.20 s, kill switch enabled`.
fn status_text(s: &FirmwareStatus, names: &[String]) -> String {
    let kill = if s.kill_switches_asserting_kill != 0 {
        "killed"
    } else if s.kill_switches_timed_out != 0 {
        "kill switch timed out"
    } else {
        "enabled"
    };
    format!(
        "{} client {}: faults {:#x} ({}), uptime {:.2} s, {kill}",
        s.board_name.as_str(),
        s.client_id,
        s.faults,
        fault_list(names, s.faults),
        s.uptime_ms as f64 / 1000.0
    )
}

/// Whether a status passes the check, or what does not.
fn check_status(
    s: &FirmwareStatus,
    check: &StatusCheck,
    names: &[String],
    raised: u32,
    clear: u32,
) -> Result<(), String> {
    let mut bad = Vec::new();
    if let Some(f) = check.faults
        && s.faults != f
    {
        bad.push(format!("faults {:#x}, not {f:#x}", s.faults));
    }
    if s.faults & raised != raised {
        bad.push(format!("not raised: {}", fault_list(names, raised & !s.faults)));
    }
    if s.faults & clear != 0 {
        bad.push(format!("raised: {}", fault_list(names, s.faults & clear)));
    }
    if let Some(k) = check.killed
        && (s.kill_switches_asserting_kill != 0) != k
    {
        bad.push(if k { "not killed".into() } else { "killed".into() });
    }
    if let Some(k) = check.kill_timed_out
        && (s.kill_switches_timed_out != 0) != k
    {
        bad.push(if k { "no kill switch timed out".into() } else { "a kill switch timed out".into() });
    }
    if let Some(r) = check.uptime
        && !r.contains(s.uptime_ms as f64 / 1000.0)
    {
        bad.push(format!("uptime {:.3} s, not {r}", s.uptime_ms as f64 / 1000.0));
    }
    if let Some(n) = &check.board_name
        && s.board_name.as_str() != n
    {
        bad.push(format!("board name {:?}", s.board_name.as_str()));
    }
    if let Some(c) = check.client_id
        && s.client_id != c
    {
        bad.push(format!("client {}", s.client_id));
    }
    if bad.is_empty() { Ok(()) } else { Err(bad.join("; ")) }
}

fn json_at<'a>(v: &'a serde_json::Value, path: &[String]) -> Option<&'a serde_json::Value> {
    let mut cur = v;
    for (k, p) in path.iter().enumerate() {
        cur = match cur {
            serde_json::Value::Object(o) => o.get(p)?,
            // A unit variant (`Heartbeat`) is a plain string
            serde_json::Value::String(s) if s == p && k + 1 == path.len() => return Some(cur),
            serde_json::Value::Array(a) => a.get(p.parse::<usize>().ok()?)?,
            _ => return None,
        };
    }
    Some(cur)
}

fn compact(v: &serde_json::Value) -> String {
    let s = v.to_string();
    if s.len() > 160 { format!("{}...", &s[..160]) } else { s }
}

/// One occurrence: when, whether it is of the kind looked at, whether it satisfies the condition.
struct Item {
    t: Time,
    relevant: bool,
    ok: bool,
    evidence: String,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Expect,
    Never,
    Always,
}

/// A condition being watched.
#[derive(Clone)]
struct Active {
    result: usize,
    mode: Mode,
    what: RWhat,
    count: Option<Range>,
    rate: Option<Range>,
    from: Time,
    to: Time,
    /// An instant check of a value stream (the latest status or message at `to`).
    latest: bool,
    /// An `expect` with no window: checked at `to`.
    instant: bool,
    max_age: Time,
    /// The next occurrence to look at (index into its source), once scanning has started.
    next: Option<usize>,
    counted: usize,
    seen: usize,
    first: Vec<String>,
    last_bad: Option<String>,
}

fn board_tag(w: &World, i: usize) -> String {
    w.robot.names[i].clone()
}

impl Active {
    /// Calls `f` with each new occurrence up to `upto`.
    fn scan(&mut self, w: &World, upto: Time, f: &mut dyn FnMut(Item)) {
        let from = self.from;
        let start = |times: &mut dyn Iterator<Item = Time>| -> usize { times.take_while(|t| *t < from).count() };
        match &self.what {
            RWhat::Log { board, text, level } => {
                let st = w.store.lock().unwrap();
                let lines = &st.lines[*board];
                let mut k = *self.next.get_or_insert_with(|| lines.partition_point(|l| l.t < from));
                while k < lines.len() && lines[k].t <= upto {
                    let l = &lines[k];
                    let ok = level.as_ref().is_none_or(|lv| *lv == l.kind) && text.matches(&l.text);
                    let evidence =
                        if ok { format!("[{}] {}: {}", l.t, board_tag(w, *board), l.display) } else { String::new() };
                    f(Item { t: l.t, relevant: true, ok, evidence });
                    k += 1;
                }
                self.next = Some(k);
            }
            RWhat::Frame { bus, sender, m } => {
                let st = w.store.lock().unwrap();
                let frames = &st.frames[*bus];
                let mut k = *self.next.get_or_insert_with(|| frames.partition_point(|r| r.end < from));
                while k < frames.len() && frames[k].end <= upto {
                    let r = &frames[k];
                    let from_board = match (sender, r.sender) {
                        (None, _) => true,
                        (Some(b), Sender::Node(n)) => w.nodes[*bus].get(n) == Some(b),
                        (Some(_), _) => false,
                    };
                    let ok = from_board && r.acked && r.error.is_none() && m.matches(&r.frame);
                    let evidence = if ok {
                        format!("[{}] {} {}", r.end, w.robot.bus_names[*bus], sim_devices::can::candump(&r.frame))
                    } else {
                        String::new()
                    };
                    f(Item { t: r.end, relevant: true, ok, evidence });
                    k += 1;
                }
                self.next = Some(k);
            }
            RWhat::Status { bridge, .. } | RWhat::Ivc { bridge, .. } | RWhat::Message { bridge, .. } => {
                let bridge = *bridge;
                let name = w.bridge_name(bridge);
                let what = self.what.clone();
                let mut next = self.next;
                w.with_received(bridge, |rx| {
                    let mut k = *next.get_or_insert_with(|| start(&mut rx.iter().map(|m| m.0)));
                    while k < rx.len() && rx[k].0 <= upto {
                        let (t, msg) = &rx[k];
                        f(judge(&what, &name, *t, msg));
                        k += 1;
                    }
                    next = Some(k);
                });
                self.next = next;
            }
            RWhat::Topic { agent, .. } => {
                let host = w.host(*agent);
                let name = w.agent_name(*agent);
                let k = *self.next.get_or_insert_with(|| {
                    let all = host.messages(0);
                    all.partition_point(|m| m.t < from)
                });
                let fresh = host.messages(k);
                let mut n = k;
                for m in fresh.iter().take_while(|m| m.t <= upto) {
                    f(judge_topic(&self.what, &name, m));
                    n += 1;
                }
                self.next = Some(n);
            }
            _ => unreachable!("not an occurrence"),
        }
    }

    /// The latest occurrence of a value stream at or before `t`.
    fn latest(&self, w: &World, t: Time) -> Option<Item> {
        if let RWhat::Topic { agent, .. } = &self.what {
            let name = w.agent_name(*agent);
            let all = w.host(*agent).messages(0);
            return all
                .iter()
                .rev()
                .filter(|m| m.t <= t)
                .map(|m| judge_topic(&self.what, &name, m))
                .find(|i| i.relevant);
        }
        let (RWhat::Status { bridge, .. } | RWhat::Message { bridge, .. }) = &self.what else { unreachable!() };
        let name = w.bridge_name(*bridge);
        w.with_received(*bridge, |rx| {
            rx.iter()
                .rev()
                .filter(|m| m.0 <= t)
                .map(|(mt, msg)| judge(&self.what, &name, *mt, msg))
                .find(|i| i.relevant)
        })
    }

    /// A state's value now: whether it holds, and what it is.
    fn sample(&self, w: &World) -> (bool, String) {
        let now = w.now();
        let r = &w.robot;
        match &self.what {
            RWhat::Link { bridge, up } => {
                let last = w.with_received(*bridge, |rx| {
                    rx.iter().rev().find(|m| matches!(m.1, DeviceToHost::Heartbeat)).map(|m| m.0)
                });
                let is_up = last.is_some_and(|t| now.saturating_sub(t) <= Time::from_ms(300));
                let text = match last {
                    Some(t) => format!("last link heartbeat at {t} ({} at {now})", if is_up { "up" } else { "down" }),
                    None => format!("no link heartbeat by {now}"),
                };
                (is_up == *up, text)
            }
            RWhat::Gpio { board, pin, high } => {
                let level = r.boards[*board].machine.sys.gpio.level & (1 << pin) != 0;
                let label = r.pins[*board]
                    .iter()
                    .find(|(_, p)| **p == *pin)
                    .map(|(n, _)| format!(" ({})", n.strip_suffix("_pin").unwrap_or(n)))
                    .unwrap_or_default();
                let lv = if level { "high" } else { "low" };
                (level == *high, format!("{}: GPIO {pin}{label} {lv} at {now}", r.names[*board]))
            }
            RWhat::Esc { board, dev, rpm } => {
                let e =
                    r.boards[*board].machine.sys.pin_devices[*dev].as_any().downcast_ref::<sim_devices::dshot::Esc>();
                let e = e.expect("an ESC");
                let v = e.rpm_at(now);
                (rpm.contains(v), format!("{} {}: {v:.0} rpm at {now}", r.names[*board], e.name))
            }
            RWhat::Vehicle { v, checks } => {
                let vs = &r.vehicles[*v].vehicle;
                let s = &vs.state;
                let [roll, pitch, yaw] = s.attitude.euler().map(f64::to_degrees);
                let mut ok = true;
                let mut parts = Vec::new();
                for (q, range) in checks {
                    let x = match q {
                        VehicleQty::Depth => s.position[2],
                        VehicleQty::North => s.position[0],
                        VehicleQty::East => s.position[1],
                        VehicleQty::Speed => sim_physics::math::norm(s.velocity),
                        VehicleQty::Roll => roll,
                        VehicleQty::Pitch => pitch,
                        VehicleQty::Yaw => yaw,
                    };
                    ok &= range.contains(x);
                    parts.push(format!("{} {x:.3}", q.name()));
                }
                (ok, format!("vehicle {}: {} at {now}", vs.name, parts.join(", ")))
            }
            RWhat::Resets { board, r: range } => {
                let n = w.resets(*board);
                let times: Vec<String> = w.reset_times[*board].iter().map(|t| t.to_string()).collect();
                let at = if times.is_empty() { String::new() } else { format!(" (at {})", times.join(", ")) };
                (range.contains(n as f64), format!("{}: {n} reset(s) by {now}{at}", r.names[*board]))
            }
            RWhat::Lockups { board, r: range } => {
                let n = r.boards[*board].lockups;
                (range.contains(n as f64), format!("{}: {n} lockup(s) by {now}", r.names[*board]))
            }
            _ => unreachable!("not a state"),
        }
    }
}

fn judge(what: &RWhat, bridge: &str, t: Time, msg: &DeviceToHost) -> Item {
    match what {
        RWhat::Status { check, names, raised, clear, .. } => match msg {
            DeviceToHost::FirmwareStatus(s) => {
                let r = check_status(s, check, names, *raised, *clear);
                let text = status_text(s, names);
                let evidence = match &r {
                    Ok(()) => format!("[{t}] {bridge} status: {text}"),
                    Err(e) => format!("[{t}] {bridge} status: {text}: {e}"),
                };
                Item { t, relevant: true, ok: r.is_ok(), evidence }
            }
            _ => Item { t, relevant: false, ok: false, evidence: String::new() },
        },
        RWhat::Ivc { byte, .. } => match sim_host::mock_bridge::ivc_received(msg) {
            Some(b) => Item {
                t,
                relevant: true,
                ok: byte.is_none_or(|x| x == b),
                evidence: format!("[{t}] {bridge}: IVC received {b:#04x}"),
            },
            None => Item { t, relevant: false, ok: false, evidence: String::new() },
        },
        RWhat::Message { path, value, .. } => {
            let j = serde_json::to_value(msg).unwrap_or_default();
            match json_at(&j, path) {
                Some(v) => {
                    let ok = match value {
                        None => true,
                        Some(ValueCheck::Range(r)) => v.as_f64().is_some_and(|x| r.contains(x)),
                        Some(ValueCheck::Equals(e)) => v == e,
                    };
                    Item {
                        t,
                        relevant: true,
                        ok,
                        evidence: format!("[{t}] {bridge}: {} = {}", path.join("."), compact(v)),
                    }
                }
                None => Item { t, relevant: false, ok: false, evidence: String::new() },
            }
        }
        _ => unreachable!(),
    }
}

fn judge_topic(what: &RWhat, agent: &str, m: &sim_host::host_protocol::HostMessage) -> Item {
    let RWhat::Topic { topic, msg, .. } = what else { unreachable!() };
    if m.topic != *topic {
        return Item { t: m.t, relevant: false, ok: false, evidence: String::new() };
    }
    let bad = msg.as_ref().and_then(|p| msg_mismatch(p, &m.msg));
    let evidence = match &bad {
        None => format!("[{}] {agent} {topic}: {}", m.t, compact(&m.msg)),
        Some(b) => format!("[{}] {agent} {topic}: {b}: {}", m.t, compact(&m.msg)),
    };
    Item { t: m.t, relevant: true, ok: bad.is_none(), evidence }
}

/// The last lines of a board up to `t`, for a failure.
fn tail(w: &World, board: usize, t: Time, n: usize) -> Vec<String> {
    let st = w.store.lock().unwrap();
    let lines = &st.lines[board];
    let end = lines.partition_point(|l| l.t <= t);
    let mut out = vec![format!("last lines of {} by {t}:", w.robot.names[board])];
    if end == 0 {
        out.push("  (none)".into());
    }
    out.extend(lines[end.saturating_sub(n)..end].iter().map(|l| format!("  [{}] {}", l.t, l.display)));
    out
}

/// What an occurrence condition that found nothing had instead.
fn nothing_found(w: &World, a: &Active, to: Time) -> Vec<String> {
    let window = format!("{} to {to}", a.from);
    match &a.what {
        RWhat::Log { board, .. } => {
            let mut v = vec![format!("no matching line from {window}")];
            v.extend(tail(w, *board, to, 8));
            v
        }
        RWhat::Frame { bus, .. } => {
            let st = w.store.lock().unwrap();
            let n = st.frames[*bus].iter().filter(|r| r.end >= a.from && r.end <= to).count();
            vec![format!("no matching frame from {window} ({n} other frame(s) on {})", w.robot.bus_names[*bus])]
        }
        RWhat::Status { bridge, .. } | RWhat::Ivc { bridge, .. } | RWhat::Message { bridge, .. } => {
            let name = w.bridge_name(*bridge);
            let mut v = vec![match (&a.what, &a.last_bad) {
                (_, Some(b)) => format!("none matched from {window}; the last: {b}"),
                (RWhat::Status { .. }, None) => format!("no FirmwareStatus at {name} from {window}"),
                (RWhat::Ivc { .. }, None) => format!("no IVC byte at {name} from {window}"),
                _ => format!("no such message at {name} from {window}"),
            }];
            if let Some(i) = w.bridge_board(*bridge) {
                v.extend(tail(w, i, to, 5));
            }
            v
        }
        RWhat::Topic { agent, topic, .. } => {
            let name = w.agent_name(*agent);
            let host = w.host(*agent);
            let mut v = vec![match &a.last_bad {
                Some(b) => format!("none matched from {window}; the last: {b}"),
                None => {
                    format!("no {topic} message at {name} from {window} ({} message(s) in all)", host.message_count())
                }
            }];
            v.extend(host.errors().into_iter().take(3).map(|e| format!("agent: {e}")));
            let board = match agent {
                AgentRef::Agent(k) => w.agents[*k].0.board,
                AgentRef::Can(k) => w.bridge_board(BridgeRef::Can(*k)),
            };
            if let Some(i) = board {
                v.extend(tail(w, i, to, 5));
            }
            v
        }
        _ => Vec::new(),
    }
}

/// Where a step stands after an evaluation.
enum Verdict {
    Pending,
    Pass(Time, Vec<String>),
    Fail(Time, Vec<String>),
}

impl Active {
    /// Looks at what happened up to now (`final_` when the run is over).
    fn evaluate(&mut self, w: &World, final_: bool) -> Verdict {
        let now = w.now();
        if now < self.from && !final_ {
            return Verdict::Pending;
        }
        let to = self.to;
        let closed = now >= to || final_;
        let horizon = now.min(to);
        if self.what.is_state() {
            if now > to {
                // Past the window (a final evaluation at the end): judged on what was sampled
                return match self.mode {
                    Mode::Expect => Verdict::Fail(to, vec![self.last_bad.clone().unwrap_or_default()]),
                    _ => Verdict::Pass(to, self.first.clone()),
                };
            }
            let (ok, text) = self.sample(w);
            return match self.mode {
                Mode::Expect if ok => Verdict::Pass(now, vec![text]),
                Mode::Never if ok => Verdict::Fail(now, vec![text]),
                Mode::Always if !ok => Verdict::Fail(now, vec![text]),
                Mode::Expect if closed => Verdict::Fail(now, vec![text]),
                Mode::Expect => {
                    self.last_bad = Some(text);
                    Verdict::Pending
                }
                _ if closed => Verdict::Pass(now, self.first.clone()),
                _ => {
                    if self.first.is_empty() {
                        self.first.push(format!("first sample: {text}"));
                    }
                    Verdict::Pending
                }
            };
        }
        if self.latest {
            if now < to && !final_ {
                return Verdict::Pending;
            }
            return match self.latest(w, to) {
                Some(i) if to.saturating_sub(i.t) > self.max_age => Verdict::Fail(
                    to,
                    vec![format!("the latest is too old (more than {}): {}", self.max_age, i.evidence)],
                ),
                Some(i) if i.ok => Verdict::Pass(to, vec![i.evidence]),
                Some(i) => Verdict::Fail(to, vec![i.evidence]),
                None => Verdict::Fail(to, nothing_found(w, self, to)),
            };
        }
        let counting = self.count.is_some() || self.rate.is_some();
        let mut hit: Option<Item> = None;
        let mut bad: Option<Item> = None;
        let mode = self.mode;
        let mut counted = self.counted;
        let mut seen = self.seen;
        let mut first = std::mem::take(&mut self.first);
        let mut last_bad = self.last_bad.take();
        self.scan(w, horizon, &mut |i| {
            if !i.relevant {
                return;
            }
            seen += 1;
            if i.ok {
                counted += 1;
                if first.len() < 3 {
                    first.push(i.evidence.clone());
                }
            } else if !i.evidence.is_empty() {
                last_bad = Some(i.evidence.clone());
            }
            match mode {
                Mode::Expect | Mode::Never if i.ok && hit.is_none() => hit = Some(i),
                Mode::Always if !i.ok && bad.is_none() => bad = Some(i),
                _ => {}
            }
        });
        self.counted = counted;
        self.seen = seen;
        self.first = first;
        self.last_bad = last_bad;
        match mode {
            Mode::Expect if counting => {
                if !closed {
                    return Verdict::Pending;
                }
                if now < to {
                    return Verdict::Fail(
                        now,
                        vec![format!(
                            "{} matching by {now}, but the run ended before the window closed at {to}",
                            self.counted
                        )],
                    );
                }
                let secs = to.saturating_sub(self.from).as_secs_f64();
                let rate = if secs > 0.0 { self.counted as f64 / secs } else { 0.0 };
                let mut ok = true;
                let mut what = vec![format!("{} matching from {} to {to}", self.counted, self.from)];
                if let Some(c) = self.count {
                    ok &= c.contains(self.counted as f64);
                    what[0] += &format!(" (count {c})");
                }
                if let Some(r) = self.rate {
                    ok &= r.contains(rate);
                    what[0] += &format!(", {rate:.2}/s (rate {r})");
                }
                what.extend(self.first.iter().cloned());
                if !ok && self.counted == 0 {
                    what.extend(nothing_found(w, self, to).into_iter().skip(1));
                }
                if ok { Verdict::Pass(to, what) } else { Verdict::Fail(to, what) }
            }
            Mode::Expect => match hit {
                Some(i) => Verdict::Pass(i.t, vec![i.evidence]),
                None if closed => Verdict::Fail(to.min(now.max(self.from)), nothing_found(w, self, to)),
                None => Verdict::Pending,
            },
            Mode::Never => match hit {
                Some(i) => Verdict::Fail(i.t, vec![i.evidence]),
                None if closed => Verdict::Pass(horizon, vec![format!("none from {} to {horizon}", self.from)]),
                None => Verdict::Pending,
            },
            Mode::Always => match bad {
                Some(i) => Verdict::Fail(i.t, vec![i.evidence]),
                None if closed && self.seen == 0 => Verdict::Fail(horizon, nothing_found(w, self, horizon)),
                None if closed => {
                    Verdict::Pass(horizon, vec![format!("{} from {} to {horizon}, all matching", self.seen, self.from)])
                }
                None => Verdict::Pending,
            },
        }
    }
}

/// A demangled symbol cut short the way the dashboard shows it: paths to their last segment, closures and
/// turbofish dropped (`<embassy_executor::platform::thread::Executor>::run::<x::__cortex_m_rt_main::{closure#0}>`
/// is `Executor::run<main>`).
pub fn short_symbol(name: &str) -> String {
    use std::sync::OnceLock;
    static RE: OnceLock<[regex::Regex; 3]> = OnceLock::new();
    let [closure, path, generic] = RE.get_or_init(|| {
        [
            regex::Regex::new(r"::\{closure#\d+\}").unwrap(),
            regex::Regex::new(r"\b(?:[A-Za-z_][A-Za-z0-9_]*::)+([A-Za-z_])").unwrap(),
            regex::Regex::new(r"<([^<>]*)>").unwrap(),
        ]
    });
    let mut s = closure.replace_all(name, "").into_owned();
    s = path.replace_all(&s, "$1").replace("::<", "<");
    if s.starts_with('<') {
        let mut depth = 0;
        let mut end = None;
        for (i, c) in s.char_indices() {
            match c {
                '<' => depth += 1,
                '>' => {
                    depth -= 1;
                    if depth == 0 {
                        end = Some(i);
                        break;
                    }
                }
                _ => {}
            }
        }
        if let Some(i) = end
            && s[i + 1..].starts_with("::")
        {
            let inner = &s[1..i];
            let inner = inner.rsplit_once(" as ").map_or(inner, |x| x.1);
            s = format!("{inner}{}", &s[i + 1..]);
        }
    }
    s = generic
        .replace_all(&s, |c: &regex::Captures| {
            let a = c[1].trim_start_matches('_').replace("_task_task", "_task");
            let a = if a == "cortex_m_rt_main" { "main".to_string() } else { a };
            format!("<{a}>")
        })
        .into_owned();
    if s.is_empty() { name.to_string() } else { s }
}

// ---- Describing steps

/// A time as a scenario writes it: `15s`, `1.5s`, `200ms`, `50us`.
pub fn short_time(t: Time) -> String {
    let ns = t.as_ns();
    let trim = |v: f64| {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    };
    if ns == 0 {
        "0s".into()
    } else if ns >= 1_000_000_000 {
        format!("{}s", trim(ns as f64 / 1e9))
    } else if ns >= 1_000_000 {
        format!("{}ms", trim(ns as f64 / 1e6))
    } else {
        format!("{}us", trim(ns as f64 / 1e3))
    }
}

fn timing_text(st: &Step) -> String {
    let mut parts = Vec::new();
    match st.start {
        Start::Cursor => {}
        Start::At(t) => parts.push(format!("at {}", short_time(t))),
        Start::After(d) => parts.push(format!("after {}", short_time(d))),
    }
    match st.end {
        Some(End::Within(d)) => parts.push(format!("within {}", short_time(d))),
        Some(End::Until(t)) => parts.push(format!("until {}", short_time(t))),
        None => {}
    }
    parts.join(" ")
}

fn what_text(c: &Condition) -> String {
    let w = match &c.what {
        What::Log { text, level } => match level {
            Some(l) => format!("log {text} ({l})"),
            None => format!("log {text}"),
        },
        What::Status(s) => {
            let mut p = Vec::new();
            if let Some(f) = s.faults {
                p.push(format!("faults={f:#x}"));
            }
            let refs = |v: &[FaultRef]| v.iter().map(|f| f.to_string()).collect::<Vec<_>>().join(",");
            if !s.raised.is_empty() {
                p.push(format!("raised={}", refs(&s.raised)));
            }
            if !s.clear.is_empty() {
                p.push(format!("clear={}", refs(&s.clear)));
            }
            if let Some(k) = s.killed {
                p.push(format!("killed={k}"));
            }
            if let Some(k) = s.kill_timed_out {
                p.push(format!("kill_timed_out={k}"));
            }
            if let Some(u) = s.uptime {
                p.push(format!("uptime {u} s"));
            }
            if let Some(n) = &s.board_name {
                p.push(format!("board_name={n}"));
            }
            if let Some(n) = s.client_id {
                p.push(format!("client_id={n}"));
            }
            if p.is_empty() { "status".into() } else { format!("status {}", p.join(" ")) }
        }
        What::Link(up) => format!("link {}", if *up { "up" } else { "down" }),
        What::Frame(m) => match &c.bus {
            Some(b) => format!("{m} on {b}"),
            None => m.to_string(),
        },
        What::Gpio { pin, high } => format!("{pin} {}", if *high { "high" } else { "low" }),
        What::Esc { name, rpm } => format!("{} at {rpm} rpm", name.as_deref().unwrap_or("ESC")),
        What::Vehicle { name, checks } => {
            let c: Vec<String> = checks.iter().map(|(q, r)| format!("{} {r}", q.name())).collect();
            format!("vehicle {}{}", name.as_deref().map(|n| format!("{n} ")).unwrap_or_default(), c.join(", "))
        }
        What::Resets(r) => format!("resets {r}"),
        What::Lockups(r) => format!("lockups {r}"),
        What::Ivc(Some(b)) => format!("IVC byte {b:#04x}"),
        What::Ivc(None) => "an IVC byte".into(),
        What::Message { path, value } => match value {
            None => format!("message {}", path.join(".")),
            Some(ValueCheck::Range(r)) => format!("message {} in {r}", path.join(".")),
            Some(ValueCheck::Equals(v)) => format!("message {} = {v}", path.join(".")),
        },
        What::Topic { topic, msg: None } => format!("topic {topic}"),
        What::Topic { topic, msg: Some(m) } => format!("topic {topic} {}", compact(m)),
    };
    let mut w = w;
    if let Some(c) = c.count {
        w += &format!(" count {c}");
    }
    if let Some(r) = c.rate {
        w += &format!(" rate {r}/s");
    }
    w
}

/// A one-line description of a step: `expect esc0 log "ROS connected" within 3s`.
pub fn describe(st: &Step) -> String {
    if let Some(l) = &st.label {
        return l.clone();
    }
    let body = match &st.kind {
        StepKind::Fault { board: Some(b), spec, .. } => format!("{b}: {spec}"),
        StepKind::Fault { board: None, spec, .. } => spec.clone(),
        StepKind::Input(v) => v.iter().map(|(k, x)| format!("{k} = {x}")).collect::<Vec<_>>().join(", "),
        StepKind::Send { target, message, every } => {
            let m = format!("{message:?}");
            let m = if m.len() > 70 { format!("{}...", &m[..70]) } else { m };
            match every {
                Some(e) => format!("{target} {m} every {}", short_time(*e)),
                None => format!("{target} {m}"),
            }
        }
        StepKind::SendBytes { target, bytes } => {
            let hex: String = bytes.iter().take(24).map(|b| format!("{b:02x}")).collect();
            format!("{target} bytes {hex}{}", if bytes.len() > 24 { "..." } else { "" })
        }
        StepKind::CanSend { target, frames, spacing, repeat } => {
            let first = sim_host::canlog::compact(&frames[0]);
            let more = if frames.len() > 1 { format!(" and {} more", frames.len() - 1) } else { String::new() };
            let rounds = if *repeat > 1 { format!(" x{repeat}") } else { String::new() };
            let gap = if *spacing > Time::ZERO { format!(" {} apart", short_time(*spacing)) } else { String::new() };
            format!("{target} {first}{more}{rounds}{gap}")
        }
        StepKind::SendTopic { target, topic, msg, every } => {
            let m = if msg.is_null() { String::new() } else { format!(" {}", compact(msg)) };
            let who =
                if target.board.is_none() && target.bridge.is_none() { String::new() } else { format!("{target} ") };
            match every {
                Some(e) => format!("{who}{topic}{m} every {}", short_time(*e)),
                None => format!("{who}{topic}{m}"),
            }
        }
        StepKind::IvcSend { board, byte } => format!("{board} {byte:#04x}"),
        StepKind::Pause(t) if t.is_empty() => String::new(),
        StepKind::Pause(t) => format!("{t:?}"),
        StepKind::Expect(c) | StepKind::Never(c) | StepKind::Always(c) => {
            let who = c.target.to_string();
            if who.is_empty() { what_text(c) } else { format!("{who} {}", what_text(c)) }
        }
    };
    let timing = timing_text(st);
    let timing = match (&st.kind, st.start, st.end) {
        (StepKind::Never(_) | StepKind::Always(_), Start::Cursor, None) => "over the whole run".to_string(),
        _ => timing,
    };
    [st.kind.word().to_string(), body, timing].into_iter().filter(|s| !s.is_empty()).collect::<Vec<_>>().join(" ")
}

// ---- Setting up

/// The robot file (edited) or the one-board robot a scenario runs, and the directory its paths are relative to.
fn robot_spec(s: &Scenario) -> R<(RobotSpec, PathBuf)> {
    match &s.setup {
        Setup::Robot { file, boards } => {
            let text = std::fs::read_to_string(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let mut spec: RobotSpec = toml::from_str(&text).map_err(|e| format!("{}: {e}", file.display()))?;
            if let Some(keep) = boards {
                for b in keep {
                    if !spec.board.iter().any(|x| &x.name == b) {
                        let all: Vec<&str> = spec.board.iter().map(|b| b.name.as_str()).collect();
                        return Err(format!("robot.boards: no board {b:?} in {} ({})", file.display(), all.join(", ")));
                    }
                }
                spec.board.retain(|b| keep.contains(&b.name));
                // Vehicle wiring to boards left out goes too
                for v in &mut spec.vehicle {
                    v.thrusters.retain(|t| keep.contains(&t.board));
                    v.sensor.retain(|t| keep.contains(&t.board));
                    v.transducer.retain(|t| keep.contains(&t.board));
                }
            }
            Ok((spec, file.parent().unwrap_or(Path::new(".")).to_path_buf()))
        }
        Setup::Board { name, image, file, rom_boot } => {
            let file = file.as_ref().ok_or("board: needs a `file` (the board file)")?;
            let board = sim_host::board::Board::load(file)?;
            let toml = format!(
                "{}[[board]]\nname = {name:?}\nboard = {:?}\nelf = {:?}\nrom_boot = {rom_boot}\n",
                if board.can.is_some() { "[[bus]]\nname = \"can\"\n" } else { "" },
                file.to_string_lossy(),
                image.to_string_lossy(),
            );
            let spec: RobotSpec = toml::from_str(&toml).map_err(|e| format!("board: {e}"))?;
            Ok((spec, PathBuf::from(".")))
        }
    }
}

/// The image each board of `spec` runs.
fn images_of(spec: &RobotSpec, dir: &Path, overrides: &[ImageOverride]) -> R<Vec<(String, PathBuf)>> {
    spec.board
        .iter()
        .map(|b| {
            let p = sim_host::resolve_image(&dir.join(sim_host::expand_path(&b.elf.to_string_lossy())?));
            let p = override_image(overrides, &b.name, &p).unwrap_or(p);
            Ok((b.name.clone(), p))
        })
        .collect()
}

fn build(s: &Scenario, opts: &Options, spec: &RobotSpec, dir: &Path) -> R<World> {
    let mut robot =
        Robot::from_spec_with_images(spec, dir, &|name, p| override_image(&opts.images, name, &p).unwrap_or(p))?;
    robot.threads = s.threads;
    let n = robot.boards.len();
    // The CANmore client each board runs as: two boards choose theirs at run time (the ESC board from its ID
    // strap, the smart battery from the serial number in its flash), which titan_bridge's profiles know
    if let Ok(plan) = sim_host::bridges::plan(spec, dir, &[], None) {
        for (i, cmd) in plan.iter().enumerate() {
            if let Some(c) = cmd.client_id
                && i < n
            {
                robot.client_ids[i] = Some(c);
            }
        }
    }
    let mut nodes = vec![Vec::new(); robot.buses.len()];
    for i in 0..n {
        if let Some(j) = robot.board_bus[i] {
            let k = robot.boards[i].machine.can_nodes().len();
            nodes[j].extend(std::iter::repeat_n(i, k));
        }
    }
    let fault_names = robot
        .boards
        .iter()
        .map(|b| {
            let mut names = b.machine.image.as_ref().map(|i| sim_host::dashboard::fault_names(i)).unwrap_or_default();
            for (k, std) in STANDARD_FAULTS.iter().enumerate() {
                if names.len() <= k {
                    names.push(std.to_string());
                }
            }
            names
        })
        .collect();
    let mut w = World {
        robot,
        can: Vec::new(),
        eth: Vec::new(),
        agents: Vec::new(),
        store: Default::default(),
        nodes,
        fault_names,
        reset_base: vec![0; n],
        reset_times: vec![Vec::new(); n],
        last_resets: vec![0; n],
        stopped: Vec::new(),
        escs: Vec::new(),
        next_esc_sample: Time::ZERO,
    };
    let add_can = |w: &mut World, j: usize, client: u8| {
        if w.can.iter().any(|(jj, c, _)| *jj == j && *c == client) {
            return;
        }
        let b = MockBridge::new(client, w.robot.buses[j].bus.rates.data.is_some(), Time::ZERO);
        b.set_kill(Some(false));
        w.robot.buses[j].endpoints.push(Box::new(b.clone()));
        w.can.push((j, client, b));
    };
    let add_eth = |w: &mut World, i: usize| -> R<()> {
        if w.robot.boards[i].machine.sys.ethernet.is_none() {
            return Err(format!("{} has no Ethernet", w.robot.names[i]));
        }
        if !w.eth.iter().any(|(ii, _)| *ii == i) {
            let b = PuddlesBridge::new(Time::ZERO);
            b.set_kill(Some(false));
            w.robot.boards[i].eth_endpoints.push(Box::new(b.clone()));
            w.eth.push((i, b));
        }
        Ok(())
    };
    match &s.bridges {
        Bridges::All => {
            for i in 0..n {
                match (w.robot.board_bus[i], w.robot.client_ids[i]) {
                    (Some(j), Some(c)) => add_can(&mut w, j, c),
                    _ if w.robot.boards[i].machine.sys.ethernet.is_some() => add_eth(&mut w, i)?,
                    _ => {}
                }
            }
        }
        Bridges::List(list) => {
            for b in list {
                match b {
                    BridgeSpec::Can { bus, client } => {
                        let j = match bus {
                            Some(name) => w.bus(name)?,
                            None => w.only_bus().map_err(|e| format!("bridges: {b}: {e}"))?,
                        };
                        add_can(&mut w, j, *client);
                    }
                    BridgeSpec::Eth { board } => {
                        let i = w.board(board)?;
                        add_eth(&mut w, i)?;
                    }
                }
            }
        }
    }
    for spec in &s.agents {
        if spec.protocol == sim_host::host_protocol::Protocol::TitanLink {
            let p = sim_host::host_protocol::place(&w.robot, spec).map_err(|e| format!("agents: {e}"))?;
            add_can(&mut w, p.bus, p.client);
            continue;
        }
        let (p, agent) =
            sim_host::host_protocol::attach(&mut w.robot, spec, dir).map_err(|e| format!("agents: {e}"))?;
        w.agents.push((p, agent));
    }
    for (k, v) in &s.inputs {
        set_input(&w, k, *v).map_err(|e| format!("inputs: {e}"))?;
    }
    if opts.record {
        for i in 0..n {
            for (k, d) in w.robot.boards[i].machine.sys.pin_devices.iter().enumerate() {
                if let Some(e) = d.as_any().downcast_ref::<sim_devices::dshot::Esc>() {
                    w.escs.push((i, k, e.name.clone(), Vec::new()));
                }
            }
        }
    }
    if opts.coverage {
        for b in w.robot.boards.iter_mut() {
            b.machine.enable_coverage();
        }
    }
    if let Some(interval) = opts.profile {
        for b in &mut w.robot.boards {
            b.start_profile(interval);
        }
    }
    w.install_hooks();
    Ok(w)
}

/// Sets `BOARD.INPUT` (or `INPUT` with one board).
fn set_input(w: &World, key: &str, v: f64) -> R<()> {
    let r = &w.robot;
    let (i, name) = match key.split_once('.') {
        Some((b, rest)) if r.index(b).is_some() => (r.index(b).unwrap(), rest),
        _ if r.boards.len() == 1 => (0, key),
        _ => return Err(format!("{key}: write BOARD.INPUT (the boards: {})", r.names.join(", "))),
    };
    let inputs = &r.inputs[i];
    if !inputs.0.contains_key(name) {
        let all: Vec<&str> = inputs.0.keys().map(String::as_str).collect();
        return Err(format!("{}: no input {name:?} (its inputs: {})", r.names[i], all.join(", ")));
    }
    inputs.set(name, v);
    Ok(())
}

// ---- The timeline

#[derive(Clone)]
struct Timeline {
    results: Vec<StepResult>,
    /// The expect being waited for.
    pending: Option<Active>,
    watchers: Vec<Active>,
    /// Messages still to send: when, through which bridge.
    sends: Vec<(Time, BridgeRef, HostToDevice)>,
    cursor: Time,
    end: Time,
    /// Steps watched from the start ([`Engine::watch_early`]).
    early: Vec<usize>,
}

/// The engine: a world and a timeline.
struct Engine {
    w: World,
    tl: Timeline,
    record: bool,
}

impl Engine {
    fn new(w: World, s: &Scenario, opts: &Options) -> Engine {
        Engine {
            w,
            tl: Timeline {
                results: Vec::new(),
                pending: None,
                watchers: Vec::new(),
                sends: Vec::new(),
                cursor: Time::ZERO,
                end: s.time,
                early: Vec::new(),
            },
            record: opts.record,
        }
    }

    /// Runs `rest` of the steps, then the cases (each from a copy), as [`run`] does after the common steps.
    fn run_rest(
        &mut self,
        s: &Scenario,
        rest: &[Step],
        images: &[(String, PathBuf)],
        wall: std::time::Instant,
    ) -> Vec<Outcome> {
        let done = self.run_steps(rest);
        if s.cases.is_empty() || !done {
            self.finish();
            if !done {
                for c in &s.cases {
                    for st in &c.steps {
                        let k = self.result(st);
                        self.tl.results[k].evidence.push("the common steps did not finish".into());
                    }
                }
            }
            return vec![self.outcome(s, None, images, wall)];
        }
        let mut out = Vec::new();
        for c in &s.cases {
            let started = std::time::Instant::now();
            let mut f = self.fork();
            f.run_steps(&c.steps);
            f.finish();
            let mut o = f.outcome(s, Some(&c.name), images, started);
            o.wall += wall.elapsed() - started.elapsed();
            out.push(o);
        }
        out
    }

    fn fork(&self) -> Engine {
        Engine { w: self.w.snapshot(), tl: self.tl.clone(), record: self.record }
    }

    fn result(&mut self, st: &Step) -> usize {
        self.tl.results.push(StepResult {
            number: st.number,
            kind: st.kind.word(),
            description: describe(st),
            status: Status::NotReached,
            time: None,
            evidence: Vec::new(),
        });
        self.tl.results.len() - 1
    }

    fn settle(&mut self, k: usize, v: Verdict) -> Option<Time> {
        let r = &mut self.tl.results[k];
        match v {
            Verdict::Pending => None,
            Verdict::Pass(t, e) => {
                r.status = Status::Pass;
                r.time = Some(t);
                r.evidence = e;
                Some(t)
            }
            Verdict::Fail(t, e) => {
                r.status = Status::Fail;
                r.time = Some(t);
                r.evidence = e;
                Some(t)
            }
        }
    }

    fn start_of(&self, st: &Step) -> Time {
        match st.start {
            Start::Cursor => self.tl.cursor,
            Start::At(t) => t,
            Start::After(d) => self.tl.cursor + d,
        }
    }

    fn end_of(&self, st: &Step, start: Time) -> Option<Time> {
        st.end.map(|e| match e {
            End::Within(d) => start + d,
            End::Until(t) => t,
        })
    }

    fn activate(&mut self, st: &Step, mode: Mode, c: &Condition, k: usize) -> R<Active> {
        let what = resolve(&self.w, c)?;
        let start = self.start_of(st);
        let explicit_start = !matches!(st.start, Start::Cursor);
        let (from, to, latest) = match mode {
            Mode::Expect => match self.end_of(st, start) {
                Some(to) => (start, to, false),
                // At an instant: a state then, the latest of a value stream then, or an occurrence by then
                None if what.is_state() => (start, start, false),
                None if what.is_stream() => (start, start, true),
                None => (Time::ZERO, start, false),
            },
            // From `at` or `after`; with neither, over the whole run, or from the cursor when the window has an end
            _ => {
                let from = if explicit_start || st.end.is_some() { start } else { Time::ZERO };
                (from, self.end_of(st, start).unwrap_or(self.tl.end).max(from), false)
            }
        };
        let max_age = match &c.what {
            What::Status(s) => s.max_age.unwrap_or(Time::from_ms(2500)),
            _ => Time::MAX,
        };
        let instant = mode == Mode::Expect && st.end.is_none();
        Ok(Active {
            result: k,
            instant,
            mode,
            what,
            count: c.count,
            rate: c.rate,
            from,
            to,
            latest,
            max_age,
            next: None,
            counted: 0,
            seen: 0,
            first: Vec::new(),
            last_bad: None,
        })
    }

    fn apply(&mut self, st: &Step) -> Result<Vec<String>, String> {
        let now = self.w.now();
        match &st.kind {
            StepKind::Fault { board, spec, fault } => {
                let w = &mut self.w;
                let i = match board {
                    Some(b) => w.board(b)?,
                    None if w.robot.boards.len() == 1 => 0,
                    None if !matches!(fault, Fault::CanUnplugged(_))
                        && matches!(
                            fault,
                            Fault::CanErrorRate(_)
                                | Fault::CanCorrupt(_)
                                | Fault::CanStuckDominant(_)
                                | Fault::CanFlood { .. }
                                | Fault::CanFloodStop
                        ) =>
                    {
                        let j = w.only_bus()?;
                        (0..w.robot.boards.len())
                            .find(|&i| w.robot.board_bus[i] == Some(j))
                            .ok_or("no board on the bus")?
                    }
                    None => {
                        return Err(format!(
                            "which board? (fault: BOARD:{spec}; the boards: {})",
                            w.robot.names.join(", ")
                        ));
                    }
                };
                let bridge = w.board_bridge(i).or_else(|| {
                    w.robot.board_bus[i].and_then(|j| w.can.iter().position(|(jj, _, _)| *jj == j)).map(BridgeRef::Can)
                });
                if let Fault::Brownout { .. } = fault {
                    w.reset_base[i] = w.resets(i) + 1;
                }
                // Handles to the bridge (they share its state), so the robot can be borrowed on its own
                let (can, eth) = match bridge {
                    Some(BridgeRef::Can(k)) => (Some(w.can[k].2.clone()), None),
                    Some(BridgeRef::Eth(k)) => (None, Some(w.eth[k].1.clone())),
                    None => (None, None),
                };
                let ctl: Option<&dyn BridgeControl> = match (&can, &eth) {
                    (Some(b), _) => Some(b),
                    (_, Some(b)) => Some(b),
                    _ => None,
                };
                let r = sim_host::fault::robot_target(&mut w.robot, i, ctl).apply(fault);
                r.map(|_| vec![format!("applied to {}", w.robot.names[i])])
            }
            StepKind::Input(list) => {
                for (k, v) in list {
                    set_input(&self.w, k, *v)?;
                }
                Ok(Vec::new())
            }
            StepKind::Send { target, message, every } => {
                let b = self.w.target_bridge(target)?;
                match every {
                    None => self.tl.sends.push((now, b, (**message).clone())),
                    Some(e) => {
                        let start = self.start_of(st);
                        let until = self.end_of(st, start).unwrap_or(start);
                        let mut t = start.max(now);
                        let mut n = 0;
                        while t <= until {
                            self.tl.sends.push((self.w.grid(t), b, (**message).clone()));
                            t += *e;
                            n += 1;
                        }
                        self.tl.sends.sort_by_key(|s| s.0);
                        return Ok(vec![format!("{n} message(s) from {} to {until}", start.max(now))]);
                    }
                }
                self.tl.sends.sort_by_key(|s| s.0);
                Ok(Vec::new())
            }
            StepKind::SendBytes { target, bytes } => {
                match self.w.target_bridge(target)? {
                    BridgeRef::Can(k) => self.w.can[k].2.send_encoded(bytes),
                    BridgeRef::Eth(k) => self.w.eth[k].1.send_encoded(bytes),
                }
                Ok(Vec::new())
            }
            StepKind::CanSend { target, frames, spacing, repeat } => {
                let BridgeRef::Can(k) = self.w.target_bridge(target)? else {
                    return Err("can_send: the bridge is on an Ethernet cable, not a CAN bus".into());
                };
                let bridge = &self.w.can[k].2;
                let mut t = now;
                for _ in 0..*repeat {
                    for f in frames {
                        bridge.send_frame_at(t, f.clone());
                        t += *spacing;
                    }
                }
                let n = frames.len() * *repeat as usize;
                Ok(vec![format!("{n} frame(s) from {now} to {}", t - *spacing)])
            }
            StepKind::SendTopic { target, topic, msg, every } => {
                let a = self.w.target_agent(target)?;
                let host = self.w.host(a);
                let times = match every {
                    None => vec![now],
                    Some(e) => {
                        let start = self.start_of(st);
                        let until = self.end_of(st, start).unwrap_or(start);
                        let mut t = start.max(now);
                        let mut v = Vec::new();
                        while t <= until {
                            v.push(self.w.grid(t));
                            t += *e;
                        }
                        v
                    }
                };
                for t in &times {
                    host.send(topic, msg, Some(*t))?;
                }
                Ok(match every {
                    Some(_) => vec![format!("{} message(s) from {}", times.len(), self.w.agent_name(a))],
                    None => Vec::new(),
                })
            }
            StepKind::IvcSend { board, byte } => {
                let i = self.w.board(board)?;
                let modem = IvcModem::from_image(&self.w.robot.images[i]).ok_or_else(|| {
                    format!("{board} runs {}, not an IVC image (ivc_board or ivc_board2)", self.w.robot.images[i])
                })?;
                let b = self
                    .w
                    .board_bridge(i)
                    .ok_or_else(|| format!("{board} has no mock bridge (add it to `bridges`)"))?;
                self.w.send(b, modem.transmit(*byte));
                Ok(Vec::new())
            }
            StepKind::Pause(_) => Ok(self.checkpoint()),
            _ => unreachable!(),
        }
    }

    /// The state of every board and bus, for a `pause`.
    fn checkpoint(&self) -> Vec<String> {
        let w = &self.w;
        let r = &w.robot;
        let mut out = Vec::new();
        for (i, s) in r.boards.iter().enumerate() {
            let m = &s.machine;
            let cores: Vec<String> = (0..2)
                .map(|c| {
                    let cpu = &m.cores[c];
                    if !r.power_on[i].1 || !m.core_enabled[c] {
                        "off".into()
                    } else if cpu.is_locked_up() {
                        "locked up".into()
                    } else {
                        let pc = cpu.reg(15);
                        let f = m.symbolize(pc).map_or_else(
                            || format!("{pc:#010x}"),
                            |f| short_symbol(&sim_host::demangle::demangle(f.split('+').next().unwrap_or(&f))),
                        );
                        if cpu.is_sleeping() { format!("asleep in {f}") } else { format!("in {f}") }
                    }
                })
                .collect();
            let status = w.board_bridge(i).and_then(|b| {
                w.with_received(b, |rx| {
                    rx.iter().rev().find_map(|(t, m)| match m {
                        DeviceToHost::FirmwareStatus(st) => {
                            Some(format!("{t}: {}", status_text(st, &w.fault_names[i])))
                        }
                        _ => None,
                    })
                })
            });
            out.push(format!(
                "{}: {} resets, core0 {}, core1 {}; status {}",
                r.names[i],
                w.resets(i),
                cores[0],
                cores[1],
                status.unwrap_or_else(|| "none".into())
            ));
            let st = w.store.lock().unwrap();
            if let Some(l) = st.lines[i].last() {
                out.push(format!("  last line [{}] {}", l.t, l.display));
            }
        }
        let st = w.store.lock().unwrap();
        for (j, name) in r.bus_names.iter().enumerate() {
            out.push(format!("bus {name}: {} frame(s) so far", st.frames[j].len()));
        }
        out
    }

    /// Takes the steps in turn as far as virtual time allows; true once every one has been taken.
    fn advance(&mut self, steps: &[Step], idx: &mut usize) -> bool {
        loop {
            if let Some(mut a) = self.tl.pending.take() {
                let k = a.result;
                match a.evaluate(&self.w, false) {
                    Verdict::Pending => {
                        self.tl.pending = Some(a);
                        return false;
                    }
                    v => {
                        let t = self.settle(k, v).unwrap();
                        // An instant check moves the cursor to its time, a window to when it was decided
                        let at = if a.instant { a.to } else { t };
                        self.tl.cursor = self.tl.cursor.max(at).max(t);
                        *idx += 1;
                    }
                }
            }
            let Some(st) = steps.get(*idx) else { return true };
            if self.tl.early.contains(&st.number) {
                *idx += 1;
                continue;
            }
            let k = self.result(st);
            match &st.kind {
                kind if kind.is_action() => {
                    let at = self.w.grid(self.start_of(st));
                    if at > self.w.now() {
                        self.tl.results.pop();
                        return false;
                    }
                    let now = self.w.now();
                    let v = match self.apply(st) {
                        Ok(e) => Verdict::Pass(now, e),
                        Err(e) => Verdict::Fail(now, vec![e]),
                    };
                    self.settle(k, v);
                    self.tl.cursor = now;
                    *idx += 1;
                }
                StepKind::Expect(c) => match self.activate(st, Mode::Expect, c, k) {
                    Ok(a) => self.tl.pending = Some(a),
                    Err(e) => {
                        let now = self.w.now();
                        self.settle(k, Verdict::Fail(now, vec![e]));
                        *idx += 1;
                    }
                },
                StepKind::Never(c) | StepKind::Always(c) => {
                    let mode = if matches!(st.kind, StepKind::Never(_)) { Mode::Never } else { Mode::Always };
                    match self.activate(st, mode, c, k) {
                        Ok(a) => self.tl.watchers.push(a),
                        Err(e) => {
                            let now = self.w.now();
                            self.settle(k, Verdict::Fail(now, vec![e]));
                        }
                    }
                    *idx += 1;
                }
                _ => unreachable!(),
            }
        }
    }

    fn watch(&mut self, final_: bool) {
        let mut ws = std::mem::take(&mut self.tl.watchers);
        ws.retain_mut(|a| {
            let k = a.result;
            let v = a.evaluate(&self.w, final_);
            self.settle(k, v).is_none()
        });
        self.tl.watchers = ws;
    }

    fn send_due(&mut self) {
        let now = self.w.now();
        while self.tl.sends.first().is_some_and(|s| s.0 <= now) {
            let (_, b, m) = self.tl.sends.remove(0);
            self.w.send(b, m);
        }
    }

    /// The next time to stop at: an action, a window opening or closing, a send, a state sample, or the end.
    fn next_stop(&self, steps: &[Step], idx: usize) -> Time {
        let now = self.w.now();
        let mut c: Vec<Time> = vec![self.tl.end, now + Time::from_ms(10)];
        let sample = now + Time::from_ms(1);
        let mut conds: Vec<&Active> = self.tl.watchers.iter().collect();
        if let Some(p) = &self.tl.pending {
            conds.push(p);
        } else if let Some(st) = steps.get(idx)
            && st.kind.is_action()
        {
            c.push(self.start_of(st));
        }
        for a in conds {
            c.push(a.from);
            c.push(a.to);
            if a.what.is_state() && a.from <= sample {
                c.push(sample);
            }
        }
        if let Some(s) = self.tl.sends.first() {
            c.push(s.0);
        }
        c.into_iter().map(|t| self.w.grid(t)).filter(|t| *t > now).min().unwrap_or(self.tl.end).min(self.tl.end)
    }

    /// Starts watching the `never` and `always` steps whose window does not depend on the steps before them (at an
    /// absolute time, or over the whole run), so they are judged even if an earlier step holds the timeline up.
    fn watch_early(&mut self, steps: &[Step]) {
        for st in steps {
            let (StepKind::Never(c) | StepKind::Always(c)) = &st.kind else { continue };
            let independent = matches!(
                (st.start, st.end),
                (Start::At(_), None | Some(End::Within(_)) | Some(End::Until(_))) | (Start::Cursor, None)
            );
            if !independent {
                continue;
            }
            let k = self.result(st);
            let mode = if matches!(st.kind, StepKind::Never(_)) { Mode::Never } else { Mode::Always };
            match self.activate(st, mode, c, k) {
                Ok(a) => self.tl.watchers.push(a),
                Err(e) => {
                    self.settle(k, Verdict::Fail(Time::ZERO, vec![e]));
                }
            }
            self.tl.early.push(st.number);
        }
    }

    /// Runs `steps` until each is taken (then returns true), or to the end of the run.
    fn run_steps(&mut self, steps: &[Step]) -> bool {
        self.watch_early(steps);
        let mut idx = 0;
        loop {
            self.w.sample(self.record);
            self.send_due();
            let done = self.advance(steps, &mut idx);
            self.watch(false);
            if done {
                return true;
            }
            if self.w.now() >= self.tl.end {
                break;
            }
            let t = self.next_stop(steps, idx);
            self.w.run_to(t);
        }
        // The run ended first
        if let Some(mut a) = self.tl.pending.take() {
            let k = a.result;
            let v = match a.evaluate(&self.w, true) {
                Verdict::Pending => Verdict::Fail(self.w.now(), vec!["the run ended first".into()]),
                Verdict::Fail(t, mut e) if a.to > self.tl.end => {
                    e.insert(0, format!("the run ended at {} before the window closed at {}", self.tl.end, a.to));
                    Verdict::Fail(t, e)
                }
                v => v,
            };
            self.settle(k, v);
            idx += 1;
        }
        for st in &steps[idx.min(steps.len())..] {
            if self.tl.early.contains(&st.number) {
                continue;
            }
            let k = self.result(st);
            self.tl.results[k].evidence.push(format!("the run ended at {} first", self.tl.end));
        }
        false
    }

    /// Runs on to the end, watching, and closes every watcher.
    fn finish(&mut self) {
        while self.w.now() < self.tl.end {
            self.w.sample(self.record);
            self.send_due();
            self.watch(false);
            let t = self.next_stop(&[], 0);
            self.w.run_to(t);
        }
        self.w.sample(self.record);
        self.watch(true);
        self.tl.results.sort_by_key(|r| r.number);
    }

    fn outcome(
        &mut self,
        s: &Scenario,
        case: Option<&str>,
        images: &[(String, PathBuf)],
        wall: std::time::Instant,
    ) -> Outcome {
        for s in self.w.robot.boards.iter_mut() {
            s.flush_uarts();
        }
        let error = if self.w.stopped.is_empty() {
            None
        } else {
            let v: Vec<String> = self
                .w
                .stopped
                .iter()
                .map(|(t, i, why)| format!("{} stopped at {t}: {why}", self.w.robot.names[*i]))
                .collect();
            Some(v.join("; "))
        };
        let record = self.record.then(|| self.make_record(images));
        Outcome {
            case: case.map(str::to_string),
            steps: self.tl.results.clone(),
            error,
            end: self.w.now(),
            wall: wall.elapsed(),
            images: images.to_vec(),
            record,
            coverage: self.w.robot.boards.iter().any(|b| b.machine.coverage.is_some()).then(|| {
                let mut c = sim_host::coverage::Coverage::new();
                for (b, name) in self.w.robot.boards.iter().zip(&self.w.robot.images) {
                    c.add_machine(name, &b.machine);
                }
                c
            }),
            ..Outcome::empty(s)
        }
    }

    fn make_record(&self, images: &[(String, PathBuf)]) -> Record {
        let w = &self.w;
        let st = w.store.lock().unwrap();
        let boards = (0..w.robot.boards.len())
            .map(|i| {
                let image = images.get(i).map(|x| x.1.clone()).unwrap_or_default();
                let statuses = w
                    .board_bridge(i)
                    .map(|b| {
                        w.with_received(b, |rx| {
                            rx.iter()
                                .filter_map(|(t, m)| match m {
                                    DeviceToHost::FirmwareStatus(s) => Some((*t, s.faults)),
                                    _ => None,
                                })
                                .collect()
                        })
                    })
                    .unwrap_or_default();
                BoardRecord {
                    name: w.robot.names[i].clone(),
                    image_hash: file_hash(&image),
                    image,
                    lines: st.lines[i].clone(),
                    resets: w.reset_times[i].clone(),
                    lockups: w.robot.boards[i].lockups,
                    fault_names: w.fault_names[i].clone(),
                    statuses,
                    escs: w.escs.iter().filter(|e| e.0 == i).map(|e| (e.2.clone(), e.3.clone())).collect(),
                    profile: w.robot.boards[i].profiler.as_ref().map(|p| p.report()),
                }
            })
            .collect();
        let buses = w
            .robot
            .bus_names
            .iter()
            .enumerate()
            .map(|(j, n)| (n.clone(), sim_host::canlog::from_records(&st.frames[j], n)))
            .collect();
        Record { boards, buses, end: w.now() }
    }
}

/// Runs a scenario: one outcome, or one per case.
pub fn run(s: &Scenario, opts: &Options) -> Vec<Outcome> {
    let wall = std::time::Instant::now();
    let fail = |e: String| {
        let mut o = Outcome::empty(s);
        o.error = Some(e);
        o.wall = wall.elapsed();
        vec![o]
    };
    let (spec, dir) = match robot_spec(s) {
        Ok(x) => x,
        Err(e) => return fail(e),
    };
    let images = match images_of(&spec, &dir, &opts.images) {
        Ok(x) => x,
        // Images from a checkout that is not there (the C robots' `${TITAN_FIRMWARE_DIR}` unset): not built here
        Err(e) if e.contains(" is not set") => {
            let mut o = Outcome::empty(s);
            o.skipped = Some(e);
            o.wall = wall.elapsed();
            return vec![o];
        }
        Err(e) => return fail(e),
    };
    if let Some((b, p)) = images.iter().find(|(_, p)| !p.exists()) {
        let why = format!("{b}'s image {} is not built", p.display());
        if opts.skip_missing && !sim_host::titan_rs_required() {
            let mut o = Outcome::empty(s);
            o.skipped = Some(why);
            o.images = images;
            return vec![o];
        }
        return fail(why);
    }
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> R<Vec<Outcome>> {
        let shared = opts.from_checkpoint.as_deref().and_then(|name| shared_prefix(s, name).map(|p| (name, p)));
        let slot = shared.map(|(name, p)| {
            let key = format!(
                "{name}|{:?}|{:?}|{:?}|{:?}|{}|{:?}|{images:?}|{:?}",
                s.setup,
                s.bridges,
                s.inputs,
                s.time,
                s.threads,
                &s.steps[..=p],
                (opts.record, opts.profile, opts.coverage)
            );
            checkpoints().lock().unwrap().entry(key).or_default().clone()
        });
        if let (Some(slot), Some((_, p))) = (&slot, shared) {
            let mut kept = slot.lock().unwrap();
            if let Some(base) = &*kept {
                let mut e = base.fork();
                drop(kept);
                return Ok(e.run_rest(s, &s.steps[p + 1..], &images, wall));
            }
            let w = build(s, opts, &spec, &dir)?;
            let mut e = Engine::new(w, s, opts);
            if e.run_steps(&s.steps[..=p]) {
                *kept = Some(e.fork());
                drop(kept);
                return Ok(e.run_rest(s, &s.steps[p + 1..], &images, wall));
            }
            drop(kept);
            e.finish();
            return Ok(vec![e.outcome(s, None, &images, wall)]);
        }
        let w = build(s, opts, &spec, &dir)?;
        let mut e = Engine {
            w,
            tl: Timeline {
                results: Vec::new(),
                pending: None,
                watchers: Vec::new(),
                sends: Vec::new(),
                cursor: Time::ZERO,
                end: s.time,
                early: Vec::new(),
            },
            record: opts.record,
        };
        let done = e.run_steps(&s.steps);
        if s.cases.is_empty() || !done {
            if done {
                e.finish();
            } else {
                e.finish();
                for c in &s.cases {
                    for st in &c.steps {
                        let k = e.result(st);
                        e.tl.results[k].evidence.push("the common steps did not finish".into());
                    }
                }
            }
            return Ok(vec![e.outcome(s, None, &images, wall)]);
        }
        let mut out = Vec::new();
        for c in &s.cases {
            let started = std::time::Instant::now();
            let mut f = e.fork();
            f.run_steps(&c.steps);
            f.finish();
            let mut o = f.outcome(s, Some(&c.name), &images, started);
            o.wall += wall.elapsed() - started.elapsed();
            out.push(o);
        }
        Ok(out)
    }));
    match r {
        Ok(Ok(v)) => v,
        Ok(Err(e)) => fail(e),
        Err(p) => {
            let msg = p
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "a panic".into());
            fail(format!("the emulator panicked: {msg}"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symbols_are_cut_short() {
        assert_eq!(
            short_symbol(
                "<embassy_executor::platform::thread::Executor>::run::<sbc_board::__cortex_m_rt_main::{closure#0}>"
            ),
            "Executor::run<main>"
        );
        assert_eq!(short_symbol("esc_board::core1::core1_main"), "core1_main");
        assert_eq!(
            short_symbol("<embassy_rp::critical_section_impl::RpSpinlockCs as critical_section::Impl>::acquire"),
            "Impl::acquire"
        );
    }

    #[test]
    fn short_times() {
        assert_eq!(short_time(Time::from_secs(15)), "15s");
        assert_eq!(short_time(Time::from_ms(1500)), "1.5s");
        assert_eq!(short_time(Time::from_ms(200)), "200ms");
        assert_eq!(short_time(Time::from_us(50)), "50us");
        assert_eq!(short_time(Time::ZERO), "0s");
    }

    #[test]
    fn image_overrides() {
        let dir = std::env::temp_dir().join(format!("scenario-images-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("esc_board"), b"x").unwrap();
        let o = vec![
            ImageOverride::Board("esc1".into(), "/b/esc_board".into()),
            ImageOverride::File("/f/power_board".into()),
            ImageOverride::Dir(dir.clone()),
        ];
        let cur = |n: &str| PathBuf::from("/release").join(n);
        assert_eq!(override_image(&o, "esc1", &cur("esc_board")), Some("/b/esc_board".into()), "the board's own");
        assert_eq!(override_image(&o, "esc0", &cur("esc_board")), Some(dir.join("esc_board")), "from the directory");
        assert_eq!(override_image(&o, "poac", &cur("power_board")), Some("/f/power_board".into()), "by file name");
        assert_eq!(override_image(&o, "led", &cur("led_board")), None, "not in the directory");
        assert_eq!(
            format!("esc1={}", "/b/x").parse::<ImageOverride>(),
            Ok(ImageOverride::Board("esc1".into(), "/b/x".into()))
        );
        assert_eq!(dir.to_str().unwrap().parse::<ImageOverride>(), Ok(ImageOverride::Dir(dir.clone())));
        assert!("/no/such/thing".parse::<ImageOverride>().is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
