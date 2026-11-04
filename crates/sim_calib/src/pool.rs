//! Replaying a real pool test into the emulated boards: the robot computer's side of a candump log (the
//! thruster commands, the kill switch, titan_link and micro-ROS requests) is handed to the boards of a robot file
//! at the times it was recorded, and what the emulated boards send is compared with what the real boards sent,
//! message stream by message stream ([`sim_host::candiff`]).
//!
//! Real logs bring their own problems, handled here:
//!
//! * **Clocks.** A candump log is stamped with the robot computer's clock, from some moment after the boards
//!   powered on; the emulator's time starts at power-on. The boards' `FirmwareStatus` messages carry their uptime,
//!   so a straight-line fit of uptime against log time gives each board's power-on moment in the log and its clock
//!   drift against the computer's. The log is mapped onto the emulator's time with the first emulated board's
//!   line; boards that powered on at other moments, or drift differently, are reported. Without uptimes (C
//!   firmware), the log's first frame goes to `log_start` and the comparison searches for the offset.
//! * **Boards that are not emulated.** Frames from CANmore clients the robot file does not have are replayed as
//!   recorded (the bus is as busy as it was) and listed; frames whose identifiers are not CANmore are replayed too
//!   and compared as raw streams.
//! * **Bus errors and damaged lines.** Error frames and unreadable lines are counted ([`canlog::parse_lenient`]).
//!
//! With a recorded pose (from the robot's EKF, converted from a ROS bag by `scripts/bag_to_calib.py`) the robot's
//! vehicle can be moved along it, so its depth sensor and transducers see what the real ones saw, or the
//! emulator's physics can run and be compared with it.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::Path;
use std::sync::{Arc, Mutex};

use canmore::id::{Direction, Fields};
use sim_core::time::Time;
use sim_devices::can::Sender;
use sim_host::candiff::{self, Align, Decoded, DiffOptions, Report};
use sim_host::canlog::{self, LogFrame, ParseStats, Replay};
use sim_host::robot::{Robot, RobotSpec};
use sim_host::vehicle::ExternalPose;
use sim_physics::Quat;

use crate::table::Table;

/// A recorded pose track (`time_s`, `north_m`, `east_m`, `depth_m`, `roll_deg`, `pitch_deg`, `yaw_deg`).
#[derive(Clone, Debug, Default)]
pub struct PoseTrack {
    /// Seconds: Unix time (as the robot computer stamps a bag), or seconds since the log's first frame.
    pub time_s: Vec<f64>,
    pub pose: Vec<[f64; 6]>,
}

impl PoseTrack {
    pub fn from_table(t: &Table) -> Result<PoseTrack, String> {
        let time_s = t.col("time_s").ok_or("pose: no time_s column")?;
        let cols: Vec<Vec<f64>> = ["north_m", "east_m", "depth_m", "roll_deg", "pitch_deg", "yaw_deg"]
            .iter()
            .map(|c| t.col(c).unwrap_or_else(|| vec![0.0; time_s.len()]))
            .collect();
        if t.col("depth_m").is_none() {
            return Err("pose: no depth_m column".into());
        }
        let pose = (0..time_s.len())
            .map(|i| std::array::from_fn(|j| if cols[j][i].is_finite() { cols[j][i] } else { 0.0 }))
            .collect();
        Ok(PoseTrack { time_s, pose })
    }

    /// The pose at `t` (same units as `time_s`), interpolated (yaw the short way round), held at the ends.
    pub fn at(&self, t: f64) -> Option<[f64; 6]> {
        let n = self.time_s.len();
        if n == 0 {
            return None;
        }
        let k = self.time_s.partition_point(|&x| x <= t);
        if k == 0 {
            return Some(self.pose[0]);
        }
        if k >= n {
            return Some(self.pose[n - 1]);
        }
        let (t0, t1) = (self.time_s[k - 1], self.time_s[k]);
        let f = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
        let (a, b) = (self.pose[k - 1], self.pose[k]);
        Some(std::array::from_fn(|j| {
            let mut d = b[j] - a[j];
            if j >= 3 {
                d = (d + 180.0).rem_euclid(360.0) - 180.0;
            }
            a[j] + f * d
        }))
    }
}

/// What to do with a recorded pose.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseMode {
    /// Move the robot's vehicle along it (external, as nereus does).
    Drive,
    /// Let the emulator's physics move the vehicle and compare.
    Compare,
}

#[derive(Clone, Debug)]
pub struct PoolOptions {
    /// Log interface to robot bus (`can0` -> `internal`); by default an interface named like a bus goes to it,
    /// and `can0`, `can1`, ... to the robot file's first, second, ... bus.
    pub iface_map: Vec<(String, String)>,
    /// Emulator time at which the log's first frame is replayed (instead of the uptime alignment).
    pub log_start: Option<Time>,
    /// The boards' clock drift against the log's clock, ppm (instead of fitting it).
    pub drift_ppm: Option<f64>,
    /// Replay the frames of CANmore clients the robot file does not have.
    pub replay_other_boards: bool,
    pub tolerance: Time,
    pub ignore_payload: Vec<String>,
    pub only: Vec<String>,
    /// Stop the emulator here at the latest.
    pub max_time: Option<Time>,
    pub threads: usize,
    pub pose: Option<(PoseTrack, PoseMode)>,
    /// The vehicle the pose is for (the robot's first by default).
    pub vehicle: Option<String>,
    pub details: usize,
    /// Compare only from this long after the log's first frame (boards that had been running long before the log
    /// started settle into the host's traffic differently from boards that meet it at once).
    pub compare_after: Time,
}

impl Default for PoolOptions {
    fn default() -> Self {
        PoolOptions {
            iface_map: Vec::new(),
            log_start: None,
            drift_ppm: None,
            replay_other_boards: true,
            tolerance: Time::from_ms(20),
            ignore_payload: Vec::new(),
            only: Vec::new(),
            max_time: None,
            threads: 1,
            pose: None,
            vehicle: None,
            details: 5,
            compare_after: Time::ZERO,
        }
    }
}

/// One recorded log file: its name, frames and what was left out reading it.
pub struct LogFile {
    pub name: String,
    pub frames: Vec<LogFrame>,
    pub stats: ParseStats,
}

impl LogFile {
    pub fn read(path: &Path) -> Result<LogFile, String> {
        let (frames, stats) = canlog::read_file_lenient(path)?;
        Ok(LogFile { name: path.display().to_string(), frames, stats })
    }
}

/// A board's clock as seen in the log, from its `FirmwareStatus` uptimes.
#[derive(Clone, Debug, PartialEq)]
pub struct BoardClock {
    pub board: String,
    pub bus: String,
    pub client: u8,
    pub samples: usize,
    /// Log time (ns, as logged) at which the board's uptime was zero.
    pub power_on_ns: f64,
    /// Board seconds per log second, minus one, in ppm (0 with a single sample).
    pub drift_ppm: f64,
    /// From power-on to the moment the firmware's uptime is zero, plus the time a FirmwareStatus takes to reach
    /// the bus: measured on the emulated board (the real one boots the same way), ns.
    pub boot_ns: f64,
}

/// How log times map to emulator times: `virtual = (log - origin) * rate + offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct ClockMap {
    pub origin_ns: u64,
    pub rate: f64,
    pub offset_ns: f64,
    /// How it was found.
    pub how: String,
}

impl ClockMap {
    pub fn to_virtual_ns(&self, log_ns: u64) -> f64 {
        (log_ns as f64 - self.origin_ns as f64) * self.rate + self.offset_ns
    }
}

/// The result of a replay.
pub struct PoolResult {
    pub clock: ClockMap,
    pub clocks: Vec<BoardClock>,
    pub report: Report,
    /// The emulated boards' frames (bus names as interfaces, emulator times).
    pub emulated: Vec<LogFrame>,
    /// The recorded boards' frames compared, on the emulator's timeline.
    pub recorded: Vec<LogFrame>,
    pub markdown: String,
    pub identical: bool,
    /// Frames replayed, and dropped for falling before power-on.
    pub replayed: usize,
    pub dropped: usize,
    /// Fault words that differ: (stream, time in s, recorded, emulated).
    pub fault_diffs: Vec<(String, f64, String, String)>,
}

fn uptime_ms(payload: &str) -> Option<u64> {
    let at = payload.find("uptime_ms: ")? + "uptime_ms: ".len();
    payload[at..].split(|c: char| !c.is_ascii_digit()).next()?.parse().ok()
}

/// A FirmwareStatus payload without its uptime (which differs between a board and its emulation by however
/// apart they powered on), so the rest (faults, kill switches) is compared.
fn without_uptime(payload: &str) -> String {
    match payload.find("uptime_ms: ") {
        Some(at) => {
            let rest = &payload[at..];
            let end = rest.find(", ").map_or(rest.len(), |e| e + 2);
            format!("{}{}", &payload[..at], &rest[end..])
        }
        None => payload.to_string(),
    }
}

fn field(payload: &str, name: &str) -> Option<String> {
    let at = payload.find(&format!("{name}: "))? + name.len() + 2;
    Some(payload[at..].split([',', ' ', '}']).next()?.to_string())
}

/// Least-squares line `y = a x + b`.
fn line(points: &[(f64, f64)]) -> (f64, f64) {
    let n = points.len() as f64;
    if points.len() < 2 {
        return (1.0, points.first().map_or(0.0, |p| p.1 - p.0));
    }
    let mx = points.iter().map(|p| p.0).sum::<f64>() / n;
    let my = points.iter().map(|p| p.1).sum::<f64>() / n;
    let sxx: f64 = points.iter().map(|p| (p.0 - mx).powi(2)).sum();
    let sxy: f64 = points.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    if sxx <= 0.0 {
        return (1.0, my - mx);
    }
    let a = sxy / sxx;
    (a, my - a * mx)
}

/// [`line`], refitted without the points more than 5 ms (or three median deviations) off it: a status that
/// waited in the board's queue (a busy or unplugged bus) arrives late for its uptime.
fn robust_line(points: &[(f64, f64)]) -> (f64, f64) {
    let mut pts = points.to_vec();
    let mut fit = line(&pts);
    for _ in 0..3 {
        let res: Vec<f64> = pts.iter().map(|p| (p.1 - (fit.0 * p.0 + fit.1)).abs()).collect();
        let mut sorted = res.clone();
        sorted.sort_by(f64::total_cmp);
        let limit = (3.0 * sorted[sorted.len() / 2]).max(0.005);
        let keep: Vec<(f64, f64)> = pts.iter().zip(&res).filter(|(_, r)| **r <= limit).map(|(p, _)| *p).collect();
        if keep.len() == pts.len() || keep.len() < 2 {
            break;
        }
        pts = keep;
        fit = line(&pts);
    }
    fit
}

fn post_process(d: &mut Decoded) {
    for e in &mut d.events {
        if e.stream.ends_with(" FirmwareStatus") {
            e.payload = without_uptime(&e.payload);
        }
    }
}

/// How long after power-on each emulated board's FirmwareStatus uptime is zero, as seen on the bus (ns): a copy
/// of the robot runs with a mock bridge for each board until each has sent its status twice (3 s at most).
fn boot_delays(robot: &Robot, emulated: &[(String, String, u8)]) -> BTreeMap<String, f64> {
    let mut copy = robot.snapshot();
    copy.threads = robot.threads;
    for (j, rb) in copy.buses.iter_mut().enumerate() {
        rb.bus.record = Some(Vec::new());
        for (_, bus, client) in emulated {
            if *bus == robot.bus_names[j] {
                let b = sim_host::mock_bridge::MockBridge::new(*client, rb.bus.rates.data.is_some(), Time::ZERO);
                b.set_kill(Some(false));
                rb.endpoints.push(Box::new(b));
            }
        }
    }
    for s in &mut copy.boards {
        s.on_log = None;
        s.keep_logs = false;
    }
    let mut out = BTreeMap::new();
    let mut t = Time::ZERO;
    while t < Time::from_secs(3) && out.len() < emulated.len() {
        t += Time::from_ms(250);
        if copy.run_until(t).is_some() {
            break;
        }
        let mut frames = Vec::new();
        for (j, rb) in copy.buses.iter().enumerate() {
            for r in rb.bus.record.as_deref().unwrap_or(&[]) {
                if matches!(r.sender, Sender::Node(_)) && r.acked {
                    frames.push(LogFrame {
                        time_ns: r.end.as_ns(),
                        iface: copy.bus_names[j].clone(),
                        frame: r.frame.clone(),
                    });
                }
            }
        }
        let d = candiff::decode(&frames, false);
        for (board, bus, client) in emulated {
            let stream = format!("{bus} client {client} board->host FirmwareStatus");
            let lat: Vec<f64> = d
                .events
                .iter()
                .filter(|e| e.stream == stream)
                .filter_map(|e| Some(e.t_ns as f64 - uptime_ms(&e.payload)? as f64 * 1e6))
                .collect();
            if lat.len() >= 2 {
                out.insert(board.clone(), lat.iter().sum::<f64>() / lat.len() as f64);
            }
        }
    }
    out
}

/// Replays the logs into the robot and compares. `spec` and `dir` are the robot file and its directory.
pub fn replay(spec: &RobotSpec, dir: &Path, logs: &[LogFile], opts: &PoolOptions) -> Result<PoolResult, String> {
    let mut notes: Vec<String> = Vec::new();
    let mut robot = Robot::from_spec(spec, dir)?;
    robot.threads = opts.threads;
    let bus_names = robot.bus_names.clone();
    // Interfaces to buses
    let mut all: Vec<LogFrame> = Vec::new();
    let mut unmapped: BTreeMap<String, usize> = BTreeMap::new();
    for lf in logs {
        for f in &lf.frames {
            let bus = opts
                .iface_map
                .iter()
                .find(|(i, _)| *i == f.iface)
                .map(|(_, b)| b.clone())
                .or_else(|| bus_names.iter().find(|b| **b == f.iface).cloned())
                .or_else(|| {
                    let k: usize =
                        f.iface.strip_prefix("can").or_else(|| f.iface.strip_prefix("vcan"))?.parse().ok()?;
                    (opts.iface_map.is_empty()).then(|| bus_names.get(k).cloned()).flatten()
                });
            match bus {
                Some(b) if bus_names.contains(&b) => {
                    all.push(LogFrame { time_ns: f.time_ns, iface: b, frame: f.frame.clone() })
                }
                Some(b) => {
                    return Err(format!(
                        "--iface {}={b}: the robot has no bus {b} ({})",
                        f.iface,
                        bus_names.join(", ")
                    ));
                }
                None => *unmapped.entry(f.iface.clone()).or_default() += 1,
            }
        }
    }
    for (i, n) in &unmapped {
        notes.push(format!(
            "{n} frame(s) on {i} are not replayed: no bus for that interface (map it with --iface {i}=BUS)"
        ));
    }
    all.sort_by_key(|f| f.time_ns);
    if all.is_empty() {
        return Err("no frames to replay".into());
    }
    // The emulated boards' clients, by bus
    let plan = sim_host::bridges::plan(spec, dir, &[], None)?;
    let emulated: Vec<(String, String, u8)> =
        plan.iter().filter_map(|c| Some((c.board.clone(), c.bus.clone()?, c.client_id?))).collect();
    let is_emulated = |bus: &str, client: u8| emulated.iter().any(|(_, b, c)| b == bus && *c == client);
    let board_frame = |f: &LogFrame| {
        let fl = Fields::decode(f.frame.id, f.frame.extended);
        fl.direction == Direction::ClientToAgent && is_emulated(&f.iface, fl.client_id)
    };
    // The clock: the emulated boards' uptimes in the log, and how long after power-on an emulated board's uptime
    // starts (measured on a copy of the robot with a mock bridge per board, so the boards report their status)
    let boot = boot_delays(&robot, &emulated);
    let decoded_all = candiff::decode(&all, false);
    let t0 = all[0].time_ns;
    let mut clocks = Vec::new();
    for (board, bus, client) in &emulated {
        let stream = format!("{bus} client {client} board->host FirmwareStatus");
        let pts: Vec<(f64, f64)> = decoded_all
            .events
            .iter()
            .filter(|e| e.stream == stream)
            .filter_map(|e| Some(((e.t_ns as u64 - t0) as f64 / 1e9, uptime_ms(&e.payload)? as f64 / 1e3)))
            .collect();
        if pts.is_empty() {
            continue;
        }
        let (a, b) = robust_line(&pts);
        let a = opts.drift_ppm.map_or(a, |p| 1.0 + p * 1e-6);
        let b = if opts.drift_ppm.is_some() {
            pts.iter().map(|p| p.1 - a * p.0).sum::<f64>() / pts.len() as f64
        } else {
            b
        };
        clocks.push(BoardClock {
            board: board.clone(),
            bus: bus.clone(),
            client: *client,
            samples: pts.len(),
            boot_ns: boot.get(board).copied().unwrap_or(0.0),
            power_on_ns: t0 as f64 - (b / a) * 1e9 - boot.get(board).copied().unwrap_or(0.0) / a,
            drift_ppm: (a - 1.0) * 1e6,
        });
    }
    let power_on_ms =
        |board: &str| spec.board.iter().find(|b| b.name == board).and_then(|b| b.power_on_ms).unwrap_or(0.0);
    let clock = match (opts.log_start, clocks.first()) {
        (Some(start), _) => ClockMap {
            origin_ns: t0,
            rate: 1.0 + opts.drift_ppm.unwrap_or(0.0) * 1e-6,
            offset_ns: start.as_ns() as f64,
            how: format!("the log's first frame replayed at {start} (--log-start)"),
        },
        (None, Some(c)) => {
            let rate = 1.0 + c.drift_ppm * 1e-6;
            ClockMap {
                origin_ns: t0,
                rate,
                offset_ns: (t0 as f64 - c.power_on_ns) * rate + power_on_ms(&c.board) * 1e6,
                how: format!(
                    "{}'s power-on, from {} FirmwareStatus uptime(s): it powered on {:.3} s before the log's first \
                     frame{}",
                    c.board,
                    c.samples,
                    (t0 as f64 - c.power_on_ns) / 1e9,
                    if c.samples > 1 {
                        format!(", its clock {:+.1} ppm against the log's", c.drift_ppm)
                    } else {
                        String::new()
                    }
                ),
            }
        }
        (None, None) => {
            notes.push(
                "no FirmwareStatus uptime from an emulated board in the log (C firmware, or the log has none): the \
                 log's first frame is replayed at power-on; give --log-start if the boards had been running"
                    .into(),
            );
            ClockMap {
                origin_ns: t0,
                rate: 1.0,
                offset_ns: 0.0,
                how: "the log's first frame at power-on (no uptimes)".into(),
            }
        }
    };
    let aligned_by_uptime = opts.log_start.is_none() && !clocks.is_empty();
    if aligned_by_uptime && clocks.iter().any(|c| c.boot_ns == 0.0) {
        notes.push(
            "an emulated board sent no FirmwareStatus while probing its boot: its power-on is taken as uptime 0".into(),
        );
    }
    // The replay: what the robot computer sent, and what boards that are not emulated sent
    let mut frames_by_bus: Vec<Vec<(Time, sim_devices::can::CanFrame)>> = vec![Vec::new(); bus_names.len()];
    let mut dropped = 0;
    let mut other_clients: BTreeMap<(String, u8), usize> = BTreeMap::new();
    for f in &all {
        let fl = Fields::decode(f.frame.id, f.frame.extended);
        let host = fl.direction == Direction::AgentToClient;
        if !host && is_emulated(&f.iface, fl.client_id) {
            continue;
        }
        if !host {
            *other_clients.entry((f.iface.clone(), fl.client_id)).or_default() += 1;
            if !opts.replay_other_boards {
                continue;
            }
        }
        let v = clock.to_virtual_ns(f.time_ns);
        let j = bus_names.iter().position(|b| *b == f.iface).unwrap();
        let rates = robot.buses[j].bus.rates;
        let dur = f.frame.duration(rates.nominal, rates.data.unwrap_or(rates.nominal)).as_ns() as f64;
        if v - dur < 0.0 {
            dropped += 1;
            continue;
        }
        frames_by_bus[j].push((Time::from_ns((v - dur).round() as u64), f.frame.clone()));
    }
    let replayed: usize = frames_by_bus.iter().map(Vec::len).sum();
    for (j, frames) in frames_by_bus.into_iter().enumerate() {
        robot.buses[j].bus.record = Some(Vec::new());
        robot.buses[j].endpoints.push(Box::new(Replay::new(frames)));
    }
    // What the emulated boards log that a pool test would care about
    let logs_seen: Arc<Mutex<Vec<(String, Time, String)>>> = Arc::default();
    for (i, s) in robot.boards.iter_mut().enumerate() {
        s.keep_logs = false;
        let name = robot.names[i].clone();
        let seen = logs_seen.clone();
        s.on_log = Some(Box::new(move |t, line| {
            let lvl = line.level.as_deref().unwrap_or("");
            if matches!(lvl, "WARN" | "ERROR") || line.message.contains("panicked") {
                seen.lock().unwrap().push((name.clone(), t, line.to_string()));
            }
        }));
    }
    // The vehicle and the pose
    let vi = match &opts.vehicle {
        Some(n) => {
            Some(robot.vehicles.iter().position(|v| v.vehicle.name == *n).ok_or_else(|| format!("no vehicle {n:?}"))?)
        }
        None => (!robot.vehicles.is_empty()).then_some(0),
    };
    let pose = opts.pose.as_ref().map(|(p, m)| (p, *m));
    if pose.is_some() && vi.is_none() {
        return Err("a pose needs a vehicle in the robot file".into());
    }
    if let (Some((_, PoseMode::Drive)), Some(i)) = (pose, vi) {
        robot.vehicles[i].external = true;
    }
    // Pose times to emulator seconds
    let pose_virtual = |t: f64| -> f64 {
        if t > canlog::RELATIVE_LIMIT_NS as f64 / 1e9 {
            clock.to_virtual_ns((t * 1e9) as u64) / 1e9
        } else {
            clock.to_virtual_ns(t0 + (t * 1e9) as u64) / 1e9
        }
    };
    let pose_v: Option<(PoseTrack, PoseMode)> = pose.map(|(p, m)| {
        (PoseTrack { time_s: p.time_s.iter().map(|&t| pose_virtual(t)).collect(), pose: p.pose.clone() }, m)
    });
    let last = clock.to_virtual_ns(all.last().unwrap().time_ns).max(0.0);
    let mut end = Time::from_ns(last as u64) + Time::from_ms(200);
    if let Some(m) = opts.max_time {
        end = end.min(m);
    }
    let mut pose_err: Vec<(f64, f64)> = Vec::new();
    let chunk = Time::from_ms(10);
    while robot.now < end {
        let next = (robot.now + chunk).min(end);
        if let (Some((track, PoseMode::Drive)), Some(i)) = (&pose_v, vi)
            && let Some(p) = track.at(robot.now.as_secs_f64())
        {
            robot.vehicles[i].set_pose(&ExternalPose {
                position: [p[0], p[1], p[2]],
                attitude: Quat::from_euler(p[3].to_radians(), p[4].to_radians(), p[5].to_radians()),
                velocity: None,
                rate: None,
            });
        }
        if let Some((i, why)) = robot.run_until(next) {
            notes.push(format!("{} stopped at {}: {why:?}", robot.names[i], robot.now));
            break;
        }
        if let (Some((track, PoseMode::Compare)), Some(i)) = (&pose_v, vi)
            && let Some(p) = track.at(robot.now.as_secs_f64())
            && robot.now.as_secs_f64() >= track.time_s[0]
        {
            let s = &robot.vehicles[i].vehicle.state;
            let yaw = s.attitude.euler()[2].to_degrees();
            pose_err.push((s.position[2] - p[2], ((yaw - p[5]) + 180.0).rem_euclid(360.0) - 180.0));
        }
    }
    // The emulated boards' frames
    let mut emulated_log = Vec::new();
    for (j, rb) in robot.buses.iter().enumerate() {
        for r in rb.bus.record.as_deref().unwrap_or(&[]) {
            if matches!(r.sender, Sender::Node(_)) && r.acked && r.error.is_none() {
                emulated_log.push(LogFrame {
                    time_ns: r.end.as_ns(),
                    iface: bus_names[j].clone(),
                    frame: r.frame.clone(),
                });
            }
        }
    }
    emulated_log.sort_by_key(|f| f.time_ns);
    // Which clients the emulated boards really sent as (a strap or a serial number the plan got wrong)
    let mut sent_as: BTreeSet<(String, u8)> = BTreeSet::new();
    for f in &emulated_log {
        let fl = Fields::decode(f.frame.id, f.frame.extended);
        if fl.direction == Direction::ClientToAgent {
            sent_as.insert((f.iface.clone(), fl.client_id));
        }
    }
    for (bus, c) in &sent_as {
        if !is_emulated(bus, *c) {
            notes.push(format!(
                "an emulated board sent as client {c} on {bus}, which the robot file's plan did not expect: the real \
                 client {c}'s frames were replayed next to it"
            ));
        }
    }
    // The comparison, on the emulator's timeline
    let recorded: Vec<LogFrame> = all
        .iter()
        .filter(|f| board_frame(f))
        .map(|f| LogFrame {
            time_ns: clock.to_virtual_ns(f.time_ns).max(0.0).round() as u64,
            iface: f.iface.clone(),
            frame: f.frame.clone(),
        })
        .collect();
    let emulated_cmp: Vec<LogFrame> = emulated_log.iter().filter(|f| board_frame(f)).cloned().collect();
    let mut da = candiff::decode(&recorded, false);
    let mut db = candiff::decode(&emulated_cmp, false);
    post_process(&mut da);
    post_process(&mut db);
    let from_ns = clock.to_virtual_ns(t0) as i64 + opts.compare_after.as_ns() as i64;
    if opts.compare_after > Time::ZERO {
        da.events.retain(|e| e.t_ns >= from_ns);
        db.events.retain(|e| e.t_ns >= from_ns);
        notes.push(format!("compared from {:.3} s on (--compare-after {})", from_ns as f64 / 1e9, opts.compare_after));
    }
    let dopts = DiffOptions {
        align: if aligned_by_uptime { Align::None } else { Align::Auto },
        tolerance_ns: opts.tolerance.as_ns() as i64,
        stream_align: true,
        full: false,
        ignore_payload: opts.ignore_payload.clone(),
        only: opts.only.clone(),
    };
    let mut report = candiff::diff(&da, &db, &dopts);
    let labels: Vec<(String, u8, String)> = emulated.iter().map(|(b, bus, c)| (bus.clone(), *c, b.clone())).collect();
    for s in &mut report.streams {
        let w: Vec<&str> = s.name.split(' ').collect();
        if w.len() > 2
            && w[1] == "client"
            && let Ok(c) = w[2].parse::<u8>()
            && let Some((_, _, b)) = labels.iter().find(|(bus, cl, _)| bus == w[0] && *cl == c)
        {
            s.name = format!("{} client {c} ({b}) {}", w[0], w[3..].join(" "));
        }
    }
    // Faults: the FirmwareStatus fault words of each board over time
    let mut fault_diffs = Vec::new();
    let faults_of = |d: &Decoded, stream: &str| -> Vec<(i64, String)> {
        d.events
            .iter()
            .filter(|e| e.stream == stream)
            .filter_map(|e| Some((e.t_ns, field(&e.payload, "faults")?)))
            .collect()
    };
    let mut fault_rows = Vec::new();
    for (board, bus, client) in &emulated {
        let stream = format!("{bus} client {client} board->host FirmwareStatus");
        let (fa, fb) = (faults_of(&da, &stream), faults_of(&db, &stream));
        if fa.is_empty() && fb.is_empty() {
            continue;
        }
        let changes = |v: &[(i64, String)]| -> String {
            let mut out: Vec<String> = Vec::new();
            let mut prev: Option<&str> = None;
            for (t, w) in v {
                if prev != Some(w.as_str()) {
                    out.push(format!("{w} from {:.3} s", *t as f64 / 1e9));
                    prev = Some(w);
                }
            }
            if out.is_empty() { "none".into() } else { out.join(", ") }
        };
        // Paired in time: each recorded status against the emulated one nearest it
        let mut first_diff = None;
        for (t, w) in &fa {
            if let Some((_, wb)) = fb.iter().min_by_key(|(tb, _)| (tb - t).abs())
                && wb != w
            {
                first_diff.get_or_insert((*t as f64 / 1e9, w.clone(), wb.clone()));
            }
        }
        if let Some((t, a, b)) = &first_diff {
            fault_diffs.push((format!("{board} (client {client})"), *t, a.clone(), b.clone()));
        }
        fault_rows.push((board.clone(), changes(&fa), changes(&fb), first_diff));
    }
    let identical = report.identical() && fault_diffs.is_empty();
    // The report
    let mut md = String::from("# Pool test replay\n\n");
    for lf in logs {
        let _ = writeln!(
            md,
            "* `{}`: {} frame(s){}{}{}",
            lf.name,
            lf.frames.len(),
            if lf.stats.error_frames.is_empty() {
                String::new()
            } else {
                format!(
                    ", {} error frame(s) (bus errors) from {:.3} s to {:.3} s",
                    lf.stats.error_frames.len(),
                    lf.stats.error_frames.iter().min().map_or(0.0, |t| t.saturating_sub(t0) as f64 / 1e9),
                    lf.stats.error_frames.iter().max().map_or(0.0, |t| t.saturating_sub(t0) as f64 / 1e9)
                )
            },
            if lf.stats.xl_frames > 0 {
                format!(", {} CAN XL frame(s) skipped", lf.stats.xl_frames)
            } else {
                String::new()
            },
            if lf.stats.bad_lines.is_empty() {
                String::new()
            } else {
                format!(", {} unreadable line(s) skipped (first: {})", lf.stats.bad_lines.len(), lf.stats.bad_lines[0])
            }
        );
    }
    let _ = writeln!(md, "\n## Clocks\n\nLog times are mapped onto the emulator's by {}.\n", clock.how);
    if !clocks.is_empty() {
        let first_on = clocks[0].power_on_ns;
        let rows: Vec<Vec<String>> = clocks
            .iter()
            .map(|c| {
                let expected = power_on_ms(&c.board) - power_on_ms(&clocks[0].board);
                vec![
                    c.board.clone(),
                    format!("{} client {}", c.bus, c.client),
                    c.samples.to_string(),
                    format!("{:+.3} s", (c.power_on_ns - first_on) / 1e9),
                    format!("{:+.3} s", expected / 1e3),
                    if c.samples > 1 { format!("{:+.1}", c.drift_ppm) } else { "-".into() },
                    format!("{:.1} ms", c.boot_ns / 1e6),
                ]
            })
            .collect();
        md += &crate::table::markdown(
            &[
                "board",
                "client",
                "uptimes",
                "powered on (recorded)",
                "in the robot file",
                "drift (ppm)",
                "boot (emulated)",
            ],
            &rows,
        );
        md += "\nA board that powered on at another moment than the robot file says starts its emulated twin out of \
               step: set its `power_on_ms` to the recorded difference. Drifts of tens of ppm are crystals; the \
               comparison lines each stream up on its own as well.\n";
    }
    let _ = writeln!(
        md,
        "\n## Replay\n\n{replayed} frame(s) replayed into {} ({} dropped: before power-on), the run ended at {}.",
        bus_names.join(", "),
        dropped,
        robot.now
    );
    let others: Vec<String> =
        other_clients.iter().map(|((bus, c), n)| format!("{bus} client {c} ({n} frame(s))")).collect();
    if !others.is_empty() {
        let _ = writeln!(
            md,
            "\nNot in the robot file, {} as recorded: {}.",
            if opts.replay_other_boards { "replayed" } else { "left out" },
            others.join(", ")
        );
    }
    for n in &notes {
        let _ = writeln!(md, "\n* {n}");
    }
    md += "\n## What the boards sent\n\nA: the recorded boards, B: the emulated ones (FirmwareStatus compared \
           without its uptime).\n\n```\n";
    md += &report.render(opts.details);
    md += "```\n";
    if !fault_rows.is_empty() {
        md += "\n## Faults (FirmwareStatus)\n\n";
        let rows: Vec<Vec<String>> = fault_rows
            .iter()
            .map(|(b, a, e, d)| {
                vec![
                    b.clone(),
                    a.clone(),
                    e.clone(),
                    d.as_ref().map_or("agree".into(), |(t, a, b)| format!("at {t:.3} s: {a} recorded, {b} emulated")),
                ]
            })
            .collect();
        md += &crate::table::markdown(&["board", "recorded", "emulated", "first difference"], &rows);
    }
    let seen = logs_seen.lock().unwrap();
    let resets: Vec<String> = robot
        .boards
        .iter()
        .zip(&robot.names)
        .filter(|(b, _)| b.machine.resets > 0 || b.lockups > 0)
        .map(|(b, n)| format!("{n}: {} reset(s), {} lockup(s)", b.machine.resets, b.lockups))
        .collect();
    if !seen.is_empty() || !resets.is_empty() {
        md += "\n## The emulated boards' warnings, errors and resets\n\n";
        for r in &resets {
            let _ = writeln!(md, "* {r}");
        }
        for (b, t, l) in seen.iter().take(40) {
            let _ = writeln!(md, "* `[{t}] {b}: {l}`");
        }
        if seen.len() > 40 {
            let _ = writeln!(md, "* ... {} more", seen.len() - 40);
        }
    }
    if !pose_err.is_empty() {
        let n = pose_err.len() as f64;
        let rms =
            |k: usize| (pose_err.iter().map(|e| if k == 0 { e.0 * e.0 } else { e.1 * e.1 }).sum::<f64>() / n).sqrt();
        let _ = writeln!(
            md,
            "\n## The vehicle against the recorded pose\n\nThe emulator's physics moved the vehicle; against the \
             recorded pose: depth RMS error {:.3} m, yaw {:.1}° ({} samples, every 10 ms).",
            rms(0),
            rms(1),
            pose_err.len()
        );
    } else if let Some((_, PoseMode::Drive)) = &pose_v {
        md += "\nThe vehicle followed the recorded pose (its depth sensors and transducers saw it).\n";
    }
    let (miss, extra, payload) = report.counts();
    let _ = writeln!(
        md,
        "\n**{}**: {miss} missing, {extra} extra, {payload} payload difference(s), {} fault difference(s).",
        if identical { "SAME" } else { "DIFFERENT" },
        fault_diffs.len()
    );
    Ok(PoolResult {
        clock,
        clocks,
        report,
        emulated: emulated_log,
        recorded,
        markdown: md,
        identical,
        replayed,
        dropped,
        fault_diffs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uptime_and_fields_from_payloads() {
        let p = "FirmwareStatus(FirmwareStatus { board_name: \"x\", bus_id: 1, client_id: 1, version_major: 1, \
                 version_minor: 0, version_release_type: 2, uptime_ms: 12345, faults: 4, kill_switches_enabled: 0 })";
        assert_eq!(uptime_ms(p), Some(12345));
        assert_eq!(field(p, "faults").as_deref(), Some("4"));
        let w = without_uptime(p);
        assert!(!w.contains("uptime") && w.contains("version_release_type: 2, faults: 4"), "{w}");
        assert_eq!(line(&[(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)]), (2.0, 1.0));
        assert_eq!(line(&[(5.0, 7.0)]), (1.0, 2.0));
        // A late point (a status that waited on a busy bus) does not tilt the line
        let mut pts: Vec<(f64, f64)> = (0..20).map(|i| (i as f64, 0.999 * i as f64 + 0.5)).collect();
        pts[7].1 -= 0.3;
        let (a, b) = robust_line(&pts);
        assert!((a - 0.999).abs() < 1e-9 && (b - 0.5).abs() < 1e-9);
    }

    #[test]
    fn poses_interpolate_the_short_way_round() {
        let t = Table::parse("time_s,depth_m,yaw_deg\n0,1,170\n2,2,-170\n").unwrap();
        let p = PoseTrack::from_table(&t).unwrap();
        let mid = p.at(1.0).unwrap();
        assert!((mid[2] - 1.5).abs() < 1e-12 && (mid[5] - 180.0).abs() < 1e-9, "{mid:?}");
        assert_eq!(p.at(-1.0).unwrap()[2], 1.0);
        assert_eq!(p.at(5.0).unwrap()[2], 2.0);
        assert!(PoseTrack::from_table(&Table::parse("time_s,x\n0,1\n").unwrap()).is_err());
        let c = ClockMap { origin_ns: 1_000, rate: 1.0 + 1e-4, offset_ns: 500.0, how: String::new() };
        assert!((c.to_virtual_ns(1_000 + 1_000_000_000) - (1_000_100_000.0 + 500.0)).abs() < 1e-3);
    }
}
