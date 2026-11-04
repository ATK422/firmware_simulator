//! Time travel: going back to any earlier moment of a run, exactly.
//!
//! Runs are deterministic, so the past can be made again: [`TimeTravel`] keeps **checkpoints** (copies of the whole
//! target, a board or a robot, taken every [`TimeTravel::every`] of virtual time at slice boundaries, within a
//! memory [`TimeTravel::budget`]) and a **log** of everything from outside that the run depended on since: each
//! run the host asked for (to which time, and where it stopped), the physical inputs it set between runs, what a
//! debugger wrote, and what host endpoints (bridges, SocketCAN, cannelloni, UDP relays) and ptys sent in, per call.
//! Going back to a moment restores the checkpoint before it and **replays** the log up to it: the endpoints are not
//! asked again, what they sent then is given again, and what the boards send them is dropped (they saw it). Once the
//! replay reaches the end of the log (the *frontier*), the run is live again.
//!
//! A **position** in the timeline ([`Pos`]) is a slice boundary or a step of one core of one board: the core and the
//! time on its clock when the step (an instruction or an exception entry) starts. Each step takes at least one
//! cycle, so the time names the step; [`rp2040::Machine::stop_at`] stops a machine right there, and a debugger stop
//! in the middle of a slice is resumed exactly (the machine finishes its quantum, the session its slice), so
//! stopping never changes the run: replaying to a position gives the state a run that never stopped had there,
//! both cores, every device and bus included.
//!
//! On top of this, [`TimeTravel::reverse_continue`] finds the most recent breakpoint or watchpoint hit before the
//! current position (replaying from checkpoint to checkpoint backwards), [`TimeTravel::reverse_step`] the step of
//! a core before the current one, and [`TimeTravel::rewind_to`] goes to a virtual time. The GDB server
//! ([`crate::gdb`]), the dashboard and the MCP server use them.
//!
//! Not recorded: USB/IP traffic and the profiler (which cuts runs at sample times; it is left out of replays).
//! Host changes other than inputs and debugger writes (faults, a vehicle's pose) must be announced with
//! [`TimeTravel::changed`], which takes a checkpoint there that is never thinned out. Changing anything while in
//! the past discards the recorded future: the run goes on live from there, and host endpoints carry on from where
//! they were at the frontier.

use std::any::Any;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use rp2040::{Machine, StopReason};
use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

use crate::board::Inputs;
use crate::ethernet::EthEndpoint;
use crate::robot::Robot;
use crate::session::{Session, SessionCheckpoint};

// ------------------------------------------------------------------------------------------------ taps

/// What a host endpoint (or a pty) gave on each call, so replays can give it again: the calls made so far on the
/// current timeline, the calls recorded (the frontier), and the outputs that were not empty, by call number.
#[derive(Debug)]
pub struct TapLog<T> {
    calls: u64,
    recorded: u64,
    items: VecDeque<(u64, T)>,
    /// Whether the last call was replayed (its output came from the log).
    last_replayed: bool,
}

/// A [`TapLog`] shared between the endpoint and the [`TimeTravel`] that seeks it.
pub type SharedTap<T> = Arc<Mutex<TapLog<T>>>;

impl<T: Clone + Default + IsEmpty> TapLog<T> {
    pub fn new() -> SharedTap<T> {
        Arc::new(Mutex::new(TapLog { calls: 0, recorded: 0, items: VecDeque::new(), last_replayed: false }))
    }

    /// Whether the next call is replayed.
    pub fn replaying(&self) -> bool {
        self.calls < self.recorded
    }

    /// Whether the last call was.
    pub fn replaying_last(&self) -> bool {
        self.last_replayed
    }

    /// One call: live, `live()` and recorded; replayed, the recorded output.
    pub fn call(&mut self, live: impl FnOnce() -> T) -> T {
        let n = self.calls;
        self.calls += 1;
        if n < self.recorded {
            self.last_replayed = true;
            return match self.items.binary_search_by_key(&n, |(c, _)| *c) {
                Ok(i) => self.items[i].1.clone(),
                Err(_) => T::default(),
            };
        }
        self.last_replayed = false;
        let out = live();
        if !out.is_empty() {
            self.items.push_back((n, out.clone()));
        }
        self.recorded = self.calls;
        out
    }

    /// While replaying, the next output recorded that was not empty (`None` if none comes before the frontier).
    pub fn next_recorded(&self) -> Option<&T> {
        if !self.replaying() {
            return None;
        }
        let i = self.items.partition_point(|(c, _)| *c < self.calls);
        self.items.get(i).map(|(_, t)| t)
    }
}

/// Whether a tap's output is empty (and need not be recorded).
pub trait IsEmpty {
    fn is_empty(&self) -> bool;
}

impl<T> IsEmpty for Vec<T> {
    fn is_empty(&self) -> bool {
        Vec::is_empty(self)
    }
}

/// What a CAN endpoint handed in on one call: the frames, and the time the bus asked up to.
#[derive(Clone, Debug, Default)]
pub struct CanCall {
    pub until: Time,
    pub frames: Vec<(Time, CanFrame)>,
}

impl IsEmpty for CanCall {
    fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }
}

/// What [`TimeTravel`] does with a tap.
pub trait TapControl: Send {
    fn calls(&self) -> u64;
    /// Goes to call `calls` of the recording (a checkpoint's).
    fn seek(&mut self, calls: u64);
    /// Drops what was recorded after the current call (the future changed).
    fn truncate(&mut self);
}

impl<T: Send> TapControl for SharedTap<T> {
    fn calls(&self) -> u64 {
        self.lock().unwrap().calls
    }
    fn seek(&mut self, calls: u64) {
        let mut t = self.lock().unwrap();
        t.calls = calls.min(t.recorded);
        t.last_replayed = false;
    }
    fn truncate(&mut self) {
        let mut t = self.lock().unwrap();
        let calls = t.calls;
        t.items.retain(|(c, _)| *c < calls);
        t.recorded = calls;
    }
}

/// A CAN endpoint whose frames are recorded, and given again while history is replayed (when what the bus delivers
/// to it is dropped: it saw that then).
pub struct TappedCan {
    inner: Box<dyn CanEndpoint + Send>,
    log: SharedTap<CanCall>,
}

impl CanEndpoint for TappedCan {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let inner = &mut self.inner;
        self.log.lock().unwrap().call(|| CanCall { until, frames: inner.take_tx(until) }).frames
    }
    fn receive(&mut self, frame: &CanFrame, at: Time) {
        // The bus delivers after asking every endpoint for frames: this slice's call was the last one
        if !self.log.lock().unwrap().replaying_last() {
            self.inner.receive(frame, at);
        }
    }
    fn tx_queue_limit(&self) -> Option<usize> {
        self.inner.tx_queue_limit()
    }
    /// Live, the endpoint's own. Replaying, the time its next recorded frames were asked for, or (once the replay
    /// gets to the frontier) the endpoint's own, whichever is first: so a robot recorded for time travel runs its
    /// boards ahead of each other ([`crate::runahead`]) live and in replays alike, if the endpoint itself can tell.
    fn idle_until(&self) -> Option<Time> {
        let live = self.inner.idle_until()?;
        let log = self.log.lock().unwrap();
        Some(log.next_recorded().map_or(live, |c| c.until.min(live)))
    }
}

/// An Ethernet endpoint, recorded like [`TappedCan`].
pub struct TappedEth {
    inner: Box<dyn EthEndpoint>,
    log: SharedTap<Vec<Vec<u8>>>,
}

impl EthEndpoint for TappedEth {
    fn receive(&mut self, frame: &[u8], at: Time) {
        // A slice's frames from the board come before its call for frames to it
        if !self.log.lock().unwrap().replaying() {
            self.inner.receive(frame, at);
        }
    }
    fn take_tx(&mut self, until: Time) -> Vec<Vec<u8>> {
        let inner = &mut self.inner;
        self.log.lock().unwrap().call(|| inner.take_tx(until))
    }
}

/// A placeholder while an endpoint is being wrapped.
struct Nothing;
impl CanEndpoint for Nothing {
    fn take_tx(&mut self, _: Time) -> Vec<(Time, CanFrame)> {
        Vec::new()
    }
    fn receive(&mut self, _: &CanFrame, _: Time) {}
}
impl EthEndpoint for Nothing {
    fn receive(&mut self, _: &[u8], _: Time) {}
    fn take_tx(&mut self, _: Time) -> Vec<Vec<u8>> {
        Vec::new()
    }
}

/// The taps of a target, and how many endpoints of each list are wrapped already (lists only grow).
#[derive(Default)]
pub struct Taps {
    list: Vec<Box<dyn TapControl>>,
    wrapped: std::collections::BTreeMap<(u8, usize), usize>,
}

impl Taps {
    /// The endpoints of list `(kind, index)` not wrapped yet.
    fn new_ones<'a, T>(&mut self, kind: u8, index: usize, eps: &'a mut [T]) -> &'a mut [T] {
        let done = self.wrapped.entry((kind, index)).or_insert(0);
        let from = (*done).min(eps.len());
        *done = eps.len();
        &mut eps[from..]
    }
}

/// Wraps endpoints in taps, adding them to `taps`.
fn tap_can(eps: &mut [Box<dyn CanEndpoint + Send>], taps: &mut Vec<Box<dyn TapControl>>) {
    for e in eps {
        let inner = std::mem::replace(e, Box::new(Nothing));
        let log = TapLog::new();
        taps.push(Box::new(log.clone()));
        *e = Box::new(TappedCan { inner, log });
    }
}

fn tap_eth(eps: &mut [Box<dyn EthEndpoint>], taps: &mut Vec<Box<dyn TapControl>>) {
    for e in eps {
        let inner = std::mem::replace(e, Box::new(Nothing));
        let log = TapLog::new();
        taps.push(Box::new(log.clone()));
        *e = Box::new(TappedEth { inner, log });
    }
}

fn tap_pty(pty: &mut Option<crate::pty::Pty>, taps: &mut Vec<Box<dyn TapControl>>) {
    if let Some(p) = pty
        && p.tap.is_none()
    {
        let log = TapLog::new();
        taps.push(Box::new(log.clone()));
        p.tap = Some(log);
    }
}

// ------------------------------------------------------------------------------------------------ targets

/// How a run of a [`Rewindable`] ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ran {
    /// It reached the time asked for.
    Reached,
    /// Board `board` stopped: a debugger stop ([`StopReason::is_debug_stop`]) in the middle of a slice, or a stop
    /// that ends the run (a final lockup, a deadlock).
    Stopped { board: usize, reason: StopReason },
    /// The run was asked to stop (SIGINT, SIGTERM) and returned at a slice boundary.
    Interrupted,
}

/// What [`TimeTravel`] needs of what it runs: a board ([`Session`]) or a robot ([`Robot`]).
pub trait Rewindable {
    fn now(&self) -> Time;
    /// The end of the slice the next `run_until(end)` runs first.
    fn next_boundary(&self, end: Time) -> Time;
    fn slice(&self) -> Time;
    /// Whether slices end exactly a slice apart (a robot's do; a board's start where its machine got to, which may
    /// be a little past the last slice's end).
    fn exact_slices(&self) -> bool;
    /// Whether the target is between slices (no debugger stop interrupted one).
    fn at_boundary(&self) -> bool;
    fn run_until(&mut self, end: Time) -> Ran;
    /// Runs to `soft` or a little past it where the target's parts meet anyway (a robot running its boards ahead of
    /// each other: where a window ends), at most to `end`. By default exactly to `soft`.
    fn run_until_after(&mut self, soft: Time, end: Time) -> Ran {
        let _ = end;
        self.run_until(soft)
    }
    /// A copy of everything; `memory` holds each board's SRAM and ROM from the last checkpoint, to share where
    /// unchanged (and is updated; the flash is shared through its version, see [`Session::checkpoint`]). Returns it
    /// with about how many bytes it holds.
    fn checkpoint(&mut self, memory: &mut Vec<Option<crate::session::SharedMemory>>) -> (Box<dyn Any + Send>, usize);
    /// Puts everything back as in checkpoint `c`; what the boards collected (logs, text) by then comes from
    /// `collected` (kept up to date here) where the boards have less now.
    fn restore(
        &mut self,
        c: &(dyn Any + Send),
        collected: &mut Vec<crate::session::CollectedText>,
        memory: &mut Vec<Option<crate::session::SharedMemory>>,
    );
    /// Cuts `collected` to what the boards have now (the future changed).
    fn cut_collected(&mut self, collected: &mut Vec<crate::session::CollectedText>) {
        collected.resize_with(self.boards(), Default::default);
        for (b, c) in collected.iter_mut().enumerate() {
            Session::cut_collected(self.session(b), c);
        }
    }
    fn boards(&self) -> usize;
    fn session(&mut self, board: usize) -> &mut Session;
    /// Wraps host endpoints and ptys not wrapped yet in taps.
    fn tap(&mut self, taps: &mut Taps);
    /// While replaying: hooks, trace sinks, the profiler and real-time pacing are put aside (`true`), and back.
    fn quiet(&mut self, quiet: bool, store: &mut Quieted);
}

/// What [`Rewindable::quiet`] put aside.
#[derive(Default)]
pub struct Quieted {
    sessions: Vec<QuietSession>,
    buses: Vec<Option<crate::session::CanCallback>>,
    realtime: bool,
}

#[derive(Default)]
struct QuietSession {
    on_log: Option<crate::session::LogHook>,
    on_uart_line: Option<crate::session::UartHook>,
    on_usb_cdc_line: Option<crate::session::LineHook>,
    on_can: Option<crate::session::CanCallback>,
    trace: Option<Box<dyn sim_core::trace::TraceSink>>,
    profiler: Option<crate::profile::Profiler>,
    realtime: bool,
}

fn quiet_session(s: &mut Session, quiet: bool, q: &mut QuietSession) {
    if quiet {
        q.on_log = s.on_log.take();
        q.on_uart_line = s.on_uart_line.take();
        q.on_usb_cdc_line = s.on_usb_cdc_line.take();
        q.on_can = s.on_can.take();
        q.trace = Some(std::mem::replace(&mut s.machine.trace, Box::new(sim_core::trace::NullSink)));
        q.profiler = s.profiler.take();
        q.realtime = std::mem::take(&mut s.realtime);
    } else {
        s.on_log = q.on_log.take();
        s.on_uart_line = q.on_uart_line.take();
        s.on_usb_cdc_line = q.on_usb_cdc_line.take();
        s.on_can = q.on_can.take();
        if let Some(t) = q.trace.take() {
            s.machine.trace = t;
        }
        s.profiler = q.profiler.take();
        s.realtime = q.realtime;
        if s.realtime {
            s.reset_pacing();
        }
    }
}

fn session_ran(r: StopReason) -> Ran {
    match r {
        StopReason::TimeLimit => Ran::Reached,
        StopReason::Requested => Ran::Interrupted,
        reason => Ran::Stopped { board: 0, reason },
    }
}

fn session_tap(s: &mut Session, board: usize, taps: &mut Taps) {
    let new = taps.new_ones(0, board, &mut s.can_endpoints);
    tap_can(new, &mut taps.list);
    let new = taps.new_ones(1, board, &mut s.eth_endpoints);
    tap_eth(new, &mut taps.list);
    for p in &mut s.uart_pty {
        tap_pty(p, &mut taps.list);
    }
    tap_pty(&mut s.usb_cdc_pty, &mut taps.list);
    tap_pty(&mut s.usb_cdc2_pty, &mut taps.list);
}

fn session_checkpoint(s: &mut Session, memory: &mut Option<crate::session::SharedMemory>) -> SessionCheckpoint {
    let c = s.checkpoint(memory.as_ref());
    *memory = Some(c.memory().clone());
    c
}

impl Rewindable for Session {
    fn now(&self) -> Time {
        self.boundary()
    }
    fn next_boundary(&self, end: Time) -> Time {
        Session::next_boundary(self, end)
    }
    fn slice(&self) -> Time {
        self.slice
    }
    fn exact_slices(&self) -> bool {
        false
    }
    fn at_boundary(&self) -> bool {
        Session::at_boundary(self)
    }
    fn run_until(&mut self, end: Time) -> Ran {
        session_ran(Session::run_until(self, end))
    }
    fn checkpoint(&mut self, memory: &mut Vec<Option<crate::session::SharedMemory>>) -> (Box<dyn Any + Send>, usize) {
        memory.resize(1, None);
        let c = session_checkpoint(self, &mut memory[0]);
        let bytes = c.bytes();
        (Box::new(c), bytes)
    }
    fn restore(
        &mut self,
        c: &(dyn Any + Send),
        collected: &mut Vec<crate::session::CollectedText>,
        memory: &mut Vec<Option<crate::session::SharedMemory>>,
    ) {
        let c = c.downcast_ref::<SessionCheckpoint>().expect("a session checkpoint");
        collected.resize_with(1, Default::default);
        self.keep_collected(&mut collected[0]);
        self.rewind_to_with(c, &collected[0]);
        *memory = vec![Some(c.memory().clone())];
    }
    fn boards(&self) -> usize {
        1
    }
    fn session(&mut self, _: usize) -> &mut Session {
        self
    }
    fn tap(&mut self, taps: &mut Taps) {
        session_tap(self, 0, taps);
    }
    fn quiet(&mut self, quiet: bool, store: &mut Quieted) {
        store.sessions.resize_with(1, Default::default);
        quiet_session(self, quiet, &mut store.sessions[0]);
    }
}

/// The recorded points of a transducer's path.
type PathPoints = VecDeque<(f64, sim_acoustics::geometry::Vec3)>;

/// A robot's checkpoint: every board, bus, vehicle (with its transducers' paths) and the water.
struct RobotCheckpoint {
    boards: Vec<SessionCheckpoint>,
    buses: Vec<(sim_devices::can::CanBus, Option<usize>)>,
    /// Each vehicle, and the points of its transducers' paths.
    vehicles: Vec<(crate::vehicle::VehicleSim, Vec<PathPoints>)>,
    medium: Option<sim_acoustics::medium::Medium>,
    now: Time,
    power_on: Vec<(Time, bool)>,
}

/// Whether a debugger could stop `robot`: then it runs its boards in turn on the calling thread, board by board, as
/// with GDB attached without time travel (on threads, the other boards would run to the end of the slice, and a
/// replay taking turns to that stop would not have them there). While a stop is armed, or GDB is attached (it stops
/// at a `BKPT`).
fn debugging(robot: &Robot) -> bool {
    robot.boards.iter().any(|b| {
        let m = &b.machine;
        m.stop_at.iter().any(Option::is_some)
            || !m.breakpoints.is_empty()
            || !m.watchpoints.is_empty()
            || m.bkpt_policy == rp2040::machine::BkptPolicy::Stop
    })
}

/// How a robot's run to `end` ended, now at `now`.
fn robot_ran(r: Option<(usize, StopReason)>, now: Time, end: Time) -> Ran {
    match r {
        Some((board, reason)) => Ran::Stopped { board, reason },
        None if now < end => Ran::Interrupted,
        None => Ran::Reached,
    }
}

impl Rewindable for Robot {
    fn now(&self) -> Time {
        self.now
    }
    fn next_boundary(&self, end: Time) -> Time {
        (self.now + self.slice).min(end)
    }
    fn slice(&self) -> Time {
        self.slice
    }
    fn exact_slices(&self) -> bool {
        true
    }
    fn at_boundary(&self) -> bool {
        // Boards stopped in the middle of a slice have not reached the robot's next slice end
        self.boards.iter().all(|b| b.at_boundary())
    }
    fn run_until(&mut self, end: Time) -> Ran {
        let debugging = debugging(self);
        let threads = self.threads;
        if debugging {
            self.threads = 1;
        }
        let r = Robot::run_until(self, end);
        self.threads = threads;
        robot_ran(r, self.now, end)
    }
    fn run_until_after(&mut self, soft: Time, end: Time) -> Ran {
        if debugging(self) {
            return Rewindable::run_until(self, soft);
        }
        let r = Robot::run_until_after(self, soft, end);
        robot_ran(r, self.now, soft.min(end))
    }
    fn checkpoint(&mut self, memory: &mut Vec<Option<crate::session::SharedMemory>>) -> (Box<dyn Any + Send>, usize) {
        memory.resize(self.boards.len(), None);
        let boards: Vec<SessionCheckpoint> =
            self.boards.iter_mut().zip(memory.iter_mut()).map(|(b, m)| session_checkpoint(b, m)).collect();
        let mut bytes = boards.iter().map(|b| b.bytes()).sum::<usize>();
        let buses = self
            .buses
            .iter_mut()
            .map(|rb| {
                let record = rb.bus.record.take();
                let len = record.as_ref().map(|r| r.len());
                let copy = rb.bus.clone();
                rb.bus.record = record;
                (copy, len)
            })
            .collect();
        let vehicles =
            self.vehicles.iter().map(|v| (v.clone(), v.paths().map(|(_, p)| p.points()).collect())).collect();
        let medium = self.medium.as_ref().map(|m| m.lock().unwrap().clone());
        bytes += 64 * 1024 * (self.buses.len() + self.vehicles.len() + medium.is_some() as usize);
        let c = RobotCheckpoint { boards, buses, vehicles, medium, now: self.now, power_on: self.power_on.clone() };
        (Box::new(c), bytes)
    }
    fn restore(
        &mut self,
        c: &(dyn Any + Send),
        collected: &mut Vec<crate::session::CollectedText>,
        memory: &mut Vec<Option<crate::session::SharedMemory>>,
    ) {
        let c = c.downcast_ref::<RobotCheckpoint>().expect("a robot checkpoint");
        collected.resize_with(self.boards.len(), Default::default);
        for ((b, s), k) in self.boards.iter_mut().zip(&c.boards).zip(collected.iter_mut()) {
            b.keep_collected(k);
            b.rewind_to_with(s, k);
        }
        *memory = c.boards.iter().map(|b| Some(b.memory().clone())).collect();
        for (rb, (copy, len)) in self.buses.iter_mut().zip(&c.buses) {
            let mut record = rb.bus.record.take();
            if let (Some(r), Some(len)) = (&mut record, len) {
                r.truncate(*len);
            }
            rb.bus = copy.clone();
            rb.bus.record = record;
        }
        for (v, (copy, paths)) in self.vehicles.iter_mut().zip(&c.vehicles) {
            *v = copy.clone();
            for ((_, p), pts) in v.paths().zip(paths) {
                p.set_points(pts.clone());
            }
            v.update_outputs();
        }
        if let (Some(m), Some(copy)) = (&self.medium, &c.medium) {
            *m.lock().unwrap() = copy.clone();
        }
        self.now = c.now;
        self.power_on = c.power_on.clone();
    }
    fn boards(&self) -> usize {
        self.boards.len()
    }
    fn session(&mut self, board: usize) -> &mut Session {
        &mut self.boards[board]
    }
    fn tap(&mut self, taps: &mut Taps) {
        for (i, b) in self.boards.iter_mut().enumerate() {
            session_tap(b, i, taps);
        }
        for (j, rb) in self.buses.iter_mut().enumerate() {
            let new = taps.new_ones(2, j, &mut rb.endpoints);
            tap_can(new, &mut taps.list);
        }
    }
    fn quiet(&mut self, quiet: bool, store: &mut Quieted) {
        store.sessions.resize_with(self.boards.len(), Default::default);
        for (b, q) in self.boards.iter_mut().zip(&mut store.sessions) {
            quiet_session(b, quiet, q);
        }
        if quiet {
            store.buses = self.buses.iter_mut().map(|b| b.on_can.take()).collect();
            store.realtime = std::mem::take(&mut self.realtime);
        } else {
            for (b, h) in self.buses.iter_mut().zip(store.buses.drain(..)) {
                b.on_can = h;
            }
            self.realtime = store.realtime;
            if self.realtime {
                self.reset_pacing();
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------ the log

/// A position in the timeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pos {
    /// A slice boundary: every board has run to this time and the slice's end (buses, RTT, vehicles) is done.
    Boundary(Time),
    /// Core `core` of board `board` is about to take its step (an instruction or an exception entry) that starts
    /// at `time` on its clock (or its first step after that time).
    At { board: usize, core: usize, time: Time },
}

impl Pos {
    pub fn time(&self) -> Time {
        match *self {
            Pos::Boundary(t) | Pos::At { time: t, .. } => t,
        }
    }
}

/// How a recorded run ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ended {
    /// It reached its end (or is still running, for the last entry while it runs live).
    Completed,
    /// It stopped at a position: a debugger stop, or an interruption at a boundary.
    At(Pos),
    /// It stopped another way (a final lockup, a deadlock); a replay stops there too.
    Other,
}

/// A change a debugger made to a board's machine (memory, registers, a reset), applied again by replays.
pub type Change = Arc<dyn Fn(&mut Machine) + Send + Sync>;

#[derive(Clone)]
enum Entry {
    Run {
        end: Time,
        ended: Ended,
    },
    /// The physical inputs' values, every board's in name order.
    Inputs(Vec<Vec<f64>>),
    Apply {
        board: usize,
        change: Change,
    },
}

/// Where the target is in the log: before entry `entry`, or (with `inside`) in the middle of that run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Cursor {
    entry: usize,
    inside: bool,
}

struct Checkpoint {
    state: Box<dyn Any + Send>,
    bytes: usize,
    time: Time,
    cursor: Cursor,
    taps: Vec<u64>,
    inputs: Vec<Vec<f64>>,
    /// Taken right after a host change (see [`TimeTravel::changed`]): replays cannot go through it, so it is kept.
    barrier: bool,
    name: Option<String>,
}

/// A breakpoint or watchpoint hit (or a step) found by a replay.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hit {
    pub pos: Pos,
    pub reason: StopReason,
}

/// How a time-travel operation ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Travelled {
    /// At the position or time asked for.
    Arrived,
    /// At a breakpoint or watchpoint hit (or another stop) on the way.
    Hit(Hit),
    /// At the start of the recorded history: nothing earlier to go to.
    Beginning,
    /// At the frontier: the end of the recorded history (going on is live).
    Frontier,
    /// The run was asked to stop.
    Interrupted,
}

/// Counts for `monitor checkpoints` and the dashboard.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub checkpoints: usize,
    pub bytes: usize,
    pub taken: u64,
    pub taking_ns: u64,
    pub restores: u64,
    /// Virtual time re-executed by replays.
    pub replayed: Time,
    pub replay_ns: u64,
    /// The oldest time that can be gone back to, and the frontier's.
    pub oldest: Time,
    pub frontier: Time,
}

/// What a replay does with stops that are not its goal (see [`TimeTravel::replay`]).
enum Probe<'a> {
    /// Stops at every breakpoint or watchpoint hit.
    Report,
    /// Goes through them.
    Ignore,
    /// Collects them (positions strictly before `before`).
    Collect { hits: &'a mut Vec<Hit>, before: Pos },
    /// Collects the steps of one core from a time on (with the breakpoints and watchpoints put aside).
    Steps { board: usize, core: usize, from: Time, steps: &'a mut Vec<Time>, ephemeral: bool },
}

/// Recorded history of a run, with checkpoints (see the module documentation).
pub struct TimeTravel {
    /// Virtual time between checkpoints (for a robot running its boards ahead of each other, up to a window more:
    /// see [`Robot::run_until_after`]).
    pub every: Time,
    /// Memory the checkpoints may take; older ones are thinned out beyond it (spacing grows with age).
    pub budget: usize,
    log: Vec<Entry>,
    checkpoints: Vec<Checkpoint>,
    /// The latest checkpoint before the current position.
    base: usize,
    cursor: Cursor,
    /// The current position.
    here: Pos,
    memory: Vec<Option<crate::session::SharedMemory>>,
    taps: Taps,
    collected: Vec<crate::session::CollectedText>,
    inputs: Vec<Inputs>,
    last_inputs: Vec<Vec<f64>>,
    last_checkpoint: Time,
    used: usize,
    stats: Stats,
    /// A checkpoint in the middle of a run from the last step search, and the steps of that core it leads to (for
    /// stepping back one at a time).
    steps_cache: Option<StepsCache>,
}

struct StepsCache {
    cp: Checkpoint,
    board: usize,
    core: usize,
    steps: Vec<Time>,
}

/// Default virtual time between checkpoints (see docs/running.md for what it costs).
pub const DEFAULT_EVERY: Time = Time::from_ms(10);
/// Default memory budget for checkpoints.
pub const DEFAULT_BUDGET: usize = 512 << 20;

impl TimeTravel {
    /// Starts recording `target` from now, with `inputs` (every board's, as the robot or the board file gave them):
    /// takes the first checkpoint (kept, so any time from now on can be gone back to) and wraps the target's host
    /// endpoints and ptys.
    pub fn new(target: &mut dyn Rewindable, inputs: Vec<Inputs>) -> TimeTravel {
        let mut tt = TimeTravel {
            every: DEFAULT_EVERY,
            budget: DEFAULT_BUDGET,
            log: Vec::new(),
            checkpoints: Vec::new(),
            base: 0,
            cursor: Cursor { entry: 0, inside: false },
            here: Pos::Boundary(target.now()),
            memory: Vec::new(),
            taps: Taps::default(),
            collected: Vec::new(),
            last_inputs: values(&inputs),
            inputs,
            last_checkpoint: target.now(),
            used: 0,
            stats: Stats::default(),
            steps_cache: None,
        };
        target.tap(&mut tt.taps);
        tt.take_checkpoint(target, true, None);
        tt
    }

    pub fn stats(&self) -> Stats {
        Stats {
            checkpoints: self.checkpoints.len(),
            bytes: self.used,
            oldest: self.checkpoints.first().map_or(Time::ZERO, |c| c.time),
            frontier: self.frontier_time(),
            ..self.stats
        }
    }

    /// The current position.
    pub fn here(&self) -> Pos {
        self.here
    }

    /// Whether the target is at the end of the recorded history (running on is live).
    pub fn at_frontier(&self) -> bool {
        self.cursor.entry == self.log.len() && !self.cursor.inside
    }

    fn frontier_time(&self) -> Time {
        // The last run's end (or where it stopped)
        let mut t = self.checkpoints.first().map_or(Time::ZERO, |c| c.time);
        for e in &self.log {
            if let Entry::Run { end, ended } = e {
                t = match ended {
                    Ended::At(p) => p.time(),
                    _ => *end,
                };
            }
        }
        if self.at_frontier() { t.max(self.here.time()) } else { t }
    }

    /// The checkpoints' times and names (oldest first).
    pub fn checkpoints(&self) -> Vec<(Time, Option<&str>)> {
        self.checkpoints.iter().map(|c| (c.time, c.name.as_deref())).collect()
    }

    fn take_checkpoint(&mut self, target: &mut dyn Rewindable, barrier: bool, name: Option<String>) {
        let start = std::time::Instant::now();
        let (state, bytes) = target.checkpoint(&mut self.memory);
        let c = Checkpoint {
            state,
            bytes,
            time: target.now(),
            cursor: self.cursor,
            taps: self.taps.list.iter().map(|t| t.calls()).collect(),
            inputs: values(&self.inputs),
            barrier,
            name,
        };
        self.used += bytes;
        self.checkpoints.push(c);
        self.base = self.checkpoints.len() - 1;
        self.last_checkpoint = target.now();
        self.stats.taken += 1;
        self.stats.taking_ns += start.elapsed().as_nanos() as u64;
        self.thin();
    }

    /// Drops checkpoints beyond the budget: the one whose neighbours are closest together for its age (so the
    /// spacing grows with age), never the first, a barrier or the latest; then the oldest.
    fn thin(&mut self) {
        while self.used > self.budget && self.checkpoints.len() > 2 {
            let now = self.checkpoints.last().unwrap().time;
            let n = self.checkpoints.len();
            let pick = (1..n - 1)
                .filter(|&i| !self.checkpoints[i].barrier && self.checkpoints[i].name.is_none() && i != self.base)
                .min_by(|&a, &b| {
                    let score = |i: usize| {
                        let c = &self.checkpoints;
                        let gap = (c[i + 1].time - c[i - 1].time).as_ps() as f64;
                        gap / ((now - c[i].time).as_ps() as f64 + 1.0)
                    };
                    score(a).total_cmp(&score(b))
                });
            let i = match pick {
                Some(i) => i,
                // Only barriers and named ones: history starts later
                None if self.base > 0 => 0,
                None => break,
            };
            let c = self.checkpoints.remove(i);
            self.used -= c.bytes;
            if self.base > i {
                self.base -= 1;
            }
        }
    }

    fn restore(&mut self, target: &mut dyn Rewindable, i: usize) {
        let c = &self.checkpoints[i];
        restore_from(target, c, &mut self.taps.list, &self.inputs, &mut self.collected, &mut self.memory);
        self.cursor = c.cursor;
        self.here = Pos::Boundary(c.time);
        self.last_inputs = c.inputs.clone();
        self.base = i;
        self.stats.restores += 1;
    }

    /// Notes host changes to the inputs since the last run (at a run's start).
    fn note_inputs(&mut self, target: &mut dyn Rewindable) {
        let now = values(&self.inputs);
        if now != self.last_inputs {
            if !self.at_frontier() {
                // The host changed the past
                let keep = now.clone();
                set_values(&self.inputs, &self.last_inputs);
                self.diverge(target);
                set_values(&self.inputs, &keep);
            }
            self.log.push(Entry::Inputs(now.clone()));
            self.cursor = Cursor { entry: self.log.len(), inside: false };
            self.last_inputs = now;
        }
    }

    /// Drops the recorded future: from here on the run is live.
    fn diverge(&mut self, target: &mut dyn Rewindable) {
        if self.at_frontier() {
            return;
        }
        log::info!("[{}] the past changed: the recorded history after it is dropped", self.here.time());
        let (cursor, here) = (self.cursor, self.here.time());
        let before = |c: &Checkpoint| c.cursor < cursor || (c.cursor == cursor && c.time <= here);
        while self.checkpoints.last().is_some_and(|c| !before(c)) {
            let c = self.checkpoints.pop().unwrap();
            self.used -= c.bytes;
        }
        let k = self.cursor.entry;
        if self.cursor.inside {
            if let Some(Entry::Run { ended, .. }) = self.log.get_mut(k) {
                *ended = Ended::At(self.here);
            }
            self.log.truncate(k + 1);
            self.cursor = Cursor { entry: k + 1, inside: false };
        } else {
            self.log.truncate(k);
        }
        self.base = self.base.min(self.checkpoints.len().saturating_sub(1));
        for t in &mut self.taps.list {
            t.truncate();
        }
        self.steps_cache = None;
        target.cut_collected(&mut self.collected);
    }

    /// The host changed something the log does not record (a fault, a vehicle's pose, a device's state): a
    /// checkpoint is taken here and kept, so going back never goes through the change. In the past, the recorded
    /// future is dropped first. Call between runs.
    pub fn changed(&mut self, target: &mut dyn Rewindable) {
        self.diverge(target);
        if target.at_boundary() {
            self.take_checkpoint(target, true, None);
        } else {
            log::warn!("time travel: a change in the middle of a slice; going back past it will not see it");
        }
    }

    /// Keeps a checkpoint here (a slice boundary) under `name`, never thinned out, to come back to with
    /// [`TimeTravel::rewind_to_saved`]. In the past it goes among the others; nothing is dropped.
    pub fn save(&mut self, target: &mut dyn Rewindable, name: &str) -> Result<(), String> {
        if !target.at_boundary() || !matches!(self.here, Pos::Boundary(_)) {
            return Err("checkpoints are kept at slice boundaries: not where a debugger stopped".into());
        }
        if self.at_frontier() {
            self.take_checkpoint(target, false, Some(name.to_string()));
            return Ok(());
        }
        let start = std::time::Instant::now();
        let (state, bytes) = target.checkpoint(&mut self.memory);
        let c = Checkpoint {
            state,
            bytes,
            time: target.now(),
            cursor: self.cursor,
            taps: self.taps.list.iter().map(|t| t.calls()).collect(),
            inputs: values(&self.inputs),
            barrier: false,
            name: Some(name.to_string()),
        };
        self.used += bytes;
        self.checkpoints.insert(self.base + 1, c);
        self.base += 1;
        self.stats.taken += 1;
        self.stats.taking_ns += start.elapsed().as_nanos() as u64;
        Ok(())
    }

    /// The time of the checkpoint saved as `name`.
    pub fn saved(&self, name: &str) -> Option<Time> {
        self.checkpoints.iter().rev().find(|c| c.name.as_deref() == Some(name)).map(|c| c.time)
    }

    /// Goes back (or forward, within the recorded history) to the checkpoint saved as `name`.
    pub fn rewind_to_saved(&mut self, target: &mut dyn Rewindable, name: &str) -> Option<Travelled> {
        let i = self.checkpoints.iter().rposition(|c| c.name.as_deref() == Some(name))?;
        let mut q = Quieted::default();
        target.quiet(true, &mut q);
        self.restore(target, i);
        target.quiet(false, &mut q);
        Some(Travelled::Arrived)
    }

    /// A debugger changes board `board`'s machine: in the past this drops the recorded future; the change is
    /// logged (replays make it again at the same position) and made.
    pub fn apply(&mut self, target: &mut dyn Rewindable, board: usize, change: Change) {
        self.diverge(target);
        change(&mut target.session(board).machine);
        self.log.push(Entry::Apply { board, change });
        self.cursor = Cursor { entry: self.log.len(), inside: false };
        self.steps_cache = None;
    }

    /// Runs `target` to `end` as the host asks: through the recorded history while in the past (stopping at a slice
    /// boundary at or before `end`, or at a breakpoint), live at the frontier, recording and taking checkpoints.
    pub fn run_until(&mut self, target: &mut dyn Rewindable, end: Time) -> Ran {
        if !self.at_frontier() {
            let inputs = values(&self.inputs);
            if inputs != self.last_inputs {
                self.note_inputs(target);
            } else {
                match self.replay_quiet(target, Goal::After(end), Probe::Report) {
                    Travelled::Frontier => {}
                    Travelled::Hit(h) => {
                        let board = match h.pos {
                            Pos::At { board, .. } => board,
                            Pos::Boundary(_) => 0,
                        };
                        return Ran::Stopped { board, reason: h.reason };
                    }
                    Travelled::Interrupted => return Ran::Interrupted,
                    Travelled::Arrived | Travelled::Beginning => return Ran::Reached,
                }
                if target.now() >= end {
                    return Ran::Reached;
                }
            }
        }
        self.live(target, end)
    }

    fn live(&mut self, target: &mut dyn Rewindable, end: Time) -> Ran {
        self.note_inputs(target);
        target.tap(&mut self.taps);
        self.steps_cache = None;
        let k = self.log.len();
        self.log.push(Entry::Run { end, ended: Ended::Completed });
        self.cursor = Cursor { entry: k, inside: true };
        let ran = loop {
            if target.at_boundary() && target.now() >= self.last_checkpoint + self.every && target.now() < end {
                self.take_checkpoint(target, false, None);
            }
            // (A robot running ahead goes on to where its boards meet anyway: the checkpoint is taken there)
            let split = self.split(target, end);
            match target.run_until_after(split, end) {
                Ran::Reached if split >= end || target.now() >= end => break Ran::Reached,
                Ran::Reached => continue,
                Ran::Stopped { board, reason } => {
                    let ended = if reason.is_debug_stop() {
                        let (core, time) = stop_position(target, board, &reason);
                        self.here = Pos::At { board, core, time };
                        Ended::At(self.here)
                    } else {
                        self.here = Pos::Boundary(target.now());
                        Ended::Other
                    };
                    self.log[k] = Entry::Run { end, ended };
                    break Ran::Stopped { board, reason };
                }
                Ran::Interrupted => {
                    self.here = Pos::Boundary(target.now());
                    self.log[k] = Entry::Run { end, ended: Ended::At(self.here) };
                    break Ran::Interrupted;
                }
            }
        };
        if ran == Ran::Reached {
            self.here = Pos::Boundary(target.now());
        }
        self.cursor = Cursor { entry: k + 1, inside: false };
        self.last_inputs = values(&self.inputs);
        ran
    }

    /// Where the next live run piece ends: at `end`, or earlier at the slice boundary where a checkpoint is due.
    fn split(&self, target: &dyn Rewindable, end: Time) -> Time {
        let due = self.last_checkpoint + self.every;
        let b0 = target.next_boundary(end);
        if b0 >= end {
            return end;
        }
        if b0 >= due || !target.exact_slices() {
            // (Slice by slice where slices do not end on a grid: a run to there is the same as one past it)
            return b0;
        }
        let slice = target.slice().as_ps().max(1);
        let k = (due - b0).as_ps().div_ceil(slice);
        Time(b0.as_ps() + k * slice).min(end)
    }

    // -------------------------------------------------------------------------------------------- replays

    fn replay_quiet(&mut self, target: &mut dyn Rewindable, goal: Goal, probe: Probe) -> Travelled {
        let mut q = Quieted::default();
        target.quiet(true, &mut q);
        let r = self.replay(target, goal, probe);
        target.quiet(false, &mut q);
        r
    }

    /// Replays the log from the cursor to `goal` (or the frontier), arming the recorded stops; stops that are not
    /// recorded ones (breakpoints, watchpoints) go to `probe`.
    fn replay(&mut self, target: &mut dyn Rewindable, goal: Goal, mut probe: Probe) -> Travelled {
        let start = std::time::Instant::now();
        let t0 = target.now();
        let r = self.replay_inner(target, goal, &mut probe);
        self.stats.replay_ns += start.elapsed().as_nanos() as u64;
        self.stats.replayed += target.now().saturating_sub(t0);
        for b in 0..target.boards() {
            target.session(b).machine.stop_at = [None; 2];
        }
        r
    }

    fn replay_inner(&mut self, target: &mut dyn Rewindable, goal: Goal, probe: &mut Probe) -> Travelled {
        // Breakpoints and watchpoints are put aside when nothing looks at them (batched runs are faster)
        let aside = matches!(probe, Probe::Ignore | Probe::Steps { .. });
        let mut kept = Vec::new();
        if aside {
            for b in 0..target.boards() {
                let m = &mut target.session(b).machine;
                kept.push((std::mem::take(&mut m.breakpoints), std::mem::take(&mut m.watchpoints)));
            }
        }
        let r = self.replay_entries(target, goal, probe);
        if aside {
            for (b, (bp, wp)) in kept.into_iter().enumerate() {
                let m = &mut target.session(b).machine;
                m.breakpoints = bp;
                m.watchpoints = wp;
            }
        }
        r
    }

    fn replay_entries(&mut self, target: &mut dyn Rewindable, goal: Goal, probe: &mut Probe) -> Travelled {
        if let Goal::Pos(Pos::Boundary(t)) | Goal::Time(t) | Goal::After(t) = goal
            && self.here == Pos::Boundary(target.now())
            && target.now() >= t
        {
            return Travelled::Arrived;
        }
        loop {
            self.cross_barrier(target);
            if self.at_frontier() {
                return Travelled::Frontier;
            }
            let k = self.cursor.entry;
            match self.log[k].clone() {
                Entry::Inputs(v) => {
                    set_values(&self.inputs, &v);
                    self.last_inputs = v;
                    self.cursor = Cursor { entry: k + 1, inside: false };
                }
                Entry::Apply { board, change } => {
                    change(&mut target.session(board).machine);
                    self.cursor = Cursor { entry: k + 1, inside: false };
                }
                Entry::Run { end, ended } => {
                    self.cursor = Cursor { entry: k, inside: true };
                    let (run_end, recorded) = match ended {
                        Ended::At(Pos::Boundary(t)) => (t, None),
                        Ended::At(p) => (end, Some(p)),
                        Ended::Completed | Ended::Other => (end, None),
                    };
                    // A goal time or boundary in this run: run slice by slice up to it (in one go where slices are
                    // on a grid)
                    let goal_time = match goal {
                        Goal::Time(t) => Some((t, Within::AtOrBefore)),
                        Goal::Pos(Pos::Boundary(t)) => Some((t, Within::Exact)),
                        Goal::After(t) => Some((t, Within::AtOrAfter)),
                        Goal::Pos(Pos::At { .. }) => None,
                    };
                    loop {
                        let mut stop = run_end;
                        if let Some((t, within)) = goal_time
                            && t < run_end
                        {
                            let b = target.next_boundary(run_end);
                            let arrived = match within {
                                Within::AtOrBefore => b > t,
                                Within::Exact | Within::AtOrAfter => target.now() >= t,
                            };
                            if arrived {
                                self.here = Pos::Boundary(target.now());
                                self.advance_base();
                                return Travelled::Arrived;
                            }
                            stop = match (target.exact_slices(), within) {
                                (false, _) => b,
                                (true, Within::AtOrAfter) => boundary_at_or_after(&*target, t, run_end),
                                (true, _) => boundary_at_or_before(&*target, t, run_end).unwrap_or(b),
                            };
                        }
                        self.arm(target, goal, recorded, probe);
                        match target.run_until(stop) {
                            Ran::Reached if stop < run_end => {
                                self.here = Pos::Boundary(target.now());
                                continue;
                            }
                            Ran::Reached => {
                                self.here = Pos::Boundary(target.now());
                                if recorded.is_some() {
                                    log::warn!("time travel: a replay did not stop where the run had (a bug)");
                                }
                                self.cursor = Cursor { entry: k + 1, inside: false };
                                self.advance_base();
                                if goal_time.is_some_and(|(t, _)| t <= target.now()) {
                                    return Travelled::Arrived;
                                }
                                break;
                            }
                            Ran::Interrupted => {
                                self.here = Pos::Boundary(target.now());
                                return Travelled::Interrupted;
                            }
                            Ran::Stopped { board, reason } if !reason.is_debug_stop() => {
                                self.here = Pos::Boundary(target.now());
                                self.cursor = Cursor { entry: k + 1, inside: false };
                                // (The run stopped there too, the same way)
                                return Travelled::Hit(Hit {
                                    pos: Pos::At { board, core: 0, time: target.now() },
                                    reason,
                                });
                            }
                            Ran::Stopped { board, reason } => {
                                let (core, time) = stop_position(target, board, &reason);
                                let pos = Pos::At { board, core, time };
                                self.here = pos;
                                let at = |p: Option<Pos>| match p {
                                    Some(Pos::At { board: b, core: c, time: t }) => {
                                        b == board && c == core && t <= time
                                    }
                                    _ => false,
                                };
                                let is_recorded = at(recorded);
                                let is_goal = matches!(goal, Goal::Pos(p) if at(Some(p)));
                                if is_recorded {
                                    self.cursor = Cursor { entry: k + 1, inside: false };
                                }
                                if is_goal {
                                    // A watchpoint hit by the step before: its stop would have come here
                                    if let Probe::Collect { hits, .. } = probe
                                        && let Some((c, addr, kind, at)) = target.session(board).machine.pending_watch()
                                    {
                                        let pc = target.session(board).machine.cores[c].pc();
                                        let reason = StopReason::Watchpoint { core: c as u8, pc, addr, kind, at };
                                        hits.push(Hit { pos: Pos::At { board, core: c, time: at }, reason });
                                    }
                                    return Travelled::Arrived;
                                }
                                if is_recorded {
                                    // The run stopped here (it may have been for a breakpoint that is set now): the
                                    // position is reached first, so a breakpoint here is a hit as well
                                    let m = &target.session(board).machine;
                                    let pc = m.cores[core].pc();
                                    if m.breakpoints.contains(&pc) && !m.cores[core].is_sleeping() {
                                        let hit = Hit { pos, reason: StopReason::Breakpoint { core: core as u8, pc } };
                                        match probe {
                                            Probe::Report => return Travelled::Hit(hit),
                                            Probe::Collect { hits, before } if pos != *before => hits.push(hit),
                                            _ => {}
                                        }
                                    }
                                    break;
                                }
                                if matches!(reason, StopReason::Reached { .. }) {
                                    // A probe's position
                                    if let Probe::Steps { board: b, core: c, steps, ephemeral, .. } = probe
                                        && *b == board
                                        && *c == core
                                    {
                                        steps.push(time);
                                        if std::mem::take(ephemeral) {
                                            let (state, bytes) = target.checkpoint(&mut self.memory);
                                            let cp = Checkpoint {
                                                state,
                                                bytes,
                                                time,
                                                cursor: self.cursor,
                                                taps: self.taps.list.iter().map(|t| t.calls()).collect(),
                                                inputs: values(&self.inputs),
                                                barrier: false,
                                                name: None,
                                            };
                                            self.steps_cache = Some(StepsCache { cp, board, core, steps: Vec::new() });
                                        }
                                        continue;
                                    }
                                    // Someone else's position (the host armed it): a stop like a breakpoint
                                    if !matches!(probe, Probe::Report) {
                                        continue;
                                    }
                                }
                                match probe {
                                    Probe::Report => {
                                        return Travelled::Hit(Hit { pos, reason });
                                    }
                                    Probe::Collect { hits, before } => {
                                        if pos != *before {
                                            hits.push(Hit { pos: hit_position(pos, &reason), reason });
                                        }
                                    }
                                    Probe::Ignore | Probe::Steps { .. } => {}
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Arms the positions the replay of the current run must stop at: the goal, the run's recorded stop and the
    /// probe's next step, the earliest per core.
    fn arm(&self, target: &mut dyn Rewindable, goal: Goal, recorded: Option<Pos>, probe: &Probe) {
        let mut want: Vec<(usize, usize, Time)> = Vec::new();
        for p in [recorded, if let Goal::Pos(p) = goal { Some(p) } else { None }].into_iter().flatten() {
            if let Pos::At { board, core, time } = p {
                want.push((board, core, time));
            }
        }
        if let Probe::Steps { board, core, from, steps, .. } = probe {
            let t = steps.last().map_or(*from, |&t| t + Time(1));
            want.push((*board, *core, t));
        }
        for (b, c, t) in want {
            let m = &mut target.session(b).machine;
            m.stop_at[c] = Some(m.stop_at[c].map_or(t, |o| o.min(t)));
        }
    }

    /// Moves the base checkpoint up to the latest one not after the current position.
    /// A replay that comes to a barrier checkpoint (a host change that is not in the log) goes on from it.
    fn cross_barrier(&mut self, target: &mut dyn Rewindable) {
        self.advance_base();
        let i = self.base;
        let c = &self.checkpoints[i];
        if c.barrier && c.cursor == self.cursor && !self.cursor.inside && self.here == Pos::Boundary(c.time) {
            // (Restoring it again where it was just restored changes nothing)
            self.restore(target, i);
        }
    }

    fn advance_base(&mut self) {
        let (cursor, now) = (self.cursor, self.here.time());
        while self.base + 1 < self.checkpoints.len() {
            let c = &self.checkpoints[self.base + 1];
            if c.cursor < cursor || (c.cursor == cursor && c.time <= now) {
                self.base += 1;
            } else {
                break;
            }
        }
    }

    // -------------------------------------------------------------------------------------------- travelling

    /// Goes to virtual time `t` (the last slice boundary at or before it), within the recorded history: back from a
    /// checkpoint, or forward by replaying. Past the frontier it stops there.
    pub fn rewind_to(&mut self, target: &mut dyn Rewindable, t: Time) -> Travelled {
        let i = self.checkpoints.iter().rposition(|c| c.time <= t).unwrap_or(0);
        let mut q = Quieted::default();
        target.quiet(true, &mut q);
        // From where it is if that is closer (going forward), else from the checkpoint
        if !(self.here.time() <= t && self.base >= i) {
            self.restore(target, i);
        }
        let r = if t < self.checkpoints[i].time {
            Travelled::Beginning
        } else {
            self.replay(target, Goal::Time(t), Probe::Ignore)
        };
        target.quiet(false, &mut q);
        r
    }

    /// Sets the base checkpoint to the latest one not after the current position.
    fn rebase(&mut self) {
        let (cursor, now) = (self.cursor, self.here.time());
        self.base = self
            .checkpoints
            .iter()
            .rposition(|c| c.cursor < cursor || (c.cursor == cursor && c.time <= now))
            .unwrap_or(0);
    }

    /// Goes to the cached step search's checkpoint and from there to its step at `prev`.
    fn goto_cached(&mut self, target: &mut dyn Rewindable, prev: Time) -> Travelled {
        let Some(c) = &self.steps_cache else { return Travelled::Beginning };
        let (board, core, at) = (c.board, c.core, c.cp.time);
        restore_from(target, &c.cp, &mut self.taps.list, &self.inputs, &mut self.collected, &mut self.memory);
        self.cursor = c.cp.cursor;
        self.last_inputs = c.cp.inputs.clone();
        self.here = Pos::At { board, core, time: at };
        self.stats.restores += 1;
        self.rebase();
        if at >= prev {
            return Travelled::Arrived;
        }
        self.replay(target, Goal::Pos(Pos::At { board, core, time: prev }), Probe::Ignore)
    }

    /// Goes to the position `pos` (one found by a search from checkpoint `from`).
    fn seek(&mut self, target: &mut dyn Rewindable, from: usize, pos: Pos) -> Travelled {
        self.restore(target, from);
        self.replay(target, Goal::Pos(pos), Probe::Ignore)
    }

    /// Runs forward while in the recorded history (a debugger's continue or step there): to `goal` if given (a
    /// position, e.g. a core's next step), stopping at breakpoints and watchpoints, or at the frontier.
    pub fn forward(&mut self, target: &mut dyn Rewindable, goal: Option<Pos>, until: Time) -> Travelled {
        let goal = match goal {
            Some(p) => Goal::Pos(p),
            None => Goal::After(until),
        };
        self.replay_quiet(target, goal, Probe::Report)
    }

    /// Goes back to the most recent breakpoint or watchpoint hit before the current position (on any core). A
    /// watchpoint's hit is the position of the access (before the accessing instruction, as GDB expects going
    /// backwards). With none, goes to the start of the history ([`Travelled::Beginning`]).
    pub fn reverse_continue(&mut self, target: &mut dyn Rewindable) -> Travelled {
        let mut q = Quieted::default();
        target.quiet(true, &mut q);
        let r = self.reverse_continue_inner(target);
        target.quiet(false, &mut q);
        r
    }

    fn reverse_continue_inner(&mut self, target: &mut dyn Rewindable) -> Travelled {
        self.steps_cache = None;
        let mut end = self.here;
        let mut i = self.base;
        loop {
            self.restore(target, i);
            let mut hits = Vec::new();
            let r = self.replay(target, Goal::Pos(end), Probe::Collect { hits: &mut hits, before: end });
            if r == Travelled::Interrupted {
                return r;
            }
            if let Some(&hit) = hits.last() {
                self.seek(target, i, hit.pos);
                return Travelled::Hit(hit);
            }
            if i == 0 {
                self.restore(target, 0);
                return Travelled::Beginning;
            }
            end = Pos::Boundary(self.checkpoints[i].time);
            i -= 1;
        }
    }

    /// Goes back one step (an instruction or an exception entry) of core `core` of board `board`: to the position of
    /// its last step before the current position. At the start of the history, [`Travelled::Beginning`].
    pub fn reverse_step(&mut self, target: &mut dyn Rewindable, board: usize, core: usize) -> Travelled {
        let mut q = Quieted::default();
        target.quiet(true, &mut q);
        let r = self.reverse_step_inner(target, board, core);
        target.quiet(false, &mut q);
        r
    }

    fn reverse_step_inner(&mut self, target: &mut dyn Rewindable, board: usize, core: usize) -> Travelled {
        let here = self.here;
        let t = here.time();
        // Steps found by the last search
        if let Some(c) = &self.steps_cache
            && (c.board, c.core) == (board, core)
            && c.cp.time < t
            && let Some(&prev) = c.steps.iter().rev().find(|&&s| s < t)
        {
            return self.goto_cached(target, prev);
        }
        self.steps_cache = None;
        // Window by window back from here (a window ends at the next checkpoint: replays never go through a
        // checkpoint, which may be a host change), the last part of each first
        let mut end = here;
        let mut i = self.base;
        loop {
            let start = self.checkpoints[i].time;
            let mut window = Time::from_us(2);
            loop {
                let from = end.time().saturating_sub(window).max(start);
                self.restore(target, i);
                let mut steps = Vec::new();
                let r = self.replay(
                    target,
                    Goal::Pos(end),
                    Probe::Steps { board, core, from, steps: &mut steps, ephemeral: true },
                );
                if r == Travelled::Interrupted {
                    return r;
                }
                if end == here {
                    steps.retain(|&s| s < t);
                } else {
                    // (A window's end is a boundary: every step found is before it)
                }
                if let Some(&prev) = steps.last() {
                    return match &mut self.steps_cache {
                        Some(c) => {
                            c.steps = steps;
                            self.goto_cached(target, prev)
                        }
                        None => self.seek(target, i, Pos::At { board, core, time: prev }),
                    };
                }
                self.steps_cache = None;
                if from <= start {
                    break;
                }
                window = Time(window.as_ps().saturating_mul(4));
            }
            if i == 0 {
                self.restore(target, 0);
                return Travelled::Beginning;
            }
            end = Pos::Boundary(self.checkpoints[i].time);
            i -= 1;
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Goal {
    Pos(Pos),
    /// The last slice boundary at or before a time.
    Time(Time),
    /// The first slice boundary at or after a time.
    After(Time),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Within {
    AtOrBefore,
    Exact,
    AtOrAfter,
}

/// Where a debugger stop left board `board`: the stopped core and the time on its clock.
fn stop_position(target: &mut dyn Rewindable, board: usize, reason: &StopReason) -> (usize, Time) {
    let core = match *reason {
        StopReason::Breakpoint { core, .. }
        | StopReason::BkptInstruction { core, .. }
        | StopReason::Watchpoint { core, .. }
        | StopReason::Reached { core, .. }
        | StopReason::Lockup { core, .. } => core as usize,
        _ => 0,
    };
    (core, target.session(board).machine.core_time(core))
}

/// The position a hit is gone back to: a watchpoint's is the access (the step that made it), a breakpoint's where
/// it stopped.
fn hit_position(pos: Pos, reason: &StopReason) -> Pos {
    match (pos, reason) {
        (Pos::At { board, core, .. }, StopReason::Watchpoint { at, .. }) => Pos::At { board, core, time: *at },
        _ => pos,
    }
}

/// The last slice boundary of a run to `end` from where `target` is that is at or before `t`; `None` if `t` comes
/// before the run's first slice end.
fn boundary_at_or_before(target: &dyn Rewindable, t: Time, end: Time) -> Option<Time> {
    let b0 = target.next_boundary(end);
    if t >= end {
        return Some(end);
    }
    if t < b0 {
        return None;
    }
    let slice = target.slice().as_ps().max(1);
    Some(Time(b0.as_ps() + (t - b0).as_ps() / slice * slice).min(end))
}

/// The first slice boundary of a run to `end` from where `target` is (on a grid) that is at or after `t`.
fn boundary_at_or_after(target: &dyn Rewindable, t: Time, end: Time) -> Time {
    let b0 = target.next_boundary(end);
    if t <= b0 {
        return b0;
    }
    let slice = target.slice().as_ps().max(1);
    Time(b0.as_ps() + (t - b0).as_ps().div_ceil(slice) * slice).min(end)
}

fn restore_from(
    target: &mut dyn Rewindable,
    c: &Checkpoint,
    taps: &mut [Box<dyn TapControl>],
    inputs: &[Inputs],
    collected: &mut Vec<crate::session::CollectedText>,
    memory: &mut Vec<Option<crate::session::SharedMemory>>,
) {
    target.restore(&*c.state, collected, memory);
    for (t, &calls) in taps.iter_mut().zip(&c.taps) {
        t.seek(calls);
    }
    // Taps added after the checkpoint start at their beginning
    for t in taps.iter_mut().skip(c.taps.len()) {
        t.seek(0);
    }
    set_values(inputs, &c.inputs);
}

fn values(inputs: &[Inputs]) -> Vec<Vec<f64>> {
    inputs.iter().map(|i| i.0.values().map(|h| *h.lock().unwrap()).collect()).collect()
}

fn set_values(inputs: &[Inputs], v: &[Vec<f64>]) {
    for (i, vals) in inputs.iter().zip(v) {
        for (h, &x) in i.0.values().zip(vals) {
            *h.lock().unwrap() = x;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hands in its frames once the bus asks up to their times.
    struct Script(Vec<(Time, CanFrame)>);

    impl CanEndpoint for Script {
        fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
            let (now, later) = std::mem::take(&mut self.0).into_iter().partition(|(t, _)| *t <= until);
            self.0 = later;
            now
        }
        fn receive(&mut self, _: &CanFrame, _: Time) {}
        fn idle_until(&self) -> Option<Time> {
            Some(self.0.iter().map(|(t, _)| *t).min().unwrap_or(Time::MAX))
        }
    }

    /// Live, a tapped endpoint tells when it sends as the endpoint does; replaying, from the recording: the time the
    /// next frames it gave were asked for; past the recording, the endpoint's own answer again.
    #[test]
    fn a_tapped_endpoint_tells_when_it_sends_live_and_in_replays() {
        let us = Time::from_us;
        let frames = [3, 7, 20].map(|t| (us(t), CanFrame::fd(0x10, false, &[1])));
        let log = TapLog::new();
        let mut e = TappedCan { inner: Box::new(Script(frames.to_vec())), log: log.clone() };
        assert_eq!(e.idle_until(), Some(us(3)));
        let sent: Vec<usize> = (1..=5).map(|k| e.take_tx(us(5 * k)).len()).collect();
        assert_eq!(sent, [1, 1, 0, 1, 0]);
        assert_eq!(e.idle_until(), Some(Time::MAX), "it has nothing more to send");
        // Back to the start: replayed
        TapControl::seek(&mut log.clone(), 0);
        assert_eq!(e.idle_until(), Some(us(5)), "the first call that gave frames asked up to 5 µs");
        assert_eq!(e.take_tx(us(5)).len(), 1);
        assert_eq!(e.idle_until(), Some(us(10)));
        assert_eq!(e.take_tx(us(10)).len(), 1);
        assert_eq!(e.idle_until(), Some(us(20)), "the call at 15 µs gave nothing");
        e.take_tx(us(15));
        e.take_tx(us(20));
        assert_eq!(e.idle_until(), Some(Time::MAX), "nothing more recorded, and the endpoint has nothing either");
        // The endpoint's own answer bounds the replay's: once past the recording, it is what it sends
        let mut f = TappedCan { inner: Box::new(Script(vec![(us(12), frames[0].1.clone())])), log: TapLog::new() };
        f.take_tx(us(5));
        f.take_tx(us(30));
        TapControl::seek(&mut f.log.clone(), 0);
        f.inner = Box::new(Script(vec![(us(8), frames[0].1.clone())]));
        assert_eq!(f.idle_until(), Some(us(8)));
        // One that cannot tell cannot when tapped either
        struct Unknown;
        impl CanEndpoint for Unknown {
            fn take_tx(&mut self, _: Time) -> Vec<(Time, CanFrame)> {
                Vec::new()
            }
            fn receive(&mut self, _: &CanFrame, _: Time) {}
        }
        let g = TappedCan { inner: Box::new(Unknown), log: TapLog::new() };
        assert_eq!(g.idle_until(), None);
    }
}
