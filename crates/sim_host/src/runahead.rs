//! Running a robot's boards ahead of each other, between the slices where they can see each other.
//!
//! Taking turns, a robot runs every board to the end of each 100 µs slice, then does the end of the slice: each bus
//! steps (arbitrates and delivers what finished by then), every board's host side and devices catch up, and the
//! vehicles step (see `crate::robot::end_slice`). On threads ([`crate::parallel`]) every slice still waits for
//! its slowest board, so boards whose heavy slices fall in different slices add up their worst slices.
//!
//! Boards see each other only through the end of a slice, and through it only in three ways:
//!
//! * **A bus step that carries a frame** (or an error, or one reported to a node recovering from bus-off). A step
//!   that carries none changes no node: it asks the nodes what they would send and reports the idle bus to them,
//!   which changes nothing in a controller that counts no dominant bits and is not bus-off
//!   ([`CanNode::idle_changes_nothing`]). Its effect on the bus itself (idle time reported, endpoints asked) does not
//!   depend on when the bus is stepped, only on what the nodes showed it at that boundary.
//! * **A vehicle**: it reads the ESCs' motors at the boundary and writes the pressure inputs of the boards it
//!   carries sensors for ([`VehicleSim::sensor_boards`]), which those boards read in the next slice.
//! * **Host hooks** (log lines, UART and USB lines): their order across boards, which observers see.
//!
//! A frame can only finish at the first boundary after it became ready plus its time on the wire
//! ([`CanBus::earliest_end`]); an endpoint says when it will next hand in a frame ([`CanEndpoint::idle_until`]), and
//! nothing can finish before that plus the shortest frame. So the robot runs in **windows**: from a boundary where
//! every board is committed, each board runs on through the slices on its own, on its thread, doing the machine
//! side of its end of each slice (devices, and taking out what RTT and the UARTs hold, `Session::collect`) and
//! noting what its CAN controllers show the bus and what its ESCs' motors are at each boundary. It stops at the
//! first boundary where one of its frames could finish, and tells the others (an atomic minimum), who stop there
//! too: the window ends at the first boundary where any frame could finish, or at an endpoint's next frame, at most
//! `MAX_WINDOW` slices on. A board that ran past that boundary before it was told is rolled back to a checkpoint
//! (`Session::rollback_point`: one at the window's first boundary, and another each time it has run for
//! `CHECKPOINT_WORK` since) and run again to the window's end; runs are deterministic, so it gets there exactly
//! as it would have, telling the others the same things.
//!
//! Meanwhile the calling thread **commits** the boundaries every board has gone past (nothing can end the window
//! there any more), as taking turns would have done them: each bus steps against stand-ins that answer as the
//! boards' controllers did at that boundary (`Proxy`: a stand-in that the step would change means the bounds above
//! were wrong, which is a bug, and panics), and each vehicle steps with the motors noted there. Boards with
//! vehicle-driven inputs (the camera cage's depth sensor) do not run ahead: the calling thread runs them slice by
//! slice as it commits, after the vehicle has moved, and if one of them turns out to send a frame earlier, the
//! window is cut there and the others are rolled back. Once the window has ended, each board's host side takes in
//! what was collected (`Session::deliver`), boundary by boundary in board order, so hooks see what they would
//! have, and the end of the slice at the window's last boundary is done as usual, everything live again.
//!
//! A board stopping (a breakpoint, a lockup) ends the window at the end of that slice, and is reported as on
//! threads: lowest index first, with the other boards run to the end of that slice.
//!
//! The result is the same run as taking turns: every bus step sees what it would have and changes what it would
//! have, every board runs every slice from the state it would have, and the host sides take in the same things in
//! the same order. Statistics too: a rolled-back machine gets back what its run loop carries
//! ([`rp2040::RunCarry`]).
//!
//! **Live inputs.** An endpoint fed from outside the simulation (SocketCAN, cannelloni: [`CanEndpoint::live`])
//! cannot tell when it will send, but hands in what reached it at the bus step that reads it. The commit steps the
//! buses at every boundary as taking turns does, so it reads such an endpoint where taking turns would; what it
//! handed in can finish at the next boundary at the earliest, so the window is cut where it can ([`Stats::live_cuts`])
//! and the boards that ran past are rolled back. A host frame thus enters the bus at a well-defined virtual time,
//! the boundary that read it, never in the past of a board that keeps what it did; given the same arrivals (which
//! step reads which frame: [`crate::canlog::Arrivals`] logs them), the run is the one taking turns gives. A board's
//! pseudo terminals are read by the board itself at each boundary (`Session::collect`), and a rollback gives back
//! what it read (`Session::unread_from`). In real time with live inputs, windows end at most `LIVE_LEAD` ahead of
//! the wall clock, so what arrives is read about when it arrives.
//!
//! **Observers.** Trace sinks get a board's events through a buffer (`HeldTrace`) that a rollback truncates and
//! the window's end passes on; profilers hold their samples likewise (`Profiler::hold`). Both see
//! only the run that was kept, as taking turns gives it.
//!
//! Robots qualify unless a board has a USB/IP server or an Ethernet cable (whose host sides answer the board at
//! once) or is traced to the instruction (too many events to hold), a bus endpoint is neither live nor can tell when
//! it will send, or there is an acoustic medium. [`crate::robot::Robot::run_ahead`] switches it off.

use std::any::Any;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Mutex, MutexGuard};
use std::thread::Thread;
use std::time::{Duration, Instant};

use rp2040::StopReason;
use sim_core::time::Time;
use sim_core::trace::{Category, TraceSink, VecSink};
#[cfg(doc)]
use sim_devices::can::CanBus;
use sim_devices::can::{BitRates, BusError, CanFrame, CanNode, TxRequest};
use sim_devices::dshot::{Esc, Motor};

use crate::parallel::{FIRST_REGROUP_SLICES, REGROUP_SLICES, lock, max_threads, wait_for};
use crate::profile::{ProfileMark, Profiler};
use crate::robot::{Robot, RobotBus, end_slice, power_on_by, step_bus};
use crate::session::{Pacing, RollbackPoint, Session, SliceOutput};
use crate::vehicle::VehicleSim;

#[cfg(doc)]
use sim_devices::can::CanEndpoint;

/// A board run ahead takes another checkpoint after running this long (host time, ns) since its last one.
const CHECKPOINT_WORK: u64 = 1_500_000;

/// Slices a window spans at most.
pub(crate) const MAX_WINDOW: usize = 64;

/// What running ahead did (see [`crate::robot::Robot::run_ahead_stats`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Windows committed, and the slices they spanned.
    pub windows: u64,
    pub slices: u64,
    /// Boards rolled back because the window turned out to end before where they had got to.
    pub rollbacks: u64,
    /// Windows cut short by a board that follows a vehicle.
    pub follower_cuts: u64,
    /// Windows cut short by a frame a live endpoint handed in ([`CanEndpoint::live`]).
    pub live_cuts: u64,
    /// Windows cut short as soon as a live endpoint had a frame waiting ([`CanEndpoint::waiting`]), before it was
    /// handed in.
    pub early_cuts: u64,
}

/// Whether `robot` can run its boards ahead of each other (see the module documentation).
pub(crate) fn possible(robot: &Robot) -> bool {
    robot.run_ahead
        && robot.medium.is_none()
        && robot.boards.iter().all(|b| b.can_run_ahead() && !b.machine.trace.wants(Category::Instruction))
        && robot.buses.iter().all(|rb| rb.endpoints.iter().all(|e| e.live() || e.idle_until().is_some()))
}

/// Whether something outside the simulation feeds `robot` as it runs, at times nobody can tell in advance (a live
/// endpoint): in real time its windows are kept close to the wall clock ([`LIVE_LEAD`]).
fn has_live_inputs(robot: &Robot) -> bool {
    robot.buses.iter().any(|rb| rb.endpoints.iter().any(|e| e.live()))
        || robot.boards.iter().any(Session::has_live_inputs)
}

/// Every category of trace event but instructions (too many to hold for a window: a board traced to the
/// instruction runs slice by slice).
const HELD_CATEGORIES: [Category; 5] =
    [Category::Exception, Category::Gpio, Category::Peripheral, Category::Unmodelled, Category::Device];

/// A board's trace sink while it runs ahead: the events go to a buffer that holds them until they are final (the
/// end of the window), so that a rollback can drop those of the run it takes back; then they go on to the sink.
struct HeldTrace {
    sink: Box<dyn TraceSink>,
    buffer: VecSink,
}

impl HeldTrace {
    /// Puts a buffer wanting what its sink wants in place of the machine's sink, if it wants anything. (The
    /// machine's own flags, set from what its sink wants, stay as they are.)
    fn hold(m: &mut rp2040::Machine) -> Option<HeldTrace> {
        let wanted: Vec<Category> = HELD_CATEGORIES.iter().copied().filter(|&c| m.trace.wants(c)).collect();
        if wanted.is_empty() {
            return None;
        }
        let buffer = VecSink::new(&wanted);
        let sink = std::mem::replace(&mut m.trace, Box::new(buffer.clone()));
        Some(HeldTrace { sink, buffer })
    }

    fn len(&self) -> usize {
        lock(&self.buffer.events).len()
    }

    /// Passes everything held on to the sink.
    fn flush(&mut self) {
        for (t, e) in self.buffer.take() {
            self.sink.event(t, e);
        }
    }
}

/// Where a board run ahead can be rolled back to: its boundary, the board there and what else running it changes.
struct LaneCheckpoint {
    at: usize,
    board: RollbackPoint,
    power: (Time, bool),
    /// Trace events held at the time.
    trace: usize,
    profile: Option<ProfileMark>,
}

/// In real time with live inputs, a window ends at most this far ahead of the wall clock (virtual time), as far
/// as taking turns lets virtual time run ahead of it before sleeping: what host tools send is read about as soon
/// as it arrives, and what the boards send goes out about when it is due.
pub(crate) const LIVE_LEAD: Time = Time::from_ms(2);

/// The last boundary of a window from `bounds` in real time with live inputs: at most [`LIVE_LEAD`] ahead of the
/// wall clock. Sleeps first while the robot is more than half of that ahead, so windows stay at least that long.
/// Returns at least 1, and the time slept.
fn wall_horizon(pacing: &mut Pacing, bounds: &Bounds) -> (usize, Duration) {
    let half = Time(LIVE_LEAD.as_ps() / 2);
    let mut wall = pacing.wall_virtual(bounds.start);
    let mut slept = Duration::ZERO;
    if bounds.start > wall + half {
        let ahead = bounds.start - half - wall;
        let start = Instant::now();
        std::thread::sleep(Duration::from_nanos(ahead.as_ns()));
        slept = start.elapsed();
        wall = pacing.wall_virtual(bounds.start);
    }
    let cap = wall + LIVE_LEAD;
    let last =
        if cap <= bounds.start { 0 } else { ((cap - bounds.start).as_ps() / bounds.slice.as_ps().max(1)) as usize };
    (last.max(1), slept)
}

/// What one CAN controller showed the bus at a boundary.
#[derive(Clone, Debug)]
struct NodeView {
    rates: Option<BitRates>,
    acks: bool,
    request: Option<TxRequest>,
    passive: bool,
    idle_ok: bool,
}

impl NodeView {
    fn of(n: &mut dyn CanNode) -> NodeView {
        NodeView {
            rates: n.bit_rates(),
            acks: n.acknowledges(),
            request: n.tx_request(),
            passive: n.error_passive(),
            idle_ok: n.idle_changes_nothing(),
        }
    }
}

/// Stands in for a CAN controller in a bus step at a boundary its board has run past: answers as the controller
/// did there, and notes whether the step would have changed it.
struct Proxy<'v> {
    view: &'v NodeView,
    changed: bool,
}

impl CanNode for Proxy<'_> {
    fn bit_rates(&self) -> Option<BitRates> {
        self.view.rates
    }
    fn acknowledges(&self) -> bool {
        self.view.acks
    }
    fn tx_request(&mut self) -> Option<TxRequest> {
        self.view.request.clone()
    }
    fn tx_done(&mut self, _token: u64, _at: Time, _acked: bool) {
        self.changed = true;
    }
    fn receive(&mut self, _frame: &CanFrame, _at: Time) {
        self.changed = true;
    }
    fn tx_error(&mut self, _token: u64, _at: Time, _error: BusError) {
        self.changed = true;
    }
    fn rx_error(&mut self, _at: Time, _error: BusError) {
        self.changed = true;
    }
    fn dominant_bits(&mut self, _at: Time, _bits: u64) {
        self.changed = true;
    }
    fn recessive_sequences(&mut self, _at: Time, _n: u64) {
        self.changed |= !self.view.idle_ok;
    }
    fn error_passive(&self) -> bool {
        self.view.passive
    }
    fn idle_changes_nothing(&self) -> bool {
        self.view.idle_ok
    }
}

/// What a board left at one boundary it ran through: what its controllers showed the bus, its ESCs' motors (for
/// the vehicles), and what its end of the slice took out of the machine.
struct Mark {
    nodes: Vec<NodeView>,
    motors: Vec<(usize, Motor)>,
    collected: SliceOutput,
}

/// Where a window ends: its number in the run and its last boundary, in one word so that both are read together
/// and the boundary can only come down within a window (an atomic minimum).
fn pack(window: u64, boundary: usize) -> u64 {
    window << 32 | boundary as u64
}

fn boundary_of(packed: u64) -> usize {
    (packed & 0xFFFF_FFFF) as usize
}

/// The boundaries of a window: from `start`, one every `slice`, the last one at `end`.
#[derive(Clone, Copy)]
struct Bounds {
    start: Time,
    slice: Time,
    end: Time,
}

impl Bounds {
    /// Boundary `j` (0 is the window's start).
    fn time(&self, j: usize) -> Time {
        Time(self.start.as_ps().saturating_add(self.slice.as_ps().saturating_mul(j as u64))).min(self.end)
    }

    /// Boundaries up to the end.
    fn count(&self) -> usize {
        (self.end - self.start).as_ps().div_ceil(self.slice.as_ps().max(1)) as usize
    }

    /// The first boundary at or after `t` (`usize::MAX` if it is after the end).
    fn at_or_after(&self, t: Time) -> usize {
        if t > self.end {
            usize::MAX
        } else if t <= self.start {
            0
        } else {
            (t - self.start).as_ps().div_ceil(self.slice.as_ps().max(1)) as usize
        }
    }
}

/// One board's run through a window.
struct Lane<'a> {
    session: &'a mut Session,
    power: &'a mut (Time, bool),
    /// The rates of its bus, to time its frames.
    rates: Option<BitRates>,
    /// The pin devices of its ESCs that a vehicle reads.
    escs: Vec<usize>,
    /// Run by the calling thread in the commit, after the vehicles, rather than ahead.
    follower: bool,
    bounds: Bounds,
    /// The window's number (see [`pack`]).
    window: u64,
    /// The machine is at this boundary...
    at: usize,
    /// ...and its end of the slice there is done (at the window's start the robot did it).
    done: bool,
    /// Per boundary from 1 that it went past: what it left there. Apart from the lane, so that the calling thread
    /// can commit boundaries every board has gone past while they run on.
    marks: &'a Mutex<Vec<Mark>>,
    /// The furthest boundary it reached in this window (rolled back or not: a board rerun from a checkpoint does
    /// the same again, so every boundary before where all have been is final).
    high: &'a AtomicUsize,
    /// The board at some of the boundaries it went past, before its end of the slice there: the first one, then
    /// one each time it has run for [`CHECKPOINT_WORK`] of host time since the last (rolling back reruns no more
    /// than that).
    checkpoints: Vec<LaneCheckpoint>,
    /// Its trace sink, while the events are held.
    trace: Option<HeldTrace>,
    /// Host time spent running it since its last checkpoint (ns).
    since_checkpoint: u64,
    /// It stopped in the slice that ends at this boundary.
    stop: Option<(usize, StopReason)>,
    /// Host time spent running it since the last regrouping (ns).
    spent: u64,
    /// Rolled back so far.
    rollbacks: u64,
    /// Ignores where the window ends until it is rolled back the first time (tests: forces rollbacks).
    blind: bool,
}

impl Lane<'_> {
    fn start_window(&mut self, bounds: Bounds, window: u64) {
        self.bounds = bounds;
        self.window = window;
        self.at = 0;
        self.done = true;
        lock(self.marks).clear();
        self.high.store(0, Ordering::Relaxed);
        self.checkpoints.clear();
        self.stop = None;
        self.blind = false;
    }

    /// Whether it went past where the window now ends (`w`), so it must be rolled back.
    fn overran(&self, w: usize) -> bool {
        self.at > w || (self.at == w && self.done && w > 0) || self.stop.is_some_and(|(s, _)| s > w)
    }

    /// Whether it can go on towards `w`.
    fn can_go(&self, w: usize) -> bool {
        self.stop.is_none() && self.at < w
    }

    /// Back to its last checkpoint at or before boundary `w`.
    fn rollback(&mut self, w: usize) {
        let k = self
            .checkpoints
            .iter()
            .rposition(|c| c.at <= w)
            .expect("a board that went past boundary 1 has a checkpoint");
        self.checkpoints.truncate(k + 1);
        let c = &self.checkpoints[k];
        self.session.roll_back(&c.board);
        self.session.unread_from(self.bounds.time(c.at));
        *self.power = c.power;
        self.at = c.at;
        if let Some(t) = &self.trace {
            lock(&t.buffer.events).truncate(c.trace);
        }
        if let (Some(p), Some(mark)) = (&mut self.session.profiler, &c.profile) {
            p.back_to(mark);
        }
        self.done = false;
        lock(self.marks).truncate(self.at - 1);
        self.since_checkpoint = 0;
        self.stop = None;
        self.rollbacks += 1;
        self.blind = false;
    }

    /// What its controllers show the bus now, and the first boundary at which one of their frames could finish
    /// (`Some(0)`: a step now could change one of them).
    fn views(&mut self) -> (Vec<NodeView>, Option<usize>) {
        let nodes: Vec<NodeView> = self.session.machine.can_nodes().into_iter().map(NodeView::of).collect();
        let mut first = None;
        for v in &nodes {
            if v.rates.is_some() && !v.idle_ok {
                first = Some(0);
            }
            if let (Some(r), Some(rates)) = (&v.request, self.rates) {
                // As `CanBus::earliest_end`
                let at = self.bounds.at_or_after(r.ready_at + rates.frame_time(&r.frame));
                if at != usize::MAX {
                    first = Some(first.map_or(at, |f: usize| f.min(at)));
                }
            }
        }
        (nodes, first)
    }

    /// The machine side of its end of the slice at the boundary it is at.
    fn end_slice(&mut self, nodes: Vec<NodeView>) {
        self.session.machine.update_devices();
        let collected = self.session.collect(self.bounds.time(self.at));
        let devices = &self.session.machine.sys.pin_devices;
        let motors = self
            .escs
            .iter()
            .filter_map(|&k| devices[k].as_any().downcast_ref::<Esc>().map(|e| (k, e.motor())))
            .collect();
        lock(self.marks).push(Mark { nodes, motors, collected });
        self.done = true;
    }

    /// Runs the slice from the boundary it is at to the next one.
    fn run_slice(&mut self, shared: &Shared) {
        let t = self.bounds.time(self.at + 1);
        let start = Instant::now();
        if power_on_by(self.session, self.power, t) {
            let run =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.session.run_machine_through_lockups(t)));
            match run {
                Ok(StopReason::TimeLimit) => {}
                Ok(other) => {
                    self.stop = Some((self.at + 1, other));
                    shared.announce(self.window, self.at + 1);
                }
                Err(panic) => {
                    lock(&shared.panic).get_or_insert(panic);
                    self.stop = Some((self.at + 1, StopReason::Requested));
                    shared.announce(self.window, self.at + 1);
                }
            }
        }
        let spent = start.elapsed().as_nanos() as u64;
        self.spent += spent;
        self.since_checkpoint += spent;
        self.at += 1;
        self.high.fetch_max(self.at, Ordering::Release);
        self.done = false;
    }

    /// One step towards the window's end: its end of the slice at the boundary it is at (unless something could
    /// change it there), then the next slice.
    fn step(&mut self, shared: &Shared) {
        if !self.done {
            let (nodes, first) = self.views();
            match first {
                Some(f) if f <= self.at => {
                    // Nobody can go past here, this board included
                    shared.announce(self.window, self.at);
                    self.blind = false;
                    return;
                }
                Some(f) => shared.announce(self.window, f),
                None => {}
            }
            if !self.blind && self.at >= boundary_of(shared.window.load(Ordering::Acquire)) {
                return;
            }
            if self.checkpoints.is_empty() || self.since_checkpoint >= CHECKPOINT_WORK {
                let start = Instant::now();
                self.checkpoints.push(LaneCheckpoint {
                    at: self.at,
                    board: self.session.rollback_point(),
                    power: *self.power,
                    trace: self.trace.as_ref().map_or(0, HeldTrace::len),
                    profile: self.session.profiler.as_ref().map(Profiler::mark),
                });
                self.since_checkpoint = 0;
                self.spent += start.elapsed().as_nanos() as u64;
            }
            self.end_slice(nodes);
        }
        self.run_slice(shared);
    }
}

/// Thread numbers: the calling thread is 0; followers run on it, in the commit.
const FOLLOWER: usize = usize::MAX - 1;
/// A board no thread runs (none: every board has a lane).
const NOBODY: usize = usize::MAX;

/// What the threads share for one run.
struct Shared<'a> {
    lanes: Vec<Mutex<Lane<'a>>>,
    /// Per lane: the thread that runs it this window.
    owner: Vec<AtomicUsize>,
    /// The current window and its last boundary ([`pack`]).
    window: AtomicU64,
    /// Per thread: the window (and boundary) its lanes all reached, once they have.
    parked: Vec<AtomicU64>,
    /// Workers leave.
    quit: std::sync::atomic::AtomicBool,
    panic: Mutex<Option<Box<dyn Any + Send>>>,
    threads: Mutex<Vec<Thread>>,
    /// Where the current window ended when it started (blind lanes run to there).
    first_end: AtomicUsize,
    /// Per lane: what it left at each boundary, and the furthest boundary it reached ([`Lane::marks`],
    /// [`Lane::high`]).
    marks: &'a [Mutex<Vec<Mark>>],
    highs: &'a [AtomicUsize],
}

impl Shared<'_> {
    /// Ends window `window` at boundary `j` or earlier; wakes everyone if that brought it down.
    fn announce(&self, window: u64, j: usize) {
        let v = pack(window, j);
        if self.window.fetch_min(v, Ordering::AcqRel) > v {
            for t in lock(&self.threads).iter() {
                t.unpark();
            }
        }
    }

    /// One step of thread `me`'s lanes towards the window's end: rolls back those that went past it, or runs the
    /// one furthest behind for a slice. Returns the window (packed) once they have all reached its end.
    fn step_group(&self, me: usize) -> Option<u64> {
        let packed = self.window.load(Ordering::Acquire);
        let w = boundary_of(packed);
        let mut next: Option<(usize, usize)> = None;
        let mut rolled = false;
        for (i, lane) in self.lanes.iter().enumerate() {
            if self.owner[i].load(Ordering::Relaxed) != me {
                continue;
            }
            let mut l = lock(lane);
            if l.overran(w) {
                let start = Instant::now();
                l.rollback(w);
                l.spent += start.elapsed().as_nanos() as u64;
                rolled = true;
            }
            let go = if l.blind { self.first_end.load(Ordering::Relaxed) } else { w };
            if l.can_go(go) && next.is_none_or(|(_, at)| l.at < at) {
                next = Some((i, l.at));
            }
        }
        match next {
            Some((i, _)) => lock(&self.lanes[i]).step(self),
            None if !rolled && self.window.load(Ordering::Acquire) == packed => return Some(packed),
            None => {}
        }
        None
    }

    /// A worker's life: run its lanes through every window, until told to leave.
    fn work(&self, me: usize, caller: &Thread) {
        let mut seen = 0;
        loop {
            let mut now = seen;
            wait_for(|| {
                now = self.window.load(Ordering::Acquire);
                now != seen || self.quit.load(Ordering::Acquire)
            });
            if self.quit.load(Ordering::Acquire) {
                return;
            }
            seen = loop {
                if let Some(p) = self.step_group(me) {
                    break p;
                }
            };
            self.parked[me].store(seen, Ordering::Release);
            caller.unpark();
        }
    }

    /// Whether every worker with lanes has brought them to the end of window `packed`.
    fn parked_at(&self, busy: &[bool], packed: u64) -> bool {
        busy.iter().enumerate().skip(1).all(|(t, b)| !b || self.parked[t].load(Ordering::Acquire) == packed)
    }

    /// Brings every lane run ahead to the current window's end: runs the calling thread's own lanes and waits for
    /// the workers, committing on the way whatever `commit` can (it returns whether it did anything, and is told
    /// whether only to see if it could).
    fn settle(&self, busy: &[bool], commit: &mut dyn FnMut(bool) -> bool) {
        loop {
            let own = self.step_group(0);
            let committed = commit(false);
            let Some(packed) = own else { continue };
            if self.window.load(Ordering::Acquire) == packed && self.parked_at(busy, packed) {
                return;
            }
            if !committed {
                wait_for(|| {
                    self.window.load(Ordering::Acquire) != packed || self.parked_at(busy, packed) || commit(true)
                });
            }
        }
    }
}

/// The workers of one run; tells them to leave when the run ends, by returning or by a panic.
struct Crew<'s, 'a> {
    shared: &'s Shared<'a>,
}

impl Drop for Crew<'_, '_> {
    fn drop(&mut self) {
        self.shared.quit.store(true, Ordering::Release);
        for t in lock(&self.shared.threads).iter() {
            t.unpark();
        }
    }
}

/// The first boundary of the window from `bounds` at which a frame an endpoint will hand in (or one waiting on a
/// bus) could finish; 1 if a bus cannot tell.
fn horizon(buses: &[RobotBus], bounds: &Bounds) -> usize {
    let mut first = Time::MAX;
    for rb in buses {
        let Some(queued) = rb.bus.earliest_queued_end() else { return 1 };
        first = first.min(queued);
        // A live endpoint's frames are handed in where the window is committed, which ends it where they can finish
        for e in rb.endpoints.iter().filter(|e| !e.live()) {
            let q = e.idle_until().unwrap_or(Time::ZERO);
            if q < Time::MAX {
                first = first.min(Time(q.as_ps().saturating_add(rb.bus.shortest_frame().as_ps())));
            }
        }
    }
    bounds.at_or_after(first).max(1)
}

/// [`Robot::run_until`] for a robot that can run its boards ahead of each other ([`possible`]). With `soft`, it
/// returns once a window ends at or after `soft` ([`Robot::run_until_after`]).
pub(crate) fn run_until(robot: &mut Robot, end: Time, soft: Option<Time>) -> Option<(usize, StopReason)> {
    let max_threads = max_threads(robot);
    let n = robot.boards.len();
    let live = has_live_inputs(robot);
    robot.balance.fit(n, max_threads);
    let mut follower = vec![false; n];
    let mut escs = vec![Vec::new(); n];
    for v in &robot.vehicles {
        // An external vehicle stays where it was put during a run, so the inputs it writes do not change
        for &b in v.sensor_boards().iter().filter(|_| !v.external) {
            follower[b] = true;
        }
        for &(b, k) in v.wiring().iter().flatten() {
            escs[b].push(k);
        }
    }
    let Robot {
        boards,
        buses,
        board_bus,
        power_on,
        vehicles,
        slice,
        now,
        realtime,
        pacing,
        balance,
        stop_flag,
        run_ahead_stress,
        run_ahead_stats,
        ..
    } = robot;
    let mut stress = run_ahead_stress.map(sim_devices::fault::Rng::new);
    let bounds = Bounds { start: *now, slice: *slice, end };
    let marks: Vec<Mutex<Vec<Mark>>> = (0..n).map(|_| Mutex::new(Vec::new())).collect();
    let highs: Vec<AtomicUsize> = (0..n).map(|_| AtomicUsize::new(0)).collect();
    let lanes = boards
        .iter_mut()
        .zip(power_on.iter_mut())
        .enumerate()
        .map(|(i, (session, power))| {
            let trace = HeldTrace::hold(&mut session.machine);
            if let Some(p) = &mut session.profiler {
                p.hold();
            }
            Mutex::new(Lane {
                session,
                power,
                rates: board_bus[i].map(|j| buses[j].bus.rates),
                escs: std::mem::take(&mut escs[i]),
                follower: follower[i],
                bounds,
                window: 0,
                at: 0,
                done: true,
                marks: &marks[i],
                high: &highs[i],
                checkpoints: Vec::new(),
                trace,
                since_checkpoint: 0,
                stop: None,
                spent: 0,
                rollbacks: 0,
                blind: false,
            })
        })
        .collect();
    let shared = Shared {
        lanes,
        owner: (0..n).map(|_| AtomicUsize::new(NOBODY)).collect(),
        window: AtomicU64::new(0),
        parked: (0..max_threads).map(|_| AtomicU64::new(0)).collect(),
        quit: Default::default(),
        panic: Mutex::new(None),
        threads: Mutex::new(vec![std::thread::current()]),
        first_end: AtomicUsize::new(0),
        marks: &marks,
        highs: &highs,
    };
    let shared = &shared;
    let caller = std::thread::current();
    let rollbacks_before = run_ahead_stats.rollbacks;
    std::thread::scope(|scope| {
        let _crew = Crew { shared };
        let mut workers = 0;
        let mut window = 0u64;
        let mut since_regroup = 0;
        let mut regroup_at = if balance.unmeasured() { FIRST_REGROUP_SLICES } else { REGROUP_SLICES };
        let mut busy = vec![false; max_threads];
        let mut wall = Instant::now();
        let mut paced = Duration::ZERO;
        let stopped = 'run: {
            while *now < end && !crate::signal::is_set(*stop_flag) {
                // A new window from here
                window += 1;
                let bounds = Bounds { start: *now, slice: *slice, end };
                let mut last = horizon(buses, &bounds).min(MAX_WINDOW).min(bounds.count());
                if *realtime && live {
                    let (wall, slept) = wall_horizon(pacing, &bounds);
                    last = last.min(wall);
                    paced += slept;
                }
                busy.fill(false);
                for (i, lane) in shared.lanes.iter().enumerate() {
                    let mut l = lock(lane);
                    l.start_window(bounds, window);
                    l.blind = stress.is_some();
                    let t = if l.follower { FOLLOWER } else { balance.threads()[i] };
                    shared.owner[i].store(t, Ordering::Relaxed);
                    if t < max_threads {
                        busy[t] = true;
                    }
                }
                while workers + 1 < max_threads && busy[workers + 1..].iter().any(|&b| b) {
                    workers += 1;
                    let me = workers;
                    let caller = caller.clone();
                    let h = std::thread::Builder::new()
                        .name(format!("board-worker-{me}"))
                        .spawn_scoped(scope, move || shared.work(me, &caller))
                        .expect("starting a board worker thread");
                    lock(&shared.threads).push(h.thread().clone());
                }
                shared.first_end.store(last, Ordering::Relaxed);
                shared.window.store(pack(window, last), Ordering::Release);
                for (t, thread) in lock(&shared.threads).iter().enumerate().skip(1) {
                    if busy[t] {
                        thread.unpark();
                    }
                }
                let mut committer = Committer { bounds, window, next: 1 };
                if let Some(rng) = &mut stress {
                    shared.settle(&busy, &mut |_| false);
                    // Any boundary will do as the window's end: cutting it anywhere rolls back whoever went past
                    let w = boundary_of(shared.window.load(Ordering::Acquire));
                    shared.announce(window, 1 + (rng.next_u64() % w as u64) as usize);
                    shared.settle(&busy, &mut |_| false);
                } else {
                    shared.settle(&busy, &mut |only_see| {
                        committer.advance(shared, buses, board_bus, vehicles, now, run_ahead_stats, only_see)
                    });
                }
                // Everything before the end is final now
                loop {
                    let settled = shared.window.load(Ordering::Acquire);
                    committer.advance(shared, buses, board_bus, vehicles, now, run_ahead_stats, false);
                    let packed = shared.window.load(Ordering::Acquire);
                    if packed == settled && committer.next >= boundary_of(packed) {
                        break;
                    }
                    // A follower cut the window short: the others go back to there
                    shared.settle(&busy, &mut |_| false);
                }
                let w = boundary_of(shared.window.load(Ordering::Acquire));
                let mut held: Vec<MutexGuard<Lane>> = shared.lanes.iter().map(lock).collect();
                // The followers' last slice
                for l in held.iter_mut().filter(|l| l.follower && l.at < w && l.stop.is_none()) {
                    l.run_slice(shared);
                }
                // The host sides take in what they collected, boundary by boundary, in board order
                for (i, l) in held.iter().enumerate() {
                    let n = lock(l.marks).len();
                    assert!(
                        n == w - 1 || l.follower && n == w,
                        "lane {i} has {n} marks, window ends at {w}, at {} done {} stop {:?}",
                        l.at,
                        l.done,
                        l.stop
                    );
                }
                let mut collected: Vec<std::vec::IntoIter<Mark>> =
                    held.iter().map(|l| std::mem::take(&mut *lock(l.marks)).into_iter()).collect();
                for _ in 1..w {
                    for (l, m) in held.iter_mut().zip(&mut collected) {
                        l.session.deliver(m.next().expect("a mark for every boundary").collected);
                    }
                }
                drop(collected);
                // Trace events and profile samples up to here are final
                for l in held.iter_mut() {
                    if let Some(t) = &mut l.trace {
                        t.flush();
                    }
                    if let Some(p) = &mut l.session.profiler {
                        p.commit();
                    }
                }
                run_ahead_stats.windows += 1;
                run_ahead_stats.rollbacks = held.iter().map(|l| l.rollbacks).sum::<u64>() + rollbacks_before;
                if let Some(panic) = lock(&shared.panic).take() {
                    std::panic::resume_unwind(panic);
                }
                if let Some(stop) =
                    held.iter().enumerate().find_map(|(i, l)| l.stop.filter(|(s, _)| *s == w).map(|(_, r)| (i, r)))
                {
                    break 'run Some(stop);
                }
                {
                    let mut boards: Vec<&mut Session> = held.iter_mut().map(|l| &mut *l.session).collect();
                    let t = bounds.time(w);
                    end_slice(buses, board_bus, &mut boards, vehicles, t);
                    *now = t;
                }
                drop(held);
                since_regroup += w;
                run_ahead_stats.slices += w as u64;
                if *realtime {
                    let start = Instant::now();
                    pacing.pace(*now);
                    paced += start.elapsed();
                }
                if since_regroup >= regroup_at {
                    let spent: Vec<u64> = shared.lanes.iter().map(|l| std::mem::take(&mut lock(l).spent)).collect();
                    let wall_ns = wall.elapsed().saturating_sub(paced).as_nanos() as u64;
                    balance.regroup(&spent, since_regroup, max_threads, wall_ns);
                    (wall, paced) = (Instant::now(), Duration::ZERO);
                    since_regroup = 0;
                    regroup_at = REGROUP_SLICES;
                }
                if soft.is_some_and(|s| *now >= s) {
                    break;
                }
            }
            None
        };
        // The boards' trace sinks and profilers as they were, with everything that is final now
        for lane in &shared.lanes {
            let mut l = lock(lane);
            if let Some(mut t) = l.trace.take() {
                t.flush();
                l.session.machine.trace = t.sink;
            }
            if let Some(p) = &mut l.session.profiler {
                p.release();
            }
        }
        stopped
    })
}

/// Commits a window boundary by boundary as taking turns would have done it, as soon as every board run ahead has
/// gone past a boundary: each bus steps as at that boundary, the followers run up to it and the vehicles step. (The
/// host sides take in what was collected once the window has ended, in order.)
struct Committer {
    bounds: Bounds,
    window: u64,
    /// The next boundary to commit.
    next: usize,
}

impl Committer {
    /// A live endpoint with a frame waiting hands it in at the next boundary committed, so the window ends where it
    /// can finish: cut it there now, before the boards run on past it (they would only be rolled back).
    fn notice_waiting(&mut self, shared: &Shared, buses: &mut [RobotBus], stats: &mut Stats) {
        let j = self.next;
        let w = boundary_of(shared.window.load(Ordering::Acquire));
        if j + 1 >= w {
            return;
        }
        let t = self.bounds.time(j);
        for rb in buses.iter_mut() {
            let shortest = rb.bus.shortest_frame();
            if rb.endpoints.iter_mut().any(|e| e.live() && e.waiting(t)) {
                let end = self.bounds.at_or_after(t + shortest).max(j + 1);
                if end < w {
                    shared.announce(self.window, end);
                    stats.early_cuts += 1;
                }
            }
        }
    }

    /// Commits every boundary it can now (with `only_see`, only tells whether it could commit one); returns
    /// whether it did (could).
    #[allow(clippy::too_many_arguments)]
    fn advance(
        &mut self,
        shared: &Shared,
        buses: &mut [RobotBus],
        board_bus: &[Option<usize>],
        vehicles: &mut [VehicleSim],
        now: &mut Time,
        stats: &mut Stats,
        only_see: bool,
    ) -> bool {
        let mut did = false;
        if !only_see {
            self.notice_waiting(shared, buses, stats);
        }
        loop {
            let j = self.next;
            if j >= boundary_of(shared.window.load(Ordering::Acquire)) {
                return did;
            }
            // Final once every board run ahead has gone past it: nothing can end the window there any more
            let leaders = || {
                shared.lanes.iter().enumerate().filter(|(i, _)| shared.owner[*i].load(Ordering::Relaxed) != FOLLOWER)
            };
            if leaders().any(|(i, _)| shared.highs[i].load(Ordering::Acquire) <= j) {
                return did;
            }
            if only_see {
                return true;
            }
            // What they showed the bus and their motors there (a board being rerun may not have got there again)
            let mut views: Vec<Vec<NodeView>> = vec![Vec::new(); shared.lanes.len()];
            let mut motors: Vec<Vec<(usize, Motor)>> = vec![Vec::new(); shared.lanes.len()];
            for (i, _) in leaders() {
                let marks = lock(&shared.marks[i]);
                let Some(m) = marks.get(j - 1) else { return did };
                views[i] = m.nodes.clone();
                motors[i] = m.motors.clone();
            }
            // The followers to boundary j; a frame or a stop of theirs may end the window sooner
            let mut followers: Vec<(usize, MutexGuard<Lane>)> = shared
                .lanes
                .iter()
                .enumerate()
                .filter(|(i, _)| shared.owner[*i].load(Ordering::Relaxed) == FOLLOWER)
                .map(|(i, l)| (i, lock(l)))
                .collect();
            let w = boundary_of(shared.window.load(Ordering::Acquire));
            for (i, l) in followers.iter_mut() {
                if l.at < j && l.stop.is_none() {
                    l.run_slice(shared);
                }
                if l.stop.is_some() {
                    shared.announce(self.window, j);
                    continue;
                }
                let (nodes, first) = l.views();
                if let Some(f) = first {
                    shared.announce(self.window, f.max(j));
                }
                views[*i] = nodes.clone();
                lock(l.marks).push(Mark { nodes, motors: Vec::new(), collected: SliceOutput::default() });
            }
            if boundary_of(shared.window.load(Ordering::Acquire)) < w {
                stats.follower_cuts += 1;
                if j >= boundary_of(shared.window.load(Ordering::Acquire)) {
                    return true;
                }
            }
            let t = self.bounds.time(j);
            // Each bus as at boundary j
            for (bj, rb) in buses.iter_mut().enumerate() {
                let mut proxies: Vec<Proxy> = Vec::new();
                for (i, on) in board_bus.iter().enumerate() {
                    if *on == Some(bj) {
                        proxies.extend(views[i].iter().map(|view| Proxy { view, changed: false }));
                    }
                }
                let mut nodes: Vec<&mut dyn CanNode> = proxies.iter_mut().map(|p| p as &mut dyn CanNode).collect();
                step_bus(rb, &mut nodes, t);
                drop(nodes);
                assert!(
                    proxies.iter().all(|p| !p.changed),
                    "bus {} changed a node at {t} in a window that should have been quiet there",
                    rb.bus.name
                );
                // What a live endpoint handed in here (ready now) can finish at the next boundary at the earliest
                if rb.endpoints.iter().any(|e| e.live()) {
                    let end = rb.bus.earliest_queued_end().map_or(j + 1, |e| self.bounds.at_or_after(e));
                    if end != usize::MAX && end.max(j + 1) < boundary_of(shared.window.load(Ordering::Acquire)) {
                        shared.announce(self.window, end.max(j + 1));
                        stats.live_cuts += 1;
                    }
                }
            }
            // The followers' end of the slice
            for (_, l) in followers.iter_mut() {
                l.session.machine.update_devices();
                let c = l.session.collect(t);
                lock(l.marks).last_mut().expect("its mark at j").collected = c;
            }
            // The vehicles, with the motors as they were at boundary j
            let motor = |b: usize, k: usize| -> Option<Motor> {
                match followers.iter().find(|(i, _)| *i == b) {
                    Some((_, l)) => l.session.machine.sys.pin_devices[k].as_any().downcast_ref::<Esc>().map(Esc::motor),
                    None => motors[b].iter().find(|(d, _)| *d == k).map(|(_, m)| *m),
                }
            };
            for v in vehicles.iter_mut() {
                v.advance_with(t, &motor);
            }
            for (_, l) in followers.iter_mut() {
                l.done = true;
            }
            *now = t;
            self.next = j + 1;
            did = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::cpu::Cpu;
    use std::sync::{Arc, Mutex};

    #[test]
    fn boundaries_of_a_window() {
        let b = Bounds { start: Time::from_us(50), slice: Time::from_us(100), end: Time::from_us(420) };
        assert_eq!(b.count(), 4);
        let times: Vec<u64> = (0..=4).map(|j| b.time(j).as_us()).collect();
        assert_eq!(times, [50, 150, 250, 350, 420], "as taking turns steps them: the last one short");
        assert_eq!(b.time(9), b.end);
        assert_eq!(b.at_or_after(Time::ZERO), 0);
        assert_eq!(b.at_or_after(Time::from_us(50)), 0);
        assert_eq!(b.at_or_after(Time::from_us(51)), 1);
        assert_eq!(b.at_or_after(Time::from_us(150)), 1);
        assert_eq!(b.at_or_after(Time::from_us(151)), 2);
        assert_eq!(b.at_or_after(Time::from_us(420)), 4);
        assert_eq!(b.at_or_after(Time::from_us(421)), usize::MAX, "after the end");
        // Within a window, packed ends compare by boundary; a later window is always above
        assert!(pack(3, 7) < pack(3, 8) && pack(3, 60) < pack(4, 1));
        assert_eq!(boundary_of(pack(5, 17)), 17);
    }

    #[test]
    fn a_proxy_answers_as_the_node_did_and_notes_changes() {
        let request = TxRequest { token: 4, frame: CanFrame::fd(0x12, false, &[1]), ready_at: Time::from_us(3) };
        let rates = BitRates { nominal: 1_000_000, data: Some(5_000_000) };
        let view = NodeView { rates: Some(rates), acks: true, request: Some(request), passive: false, idle_ok: true };
        let mut p = Proxy { view: &view, changed: false };
        assert_eq!(p.bit_rates(), Some(rates));
        assert!(p.acknowledges() && !p.error_passive() && p.idle_changes_nothing());
        assert_eq!(p.tx_request().map(|r| (r.token, r.ready_at)), Some((4, Time::from_us(3))));
        p.recessive_sequences(Time::ZERO, 3);
        assert!(!p.changed, "an idle report changes a node that says it does not");
        let calls: [&dyn Fn(&mut Proxy); 6] = [
            &|p| p.tx_done(4, Time::ZERO, true),
            &|p| p.receive(&CanFrame::fd(1, false, &[]), Time::ZERO),
            &|p| p.tx_error(4, Time::ZERO, BusError::StuckDominant),
            &|p| p.rx_error(Time::ZERO, BusError::StuckDominant),
            &|p| p.dominant_bits(Time::ZERO, 1),
            &|p| {
                let view = NodeView { idle_ok: false, ..p.view.clone() };
                let mut q = Proxy { view: &view, changed: false };
                q.recessive_sequences(Time::ZERO, 1);
                p.changed = q.changed;
            },
        ];
        for call in calls {
            let mut p = Proxy { view: &view, changed: false };
            call(&mut p);
            assert!(p.changed);
        }
    }

    fn boot(name: &str) -> Option<Session> {
        let repo = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let path = crate::titan_rs_image(&repo, name)?;
        let image = sim_core::image::Image::load(&path).unwrap();
        let mut m = rp2040::Machine::new(rp2040::MachineConfig {
            bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
            ..Default::default()
        });
        m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
        m.load(&image).unwrap();
        m.direct_boot().unwrap();
        Some(Session::new(m, &image))
    }

    /// What a session holds after a run, and every line its hooks saw.
    fn outcome(s: &Session, lines: &Mutex<Vec<String>>) -> String {
        let m = &s.machine;
        let regs: Vec<u32> = (0..m.cores[0].num_regs()).map(|n| m.cores[0].read_reg(n)).collect();
        format!(
            "{:?} {:?} {:?} {} {regs:x?} {} {} {} {:?}",
            s.logs,
            s.rtt_text,
            s.uart_text,
            m.now,
            {
                use std::hash::{Hash, Hasher};
                let mut h = std::collections::hash_map::DefaultHasher::new();
                m.sys.sram.hash(&mut h);
                m.sys.flash[..].hash(&mut h);
                h.finish()
            },
            m.spin_skipped,
            m.cores[0].instructions(),
            lines.lock().unwrap()
        )
    }

    fn hooked(s: &mut Session) -> Arc<Mutex<Vec<String>>> {
        let lines = Arc::new(Mutex::new(Vec::new()));
        let l = lines.clone();
        s.on_log = Some(Box::new(move |t, line| l.lock().unwrap().push(format!("[{t}] {line}"))));
        lines
    }

    /// Taking out what RTT and the UARTs hold at each boundary and taking it in later is what polling them there
    /// does.
    #[test]
    fn collecting_then_delivering_is_polling() {
        let Some(mut a) = boot("template") else { return };
        let mut b = boot("template").unwrap();
        let (la, lb) = (hooked(&mut a), hooked(&mut b));
        let mut kept = Vec::new();
        for k in 1..=3000u64 {
            let t = Time::from_us(100 * k);
            a.run_machine(t);
            a.machine.update_devices();
            a.poll_rtt();
            a.poll_uarts();
            b.run_machine(t);
            b.machine.update_devices();
            kept.push(b.collect(t));
            if k % 37 == 0 {
                for c in kept.drain(..) {
                    b.deliver(c);
                }
            }
        }
        for c in kept.drain(..) {
            b.deliver(c);
        }
        assert!(la.lock().unwrap().len() > 3, "{:?}", la.lock().unwrap());
        assert_eq!(outcome(&a, &la), outcome(&b, &lb));
    }

    /// A profiler holding its samples drops those of a run that is rolled back and takes the same ones again on the
    /// rerun: the profile is that of the run without the rollback.
    #[test]
    fn a_held_profile_rolled_back_is_the_profile_of_one_run() {
        let Some(mut a) = boot("esc_board") else { return };
        let mut b = boot("esc_board").unwrap();
        for s in [&mut a, &mut b] {
            s.run_machine(Time::from_ms(5));
            s.start_profile(Time::from_us(23));
        }
        a.run_machine(Time::from_ms(40));
        let p = b.profiler.as_mut().unwrap();
        p.hold();
        b.run_machine(Time::from_ms(20));
        assert_eq!(b.profiler.as_ref().unwrap().samples(0), 0, "held, not counted");
        b.profiler.as_mut().unwrap().commit();
        assert!(b.profiler.as_ref().unwrap().samples(0) > 0, "committed, counted");
        b.run_machine(Time::from_ms(25));
        let (c, mark) = (b.rollback_point(), b.profiler.as_ref().unwrap().mark());
        b.run_machine(Time::from_ms(37));
        b.roll_back(&c);
        b.profiler.as_mut().unwrap().back_to(&mark);
        b.run_machine(Time::from_ms(40));
        b.profiler.as_mut().unwrap().release();
        let (pa, pb) = (a.profiler.as_ref().unwrap(), b.profiler.as_ref().unwrap());
        assert!(pa.samples(0) > 1000);
        assert_eq!(format!("{:?}", pa.report()), format!("{:?}", pb.report()));
    }

    /// In real time with live inputs a window ends at most `LIVE_LEAD` ahead of the wall clock, after sleeping while
    /// the robot is more than half of that ahead.
    #[test]
    fn live_windows_end_near_the_wall_clock() {
        let mut pacing = Pacing::default();
        let start = Time::from_ms(500);
        let bounds = |start: Time| Bounds { start, slice: Time::from_us(100), end: Time::from_secs(9) };
        // Pacing starts here: 2 ms ahead of it at most, 20 slices
        let (last, slept) = wall_horizon(&mut pacing, &bounds(start));
        assert!((19..=20).contains(&last), "{last}");
        assert!(slept < Duration::from_millis(1));
        // 3 ms ahead of the wall clock: it sleeps until it is 1 ms ahead, then runs to 2 ms ahead
        let t0 = Instant::now();
        let (last, slept) = wall_horizon(&mut pacing, &bounds(start + Time::from_ms(3)));
        assert!(slept >= Duration::from_micros(1900) && t0.elapsed() >= Duration::from_micros(1900), "{slept:?}");
        assert!((9..=10).contains(&last), "{last}");
        // Far behind: from where the wall clock is, 2 ms on (the window's other bounds cut that)
        std::thread::sleep(Duration::from_millis(30));
        let (last, slept) = wall_horizon(&mut pacing, &bounds(start + Time::from_ms(1)));
        assert!(last > 300 && slept == Duration::ZERO, "{last}");
    }

    /// A session rolled back to a checkpoint and run again gets to exactly where it got the first time, statistics
    /// included; its flash is put back if it was written meanwhile, and kept otherwise.
    #[test]
    fn a_rolled_back_session_reruns_the_same() {
        let Some(mut s) = boot("esc_board") else { return };
        s.run_machine(Time::from_ms(20));
        let at = s.machine.now;
        let c = s.rollback_point();
        let lines = hooked(&mut s);
        s.run_machine(Time::from_ms(140));
        s.poll_rtt();
        let first = outcome(&s, &lines);
        let first_flash = s.machine.sys.flash.version();
        for written in [false, true] {
            if written {
                s.machine.sys.flash[0x1F_0000] ^= 0xFF;
            }
            s.roll_back(&c);
            assert_eq!(s.machine.now, at);
            assert_eq!(s.machine.sys.flash.version(), first_flash, "the same contents as at the checkpoint");
            s.logs.clear();
            s.rtt_text.clear();
            lines.lock().unwrap().clear();
            s.run_machine(Time::from_ms(140));
            s.poll_rtt();
            assert_eq!(outcome(&s, &lines), first, "written: {written}");
        }
    }

    /// What a board ran before it was rolled back does not count as covered: after a rollback and a rerun, the
    /// coverage is what the rerun alone gives.
    #[test]
    fn rolled_back_execution_is_not_covered() {
        let Some(mut s) = boot("esc_board") else { return };
        s.machine.enable_coverage();
        s.run_machine(Time::from_ms(5));
        let mut fresh = s.snapshot();
        let c = s.rollback_point();
        // A run the rollback undoes, further than the rerun goes (so it runs code the rerun does not)
        s.run_machine(Time::from_ms(300));
        let undone = s.machine.coverage.clone().unwrap();
        s.roll_back(&c);
        s.run_machine(Time::from_ms(8));
        fresh.run_machine(Time::from_ms(8));
        assert_ne!(Some(&undone), s.machine.coverage.as_ref(), "the undone run covered other code");
        assert_eq!(s.machine.coverage, fresh.machine.coverage);
        assert_eq!(s.machine.cores[0].instructions(), fresh.machine.cores[0].instructions());
    }
}
