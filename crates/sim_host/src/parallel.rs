//! Running a robot's boards side by side.
//!
//! Within one slice of [`crate::robot::Robot::run_until`] the boards do not see each other: frames between them are
//! delivered by the buses after the slice, and each machine (its cores, peripherals and devices) is its own. So
//! they can run on several threads and end the slice exactly as they would have one after the other. Boards in an
//! acoustic medium are the exception (a hydrophone can hear a projector of another board within the same slice
//! over a short enough path), so robots with a medium run their boards in turn.
//!
//! Slices are short (100 µs of virtual time), so what it costs to hand the boards out each slice decides whether
//! threads pay at all. For each call of `run_until`, scoped worker threads start once and each runs a fixed group
//! of boards, in place, for every slice: nothing is moved between threads. Each slice:
//!
//! 1. the calling thread publishes the slice's end and wakes the workers that have boards;
//! 2. every thread (the calling one included) runs its own group to the end of the slice;
//! 3. the calling thread waits for the others, then does the end of the slice alone (CAN buses, devices, host
//!    sides, vehicles, pacing), exactly as when the boards take turns.
//!
//! Waiting spins first, then yields, then parks: a slice is shorter than a wake-up from a park on many machines.
//! Each board sits behind a [`Mutex`] that only one thread ever wants at a time (the barrier orders them), so no
//! `unsafe` is needed to hand the boards to the workers and back.
//!
//! Which board runs where follows what each one costs: every board's host time is measured, and every 64 slices
//! the groups are recomputed (longest first onto the least loaded thread, for each thread count up to the limit,
//! counting a cost for each extra thread to keep in step). The count that gives the shortest slice is used, so a
//! robot with one heavy board stays on one thread. A new grouping is taken only if it is clearly better. If the
//! slices on threads take nearly as long on the wall clock as the boards' costs added up (the cores are busy with
//! other programs), every board goes back to the calling thread for eight regroupings before threads are tried
//! again (twice as long each time in a row that they do not pay, up to 512 regroupings: 3.3 s of virtual time). None of this changes what the boards do, only where they run.

use std::any::Any;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};
use std::thread::Thread;
use std::time::{Duration, Instant};

use rp2040::StopReason;
use sim_core::time::Time;

use crate::robot::{Robot, end_slice, power_on_by};
use crate::session::Session;

/// Threads for `threads` = 0: one per core.
fn cores() -> usize {
    static N: OnceLock<usize> = OnceLock::new();
    *N.get_or_init(|| std::thread::available_parallelism().map_or(1, |n| n.get()))
}

/// The most threads `robot` may use.
fn max_threads(robot: &Robot) -> usize {
    let n = if robot.threads == 0 { cores() } else { robot.threads };
    n.min(robot.boards.len()).max(1)
}

/// Runs shorter than this many slices take turns on the calling thread (starting the workers would cost more).
const MIN_SLICES: u64 = 8;

/// Whether running `robot` to `end` should go through [`run_until`].
pub(crate) fn worth_it(robot: &Robot, end: Time) -> bool {
    robot.medium.is_none() && max_threads(robot) > 1 && end > robot.now + Time(robot.slice.as_ps() * MIN_SLICES)
}

/// Host time per slice that each extra thread is assumed to cost (waking it, waiting for it), in nanoseconds:
/// a grouping on more threads must win more than this per extra thread.
const THREAD_COST_NS: f64 = 3000.0;

/// Slices between regroupings; the first comes sooner, when nothing is known about the boards yet.
const REGROUP_SLICES: usize = 64;
const FIRST_REGROUP_SLICES: usize = 16;

/// Regroupings to wait, with every board on the calling thread, before trying threads again after they did not pay;
/// twice as many each time they again do not, up to the maximum.
const HOLD_REGROUPS: usize = 8;
const MAX_HOLD_REGROUPS: usize = 512;

/// A board that does not run this slice (not switched on yet).
const IDLE: usize = usize::MAX;

/// How a robot's boards are spread over threads, and what they were measured to cost. Kept between runs.
#[derive(Debug, Clone, Default)]
pub struct Balance {
    /// Per board: host time per slice (ns), smoothed over regroupings.
    cost: Vec<f64>,
    /// Per board: the thread it runs on (0: the calling thread).
    thread: Vec<usize>,
    /// Regroupings still to skip with every board on the calling thread, after threads did not pay.
    hold: usize,
    /// How many to skip the next time they do not pay (doubles each time in a row, up to [`MAX_HOLD_REGROUPS`]).
    next_hold: usize,
    /// Never regroup: the threads were given ([`Robot::fix_threads`]).
    fixed: bool,
}

impl Balance {
    /// Each board's thread (0 is the calling thread).
    pub fn threads(&self) -> &[usize] {
        &self.thread
    }

    fn fit(&mut self, boards: usize, max_threads: usize) {
        if self.thread.len() != boards || self.thread.iter().any(|&t| t >= max_threads) {
            self.thread = vec![0; boards];
            self.cost.clear();
            self.fixed = false;
        }
    }

    /// Puts board `i` on thread `thread[i]` for good.
    pub(crate) fn fix(&mut self, thread: Vec<usize>) {
        self.thread = thread;
        self.fixed = true;
    }

    /// Takes in what each board cost over the last `slices` slices (ns) and how long those slices took on the wall
    /// clock (`wall`, ns, real-time pacing left out), and regroups if that is clearly better. If threads did not
    /// make the slices clearly shorter than running the boards in turn would have been (other programs keep the
    /// cores busy, or the threads share cores), every board goes back to the calling thread for a while.
    fn regroup(&mut self, spent: &[u64], slices: usize, max_threads: usize, wall: u64) {
        if self.fixed {
            return;
        }
        let per_slice = |ns: u64| ns as f64 / slices.max(1) as f64;
        let now: Vec<f64> = spent.iter().map(|&ns| per_slice(ns)).collect();
        if self.cost.len() == now.len() {
            for (c, n) in self.cost.iter_mut().zip(&now) {
                *c = 0.5 * *c + 0.5 * n;
            }
        } else {
            self.cost = now.clone();
        }
        let threads_used = (0..max_threads).filter(|t| self.thread.contains(t)).count();
        if threads_used > 1 {
            if per_slice(wall) > 0.9 * now.iter().sum::<f64>() {
                self.thread.fill(0);
                self.hold = self.next_hold.max(HOLD_REGROUPS);
                self.next_hold = (self.hold * 2).min(MAX_HOLD_REGROUPS);
                return;
            }
            self.next_hold = HOLD_REGROUPS;
        }
        if self.hold > 0 {
            self.hold -= 1;
            return;
        }
        let (groups, best) = plan(&self.cost, max_threads);
        if best < 0.95 * score(&self.cost, &self.thread) {
            self.thread = relabel(&groups, &self.thread, &self.cost, max_threads);
        }
    }
}

/// The time a slice takes with boards costing `cost` on threads `thread`: the busiest thread, plus the cost of
/// keeping the others in step.
fn score(cost: &[f64], thread: &[usize]) -> f64 {
    let used = thread.iter().max().map_or(1, |&t| t + 1);
    let mut load = vec![0.0; used];
    for (&c, &t) in cost.iter().zip(thread) {
        load[t] += c;
    }
    let threads = load.iter().filter(|&&l| l > 0.0).count().max(1);
    load.iter().cloned().fold(0.0, f64::max) + THREAD_COST_NS * (threads - 1) as f64
}

/// The best grouping of boards costing `cost` on up to `max_threads` threads (each board's group, groups numbered
/// from 0), and its score: for each thread count, the costliest board first onto the least loaded thread.
fn plan(cost: &[f64], max_threads: usize) -> (Vec<usize>, f64) {
    let mut order: Vec<usize> = (0..cost.len()).collect();
    order.sort_by(|&a, &b| cost[b].total_cmp(&cost[a]).then(a.cmp(&b)));
    let mut best = (vec![0; cost.len()], f64::INFINITY);
    for k in 1..=max_threads.max(1) {
        let mut load = vec![0.0f64; k];
        let mut group = vec![0; cost.len()];
        for &i in &order {
            let g = (0..k).min_by(|&a, &b| load[a].total_cmp(&load[b])).unwrap();
            group[i] = g;
            load[g] += cost[i];
        }
        let s = score(cost, &group);
        if s < best.1 {
            best = (group, s);
        }
    }
    best
}

/// Thread numbers for `groups` that keep as much of the boards' cost as possible on the thread it ran on (their
/// caches stay warm); the calling thread (0) always gets a group.
fn relabel(groups: &[usize], old: &[usize], cost: &[f64], max_threads: usize) -> Vec<usize> {
    let k = groups.iter().max().map_or(1, |&g| g + 1);
    let weight = |g: usize| groups.iter().zip(cost).filter(|(x, _)| **x == g).map(|(_, c)| c).sum::<f64>();
    let mut by_weight: Vec<usize> = (0..k).collect();
    by_weight.sort_by(|&a, &b| weight(b).total_cmp(&weight(a)).then(a.cmp(&b)));
    let mut label = vec![usize::MAX; k];
    let mut taken = vec![false; max_threads];
    for &g in &by_weight {
        let kept = |t: usize| {
            (0..groups.len()).filter(|&i| groups[i] == g && old.get(i) == Some(&t)).map(|i| cost[i]).sum::<f64>()
        };
        let t = (0..max_threads).filter(|&t| !taken[t]).max_by(|&a, &b| kept(a).total_cmp(&kept(b)).then(b.cmp(&a)));
        let t = t.unwrap();
        taken[t] = true;
        label[g] = t;
    }
    if !taken[0] {
        // The lightest group goes to the calling thread
        let g = *by_weight.last().unwrap();
        label[g] = 0;
    }
    groups.iter().map(|&g| label[g]).collect()
}

/// Waits until `ready`: spins for a while, then yields, then parks (whoever makes it ready unparks the waiter).
fn wait_for(mut ready: impl FnMut() -> bool) {
    const SPIN: Duration = Duration::from_micros(50);
    const YIELD: Duration = Duration::from_millis(2);
    for _ in 0..256 {
        if ready() {
            return;
        }
        std::hint::spin_loop();
    }
    let start = Instant::now();
    loop {
        for _ in 0..64 {
            if ready() {
                return;
            }
            std::hint::spin_loop();
        }
        let waited = start.elapsed();
        if waited < SPIN {
        } else if waited < YIELD {
            std::thread::yield_now();
        } else {
            std::thread::park();
        }
    }
}

fn lock<'a, T>(m: &'a Mutex<T>) -> MutexGuard<'a, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What the threads share for one run.
struct Shared<'a> {
    /// The boards. Only one thread wants each at a time: its group's thread within a slice, the calling thread
    /// between slices.
    boards: Vec<Mutex<&'a mut Session>>,
    /// Per board: the thread that runs it this slice, or [`IDLE`].
    owner: Vec<AtomicUsize>,
    /// Per board: host time spent running it since the last regrouping (ns).
    spent: Vec<AtomicU64>,
    /// Per thread (0, the calling thread, unused): the slice it is to run (they are numbered from 1).
    go: Vec<AtomicU64>,
    /// The end of the slice (ps).
    until: AtomicU64,
    /// Workers still running the slice.
    running: AtomicUsize,
    /// Workers leave.
    quit: AtomicBool,
    /// Boards that stopped for a reason other than reaching the end of the slice.
    stops: Mutex<Vec<(usize, StopReason)>>,
    /// The first panic of a board, to go on unwinding on the calling thread.
    panic: Mutex<Option<Box<dyn Any + Send>>>,
    caller: Thread,
}

impl Shared<'_> {
    /// Runs thread `me`'s boards to the end of the slice.
    fn run_group(&self, me: usize) {
        let until = Time(self.until.load(Ordering::Relaxed));
        for (i, board) in self.boards.iter().enumerate() {
            if self.owner[i].load(Ordering::Relaxed) != me {
                continue;
            }
            let mut board = lock(board);
            let start = Instant::now();
            // A panic goes on on the calling thread, once every thread is done with the slice
            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| board.run_machine(until))) {
                Ok(StopReason::TimeLimit) => {}
                Ok(other) => lock(&self.stops).push((i, other)),
                Err(panic) => {
                    lock(&self.panic).get_or_insert(panic);
                }
            }
            self.spent[i].fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
        }
    }

    /// A worker thread's life: run its group every slice it is woken for, until told to leave.
    fn work(&self, me: usize) {
        let mut seen = 0;
        loop {
            let mut slice = seen;
            wait_for(|| {
                slice = self.go[me].load(Ordering::Acquire);
                slice != seen || self.quit.load(Ordering::Acquire)
            });
            if self.quit.load(Ordering::Acquire) {
                return;
            }
            seen = slice;
            self.run_group(me);
            if self.running.fetch_sub(1, Ordering::AcqRel) == 1 {
                self.caller.unpark();
            }
        }
    }
}

/// The workers of one run; tells them to leave when the run ends, by returning or by a panic.
struct Crew<'s, 'a> {
    shared: &'s Shared<'a>,
    /// Thread `w`'s handle at `w - 1`.
    workers: Vec<Thread>,
}

impl Drop for Crew<'_, '_> {
    fn drop(&mut self) {
        self.shared.quit.store(true, Ordering::Release);
        for w in &self.workers {
            w.unpark();
        }
    }
}

/// [`Robot::run_until`] for boards that may run side by side (no acoustic medium, more than one thread allowed).
/// The same run as taking turns: every slice ends with the same boards in the same state, and the end of the
/// slice is done on the calling thread in the same order. If a board stops for another reason than reaching the
/// slice's end, the others have still run to it, and the one with the lowest index is returned.
pub(crate) fn run_until(robot: &mut Robot, end: Time) -> Option<(usize, StopReason)> {
    let max_threads = max_threads(robot);
    let Robot {
        boards, buses, board_bus, power_on, vehicles, slice, now, realtime, pacing, balance, stop_flag, ..
    } = robot;
    let n = boards.len();
    balance.fit(n, max_threads);
    let shared = Shared {
        boards: boards.iter_mut().map(Mutex::new).collect(),
        owner: (0..n).map(|_| AtomicUsize::new(IDLE)).collect(),
        spent: (0..n).map(|_| AtomicU64::new(0)).collect(),
        go: (0..max_threads).map(|_| AtomicU64::new(0)).collect(),
        until: AtomicU64::new(0),
        running: AtomicUsize::new(0),
        quit: AtomicBool::new(false),
        stops: Mutex::new(Vec::new()),
        panic: Mutex::new(None),
        caller: std::thread::current(),
    };
    let shared = &shared;
    std::thread::scope(|scope| {
        let mut crew = Crew { shared, workers: Vec::new() };
        let mut slice_no = 0u64;
        let mut since_regroup = 0;
        let mut regroup_at = if balance.cost.is_empty() { FIRST_REGROUP_SLICES } else { REGROUP_SLICES };
        let mut busy = vec![false; max_threads];
        // Wall clock since the last regrouping, and the time spent pacing in it
        let mut window = Instant::now();
        let mut paced = Duration::ZERO;
        while *now < end && !crate::signal::is_set(*stop_flag) {
            let t = (*now + *slice).min(end);
            slice_no += 1;
            busy.fill(false);
            for (i, power) in power_on.iter_mut().enumerate() {
                let on = power.1 || power_on_by(&mut lock(&shared.boards[i]), power, t);
                let thread = if on { balance.thread[i] } else { IDLE };
                shared.owner[i].store(thread, Ordering::Relaxed);
                if on {
                    busy[thread] = true;
                }
            }
            shared.until.store(t.as_ps(), Ordering::Relaxed);
            shared.running.store(busy[1..].iter().filter(|&&b| b).count(), Ordering::Relaxed);
            for (w, _) in busy.iter().enumerate().skip(1).filter(|(_, b)| **b) {
                while crew.workers.len() < w {
                    let me = crew.workers.len() + 1;
                    let h = std::thread::Builder::new()
                        .name(format!("board-worker-{me}"))
                        .spawn_scoped(scope, move || shared.work(me))
                        .expect("starting a board worker thread");
                    crew.workers.push(h.thread().clone());
                }
                shared.go[w].store(slice_no, Ordering::Release);
                crew.workers[w - 1].unpark();
            }
            shared.run_group(0);
            wait_for(|| shared.running.load(Ordering::Acquire) == 0);
            if let Some(panic) = lock(&shared.panic).take() {
                std::panic::resume_unwind(panic);
            }
            if let Some(stop) = lock(&shared.stops).drain(..).min_by_key(|s| s.0) {
                return Some(stop);
            }
            {
                let mut held: Vec<MutexGuard<&mut Session>> = shared.boards.iter().map(lock).collect();
                let mut boards: Vec<&mut Session> = held.iter_mut().map(|b| &mut ***b).collect();
                end_slice(buses, board_bus, &mut boards, vehicles, t);
            }
            *now = t;
            if *realtime {
                let start = Instant::now();
                pacing.pace(*now);
                paced += start.elapsed();
            }
            since_regroup += 1;
            if since_regroup >= regroup_at {
                let spent: Vec<u64> = shared.spent.iter().map(|s| s.swap(0, Ordering::Relaxed)).collect();
                let wall = window.elapsed().saturating_sub(paced).as_nanos() as u64;
                balance.regroup(&spent, since_regroup, max_threads, wall);
                (window, paced) = (Instant::now(), Duration::ZERO);
                since_regroup = 0;
                regroup_at = REGROUP_SLICES;
            }
        }
        None
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::cpu::Cpu;

    #[test]
    fn at_least_one_core() {
        assert!(cores() >= 1);
    }

    #[test]
    fn heavy_boards_get_threads_of_their_own() {
        // Talos as measured: two ESC boards, the IVC, six light boards (µs per slice)
        let cost: Vec<f64> = [3.1, 95.0, 93.0, 2.3, 3.0, 3.9, 0.9, 30.0, 5.2].iter().map(|c| c * 1000.0).collect();
        let (groups, s) = plan(&cost, 4);
        assert_ne!(groups[1], groups[2]);
        for i in [0, 3, 4, 5, 6, 7, 8] {
            assert_ne!(groups[i], groups[1]);
            assert_ne!(groups[i], groups[2]);
        }
        // Three threads are enough: a fourth would only cost
        assert_eq!(groups.iter().max(), Some(&2));
        assert!((s - (95_000.0 + 2.0 * THREAD_COST_NS)).abs() < 1e-6, "{s}");
        // On two threads the ESC boards are apart
        let (two, _) = plan(&cost, 2);
        assert_ne!(two[1], two[2]);
    }

    #[test]
    fn light_boards_stay_on_one_thread() {
        let (groups, _) = plan(&[2000.0, 1500.0, 1000.0], 8);
        assert_eq!(groups, [0, 0, 0]);
        let (groups, _) = plan(&[], 8);
        assert!(groups.is_empty());
    }

    /// Threads that make slices no shorter (the cores are busy with other programs) are given up for a while.
    #[test]
    fn threads_that_do_not_pay_are_given_up_for_a_while() {
        let mut b = Balance::default();
        b.fit(2, 2);
        let spent = [10_000_000, 10_000_000];
        b.regroup(&spent, 100, 2, 20_000_000);
        assert_eq!(b.threads(), [0, 1]);
        // The slices took as long as in turn
        b.regroup(&spent, 100, 2, 19_000_000);
        assert_eq!(b.threads(), [0, 0]);
        for _ in 0..HOLD_REGROUPS {
            b.regroup(&spent, 100, 2, 20_000_000);
            assert_eq!(b.threads(), [0, 0]);
        }
        b.regroup(&spent, 100, 2, 20_000_000);
        assert_eq!(b.threads(), [0, 1]);
        // Again no better: given up for twice as long
        b.regroup(&spent, 100, 2, 19_000_000);
        for _ in 0..2 * HOLD_REGROUPS {
            assert_eq!(b.threads(), [0, 0]);
            b.regroup(&spent, 100, 2, 20_000_000);
        }
        b.regroup(&spent, 100, 2, 20_000_000);
        assert_eq!(b.threads(), [0, 1]);
        // Shorter slices: kept, and the next hold is short again
        b.regroup(&spent, 100, 2, 10_500_000);
        assert_eq!(b.threads(), [0, 1]);
        b.regroup(&spent, 100, 2, 19_000_000);
        for _ in 0..HOLD_REGROUPS {
            assert_eq!(b.threads(), [0, 0]);
            b.regroup(&spent, 100, 2, 20_000_000);
        }
        b.regroup(&spent, 100, 2, 20_000_000);
        assert_eq!(b.threads(), [0, 1]);
    }

    #[test]
    fn regrouping_keeps_boards_where_they_were() {
        let cost = [10.0, 50.0, 50.0, 10.0];
        // Same groups, numbered differently: the boards keep their threads
        assert_eq!(relabel(&[0, 1, 2, 0], &[3, 1, 0, 3], &cost, 4), [3, 1, 0, 3]);
        // The calling thread always gets a group
        let t = relabel(&[0, 1, 0, 0], &[2, 3, 2, 2], &cost, 4);
        assert!(t.contains(&0), "{t:?}");
        assert_ne!(t[0], t[1]);
    }

    #[test]
    fn a_grouping_is_kept_unless_clearly_better() {
        let mut b = Balance::default();
        b.fit(3, 3);
        assert_eq!(b.threads(), [0, 0, 0]);
        // 100 slices: board 0 and 1 heavy
        b.regroup(&[10_000_000, 10_000_000, 100_000], 100, 3, 20_100_000);
        let t = b.threads().to_vec();
        assert_ne!(t[0], t[1]);
        // Nearly the same costs again, and the threads paid: no change
        b.regroup(&[10_100_000, 9_900_000, 120_000], 100, 3, 10_500_000);
        assert_eq!(b.threads(), t);
        // A different robot (or fewer threads) starts over
        b.fit(3, 1);
        assert_eq!(b.threads(), [0, 0, 0]);
    }

    /// A device that panics when the firmware first changes a pin it watches.
    #[derive(Clone)]
    struct Bomb;

    impl sim_devices::pin::PinDevice for Bomb {
        fn name(&self) -> String {
            "bomb".into()
        }
        fn watches(&self, pin: u8) -> bool {
            pin == 25
        }
        fn pin_changed(&mut self, _: &mut dyn sim_devices::DeviceCtx, _: u8, _: bool) {
            panic!("bomb went off");
        }
        fn clone_box(&self) -> Box<dyn sim_devices::pin::PinDevice> {
            Box::new(self.clone())
        }
        fn as_any(&self) -> &dyn Any {
            self
        }
    }

    /// Three blinky boards, each on a thread of its own; the last one with a [`Bomb`] on its LED pin if `bomb`.
    fn blinkies(bomb: bool) -> Robot {
        let elf = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../test_programs/embassy_blinky.elf");
        let image = sim_core::image::Image::load(&elf).unwrap();
        let mut r = Robot::new(sim_devices::can::CanBus::new("can", crate::board::bus_rates("internal").unwrap()));
        for i in 0..3 {
            let mut m = rp2040::Machine::new(rp2040::MachineConfig {
                bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
                ..Default::default()
            });
            m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
            if bomb && i == 2 {
                m.sys.attach_pin_device(Box::new(Bomb));
            }
            m.load(&image).unwrap();
            m.direct_boot().unwrap();
            r.add(&format!("blinky{i}"), Session::new(m, &image), None);
        }
        r.threads = 3;
        r.fix_threads(vec![0, 1, 2]);
        r
    }

    #[test]
    fn boards_on_their_own_threads_run_as_in_turn() {
        let mut a = blinkies(false);
        let mut b = blinkies(false);
        b.threads = 1;
        assert!(a.run_until(Time::from_ms(600)).is_none());
        assert!(b.run_until(Time::from_ms(600)).is_none());
        assert_eq!(a.thread_of_boards(), [0, 1, 2]);
        for (x, y) in a.boards.iter().zip(&b.boards) {
            assert_eq!(x.machine.now, y.machine.now);
            assert_eq!(x.machine.sys.sram, y.machine.sys.sram);
            assert_eq!(x.machine.cores[0].cycles(), y.machine.cores[0].cycles());
        }
    }

    /// A board panicking on a worker thread panics the calling thread, with its message, and the workers leave.
    #[test]
    fn a_panic_on_a_worker_goes_on_on_the_calling_thread() {
        let mut r = blinkies(true);
        let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| r.run_until(Time::from_ms(600))));
        let msg = panic.unwrap_err();
        assert_eq!(msg.downcast_ref::<&str>(), Some(&"bomb went off"));
        // It stopped in the slice the LED first changed in, with the other boards there too
        assert!(r.now < Time::from_ms(600));
        assert_eq!(r.boards[0].machine.now, r.boards[1].machine.now);
    }

    #[test]
    fn waiting_ends_when_another_thread_says_so() {
        let flag = std::sync::Arc::new(AtomicBool::new(false));
        let me = std::thread::current();
        let f = flag.clone();
        let t = std::thread::spawn(move || {
            // Long enough for the waiter to get to parking
            std::thread::sleep(Duration::from_millis(20));
            f.store(true, Ordering::Release);
            me.unpark();
        });
        wait_for(|| flag.load(Ordering::Acquire));
        t.join().unwrap();
    }
}
