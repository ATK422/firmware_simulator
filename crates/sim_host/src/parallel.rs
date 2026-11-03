//! Running a robot's boards side by side within a slice.
//!
//! Within one slice of [`crate::robot::Robot::run_until`] the boards do not see each other: frames between them are
//! delivered by the buses after the slice, and each machine (its cores, peripherals and devices) is its own. So
//! they can run on several threads and end the slice exactly as they would have one after the other. Boards in an
//! acoustic medium are the exception (a hydrophone can hear a projector of another board within the same slice
//! over a short enough path), so robots with a medium run their boards in turn.
//!
//! Slices are short (100 µs of virtual time), so starting threads for each would cost more than it saves. A pool of
//! worker threads lives for the whole process instead; each slice, the machines are moved to the workers (a
//! placeholder takes each one's place in its session meanwhile) and moved back when they reach the slice's end.

use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex, OnceLock};

use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::time::Time;

use crate::session::Session;

/// Threads to use for `threads` = 0: one per core.
fn default_threads() -> usize {
    static N: OnceLock<usize> = OnceLock::new();
    *N.get_or_init(|| std::thread::available_parallelism().map_or(1, |n| n.get()))
}

/// What a worker hands back: the machine and why it stopped, or the panic it stopped with.
type Done = (usize, std::thread::Result<(Machine, StopReason)>);

/// A machine to run to a time, and where its result goes.
struct Job {
    index: usize,
    machine: Machine,
    until: Time,
    done: Sender<Done>,
}

/// The process's worker threads, started as needed.
struct Pool {
    jobs: Sender<Job>,
    queue: Arc<Mutex<Receiver<Job>>>,
    workers: usize,
}

impl Pool {
    fn get() -> &'static Mutex<Pool> {
        static POOL: OnceLock<Mutex<Pool>> = OnceLock::new();
        POOL.get_or_init(|| {
            let (jobs, queue) = channel();
            Mutex::new(Pool { jobs, queue: Arc::new(Mutex::new(queue)), workers: 0 })
        })
    }

    /// Makes sure at least `n` workers run; returns a handle to send jobs on.
    fn workers(n: usize) -> Sender<Job> {
        let mut pool = Pool::get().lock().unwrap();
        while pool.workers < n {
            let queue = pool.queue.clone();
            std::thread::Builder::new()
                .name(format!("board-worker-{}", pool.workers))
                .spawn(move || {
                    loop {
                        let job = queue.lock().unwrap().recv();
                        let Ok(job) = job else { return };
                        let Job { index, mut machine, until, done } = job;
                        // A panic goes back to the robot's thread (and the worker carries on)
                        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                            let reason = machine.run_until(until);
                            (machine, reason)
                        }));
                        // The robot waits for every job it sent; it is gone only if it panicked
                        let _ = done.send((index, r));
                    }
                })
                .expect("starting a board worker thread");
            pool.workers += 1;
        }
        pool.jobs.clone()
    }
}

/// Machines that stand in for the ones out at the workers (only so that sessions always have one).
fn placeholders() -> &'static Mutex<Vec<Machine>> {
    static SPARE: OnceLock<Mutex<Vec<Machine>>> = OnceLock::new();
    SPARE.get_or_init(|| Mutex::new(Vec::new()))
}

fn placeholder() -> Machine {
    placeholders()
        .lock()
        .unwrap()
        .pop()
        .unwrap_or_else(|| Machine::new(MachineConfig { flash_size: 4096, ..Default::default() }))
}

/// Switches on the boards due by `t` (`power_on` holds when and whether each was), then runs every powered board's
/// machine to `t` on up to `threads` threads (0: one per core). Returns the board with the lowest index that
/// stopped for a reason other than reaching `t`; the others have still run to `t`.
pub fn run_boards(
    boards: &mut [Session],
    power_on: &mut [(Time, bool)],
    t: Time,
    threads: usize,
) -> Option<(usize, StopReason)> {
    let mut powered = Vec::new();
    for (i, (b, (on_at, on))) in boards.iter_mut().zip(power_on.iter_mut()).enumerate() {
        if !*on {
            if t <= *on_at {
                continue;
            }
            b.machine.power_on_at(*on_at);
            *on = true;
        }
        powered.push(i);
    }
    let threads = if threads == 0 { default_threads() } else { threads }.min(powered.len());
    if threads <= 1 {
        for i in powered {
            match boards[i].machine.run_until(t) {
                StopReason::TimeLimit => {}
                other => return Some((i, other)),
            }
        }
        return None;
    }
    // The calling thread waits, so `threads` workers run the boards
    let jobs = Pool::workers(threads);
    let (done, results) = channel();
    for &i in &powered {
        let machine = std::mem::replace(&mut boards[i].machine, placeholder());
        jobs.send(Job { index: i, machine, until: t, done: done.clone() }).expect("board workers are gone");
    }
    drop(done);
    let mut stop: Option<(usize, StopReason)> = None;
    for _ in &powered {
        let (i, r) = results.recv().expect("board workers are gone");
        let (machine, reason) = r.unwrap_or_else(|panic| std::panic::resume_unwind(panic));
        let spare = std::mem::replace(&mut boards[i].machine, machine);
        placeholders().lock().unwrap().push(spare);
        if reason != StopReason::TimeLimit && stop.is_none_or(|(j, _)| i < j) {
            stop = Some((i, reason));
        }
    }
    stop
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_at_least_one_thread() {
        assert!(default_threads() >= 1);
    }

    #[test]
    fn workers_start_once_and_run_jobs() {
        let jobs = Pool::workers(2);
        let (done, results) = channel();
        for index in 0..3 {
            let machine = placeholder();
            jobs.send(Job { index, machine, until: Time::from_us(10), done: done.clone() }).unwrap();
        }
        let mut seen: Vec<usize> = (0..3).map(|_| results.recv().unwrap().0).collect();
        seen.sort();
        assert_eq!(seen, [0, 1, 2]);
        assert!(Pool::get().lock().unwrap().workers >= 2);
    }
}
