//! Running a robot's boards side by side within a slice.
//!
//! Within one slice of [`crate::robot::Robot::run_until`] the boards do not see each other: frames between them are
//! delivered by the buses after the slice, and each machine (its cores, peripherals and devices) is its own. So
//! they can run on several threads and end the slice exactly as they would have one after the other. Boards in an
//! acoustic medium are the exception (a hydrophone can hear a projector of another board within the same slice
//! over a short enough path), so robots with a medium run their boards in turn.

use std::sync::{Mutex, OnceLock};

use rp2040::{Machine, StopReason};
use sim_core::time::Time;

use crate::session::Session;

/// Threads to use for `threads` = 0: one per core.
fn default_threads() -> usize {
    static N: OnceLock<usize> = OnceLock::new();
    *N.get_or_init(|| std::thread::available_parallelism().map_or(1, |n| n.get()))
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
    let mut work: Vec<(usize, &mut Machine)> = Vec::new();
    for (i, (b, (on_at, on))) in boards.iter_mut().zip(power_on.iter_mut()).enumerate() {
        if !*on {
            if t <= *on_at {
                continue;
            }
            b.machine.power_on_at(*on_at);
            *on = true;
        }
        work.push((i, &mut b.machine));
    }
    let threads = if threads == 0 { default_threads() } else { threads }.min(work.len());
    if threads <= 1 {
        for (i, m) in work {
            match m.run_until(t) {
                StopReason::TimeLimit => {}
                other => return Some((i, other)),
            }
        }
        return None;
    }
    // Hand out the busiest boards (most instructions so far) first, so that a long one does not start last
    work.sort_by_key(|(_, m)| m.cores.iter().map(|c| c.instructions()).sum::<u64>());
    let queue = Mutex::new(work);
    let stops = Mutex::new(Vec::new());
    let worker = || {
        loop {
            let Some((i, m)) = queue.lock().unwrap().pop() else { break };
            match m.run_until(t) {
                StopReason::TimeLimit => {}
                other => stops.lock().unwrap().push((i, other)),
            }
        }
    };
    std::thread::scope(|s| {
        for _ in 1..threads {
            s.spawn(worker);
        }
        worker();
    });
    stops.into_inner().unwrap().into_iter().min_by_key(|&(i, _)| i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_at_least_one_thread() {
        assert!(default_threads() >= 1);
    }
}
