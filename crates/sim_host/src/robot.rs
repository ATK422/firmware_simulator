//! Several boards in one process on a shared CAN bus.
//!
//! Every board runs to the same virtual time in short slices; after each slice the bus arbitrates and delivers
//! everything the boards (and host endpoints) sent, and the boards' devices update their pins. The slice bounds
//! how late a frame can reach another board, and keeps runs deterministic.

use rp2040::StopReason;
use sim_core::time::Time;
use sim_devices::can::{CanBus, CanEndpoint, CanNode};

use crate::session::{CanCallback, Session};

pub struct Robot {
    pub boards: Vec<Session>,
    pub bus: CanBus,
    pub endpoints: Vec<Box<dyn CanEndpoint + Send>>,
    pub slice: Time,
    pub now: Time,
    /// Called with every frame that went over the bus.
    pub on_can: Option<CanCallback>,
}

impl Robot {
    pub fn new(bus: CanBus) -> Robot {
        Robot {
            boards: Vec::new(),
            bus,
            endpoints: Vec::new(),
            slice: Time::from_us(100),
            now: Time::ZERO,
            on_can: None,
        }
    }

    /// Runs every board until `end`. Returns the first board that stopped for another reason, with the reason.
    pub fn run_until(&mut self, end: Time) -> Option<(usize, StopReason)> {
        while self.now < end {
            let t = (self.now + self.slice).min(end);
            for (i, b) in self.boards.iter_mut().enumerate() {
                match b.machine.run_until(t) {
                    StopReason::TimeLimit => {}
                    other => return Some((i, other)),
                }
            }
            let mut nodes: Vec<&mut dyn CanNode> = Vec::new();
            for b in self.boards.iter_mut() {
                nodes.extend(b.machine.can_nodes());
            }
            let mut eps: Vec<&mut dyn CanEndpoint> =
                self.endpoints.iter_mut().map(|e| &mut **e as &mut dyn CanEndpoint).collect();
            let before = self.bus.record.as_ref().map_or(0, |r| r.len());
            self.bus.step(&mut nodes, &mut eps, t);
            if let (Some(rec), Some(hook)) = (&self.bus.record, &mut self.on_can) {
                for r in &rec[before..] {
                    hook(r);
                }
            }
            for b in self.boards.iter_mut() {
                b.machine.update_devices();
                b.poll_rtt();
            }
            self.now = t;
        }
        None
    }
}
