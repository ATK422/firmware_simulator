//! TI DRV8243 H-bridge drivers sharing one nSLEEP line: the wake-up handshake the firmware runs to find them.
//!
//! From the datasheet's "device functional modes": with nSLEEP low for longer than `t_SLEEP` the driver sleeps and
//! nFAULT is released. When nSLEEP rises, the driver wakes and, once ready (`t_READY`), pulls nFAULT low to say so.
//! A short nSLEEP low pulse (`t_RESET`, shorter than `t_SLEEP`) acknowledges that, and the driver releases nFAULT
//! and becomes active. Bridges marked missing never answer, so the firmware sees them as disconnected.
//!
//! nFAULT is open drain with a pull-up on the board. It reaches the SoC either on a GPIO or as a voltage on an
//! analog mux input (0 V pulled low, 3.3 V released).
//!
//! The model also follows DRVOFF (outputs off while high), for reports.

use std::sync::{Arc, Mutex};

use sim_core::time::Time;
use sim_core::wire::{External, Level};

use crate::DeviceCtx;
use crate::pin::PinDevice;

/// nSLEEP low for at least this long puts the driver to sleep.
pub const T_SLEEP: Time = Time::from_us(100);
/// From nSLEEP rising to nFAULT low (ready).
pub const T_READY: Time = Time::from_us(300);

/// Where one bridge's nFAULT goes.
#[derive(Clone, Debug)]
pub enum Nfault {
    Pin(u8),
    /// A voltage, e.g. a mux input: 0 V when pulled low, 3.3 V when released.
    Volts(Arc<Mutex<f64>>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Sleep,
    /// Woken, nFAULT not yet low.
    Waking,
    /// nFAULT low: waiting for the reset pulse.
    Ready,
    Active,
}

#[derive(Clone, Debug)]
pub struct Drv8243 {
    pub nsleep: u8,
    pub drvoff: Option<u8>,
    /// One entry per bridge on the nSLEEP line, with whether it is fitted (answers at all).
    pub bridges: Vec<(Nfault, bool)>,
    pub state: State,
    nsleep_level: bool,
    since: Time,
    pub outputs_off: bool,
    /// Completed wake-ups.
    pub wakeups: u32,
}

impl Drv8243 {
    pub fn new(nsleep: u8, drvoff: Option<u8>, bridges: Vec<(Nfault, bool)>) -> Self {
        Drv8243 {
            nsleep,
            drvoff,
            bridges,
            state: State::Sleep,
            nsleep_level: false,
            since: Time::ZERO,
            outputs_off: true,
            wakeups: 0,
        }
    }

    /// Sets every fitted bridge's nFAULT: pulled low or released.
    fn nfault(&self, ctx: &mut dyn DeviceCtx, low: bool) {
        for (out, fitted) in &self.bridges {
            let low = low && *fitted;
            match out {
                Nfault::Pin(p) => ctx.set_pin(*p, External::Driven(if low { Level::Low } else { Level::High })),
                Nfault::Volts(v) => *v.lock().unwrap() = if low { 0.0 } else { 3.3 },
            }
        }
    }
}

impl PinDevice for Drv8243 {
    fn name(&self) -> String {
        format!("DRV8243 x{} (nSLEEP GPIO {})", self.bridges.len(), self.nsleep)
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.nsleep || Some(pin) == self.drvoff
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, pin: u8, level: bool) {
        let now = ctx.now();
        if Some(pin) == self.drvoff {
            if self.outputs_off != level {
                self.outputs_off = level;
                ctx.report(format!("DRV8243: outputs {}", if level { "off (DRVOFF)" } else { "on" }));
            }
            return;
        }
        if level == self.nsleep_level {
            return;
        }
        self.nsleep_level = level;
        let low_for = now - self.since;
        self.since = now;
        if !level {
            // Going to sleep unless it comes back up within t_SLEEP
            ctx.wake_at(now + T_SLEEP);
            return;
        }
        match self.state {
            State::Sleep => {
                self.state = State::Waking;
                ctx.wake_at(now + T_READY);
            }
            State::Ready if low_for < T_SLEEP => {
                self.state = State::Active;
                self.wakeups += 1;
                self.nfault(ctx, false);
                ctx.report("DRV8243: awake".into());
            }
            _ => {}
        }
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        let now = ctx.now();
        if !self.nsleep_level && now - self.since >= T_SLEEP && self.state != State::Sleep {
            self.state = State::Sleep;
            self.nfault(ctx, false);
        } else if self.nsleep_level && self.state == State::Waking && now - self.since >= T_READY {
            self.state = State::Ready;
            self.nfault(ctx, true);
        }
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    fn at(ctx: &mut TestCtx, d: &mut Drv8243, us: u64, nsleep: bool) {
        ctx.now = Time::from_us(us);
        d.pin_changed(ctx, d.nsleep, nsleep);
    }

    fn tick(ctx: &mut TestCtx, d: &mut Drv8243, us: u64) {
        ctx.now = Time::from_us(us);
        d.update(ctx);
    }

    fn device() -> (Drv8243, Arc<Mutex<f64>>) {
        let v = Arc::new(Mutex::new(3.3));
        (Drv8243::new(25, Some(12), vec![(Nfault::Volts(v.clone()), true), (Nfault::Pin(11), true)]), v)
    }

    #[test]
    fn wake_up_handshake() {
        let mut ctx = TestCtx::default();
        let (mut d, v) = device();
        // The firmware's sequence: sleep 120 us, wake, read nFAULT after 500 us, 50 us reset pulse, read again
        at(&mut ctx, &mut d, 0, false);
        tick(&mut ctx, &mut d, 120);
        assert_eq!(d.state, State::Sleep);
        at(&mut ctx, &mut d, 120, true);
        tick(&mut ctx, &mut d, 420);
        assert_eq!(d.state, State::Ready);
        assert_eq!(*v.lock().unwrap(), 0.0, "nFAULT low: ready");
        assert_eq!(ctx.pins[11], External::Driven(Level::Low));
        at(&mut ctx, &mut d, 1120, false);
        at(&mut ctx, &mut d, 1170, true);
        tick(&mut ctx, &mut d, 1220);
        assert_eq!(d.state, State::Active);
        assert_eq!(*v.lock().unwrap(), 3.3, "nFAULT released: awake");
        assert_eq!(ctx.pins[11], External::Driven(Level::High));
        assert_eq!(d.wakeups, 1);
    }

    #[test]
    fn long_low_sleeps_and_unfitted_bridges_never_answer() {
        let mut ctx = TestCtx::default();
        let v = Arc::new(Mutex::new(3.3));
        let mut d = Drv8243::new(25, None, vec![(Nfault::Volts(v.clone()), false)]);
        at(&mut ctx, &mut d, 0, true);
        tick(&mut ctx, &mut d, 400);
        assert_eq!(d.state, State::Ready);
        assert_eq!(*v.lock().unwrap(), 3.3, "a missing bridge leaves nFAULT high");
        // nSLEEP low for longer than t_SLEEP: back to sleep, and the next rise starts a new wake-up
        at(&mut ctx, &mut d, 500, false);
        tick(&mut ctx, &mut d, 650);
        assert_eq!(d.state, State::Sleep);
        at(&mut ctx, &mut d, 700, true);
        assert_eq!(d.state, State::Waking);
    }

    #[test]
    fn drvoff_is_reported() {
        let mut ctx = TestCtx::default();
        let (mut d, _) = device();
        ctx.now = Time::from_us(5);
        d.pin_changed(&mut ctx, 12, false);
        assert!(!d.outputs_off);
        assert_eq!(ctx.reports, ["DRV8243: outputs on"]);
    }
}
