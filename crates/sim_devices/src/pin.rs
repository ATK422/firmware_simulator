//! Pin-level devices: models that watch pin waveforms and answer on pins, for protocols the SoC bit-bangs (PIO
//! programs: WS2812 strips, DShot ESCs, half-duplex servo UARTs, a software SMBus).
//!
//! The machine tells a device about every level change on the pins it watches, with the time of the change. The
//! device answers by driving pins, now (`set_pin`) or later (`drive_at`), and can ask to be woken (`wake_at`) to
//! notice a gap or a timeout.

use crate::DeviceCtx;

pub trait PinDevice: Send {
    fn name(&self) -> String;
    /// Whether the device watches `pin`.
    fn watches(&self, pin: u8) -> bool;
    /// `pin` changed to `level` at `ctx.now()` (the resolved level, including the device's own driving).
    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, pin: u8, level: bool);
    /// Called at times asked for with `wake_at` (and possibly others).
    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        let _ = ctx;
    }
    fn clone_box(&self) -> Box<dyn PinDevice>;
    fn as_any(&self) -> &dyn std::any::Any;
}

sim_core::clone_box!(PinDevice);

use sim_core::time::Time;
use sim_core::wire::{External, Level};

/// Decodes 8N1 UART bytes from the level changes of one pin (idle high).
#[derive(Clone, Debug)]
pub struct UartRx {
    pub baud: u32,
    level: bool,
    /// Level changes since the current start bit.
    edges: Vec<(Time, bool)>,
    start: Option<Time>,
    pub framing_errors: u32,
}

impl UartRx {
    pub fn new(baud: u32) -> Self {
        UartRx { baud, level: true, edges: Vec::new(), start: None, framing_errors: 0 }
    }

    pub fn bit(&self) -> Time {
        Time::from_ps(1_000_000_000_000 / self.baud as u64)
    }

    /// A level change; asks for a wake-up at the end of a frame that starts here.
    pub fn edge(&mut self, ctx: &mut dyn DeviceCtx, level: bool) {
        let now = ctx.now();
        if level == self.level {
            return;
        }
        self.level = level;
        if self.start.is_none() {
            if !level {
                self.start = Some(now);
                self.edges.clear();
                ctx.wake_at(now + Time(self.bit().as_ps() * 19 / 2));
            }
            return;
        }
        self.edges.push((now, level));
    }

    /// Line level at `t` within the current frame (low from the start bit on, then each recorded change).
    fn level_at(&self, t: Time) -> bool {
        self.edges.iter().take_while(|e| e.0 <= t).last().is_some_and(|e| e.1)
    }

    /// Call from the device's `update`: returns a byte whose frame has ended.
    pub fn poll(&mut self, now: Time) -> Option<u8> {
        let start = self.start?;
        let bit = self.bit().as_ps();
        if now < start + Time(bit * 19 / 2) {
            return None;
        }
        let mut b = 0u8;
        for i in 0..8 {
            if self.level_at(start + Time(bit * (3 + 2 * i) / 2)) {
                b |= 1 << i;
            }
        }
        if !self.level_at(start + Time(bit * 19 / 2)) {
            self.framing_errors += 1;
        }
        self.start = None;
        // A falling edge right at the stop bit's end starts the next byte
        let last = self.edges.last().copied();
        self.edges.clear();
        if let Some((t, false)) = last
            && t > start + Time(bit * 9)
        {
            self.start = Some(t);
        }
        Some(b)
    }

    /// Whether a frame is being received.
    pub fn busy(&self) -> bool {
        self.start.is_some()
    }
}

/// Schedules 8N1 `bytes` on `pin` from `start` (driving both levels), releasing the line afterwards. Returns when
/// the last stop bit ends.
pub fn uart_send(ctx: &mut dyn DeviceCtx, pin: u8, start: Time, baud: u32, bytes: &[u8]) -> Time {
    let bit = 1_000_000_000_000 / baud as u64;
    let mut t = start;
    let mut level = None;
    for &b in bytes {
        let mut levels = vec![false];
        levels.extend((0..8).map(|i| b >> i & 1 != 0));
        levels.push(true);
        for l in levels {
            if level != Some(l) {
                ctx.drive_at(t, pin, External::Driven(Level::from_bool(l)));
                level = Some(l);
            }
            t += Time(bit);
        }
    }
    ctx.drive_at(t, pin, External::Floating);
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    #[test]
    fn uart_round_trip_through_edges() {
        let mut ctx = TestCtx { now: Time::from_us(10), ..Default::default() };
        let t0 = ctx.now;
        let end = uart_send(&mut ctx, 3, t0, 57_600, &[0x55, 0xFF, 0x00, 0xA5]);
        assert_eq!(ctx.scheduled.last().unwrap(), &(end, 3, External::Floating));
        let mut rx = UartRx::new(57_600);
        let mut got = Vec::new();
        let edges: Vec<(Time, bool)> =
            ctx.scheduled.iter().map(|&(t, _, e)| (t, !matches!(e, External::Driven(Level::Low)))).collect();
        let mut wakes: Vec<Time> = Vec::new();
        for (t, l) in edges {
            // Deliver wake-ups due before this edge
            while let Some(&w) = wakes.first().filter(|&&w| w <= t) {
                wakes.remove(0);
                got.extend(rx.poll(w));
            }
            let mut c = TestCtx { now: t, ..Default::default() };
            rx.edge(&mut c, l);
            wakes.extend(c.wakes);
        }
        for w in wakes {
            got.extend(rx.poll(w));
        }
        got.extend(rx.poll(end + Time::from_ms(1)));
        assert_eq!(got, [0x55, 0xFF, 0x00, 0xA5]);
        assert_eq!(rx.framing_errors, 0);
    }
}
