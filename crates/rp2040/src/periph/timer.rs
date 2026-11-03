//! The microsecond tick generator (in WATCHDOG), the 64-bit TIMER with its four alarms, and the WATCHDOG
//! (datasheet 4.6 and 4.7).
//!
//! The tick generator divides `clk_ref` by `WATCHDOG.TICK.CYCLES`; both the TIMER and the watchdog countdown count
//! its ticks, so a wrongly configured tick makes `embassy-time` run at the wrong rate exactly as on hardware. All
//! counters are computed lazily from virtual time; alarms and the watchdog timeout are scheduler events.

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

/// Derives tick counts from virtual time.
#[derive(Clone, Debug)]
pub struct TickGen {
    pub enabled: bool,
    pub cycles: u32,
    ref_period: ClockPeriod,
    base_time: Time,
    base_ticks: u64,
}

impl TickGen {
    pub fn new(clk_ref_hz: u64) -> Self {
        // Reset value of WATCHDOG.TICK: ENABLE set, CYCLES 0 (no ticks until software sets CYCLES)
        TickGen {
            enabled: true,
            cycles: 0,
            ref_period: ClockPeriod::from_hz(clk_ref_hz),
            base_time: Time::ZERO,
            base_ticks: 0,
        }
    }

    fn running(&self) -> bool {
        self.enabled && self.cycles != 0 && !self.ref_period.is_stopped()
    }

    /// Ticks generated since power-on.
    pub fn ticks(&self, now: Time) -> u64 {
        if !self.running() || now <= self.base_time {
            return self.base_ticks;
        }
        self.base_ticks + self.ref_period.cycles_in(now - self.base_time) / self.cycles as u64
    }

    /// Earliest time at which [`TickGen::ticks`] reaches `k`, if ticks are running.
    pub fn time_of_tick(&self, k: u64) -> Option<Time> {
        if k <= self.base_ticks {
            return Some(self.base_time);
        }
        if !self.running() {
            return None;
        }
        let cycles = (k - self.base_ticks).checked_mul(self.cycles as u64)?;
        Some(self.base_time + self.ref_period.cycles(cycles) + Time(1))
    }

    /// Re-anchors the tick count at `now` before a configuration change.
    pub fn rebase(&mut self, now: Time) {
        self.base_ticks = self.ticks(now);
        self.base_time = now;
    }

    /// `clk_ref` changed frequency at `now`.
    pub fn set_ref_hz(&mut self, now: Time, hz: u64) {
        if self.ref_period.hz() != hz {
            self.rebase(now);
            self.ref_period = ClockPeriod::from_hz(hz);
        }
    }

    pub fn tick_reg(&self, now: Time) -> u32 {
        let mut v = self.cycles & 0x1FF;
        if self.enabled {
            v |= 1 << 9;
        }
        if self.running() {
            v |= 1 << 10;
            // COUNT: clk_ref cycles left until the next tick
            let into = if now > self.base_time { self.ref_period.cycles_in(now - self.base_time) } else { 0 };
            let left = self.cycles as u64 - into % self.cycles as u64;
            v |= ((left as u32) & 0x1FF) << 11;
        }
        v
    }

    pub fn write_tick_reg(&mut self, now: Time, value: u32) {
        self.rebase(now);
        self.cycles = value & 0x1FF;
        self.enabled = value & (1 << 9) != 0;
    }
}

// ------------------------------------------------------------------------------------------------ TIMER

pub const TIMEHW: u32 = 0x00;
pub const TIMELW: u32 = 0x04;
pub const TIMEHR: u32 = 0x08;
pub const TIMELR: u32 = 0x0C;
pub const ALARM0: u32 = 0x10;
pub const ALARM3: u32 = 0x1C;
pub const ARMED: u32 = 0x20;
pub const TIMERAWH: u32 = 0x24;
pub const TIMERAWL: u32 = 0x28;
pub const DBGPAUSE: u32 = 0x2C;
pub const PAUSE: u32 = 0x30;
pub const INTR: u32 = 0x34;
pub const INTE: u32 = 0x38;
pub const INTF: u32 = 0x3C;
pub const INTS: u32 = 0x40;

#[derive(Clone, Debug, Default)]
pub struct Timer {
    /// Timer value at `base_ticks` ticks.
    base_value: u64,
    base_ticks: u64,
    paused: bool,
    latched_high: u32,
    pending_low: u32,
    pub alarm: [u32; 4],
    pub armed: u8,
    pub intr: u32,
    pub inte: u32,
    pub intf: u32,
    pub dbgpause: u32,
}

impl Timer {
    pub fn new() -> Self {
        Timer { dbgpause: 7, ..Default::default() }
    }

    /// The 64-bit microsecond count at `now`.
    pub fn value(&self, ticks: &TickGen, now: Time) -> u64 {
        if self.paused {
            return self.base_value;
        }
        self.base_value.wrapping_add(ticks.ticks(now).saturating_sub(self.base_ticks))
    }

    fn set_value(&mut self, ticks: &TickGen, now: Time, v: u64) {
        self.base_value = v;
        self.base_ticks = ticks.ticks(now);
    }

    pub fn irq_lines(&self) -> u32 {
        (self.intr | self.intf) & self.inte & 0xF
    }

    /// When alarm `n` fires, given the current value; `None` if disarmed or the tick is stopped.
    pub fn alarm_due(&self, ticks: &TickGen, now: Time, n: usize) -> Option<Time> {
        if self.armed & (1 << n) == 0 || self.paused {
            return None;
        }
        let v = self.value(ticks, now);
        let mut target = (v & !0xFFFF_FFFF) | self.alarm[n] as u64;
        if target < v {
            target += 1 << 32;
        }
        let tick = self.base_ticks + (target - self.base_value);
        ticks.time_of_tick(tick).map(|t| t.max(now))
    }

    /// Fires alarm `n` if it is due at `now`. Returns true if it fired.
    pub fn fire(&mut self, ticks: &TickGen, now: Time, n: usize) -> bool {
        if self.armed & (1 << n) == 0 {
            return false;
        }
        if self.value(ticks, now) as u32 != self.alarm[n] {
            return false;
        }
        self.armed &= !(1 << n);
        self.intr |= 1 << n;
        true
    }

    pub fn read(&mut self, ticks: &TickGen, now: Time, offset: u32) -> Option<u32> {
        let v = self.value(ticks, now);
        Some(match offset {
            TIMEHR => self.latched_high,
            TIMELR => {
                self.latched_high = (v >> 32) as u32;
                v as u32
            }
            TIMEHW | TIMELW => 0,
            ALARM0..=ALARM3 => self.alarm[((offset - ALARM0) / 4) as usize],
            ARMED => self.armed as u32,
            TIMERAWH => (v >> 32) as u32,
            TIMERAWL => v as u32,
            DBGPAUSE => self.dbgpause,
            PAUSE => self.paused as u32,
            INTR => self.intr,
            INTE => self.inte,
            INTF => self.intf,
            INTS => self.irq_lines(),
            _ => return None,
        })
    }

    /// Reads a register without side effects (TIMELR does not latch the high word).
    pub fn peek(&self, ticks: &TickGen, now: Time, offset: u32) -> Option<u32> {
        let v = self.value(ticks, now);
        match offset {
            TIMELR => Some(v as u32),
            _ => {
                let mut copy = self.clone();
                copy.read(ticks, now, offset)
            }
        }
    }

    /// Writes a register. Returns `None` for unknown offsets, otherwise the alarms whose schedule may have changed
    /// (bitmask).
    pub fn write(&mut self, ticks: &TickGen, now: Time, offset: u32, value: u32, mode: WriteMode) -> Option<u8> {
        let mut resched = 0u8;
        match offset {
            TIMELW => self.pending_low = value,
            TIMEHW => {
                let v = (value as u64) << 32 | self.pending_low as u64;
                self.set_value(ticks, now, v);
                resched = 0xF;
            }
            ALARM0..=ALARM3 => {
                let n = ((offset - ALARM0) / 4) as usize;
                self.alarm[n] = mode.apply(self.alarm[n], value);
                self.armed |= 1 << n;
                resched = 1 << n;
            }
            ARMED => {
                let bits = (mode.w1c_bits(value) & 0xF) as u8;
                self.armed &= !bits;
                resched = bits;
            }
            DBGPAUSE => mode.update(&mut self.dbgpause, value, 0x6),
            PAUSE => {
                let pause = mode.apply(self.paused as u32, value) & 1 != 0;
                if pause != self.paused {
                    let v = self.value(ticks, now);
                    self.paused = pause;
                    self.set_value(ticks, now, v);
                    resched = 0xF;
                }
            }
            INTR => self.intr &= !(mode.w1c_bits(value) & 0xF),
            INTE => mode.update(&mut self.inte, value, 0xF),
            INTF => mode.update(&mut self.intf, value, 0xF),
            TIMEHR | TIMELR | TIMERAWH | TIMERAWL | INTS => {}
            _ => return None,
        }
        Some(resched)
    }

    /// Restarts the count from zero at `now` (the block came out of reset).
    pub fn restart(&mut self, ticks: &TickGen, now: Time) {
        self.base_value = 0;
        self.base_ticks = ticks.ticks(now);
    }
}

// ------------------------------------------------------------------------------------------------ WATCHDOG

pub const WD_CTRL: u32 = 0x00;
pub const WD_LOAD: u32 = 0x04;
pub const WD_REASON: u32 = 0x08;
pub const WD_SCRATCH0: u32 = 0x0C;
pub const WD_SCRATCH7: u32 = 0x28;
pub const WD_TICK: u32 = 0x2C;

const CTRL_ENABLE: u32 = 1 << 30;
const CTRL_TRIGGER: u32 = 1 << 31;

/// Why the watchdog reset the chip.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WatchdogReset {
    Timer,
    Force,
}

#[derive(Clone, Debug)]
pub struct Watchdog {
    pub tick: TickGen,
    /// CTRL bits other than TIME and TRIGGER.
    ctrl: u32,
    /// Counter value at `base_ticks`.
    counter: u32,
    base_ticks: u64,
    pub reason: u32,
    pub scratch: [u32; 8],
}

impl Watchdog {
    pub fn new(clk_ref_hz: u64) -> Self {
        Watchdog {
            tick: TickGen::new(clk_ref_hz),
            ctrl: 0x0700_0000,
            counter: 0,
            base_ticks: 0,
            reason: 0,
            scratch: [0; 8],
        }
    }

    pub fn enabled(&self) -> bool {
        self.ctrl & CTRL_ENABLE != 0
    }

    /// Current counter value. RP2040-E1: the counter decrements twice per tick.
    pub fn counter(&self, now: Time) -> u32 {
        if !self.enabled() {
            return self.counter;
        }
        let elapsed = self.tick.ticks(now).saturating_sub(self.base_ticks);
        self.counter.saturating_sub(elapsed.saturating_mul(2).min(u32::MAX as u64) as u32)
    }

    fn rebase_counter(&mut self, now: Time) {
        self.counter = self.counter(now);
        self.base_ticks = self.tick.ticks(now);
    }

    /// When the countdown reaches zero, if enabled.
    pub fn timeout_due(&self, now: Time) -> Option<Time> {
        if !self.enabled() {
            return None;
        }
        let c = self.counter(now) as u64;
        let tick = self.tick.ticks(now) + c.div_ceil(2);
        self.tick.time_of_tick(tick).map(|t| t.max(now))
    }

    /// Checks for expiry at `now`.
    pub fn expired(&self, now: Time) -> bool {
        self.enabled() && self.counter(now) == 0
    }

    /// Records a watchdog reset in REASON. Scratch registers survive.
    pub fn record_reset(&mut self, kind: WatchdogReset) {
        self.reason = match kind {
            WatchdogReset::Timer => 1,
            WatchdogReset::Force => 2,
        };
        self.ctrl &= !CTRL_ENABLE;
        self.counter = 0;
    }

    /// A reset not caused by the watchdog clears REASON.
    pub fn clear_reason(&mut self) {
        self.reason = 0;
    }

    pub fn read(&self, now: Time, offset: u32) -> Option<u32> {
        Some(match offset {
            WD_CTRL => self.ctrl | (self.counter(now) & 0x00FF_FFFF),
            WD_LOAD => 0,
            WD_REASON => self.reason,
            WD_SCRATCH0..=WD_SCRATCH7 => self.scratch[((offset - WD_SCRATCH0) / 4) as usize],
            WD_TICK => self.tick.tick_reg(now),
            _ => return None,
        })
    }

    /// Writes a register. Returns `None` for unknown offsets, otherwise `Some(force_reset)`.
    pub fn write(&mut self, now: Time, offset: u32, value: u32, mode: WriteMode) -> Option<bool> {
        match offset {
            WD_CTRL => {
                self.rebase_counter(now);
                let full = mode.apply(self.ctrl, value);
                self.ctrl = full & 0x4700_0000;
                if full & CTRL_TRIGGER != 0 && mode != WriteMode::Clear {
                    return Some(true);
                }
            }
            WD_LOAD => {
                self.counter = value & 0x00FF_FFFF;
                self.base_ticks = self.tick.ticks(now);
            }
            WD_SCRATCH0..=WD_SCRATCH7 => {
                let i = ((offset - WD_SCRATCH0) / 4) as usize;
                self.scratch[i] = mode.apply(self.scratch[i], value);
            }
            WD_TICK => {
                self.rebase_counter(now);
                let v = mode.apply(self.tick.tick_reg(now) & 0x3FF, value);
                self.tick.write_tick_reg(now, v);
                self.base_ticks = self.tick.ticks(now);
            }
            WD_REASON => {}
            _ => return None,
        }
        Some(false)
    }

    /// `clk_ref` changed frequency.
    pub fn set_ref_hz(&mut self, now: Time, hz: u64) {
        self.rebase_counter(now);
        self.tick.set_ref_hz(now, hz);
        self.base_ticks = self.tick.ticks(now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tick_1us() -> TickGen {
        let mut t = TickGen::new(12_000_000);
        t.write_tick_reg(Time::ZERO, 1 << 9 | 12);
        t
    }

    #[test]
    fn tick_generator() {
        let t = tick_1us();
        assert_eq!(t.ticks(Time::from_us(1000)), 1000);
        assert_eq!(t.ticks(Time::from_ns(999)), 0);
        let at = t.time_of_tick(5).unwrap();
        assert_eq!(t.ticks(at), 5);
        assert_eq!(t.ticks(at - Time(2)), 4);
        let stopped = TickGen::new(12_000_000);
        assert_eq!(stopped.ticks(Time::from_ms(5)), 0, "CYCLES = 0 after reset: no ticks");
        assert!(stopped.time_of_tick(1).is_none());
        assert_eq!(t.tick_reg(Time::ZERO) & 0x7FF, 1 << 10 | 1 << 9 | 12, "RUNNING, ENABLE, CYCLES");
    }

    #[test]
    fn tick_rebase_on_ref_change() {
        let mut t = tick_1us();
        t.set_ref_hz(Time::from_us(100), 6_000_000);
        // 12 cycles of 6 MHz = 2 us per tick from now on
        assert_eq!(t.ticks(Time::from_us(100)), 100);
        assert_eq!(t.ticks(Time::from_us(120)), 110);
    }

    #[test]
    fn timer_read_latching_and_write() {
        let t = tick_1us();
        let mut timer = Timer::new();
        let now = Time::from_us(42);
        assert_eq!(timer.read(&t, now, TIMERAWL), Some(42));
        assert_eq!(timer.read(&t, now, TIMELR), Some(42));
        assert_eq!(timer.read(&t, now, TIMEHR), Some(0));
        timer.write(&t, now, TIMELW, 0xFFFF_FFF0, WriteMode::Normal);
        timer.write(&t, now, TIMEHW, 1, WriteMode::Normal);
        let later = Time::from_us(42 + 0x20);
        assert_eq!(timer.read(&t, later, TIMERAWH), Some(2), "carried into the high word");
        assert_eq!(timer.read(&t, later, TIMERAWL), Some(0x10));
    }

    #[test]
    fn timer_pause() {
        let t = tick_1us();
        let mut timer = Timer::new();
        timer.write(&t, Time::from_us(10), PAUSE, 1, WriteMode::Normal);
        assert_eq!(timer.value(&t, Time::from_us(50)), 10);
        timer.write(&t, Time::from_us(50), PAUSE, 0, WriteMode::Normal);
        assert_eq!(timer.value(&t, Time::from_us(60)), 20);
    }

    #[test]
    fn alarm_scheduling_and_firing() {
        let t = tick_1us();
        let mut timer = Timer::new();
        let now = Time::from_us(100);
        timer.write(&t, now, INTE, 1, WriteMode::Normal);
        timer.write(&t, now, ALARM0, 150, WriteMode::Normal);
        assert_eq!(timer.armed, 1);
        let due = timer.alarm_due(&t, now, 0).unwrap();
        assert_eq!(timer.value(&t, due), 150);
        assert!(!timer.fire(&t, Time::from_us(149), 0));
        assert!(timer.fire(&t, due, 0));
        assert_eq!(timer.armed, 0);
        assert_eq!(timer.irq_lines(), 1);
        assert_eq!(timer.read(&t, due, INTS), Some(1));
        timer.write(&t, due, INTR, 1, WriteMode::Normal);
        assert_eq!(timer.irq_lines(), 0);
        // An alarm in the past waits for the low word to wrap
        timer.write(&t, due, ALARM1, 10, WriteMode::Normal);
        let due1 = timer.alarm_due(&t, due, 1).unwrap();
        assert_eq!(timer.value(&t, due1), (1 << 32) + 10);
        // Disarm through ARMED
        timer.write(&t, due, ARMED, 2, WriteMode::Normal);
        assert!(timer.alarm_due(&t, due, 1).is_none());
        // INTF forces
        timer.write(&t, due, INTF, 1, WriteMode::Normal);
        assert_eq!(timer.irq_lines(), 1);
    }

    const ALARM1: u32 = ALARM0 + 4;

    #[test]
    fn watchdog_countdown_twice_per_tick() {
        let mut w = Watchdog::new(12_000_000);
        w.write(Time::ZERO, WD_TICK, 1 << 9 | 12, WriteMode::Normal);
        w.write(Time::ZERO, WD_LOAD, 2000, WriteMode::Normal);
        w.write(Time::ZERO, WD_CTRL, CTRL_ENABLE, WriteMode::Set);
        assert_eq!(w.counter(Time::from_us(100)), 1800);
        assert_eq!(w.read(Time::from_us(100), WD_CTRL).unwrap() & 0xFF_FFFF, 1800);
        let due = w.timeout_due(Time::ZERO).unwrap();
        assert_eq!(due.as_us(), 1000, "LOAD 2000 times out after 1 ms (RP2040-E1)");
        assert!(!w.expired(Time::from_us(999)));
        assert!(w.expired(due));
        // Feeding reloads
        w.write(Time::from_us(500), WD_LOAD, 2000, WriteMode::Normal);
        assert_eq!(w.timeout_due(Time::from_us(500)).unwrap().as_us(), 1500);
    }

    #[test]
    fn watchdog_trigger_and_scratch() {
        let mut w = Watchdog::new(12_000_000);
        w.write(Time::ZERO, WD_SCRATCH0 + 12, 0xDEAD, WriteMode::Normal);
        assert_eq!(w.write(Time::ZERO, WD_CTRL, CTRL_TRIGGER, WriteMode::Set), Some(true));
        w.record_reset(WatchdogReset::Force);
        assert_eq!(w.read(Time::ZERO, WD_REASON), Some(2));
        assert_eq!(w.read(Time::ZERO, WD_SCRATCH0 + 12), Some(0xDEAD), "scratch survives");
        w.record_reset(WatchdogReset::Timer);
        assert_eq!(w.read(Time::ZERO, WD_REASON), Some(1));
        assert!(!w.enabled());
    }
}
