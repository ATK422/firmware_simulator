//! Virtual time.
//!
//! Time is counted in picoseconds from machine power-on. A `u64` of picoseconds lasts about 213 days of simulated
//! time. CPUs and peripherals convert their own clock cycles to time with a [`ClockPeriod`], so a firmware that
//! reprograms its PLL changes how much virtual time each cycle takes, as on hardware.

use std::fmt;
use std::ops::{Add, AddAssign, Sub};

/// A point in virtual time, in picoseconds since power-on.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Time(pub u64);

impl Time {
    pub const ZERO: Time = Time(0);
    pub const MAX: Time = Time(u64::MAX);

    pub const fn from_ps(ps: u64) -> Time {
        Time(ps)
    }
    pub const fn from_ns(ns: u64) -> Time {
        Time(ns * 1_000)
    }
    pub const fn from_us(us: u64) -> Time {
        Time(us * 1_000_000)
    }
    pub const fn from_ms(ms: u64) -> Time {
        Time(ms * 1_000_000_000)
    }
    pub const fn from_secs(s: u64) -> Time {
        Time(s * 1_000_000_000_000)
    }
    pub fn from_secs_f64(s: f64) -> Time {
        Time((s * 1e12).round() as u64)
    }
    pub const fn as_ps(self) -> u64 {
        self.0
    }
    pub const fn as_ns(self) -> u64 {
        self.0 / 1_000
    }
    pub const fn as_us(self) -> u64 {
        self.0 / 1_000_000
    }
    pub const fn as_ms(self) -> u64 {
        self.0 / 1_000_000_000
    }
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-12
    }
    pub fn saturating_sub(self, other: Time) -> Time {
        Time(self.0.saturating_sub(other.0))
    }
}

impl Add for Time {
    type Output = Time;
    fn add(self, rhs: Time) -> Time {
        Time(self.0.saturating_add(rhs.0))
    }
}

impl AddAssign for Time {
    fn add_assign(&mut self, rhs: Time) {
        self.0 = self.0.saturating_add(rhs.0);
    }
}

impl Sub for Time {
    type Output = Time;
    fn sub(self, rhs: Time) -> Time {
        Time(self.0 - rhs.0)
    }
}

impl fmt::Debug for Time {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

impl fmt::Display for Time {
    /// Seconds with microsecond resolution, e.g. `1.000250s`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let us = self.0 / 1_000_000;
        write!(f, "{}.{:06}s", us / 1_000_000, us % 1_000_000)
    }
}

/// The period of a clock, with sub-picosecond precision so that clocks such as 12 MHz (83 333.33 ps) do not drift.
///
/// Stored as a fixed point number of picoseconds with [`ClockPeriod::FRAC_BITS`] fractional bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ClockPeriod {
    fixed: u128,
    hz: u64,
}

impl ClockPeriod {
    pub const FRAC_BITS: u32 = 32;

    /// The period of a clock running at `hz`. A stopped clock (`hz == 0`) has an infinite period.
    pub fn from_hz(hz: u64) -> ClockPeriod {
        if hz == 0 {
            return ClockPeriod { fixed: u128::MAX, hz: 0 };
        }
        let fixed = (1_000_000_000_000u128 << Self::FRAC_BITS) / hz as u128;
        ClockPeriod { fixed, hz }
    }

    pub fn hz(&self) -> u64 {
        self.hz
    }

    pub fn is_stopped(&self) -> bool {
        self.hz == 0
    }

    /// The duration of `cycles` cycles, rounded down to a picosecond.
    pub fn cycles(&self, cycles: u64) -> Time {
        if self.hz == 0 {
            return Time::MAX;
        }
        Time(((cycles as u128 * self.fixed) >> Self::FRAC_BITS) as u64)
    }

    /// Whole cycles that fit in `t`.
    pub fn cycles_in(&self, t: Time) -> u64 {
        if self.hz == 0 {
            return 0;
        }
        (((t.0 as u128) << Self::FRAC_BITS) / self.fixed) as u64
    }
}

/// Accumulates cycles of a clock into virtual time without losing the fractional picoseconds.
#[derive(Clone, Copy, Debug, Default)]
pub struct CycleClock {
    /// Current time.
    pub now: Time,
    frac: u128,
}

impl CycleClock {
    pub fn new(now: Time) -> Self {
        CycleClock { now, frac: 0 }
    }

    /// Advances by `cycles` of a clock with period `period`.
    pub fn advance(&mut self, period: &ClockPeriod, cycles: u64) {
        if period.is_stopped() {
            return;
        }
        let total = cycles as u128 * period.fixed + self.frac;
        self.now += Time((total >> ClockPeriod::FRAC_BITS) as u64);
        self.frac = total & ((1 << ClockPeriod::FRAC_BITS) - 1);
    }

    /// Jumps forward to `t` (no-op if already past it). Drops the fractional part.
    pub fn jump_to(&mut self, t: Time) {
        if t > self.now {
            self.now = t;
            self.frac = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units() {
        assert_eq!(Time::from_us(3).as_ps(), 3_000_000);
        assert_eq!(Time::from_ms(2).as_us(), 2000);
        assert_eq!(Time::from_secs(1).as_ms(), 1000);
        assert_eq!(format!("{}", Time::from_us(1_000_250)), "1.000250s");
    }

    #[test]
    fn period_exact_and_fractional() {
        let p = ClockPeriod::from_hz(125_000_000);
        assert_eq!(p.cycles(1), Time(8000));
        assert_eq!(p.cycles(125_000_000), Time::from_secs(1));
        let p12 = ClockPeriod::from_hz(12_000_000);
        // 12 M cycles must be one second to within a picosecond per million cycles
        let t = p12.cycles(12_000_000);
        assert!(t.as_ps().abs_diff(1_000_000_000_000) < 20, "{t:?}");
        assert_eq!(p12.cycles_in(Time::from_us(1)), 12);
    }

    #[test]
    fn cycle_clock_keeps_fraction() {
        let p = ClockPeriod::from_hz(12_000_000);
        let mut c = CycleClock::default();
        for _ in 0..12_000_000 {
            c.advance(&p, 1);
        }
        assert!(c.now.as_ps().abs_diff(1_000_000_000_000) < 20, "{:?}", c.now);
    }

    #[test]
    fn stopped_clock() {
        let p = ClockPeriod::from_hz(0);
        assert!(p.is_stopped());
        assert_eq!(p.cycles(5), Time::MAX);
        let mut c = CycleClock::default();
        c.advance(&p, 5);
        assert_eq!(c.now, Time::ZERO);
    }
}
