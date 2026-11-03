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
    /// The period when it is a whole number of picoseconds (125 MHz: 8000 ps): conversions are then plain 64-bit
    /// arithmetic, with the same results.
    whole_ps: Option<u64>,
}

impl ClockPeriod {
    pub const FRAC_BITS: u32 = 32;

    /// The period of a clock running at `hz`. A stopped clock (`hz == 0`) has an infinite period.
    pub fn from_hz(hz: u64) -> ClockPeriod {
        if hz == 0 {
            return ClockPeriod { fixed: u128::MAX, hz: 0, whole_ps: None };
        }
        let fixed = (1_000_000_000_000u128 << Self::FRAC_BITS) / hz as u128;
        let whole_ps = 1_000_000_000_000u64.is_multiple_of(hz).then(|| 1_000_000_000_000 / hz);
        ClockPeriod { fixed, hz, whole_ps }
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
        if let Some(p) = self.whole_ps
            && let Some(t) = cycles.checked_mul(p)
        {
            return Time(t);
        }
        Time(((cycles as u128 * self.fixed) >> Self::FRAC_BITS) as u64)
    }

    /// Whole cycles that fit in `t`.
    pub fn cycles_in(&self, t: Time) -> u64 {
        if self.hz == 0 {
            return 0;
        }
        if let Some(p) = self.whole_ps {
            return t.0 / p;
        }
        (((t.0 as u128) << Self::FRAC_BITS) / self.fixed) as u64
    }
}

/// Accumulates cycles of a clock into virtual time without losing the fractional picoseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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

    /// The time after `cycles` more cycles of `period`, without advancing ([`CycleClock::advance`] then reading
    /// `now` gives the same).
    pub fn time_after(&self, period: &ClockPeriod, cycles: u64) -> Time {
        if period.is_stopped() {
            return self.now;
        }
        let total = cycles as u128 * period.fixed + self.frac;
        self.now + Time((total >> ClockPeriod::FRAC_BITS) as u64)
    }

    /// The fewest cycles of `period` after which `now` is at or past `t`: 0 if it already is, `u64::MAX` if the
    /// clock is stopped (it never gets there).
    pub fn cycles_until(&self, period: &ClockPeriod, t: Time) -> u64 {
        if self.now >= t {
            return 0;
        }
        if period.is_stopped() {
            return u64::MAX;
        }
        // Smallest n with n * fixed + frac >= (t - now) << FRAC_BITS
        let need = (((t.0 - self.now.0) as u128) << ClockPeriod::FRAC_BITS) - self.frac;
        if let (Ok(need), Ok(fixed)) = (u64::try_from(need), u64::try_from(period.fixed)) {
            // The usual case (a short distance, a fast clock): a 64-bit division is much cheaper
            return need.div_ceil(fixed);
        }
        need.div_ceil(period.fixed).min(u64::MAX as u128) as u64
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
    fn whole_picosecond_periods_convert_as_the_general_case() {
        for hz in [125_000_000u64, 100_000_000, 1_000_000, 50_000_000, 250_000_000, 1] {
            let p = ClockPeriod::from_hz(hz);
            assert!(p.whole_ps.is_some(), "{hz}");
            let general = ClockPeriod { whole_ps: None, ..p };
            for v in [0u64, 1, 7_999, 8_000, 8_001, 123_456_789, 2_000_000_000_000, u64::MAX / 2] {
                assert_eq!(p.cycles_in(Time(v)), general.cycles_in(Time(v)), "{hz} Hz, {v} ps");
                assert_eq!(p.cycles(v % (1 << 40)), general.cycles(v % (1 << 40)), "{hz} Hz, {v} cycles");
            }
        }
        assert!(ClockPeriod::from_hz(133_000_000).whole_ps.is_none());
        assert!(ClockPeriod::from_hz(48_000_000).whole_ps.is_none());
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
    fn cycles_until_matches_stepping() {
        for hz in [125_000_000, 12_000_000, 133_000_000, 48_000_000, 1] {
            let p = ClockPeriod::from_hz(hz);
            let mut c = CycleClock::new(Time(12_345));
            c.advance(&p, 7); // leave a fraction
            for d in [0u64, 1, 7_999, 8_000, 8_001, 83_333, 83_334, 1_000_000, 123_456_789] {
                let t = c.now + Time(d);
                let n = c.cycles_until(&p, t);
                assert!(c.time_after(&p, n) >= t, "{hz} Hz, {d} ps: {n} cycles");
                if n > 0 {
                    assert!(c.time_after(&p, n - 1) < t, "{hz} Hz, {d} ps: {n} cycles is not the fewest");
                }
                let mut stepped = c;
                for _ in 0..n.min(10_000) {
                    stepped.advance(&p, 1);
                }
                if n <= 10_000 {
                    assert_eq!(stepped.now, c.time_after(&p, n));
                }
            }
        }
        let stopped = ClockPeriod::from_hz(0);
        assert_eq!(CycleClock::default().cycles_until(&stopped, Time(5)), u64::MAX);
        assert_eq!(CycleClock::new(Time(9)).cycles_until(&stopped, Time(5)), 0);
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
