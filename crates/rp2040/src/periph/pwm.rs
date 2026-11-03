//! PWM (datasheet 4.5): eight slices with a 16-bit counter, fractional divider, free-running and phase-correct
//! modes, double-buffered CC/TOP, wrap interrupts and the two channel outputs per slice.
//!
//! Counters are computed lazily from virtual time. The machine asks for [`Pwm::next_event`] to schedule wrap
//! interrupts and, for pins someone observes (traces, attached devices), output edges.
//!
//! The input modes (DIVMODE gated/rising/falling on the B pin) are not modelled yet: the counter holds still in
//! those modes.

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

pub const NUM_SLICES: usize = 8;

const CSR_EN: u32 = 1 << 0;
const CSR_PH_CORRECT: u32 = 1 << 1;
const CSR_A_INV: u32 = 1 << 2;
const CSR_B_INV: u32 = 1 << 3;
const CSR_PH_RET: u32 = 1 << 6;
const CSR_PH_ADV: u32 = 1 << 7;

#[derive(Clone, Debug)]
pub struct Slice {
    pub csr: u32,
    pub div: u32,
    pub cc: u32,
    pub top: u32,
    /// Values written while running, applied at the next wrap.
    pending: Option<(u32, u32)>,
    base_time: Time,
    /// Position in the counting sequence (0..period_len) at `base_time`, plus whole periods already elapsed.
    base_pos: u64,
    /// Wraps counted before `base_time` (to report wraps lazily).
    wraps_before: u64,
    wraps_seen: u64,
}

impl Default for Slice {
    fn default() -> Self {
        Slice {
            csr: 0,
            div: 0x10,
            cc: 0,
            top: 0xFFFF,
            pending: None,
            base_time: Time::ZERO,
            base_pos: 0,
            wraps_before: 0,
            wraps_seen: 0,
        }
    }
}

impl Slice {
    fn running(&self) -> bool {
        self.csr & CSR_EN != 0 && (self.csr >> 4) & 3 == 0
    }

    fn phase_correct(&self) -> bool {
        self.csr & CSR_PH_CORRECT != 0
    }

    fn period_len(&self) -> u64 {
        let n = self.top as u64 + 1;
        if self.phase_correct() { 2 * n } else { n }
    }

    /// Divider in 1/16ths (INT 0 means 256).
    fn divfix(&self) -> u64 {
        let int = (self.div >> 4) & 0xFF;
        let int = if int == 0 { 256 } else { int as u64 };
        int * 16 + (self.div & 0xF) as u64
    }

    /// Counter ticks elapsed since the base.
    fn ticks(&self, clk: &ClockPeriod, now: Time) -> u64 {
        if !self.running() || now <= self.base_time {
            return 0;
        }
        clk.cycles_in(now - self.base_time) * 16 / self.divfix()
    }

    fn time_of_ticks(&self, clk: &ClockPeriod, ticks: u64) -> Time {
        let cycles = (ticks * self.divfix()).div_ceil(16);
        self.base_time + clk.cycles(cycles) + Time(1)
    }

    /// Absolute position (periods * len + position) at `now`.
    fn abs_pos(&self, clk: &ClockPeriod, now: Time) -> u64 {
        self.base_pos + self.ticks(clk, now)
    }

    fn counter_from_pos(&self, pos: u64) -> u32 {
        let p = pos % self.period_len();
        if self.phase_correct() && p > self.top as u64 { (2 * self.top as u64 + 1 - p) as u32 } else { p as u32 }
    }

    /// Re-anchors at `now` so that configuration can change without disturbing the count.
    fn rebase(&mut self, clk: &ClockPeriod, now: Time) {
        let abs = self.abs_pos(clk, now);
        let len = self.period_len();
        self.wraps_before += abs / len;
        self.base_pos = abs % len;
        self.base_time = now;
    }

    /// Applies double-buffered values if a wrap happened since they were written, and counts wraps.
    fn sync(&mut self, clk: &ClockPeriod, now: Time) {
        if let Some((cc, top)) = self.pending {
            let len = self.period_len();
            let abs = self.abs_pos(clk, now);
            if abs >= len {
                // Re-anchor at the wrap that latched the new values
                let wrap_ticks = len - self.base_pos;
                let wrap_time = self.time_of_ticks(clk, wrap_ticks).min(now);
                self.wraps_before += 1;
                self.base_time = wrap_time;
                self.base_pos = 0;
                self.cc = cc;
                self.top = top;
                self.pending = None;
            }
        }
    }

    fn total_wraps(&self, clk: &ClockPeriod, now: Time) -> u64 {
        self.wraps_before + self.abs_pos(clk, now) / self.period_len()
    }

    pub fn counter(&self, clk: &ClockPeriod, now: Time) -> u32 {
        self.counter_from_pos(self.abs_pos(clk, now))
    }

    /// Output level of channel `ch` (0 = A, 1 = B) at `now`.
    pub fn output(&self, clk: &ClockPeriod, now: Time, ch: usize) -> bool {
        let cc = if ch == 0 { self.cc & 0xFFFF } else { self.cc >> 16 };
        let inv = self.csr & if ch == 0 { CSR_A_INV } else { CSR_B_INV } != 0;
        let level = if self.csr & CSR_EN == 0 {
            // A disabled slice holds its output where the counter stopped
            self.counter_from_pos(self.base_pos) < cc
        } else {
            self.counter(clk, now) < cc
        };
        level != inv
    }

    /// Time of the next output change of channel `ch`, if it toggles at all.
    fn next_edge(&self, clk: &ClockPeriod, now: Time, ch: usize) -> Option<Time> {
        if !self.running() {
            return None;
        }
        let cc = (if ch == 0 { self.cc & 0xFFFF } else { self.cc >> 16 }) as u64;
        let top = self.top as u64;
        if cc == 0 || cc > top {
            return None;
        }
        let len = self.period_len();
        let abs = self.abs_pos(clk, now);
        let pos = abs % len;
        let edges: [u64; 2] = if self.phase_correct() { [cc, 2 * top + 2 - cc] } else { [cc, len] };
        let next = edges.iter().copied().filter(|&e| e > pos).min().unwrap_or(len + edges[0].min(edges[1]));
        Some(self.time_of_ticks(clk, abs - pos + next - self.base_pos))
    }

    /// Time of the next wrap.
    fn next_wrap(&self, clk: &ClockPeriod, now: Time) -> Option<Time> {
        if !self.running() {
            return None;
        }
        let len = self.period_len();
        let abs = self.abs_pos(clk, now);
        let next = (abs / len + 1) * len;
        Some(self.time_of_ticks(clk, next - self.base_pos))
    }
}

#[derive(Clone, Debug, Default)]
pub struct Pwm {
    pub slices: [Slice; NUM_SLICES],
    pub intr: u32,
    pub inte: u32,
    pub intf: u32,
    /// Bitmask of slices whose output edges someone observes.
    pub observed: u32,
}

impl Pwm {
    pub fn irq_line(&self) -> bool {
        (self.intr | self.intf) & self.inte & 0xFF != 0
    }

    /// Brings every slice up to `now`: latches buffered values and raw wrap interrupts.
    pub fn sync(&mut self, clk: &ClockPeriod, now: Time) {
        for (i, s) in self.slices.iter_mut().enumerate() {
            s.sync(clk, now);
            let w = s.total_wraps(clk, now);
            if w > s.wraps_seen {
                s.wraps_seen = w;
                self.intr |= 1 << i;
            }
        }
    }

    /// Re-anchors every slice before `clk_sys` changes frequency.
    pub fn rebase_all(&mut self, clk: &ClockPeriod, now: Time) {
        self.sync(clk, now);
        for s in &mut self.slices {
            s.rebase(clk, now);
        }
    }

    /// Output of the PWM function on GPIO `pin`.
    pub fn pin_output(&self, clk: &ClockPeriod, now: Time, pin: usize) -> bool {
        self.slices[(pin / 2) % NUM_SLICES].output(clk, now, pin % 2)
    }

    /// Earliest time something needs the machine's attention: a wrap with its interrupt enabled, or an output
    /// edge of an observed slice.
    pub fn next_event(&self, clk: &ClockPeriod, now: Time) -> Option<Time> {
        let mut best: Option<Time> = None;
        let mut consider = |t: Option<Time>| {
            if let Some(t) = t {
                best = Some(best.map_or(t, |b: Time| b.min(t)));
            }
        };
        for (i, s) in self.slices.iter().enumerate() {
            if self.inte & (1 << i) != 0 {
                consider(s.next_wrap(clk, now));
            }
            if self.observed & (1 << i) != 0 {
                consider(s.next_edge(clk, now, 0));
                consider(s.next_edge(clk, now, 1));
                if s.pending.is_some() {
                    consider(s.next_wrap(clk, now));
                }
            }
        }
        best.map(|t| t.max(now))
    }

    pub fn read(&mut self, clk: &ClockPeriod, now: Time, offset: u32) -> Option<u32> {
        self.sync(clk, now);
        if offset < 0xA0 {
            let s = &self.slices[(offset / 0x14) as usize];
            return Some(match offset % 0x14 {
                0x0 => s.csr,
                0x4 => s.div,
                0x8 => s.counter(clk, now),
                0xC => s.pending.map_or(s.cc, |p| p.0),
                _ => s.pending.map_or(s.top, |p| p.1),
            });
        }
        Some(match offset {
            0xA0 => self.slices.iter().enumerate().fold(0, |acc, (i, s)| acc | (s.csr & CSR_EN) << i),
            0xA4 => self.intr,
            0xA8 => self.inte,
            0xAC => self.intf,
            0xB0 => (self.intr | self.intf) & self.inte & 0xFF,
            _ => return None,
        })
    }

    pub fn write(&mut self, clk: &ClockPeriod, now: Time, offset: u32, value: u32, mode: WriteMode) -> bool {
        self.sync(clk, now);
        if offset < 0xA0 {
            let i = (offset / 0x14) as usize;
            let s = &mut self.slices[i];
            match offset % 0x14 {
                0x0 => {
                    s.rebase(clk, now);
                    let new = mode.apply(s.csr, value);
                    if new & CSR_PH_ADV != 0 {
                        s.base_pos = (s.base_pos + 1) % s.period_len();
                    }
                    if new & CSR_PH_RET != 0 {
                        s.base_pos = (s.base_pos + s.period_len() - 1) % s.period_len();
                    }
                    let was_running = s.csr & CSR_EN != 0;
                    s.csr = new & 0x3F;
                    if !was_running && s.csr & CSR_EN != 0 {
                        if let Some((cc, top)) = s.pending.take() {
                            s.cc = cc;
                            s.top = top;
                        }
                        s.wraps_seen = s.total_wraps(clk, now);
                    }
                }
                0x4 => {
                    s.rebase(clk, now);
                    mode.update(&mut s.div, value, 0xFFF);
                }
                0x8 => {
                    s.rebase(clk, now);
                    s.base_pos = (mode.apply(s.counter(clk, now), value) & 0xFFFF) as u64 % s.period_len();
                }
                0xC | 0x10 => {
                    let (mut cc, mut top) = s.pending.unwrap_or((s.cc, s.top));
                    if offset % 0x14 == 0xC {
                        cc = mode.apply(cc, value);
                    } else {
                        top = mode.apply(top, value) & 0xFFFF;
                    }
                    if s.csr & CSR_EN != 0 {
                        s.pending = Some((cc, top));
                    } else {
                        s.rebase(clk, now);
                        s.cc = cc;
                        s.top = top;
                        s.base_pos %= s.period_len();
                    }
                }
                _ => unreachable!(),
            }
            return true;
        }
        match offset {
            0xA0 => {
                let current = self.slices.iter().enumerate().fold(0, |acc, (i, s)| acc | (s.csr & CSR_EN) << i);
                let en = mode.apply(current, value) & 0xFF;
                for i in 0..NUM_SLICES {
                    let csr = (self.slices[i].csr & !CSR_EN) | (en >> i) & 1;
                    self.write(clk, now, i as u32 * 0x14, csr, WriteMode::Normal);
                }
            }
            0xA4 => self.intr &= !(mode.w1c_bits(value) & 0xFF),
            0xA8 => mode.update(&mut self.inte, value, 0xFF),
            0xAC => mode.update(&mut self.intf, value, 0xFF),
            0xB0 => {}
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clk() -> ClockPeriod {
        ClockPeriod::from_hz(125_000_000)
    }

    fn setup(top: u32, cc: u32, div: u32, csr: u32) -> Pwm {
        let mut p = Pwm::default();
        let c = clk();
        p.write(&c, Time::ZERO, 0x10, top, WriteMode::Normal);
        p.write(&c, Time::ZERO, 0x0C, cc, WriteMode::Normal);
        p.write(&c, Time::ZERO, 0x04, div, WriteMode::Normal);
        p.write(&c, Time::ZERO, 0x00, csr | CSR_EN, WriteMode::Normal);
        p
    }

    #[test]
    fn free_running_counter_and_duty() {
        let c = clk();
        // TOP 99, div 1: 100 cycles period = 800 ns; CC_A 25 -> high for 200 ns
        let mut p = setup(99, 25, 0x10, 0);
        assert_eq!(p.read(&c, Time::from_ns(80), 0x08), Some(10));
        assert!(p.pin_output(&c, Time::from_ns(100), 0));
        assert!(!p.pin_output(&c, Time::from_ns(300), 0));
        assert!(p.pin_output(&c, Time::from_ns(810), 0), "next period");
        let e = p.slices[0].next_edge(&c, Time::from_ns(100), 0).unwrap();
        assert_eq!(p.slices[0].counter(&c, e), 25);
        let e2 = p.slices[0].next_edge(&c, e, 0).unwrap();
        assert_eq!(p.slices[0].counter(&c, e2), 0);
        assert_eq!(e2.as_ns(), 800);
    }

    #[test]
    fn divider_and_inversion() {
        let c = clk();
        // div 2.5 (0x28): counter at 50 MHz
        let p = setup(9, 5 << 16, 0x28, CSR_B_INV);
        assert_eq!(p.slices[0].counter(&c, Time::from_ns(200)), 10 % 10);
        assert_eq!(p.slices[0].counter(&c, Time::from_ns(110)), 5);
        // Channel B: CC 5, inverted: low while counter < 5
        assert!(!p.pin_output(&c, Time::from_ns(40), 1));
        assert!(p.pin_output(&c, Time::from_ns(140), 1));
    }

    #[test]
    fn phase_correct_counts_up_and_down() {
        let c = clk();
        let p = setup(3, 2, 0x10, CSR_PH_CORRECT);
        let seq: Vec<u32> = (0..10).map(|i| p.slices[0].counter(&c, Time::from_ns(8 * i + 4))).collect();
        assert_eq!(seq, vec![0, 1, 2, 3, 3, 2, 1, 0, 0, 1]);
    }

    #[test]
    fn fully_on_and_off_have_no_edges() {
        let c = clk();
        let p = setup(99, 100, 0x10, 0);
        assert!(p.pin_output(&c, Time::from_us(3), 0));
        assert!(p.slices[0].next_edge(&c, Time::ZERO, 0).is_none());
        let p = setup(99, 0, 0x10, CSR_A_INV);
        assert!(p.pin_output(&c, Time::from_us(3), 0), "0% inverted is always high");
    }

    #[test]
    fn double_buffered_cc() {
        let c = clk();
        let mut p = setup(99, 25, 0x10, 0);
        p.write(&c, Time::from_ns(100), 0x0C, 75, WriteMode::Normal);
        assert_eq!(p.read(&c, Time::from_ns(100), 0x0C), Some(75), "reads back the written value");
        assert!(!p.pin_output(&c, Time::from_ns(400), 0), "old CC until the wrap");
        p.sync(&c, Time::from_ns(1000));
        assert!(p.pin_output(&c, Time::from_ns(1300), 0), "new CC after the wrap");
    }

    #[test]
    fn wrap_interrupt() {
        let c = clk();
        let mut p = setup(99, 25, 0x10, 0);
        p.write(&c, Time::ZERO, 0xA8, 1, WriteMode::Normal);
        let t = p.next_event(&c, Time::ZERO).unwrap();
        assert_eq!(t.as_ns(), 800);
        assert!(!p.irq_line());
        p.sync(&c, t);
        assert!(p.irq_line());
        p.write(&c, t, 0xA4, 1, WriteMode::Normal);
        assert!(!p.irq_line());
        assert_eq!(p.read(&c, t, 0xA0), Some(1), "EN mirror");
    }

    #[test]
    fn disabled_slice_holds() {
        let c = clk();
        let mut p = setup(99, 25, 0x10, 0);
        p.write(&c, Time::from_ns(80), 0x00, CSR_EN, WriteMode::Clear);
        assert_eq!(p.read(&c, Time::from_us(5), 0x08), Some(10));
        assert!(p.pin_output(&c, Time::from_us(5), 0));
    }
}
