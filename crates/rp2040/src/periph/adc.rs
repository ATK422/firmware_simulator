//! ADC (datasheet 4.9): one 12-bit SAR converter, a 5-input mux (GPIO 26-29 and the temperature sensor), round
//! robin sampling, a 4-deep FIFO with DMA requests and a threshold interrupt.
//!
//! A conversion takes 96 `clk_adc` cycles (2 µs at 48 MHz). With START_MANY, conversions start every
//! `1 + INT + FRAC/256` cycles of DIV, or back to back when that is under 96. The system samples the selected
//! input's voltage when a conversion ends ([`Adc::take_due`] / [`Adc::complete`]), so a scripted waveform is seen at
//! exactly the conversion times.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

pub const CS: u32 = 0x00;
pub const RESULT: u32 = 0x04;
pub const FCS: u32 = 0x08;
pub const FIFO: u32 = 0x0C;
pub const DIV: u32 = 0x10;

const CS_EN: u32 = 1 << 0;
const CS_TS_EN: u32 = 1 << 1;
const CS_START_ONCE: u32 = 1 << 2;
const CS_START_MANY: u32 = 1 << 3;
const CS_READY: u32 = 1 << 8;
const CS_ERR: u32 = 1 << 9;

const FCS_EN: u32 = 1 << 0;
const FCS_SHIFT: u32 = 1 << 1;
const FCS_ERR: u32 = 1 << 2;
const FCS_DREQ_EN: u32 = 1 << 3;
const FCS_UNDER: u32 = 1 << 10;
const FCS_OVER: u32 = 1 << 11;

const FIFO_DEPTH: usize = 4;
/// Cycles of `clk_adc` per conversion.
pub const CONVERSION_CYCLES: u64 = 96;

#[derive(Clone, Debug, Default)]
pub struct Adc {
    /// CS without READY/ERR (computed) and the self-clearing START_ONCE.
    cs: u32,
    result: u32,
    /// FCS control bits (EN, SHIFT, ERR, DREQ_EN, THRESH) and the sticky UNDER/OVER flags.
    fcs: u32,
    fifo: VecDeque<u16>,
    div: u32,
    inte: u32,
    intf: u32,
    /// Conversion in progress: input and when it ends.
    converting: Option<(u8, Time)>,
    /// When START_MANY starts the next conversion.
    next_start: Option<Time>,
    warned_clock: bool,
}

impl Adc {
    pub fn enabled(&self) -> bool {
        self.cs & CS_EN != 0
    }

    /// The selected input (AINSEL).
    pub fn ainsel(&self) -> u8 {
        ((self.cs >> 12) & 7) as u8
    }

    pub fn temp_sensor_enabled(&self) -> bool {
        self.cs & CS_TS_EN != 0
    }

    fn rrobin(&self) -> u32 {
        (self.cs >> 16) & 0x1F
    }

    fn thresh(&self) -> usize {
        ((self.fcs >> 24) & 0xF) as usize
    }

    /// Sampling interval for START_MANY in `clk_adc` cycles, as a 24.8 fixed point number.
    fn interval_fx(&self) -> u64 {
        let int = ((self.div >> 8) & 0xFFFF) as u64;
        let frac = (self.div & 0xFF) as u64;
        if int == 0 && frac == 0 {
            return CONVERSION_CYCLES << 8;
        }
        ((1 + int) << 8 | frac).max(CONVERSION_CYCLES << 8)
    }

    fn start_conversion(&mut self, clk: &ClockPeriod, at: Time) -> bool {
        if clk.is_stopped() {
            if !self.warned_clock {
                self.warned_clock = true;
                log::warn!("ADC: conversion requested with clk_adc stopped");
            }
            return false;
        }
        self.converting = Some((self.ainsel(), at + clk.cycles(CONVERSION_CYCLES)));
        true
    }

    /// Starts conversions that are due by `now` (START_ONCE, START_MANY).
    pub fn advance(&mut self, clk: &ClockPeriod, now: Time) {
        if !self.enabled() || self.converting.is_some() {
            return;
        }
        if self.cs & CS_START_ONCE != 0 {
            self.cs &= !CS_START_ONCE;
            self.start_conversion(clk, now);
        } else if self.cs & CS_START_MANY != 0 {
            let at = self.next_start.unwrap_or(now);
            if at <= now && self.start_conversion(clk, at) {
                // 24.8 cycles to time
                let step = Time((clk.cycles(self.interval_fx()).as_ps()) >> 8);
                self.next_start = Some(at + step);
            }
        }
    }

    /// The conversion that ends by `now`, if any: its input and end time. The caller samples the input and calls
    /// [`Adc::complete`].
    pub fn take_due(&mut self, now: Time) -> Option<(u8, Time)> {
        match self.converting {
            Some((input, t)) if t <= now => {
                self.converting = None;
                Some((input, t))
            }
            _ => None,
        }
    }

    /// Stores a finished conversion's 12-bit result and moves round robin on.
    pub fn complete(&mut self, code: u16, clk: &ClockPeriod, at: Time) {
        let code = code.min(0xFFF);
        self.result = code as u32;
        if self.fcs & FCS_EN != 0 {
            let mut v = if self.fcs & FCS_SHIFT != 0 { code >> 4 } else { code };
            // ERR (bit 15) is never set: conversions do not fail here
            if self.fcs & FCS_ERR != 0 {
                v &= 0x7FFF;
            }
            if self.fifo.len() < FIFO_DEPTH {
                self.fifo.push_back(v);
            } else {
                self.fcs |= FCS_OVER;
            }
        }
        let rr = self.rrobin();
        if rr != 0 {
            let cur = self.ainsel() as u32;
            let next = (1..=5).map(|k| (cur + k) % 5).find(|c| rr & (1 << c) != 0).unwrap_or(cur);
            self.cs = (self.cs & !(7 << 12)) | next << 12;
        }
        if self.cs & CS_START_MANY != 0 {
            self.advance(clk, at);
        }
    }

    /// When the next conversion ends, or the next one starts.
    pub fn next_event(&self) -> Option<Time> {
        match self.converting {
            Some((_, t)) => Some(t),
            None if self.enabled() && self.cs & CS_START_MANY != 0 => self.next_start,
            None => None,
        }
    }

    pub fn dreq(&self) -> bool {
        self.fcs & FCS_DREQ_EN != 0 && !self.fifo.is_empty()
    }

    fn intr(&self) -> u32 {
        (self.fifo.len() >= self.thresh()) as u32
    }

    pub fn irq(&self) -> bool {
        (self.intr() | self.intf) & self.inte & 1 != 0
    }

    fn cs_value(&self) -> u32 {
        let mut v = self.cs & !(CS_READY | CS_ERR | CS_START_ONCE);
        if self.enabled() && self.converting.is_none() {
            v |= CS_READY;
        }
        v
    }

    fn fcs_value(&self) -> u32 {
        let mut v = self.fcs & !(0xF << 16 | 3 << 8);
        if self.fifo.is_empty() {
            v |= 1 << 8;
        }
        if self.fifo.len() >= FIFO_DEPTH {
            v |= 1 << 9;
        }
        v | (self.fifo.len() as u32) << 16
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            CS => self.cs_value(),
            RESULT => self.result,
            FCS => self.fcs_value(),
            FIFO => match self.fifo.pop_front() {
                Some(v) => v as u32,
                None => {
                    self.fcs |= FCS_UNDER;
                    0
                }
            },
            DIV => self.div,
            0x14 => self.intr(),
            0x18 => self.inte,
            0x1C => self.intf,
            0x20 => (self.intr() | self.intf) & self.inte,
            _ => return None,
        })
    }

    pub fn peek(&self, offset: u32) -> Option<u32> {
        match offset {
            FIFO => Some(self.fifo.front().copied().unwrap_or(0) as u32),
            _ => self.clone().read(offset),
        }
    }

    pub fn write(&mut self, clk: &ClockPeriod, now: Time, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            CS => {
                // ERR and ERR_STICKY never set: conversions do not fail here
                let was_many = self.cs & CS_START_MANY != 0;
                mode.update(&mut self.cs, value, 0x1F_7000 | CS_EN | CS_TS_EN | CS_START_ONCE | CS_START_MANY);
                if !self.enabled() {
                    self.converting = None;
                    self.cs &= !CS_START_ONCE;
                }
                if self.cs & CS_START_MANY == 0 {
                    self.next_start = None;
                } else if !was_many {
                    self.next_start = Some(now);
                }
                self.advance(clk, now);
            }
            FCS => {
                let flags = mode.w1c_bits(value) & (FCS_UNDER | FCS_OVER);
                let mut v = self.fcs & !(FCS_UNDER | FCS_OVER);
                mode.update(&mut v, value, 0xF00_000F);
                // With EN clear the FIFO stops filling but keeps what it has
                self.fcs = v | (self.fcs & (FCS_UNDER | FCS_OVER) & !flags);
            }
            DIV => mode.update(&mut self.div, value, 0xFF_FFFF),
            0x18 => mode.update(&mut self.inte, value, 1),
            0x1C => mode.update(&mut self.intf, value, 1),
            RESULT | FIFO | 0x14 | 0x20 => {}
            _ => return false,
        }
        true
    }
}

/// The RP2040 temperature sensor: `V = 0.706 - 0.001721 * (T - 27)` volts.
pub fn temp_sensor_volts(celsius: f64) -> f64 {
    0.706 - 0.001721 * (celsius - 27.0)
}

/// 12-bit code for `volts` against `vref`.
pub fn code_for(volts: f64, vref: f64) -> u16 {
    if !volts.is_finite() || vref <= 0.0 {
        return 0;
    }
    (volts / vref * 4096.0).floor().clamp(0.0, 4095.0) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clk() -> ClockPeriod {
        ClockPeriod::from_hz(48_000_000)
    }

    /// Microseconds, rounded (48 MHz periods are not a whole number of picoseconds).
    fn us(t: Time) -> u64 {
        (t.as_ps() + 500_000) / 1_000_000
    }

    fn convert_all(a: &mut Adc, until: Time, code: impl Fn(u8) -> u16) -> Vec<(u8, Time)> {
        let mut out = Vec::new();
        while let Some(t) = a.next_event() {
            if t > until {
                break;
            }
            a.advance(&clk(), t);
            if let Some((input, at)) = a.take_due(t) {
                a.complete(code(input), &clk(), at);
                out.push((input, at));
            }
        }
        out
    }

    #[test]
    fn single_conversion_ready_and_result() {
        let mut a = Adc::default();
        assert_eq!(a.read(CS).unwrap() & CS_READY, 0, "not ready while disabled");
        a.write(&clk(), Time::ZERO, CS, CS_EN, WriteMode::Normal);
        assert_ne!(a.read(CS).unwrap() & CS_READY, 0);
        a.write(&clk(), Time::ZERO, CS, CS_EN | 1 << 12 | CS_START_ONCE, WriteMode::Normal);
        assert_eq!(a.read(CS).unwrap() & (CS_READY | CS_START_ONCE), 0);
        let done = a.next_event().unwrap();
        assert_eq!(us(done), 2, "96 cycles at 48 MHz");
        assert_eq!(a.take_due(done), Some((1, done)));
        a.complete(1234, &clk(), done);
        assert_eq!(a.read(RESULT), Some(1234));
        assert_ne!(a.read(CS).unwrap() & CS_READY, 0);
        assert!(a.next_event().is_none());
    }

    #[test]
    fn start_many_round_robin_fifo_and_shift() {
        let mut a = Adc::default();
        a.write(&clk(), Time::ZERO, FCS, FCS_EN | FCS_SHIFT | FCS_DREQ_EN | 2 << 24, WriteMode::Normal);
        a.write(&clk(), Time::ZERO, 0x18, 1, WriteMode::Normal);
        // Inputs 0 and 2 round robin, a sample every 480 cycles (10 us)
        a.write(&clk(), Time::ZERO, DIV, 479 << 8, WriteMode::Normal);
        a.write(&clk(), Time::ZERO, CS, CS_EN | 0b101 << 16 | CS_START_MANY, WriteMode::Normal);
        let conv = convert_all(&mut a, Time::from_us(35), |i| 0x100 * (i as u16 + 1) + 0x0F);
        let times: Vec<u64> = conv.iter().map(|c| us(c.1)).collect();
        assert_eq!(times, [2, 12, 22, 32]);
        assert_eq!(conv.iter().map(|c| c.0).collect::<Vec<_>>(), [0, 2, 0, 2]);
        assert!(a.irq() && a.dreq());
        assert_eq!(a.read(FCS).unwrap() >> 16 & 0xF, 4);
        assert_eq!(a.read(FIFO), Some(0x10), "shifted to 8 bits");
        assert_eq!(a.read(FIFO), Some(0x30));
        // A fifth conversion overflows the 4-deep FIFO
        let mut a2 = a.clone();
        a2.read(FIFO);
        a2.read(FIFO);
        assert!(!a2.irq(), "below threshold 2");
        convert_all(&mut a, Time::from_us(75), |_| 0);
        assert_ne!(a.read(FCS).unwrap() & FCS_OVER, 0);
        a.write(&clk(), Time::ZERO, FCS, FCS_OVER, WriteMode::Set);
        assert_eq!(a.read(FCS).unwrap() & FCS_OVER, 0, "write one to clear");
        assert_ne!(a.read(FCS).unwrap() & FCS_EN, 0);
    }

    #[test]
    fn back_to_back_with_div_zero_and_stop() {
        let mut a = Adc::default();
        a.write(&clk(), Time::ZERO, FCS, FCS_EN, WriteMode::Normal);
        a.write(&clk(), Time::ZERO, CS, CS_EN | CS_START_MANY, WriteMode::Normal);
        let conv = convert_all(&mut a, Time::from_us(10), |_| 1);
        assert_eq!(conv.iter().map(|c| us(c.1)).collect::<Vec<_>>(), [2, 4, 6, 8, 10], "500 kS/s");
        a.write(&clk(), Time::from_us(10), CS, CS_START_MANY, WriteMode::Clear);
        let conv = convert_all(&mut a, Time::from_us(20), |_| 1);
        assert_eq!(conv.len(), 1, "the conversion in progress finishes");
        assert!(a.next_event().is_none());
        for _ in 0..5 {
            a.read(FIFO);
        }
        assert_ne!(a.read(FCS).unwrap() & FCS_UNDER, 0);
    }

    #[test]
    fn conversions_and_temperature() {
        assert_eq!(code_for(1.65, 3.3), 2048);
        assert_eq!(code_for(5.0, 3.3), 4095);
        assert_eq!(code_for(-1.0, 3.3), 0);
        assert!((temp_sensor_volts(27.0) - 0.706).abs() < 1e-9);
        assert!(temp_sensor_volts(50.0) < 0.706);
    }
}
