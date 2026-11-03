//! I2C controller (Synopsys DW_apb_i2c, datasheet 4.3) in master mode.
//!
//! Commands leave the 16-deep TX FIFO one at a time. Each is one bus step: a START (or repeated START, on a change of
//! direction or the RESTART bit) with the address from IC_TAR, then one data byte either way, then a STOP if the
//! command asks for one. The system performs the step on the attached devices when it begins (see
//! [`I2c::next_step`] and [`I2c::begin`]); the step takes 9 SCL periods per byte (plus START/STOP and any clock
//! stretching) at the rate HCNT/LCNT give, and its effects (RX data, NAK aborts, STOP_DET) appear when it ends. When
//! the TX FIFO runs dry without a STOP the controller holds the bus (IC_EMPTYFIFO_HOLD_MASTER_EN is 1 on the RP2040).
//!
//! Modelled: FIFOs and thresholds, every interrupt the master raises, TX_ABRT with its sources (address and data
//! NAK, user abort, master disable) and the flushed-FIFO state until IC_CLR_TX_ABRT, TX_EMPTY_CTRL, DMA requests,
//! ENABLE/ENABLE_STATUS. Not modelled: slave mode, 10-bit addressing, general call and START bytes (logged).

use std::collections::VecDeque;

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

pub const IC_CON: u32 = 0x00;
pub const IC_TAR: u32 = 0x04;
pub const IC_DATA_CMD: u32 = 0x10;
pub const IC_ENABLE: u32 = 0x6C;

const FIFO_DEPTH: usize = 16;

pub mod intr {
    pub const RX_UNDER: u32 = 1 << 0;
    pub const RX_OVER: u32 = 1 << 1;
    pub const RX_FULL: u32 = 1 << 2;
    pub const TX_OVER: u32 = 1 << 3;
    pub const TX_EMPTY: u32 = 1 << 4;
    pub const RD_REQ: u32 = 1 << 5;
    pub const TX_ABRT: u32 = 1 << 6;
    pub const RX_DONE: u32 = 1 << 7;
    pub const ACTIVITY: u32 = 1 << 8;
    pub const STOP_DET: u32 = 1 << 9;
    pub const START_DET: u32 = 1 << 10;
    pub const GEN_CALL: u32 = 1 << 11;
    pub const RESTART_DET: u32 = 1 << 12;
}

pub mod abrt {
    pub const ADDR7_NOACK: u32 = 1 << 0;
    pub const TXDATA_NOACK: u32 = 1 << 3;
    pub const MASTER_DIS: u32 = 1 << 11;
    pub const USER_ABRT: u32 = 1 << 16;
}

const CMD_READ: u16 = 1 << 8;
const CMD_STOP: u16 = 1 << 9;
const CMD_RESTART: u16 = 1 << 10;

const CON_MASTER_MODE: u32 = 1 << 0;
const CON_10BIT_MASTER: u32 = 1 << 4;
const CON_TX_EMPTY_CTRL: u32 = 1 << 8;

const TAR_SPECIAL: u32 = 1 << 11;

/// What one bus step does, for the system to carry out on the devices.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Step {
    /// START (or repeated START) with this 7-bit address and direction before the byte.
    pub start: Option<(u8, bool)>,
    /// Whether the START is a repeated START.
    pub restart: bool,
    /// The byte written, or `None` for a read.
    pub write: Option<u8>,
    /// For a read: whether the controller ACKs the byte (it NAKs the last one of a read).
    pub ack: bool,
    pub stop: bool,
}

/// How the devices answered a step.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct StepResult {
    /// The address was ACKed (true when the step has no START).
    pub addr_ack: bool,
    /// The written byte was ACKed.
    pub data_ack: bool,
    /// The byte read.
    pub read: u8,
    /// Extra time the device held SCL low.
    pub stretch: Time,
}

#[derive(Clone, Debug)]
pub struct I2c {
    pub index: u8,
    con: u32,
    tar: u32,
    sar: u32,
    ss_hcnt: u32,
    ss_lcnt: u32,
    fs_hcnt: u32,
    fs_lcnt: u32,
    intr_mask: u32,
    rx_tl: u32,
    tx_tl: u32,
    enable: bool,
    sda_hold: u32,
    dma_cr: u32,
    dma_tdlr: u32,
    dma_rdlr: u32,
    sda_setup: u32,
    ack_general_call: u32,
    fs_spklen: u32,
    /// Latched interrupt flags (the level-type RX_FULL/TX_EMPTY are computed).
    raw: u32,
    abrt_source: u32,
    tx: VecDeque<u16>,
    rx: VecDeque<u8>,
    /// The bus is held by us: address and direction of the current transfer.
    held: Option<(u8, bool)>,
    /// The step on the wire, what it did, and when it ends.
    current: Option<(Step, StepResult, Time)>,
    warned: u32,
}

impl I2c {
    pub fn new(index: u8) -> Self {
        I2c {
            index,
            con: 0x65,
            tar: 0x55,
            sar: 0x55,
            ss_hcnt: 0x28,
            ss_lcnt: 0x2F,
            fs_hcnt: 0x06,
            fs_lcnt: 0x0D,
            intr_mask: 0x8FF,
            rx_tl: 0,
            tx_tl: 0,
            enable: false,
            sda_hold: 1,
            dma_cr: 0,
            dma_tdlr: 0,
            dma_rdlr: 0,
            sda_setup: 0x64,
            ack_general_call: 1,
            fs_spklen: 7,
            raw: 0,
            abrt_source: 0,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            held: None,
            current: None,
            warned: 0,
        }
    }

    pub fn reset(&mut self) {
        *self = I2c::new(self.index);
    }

    fn warn_once(&mut self, bit: u32, msg: &str) {
        if self.warned & bit == 0 {
            self.warned |= bit;
            log::warn!("I2C{}: {msg} is not modelled", self.index);
        }
    }

    /// One SCL period, from the HCNT/LCNT of the selected speed (high = HCNT + SPKLEN + 7, low = LCNT + 1 cycles).
    pub fn scl_period(&self, clk_sys: &ClockPeriod) -> Time {
        let (h, l) = match (self.con >> 1) & 3 {
            1 => (self.ss_hcnt, self.ss_lcnt),
            _ => (self.fs_hcnt, self.fs_lcnt),
        };
        clk_sys.cycles(h as u64 + self.fs_spklen as u64 + 7 + l as u64 + 1)
    }

    /// SCL frequency at `clk_sys_hz`.
    pub fn scl_hz(&self, clk_sys_hz: u64) -> u64 {
        let p = self.scl_period(&ClockPeriod::from_hz(clk_sys_hz)).as_ps();
        1_000_000_000_000u64.checked_div(p).unwrap_or(0)
    }

    pub fn busy(&self) -> bool {
        self.current.is_some()
    }

    fn tx_aborted(&self) -> bool {
        self.raw & intr::TX_ABRT != 0
    }

    fn raw_intr(&self) -> u32 {
        let mut v = self.raw;
        if self.rx.len() as u32 > self.rx_tl.min(15) {
            v |= intr::RX_FULL;
        }
        let tx_low = self.tx.len() as u32 <= self.tx_tl.min(15);
        let idle_ok = self.con & CON_TX_EMPTY_CTRL == 0 || self.current.is_none();
        if self.enable && tx_low && idle_ok {
            v |= intr::TX_EMPTY;
        }
        v
    }

    pub fn irq(&self) -> bool {
        self.raw_intr() & self.intr_mask != 0
    }

    pub fn tx_dreq(&self) -> bool {
        self.dma_cr & 2 != 0 && self.enable && (self.tx.len() as u32) <= self.dma_tdlr
    }

    pub fn rx_dreq(&self) -> bool {
        self.dma_cr & 1 != 0 && self.rx.len() as u32 > self.dma_rdlr
    }

    /// The next step to put on the bus, popped from the TX FIFO, if the controller is idle and has one.
    pub fn next_step(&mut self) -> Option<Step> {
        if self.current.is_some() || !self.enable || self.tx_aborted() {
            return None;
        }
        if self.con & CON_MASTER_MODE == 0 {
            self.warn_once(1, "slave mode");
            return None;
        }
        let cmd = self.tx.pop_front()?;
        if self.con & CON_10BIT_MASTER != 0 || self.tar & (1 << 12) != 0 {
            self.warn_once(2, "10-bit addressing");
        }
        if self.tar & TAR_SPECIAL != 0 {
            self.warn_once(4, "general call / START byte");
        }
        let addr = (self.tar & 0x7F) as u8;
        let read = cmd & CMD_READ != 0;
        let start = match self.held {
            Some((a, r)) if a == addr && r == read && cmd & CMD_RESTART == 0 => None,
            _ => Some((addr, read)),
        };
        let stop = cmd & CMD_STOP != 0;
        // A read is NAKed when it ends the transfer: a STOP, or the next command turns the bus round
        let ack = match self.tx.front() {
            _ if stop => false,
            Some(next) => next & CMD_READ != 0 && next & CMD_RESTART == 0,
            None => true,
        };
        Some(Step {
            start,
            restart: start.is_some() && self.held.is_some(),
            write: (!read).then_some(cmd as u8),
            ack,
            stop,
        })
    }

    /// Puts `step` (as the devices answered it) on the wire at `now`. Returns when it ends.
    pub fn begin(&mut self, step: Step, result: StepResult, clk_sys: &ClockPeriod, now: Time) -> Time {
        let period = self.scl_period(clk_sys);
        let mut periods = 9;
        if step.start.is_some() {
            periods += 10;
            if !result.addr_ack {
                periods = 10;
            }
        }
        if step.stop || !result.addr_ack || (step.write.is_some() && !result.data_ack) {
            periods += 1;
        }
        let done = now + Time(period.as_ps() * periods) + result.stretch;
        self.raw |= intr::ACTIVITY;
        if step.start.is_some() {
            self.raw |= if step.restart { intr::RESTART_DET } else { intr::START_DET };
        }
        self.current = Some((step, result, done));
        done
    }

    /// When the step on the wire ends.
    pub fn next_event(&self) -> Option<Time> {
        self.current.map(|c| c.2)
    }

    /// Finishes the step on the wire if it has ended by `now`. Returns whether a step finished; the caller then
    /// starts the next one.
    pub fn finish(&mut self, now: Time) -> bool {
        let Some((step, result, done)) = self.current else { return false };
        if done > now {
            return false;
        }
        self.current = None;
        let nak = if !result.addr_ack {
            Some(abrt::ADDR7_NOACK)
        } else if step.write.is_some() && !result.data_ack {
            Some(abrt::TXDATA_NOACK)
        } else {
            None
        };
        if let Some(src) = nak {
            self.abort(src);
            return true;
        }
        if let Some(start) = step.start {
            self.held = Some(start);
        }
        if step.write.is_none() {
            if self.rx.len() < FIFO_DEPTH {
                self.rx.push_back(result.read);
            } else {
                self.raw |= intr::RX_OVER;
            }
        }
        if step.stop {
            self.held = None;
            self.raw |= intr::STOP_DET;
        }
        true
    }

    /// Aborts the transfer: flushes the TX FIFO (until IC_CLR_TX_ABRT is read) and releases the bus with a STOP.
    fn abort(&mut self, source: u32) {
        let flushed = self.tx.len() as u32;
        self.tx.clear();
        self.abrt_source = source | flushed.min(0x1FF) << 23;
        self.raw |= intr::TX_ABRT;
        let was_active = self.held.take().is_some() | self.current.take().is_some();
        if was_active || source & (abrt::ADDR7_NOACK | abrt::TXDATA_NOACK) != 0 {
            self.raw |= intr::STOP_DET;
        }
    }

    /// Whether the controller holds the bus (between START and STOP).
    pub fn holds_bus(&self) -> bool {
        self.held.is_some()
    }

    fn status(&self) -> u32 {
        let active = self.current.is_some() || self.held.is_some() || (!self.tx.is_empty() && self.enable);
        let mut v = 0;
        if active {
            v |= 1 | 1 << 5;
        }
        if self.tx.len() < FIFO_DEPTH {
            v |= 1 << 1;
        }
        if self.tx.is_empty() {
            v |= 1 << 2;
        }
        if !self.rx.is_empty() {
            v |= 1 << 3;
        }
        if self.rx.len() >= FIFO_DEPTH {
            v |= 1 << 4;
        }
        v
    }

    /// Clears latched flags on a read of a clear register; returns the register's value (the flag's old state).
    fn clear(&mut self, bits: u32) -> u32 {
        let was = self.raw & bits != 0;
        self.raw &= !bits;
        if bits & intr::TX_ABRT != 0 {
            self.abrt_source = 0;
        }
        was as u32
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            IC_CON => self.con,
            IC_TAR => self.tar,
            0x08 => self.sar,
            IC_DATA_CMD => match self.rx.pop_front() {
                Some(b) => b as u32,
                None => {
                    self.raw |= intr::RX_UNDER;
                    0
                }
            },
            0x14 => self.ss_hcnt,
            0x18 => self.ss_lcnt,
            0x1C => self.fs_hcnt,
            0x20 => self.fs_lcnt,
            0x2C => self.raw_intr() & self.intr_mask,
            0x30 => self.intr_mask,
            0x34 => self.raw_intr(),
            0x38 => self.rx_tl,
            0x3C => self.tx_tl,
            0x40 => {
                let was = (self.raw & !(intr::RX_FULL | intr::TX_EMPTY) != 0) as u32;
                self.clear(
                    intr::RX_UNDER
                        | intr::RX_OVER
                        | intr::TX_OVER
                        | intr::RD_REQ
                        | intr::TX_ABRT
                        | intr::RX_DONE
                        | intr::ACTIVITY
                        | intr::STOP_DET
                        | intr::START_DET
                        | intr::GEN_CALL
                        | intr::RESTART_DET,
                );
                was
            }
            0x44 => self.clear(intr::RX_UNDER),
            0x48 => self.clear(intr::RX_OVER),
            0x4C => self.clear(intr::TX_OVER),
            0x50 => self.clear(intr::RD_REQ),
            0x54 => self.clear(intr::TX_ABRT),
            0x58 => self.clear(intr::RX_DONE),
            0x5C => {
                // ACTIVITY stays set while the controller is active
                if self.status() & 1 != 0 { 1 } else { self.clear(intr::ACTIVITY) }
            }
            0x60 => self.clear(intr::STOP_DET),
            0x64 => self.clear(intr::START_DET),
            0x68 => self.clear(intr::GEN_CALL),
            IC_ENABLE => self.enable as u32,
            0x70 => self.status(),
            0x74 => self.tx.len() as u32,
            0x78 => self.rx.len() as u32,
            0x7C => self.sda_hold,
            0x80 => self.abrt_source,
            0x84 => 0,
            0x88 => self.dma_cr,
            0x8C => self.dma_tdlr,
            0x90 => self.dma_rdlr,
            0x94 => self.sda_setup,
            0x98 => self.ack_general_call,
            0x9C => self.enable as u32,
            0xA0 => self.fs_spklen,
            0xA8 => self.clear(intr::RESTART_DET),
            0xF4 => 0,
            0xF8 => 0x3230_312A,
            0xFC => 0x4457_0140,
            _ => return None,
        })
    }

    /// Side-effect free read.
    pub fn peek(&self, offset: u32) -> Option<u32> {
        match offset {
            IC_DATA_CMD => Some(self.rx.front().copied().unwrap_or(0) as u32),
            0x40..=0x68 | 0xA8 => Some(0),
            _ => self.clone().read(offset),
        }
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        // Configuration registers only take writes while the controller is disabled
        let cfg = !self.enable;
        match offset {
            IC_CON if cfg => mode.update(&mut self.con, value, 0x7FF),
            IC_TAR if cfg => mode.update(&mut self.tar, value, 0xFFF),
            0x08 if cfg => mode.update(&mut self.sar, value, 0x3FF),
            IC_DATA_CMD => {
                let mut v = 0;
                mode.update(&mut v, value, 0x7FF);
                if self.tx_aborted() {
                    // Flushed until IC_CLR_TX_ABRT is read
                } else if self.tx.len() >= FIFO_DEPTH {
                    self.raw |= intr::TX_OVER;
                } else if self.enable {
                    self.tx.push_back(v as u16);
                }
            }
            0x14 if cfg => mode.update(&mut self.ss_hcnt, value, 0xFFFF),
            0x18 if cfg => mode.update(&mut self.ss_lcnt, value, 0xFFFF),
            0x1C if cfg => mode.update(&mut self.fs_hcnt, value, 0xFFFF),
            0x20 if cfg => mode.update(&mut self.fs_lcnt, value, 0xFFFF),
            0x30 => mode.update(&mut self.intr_mask, value, 0x1FFF),
            0x38 => mode.update(&mut self.rx_tl, value, 0xFF),
            0x3C => mode.update(&mut self.tx_tl, value, 0xFF),
            IC_ENABLE => {
                let mut v = self.enable as u32;
                mode.update(&mut v, value, 0x7);
                if v & 2 != 0 && self.enable {
                    self.abort(abrt::USER_ABRT);
                }
                let en = v & 1 != 0;
                if self.enable && !en {
                    if self.current.is_some() || self.held.is_some() {
                        self.abort(abrt::MASTER_DIS);
                    }
                    self.tx.clear();
                    self.rx.clear();
                    self.held = None;
                    self.current = None;
                }
                self.enable = en;
            }
            0x7C => mode.update(&mut self.sda_hold, value, 0xFF_FFFF),
            0x84 => {}
            0x88 => mode.update(&mut self.dma_cr, value, 3),
            0x8C => mode.update(&mut self.dma_tdlr, value, 0xF),
            0x90 => mode.update(&mut self.dma_rdlr, value, 0xF),
            0x94 if cfg => mode.update(&mut self.sda_setup, value, 0xFF),
            0x98 => mode.update(&mut self.ack_general_call, value, 1),
            0xA0 if cfg => mode.update(&mut self.fs_spklen, value, 0xFF),
            // Read-only, or configuration written while enabled (ignored, as on hardware)
            IC_CON | IC_TAR | 0x08 | 0x14 | 0x18 | 0x1C | 0x20 | 0x94 | 0xA0 => {
                log::debug!("I2C{}: write to {offset:#x} while enabled ignored", self.index);
            }
            0x2C | 0x34 | 0x40..=0x68 | 0x70..=0x80 | 0x9C | 0xA8 | 0xF4..=0xFC => {}
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

    fn enabled(addr: u32) -> I2c {
        let mut i = I2c::new(0);
        i.write(IC_CON, 0x65 | CON_TX_EMPTY_CTRL, WriteMode::Normal);
        i.write(IC_TAR, addr, WriteMode::Normal);
        i.write(IC_ENABLE, 1, WriteMode::Normal);
        i
    }

    /// Runs every queued step against a device that ACKs everything and returns `0xA0 + n` for read n.
    fn run(i: &mut I2c, addr_ack: bool, data_ack: bool) -> Vec<Step> {
        let mut steps = Vec::new();
        let mut t = Time::ZERO;
        while let Some(s) = i.next_step() {
            let r = StepResult { addr_ack, data_ack, read: 0xA0 + steps.len() as u8, stretch: Time::ZERO };
            t = i.begin(s, r, &clk(), t);
            assert!(i.finish(t));
            steps.push(s);
        }
        steps
    }

    #[test]
    fn reset_values_and_ids() {
        let mut i = I2c::new(0);
        assert_eq!(i.read(IC_CON), Some(0x65));
        assert_eq!(i.read(0x70), Some(0x6));
        assert_eq!(i.read(0xFC), Some(0x4457_0140));
        assert_eq!(i.read(0x30), Some(0x8FF));
    }

    #[test]
    fn write_then_read_with_restart_and_stop() {
        let mut i = enabled(0x44);
        i.write(IC_DATA_CMD, 0xFD, WriteMode::Normal);
        i.write(IC_DATA_CMD, CMD_READ as u32, WriteMode::Normal);
        i.write(IC_DATA_CMD, (CMD_READ | CMD_STOP) as u32, WriteMode::Normal);
        let steps = run(&mut i, true, true);
        assert_eq!(steps.len(), 3);
        assert_eq!(steps[0].start, Some((0x44, false)));
        assert_eq!(steps[0].write, Some(0xFD));
        assert!(!steps[0].restart);
        assert_eq!(steps[1].start, Some((0x44, true)), "direction change restarts");
        assert!(steps[1].restart && steps[1].ack);
        assert_eq!((steps[2].start, steps[2].ack, steps[2].stop), (None, false, true));
        assert_eq!(i.read(0x78), Some(2));
        assert_eq!(i.read(IC_DATA_CMD), Some(0xA1));
        assert_eq!(i.read(IC_DATA_CMD), Some(0xA2));
        let raw = i.read(0x34).unwrap();
        assert!(raw & intr::STOP_DET != 0 && raw & intr::START_DET != 0 && raw & intr::RESTART_DET != 0);
        assert_eq!(i.read(0x60), Some(1));
        assert_eq!(i.read(0x34).unwrap() & intr::STOP_DET, 0);
        assert!(!i.holds_bus());
    }

    #[test]
    fn address_nak_aborts_and_flushes_until_cleared() {
        let mut i = enabled(0x50);
        i.write(0x30, intr::TX_ABRT, WriteMode::Normal);
        for _ in 0..3 {
            i.write(IC_DATA_CMD, 0x01, WriteMode::Normal);
        }
        let s = i.next_step().unwrap();
        let t = i.begin(s, StepResult { addr_ack: false, ..Default::default() }, &clk(), Time::ZERO);
        assert!(i.finish(t));
        assert!(i.irq());
        assert_eq!(i.read(0x80), Some(abrt::ADDR7_NOACK | 2 << 23), "two commands flushed");
        assert_eq!(i.read(0x74), Some(0));
        i.write(IC_DATA_CMD, 0x01, WriteMode::Normal);
        assert_eq!(i.read(0x74), Some(0), "flushed state ignores writes");
        assert!(i.next_step().is_none());
        assert_eq!(i.read(0x54), Some(1));
        assert_eq!(i.read(0x80), Some(0));
        assert!(!i.irq());
        i.write(IC_DATA_CMD, 0x01, WriteMode::Normal);
        assert_eq!(i.read(0x74), Some(1));
    }

    #[test]
    fn data_nak() {
        let mut i = enabled(0x50);
        i.write(IC_DATA_CMD, 0x01, WriteMode::Normal);
        i.write(IC_DATA_CMD, 0x02 | CMD_STOP as u32, WriteMode::Normal);
        let s = i.next_step().unwrap();
        let t = i.begin(s, StepResult { addr_ack: true, data_ack: false, ..Default::default() }, &clk(), Time::ZERO);
        i.finish(t);
        assert_eq!(i.read(0x80).unwrap() & 0xFF, abrt::TXDATA_NOACK);
        assert!(i.read(0x34).unwrap() & intr::STOP_DET != 0);
    }

    #[test]
    fn timing_follows_hcnt_lcnt_and_stretching() {
        let mut i = I2c::new(0);
        // 1 MHz at 125 MHz: 125 cycles per SCL period
        i.write(0x1C, 50, WriteMode::Normal);
        i.write(0x20, 60, WriteMode::Normal);
        i.write(0xA0, 7, WriteMode::Normal);
        i.write(IC_ENABLE, 1, WriteMode::Normal);
        assert_eq!(i.scl_period(&clk()), clk().cycles(125));
        assert_eq!(i.scl_hz(125_000_000), 1_000_000);
        i.write(IC_DATA_CMD, 0x12 | CMD_STOP as u32, WriteMode::Normal);
        let s = i.next_step().unwrap();
        let r = StepResult { addr_ack: true, data_ack: true, read: 0, stretch: Time::from_us(5) };
        let done = i.begin(s, r, &clk(), Time::ZERO);
        // START + address (10) + byte (9) + STOP (1) = 20 periods of 1 us, plus 5 us stretched
        assert_eq!(done, Time::from_us(25));
        assert!(i.read(0x70).unwrap() & 1 != 0, "active");
        assert!(!i.finish(done - Time(1)));
        assert!(i.finish(done));
        assert_eq!(i.read(0x70).unwrap() & 0x21, 0);
    }

    #[test]
    fn fifo_levels_interrupts_and_dreqs() {
        let mut i = enabled(0x10);
        i.write(0x30, intr::TX_EMPTY | intr::RX_FULL, WriteMode::Normal);
        assert!(i.irq(), "TX empty");
        for _ in 0..17 {
            i.write(IC_DATA_CMD, CMD_READ as u32, WriteMode::Normal);
        }
        assert!(i.read(0x34).unwrap() & intr::TX_OVER != 0);
        assert!(!i.irq());
        i.write(0x38, 3, WriteMode::Normal);
        i.write(0x88, 3, WriteMode::Normal);
        i.write(0x90, 1, WriteMode::Normal);
        i.write(0x8C, 4, WriteMode::Normal);
        assert!(!i.tx_dreq());
        let mut t = Time::ZERO;
        for n in 0..4 {
            let s = i.next_step().unwrap();
            t = i.begin(s, StepResult { addr_ack: true, read: n, ..Default::default() }, &clk(), t);
            i.finish(t);
            assert_eq!(i.rx_dreq(), n >= 1, "RDLR 1: two bytes");
        }
        assert!(i.irq(), "RX_FULL above RX_TL 3");
        i.read(IC_DATA_CMD);
        i.read(IC_DATA_CMD);
        i.read(IC_DATA_CMD);
        i.read(IC_DATA_CMD);
        i.read(IC_DATA_CMD);
        assert!(i.read(0x34).unwrap() & intr::RX_UNDER != 0);
        // Disabling flushes everything
        i.write(IC_ENABLE, 0, WriteMode::Normal);
        assert_eq!((i.read(0x74), i.read(0x9C)), (Some(0), Some(0)));
    }

    #[test]
    fn user_abort_and_config_locked_while_enabled() {
        let mut i = enabled(0x10);
        i.write(IC_TAR, 0x20, WriteMode::Normal);
        assert_eq!(i.read(IC_TAR), Some(0x10));
        i.write(IC_DATA_CMD, 1, WriteMode::Normal);
        i.write(IC_ENABLE, 3, WriteMode::Normal);
        assert_eq!(i.read(0x80).unwrap() & abrt::USER_ABRT, abrt::USER_ABRT);
        assert_eq!(i.read(IC_ENABLE), Some(1), "ABORT self-clears");
    }

    #[test]
    fn bus_held_between_commands_without_stop() {
        let mut i = enabled(0x68);
        i.write(IC_DATA_CMD, 0x01, WriteMode::Normal);
        run(&mut i, true, true);
        assert!(i.holds_bus());
        i.write(IC_DATA_CMD, 0x02 | CMD_STOP as u32, WriteMode::Normal);
        let s = run(&mut i, true, true);
        assert_eq!(s[0].start, None, "same transfer continues");
        assert!(!i.holds_bus());
        i.write(IC_DATA_CMD, 0x03 | CMD_RESTART as u32, WriteMode::Normal);
        let s = run(&mut i, true, true);
        assert_eq!(s[0].start, Some((0x68, false)));
        assert!(!s[0].restart, "a START after STOP is not a repeated START");
    }
}
