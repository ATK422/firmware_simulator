//! SPI (ARM PL022, datasheet 4.4) in master mode.
//!
//! Frames leave the 8-deep TX FIFO one at a time; each takes `(DSS + 1)` bit periods of
//! `clk_peri / (CPSDVSR * (1 + SCR))`. When a frame completes the system exchanges it with the devices whose chip
//! select is asserted and the reply goes into the RX FIFO (or the frame itself, in loopback). DMA requests, the
//! level/timeout/overrun interrupts and the hardware chip select output are modelled. Slave mode is not.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

pub const SSPCR0: u32 = 0x00;
pub const SSPCR1: u32 = 0x04;
pub const SSPDR: u32 = 0x08;
pub const SSPSR: u32 = 0x0C;
pub const SSPCPSR: u32 = 0x10;
pub const SSPIMSC: u32 = 0x14;
pub const SSPRIS: u32 = 0x18;
pub const SSPMIS: u32 = 0x1C;
pub const SSPICR: u32 = 0x20;
pub const SSPDMACR: u32 = 0x24;

const CR1_LBM: u32 = 1 << 0;
const CR1_SSE: u32 = 1 << 1;
const CR1_MS: u32 = 1 << 2;

const SR_TFE: u32 = 1 << 0;
const SR_TNF: u32 = 1 << 1;
const SR_RNE: u32 = 1 << 2;
const SR_RFF: u32 = 1 << 3;
const SR_BSY: u32 = 1 << 4;

const INT_ROR: u32 = 1 << 0;
const INT_RT: u32 = 1 << 1;
const INT_RX: u32 = 1 << 2;
const INT_TX: u32 = 1 << 3;

const FIFO_DEPTH: usize = 8;

#[derive(Clone, Debug)]
pub struct Spi {
    pub index: u8,
    cr0: u32,
    cr1: u32,
    cpsr: u32,
    imsc: u32,
    /// Latched overrun and timeout flags.
    ris_latched: u32,
    dmacr: u32,
    tx: VecDeque<u16>,
    rx: VecDeque<u16>,
    /// Frame being shifted and when it finishes.
    shifting: Option<(u16, Time)>,
    /// When the receive timeout fires.
    rt_at: Option<Time>,
    warned_slave: bool,
}

impl Spi {
    pub fn new(index: u8) -> Self {
        Spi {
            index,
            cr0: 0,
            cr1: 0,
            cpsr: 0,
            imsc: 0,
            ris_latched: 0,
            dmacr: 0,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            shifting: None,
            rt_at: None,
            warned_slave: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Spi::new(self.index);
    }

    pub fn bits(&self) -> u8 {
        ((self.cr0 & 0xF) as u8 + 1).max(4)
    }

    fn enabled(&self) -> bool {
        self.cr1 & CR1_SSE != 0
    }

    /// Duration of one bit.
    pub fn bit_time(&self, clk_peri: &ClockPeriod) -> Option<Time> {
        let cpsdvsr = (self.cpsr & 0xFE) as u64;
        if cpsdvsr == 0 || clk_peri.is_stopped() {
            return None;
        }
        let scr = ((self.cr0 >> 8) & 0xFF) as u64;
        Some(clk_peri.cycles(cpsdvsr * (1 + scr)))
    }

    /// Serial clock frequency.
    pub fn sck_hz(&self, clk_peri_hz: u64) -> u64 {
        let cpsdvsr = (self.cpsr & 0xFE) as u64;
        let scr = ((self.cr0 >> 8) & 0xFF) as u64;
        if cpsdvsr == 0 { 0 } else { clk_peri_hz / (cpsdvsr * (1 + scr)) }
    }

    pub fn loopback(&self) -> bool {
        self.cr1 & CR1_LBM != 0
    }

    /// Whether a frame is on the wire (drives the hardware chip select low).
    pub fn busy(&self) -> bool {
        self.shifting.is_some() || (!self.tx.is_empty() && self.enabled())
    }

    pub fn tx_dreq(&self) -> bool {
        self.dmacr & 2 != 0 && self.tx.len() < FIFO_DEPTH
    }

    pub fn rx_dreq(&self) -> bool {
        self.dmacr & 1 != 0 && !self.rx.is_empty()
    }

    fn ris(&self) -> u32 {
        let mut v = self.ris_latched;
        if self.rx.len() >= FIFO_DEPTH / 2 {
            v |= INT_RX;
        }
        if self.tx.len() <= FIFO_DEPTH / 2 {
            v |= INT_TX;
        }
        v
    }

    pub fn irq(&self) -> bool {
        self.ris() & self.imsc & 0xF != 0
    }

    /// Starts the next frame if the controller is idle. Returns the frame and when it completes.
    pub fn start(&mut self, clk: &ClockPeriod, now: Time) -> Option<Time> {
        if self.shifting.is_some() || !self.enabled() {
            return self.shifting.map(|(_, t)| t);
        }
        if self.cr1 & CR1_MS != 0 {
            if !self.warned_slave {
                self.warned_slave = true;
                log::warn!("SPI{}: slave mode is not modelled", self.index);
            }
            return None;
        }
        let bit = self.bit_time(clk)?;
        let f = self.tx.pop_front()?;
        let done = now + Time(bit.as_ps() * self.bits() as u64);
        self.shifting = Some((f, done));
        Some(done)
    }

    /// The frame that completes at or before `now`, if any (its exchange is the caller's business).
    pub fn take_completed(&mut self, now: Time) -> Option<(u16, Time)> {
        match self.shifting {
            Some((f, t)) if t <= now => {
                self.shifting = None;
                Some((f, t))
            }
            _ => None,
        }
    }

    /// Stores the reply to a completed frame.
    pub fn complete(&mut self, reply: u16, at: Time, clk: &ClockPeriod) {
        let mask = (1u32 << self.bits()) - 1;
        if self.rx.len() >= FIFO_DEPTH {
            self.ris_latched |= INT_ROR;
        } else {
            self.rx.push_back((reply as u32 & mask) as u16);
        }
        // Receive timeout: 32 bit periods without a new frame while data waits
        self.rt_at = self.bit_time(clk).map(|b| at + Time(b.as_ps() * 32));
    }

    /// Time of the next thing this SPI needs to happen (frame end or receive timeout).
    pub fn next_event(&self) -> Option<Time> {
        match (self.shifting.map(|(_, t)| t), self.rt_at) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// Fires the receive timeout if due.
    pub fn check_timeout(&mut self, now: Time) {
        if let Some(t) = self.rt_at
            && t <= now
        {
            self.rt_at = None;
            if !self.rx.is_empty() {
                self.ris_latched |= INT_RT;
            }
        }
    }

    fn sr(&self) -> u32 {
        let mut v = 0;
        if self.tx.is_empty() {
            v |= SR_TFE;
        }
        if self.tx.len() < FIFO_DEPTH {
            v |= SR_TNF;
        }
        if !self.rx.is_empty() {
            v |= SR_RNE;
        }
        if self.rx.len() >= FIFO_DEPTH {
            v |= SR_RFF;
        }
        if self.busy() {
            v |= SR_BSY;
        }
        v
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            SSPCR0 => self.cr0,
            SSPCR1 => self.cr1,
            SSPDR => {
                let v = self.rx.pop_front().unwrap_or(0) as u32;
                if self.rx.is_empty() {
                    self.ris_latched &= !INT_RT;
                }
                v
            }
            SSPSR => self.sr(),
            SSPCPSR => self.cpsr,
            SSPIMSC => self.imsc,
            SSPRIS => self.ris(),
            SSPMIS => self.ris() & self.imsc,
            SSPICR => 0,
            SSPDMACR => self.dmacr,
            0xFE0 => 0x22,
            0xFE4 => 0x10,
            0xFE8 => 0x34,
            0xFEC => 0x00,
            0xFF0 => 0x0D,
            0xFF4 => 0xF0,
            0xFF8 => 0x05,
            0xFFC => 0xB1,
            _ => return None,
        })
    }

    /// Side-effect free read.
    pub fn peek(&self, offset: u32) -> Option<u32> {
        match offset {
            SSPDR => Some(self.rx.front().copied().unwrap_or(0) as u32),
            _ => self.clone().read(offset),
        }
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            SSPCR0 => mode.update(&mut self.cr0, value, 0xFFFF),
            SSPCR1 => mode.update(&mut self.cr1, value, 0xF),
            SSPDR => {
                if self.tx.len() < FIFO_DEPTH {
                    let mask = (1u32 << self.bits()) - 1;
                    self.tx.push_back((value & mask) as u16);
                }
            }
            SSPCPSR => mode.update(&mut self.cpsr, value, 0xFF),
            SSPIMSC => mode.update(&mut self.imsc, value, 0xF),
            SSPICR => self.ris_latched &= !(mode.w1c_bits(value) & (INT_ROR | INT_RT)),
            SSPDMACR => mode.update(&mut self.dmacr, value, 3),
            SSPSR | SSPRIS | SSPMIS => {}
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

    fn configured() -> Spi {
        let mut s = Spi::new(0);
        s.write(SSPCPSR, 2, WriteMode::Normal);
        s.write(SSPCR0, 7 | 61 << 8, WriteMode::Normal); // 8 bits, 125 MHz / (2 * 62) ~ 1 MHz
        s.write(SSPCR1, CR1_SSE, WriteMode::Normal);
        s
    }

    #[test]
    fn frame_timing_and_exchange() {
        let c = clk();
        let mut s = configured();
        assert_eq!(s.sck_hz(125_000_000), 1_008_064);
        s.write(SSPDR, 0xA5, WriteMode::Normal);
        assert_ne!(s.read(SSPSR).unwrap() & SR_BSY, 0);
        let done = s.start(&c, Time::ZERO).unwrap();
        assert_eq!(done, c.cycles(2 * 62 * 8), "8 bit periods");
        assert!(s.take_completed(done - Time(1)).is_none());
        let (f, t) = s.take_completed(done).unwrap();
        assert_eq!((f, t), (0xA5, done));
        s.complete(0x3C, t, &c);
        assert_eq!(s.read(SSPSR).unwrap() & (SR_RNE | SR_BSY | SR_TFE), SR_RNE | SR_TFE);
        assert_eq!(s.read(SSPDR), Some(0x3C));
        assert_eq!(s.read(SSPSR).unwrap() & SR_RNE, 0);
    }

    #[test]
    fn fifo_limits_dreqs_and_overrun() {
        let c = clk();
        let mut s = configured();
        s.write(SSPDMACR, 3, WriteMode::Normal);
        for i in 0..10 {
            s.write(SSPDR, i, WriteMode::Normal);
        }
        assert_eq!(s.read(SSPSR).unwrap() & SR_TNF, 0, "TX FIFO full at 8");
        assert!(!s.tx_dreq());
        let mut exchange = |s: &mut Spi| {
            let done = s.start(&c, Time::ZERO).unwrap();
            let (f, t) = s.take_completed(done).unwrap();
            s.complete(f, t, &c);
        };
        for _ in 0..8 {
            exchange(&mut s);
        }
        assert!(s.start(&c, Time::ZERO).is_none(), "the two extra writes were dropped");
        assert!(s.tx_dreq() && s.rx_dreq());
        assert_eq!(s.read(SSPRIS).unwrap() & INT_ROR, 0);
        s.write(SSPDR, 8, WriteMode::Normal);
        exchange(&mut s);
        assert_ne!(s.read(SSPRIS).unwrap() & INT_ROR, 0, "nine frames into an 8-deep RX FIFO");
        s.write(SSPICR, INT_ROR, WriteMode::Normal);
        assert_eq!(s.read(SSPRIS).unwrap() & INT_ROR, 0);
    }

    #[test]
    fn disabled_controller_holds_frames() {
        let c = clk();
        let mut s = configured();
        s.write(SSPCR1, CR1_SSE, WriteMode::Clear);
        s.write(SSPDR, 1, WriteMode::Normal);
        assert!(s.start(&c, Time::ZERO).is_none());
        assert_eq!(s.read(SSPSR).unwrap() & SR_BSY, 0);
    }

    #[test]
    fn receive_timeout_interrupt() {
        let c = clk();
        let mut s = configured();
        s.write(SSPIMSC, INT_RT, WriteMode::Normal);
        s.write(SSPDR, 1, WriteMode::Normal);
        let done = s.start(&c, Time::ZERO).unwrap();
        let (f, t) = s.take_completed(done).unwrap();
        s.complete(f, t, &c);
        s.check_timeout(done + Time::from_us(30));
        assert!(!s.irq(), "32 bit periods (~32 us) not yet over");
        s.check_timeout(done + Time::from_us(33));
        assert!(s.irq());
        s.read(SSPDR);
        assert!(!s.irq());
    }

    #[test]
    fn ids() {
        let mut s = Spi::new(1);
        assert_eq!(s.read(0xFE0), Some(0x22));
        assert_eq!(s.read(SSPSR), Some(SR_TFE | SR_TNF));
    }
}
