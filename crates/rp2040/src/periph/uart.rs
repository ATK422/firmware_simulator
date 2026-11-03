//! UART (ARM PL011, datasheet 4.2).
//!
//! Bytes leave the transmitter at the configured baud rate (`clk_peri / (16 * (IBRD + FBRD/64))` per bit, with
//! start, data, parity and stop bits) and are handed to the host with the time their stop bit ends. Received
//! bytes queued by the host arrive at the same rate. FIFOs (32 deep, or a single holding register with FEN clear),
//! FIFO level interrupts, the receive timeout, loopback, break and the DMA request lines are modelled. Modem lines
//! read as inactive.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;
use sim_core::time::{ClockPeriod, Time};

pub const UARTDR: u32 = 0x00;
pub const UARTRSR: u32 = 0x04;
pub const UARTFR: u32 = 0x18;
pub const UARTILPR: u32 = 0x20;
pub const UARTIBRD: u32 = 0x24;
pub const UARTFBRD: u32 = 0x28;
pub const UARTLCR_H: u32 = 0x2C;
pub const UARTCR: u32 = 0x30;
pub const UARTIFLS: u32 = 0x34;
pub const UARTIMSC: u32 = 0x38;
pub const UARTRIS: u32 = 0x3C;
pub const UARTMIS: u32 = 0x40;
pub const UARTICR: u32 = 0x44;
pub const UARTDMACR: u32 = 0x48;

const FR_BUSY: u32 = 1 << 3;
const FR_RXFE: u32 = 1 << 4;
const FR_TXFF: u32 = 1 << 5;
const FR_RXFF: u32 = 1 << 6;
const FR_TXFE: u32 = 1 << 7;

const CR_UARTEN: u32 = 1 << 0;
const CR_LBE: u32 = 1 << 7;
const CR_TXE: u32 = 1 << 8;
const CR_RXE: u32 = 1 << 9;

const LCR_BRK: u32 = 1 << 0;
const LCR_PEN: u32 = 1 << 1;
const LCR_STP2: u32 = 1 << 3;
const LCR_FEN: u32 = 1 << 4;

pub const INT_RX: u32 = 1 << 4;
pub const INT_TX: u32 = 1 << 5;
pub const INT_RT: u32 = 1 << 6;
pub const INT_OE: u32 = 1 << 10;

const FIFO_DEPTH: usize = 32;
/// Receive timeout: 32 bit periods without new data while the RX FIFO holds something.
const RT_BITS: u64 = 32;

/// Scheduler work a UART needs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UartSchedule {
    /// When the byte being shifted out finishes.
    pub tx_done: Option<Time>,
    /// When the next queued input byte finishes arriving, or the receive timeout expires.
    pub rx_next: Option<Time>,
}

#[derive(Clone, Debug)]
pub struct Uart {
    pub index: u8,
    ibrd: u32,
    fbrd: u32,
    /// LCR_H as written; the baud divisors latch when it is written.
    lcr_h: u32,
    latched_div: (u32, u32),
    pub cr: u32,
    ifls: u32,
    imsc: u32,
    ris: u32,
    dmacr: u32,
    ilpr: u32,
    rsr: u32,
    tx_fifo: VecDeque<u8>,
    /// Byte in the shift register and when it finishes.
    tx_shift: Option<(u8, Time)>,
    /// Received data with error flags in bits 8-11.
    rx_fifo: VecDeque<u16>,
    /// Bytes from the host waiting to be received, and when the next one completes.
    rx_queue: VecDeque<u8>,
    rx_next_at: Option<Time>,
    /// When the receive timeout fires, if armed.
    rt_at: Option<Time>,
    /// Bytes transmitted, with the time each finished, for the host to collect.
    pub output: Vec<(Time, u8)>,
}

impl Uart {
    pub fn new(index: u8) -> Self {
        Uart {
            index,
            ibrd: 0,
            fbrd: 0,
            lcr_h: 0,
            latched_div: (0, 0),
            cr: 0x300,
            ifls: 0x12,
            imsc: 0,
            ris: 0,
            dmacr: 0,
            ilpr: 0,
            rsr: 0,
            tx_fifo: VecDeque::new(),
            tx_shift: None,
            rx_fifo: VecDeque::new(),
            rx_queue: VecDeque::new(),
            rx_next_at: None,
            rt_at: None,
            output: Vec::new(),
        }
    }

    /// Resets the block (RESETS), keeping collected output and pending host input.
    pub fn reset(&mut self) {
        let output = std::mem::take(&mut self.output);
        let rx_queue = std::mem::take(&mut self.rx_queue);
        *self = Uart::new(self.index);
        self.output = output;
        self.rx_queue = rx_queue;
    }

    fn fifo_depth(&self) -> usize {
        if self.lcr_h & LCR_FEN != 0 { FIFO_DEPTH } else { 1 }
    }

    fn enabled(&self, bit: u32) -> bool {
        self.cr & CR_UARTEN != 0 && self.cr & bit != 0
    }

    /// Duration of one bit at the latched baud rate.
    pub fn bit_time(&self, clk_peri: &ClockPeriod) -> Option<Time> {
        let (i, f) = self.latched_div;
        let div64 = i as u64 * 64 + f as u64;
        if div64 == 0 || clk_peri.is_stopped() {
            return None;
        }
        // 16 * (div64 / 64) clk_peri cycles per bit
        let ps = clk_peri.cycles(div64).as_ps() / 4;
        Some(Time(ps.max(1)))
    }

    /// Baud rate at the latched divisors.
    pub fn baud(&self, clk_peri_hz: u64) -> u64 {
        let (i, f) = self.latched_div;
        let div64 = i as u64 * 64 + f as u64;
        (clk_peri_hz * 4).checked_div(div64).unwrap_or(0)
    }

    fn frame_bits(&self) -> u64 {
        let data = 5 + ((self.lcr_h >> 5) & 3) as u64;
        let parity = (self.lcr_h & LCR_PEN != 0) as u64;
        let stop = if self.lcr_h & LCR_STP2 != 0 { 2 } else { 1 };
        1 + data + parity + stop
    }

    fn frame_time(&self, clk: &ClockPeriod) -> Option<Time> {
        self.bit_time(clk).map(|b| Time(b.as_ps() * self.frame_bits()))
    }

    fn tx_level(&self) -> usize {
        // Trigger levels as eighths of the FIFO: 1/8, 1/4, 1/2, 3/4, 7/8
        let eighths = [1, 2, 4, 6, 7][(self.ifls & 7).min(4) as usize];
        if self.lcr_h & LCR_FEN != 0 { FIFO_DEPTH * eighths / 8 } else { 0 }
    }

    fn rx_level(&self) -> usize {
        let eighths = [1, 2, 4, 6, 7][((self.ifls >> 3) & 7).min(4) as usize];
        if self.lcr_h & LCR_FEN != 0 { FIFO_DEPTH * eighths / 8 } else { 1 }
    }

    fn update_tx_int(&mut self) {
        if self.tx_fifo.len() <= self.tx_level() {
            self.ris |= INT_TX;
        } else {
            self.ris &= !INT_TX;
        }
    }

    fn update_rx_int(&mut self) {
        if self.rx_fifo.len() >= self.rx_level() {
            self.ris |= INT_RX;
        } else {
            self.ris &= !INT_RX;
        }
    }

    pub fn irq(&self) -> bool {
        self.ris & self.imsc & 0x7FF != 0
    }

    /// DREQ to the DMA: TX wants data, RX has data.
    pub fn tx_dreq(&self) -> bool {
        self.dmacr & 2 != 0 && self.tx_fifo.len() < self.fifo_depth()
    }

    pub fn rx_dreq(&self) -> bool {
        self.dmacr & 1 != 0 && !self.rx_fifo.is_empty()
    }

    /// Queues bytes the outside world sends to this UART's RX pin.
    pub fn inject(&mut self, bytes: &[u8]) {
        self.rx_queue.extend(bytes.iter().copied());
    }

    /// Starts shifting out the next byte if the transmitter is idle.
    fn start_tx(&mut self, clk: &ClockPeriod, now: Time) {
        if self.tx_shift.is_some() || !self.enabled(CR_TXE) || self.lcr_h & LCR_BRK != 0 {
            return;
        }
        let Some(frame) = self.frame_time(clk) else { return };
        if let Some(b) = self.tx_fifo.pop_front() {
            self.tx_shift = Some((b, now + frame));
            self.update_tx_int();
        }
    }

    fn arm_rx(&mut self, clk: &ClockPeriod, now: Time) {
        if self.rx_next_at.is_none()
            && !self.rx_queue.is_empty()
            && self.enabled(CR_RXE)
            && let Some(frame) = self.frame_time(clk)
        {
            self.rx_next_at = Some(now + frame);
        }
    }

    fn receive(&mut self, byte: u16, clk: &ClockPeriod, now: Time) {
        if self.rx_fifo.len() >= self.fifo_depth() {
            self.ris |= INT_OE;
            self.rsr |= 1 << 3;
            if let Some(last) = self.rx_fifo.back_mut() {
                *last |= 1 << 11;
            }
        } else {
            self.rx_fifo.push_back(byte);
        }
        self.update_rx_int();
        self.rt_at = self.bit_time(clk).map(|b| now + Time(b.as_ps() * RT_BITS));
    }

    /// Advances the transmitter and receiver to `now`. Returns what to schedule next.
    pub fn advance(&mut self, clk: &ClockPeriod, now: Time) -> UartSchedule {
        while let Some((b, done)) = self.tx_shift {
            if done > now {
                break;
            }
            self.tx_shift = None;
            if self.cr & CR_LBE != 0 {
                self.receive(b as u16, clk, done);
            } else {
                self.output.push((done, b));
            }
            self.start_tx(clk, done);
        }
        while let Some(at) = self.rx_next_at {
            if at > now {
                break;
            }
            self.rx_next_at = None;
            if let Some(b) = self.rx_queue.pop_front() {
                self.receive(b as u16, clk, at);
            }
            self.arm_rx(clk, at);
        }
        if let Some(rt) = self.rt_at
            && rt <= now
        {
            self.rt_at = None;
            if !self.rx_fifo.is_empty() {
                self.ris |= INT_RT;
            }
        }
        self.start_tx(clk, now);
        self.arm_rx(clk, now);
        UartSchedule {
            tx_done: self.tx_shift.map(|(_, t)| t),
            rx_next: match (self.rx_next_at, self.rt_at) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            },
        }
    }

    fn fr(&self) -> u32 {
        let mut v = 0;
        if self.tx_shift.is_some() || !self.tx_fifo.is_empty() {
            v |= FR_BUSY;
        }
        if self.rx_fifo.is_empty() {
            v |= FR_RXFE;
        }
        if self.rx_fifo.len() >= self.fifo_depth() {
            v |= FR_RXFF;
        }
        if self.tx_fifo.len() >= self.fifo_depth() {
            v |= FR_TXFF;
        }
        if self.tx_fifo.is_empty() {
            v |= FR_TXFE;
        }
        v
    }

    pub fn read(&mut self, clk: &ClockPeriod, now: Time, offset: u32) -> Option<u32> {
        self.advance(clk, now);
        Some(match offset {
            UARTDR => match self.rx_fifo.pop_front() {
                Some(v) => {
                    self.rsr = ((v >> 8) & 0xF) as u32;
                    self.update_rx_int();
                    if self.rx_fifo.is_empty() {
                        self.ris &= !INT_RT;
                    }
                    v as u32
                }
                None => 0,
            },
            UARTRSR => self.rsr,
            UARTFR => self.fr(),
            UARTILPR => self.ilpr,
            UARTIBRD => self.ibrd,
            UARTFBRD => self.fbrd,
            UARTLCR_H => self.lcr_h,
            UARTCR => self.cr,
            UARTIFLS => self.ifls,
            UARTIMSC => self.imsc,
            UARTRIS => self.ris,
            UARTMIS => self.ris & self.imsc,
            UARTICR => 0,
            UARTDMACR => self.dmacr,
            0xFE0 => 0x11,
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

    pub fn write(&mut self, clk: &ClockPeriod, now: Time, offset: u32, value: u32, mode: WriteMode) -> bool {
        self.advance(clk, now);
        match offset {
            UARTDR => {
                if self.tx_fifo.len() < self.fifo_depth() {
                    self.tx_fifo.push_back(value as u8);
                } else {
                    log::debug!("UART{} TX FIFO overflow, byte dropped", self.index);
                }
                self.update_tx_int();
                self.start_tx(clk, now);
            }
            UARTRSR => self.rsr = 0,
            UARTILPR => mode.update(&mut self.ilpr, value, 0xFF),
            UARTIBRD => mode.update(&mut self.ibrd, value, 0xFFFF),
            UARTFBRD => mode.update(&mut self.fbrd, value, 0x3F),
            UARTLCR_H => {
                let old_fen = self.lcr_h & LCR_FEN;
                mode.update(&mut self.lcr_h, value, 0xFF);
                // Writing LCR_H latches the baud divisors
                self.latched_div = (self.ibrd, self.fbrd);
                if self.lcr_h & LCR_FEN != old_fen {
                    // Changing FEN flushes the FIFOs
                    self.tx_fifo.clear();
                    self.rx_fifo.clear();
                }
                self.start_tx(clk, now);
            }
            UARTCR => {
                mode.update(&mut self.cr, value, 0xFF87);
                self.start_tx(clk, now);
                self.arm_rx(clk, now);
            }
            UARTIFLS => mode.update(&mut self.ifls, value, 0x3F),
            UARTIMSC => mode.update(&mut self.imsc, value, 0x7FF),
            UARTICR => self.ris &= !(mode.w1c_bits(value) & 0x7FF),
            UARTDMACR => mode.update(&mut self.dmacr, value, 7),
            UARTFR | UARTRIS | UARTMIS => {}
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

    /// 115200 baud as pico-sdk computes it at 125 MHz: IBRD 67, FBRD 52.
    fn configured(fifo: bool) -> Uart {
        let mut u = Uart::new(0);
        let c = clk();
        u.write(&c, Time::ZERO, UARTIBRD, 67, WriteMode::Normal);
        u.write(&c, Time::ZERO, UARTFBRD, 52, WriteMode::Normal);
        u.write(&c, Time::ZERO, UARTLCR_H, 0b11 << 5 | if fifo { LCR_FEN } else { 0 }, WriteMode::Normal);
        u.write(&c, Time::ZERO, UARTCR, CR_UARTEN | CR_TXE | CR_RXE, WriteMode::Normal);
        u
    }

    #[test]
    fn baud_and_frame_timing() {
        let u = configured(true);
        let baud = u.baud(125_000_000);
        assert!((115_100..115_300).contains(&baud), "{baud}");
        let frame = u.frame_time(&clk()).unwrap();
        // 10 bits at ~115207 baud = 86.8 us
        assert!((86_700..86_900).contains(&frame.as_ns()), "{frame:?}");
    }

    #[test]
    fn transmit_paces_bytes() {
        let c = clk();
        let mut u = configured(true);
        for b in b"Hi!" {
            u.write(&c, Time::ZERO, UARTDR, *b as u32, WriteMode::Normal);
        }
        assert_eq!(u.read(&c, Time::ZERO, UARTFR).unwrap() & (FR_BUSY | FR_TXFE), FR_BUSY);
        let s = u.advance(&c, Time::from_us(100));
        assert_eq!(u.output.len(), 1);
        assert!(s.tx_done.is_some());
        u.advance(&c, Time::from_us(300));
        let bytes: Vec<u8> = u.output.iter().map(|(_, b)| *b).collect();
        assert_eq!(bytes, b"Hi!");
        let gaps: Vec<u64> = u.output.windows(2).map(|w| (w[1].0 - w[0].0).as_ns()).collect();
        assert!(gaps.iter().all(|&g| (86_700..86_900).contains(&g)), "{gaps:?}");
        assert_eq!(u.read(&c, Time::from_us(300), UARTFR).unwrap() & (FR_BUSY | FR_TXFE), FR_TXFE);
    }

    #[test]
    fn holding_register_without_fifo() {
        let c = clk();
        let mut u = configured(false);
        u.write(&c, Time::ZERO, UARTDR, b'a' as u32, WriteMode::Normal);
        // Moved straight into the shift register: holding register free again
        assert_eq!(u.read(&c, Time::ZERO, UARTFR).unwrap() & FR_TXFF, 0);
        u.write(&c, Time::ZERO, UARTDR, b'b' as u32, WriteMode::Normal);
        assert_ne!(u.read(&c, Time::ZERO, UARTFR).unwrap() & FR_TXFF, 0, "one byte holds a 1-deep FIFO");
    }

    #[test]
    fn receive_interrupts_and_timeout() {
        let c = clk();
        let mut u = configured(true);
        u.write(&c, Time::ZERO, UARTIMSC, INT_RX | INT_RT, WriteMode::Normal);
        u.inject(b"xyz");
        let s = u.advance(&c, Time::ZERO);
        assert!(s.rx_next.is_some());
        u.advance(&c, Time::from_us(300));
        assert_eq!(u.read(&c, Time::from_us(300), UARTFR).unwrap() & FR_RXFE, 0);
        // 3 bytes < 1/2 FIFO trigger level: only the receive timeout interrupts
        assert!(!u.irq());
        u.advance(&c, Time::from_us(600));
        assert!(u.irq(), "receive timeout");
        assert_eq!(u.read(&c, Time::from_us(600), UARTMIS), Some(INT_RT));
        let got: Vec<u32> = (0..3).map(|_| u.read(&c, Time::from_us(600), UARTDR).unwrap()).collect();
        assert_eq!(got, vec![b'x' as u32, b'y' as u32, b'z' as u32]);
        assert!(!u.irq(), "draining clears the timeout");
    }

    #[test]
    fn loopback_and_dma_requests() {
        let c = clk();
        let mut u = configured(true);
        u.write(&c, Time::ZERO, UARTCR, CR_LBE, WriteMode::Set);
        u.write(&c, Time::ZERO, UARTDMACR, 3, WriteMode::Normal);
        assert!(u.tx_dreq());
        assert!(!u.rx_dreq());
        u.write(&c, Time::ZERO, UARTDR, 0x55, WriteMode::Normal);
        u.advance(&c, Time::from_us(100));
        assert!(u.output.is_empty());
        assert!(u.rx_dreq());
        assert_eq!(u.read(&c, Time::from_us(100), UARTDR), Some(0x55));
    }

    #[test]
    fn ids_and_reset_values() {
        let c = clk();
        let mut u = Uart::new(1);
        assert_eq!(u.read(&c, Time::ZERO, UARTCR), Some(0x300));
        assert_eq!(u.read(&c, Time::ZERO, UARTIFLS), Some(0x12));
        assert_eq!(u.read(&c, Time::ZERO, 0xFE0), Some(0x11));
        assert_eq!(u.read(&c, Time::ZERO, UARTFR), Some(FR_RXFE | FR_TXFE));
    }
}
