//! DMA controller (datasheet 2.5): 12 channels with the four register alias layouts, triggers (including null
//! triggers and MULTI_CHAN_TRIGGER), chaining, address rings, byte swap, IRQ_QUIET, abort, the two interrupt
//! outputs, the sniffer and the pacing timers.
//!
//! Transfers happen when the system services the DMA ([`Dma::service`]): every busy channel whose DREQ is asserted
//! moves data until the request drops or its count runs out. Peripherals with FIFOs deassert their DREQ as the FIFO
//! fills or empties, so transfers interleave with the peripheral the way they do on hardware; unpaced (permanent)
//! transfers complete at once.

use sim_core::bus::{Size, WriteMode};

pub const NUM_CHANNELS: usize = 12;

/// DREQ numbers (TREQ_SEL values).
pub mod dreq {
    pub const PIO0_TX0: u32 = 0;
    pub const PIO0_RX0: u32 = 4;
    pub const PIO1_TX0: u32 = 8;
    pub const PIO1_RX0: u32 = 12;
    pub const SPI0_TX: u32 = 16;
    pub const SPI0_RX: u32 = 17;
    pub const SPI1_TX: u32 = 18;
    pub const SPI1_RX: u32 = 19;
    pub const UART0_TX: u32 = 20;
    pub const UART0_RX: u32 = 21;
    pub const UART1_TX: u32 = 22;
    pub const UART1_RX: u32 = 23;
    pub const PWM_WRAP0: u32 = 24;
    pub const I2C0_TX: u32 = 32;
    pub const I2C0_RX: u32 = 33;
    pub const I2C1_TX: u32 = 34;
    pub const I2C1_RX: u32 = 35;
    pub const ADC: u32 = 36;
    pub const XIP_STREAM: u32 = 37;
    pub const XIP_SSITX: u32 = 38;
    pub const XIP_SSIRX: u32 = 39;
    pub const TIMER0: u32 = 0x3B;
    pub const PERMANENT: u32 = 0x3F;
}

const CTRL_EN: u32 = 1 << 0;
const CTRL_INCR_READ: u32 = 1 << 4;
const CTRL_INCR_WRITE: u32 = 1 << 5;
const CTRL_RING_SEL: u32 = 1 << 10;
const CTRL_IRQ_QUIET: u32 = 1 << 21;
const CTRL_BSWAP: u32 = 1 << 22;
const CTRL_SNIFF_EN: u32 = 1 << 23;
const CTRL_BUSY: u32 = 1 << 24;
const CTRL_WRITE_ERROR: u32 = 1 << 29;
const CTRL_READ_ERROR: u32 = 1 << 30;
/// Writable CTRL bits (BUSY and AHB_ERROR are read-only; the error flags are write-1-to-clear).
const CTRL_RW: u32 = 0x00FF_FFFF & !CTRL_BUSY;

#[derive(Clone, Debug, Default)]
pub struct Channel {
    pub read_addr: u32,
    pub write_addr: u32,
    /// Transfers left in the current sequence.
    pub count: u32,
    /// TRANS_COUNT as last written; loaded into `count` on trigger.
    pub reload: u32,
    /// CTRL without BUSY.
    pub ctrl: u32,
    pub busy: bool,
}

impl Channel {
    fn data_size(&self) -> Size {
        match (self.ctrl >> 2) & 3 {
            0 => Size::Byte,
            1 => Size::Half,
            _ => Size::Word,
        }
    }

    pub fn treq(&self) -> u32 {
        (self.ctrl >> 15) & 0x3F
    }

    fn chain_to(&self) -> usize {
        ((self.ctrl >> 11) & 0xF) as usize
    }

    fn ctrl_value(&self) -> u32 {
        let mut v = self.ctrl;
        if self.busy {
            v |= CTRL_BUSY;
        }
        if v & (CTRL_READ_ERROR | CTRL_WRITE_ERROR) != 0 {
            v |= 1 << 31;
        }
        v
    }

    /// Advances an address by one transfer, honouring the ring for the selected side.
    fn advance(&self, addr: u32, is_write: bool) -> u32 {
        let incr = if is_write { CTRL_INCR_WRITE } else { CTRL_INCR_READ };
        if self.ctrl & incr == 0 {
            return addr;
        }
        let step = self.data_size().bytes();
        let ring = (self.ctrl >> 6) & 0xF;
        let ring_on_write = self.ctrl & CTRL_RING_SEL != 0;
        if ring != 0 && ring_on_write == is_write {
            let mask = (1u32 << ring) - 1;
            (addr & !mask) | (addr.wrapping_add(step) & mask)
        } else {
            addr.wrapping_add(step)
        }
    }
}

/// What the DMA needs from the rest of the system to move data. Accesses to the DMA's own registers (control
/// blocks) are handled by [`Dma::service`] and never reach the bus.
pub trait DmaBus {
    /// Whether data request `n` is asserted.
    fn dreq(&mut self, n: u32) -> bool;
    /// Consumes one request from a pulse-type source (PWM wraps, pacing timers). Level-type sources ignore it.
    fn ack_dreq(&mut self, n: u32);
    fn dma_read(&mut self, addr: u32, size: Size) -> Option<u32>;
    fn dma_write(&mut self, addr: u32, size: Size, value: u32) -> bool;
}

#[derive(Clone, Debug, Default)]
pub struct Dma {
    pub ch: [Channel; NUM_CHANNELS],
    pub intr: u32,
    pub inte: [u32; 2],
    pub intf: [u32; 2],
    pub timers: [u32; 4],
    pub sniff_ctrl: u32,
    pub sniff_data: u32,
    /// Channels triggered but not yet serviced.
    triggered: u32,
}

/// CRC-32 (MSB first) step over one byte.
fn crc32_byte(mut crc: u32, b: u8) -> u32 {
    crc ^= (b as u32) << 24;
    for _ in 0..8 {
        crc = if crc & 0x8000_0000 != 0 { (crc << 1) ^ 0x04C1_1DB7 } else { crc << 1 };
    }
    crc
}

fn crc16_ccitt_byte(mut crc: u32, b: u8) -> u32 {
    crc ^= (b as u32) << 8;
    for _ in 0..8 {
        crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
    }
    crc & 0xFFFF
}

/// Base address of the DMA registers.
pub const DMA_BASE: u32 = 0x5000_0000;

/// Offset into the DMA's own registers (alias bits removed) if `addr` is one.
fn own_register(addr: u32) -> Option<u32> {
    (DMA_BASE..DMA_BASE + 0x4000).contains(&addr).then_some(addr & 0xFFC)
}

impl Dma {
    pub fn irq_line(&self, n: usize) -> bool {
        (self.intr | self.intf[n]) & self.inte[n] & 0xFFF != 0
    }

    /// Channels that are busy and waiting on `dreq`.
    pub fn waiting_on(&self, dreq: u32) -> bool {
        self.ch.iter().any(|c| c.busy && c.treq() == dreq)
    }

    pub fn any_busy(&self) -> bool {
        self.ch.iter().any(|c| c.busy) || self.triggered != 0
    }

    fn trigger(&mut self, n: usize) {
        let c = &mut self.ch[n];
        if c.ctrl & CTRL_EN == 0 || c.busy {
            return;
        }
        c.count = c.reload;
        c.busy = true;
        self.triggered |= 1 << n;
    }

    /// A write to a trigger register: a null trigger (zero) does not start the channel, but raises the IRQ when the
    /// channel has IRQ_QUIET set (the end of a control block chain).
    fn write_trigger(&mut self, n: usize, value: u32) {
        if value == 0 {
            if self.ch[n].ctrl & CTRL_IRQ_QUIET != 0 {
                self.intr |= 1 << n;
            }
            return;
        }
        self.trigger(n);
    }

    fn complete(&mut self, n: usize) {
        let c = &mut self.ch[n];
        c.busy = false;
        if c.ctrl & CTRL_IRQ_QUIET == 0 {
            self.intr |= 1 << n;
        }
        let chain = c.chain_to();
        if chain != n && chain < NUM_CHANNELS {
            self.trigger(chain);
        }
    }

    fn sniff(&mut self, n: usize, value: u32, size: Size) {
        let ctrl = self.sniff_ctrl;
        if ctrl & 1 == 0 || ((ctrl >> 1) & 0xF) as usize != n || self.ch[n].ctrl & CTRL_SNIFF_EN == 0 {
            return;
        }
        let mut bytes: Vec<u8> = value.to_le_bytes()[..size.bytes() as usize].to_vec();
        if ctrl & (1 << 9) != 0 {
            bytes.reverse();
        }
        let d = &mut self.sniff_data;
        match (ctrl >> 5) & 0xF {
            0 => bytes.iter().for_each(|&b| *d = crc32_byte(*d, b)),
            1 => bytes.iter().for_each(|&b| *d = crc32_byte(*d, b.reverse_bits())),
            2 => bytes.iter().for_each(|&b| *d = crc16_ccitt_byte(*d, b)),
            3 => bytes.iter().for_each(|&b| *d = crc16_ccitt_byte(*d, b.reverse_bits())),
            // XOR reduction: the parity of every bit sniffed
            0xE => *d ^= (value & size.mask()).count_ones() & 1,
            0xF => *d = d.wrapping_add(value & size.mask()),
            _ => {}
        }
    }

    fn sniff_read(&self) -> u32 {
        let mut v = self.sniff_data;
        if self.sniff_ctrl & (1 << 10) != 0 {
            v = v.reverse_bits();
        }
        if self.sniff_ctrl & (1 << 11) != 0 {
            v = !v;
        }
        v
    }

    /// Moves data on every channel that can make progress. Returns true if anything happened.
    pub fn service(&mut self, bus: &mut dyn DmaBus) -> bool {
        let mut progressed = false;
        // Channels can trigger each other through chaining and control blocks; loop until nothing moves
        for _round in 0..64 {
            self.triggered = 0;
            let mut moved = false;
            for n in 0..NUM_CHANNELS {
                while self.ch[n].busy {
                    if self.ch[n].count == 0 {
                        self.complete(n);
                        moved = true;
                        break;
                    }
                    let treq = self.ch[n].treq();
                    if treq != dreq::PERMANENT && !bus.dreq(treq) {
                        break;
                    }
                    bus.ack_dreq(treq);
                    let c = self.ch[n].clone();
                    let size = c.data_size();
                    let read = if let Some(off) = own_register(c.read_addr) {
                        self.read(off).map(|w| sim_core::bus::mem::lane(w, c.read_addr, size))
                    } else {
                        bus.dma_read(c.read_addr, size)
                    };
                    let Some(mut v) = read else {
                        self.ch[n].ctrl |= CTRL_READ_ERROR;
                        self.complete(n);
                        moved = true;
                        break;
                    };
                    if c.ctrl & CTRL_BSWAP != 0 {
                        v = match size {
                            Size::Byte => v,
                            Size::Half => (v as u16).swap_bytes() as u32,
                            Size::Word => v.swap_bytes(),
                        };
                    }
                    self.sniff(n, v, size);
                    // Control blocks: a channel writing the DMA's own registers (another channel's trigger)
                    let written = if let Some(off) = own_register(c.write_addr) {
                        let mode = match (c.write_addr >> 12) & 3 {
                            1 => WriteMode::Xor,
                            2 => WriteMode::Set,
                            3 => WriteMode::Clear,
                            _ => WriteMode::Normal,
                        };
                        self.write(off, sim_core::bus::mem::replicate(v, size), mode)
                    } else {
                        bus.dma_write(c.write_addr, size, v)
                    };
                    if !written {
                        self.ch[n].ctrl |= CTRL_WRITE_ERROR;
                        self.complete(n);
                        moved = true;
                        break;
                    }
                    let ch = &mut self.ch[n];
                    ch.read_addr = c.advance(c.read_addr, false);
                    ch.write_addr = c.advance(c.write_addr, true);
                    ch.count -= 1;
                    moved = true;
                    if ch.count == 0 {
                        self.complete(n);
                        break;
                    }
                }
            }
            progressed |= moved;
            if !moved && self.triggered == 0 {
                break;
            }
        }
        progressed
    }

    pub fn read(&self, offset: u32) -> Option<u32> {
        if offset < 0x300 {
            let n = (offset / 0x40) as usize;
            if n >= NUM_CHANNELS {
                return None;
            }
            let c = &self.ch[n];
            return Some(match offset % 0x40 {
                0x00 | 0x14 | 0x28 | 0x3C => c.read_addr,
                0x04 | 0x18 | 0x2C | 0x34 => c.write_addr,
                0x08 | 0x1C | 0x24 | 0x38 => c.count,
                _ => c.ctrl_value(),
            });
        }
        Some(match offset {
            0x400 => self.intr,
            0x404 => self.inte[0],
            0x408 => self.intf[0],
            0x40C => (self.intr | self.intf[0]) & self.inte[0],
            0x414 => self.inte[1],
            0x418 => self.intf[1],
            0x41C => (self.intr | self.intf[1]) & self.inte[1],
            0x420..=0x42C => self.timers[((offset - 0x420) / 4) as usize],
            0x430 => 0,
            0x434 => self.sniff_ctrl,
            0x438 => self.sniff_read(),
            0x440 => 0,
            0x444 => 0,
            0x448 => NUM_CHANNELS as u32,
            0x800..=0xAFC => {
                let n = ((offset - 0x800) / 0x40) as usize;
                if n >= NUM_CHANNELS {
                    return None;
                }
                match offset % 0x40 {
                    0 => 0,
                    4 => self.ch[n].reload,
                    _ => return None,
                }
            }
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        if offset < 0x300 {
            let n = (offset / 0x40) as usize;
            if n >= NUM_CHANNELS {
                return false;
            }
            let reg = offset % 0x40;
            let c = &mut self.ch[n];
            let set_ctrl = |c: &mut Channel, v: u32| {
                let errors = mode.w1c_bits(v) & (CTRL_READ_ERROR | CTRL_WRITE_ERROR);
                let new = mode.apply(c.ctrl, v);
                c.ctrl = (c.ctrl & !CTRL_RW) | (new & CTRL_RW);
                c.ctrl &= !errors;
            };
            match reg {
                0x00 | 0x14 | 0x28 | 0x3C => c.read_addr = mode.apply(c.read_addr, value),
                0x04 | 0x18 | 0x2C | 0x34 => c.write_addr = mode.apply(c.write_addr, value),
                0x08 | 0x1C | 0x24 | 0x38 => c.reload = mode.apply(c.reload, value),
                0x0C | 0x10 | 0x20 | 0x30 => set_ctrl(c, value),
                _ => unreachable!(),
            }
            // Trigger registers: CTRL_TRIG, AL1_TRANS_COUNT_TRIG, AL2_WRITE_ADDR_TRIG, AL3_READ_ADDR_TRIG
            if matches!(reg, 0x0C | 0x1C | 0x2C | 0x3C) {
                self.write_trigger(n, value);
            }
            return true;
        }
        match offset {
            0x400 | 0x40C | 0x41C => self.intr &= !(mode.w1c_bits(value) & 0xFFF),
            0x404 => mode.update(&mut self.inte[0], value, 0xFFF),
            0x408 => mode.update(&mut self.intf[0], value, 0xFFF),
            0x414 => mode.update(&mut self.inte[1], value, 0xFFF),
            0x418 => mode.update(&mut self.intf[1], value, 0xFFF),
            0x420..=0x42C => {
                let i = ((offset - 0x420) / 4) as usize;
                self.timers[i] = mode.apply(self.timers[i], value);
            }
            0x430 => {
                let bits = mode.apply(0, value) & 0xFFF;
                for n in 0..NUM_CHANNELS {
                    if bits & (1 << n) != 0 {
                        self.trigger(n);
                    }
                }
            }
            0x434 => mode.update(&mut self.sniff_ctrl, value, 0xFFF),
            0x438 => self.sniff_data = mode.apply(self.sniff_data, value),
            0x444 => {
                let bits = mode.apply(0, value) & 0xFFF;
                for n in 0..NUM_CHANNELS {
                    if bits & (1 << n) != 0 {
                        self.ch[n].busy = false;
                    }
                }
            }
            0x440 | 0x448 => {}
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 4 KiB of memory at 0x2000_0000 and a fake peripheral FIFO at 0x4000_0000 with a DREQ.
    struct Bus {
        mem: Vec<u8>,
        fifo: Vec<u32>,
        fifo_room: usize,
        pulses: u32,
    }

    impl DmaBus for Bus {
        fn dreq(&mut self, n: u32) -> bool {
            match n {
                5 => self.fifo.len() < self.fifo_room,
                24 => self.pulses > 0,
                _ => false,
            }
        }
        fn ack_dreq(&mut self, n: u32) {
            if n == 24 {
                self.pulses -= 1;
            }
        }
        fn dma_read(&mut self, addr: u32, size: Size) -> Option<u32> {
            let off = addr.checked_sub(0x2000_0000)? as usize;
            if off + size.bytes() as usize > self.mem.len() {
                return None;
            }
            Some(sim_core::bus::mem::read(&self.mem, off, size))
        }
        fn dma_write(&mut self, addr: u32, size: Size, value: u32) -> bool {
            if addr == 0x4000_0000 {
                self.fifo.push(value);
                return true;
            }
            let Some(off) = addr.checked_sub(0x2000_0000) else { return false };
            if off as usize + size.bytes() as usize > self.mem.len() {
                return false;
            }
            sim_core::bus::mem::write(&mut self.mem, off as usize, size, value);
            true
        }
    }

    fn bus() -> Bus {
        let mem = (0..4096).map(|i| i as u8).collect();
        Bus { mem, fifo: Vec::new(), fifo_room: 1000, pulses: 0 }
    }

    fn ctrl(size: u32, incr_r: bool, incr_w: bool, treq: u32, chain: u32) -> u32 {
        CTRL_EN | size << 2 | (incr_r as u32) << 4 | (incr_w as u32) << 5 | chain << 11 | treq << 15
    }

    #[test]
    fn memory_to_memory_copy_and_irq() {
        let mut d = Dma::default();
        let mut b = bus();
        d.write(0x404, 1, WriteMode::Normal); // INTE0 ch0
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x2000_0800, WriteMode::Normal);
        d.write(0x08, 4, WriteMode::Normal);
        d.write(0x0C, ctrl(2, true, true, dreq::PERMANENT, 0), WriteMode::Normal);
        assert!(d.read(0x0C).unwrap() & CTRL_BUSY != 0);
        d.service(&mut b);
        assert_eq!(&b.mem[0x800..0x810], &b.mem[0..16].to_vec()[..]);
        assert_eq!(d.read(0x0C).unwrap() & CTRL_BUSY, 0);
        assert_eq!(d.read(0x08), Some(0), "count reads remaining");
        assert_eq!(d.read(0x804), Some(4), "DBG_TCR reads the reload value");
        assert_eq!(d.read(0x00), Some(0x2000_0010));
        assert!(d.irq_line(0));
        assert!(!d.irq_line(1));
        d.write(0x40C, 1, WriteMode::Normal); // clear via INTS0
        assert!(!d.irq_line(0));
    }

    #[test]
    fn dreq_paces_transfers() {
        let mut d = Dma::default();
        let mut b = bus();
        b.fifo_room = 3;
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 8, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, 5, 0), WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(b.fifo, vec![0, 1, 2], "stops when the DREQ drops");
        assert!(d.ch[0].busy);
        b.fifo.clear();
        d.service(&mut b);
        assert_eq!(b.fifo, vec![3, 4, 5]);
        b.fifo.clear();
        d.service(&mut b);
        assert_eq!(b.fifo, vec![6, 7]);
        assert!(!d.ch[0].busy);
    }

    #[test]
    fn pulse_dreq_consumes_requests() {
        let mut d = Dma::default();
        let mut b = bus();
        b.pulses = 2;
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 5, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, 24, 0), WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(b.fifo.len(), 2);
    }

    #[test]
    fn ring_and_bswap() {
        let mut d = Dma::default();
        let mut b = bus();
        // Read 8 words from a 8-byte ring: 0,4,0,4,...
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 4, WriteMode::Normal);
        d.write(0x0C, ctrl(2, true, false, dreq::PERMANENT, 0) | 3 << 6 | CTRL_BSWAP, WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(b.fifo, vec![0x0001_0203, 0x0405_0607, 0x0001_0203, 0x0405_0607]);
    }

    #[test]
    fn chaining_and_control_blocks_with_null_trigger() {
        let mut d = Dma::default();
        let mut b = bus();
        // ch1: copies 2 words to 0x800, chains to ch2 which copies 1 word to 0x900
        d.write(0x40, 0x2000_0000, WriteMode::Normal);
        d.write(0x40 + 0x04, 0x2000_0800, WriteMode::Normal);
        d.write(0x40 + 0x08, 2, WriteMode::Normal);
        d.write(0x80, 0x2000_0100, WriteMode::Normal);
        d.write(0x80 + 0x04, 0x2000_0900, WriteMode::Normal);
        d.write(0x80 + 0x08, 1, WriteMode::Normal);
        d.write(0x80 + 0x10, ctrl(2, true, true, dreq::PERMANENT, 2), WriteMode::Normal); // AL1_CTRL: no trigger
        assert!(!d.ch[2].busy);
        d.write(0x40 + 0x0C, ctrl(2, true, true, dreq::PERMANENT, 2), WriteMode::Normal);
        d.service(&mut b);
        let first = b.mem[0x000..0x008].to_vec();
        let second = b.mem[0x100..0x104].to_vec();
        assert_eq!(&b.mem[0x800..0x808], &first[..]);
        assert_eq!(&b.mem[0x900..0x904], &second[..], "chained channel ran");
        assert!(!d.ch[1].busy && !d.ch[2].busy);
        // A null trigger with IRQ_QUIET raises the IRQ without starting the channel
        d.write(0xC0 + 0x10, ctrl(2, true, true, dreq::PERMANENT, 3) | CTRL_IRQ_QUIET, WriteMode::Normal);
        d.write(0xC0 + 0x3C, 0, WriteMode::Normal);
        assert!(!d.ch[3].busy);
        assert_eq!(d.intr & (1 << 3), 1 << 3);
    }

    #[test]
    fn control_blocks_reprogram_another_channel() {
        // The pico-examples control_blocks pattern: ch0 copies (TRANS_COUNT, READ_ADDR) pairs from a list into ch1's
        // alias 3 registers through an 8-byte write ring, triggering ch1 each time; ch1 chains back to ch0 for the
        // next block, and the final (0, 0) block is a null trigger that stops the sequence.
        let mut d = Dma::default();
        let mut b = bus();
        for (i, w) in [4u32, 0x2000_0000, 2, 0x2000_0100, 0, 0].iter().enumerate() {
            sim_core::bus::mem::write(&mut b.mem, 0x400 + 4 * i, Size::Word, *w);
        }
        d.write(0x40 + 0x04, 0x2000_0800, WriteMode::Normal);
        d.write(0x40 + 0x10, ctrl(2, true, true, dreq::PERMANENT, 0), WriteMode::Normal); // AL1_CTRL: no trigger
        d.write(0x00, 0x2000_0400, WriteMode::Normal);
        d.write(0x04, DMA_BASE + 0x40 + 0x38, WriteMode::Normal); // CH1_AL3_TRANS_COUNT
        d.write(0x08, 2, WriteMode::Normal);
        d.write(0x0C, ctrl(2, true, true, dreq::PERMANENT, 0) | CTRL_RING_SEL | 3 << 6, WriteMode::Normal);
        d.service(&mut b);
        let (a, c) = (b.mem[0x000..0x010].to_vec(), b.mem[0x100..0x108].to_vec());
        assert_eq!(&b.mem[0x800..0x810], &a[..], "first block");
        assert_eq!(&b.mem[0x810..0x818], &c[..], "second block, continuing at ch1's write address");
        assert_eq!(&b.mem[0x818..0x820], &[0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F], "nothing more");
        assert!(!d.ch[0].busy && !d.ch[1].busy);
        assert_eq!(d.read(0x00), Some(0x2000_0418), "ch0 read all three blocks");
        assert_eq!(d.read(0x04), Some(DMA_BASE + 0x40 + 0x38), "write address wrapped in the ring");
    }

    #[test]
    fn abort_and_errors() {
        let mut d = Dma::default();
        let mut b = bus();
        b.fifo_room = 0;
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 10, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, 5, 0), WriteMode::Normal);
        d.write(0x444, 1, WriteMode::Normal);
        assert!(!d.ch[0].busy);
        // Read from an unmapped address: READ_ERROR and AHB_ERROR, channel stops
        d.write(0x00, 0x1000_0000, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, dreq::PERMANENT, 0), WriteMode::Normal);
        d.service(&mut b);
        let ctrl = d.read(0x0C).unwrap();
        assert!(ctrl & CTRL_READ_ERROR != 0 && ctrl & (1 << 31) != 0);
        d.write(0x0C, CTRL_READ_ERROR, WriteMode::Normal);
        assert_eq!(d.read(0x0C).unwrap() & CTRL_READ_ERROR, 0);
    }

    #[test]
    fn multi_channel_trigger_and_sniff() {
        let mut d = Dma::default();
        let mut b = bus();
        d.write(0x434, 1 | 0xF << 5, WriteMode::Normal); // sniff ch0, 32-bit sum
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 4, WriteMode::Normal);
        d.write(0x10, ctrl(0, true, false, dreq::PERMANENT, 0) | CTRL_SNIFF_EN, WriteMode::Normal);
        d.write(0x430, 1, WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(d.read(0x438), Some(1 + 2 + 3));
        // CRC32 of "123456789" (MSB first, init 0xFFFFFFFF) = 0x0376E6E7 before output inversion (CRC-32/MPEG-2)
        b.mem[0..9].copy_from_slice(b"123456789");
        d.write(0x434, 1, WriteMode::Normal);
        d.write(0x438, 0xFFFF_FFFF, WriteMode::Normal);
        d.write(0x00, 0x2000_0000, WriteMode::Normal);
        d.write(0x08, 9, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, dreq::PERMANENT, 0) | CTRL_SNIFF_EN, WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(d.read(0x438), Some(0x0376_E6E7));
    }

    #[test]
    fn sniff_xor_reduction_is_parity() {
        // CALC 0xE: "XOR reduction over all data. == 1 if the total 1 population count is odd" (datasheet 2.5.5.2)
        let mut d = Dma::default();
        let mut b = bus();
        d.write(0x434, 1 | 0xE << 5, WriteMode::Normal);
        d.write(0x00, 0x2000_0001, WriteMode::Normal); // bytes 0x01, 0x02: two ones
        d.write(0x04, 0x4000_0000, WriteMode::Normal);
        d.write(0x08, 2, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, dreq::PERMANENT, 0) | CTRL_SNIFF_EN, WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(d.read(0x438), Some(0), "even population count");
        d.write(0x00, 0x2000_0007, WriteMode::Normal); // byte 0x07: three ones
        d.write(0x08, 1, WriteMode::Normal);
        d.write(0x0C, ctrl(0, true, false, dreq::PERMANENT, 0) | CTRL_SNIFF_EN, WriteMode::Normal);
        d.service(&mut b);
        assert_eq!(d.read(0x438), Some(1), "odd population count");
    }
}
