//! WIZnet W5500 Ethernet controller on SPI, as `embassy-net-wiznet` uses it: socket 0 in MACRAW mode, raw
//! Ethernet frames through the socket's 16 KiB TX and RX buffers, the receive interrupt on INTn, and the PHY link
//! status.
//!
//! SPI frames (datasheet 2.2): a 16-bit offset, a control byte (block select `BSB[4:0]`, read/write `RWB`, operation
//! mode `OM[1:0]`, variable length `00` only), then data until CS rises; the offset advances with every data byte
//! and wraps within the block (buffer blocks wrap at their size). Blocks: 0 common registers, `1 + 4n` socket n's
//! registers, `2 + 4n` its TX buffer, `3 + 4n` its RX buffer. Only socket 0 does anything; the others read back
//! what was written.
//!
//! The network side is an [`EthWire`]: frames the firmware sends (SEND command) are queued on it for the host, and
//! frames the host queues are moved into the RX buffer (with the W5500's 2-byte length header) when there is room,
//! setting `Sn_IR.RECV`. MAC filtering (`Sn_MR.MFEN`) passes frames for the chip's MAC, broadcasts and multicasts.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use sim_core::time::Time;
use sim_core::wire::{External, Level};

use crate::DeviceCtx;
use crate::spi::SpiDevice;

pub const VERSION: u8 = 0x04;
const BUF: usize = 16 * 1024;

// Common registers
const MR: usize = 0x00;
const SHAR: usize = 0x09;
const SIR: usize = 0x17;
const SIMR: usize = 0x18;
const PHYCFGR: usize = 0x2E;
const VERSIONR: usize = 0x39;
// Socket registers
const SN_MR: usize = 0x00;
const SN_CR: usize = 0x01;
const SN_IR: usize = 0x02;
const SN_SR: usize = 0x03;
const SN_TX_FSR: usize = 0x20;
const SN_TX_RD: usize = 0x22;
const SN_TX_WR: usize = 0x24;
const SN_RX_RSR: usize = 0x26;
const SN_RX_RD: usize = 0x28;
const SN_RX_WR: usize = 0x2A;
const SN_IMR: usize = 0x2C;

const IR_SENDOK: u8 = 1 << 4;
const IR_RECV: u8 = 1 << 2;
const SOCK_MACRAW: u8 = 0x42;
const SOCK_CLOSED: u8 = 0x00;

/// The cable between the chip and the host network.
#[derive(Debug, Default)]
pub struct WireState {
    /// Frames for the chip, oldest first.
    pub to_device: VecDeque<Vec<u8>>,
    /// Frames the chip sent, with when.
    pub from_device: Vec<(Time, Vec<u8>)>,
    /// Cable plugged in (PHY link).
    pub link_up: bool,
    /// Frames dropped because the RX buffer was full.
    pub dropped: u64,
}

pub type EthWire = Arc<Mutex<WireState>>;

pub fn wire() -> EthWire {
    Arc::new(Mutex::new(WireState { link_up: true, ..WireState::default() }))
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Addr0,
    Addr1,
    Control,
    Data { block: u8, write: bool },
}

#[derive(Clone)]
pub struct W5500 {
    pub wire: EthWire,
    pub int_pin: u8,
    common: [u8; 0x40],
    sockets: [[u8; 0x30]; 8],
    tx: Vec<u8>,
    rx: Vec<u8>,
    /// Socket 0 pointers (the chip's internal ones; registers show them).
    tx_rd: u16,
    rx_wr: u16,
    phase: Phase,
    offset: u16,
    int_low: Option<bool>,
    /// Frames sent and received.
    pub sent: u64,
    pub received: u64,
}

impl W5500 {
    pub fn new(wire: EthWire, int_pin: u8) -> Self {
        let mut w = W5500 {
            wire,
            int_pin,
            common: [0; 0x40],
            sockets: [[0; 0x30]; 8],
            tx: vec![0; BUF],
            rx: vec![0; BUF],
            tx_rd: 0,
            rx_wr: 0,
            phase: Phase::Addr0,
            offset: 0,
            int_low: None,
            sent: 0,
            received: 0,
        };
        w.reset();
        w
    }

    /// Power lost and restored (a brownout of the board's 3.3 V rail): registers back to their power-on values,
    /// sockets closed, no SPI frame in progress.
    pub fn power_on_reset(&mut self) {
        self.reset();
        self.phase = Phase::Addr0;
        self.int_low = None;
    }

    /// Software reset (MR.RST) and power-on values.
    fn reset(&mut self) {
        self.common = [0; 0x40];
        self.common[0x19] = 0x07; // RTR = 2000 (0x07D0)
        self.common[0x1A] = 0xD0;
        self.common[0x1B] = 0x08; // RCR
        self.common[VERSIONR] = VERSION;
        for s in &mut self.sockets {
            *s = [0; 0x30];
            s[0x1E] = 2; // 2 KiB buffers
            s[0x1F] = 2;
            s[SN_IMR] = 0xFF;
        }
        self.tx_rd = 0;
        self.rx_wr = 0;
    }

    fn reg16(s: &[u8; 0x30], at: usize) -> u16 {
        u16::from_be_bytes([s[at], s[at + 1]])
    }

    fn set16(s: &mut [u8; 0x30], at: usize, v: u16) {
        s[at..at + 2].copy_from_slice(&v.to_be_bytes());
    }

    /// Socket 0's derived registers.
    fn refresh(&mut self) {
        let link = self.wire.lock().unwrap().link_up;
        // PHYCFGR: RST released, all capable auto-negotiation, 100 Mbit full duplex, link bit
        self.common[PHYCFGR] = 0b1011_1110 | link as u8;
        let tx_wr = Self::reg16(&self.sockets[0], SN_TX_WR);
        let rx_rd = Self::reg16(&self.sockets[0], SN_RX_RD);
        let s = &mut self.sockets[0];
        Self::set16(s, SN_TX_FSR, (BUF as u16).wrapping_sub(tx_wr.wrapping_sub(self.tx_rd)));
        Self::set16(s, SN_TX_RD, self.tx_rd);
        Self::set16(s, SN_RX_RSR, self.rx_wr.wrapping_sub(rx_rd));
        Self::set16(s, SN_RX_WR, self.rx_wr);
        // Socket interrupt summary and the INTn pin
        let mut sir = 0u8;
        for (n, s) in self.sockets.iter().enumerate() {
            if s[SN_IR] & s[SN_IMR] != 0 {
                sir |= 1 << n;
            }
        }
        self.common[SIR] = sir;
    }

    fn drive_int(&mut self, ctx: &mut dyn DeviceCtx) {
        let low = self.common[SIR] & self.common[SIMR] != 0;
        if self.int_low != Some(low) {
            self.int_low = Some(low);
            ctx.set_pin(self.int_pin, External::Driven(if low { Level::Low } else { Level::High }));
        }
    }

    fn command(&mut self, ctx: &mut dyn DeviceCtx, cmd: u8) {
        match cmd {
            0x01 => {
                // OPEN: MACRAW when Sn_MR's protocol is 0100
                let s = &mut self.sockets[0];
                s[SN_SR] = if s[SN_MR] & 0x0F == 0x04 { SOCK_MACRAW } else { SOCK_CLOSED };
                self.tx_rd = Self::reg16(s, SN_TX_WR);
                self.rx_wr = Self::reg16(s, SN_RX_RD);
            }
            0x10 => self.sockets[0][SN_SR] = SOCK_CLOSED,
            0x20 if self.sockets[0][SN_SR] == SOCK_MACRAW => {
                // SEND: everything between TX_RD and TX_WR is one frame
                let wr = Self::reg16(&self.sockets[0], SN_TX_WR);
                let len = wr.wrapping_sub(self.tx_rd) as usize;
                let frame: Vec<u8> = (0..len).map(|i| self.tx[(self.tx_rd as usize + i) % BUF]).collect();
                self.tx_rd = wr;
                self.sockets[0][SN_IR] |= IR_SENDOK;
                self.sent += 1;
                let mut w = self.wire.lock().unwrap();
                if w.link_up {
                    w.from_device.push((ctx.now(), frame));
                }
            }
            0x40 => {
                // RECV: RX_RD (already written) frees the space; if more data is waiting, RECV is set again
                let rx_rd = Self::reg16(&self.sockets[0], SN_RX_RD);
                if self.rx_wr != rx_rd {
                    self.sockets[0][SN_IR] |= IR_RECV;
                }
            }
            _ => {}
        }
    }

    /// Moves frames from the wire into the RX buffer while they fit.
    fn take_frames(&mut self) {
        if self.sockets[0][SN_SR] != SOCK_MACRAW {
            return;
        }
        let mac: [u8; 6] = self.common[SHAR..SHAR + 6].try_into().unwrap();
        let filter = self.sockets[0][SN_MR] & 0x80 != 0;
        let mut w = self.wire.lock().unwrap();
        if !w.link_up {
            w.to_device.clear();
            return;
        }
        while let Some(frame) = w.to_device.front() {
            let dst = &frame[..6.min(frame.len())];
            if filter && dst != mac && dst[0] & 1 == 0 {
                w.to_device.pop_front();
                continue;
            }
            let rx_rd = Self::reg16(&self.sockets[0], SN_RX_RD);
            let used = self.rx_wr.wrapping_sub(rx_rd) as usize;
            let need = frame.len() + 2;
            if used + need > BUF {
                break;
            }
            let frame = w.to_device.pop_front().unwrap();
            for (i, b) in (need as u16).to_be_bytes().iter().chain(frame.iter()).enumerate() {
                self.rx[(self.rx_wr as usize + i) % BUF] = *b;
            }
            self.rx_wr = self.rx_wr.wrapping_add(need as u16);
            self.sockets[0][SN_IR] |= IR_RECV;
            self.received += 1;
        }
    }

    fn read(&mut self, block: u8, off: u16) -> u8 {
        match block {
            0 => self.common.get(off as usize).copied().unwrap_or(0),
            b if b % 4 == 1 => self.sockets[(b / 4) as usize].get(off as usize).copied().unwrap_or(0),
            2 => self.tx[off as usize % BUF],
            3 => self.rx[off as usize % BUF],
            _ => 0,
        }
    }

    fn write(&mut self, ctx: &mut dyn DeviceCtx, block: u8, off: u16, v: u8) {
        let o = off as usize;
        match block {
            0 => match o {
                MR if v & 0x80 != 0 => self.reset(),
                SIR | VERSIONR | PHYCFGR => {}
                _ if o < self.common.len() => self.common[o] = v,
                _ => {}
            },
            b if b % 4 == 1 => {
                let n = (b / 4) as usize;
                match o {
                    SN_CR if n == 0 => self.command(ctx, v),
                    SN_IR => self.sockets[n][SN_IR] &= !v,
                    SN_SR | SN_TX_FSR | 0x21 | SN_TX_RD | 0x23 | SN_RX_RSR | 0x27 | SN_RX_WR | 0x2B if n == 0 => {}
                    _ if o < 0x30 => self.sockets[n][o] = v,
                    _ => {}
                }
            }
            2 => self.tx[o % BUF] = v,
            3 => self.rx[o % BUF] = v,
            _ => {}
        }
    }
}

impl SpiDevice for W5500 {
    fn name(&self) -> String {
        "W5500".into()
    }

    fn select(&mut self, ctx: &mut dyn DeviceCtx, selected: bool) {
        self.phase = Phase::Addr0;
        if !selected {
            self.take_frames();
            self.refresh();
            self.drive_int(ctx);
        }
    }

    fn transfer(&mut self, ctx: &mut dyn DeviceCtx, mosi: u16, _bits: u8) -> u16 {
        let b = mosi as u8;
        match self.phase {
            Phase::Addr0 => {
                self.offset = (b as u16) << 8;
                self.phase = Phase::Addr1;
                0
            }
            Phase::Addr1 => {
                self.offset |= b as u16;
                self.phase = Phase::Control;
                0
            }
            Phase::Control => {
                self.refresh();
                self.phase = Phase::Data { block: b >> 3, write: b & 0x04 != 0 };
                0
            }
            Phase::Data { block, write } => {
                let off = self.offset;
                self.offset = self.offset.wrapping_add(1);
                if write {
                    self.write(ctx, block, off, b);
                    0
                } else {
                    self.read(block, off) as u16
                }
            }
        }
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        self.take_frames();
        self.refresh();
        self.drive_int(ctx);
    }

    fn clone_box(&self) -> Box<dyn SpiDevice> {
        Box::new(self.clone())
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    fn frame(w: &mut W5500, ctx: &mut TestCtx, block: u8, off: u16, write: bool, data: &[u8]) -> Vec<u8> {
        w.select(ctx, true);
        w.transfer(ctx, off >> 8, 8);
        w.transfer(ctx, off & 0xFF, 8);
        w.transfer(ctx, ((block << 3) | if write { 4 } else { 0 }) as u16, 8);
        let out = data.iter().map(|b| w.transfer(ctx, *b as u16, 8) as u8).collect();
        w.select(ctx, false);
        out
    }

    fn read(w: &mut W5500, ctx: &mut TestCtx, block: u8, off: u16, n: usize) -> Vec<u8> {
        frame(w, ctx, block, off, false, &vec![0; n])
    }

    /// The embassy-net-wiznet init sequence.
    fn init(w: &mut W5500, ctx: &mut TestCtx, mac: [u8; 6]) {
        frame(w, ctx, 0, 0x00, true, &[0x80]);
        assert_eq!(read(w, ctx, 0, 0x39, 1), [VERSION]);
        frame(w, ctx, 0, 0x18, true, &[0x01]);
        frame(w, ctx, 1, 0x2C, true, &[IR_RECV]);
        frame(w, ctx, 0, 0x09, true, &mac);
        frame(w, ctx, 1, 0x1F, true, &[16]);
        frame(w, ctx, 1, 0x1E, true, &[16]);
        frame(w, ctx, 1, 0x00, true, &[0x84]);
        frame(w, ctx, 1, 0x01, true, &[0x01]);
    }

    const MAC: [u8; 6] = [0x2A, 0xCD, 1, 2, 3, 4];

    #[test]
    fn init_send_and_receive() {
        let mut ctx = TestCtx::default();
        let wire = wire();
        let mut w = W5500::new(wire.clone(), 4);
        init(&mut w, &mut ctx, MAC);
        assert_eq!(read(&mut w, &mut ctx, 1, 0x03, 1), [SOCK_MACRAW]);
        assert_eq!(read(&mut w, &mut ctx, 0, 0x2E, 1)[0] & 1, 1, "link up");
        assert_eq!(read(&mut w, &mut ctx, 1, 0x20, 2), [0x40, 0x00], "16 KiB free");

        // Send: write at TX_WR, advance it, SEND
        let f: Vec<u8> = (0..60).collect();
        let wr = u16::from_be_bytes(read(&mut w, &mut ctx, 1, 0x24, 2).try_into().unwrap());
        frame(&mut w, &mut ctx, 2, wr, true, &f);
        frame(&mut w, &mut ctx, 1, 0x24, true, &(wr + 60).to_be_bytes());
        frame(&mut w, &mut ctx, 1, 0x01, true, &[0x20]);
        assert_eq!(wire.lock().unwrap().from_device[0].1, f);
        assert_eq!(read(&mut w, &mut ctx, 1, 0x20, 2), [0x40, 0x00]);

        // Receive: a frame for us, one for someone else (filtered), a broadcast
        let mut ours = MAC.to_vec();
        ours.extend([9u8; 54]);
        let mut theirs = vec![2, 0, 0, 0, 0, 9];
        theirs.extend([7u8; 54]);
        let mut bcast = vec![0xFF; 6];
        bcast.extend([5u8; 20]);
        wire.lock().unwrap().to_device.extend([ours.clone(), theirs, bcast.clone()]);
        w.update(&mut ctx);
        assert_eq!(ctx.pins[4], External::Driven(Level::Low), "INTn asserted");
        assert_eq!(read(&mut w, &mut ctx, 1, 0x26, 2), ((60 + 2 + 26 + 2) as u16).to_be_bytes());
        // Read the first: length header, then the frame
        let rd = u16::from_be_bytes(read(&mut w, &mut ctx, 1, 0x28, 2).try_into().unwrap());
        assert_eq!(read(&mut w, &mut ctx, 3, rd, 2), 62u16.to_be_bytes());
        assert_eq!(read(&mut w, &mut ctx, 3, rd + 2, 60), ours);
        // The driver's order: clear RECV, read, advance RX_RD, RECV command
        frame(&mut w, &mut ctx, 1, 0x02, true, &[IR_RECV]);
        frame(&mut w, &mut ctx, 1, 0x28, true, &(rd + 62).to_be_bytes());
        frame(&mut w, &mut ctx, 1, 0x01, true, &[0x40]);
        assert_eq!(read(&mut w, &mut ctx, 1, 0x26, 2), 28u16.to_be_bytes());
        assert_eq!(ctx.pins[4], External::Driven(Level::Low), "RECV set again: another frame is waiting");
        assert_eq!(read(&mut w, &mut ctx, 3, rd + 64, 26), bcast);
        frame(&mut w, &mut ctx, 1, 0x02, true, &[IR_RECV]);
        frame(&mut w, &mut ctx, 1, 0x28, true, &(rd + 90).to_be_bytes());
        frame(&mut w, &mut ctx, 1, 0x01, true, &[0x40]);
        assert_eq!(ctx.pins[4], External::Driven(Level::High), "INTn released once the buffer is empty");
    }

    #[test]
    fn buffers_wrap_and_link_down_drops() {
        let mut ctx = TestCtx::default();
        let wire = wire();
        let mut w = W5500::new(wire.clone(), 4);
        init(&mut w, &mut ctx, MAC);
        // Writing across the end of the 16 KiB TX buffer wraps to its start
        frame(&mut w, &mut ctx, 1, 0x24, true, &0x3FFEu16.to_be_bytes());
        frame(&mut w, &mut ctx, 1, 0x01, true, &[0x01]);
        frame(&mut w, &mut ctx, 2, 0x3FFE, true, &[1, 2, 3, 4]);
        frame(&mut w, &mut ctx, 1, 0x24, true, &0x4002u16.to_be_bytes());
        frame(&mut w, &mut ctx, 1, 0x01, true, &[0x20]);
        assert_eq!(wire.lock().unwrap().from_device[0].1, [1, 2, 3, 4]);
        wire.lock().unwrap().link_up = false;
        assert_eq!(read(&mut w, &mut ctx, 0, 0x2E, 1)[0] & 1, 0, "link down");
        wire.lock().unwrap().to_device.push_back(vec![0xFF; 64]);
        w.update(&mut ctx);
        assert_eq!(read(&mut w, &mut ctx, 1, 0x26, 2), [0, 0]);
    }
}
