//! A ROBOTIS Dynamixel X-series servo (XL430-W250 by default) on a half-duplex TTL bus pin, protocol 2.0.
//!
//! Packets are `FF FF FD 00 | ID | LEN | INST | params | CRC16` (CRC-16/BUYPASS, polynomial 0x8005, over the packet
//! as sent, after byte stuffing: `FF FF FD` in the body is followed by an extra `FD`). The servo answers its own ID
//! (and broadcasts where the protocol says so) after its return delay time (register 9, 2 µs units) with a status
//! packet (instruction 0x55, error byte, parameters), as limited by the status return level (register 68).
//! Supported: PING, READ, WRITE, REG_WRITE, ACTION, FACTORY_RESET, REBOOT, SYNC_READ, SYNC_WRITE. EEPROM writes need
//! torque off (access error otherwise); writes outside the table or to read-only fields give access errors.
//!
//! The horn moves towards the goal position at the profile velocity (or the velocity limit) while torque is on;
//! present position includes the homing offset.

use sim_core::time::Time;

use crate::DeviceCtx;
use crate::pin::{PinDevice, UartRx, uart_send};

pub const XL430_MODEL: u16 = 1060;
const TABLE_LEN: usize = 148;
const BROADCAST: u8 = 0xFE;

/// CRC-16 with polynomial 0x8005, initial 0, no reflection (the Dynamixel protocol 2.0 CRC).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x8005 } else { crc << 1 };
        }
    }
    crc
}

/// Builds a protocol 2.0 packet: header, ID, length, `inst`, stuffed `params`, CRC.
pub fn packet(id: u8, inst: u8, params: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(params.len() + 4);
    body.push(inst);
    for &b in params {
        body.push(b);
        let n = body.len();
        if n >= 3 && body[n - 3..] == [0xFF, 0xFF, 0xFD] {
            body.push(0xFD);
        }
    }
    let len = (body.len() + 2) as u16;
    let mut p = vec![0xFF, 0xFF, 0xFD, 0x00, id, len as u8, (len >> 8) as u8];
    p.extend(body);
    let crc = crc16(&p);
    p.extend(crc.to_le_bytes());
    p
}

/// Removes byte stuffing from a packet body.
fn unstuff(body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(body.len());
    let mut i = 0;
    while i < body.len() {
        out.push(body[i]);
        let n = out.len();
        if n >= 3 && out[n - 3..] == [0xFF, 0xFF, 0xFD] && body.get(i + 1) == Some(&0xFD) {
            i += 1;
        }
        i += 1;
    }
    out
}

mod err {
    pub const INSTRUCTION: u8 = 2;
    pub const CRC: u8 = 3;
    pub const DATA_LENGTH: u8 = 5;
    pub const ACCESS: u8 = 7;
}

#[derive(Clone, Debug)]
pub struct Dynamixel {
    pub pin: u8,
    pub table: [u8; TABLE_LEN],
    /// Physical horn position in pulses (4096 per turn).
    pub position: f64,
    last_update: Time,
    rx: UartRx,
    buf: Vec<u8>,
    registered: Option<(u16, Vec<u8>)>,
    /// When the servo's own transmission ends (it ignores the line until then).
    busy_until: Time,
    /// Instruction packets addressed to this servo, by instruction.
    pub received: Vec<u8>,
    pub crc_errors: u32,
}

impl Dynamixel {
    pub fn xl430(pin: u8, id: u8, baud: u32) -> Self {
        let mut d = Dynamixel {
            pin,
            table: [0; TABLE_LEN],
            position: 0.0,
            last_update: Time::ZERO,
            rx: UartRx::new(baud),
            buf: Vec::new(),
            registered: None,
            busy_until: Time::ZERO,
            received: Vec::new(),
            crc_errors: 0,
        };
        d.factory_reset();
        d.table[7] = id;
        d.set_physical_position(2048.0);
        d.table[8] = match baud {
            9600 => 0,
            115_200 => 2,
            1_000_000 => 3,
            2_000_000 => 4,
            3_000_000 => 5,
            4_000_000 => 6,
            4_500_000 => 7,
            _ => 1,
        };
        d
    }

    fn factory_reset(&mut self) {
        let id = self.table[7];
        self.table = [0; TABLE_LEN];
        let mut w = |addr: usize, v: u32, n: usize| self.table[addr..addr + n].copy_from_slice(&v.to_le_bytes()[..n]);
        w(0, XL430_MODEL as u32, 2);
        w(6, 46, 1); // firmware version
        w(7, 1, 1);
        w(8, 1, 1); // 57600
        w(9, 250, 1); // 500 us
        w(11, 3, 1); // position control
        w(12, 255, 1);
        w(13, 2, 1);
        w(24, 10, 4);
        w(31, 72, 1);
        w(32, 140, 2);
        w(34, 60, 2);
        w(36, 885, 2);
        w(44, 265, 4);
        w(48, 4095, 4);
        w(52, 0, 4);
        w(63, 52, 1);
        w(68, 2, 1);
        w(76, 1000, 2);
        w(78, 100, 2);
        w(80, 4000, 2);
        w(84, 640, 2);
        w(100, 885, 2);
        w(116, 2048, 4);
        w(144, 120, 2); // 12.0 V
        w(146, 30, 1);
        if id != 0 {
            self.table[7] = id;
        }
    }

    pub fn id(&self) -> u8 {
        self.table[7]
    }

    fn u(&self, addr: usize, n: usize) -> u32 {
        let mut b = [0u8; 4];
        b[..n].copy_from_slice(&self.table[addr..addr + n]);
        u32::from_le_bytes(b)
    }

    fn set(&mut self, addr: usize, v: u32, n: usize) {
        self.table[addr..addr + n].copy_from_slice(&v.to_le_bytes()[..n]);
    }

    /// Puts the horn at `pulses` (present position follows, plus the homing offset).
    pub fn set_physical_position(&mut self, pulses: f64) {
        self.position = pulses;
        let present = (pulses + self.u(20, 4) as i32 as f64).round() as i32;
        self.set(132, present as u32, 4);
        self.set(116, present as u32, 4);
    }

    pub fn torque_enabled(&self) -> bool {
        self.table[64] != 0
    }

    pub fn goal_position(&self) -> i32 {
        self.u(116, 4) as i32
    }

    pub fn present_position(&self) -> i32 {
        self.u(132, 4) as i32
    }

    /// Moves the horn and updates the present-value registers.
    fn integrate(&mut self, now: Time) {
        if now <= self.last_update {
            return;
        }
        let dt = (now - self.last_update).as_secs_f64();
        self.last_update = now;
        let offset = self.u(20, 4) as i32 as f64;
        let goal = self.goal_position() as f64 - offset;
        let profile = self.u(112, 4);
        let vlim = if profile != 0 { profile } else { self.u(44, 4) };
        // 0.229 rpm per unit, 4096 pulses per turn
        let speed = vlim as f64 * 0.229 / 60.0 * 4096.0;
        let mut velocity = 0.0;
        if self.torque_enabled() {
            let d = goal - self.position;
            let step = speed * dt;
            if d.abs() <= step {
                self.position = goal;
            } else {
                self.position += step * d.signum();
                velocity = d.signum() * vlim as f64;
            }
        }
        let present = (self.position + offset).round() as i32;
        self.set(132, present as u32, 4);
        self.set(128, velocity as i32 as u32, 4);
        let moving = (goal - self.position).abs() > self.u(24, 4) as f64;
        self.table[122] = moving as u8;
        self.set(120, ((now.as_us() / 1000) % 32768) as u32, 2);
    }

    fn writable(&self, addr: u16, len: usize) -> bool {
        let end = addr as usize + len;
        if end > TABLE_LEN || len == 0 {
            return false;
        }
        // Read-only: model, model info, firmware version, the present-value block, realtime tick, moving
        let ro = |a: usize| a < 7 || (120..=147).contains(&a) || a == 69 || a == 70;
        let eeprom = addr < 64;
        !(addr as usize..end).any(ro) && !(eeprom && self.torque_enabled())
    }

    fn write(&mut self, addr: u16, data: &[u8]) -> u8 {
        if !self.writable(addr, data.len()) {
            return err::ACCESS;
        }
        let a = addr as usize;
        self.table[a..a + data.len()].copy_from_slice(data);
        0
    }

    fn status(&self, error: u8, params: &[u8]) -> Vec<u8> {
        let alert = if self.table[70] != 0 { 0x80 } else { 0 };
        packet(self.id(), 0x55, &[&[error | alert], params].concat())
    }

    /// Handles a complete instruction packet; returns the status packet to send, if any.
    fn handle(&mut self, id: u8, inst: u8, params: &[u8], now: Time) -> Option<Vec<u8>> {
        let me = id == self.id();
        if !me && id != BROADCAST {
            return None;
        }
        if inst == 0x55 {
            return None;
        }
        self.received.push(inst);
        self.integrate(now);
        let level = self.table[68];
        let read_level = level >= 1;
        let all = level >= 2;
        let reply = |s: &Self, e: u8, p: &[u8], allowed: bool| (me && allowed).then(|| s.status(e, p));
        let u16le = |i: usize| params.get(i..i + 2).map(|b| u16::from_le_bytes([b[0], b[1]]));
        match inst {
            // PING answers even broadcasts (all servos, staggered by ID in real buses; one servo here)
            0x01 => Some(self.status(0, &[self.table[0], self.table[1], self.table[6]])),
            0x02 => {
                let (Some(addr), Some(len)) = (u16le(0), u16le(2)) else {
                    return reply(self, err::DATA_LENGTH, &[], read_level);
                };
                let end = addr as usize + len as usize;
                if end > TABLE_LEN {
                    return reply(self, err::ACCESS, &[], read_level);
                }
                let data = self.table[addr as usize..end].to_vec();
                reply(self, 0, &data, read_level)
            }
            0x03 | 0x04 => {
                let Some(addr) = u16le(0) else { return reply(self, err::DATA_LENGTH, &[], all) };
                let data = &params[2..];
                let e = if inst == 0x03 {
                    // A new ID takes effect for the status packet too
                    self.write(addr, data)
                } else if self.writable(addr, data.len()) {
                    self.registered = Some((addr, data.to_vec()));
                    self.table[69] = 1;
                    0
                } else {
                    err::ACCESS
                };
                reply(self, e, &[], all)
            }
            0x05 => {
                if let Some((addr, data)) = self.registered.take() {
                    self.write(addr, &data);
                }
                self.table[69] = 0;
                reply(self, 0, &[], all)
            }
            0x06 => {
                self.factory_reset();
                reply(self, 0, &[], all)
            }
            0x08 => {
                self.table[64] = 0;
                reply(self, 0, &[], all)
            }
            0x82 => {
                // SYNC_READ: addr, len, then IDs; only the first listed servo answers here
                let (Some(addr), Some(len)) = (u16le(0), u16le(2)) else { return None };
                if !params[4..].contains(&self.id()) {
                    return None;
                }
                let end = (addr as usize + len as usize).min(TABLE_LEN);
                let data = self.table[addr as usize..end].to_vec();
                Some(self.status(0, &data))
            }
            0x83 => {
                // SYNC_WRITE: addr, len, then (ID, data) blocks; no status
                let (Some(addr), Some(len)) = (u16le(0), u16le(2)) else { return None };
                for block in params[4..].chunks(1 + len as usize) {
                    if block[0] == self.id() && block.len() == 1 + len as usize {
                        self.write(addr, &block[1..]);
                    }
                }
                None
            }
            _ => reply(self, err::INSTRUCTION, &[], true),
        }
    }

    /// Takes complete packets out of the receive buffer.
    fn parse(&mut self, ctx: &mut dyn DeviceCtx) {
        loop {
            let Some(start) = self.buf.windows(4).position(|w| w == [0xFF, 0xFF, 0xFD, 0x00]) else {
                if self.buf.len() > 3 {
                    self.buf.drain(..self.buf.len() - 3);
                }
                return;
            };
            self.buf.drain(..start);
            if self.buf.len() < 7 {
                return;
            }
            let len = u16::from_le_bytes([self.buf[5], self.buf[6]]) as usize;
            if self.buf.len() < 7 + len {
                return;
            }
            let pkt: Vec<u8> = self.buf.drain(..7 + len).collect();
            let crc = u16::from_le_bytes([pkt[5 + len], pkt[6 + len]]);
            let id = pkt[4];
            if crc16(&pkt[..5 + len]) != crc {
                self.crc_errors += 1;
                if id == self.id() {
                    let st = self.status(err::CRC, &[]);
                    self.send(ctx, &st);
                }
                continue;
            }
            let body = unstuff(&pkt[7..5 + len]);
            if let Some(status) = self.handle(id, body[0], &body[1..], ctx.now()) {
                self.send(ctx, &status);
            }
        }
    }

    fn send(&mut self, ctx: &mut dyn DeviceCtx, bytes: &[u8]) {
        let delay = Time::from_us(2 * self.table[9] as u64);
        let start = ctx.now() + delay;
        self.busy_until = uart_send(ctx, self.pin, start, self.rx.baud, bytes);
    }
}

impl PinDevice for Dynamixel {
    fn name(&self) -> String {
        format!("Dynamixel XL430 ID {}", self.id())
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        if ctx.now() < self.busy_until {
            return;
        }
        self.rx.edge(ctx, level);
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        if let Some(b) = self.rx.poll(ctx.now()) {
            self.buf.push(b);
            self.parse(ctx);
        }
        self.integrate(ctx.now());
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    #[test]
    fn crc_and_stuffing() {
        // e-manual example: PING to ID 1 is FF FF FD 00 01 03 00 01 19 4E
        assert_eq!(packet(1, 0x01, &[]), [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E]);
        // Its status: FF FF FD 00 01 07 00 55 00 06 04 26 65 5D
        assert_eq!(crc16(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26]), 0x5D65);
        let p = packet(1, 0x03, &[0x10, 0x00, 0xFF, 0xFF, 0xFD, 0x01]);
        assert_eq!(&p[7..13], &[0x03, 0x10, 0x00, 0xFF, 0xFF, 0xFD]);
        assert_eq!(p[13], 0xFD, "stuffed");
        assert_eq!(unstuff(&p[7..p.len() - 2]), [0x03, 0x10, 0x00, 0xFF, 0xFF, 0xFD, 0x01]);
    }

    fn exchange(d: &mut Dynamixel, ctx: &mut TestCtx, pkt: &[u8]) -> Option<Vec<u8>> {
        ctx.scheduled.clear();
        d.buf.extend_from_slice(pkt);
        d.parse(ctx);
        if ctx.scheduled.is_empty() {
            return None;
        }
        // Decode the scheduled levels back to bytes
        let mut rx = UartRx::new(d.rx.baud);
        let mut out = Vec::new();
        let sched = ctx.scheduled.clone();
        let mut wakes = Vec::new();
        for (t, _, e) in sched {
            while let Some(&w) = wakes.first().filter(|&&w| w <= t) {
                wakes.remove(0);
                out.extend(rx.poll(w));
            }
            let mut c = TestCtx { now: t, ..Default::default() };
            rx.edge(&mut c, !matches!(e, sim_core::wire::External::Driven(sim_core::wire::Level::Low)));
            wakes.extend(c.wakes);
        }
        for w in wakes {
            out.extend(rx.poll(w));
        }
        Some(out)
    }

    #[test]
    fn ping_read_write_and_motion() {
        let mut ctx = TestCtx { now: Time::from_ms(1), ..Default::default() };
        let mut d = Dynamixel::xl430(10, 2, 57_600);
        let st = exchange(&mut d, &mut ctx, &packet(2, 0x01, &[])).unwrap();
        assert_eq!(st, packet(2, 0x55, &[0, 0x24, 0x04, 46]), "model 1060, firmware 46");
        // Reply after the 500 us return delay
        assert_eq!(ctx.scheduled[0].0, Time::from_ms(1) + Time::from_us(500));
        // Other IDs are ignored
        assert!(exchange(&mut d, &mut ctx, &packet(3, 0x01, &[])).is_none());
        // READ 61 bytes of EEPROM
        let st = exchange(&mut d, &mut ctx, &packet(2, 0x02, &[0, 0, 61, 0])).unwrap();
        assert_eq!(st.len(), 11 + 61);
        assert_eq!(st[9 + 7], 2, "ID register");
        // Torque on, goal 3072: EEPROM writes are refused while torque is on
        exchange(&mut d, &mut ctx, &packet(2, 0x03, &[64, 0, 1]));
        assert!(d.torque_enabled());
        let st = exchange(&mut d, &mut ctx, &packet(2, 0x03, &[20, 0, 0, 0, 0, 0])).unwrap();
        assert_eq!(st[8], err::ACCESS);
        exchange(&mut d, &mut ctx, &packet(2, 0x03, &[116, 0, 0x00, 0x0C, 0, 0]));
        assert_eq!(d.goal_position(), 3072);
        ctx.now += Time::from_ms(100);
        d.integrate(ctx.now);
        assert_eq!(d.table[122], 1, "moving");
        ctx.now += Time::from_secs(2);
        d.integrate(ctx.now);
        assert_eq!((d.present_position(), d.table[122]), (3072, 0));
        // A corrupted packet gets a CRC error status
        let mut bad = packet(2, 0x01, &[]);
        *bad.last_mut().unwrap() ^= 1;
        let st = exchange(&mut d, &mut ctx, &bad).unwrap();
        assert_eq!(st[8], err::CRC);
        assert_eq!(d.crc_errors, 1);
    }
}
