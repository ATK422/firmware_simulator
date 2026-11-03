//! A Hiwonder LX/HTS series bus servo on a half-duplex serial line (115200 8N1).
//!
//! Packets both ways are `55 55 | id | length | command | params | checksum`, the checksum the inverted low byte
//! of the sum of id, length, command and parameters. The servo answers read commands addressed to it (or to the
//! broadcast ID 0xFE) after [`Hiwonder::reply_delay`]. Position is 0-1000 over 240°; `MOVE_TIME_WRITE` moves there
//! linearly over the given time, `MOVE_TIME_WAIT_WRITE` + `MOVE_START` likewise once started, `MOVE_STOP` stops.

use sim_core::time::Time;

use crate::DeviceCtx;
use crate::pin::{PinDevice, UartRx, uart_send};

pub const BROADCAST: u8 = 0xFE;

/// Builds a packet.
pub fn packet(id: u8, command: u8, params: &[u8]) -> Vec<u8> {
    let length = params.len() as u8 + 3;
    let sum = params.iter().fold(id as u32 + length as u32 + command as u32, |s, &p| s + p as u32);
    let mut p = vec![0x55, 0x55, id, length, command];
    p.extend_from_slice(params);
    p.push(!(sum as u8));
    p
}

#[derive(Clone, Debug)]
pub struct Hiwonder {
    pub pin: u8,
    pub id: u8,
    pub reply_delay: Time,
    pub temperature_c: u8,
    pub vin_mv: u16,
    pub loaded: bool,
    pub led_off: bool,
    pub angle_offset: i8,
    pub limits: (u16, u16),
    /// Motion: from, to, start, duration; and a move waiting for MOVE_START.
    from: f64,
    to: f64,
    started: Time,
    duration: Time,
    pending: Option<(f64, Time)>,
    rx: UartRx,
    buf: Vec<u8>,
    busy_until: Time,
    /// Commands received for this servo.
    pub commands: Vec<u8>,
    pub checksum_errors: u32,
}

impl Hiwonder {
    pub fn new(pin: u8, id: u8) -> Self {
        Hiwonder {
            pin,
            id,
            reply_delay: Time::from_us(200),
            temperature_c: 32,
            vin_mv: 7400,
            loaded: true,
            led_off: false,
            angle_offset: 0,
            limits: (0, 1000),
            from: 500.0,
            to: 500.0,
            started: Time::ZERO,
            duration: Time::ZERO,
            pending: None,
            rx: UartRx::new(115_200),
            buf: Vec::new(),
            busy_until: Time::ZERO,
            commands: Vec::new(),
            checksum_errors: 0,
        }
    }

    /// Position at `t` (0-1000).
    pub fn position(&self, t: Time) -> f64 {
        if t >= self.started + self.duration || self.duration == Time::ZERO {
            return self.to;
        }
        let f = (t - self.started).as_secs_f64() / self.duration.as_secs_f64();
        self.from + (self.to - self.from) * f.clamp(0.0, 1.0)
    }

    pub fn target(&self) -> f64 {
        self.to
    }

    fn start_move(&mut self, to: f64, over: Time, now: Time) {
        self.from = self.position(now);
        self.to = to.clamp(self.limits.0 as f64, self.limits.1 as f64);
        self.started = now;
        self.duration = over;
    }

    fn handle(&mut self, id: u8, cmd: u8, params: &[u8], now: Time) -> Option<Vec<u8>> {
        if id != self.id && id != BROADCAST {
            return None;
        }
        self.commands.push(cmd);
        let u16p = |i: usize| params.get(i..i + 2).map_or(0, |b| u16::from_le_bytes([b[0], b[1]]));
        let reply = |p: &[u8]| Some(packet(self.id, cmd, p));
        match cmd {
            1 => {
                self.start_move(u16p(0) as f64, Time::from_ms(u16p(2) as u64), now);
                None
            }
            7 => {
                self.pending = Some((u16p(0) as f64, Time::from_ms(u16p(2) as u64)));
                None
            }
            11 => {
                if let Some((to, over)) = self.pending.take() {
                    self.start_move(to, over, now);
                }
                None
            }
            12 => {
                let p = self.position(now);
                self.start_move(p, Time::ZERO, now);
                None
            }
            13 => {
                self.id = params.first().copied().unwrap_or(self.id);
                None
            }
            17 | 18 => {
                self.angle_offset = params.first().copied().unwrap_or(0) as i8;
                None
            }
            20 => {
                self.limits = (u16p(0), u16p(2));
                None
            }
            31 => {
                self.loaded = params.first() == Some(&1);
                None
            }
            33 => {
                self.led_off = params.first() == Some(&1);
                None
            }
            2 => {
                let remaining = (self.started + self.duration).saturating_sub(now).as_us() / 1000;
                let pos = (self.to as u16).to_le_bytes();
                reply(&[pos[0], pos[1], remaining as u8, (remaining >> 8) as u8])
            }
            14 => reply(&[self.id]),
            19 => reply(&[self.angle_offset as u8]),
            21 => {
                let (a, b) = (self.limits.0.to_le_bytes(), self.limits.1.to_le_bytes());
                reply(&[a[0], a[1], b[0], b[1]])
            }
            26 => reply(&[self.temperature_c]),
            27 => reply(&self.vin_mv.to_le_bytes()),
            28 => reply(&(self.position(now).round() as i16).to_le_bytes()),
            32 => reply(&[self.loaded as u8]),
            34 => reply(&[self.led_off as u8]),
            _ => None,
        }
    }

    fn parse(&mut self, ctx: &mut dyn DeviceCtx) {
        loop {
            let Some(start) = self.buf.windows(2).position(|w| w == [0x55, 0x55]) else {
                if self.buf.len() > 1 {
                    self.buf.drain(..self.buf.len() - 1);
                }
                return;
            };
            self.buf.drain(..start);
            if self.buf.len() < 4 {
                return;
            }
            let len = self.buf[3] as usize;
            if !(3..=7).contains(&len) {
                self.buf.drain(..2);
                continue;
            }
            if self.buf.len() < len + 3 {
                return;
            }
            let pkt: Vec<u8> = self.buf.drain(..len + 3).collect();
            let expect = packet(pkt[2], pkt[4], &pkt[5..len + 2]);
            if expect.last() != pkt.last() {
                self.checksum_errors += 1;
                continue;
            }
            if let Some(reply) = self.handle(pkt[2], pkt[4], &pkt[5..len + 2], ctx.now()) {
                let start = ctx.now() + self.reply_delay;
                self.busy_until = uart_send(ctx, self.pin, start, self.rx.baud, &reply);
            }
        }
    }
}

impl PinDevice for Hiwonder {
    fn name(&self) -> String {
        format!("Hiwonder servo ID {}", self.id)
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
    fn checksum_moves_and_reads() {
        // Datasheet example: move servo 1 to 500 in 1000 ms
        assert_eq!(packet(1, 1, &[0xF4, 0x01, 0xE8, 0x03]), [0x55, 0x55, 1, 7, 1, 0xF4, 0x01, 0xE8, 0x03, 0x16]);
        let mut ctx = TestCtx::default();
        let mut s = Hiwonder::new(27, 1);
        s.buf.extend(packet(1, 1, &[0xE8, 0x03, 0xE8, 0x03]));
        s.parse(&mut ctx);
        assert_eq!(s.target(), 1000.0);
        assert!((s.position(Time::from_ms(500)) - 750.0).abs() < 1e-6);
        s.buf.extend(packet(1, 28, &[]));
        ctx.now = Time::from_ms(2000);
        s.parse(&mut ctx);
        assert_eq!(ctx.scheduled.first().unwrap().0, Time::from_ms(2000) + Time::from_us(200));
        // Wrong ID: nothing; bad checksum: counted
        ctx.scheduled.clear();
        s.buf.extend(packet(2, 28, &[]));
        let mut bad = packet(1, 28, &[]);
        *bad.last_mut().unwrap() ^= 0xFF;
        s.buf.extend(bad);
        s.parse(&mut ctx);
        assert!(ctx.scheduled.is_empty());
        assert_eq!(s.checksum_errors, 1);
    }
}
