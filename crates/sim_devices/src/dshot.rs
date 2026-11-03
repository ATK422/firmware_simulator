//! A DShot ESC and its motor, on a DShot signal pin and (optionally) a UART telemetry pin.
//!
//! Frames are decoded from the signal: 16 bits (11-bit throttle, telemetry request, 4-bit checksum), each bit one
//! pulse whose asserted part is 37.5 % (0) or 75 % (1) of the bit period. The bit rate (DShot150-1200) is measured
//! from the frame. A line that idles high is bidirectional DShot: asserted means low and the checksum is inverted.
//!
//! In bidirectional mode the ESC answers every valid frame [`Esc::reply_delay`] (30 µs) after it ends with its
//! eRPM: 21 bits at 5/4 of the bit rate, a start bit then four GCR symbols (12-bit period in µs as a 3-bit exponent
//! and 9-bit mantissa, 4-bit checksum) sent as level changes. A frame with the telemetry bit set is also answered on
//! the telemetry pin with a 10-byte KISS frame (temperature, voltage, current, consumption, eRPM/100, CRC-8) at
//! 115200 baud.
//!
//! The motor follows the throttle with a first-order lag: throttle 48-1047 forward and 1049-2047 reverse (3D
//! mode, as the UWRT ESCs are patched), `max_rpm` at full throttle; 0 and the special commands 1-47 stop it.

use sim_core::time::Time;
use sim_core::wire::{External, Level};

use crate::DeviceCtx;
use crate::pin::PinDevice;

/// Nibble to 5-bit GCR symbol.
pub const GCR: [u32; 16] =
    [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

/// The checksum of a 12-bit DShot command (throttle and telemetry bit); `inverted` for bidirectional DShot.
pub fn checksum(cmd: u16, inverted: bool) -> u16 {
    let c = (cmd ^ (cmd >> 4) ^ (cmd >> 8)) & 0xF;
    if inverted { !c & 0xF } else { c }
}

/// The 21 line levels (MSB first, start bit included) of an eRPM reply carrying `period_us`.
pub fn erpm_reply_bits(period_us: u32) -> u32 {
    // 3-bit exponent, 9-bit mantissa
    let data = if period_us >= 0x1FF << 7 {
        0xFFF
    } else {
        let mut s = 0;
        while period_us >> s >= 512 {
            s += 1;
        }
        s << 9 | period_us >> s
    };
    let crc = !(data ^ (data >> 4) ^ (data >> 8)) & 0xF;
    let value = data << 4 | crc;
    let gcr = (0..4).fold(0u32, |g, i| g << 5 | GCR[((value >> (12 - 4 * i)) & 0xF) as usize]);
    // Bit 20 is the start bit (0); each following level changes where the GCR bit is 1
    let mut packet = 0u32;
    for i in (0..20).rev() {
        let prev = (packet >> (i + 1)) & 1;
        packet |= (prev ^ ((gcr >> i) & 1)) << i;
    }
    packet
}

/// One decoded frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub at: Time,
    pub throttle: u16,
    pub telemetry: bool,
    pub checksum_ok: bool,
    pub bidirectional: bool,
    /// Measured bit rate, kbit/s.
    pub rate_kbps: u32,
}

#[derive(Clone, Debug)]
pub struct Esc {
    pub name: String,
    pub signal_pin: u8,
    pub telemetry_pin: Option<u8>,
    /// Mechanical RPM at full throttle, motor pole pairs, time constant of the motor's response.
    pub max_rpm: f64,
    pub pole_pairs: u32,
    pub tau: Time,
    pub reply_delay: Time,
    /// Telemetry values (temperature °C, volts, amps; consumption is integrated).
    pub temperature_c: f64,
    pub voltage_v: f64,
    pub current_per_krpm_a: f64,
    pub frames: Vec<Frame>,
    pub keep_frames: usize,
    pub bad_frames: u32,
    pub replies: u32,
    pub telemetry_sent: u32,
    /// Motor state: signed mechanical RPM, its target, and when it was last integrated.
    pub rpm: f64,
    target_rpm: f64,
    last_update: Time,
    consumption_mah: f64,
    // Decoder
    idle_high: Option<bool>,
    level: bool,
    edge: Option<Time>,
    bit_start: Option<Time>,
    bits: Vec<(Time, Time)>,
    busy_until: Time,
}

impl Esc {
    pub fn new(name: &str, signal_pin: u8, telemetry_pin: Option<u8>) -> Self {
        Esc {
            name: name.into(),
            signal_pin,
            telemetry_pin,
            max_rpm: 4000.0,
            pole_pairs: 7,
            tau: Time::from_ms(60),
            reply_delay: Time::from_us(30),
            temperature_c: 35.0,
            voltage_v: 16.0,
            current_per_krpm_a: 2.5,
            frames: Vec::new(),
            keep_frames: 20_000,
            bad_frames: 0,
            replies: 0,
            telemetry_sent: 0,
            rpm: 0.0,
            target_rpm: 0.0,
            last_update: Time::ZERO,
            consumption_mah: 0.0,
            idle_high: None,
            level: false,
            edge: None,
            bit_start: None,
            bits: Vec::new(),
            busy_until: Time::ZERO,
        }
    }

    fn integrate(&mut self, now: Time) {
        if now <= self.last_update {
            return;
        }
        let dt = (now - self.last_update).as_secs_f64();
        let k = 1.0 - (-dt / self.tau.as_secs_f64()).exp();
        self.rpm += (self.target_rpm - self.rpm) * k;
        self.consumption_mah += self.current_a() * dt / 3.6;
        self.last_update = now;
    }

    /// The motor's signed mechanical RPM at `now`: the lag towards the last commanded speed carried on from the
    /// last frame (`rpm` is only brought up to date when a frame arrives). For a vehicle simulation reading the
    /// propeller speed between frames.
    pub fn rpm_at(&self, now: Time) -> f64 {
        if now <= self.last_update {
            return self.rpm;
        }
        let dt = (now - self.last_update).as_secs_f64();
        self.rpm + (self.target_rpm - self.rpm) * (1.0 - (-dt / self.tau.as_secs_f64()).exp())
    }

    fn current_a(&self) -> f64 {
        self.rpm.abs() / 1000.0 * self.current_per_krpm_a
    }

    /// Electrical RPM.
    pub fn erpm(&self) -> f64 {
        self.rpm.abs() * self.pole_pairs as f64
    }

    /// The eRPM period the ESC reports, µs (the "stopped" value below about 130 eRPM).
    pub fn period_us(&self) -> u32 {
        let e = self.erpm();
        if e < 60e6 / (0x1FF << 7) as f64 { 0x1FF << 7 } else { (60e6 / e) as u32 }
    }

    fn apply_throttle(&mut self, v: u16) {
        self.target_rpm = match v {
            48..=1047 => (v - 47) as f64 / 1000.0 * self.max_rpm,
            1049..=2047 => -((v - 1048) as f64 / 999.0 * self.max_rpm),
            _ => 0.0,
        };
    }

    fn finish_frame(&mut self, ctx: &mut dyn DeviceCtx) {
        let bits = std::mem::take(&mut self.bits);
        if bits.len() != 16 {
            if !bits.is_empty() {
                self.bad_frames += 1;
            }
            return;
        }
        let period = (bits[15].0 - bits[0].0).as_ps() as f64 / 15.0;
        let value =
            bits.iter().fold(0u16, |v, &(_, asserted)| v << 1 | (asserted.as_ps() as f64 > period * 0.56) as u16);
        let bidir = self.idle_high == Some(true);
        let cmd = value >> 4;
        let ok = checksum(cmd, bidir) == value & 0xF;
        let rate_kbps = (1e9 / period).round() as u32;
        let now = ctx.now();
        let frame = Frame {
            at: bits[0].0,
            throttle: cmd >> 1,
            telemetry: cmd & 1 != 0,
            checksum_ok: ok,
            bidirectional: bidir,
            rate_kbps,
        };
        self.frames.push(frame);
        if self.frames.len() > self.keep_frames {
            self.frames.drain(..self.keep_frames / 2);
        }
        if !ok {
            self.bad_frames += 1;
            ctx.report(format!("{}: bad DShot checksum in {value:#06x}", self.name));
            return;
        }
        self.integrate(now);
        self.apply_throttle(frame.throttle);
        let frame_end = bits[15].0 + Time((period) as u64);
        if bidir {
            self.reply(ctx, frame_end + self.reply_delay, period / 1.25);
        }
        if frame.telemetry {
            self.send_telemetry(ctx, frame_end + Time::from_us(100));
        }
    }

    fn reply(&mut self, ctx: &mut dyn DeviceCtx, start: Time, bit_ps: f64) {
        let packet = erpm_reply_bits(self.period_us());
        let pin = self.signal_pin;
        let mut level = true;
        for i in 0..21 {
            let l = (packet >> (20 - i)) & 1 != 0;
            if l != level || i == 0 {
                let ext = if l { External::Floating } else { External::Driven(Level::Low) };
                ctx.drive_at(start + Time((i as f64 * bit_ps) as u64), pin, ext);
                level = l;
            }
        }
        let end = start + Time((21.0 * bit_ps) as u64);
        ctx.drive_at(end, pin, External::Floating);
        self.busy_until = end;
        self.replies += 1;
    }

    /// The 10-byte KISS telemetry frame for the current state.
    pub fn telemetry_frame(&self) -> [u8; 10] {
        let w = |v: f64| (v.round().clamp(0.0, 65535.0) as u16).to_be_bytes();
        let mut f = [0u8; 10];
        f[0] = self.temperature_c.round().clamp(0.0, 255.0) as u8;
        f[1..3].copy_from_slice(&w(self.voltage_v * 100.0));
        f[3..5].copy_from_slice(&w(self.current_a() * 100.0));
        f[5..7].copy_from_slice(&w(self.consumption_mah));
        f[7..9].copy_from_slice(&w(self.erpm() / 100.0));
        f[9] = f[..9].iter().fold(0u8, |crc, &b| {
            let mut c = crc ^ b;
            for _ in 0..8 {
                c = if c & 0x80 != 0 { 0x07 ^ (c << 1) } else { c << 1 };
            }
            c
        });
        f
    }

    fn send_telemetry(&mut self, ctx: &mut dyn DeviceCtx, start: Time) {
        let Some(pin) = self.telemetry_pin else { return };
        let bit = Time::from_ps(1_000_000_000_000 / 115_200);
        let mut t = start;
        for b in self.telemetry_frame() {
            let mut levels = vec![false];
            levels.extend((0..8).map(|i| b >> i & 1 != 0));
            levels.push(true);
            for l in levels {
                ctx.drive_at(t, pin, External::Driven(Level::from_bool(l)));
                t += bit;
            }
        }
        self.telemetry_sent += 1;
    }

    /// The last valid frame's throttle.
    pub fn last_throttle(&self) -> Option<u16> {
        self.frames.iter().rev().find(|f| f.checksum_ok).map(|f| f.throttle)
    }
}

impl PinDevice for Esc {
    fn name(&self) -> String {
        self.name.clone()
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.signal_pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        let now = ctx.now();
        if now < self.busy_until || level == self.level {
            self.level = level;
            return;
        }
        // The idle level is the one the line rests at between frames
        if let Some(e) = self.edge
            && now - e > Time::from_us(20)
        {
            self.idle_high = Some(self.level);
            if !self.bits.is_empty() {
                self.finish_frame(ctx);
            }
        }
        self.level = level;
        self.edge = Some(now);
        let asserting = Some(level) != self.idle_high;
        if self.idle_high.is_none() {
            return;
        }
        if asserting {
            self.bit_start = Some(now);
        } else if let Some(s) = self.bit_start.take() {
            self.bits.push((s, now - s));
            if self.bits.len() == 16 {
                // The 16th bit ends with the release: finish once its period is over
                ctx.wake_at(now + Time::from_us(3));
            }
        }
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        if self.bits.len() == 16 && self.edge.is_some_and(|e| ctx.now() - e >= Time::from_us(2)) {
            self.finish_frame(ctx);
        }
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    /// The decoder of `esc_core::erpm`, written out again: undo the level-change coding, then GCR.
    fn decode(packet: u32) -> Option<u32> {
        let gcr = packet ^ (packet >> 1);
        let nib = |s: u32| GCR.iter().position(|&g| g == (gcr >> s) & 0x1F).map(|v| v as u32);
        let (crc, low, mid, high) = (nib(0)?, nib(5)?, nib(10)?, nib(15)?);
        if !(low ^ mid ^ high) & 0xF != crc {
            return None;
        }
        let data = high << 8 | mid << 4 | low;
        Some((data & 0x1FF) << (data >> 9))
    }

    #[test]
    fn erpm_encoding_round_trips() {
        for period in [100u32, 511, 512, 1000, 4321, 30_000, 0x1FF << 7] {
            let p = erpm_reply_bits(period);
            assert_eq!(p >> 20, 0, "start bit low");
            let back = decode(p).unwrap();
            // Truncated to 9 significant bits
            assert!(back <= period && period - back < period / 256 + 1, "{period} -> {back}");
        }
        assert_eq!(checksum(0x123 << 1, false), (0x246 ^ 0x24 ^ 0x2) & 0xF);
    }

    /// Plays a bidirectional DShot600 frame on the pin (idle high, asserted low).
    fn play(e: &mut Esc, ctx: &mut TestCtx, frame: u16) {
        let bit = 1_666_667u64;
        for i in (0..16).rev() {
            let one = frame >> i & 1 != 0;
            e.pin_changed(ctx, 6, false);
            let low = if one { bit * 3 / 4 } else { bit * 3 / 8 };
            ctx.now += Time(low);
            e.pin_changed(ctx, 6, true);
            ctx.now += Time(bit - low);
        }
        ctx.now += Time::from_us(5);
        e.update(ctx);
    }

    fn bidir_frame(throttle: u16, telemetry: bool) -> u16 {
        let cmd = throttle << 1 | telemetry as u16;
        cmd << 4 | checksum(cmd, true)
    }

    #[test]
    fn decodes_frames_drives_the_motor_and_replies() {
        let mut ctx = TestCtx { now: Time::from_ms(1), ..Default::default() };
        let mut e = Esc::new("esc", 6, Some(7));
        // The line rests high first
        e.pin_changed(&mut ctx, 6, true);
        ctx.now += Time::from_us(100);
        play(&mut e, &mut ctx, bidir_frame(548, false));
        let f = e.frames[0];
        assert!(f.checksum_ok && f.bidirectional && !f.telemetry);
        assert_eq!((f.throttle, f.rate_kbps), (548, 600));
        // The eRPM reply: 21 bits at 750 kbit/s, 30 us after the frame, starting with the line pulled low
        assert_eq!(e.replies, 1);
        let first = ctx.scheduled[0];
        assert_eq!((first.1, first.2), (6, External::Driven(Level::Low)));
        let frame_end = Time::from_ms(1) + Time::from_us(100) + Time(1_666_667 * 16);
        assert!((first.0 - frame_end).as_us() >= 29 && (first.0 - frame_end).as_us() <= 31);
        assert_eq!(ctx.scheduled.last().unwrap().2, External::Floating);
        // Half throttle: the motor spins up towards 2000 RPM (7 time constants)
        for _ in 0..1000 {
            ctx.scheduled.clear();
            ctx.now += Time::from_us(400);
            play(&mut e, &mut ctx, bidir_frame(548, false));
        }
        assert!((e.rpm - 2004.0).abs() < 50.0, "{}", e.rpm);

        assert!((e.period_us() as f64 - 60e6 / (e.rpm * 7.0)).abs() < 2.0);
        // Telemetry request: 10 bytes on GPIO 7
        ctx.scheduled.clear();
        ctx.now += Time::from_us(400);
        play(&mut e, &mut ctx, bidir_frame(548, true));
        assert_eq!(e.telemetry_sent, 1);
        assert_eq!(ctx.scheduled.iter().filter(|s| s.1 == 7).count(), 100, "10 bytes of 10 bits");
        let f = e.telemetry_frame();
        assert_eq!(f[0], 35);
        assert_eq!(u16::from_be_bytes([f[1], f[2]]), 1600);
        // Reverse in 3D mode, and a corrupted checksum
        ctx.now += Time::from_ms(2);
        play(&mut e, &mut ctx, bidir_frame(1548, false));
        assert!(e.target_rpm < 0.0);
        // Between frames the speed carries on towards the new target, one time constant (60 ms) at a time
        let (rpm, target) = (e.rpm, e.target_rpm);
        assert!(rpm > 1900.0);
        assert_eq!(e.rpm_at(ctx.now - Time::from_ms(5)), rpm);
        let later = e.rpm_at(e.last_update + Time::from_ms(60));
        assert!((later - (rpm + (target - rpm) * (1.0 - (-1f64).exp()))).abs() < 1e-6, "{later}");
        assert_eq!(e.rpm, rpm, "reading it changes nothing");
        ctx.now += Time::from_us(400);
        play(&mut e, &mut ctx, bidir_frame(100, false) ^ 1);
        assert_eq!(e.bad_frames, 1);
    }
}
