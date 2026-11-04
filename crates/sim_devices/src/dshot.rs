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
//! mode, as the UWRT ESCs are patched), 0 and the special commands 1-47 stop it. How fast it turns for a throttle
//! in the steady state is the ESC's [`SpeedCurve`]: by default proportional to the throttle (`max_rpm` at full
//! throttle, either way), or a fit of bench data per direction. The UWRT ESC boards' APD 80F3 ESCs with T200
//! thrusters use the fit their own firmware's feed-forward was made from ([`SpeedCurve::APD_T200`]).
//!
//! When the frames stop, the ESC stops the motor [`Esc::signal_loss_timeout`] after the last valid one (it coasts
//! down with the same lag), as the ESC firmwares do: BLHeli_S and Bluejay cut the motor after about 250 ms without
//! a signal, AM32 after about half a second when armed. The 250 ms default is an assumption from those (the UWRT
//! ESCs' exact firmware and settings are not known here). The real firmwares also disarm and want a zero throttle
//! before they spin again; this model has no arming sequence and follows the next valid frame.

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
    /// Steady-state speed for a throttle; motor pole pairs, time constant of the motor's response.
    pub curve: SpeedCurve,
    pub pole_pairs: u32,
    pub tau: Time,
    pub reply_delay: Time,
    /// How long after the last valid frame the motor is stopped (`None`: it keeps the last throttle forever).
    pub signal_loss_timeout: Option<Time>,
    /// Times the signal was lost with the motor commanded to turn.
    pub signal_losses: u32,
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
    /// When the last valid frame arrived, and when a wake-up to check for signal loss is due.
    last_frame: Option<Time>,
    loss_check: Option<Time>,
    consumption_mah: f64,
    // Decoder
    idle_high: Option<bool>,
    level: bool,
    edge: Option<Time>,
    bit_start: Option<Time>,
    bits: Vec<(Time, Time)>,
    busy_until: Time,
}

/// How fast an ESC turns its motor for a DShot throttle, once the motor has settled (signed mechanical RPM).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpeedCurve {
    /// Proportional to the throttle: `max_rpm` at full throttle (1047 ahead: 1000 steps above 47; 2047 astern: 999
    /// steps above 1048).
    Linear { max_rpm: f64 },
    /// Measured, per direction: at the steady state the throttle step `u` (the DShot value minus 48 ahead, minus
    /// 1048 astern: `esc_core`'s throttle units) is `c0 + c1 rpm + c2 rpm^2` for the speed's magnitude. The motor
    /// stands still below `c0`.
    Fit { forward: [f64; 3], reverse: [f64; 3] },
}

impl SpeedCurve {
    /// titan_firmware's ESC board feed-forward (`Thruster_Controller/ThrusterController.c`, ported as
    /// `esc_core::controller::feed_forward`): a quadratic fit of DShot throttle against RPM that `generateFF.m`
    /// made from riptide's `calibrate_thruster.py` bench runs (throttle steps of +-20 to +-725, the RPM the ESC
    /// reported, the force on a gauge) with the boards' APD 80F3 ESCs and T200 thrusters; the same runs gave
    /// riptide's `force_to_rpm_coefficents`. Measured up to the boards' 725 hard limit (3068 RPM ahead, 3100
    /// astern), extrapolated beyond (3770 / 3804 RPM at full throttle).
    pub const APD_T200: SpeedCurve =
        SpeedCurve::Fit { forward: [12.369874, 0.103410, 0.000042], reverse: [14.791378, 0.098938, 0.000042] };

    /// Signed steady-state RPM for an 11-bit DShot throttle value.
    pub fn rpm(&self, v: u16) -> f64 {
        match *self {
            SpeedCurve::Linear { max_rpm } => match v {
                48..=1047 => (v - 47) as f64 / 1000.0 * max_rpm,
                1049..=2047 => -((v - 1048) as f64 / 999.0 * max_rpm),
                _ => 0.0,
            },
            SpeedCurve::Fit { forward, reverse } => match v {
                48..=1047 => solve(forward, (v - 48) as f64),
                1049..=2047 => -solve(reverse, (v - 1048) as f64),
                _ => 0.0,
            },
        }
    }
}

/// The speed at which `c0 + c1 rpm + c2 rpm^2` reaches throttle `u` (0 below `c0`).
fn solve([c0, c1, c2]: [f64; 3], u: f64) -> f64 {
    let u = u - c0;
    if u <= 0.0 {
        0.0
    } else if c2 == 0.0 {
        u / c1
    } else {
        2.0 * u / (c1 + (c1 * c1 + 4.0 * c2 * u).sqrt())
    }
}

/// The lag of a motor with time constant `tau` towards `target` over `dt` seconds from `rpm`.
fn lag(tau: Time, rpm: f64, target: f64, dt: f64) -> f64 {
    rpm + (target - rpm) * (1.0 - (-dt / tau.as_secs_f64()).exp())
}

/// An ESC's motor state at one moment ([`Esc::motor`]): enough to tell its speed at any later time, as long as no
/// frame comes in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor {
    rpm: f64,
    target_rpm: f64,
    last_update: Time,
    /// When the motor stops for lack of a signal, if it is turning.
    loss_at: Option<Time>,
    tau: Time,
}

impl Motor {
    /// Signed mechanical RPM at `now` ([`Esc::rpm_at`]).
    pub fn rpm_at(&self, now: Time) -> f64 {
        if now <= self.last_update {
            return self.rpm;
        }
        match self.loss_at {
            Some(lost) if lost < now => {
                let at_loss = lag(
                    self.tau,
                    self.rpm,
                    self.target_rpm,
                    (lost.max(self.last_update) - self.last_update).as_secs_f64(),
                );
                lag(self.tau, at_loss, 0.0, (now - lost.max(self.last_update)).as_secs_f64())
            }
            _ => lag(self.tau, self.rpm, self.target_rpm, (now - self.last_update).as_secs_f64()),
        }
    }
}

impl Esc {
    pub fn new(name: &str, signal_pin: u8, telemetry_pin: Option<u8>) -> Self {
        Esc {
            name: name.into(),
            signal_pin,
            telemetry_pin,
            curve: SpeedCurve::Linear { max_rpm: 4000.0 },
            pole_pairs: 7,
            tau: Time::from_ms(60),
            reply_delay: Time::from_us(30),
            signal_loss_timeout: Some(Self::DEFAULT_SIGNAL_LOSS_TIMEOUT),
            signal_losses: 0,
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
            last_frame: None,
            loss_check: None,
            consumption_mah: 0.0,
            idle_high: None,
            level: false,
            edge: None,
            bit_start: None,
            bits: Vec::new(),
            busy_until: Time::ZERO,
        }
    }

    /// Assumed signal-loss timeout (see the module documentation).
    pub const DEFAULT_SIGNAL_LOSS_TIMEOUT: Time = Time::from_ms(250);

    /// When the signal counts as lost: the timeout after the last valid frame, if the motor is commanded to turn.
    fn loss_at(&self) -> Option<Time> {
        match (self.signal_loss_timeout, self.last_frame) {
            (Some(timeout), Some(last)) if self.target_rpm != 0.0 => Some(last + timeout),
            _ => None,
        }
    }

    /// The lag towards `target` over `dt` seconds from `rpm`.
    fn lag(&self, rpm: f64, target: f64, dt: f64) -> f64 {
        lag(self.tau, rpm, target, dt)
    }

    /// Brings the motor up to `now`, stopping it at the signal-loss time on the way.
    fn integrate(&mut self, now: Time) {
        if let Some(lost) = self.loss_at()
            && lost <= now
        {
            self.integrate_to(lost);
            self.target_rpm = 0.0;
            self.signal_losses += 1;
        }
        self.integrate_to(now);
    }

    fn integrate_to(&mut self, now: Time) {
        if now <= self.last_update {
            return;
        }
        let dt = (now - self.last_update).as_secs_f64();
        self.rpm = self.lag(self.rpm, self.target_rpm, dt);
        self.consumption_mah += self.current_a() * dt / 3.6;
        self.last_update = now;
    }

    /// The motor's signed mechanical RPM at `now`: the lag towards the last commanded speed carried on from the
    /// last frame (`rpm` is only brought up to date when a frame arrives), and towards a stop once the signal has
    /// been lost for the timeout. For a vehicle simulation reading the propeller speed between frames.
    pub fn rpm_at(&self, now: Time) -> f64 {
        self.motor().rpm_at(now)
    }

    /// The motor's state as it is now: what [`Esc::rpm_at`] works from, kept apart from the rest of the model (a
    /// robot running its boards ahead of each other keeps it for each slice, for its vehicles).
    pub fn motor(&self) -> Motor {
        Motor {
            rpm: self.rpm,
            target_rpm: self.target_rpm,
            last_update: self.last_update,
            loss_at: self.loss_at(),
            tau: self.tau,
        }
    }

    /// Whether the motor has been stopped for lack of a signal (and no valid frame came since).
    pub fn signal_lost(&self, now: Time) -> bool {
        match (self.signal_loss_timeout, self.last_frame) {
            (Some(timeout), Some(last)) => now >= last + timeout,
            _ => false,
        }
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

    /// What a valid frame carrying `throttle` does to the motor when it arrives at `now`: the motor is brought up
    /// to `now` (stopping on the way if the signal had been lost) and follows the new throttle from there. Frames
    /// decoded from the pin come here; so does whoever drives the motor model without a pin (calibration against
    /// test-stand data).
    pub fn command(&mut self, now: Time, throttle: u16) {
        self.integrate(now);
        self.apply_throttle(throttle);
        self.last_frame = Some(now);
    }

    fn apply_throttle(&mut self, v: u16) {
        self.target_rpm = self.curve.rpm(v);
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
        self.command(now, frame.throttle);
        // One wake-up per timeout at most, not one per frame: when it comes, it looks at the newest frame
        if let Some(timeout) = self.signal_loss_timeout
            && self.loss_check.is_none()
        {
            self.loss_check = Some(now + timeout);
            ctx.wake_at(now + timeout);
        }
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
        let now = ctx.now();
        if self.loss_check.is_some_and(|t| now >= t) {
            self.loss_check = None;
            match self.loss_at() {
                Some(lost) if lost <= now => {
                    self.integrate(now);
                    let ms = self.signal_loss_timeout.unwrap_or(Time::ZERO).as_secs_f64() * 1e3;
                    ctx.report(format!("{}: no DShot frame for {ms:.0} ms, motor stopped", self.name));
                }
                // Frames came since: look again when the newest one times out
                Some(lost) => {
                    self.loss_check = Some(lost);
                    ctx.wake_at(lost);
                }
                None => {}
            }
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

    /// `esc_core::controller::feed_forward` written out again (its integer arithmetic): the throttle the ESC board
    /// firmware puts out for a target RPM before its P and I terms.
    fn feed_forward(rpm: i64) -> i64 {
        if rpm.abs() < 50 {
            return 0;
        }
        let v = if rpm > 0 {
            12_369_874 + 103_410 * rpm + 42 * rpm * rpm
        } else {
            -14_791_378 + 98_938 * rpm - 42 * rpm * rpm
        };
        v / 1_000_000
    }

    /// The DShot value `esc_core::dshot::throttle_command` sends for a throttle.
    fn dshot_value(throttle: i64) -> u16 {
        match throttle {
            1..=999 => throttle as u16 + 48,
            -999..=-1 => (-throttle) as u16 + 1048,
            _ => 0,
        }
    }

    #[test]
    fn the_apd_t200_curve_turns_the_feed_forward_throttle_into_its_target() {
        let c = SpeedCurve::APD_T200;
        // Each way, the feed-forward throttle for a target spins the motor to that target, to within the RPM one
        // throttle step is worth (the firmware truncates the polynomial); its P term (0.1 throttle per RPM) and I
        // term take the rest out
        for rpm in (100..=3700).step_by(50) {
            for sign in [1i64, -1] {
                let target = sign * rpm;
                let got = c.rpm(dshot_value(feed_forward(target)));
                let step = 1.0 / (0.1034 + 2.0 * 0.000042 * rpm as f64);
                assert!(got.signum() == sign as f64, "{target}: {got}");
                assert!(
                    (got.abs() - rpm as f64).abs() <= step + 0.5 && got.abs() <= rpm as f64 + 0.5,
                    "{target}: {got}"
                );
            }
        }
        // The boards' hard limit (725) and full throttle
        assert!((c.rpm(725 + 48) - 3068.0).abs() < 1.0, "{}", c.rpm(773));
        assert!((c.rpm(725 + 1048) + 3100.0).abs() < 1.0, "{}", c.rpm(1773));
        assert!((c.rpm(1047) - 3770.0).abs() < 1.0 && (c.rpm(2047) + 3804.0).abs() < 1.0);
        // Below the fit's offset (about 12 steps ahead, 15 astern) the motor stands; neutral and commands stop it
        for v in [0, 1, 47, 48, 48 + 12, 1048, 1048 + 14] {
            assert_eq!(c.rpm(v), 0.0, "{v}");
        }
        assert!(c.rpm(48 + 13) > 0.0 && c.rpm(1048 + 15) < 0.0);
        // Rising all the way, each way
        for v in 49..=1047u16 {
            assert!(c.rpm(v) >= c.rpm(v - 1) && c.rpm(v + 1000) <= c.rpm(v + 999), "{v}");
        }
        // A straight-line fit, and the default linear curve as before
        let lin = SpeedCurve::Fit { forward: [10.0, 0.25, 0.0], reverse: [0.0, 0.5, 0.0] };
        assert_eq!((lin.rpm(48 + 110), lin.rpm(1048 + 100)), (400.0, -200.0));
        let def = SpeedCurve::Linear { max_rpm: 4000.0 };
        assert_eq!((def.rpm(547), def.rpm(1047), def.rpm(2047)), (2000.0, 4000.0, -4000.0));
    }

    /// The motor of an ESC with the measured curve settles where the curve says, the lag as before.
    #[test]
    fn a_fitted_esc_settles_on_its_curve() {
        let mut ctx = TestCtx { now: Time::from_ms(1), ..Default::default() };
        let mut e = Esc::new("esc", 6, None);
        e.curve = SpeedCurve::APD_T200;
        e.pin_changed(&mut ctx, 6, true);
        for (v, want) in [(48 + 400, 2046.9), (1048 + 400, -2071.6)] {
            assert!((e.curve.rpm(v) - want).abs() < 0.1, "{v}: {}", e.curve.rpm(v));
            for _ in 0..1250 {
                ctx.now += Time::from_us(400);
                play(&mut e, &mut ctx, bidir_frame(v, false));
            }
            // 0.5 s is eight time constants
            assert!((e.rpm - want).abs() < 2.0, "{v}: {}", e.rpm);
        }
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

#[cfg(test)]
mod signal_loss {
    use super::*;
    use crate::TestCtx;

    /// A bidirectional DShot600 frame at `ctx.now` (as `tests::play`), then 400 µs on.
    fn frame(e: &mut Esc, ctx: &mut TestCtx, throttle: u16) {
        let cmd = throttle << 1;
        let value = cmd << 4 | checksum(cmd, true);
        let bit = 1_666_667u64;
        for i in (0..16).rev() {
            let low = if value >> i & 1 != 0 { bit * 3 / 4 } else { bit * 3 / 8 };
            e.pin_changed(ctx, 6, false);
            ctx.now += Time(low);
            e.pin_changed(ctx, 6, true);
            ctx.now += Time(bit - low);
        }
        ctx.now += Time::from_us(5);
        e.update(ctx);
        ctx.now += Time::from_us(400) - Time(bit * 16) - Time::from_us(5);
    }

    fn spun_up() -> (Esc, TestCtx) {
        let mut ctx = TestCtx { now: Time::from_ms(1), ..Default::default() };
        let mut e = Esc::new("esc", 6, None);
        e.pin_changed(&mut ctx, 6, true);
        ctx.now += Time::from_us(100);
        // Half throttle for 0.5 s: 2000 RPM
        for _ in 0..1250 {
            frame(&mut e, &mut ctx, 548);
        }
        assert!((e.rpm - 2004.0).abs() < 5.0, "{}", e.rpm);
        (e, ctx)
    }

    #[test]
    fn the_motor_stops_when_the_frames_stop() {
        let (mut e, mut ctx) = spun_up();
        assert_eq!(e.signal_loss_timeout, Some(Time::from_ms(250)));
        let last = e.last_frame.unwrap();
        // One wake-up per timeout while the frames come (on top of the decoder's one per frame), not one per frame
        let checks = ctx.wakes.len() - e.frames.len();
        assert!((1..=3).contains(&checks), "{checks}");
        // Up to the timeout it holds its speed; after it, it coasts down with the motor's lag
        assert!((e.rpm_at(last + Time::from_ms(249)) - 2004.0).abs() < 5.0);
        let lost = last + Time::from_ms(250);
        let one_tau = e.rpm_at(lost + e.tau);
        assert!((one_tau - e.rpm_at(lost) * (-1f64).exp()).abs() < 1.0, "{one_tau}");
        assert!(e.rpm_at(lost + Time::from_secs(1)).abs() < 1.0);
        assert!(!e.signal_lost(lost - Time::from_ms(1)) && e.signal_lost(lost));
        // The wake-up it asked for finds the newest frame timed out: the motor state follows, and it says so
        while let Some(at) = e.loss_check {
            assert!(at <= lost);
            ctx.now = at;
            e.update(&mut ctx);
        }
        assert_eq!(ctx.now, lost);
        assert_eq!(e.signal_losses, 1);
        assert!(ctx.reports.iter().any(|r| r.contains("no DShot frame for 250 ms")), "{:?}", ctx.reports);
        assert_eq!(e.target_rpm, 0.0);
        assert!((e.rpm - e.rpm_at(lost)).abs() < 1e-9);
        // Frames again: it follows them (no arming sequence is modelled)
        ctx.now = lost + Time::from_ms(500);
        frame(&mut e, &mut ctx, 548);
        assert!(e.target_rpm > 1900.0 && !e.signal_lost(ctx.now));
    }

    /// A motor state taken at one moment goes on telling the speeds the ESC would have, signal loss included, until
    /// a frame comes; the ESC itself moves on from then.
    #[test]
    fn a_motor_state_tells_the_speeds_until_the_next_frame() {
        let (mut e, mut ctx) = spun_up();
        let kept = e.motor();
        let last = e.last_frame.unwrap();
        for ms in [0, 1, 7, 100, 249, 250, 251, 260, 400, 2000] {
            let t = last + Time::from_ms(ms);
            assert_eq!(kept.rpm_at(t).to_bits(), e.rpm_at(t).to_bits(), "{ms} ms");
        }
        ctx.now = last + Time::from_ms(5);
        frame(&mut e, &mut ctx, 100);
        let later = last + Time::from_ms(80);
        assert!(e.rpm_at(later) < kept.rpm_at(later) - 100.0, "the new frame slows the motor down");
        assert_eq!(e.motor(), e.motor());
    }

    #[test]
    fn commands_without_a_pin_drive_the_motor_like_frames() {
        // A frame through the pin is a command at the time it was decoded
        let (a, _) = spun_up();
        assert_eq!(a.target_rpm, 2000.0);
        // Half throttle forward, then the lag: 1 - 1/e of the way after one time constant
        let mut c = Esc::new("c", 0, None);
        c.signal_loss_timeout = None;
        c.command(Time::ZERO, 547);
        assert!((c.rpm_at(c.tau) - 2000.0 * (1.0 - (-1.0f64).exp())).abs() < 1e-6);
        assert!((c.rpm_at(Time::from_secs(10)) - 2000.0).abs() < 1e-6);
        c.command(Time::from_secs(10), 1048 + 999);
        assert!((c.rpm_at(Time::from_secs(20)) + 4000.0).abs() < 1e-6);
    }

    #[test]
    fn a_stopped_motor_has_nothing_to_lose_and_the_timeout_can_be_off() {
        // Throttle 0 last: nothing happens at the timeout
        let (mut e, mut ctx) = spun_up();
        frame(&mut e, &mut ctx, 0);
        ctx.now += Time::from_secs(1);
        e.update(&mut ctx);
        assert_eq!(e.signal_losses, 0);
        // Without a timeout, the old behaviour: the last throttle holds forever
        let (mut e, mut ctx) = spun_up();
        e.signal_loss_timeout = None;
        let last = ctx.now;
        assert!((e.rpm_at(last + Time::from_secs(10)) - 2004.0).abs() < 5.0);
        ctx.now += Time::from_secs(10);
        e.update(&mut ctx);
        frame(&mut e, &mut ctx, 548);
        assert_eq!(e.signal_losses, 0);
        assert!((e.rpm - 2004.0).abs() < 5.0);
    }
}
