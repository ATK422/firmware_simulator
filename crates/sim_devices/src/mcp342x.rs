//! Microchip MCP3426/MCP3427/MCP3428 delta-sigma ADC (datasheet DS22226A).
//!
//! The configuration byte selects channel (C1:C0), continuous or one-shot (O/C), resolution (12/14/16 bits at
//! 240/60/15 SPS) and PGA gain (1/2/4/8); in one-shot mode writing RDY = 1 starts a conversion. A read returns the
//! output code (two bytes, MSB first, sign extended) and then the configuration byte, repeated, whose RDY bit is 0
//! only for a result not read before. Channel inputs are differential voltages given as [`Signal`]s, sampled when
//! each conversion ends. On the 2-channel MCP3426/7, selections 3 and 4 convert channels 1 and 2.
//!
//! The general call reset/latch/conversion commands are not modelled (the controller does not send general calls).

use sim_core::analog::Signal;
use sim_core::time::Time;

use crate::DeviceCtx;
use crate::i2c::I2cDevice;

pub const DEFAULT_ADDRESS: u8 = 0x68;

#[derive(Clone, Debug)]
pub struct Mcp342x {
    pub address: u8,
    /// Differential input voltage of each channel (2 for the MCP3426/7, 4 for the MCP3428).
    pub inputs: Vec<Signal>,
    config: u8,
    /// Start of the conversion sequence (continuous) or of the one-shot conversion.
    started: Option<Time>,
    /// Index of the last conversion read (continuous mode), or whether the one-shot result was read.
    read_upto: u64,
    read_pos: usize,
    out: [u8; 3],
    pub config_writes: u32,
}

impl Mcp342x {
    /// An MCP3426 (2 channels).
    pub fn mcp3426(ch1: impl Into<Signal>, ch2: impl Into<Signal>) -> Self {
        Self::with_inputs(vec![ch1.into(), ch2.into()])
    }

    pub fn with_inputs(inputs: Vec<Signal>) -> Self {
        Mcp342x {
            address: DEFAULT_ADDRESS,
            inputs,
            // Power-on default: continuous, channel 1, 12 bits, gain 1, RDY set
            config: 0x90,
            started: Some(Time::ZERO),
            read_upto: 0,
            read_pos: 0,
            out: [0; 3],
            config_writes: 0,
        }
    }

    fn continuous(&self) -> bool {
        self.config & 0x10 != 0
    }

    fn bits(&self) -> u32 {
        match (self.config >> 2) & 3 {
            0 => 12,
            1 => 14,
            _ => 16,
        }
    }

    /// Conversion period (1 / data rate).
    pub fn conversion_time(&self) -> Time {
        match self.bits() {
            12 => Time::from_us(1_000_000 / 240),
            14 => Time::from_us(1_000_000 / 60),
            _ => Time::from_us(1_000_000 / 15),
        }
    }

    fn gain(&self) -> f64 {
        (1 << (self.config & 3)) as f64
    }

    fn channel(&self) -> usize {
        let c = ((self.config >> 5) & 3) as usize;
        c % self.inputs.len().max(1)
    }

    /// Output code for `volts` at the current settings: `code = V * gain * 2^(N-1) / 2.048`, saturating.
    pub fn code_for(&self, volts: f64) -> i32 {
        let n = self.bits();
        let max = (1i64 << (n - 1)) - 1;
        let code = (volts * self.gain() * (1i64 << (n - 1)) as f64 / 2.048).round() as i64;
        code.clamp(-max - 1, max) as i32
    }

    /// Number of completed conversions at `now`, and when the latest ended.
    fn completed(&self, now: Time) -> (u64, Time) {
        let Some(s) = self.started else { return (0, Time::ZERO) };
        let p = self.conversion_time();
        if now < s + p {
            return (0, Time::ZERO);
        }
        if self.continuous() {
            let n = (now - s).as_ps() / p.as_ps();
            (n, s + Time(n * p.as_ps()))
        } else {
            (1, s + p)
        }
    }

    fn prepare_read(&mut self, now: Time) {
        let (n, at) = self.completed(now);
        let fresh = n > self.read_upto;
        let code = if n == 0 {
            0
        } else {
            let v = self.inputs.get(self.channel()).map_or(0.0, |s| s.at(at));
            self.code_for(v)
        };
        if fresh {
            self.read_upto = n;
        }
        let rdy = if fresh { 0 } else { 0x80 };
        let cfg = (self.config & 0x7F) | rdy;
        // 12/14/16-bit codes are right justified and sign extended to 16 bits (18-bit mode is MCP3421-3 only)
        let w = (code as i16).to_be_bytes();
        self.out = [w[0], w[1], cfg];
        self.read_pos = 0;
    }
}

impl I2cDevice for Mcp342x {
    fn name(&self) -> String {
        if self.inputs.len() > 2 { "MCP3428".into() } else { "MCP3426".into() }
    }

    fn responds_to(&self, address: u8) -> bool {
        address == self.address
    }

    fn start(&mut self, ctx: &mut dyn DeviceCtx, _address: u8, read: bool) -> bool {
        if read {
            self.prepare_read(ctx.now());
        }
        true
    }

    fn write(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        self.config_writes += 1;
        let now = ctx.now();
        self.config = byte & 0x7F | (self.config & 0x80);
        if self.continuous() {
            // Any configuration write restarts conversions with the new settings
            self.started = Some(now);
        } else if byte & 0x80 != 0 {
            self.started = Some(now);
        } else {
            self.started = None;
        }
        self.read_upto = 0;
        true
    }

    fn read(&mut self, _ctx: &mut dyn DeviceCtx, _ack: bool) -> u8 {
        // After the code, the configuration byte repeats
        let b = self.out[self.read_pos.min(2)];
        self.read_pos += 1;
        b
    }

    fn clone_box(&self) -> Box<dyn I2cDevice> {
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

    fn read3(m: &mut Mcp342x, ctx: &mut TestCtx) -> (i16, u8) {
        assert!(m.start(ctx, 0x68, true));
        let b: Vec<u8> = (0..4).map(|_| m.read(ctx, true)).collect();
        assert_eq!(b[2], b[3], "config byte repeats");
        (i16::from_be_bytes([b[0], b[1]]), b[2])
    }

    #[test]
    fn continuous_16_bit_conversions_and_ready_bit() {
        let mut ctx = TestCtx::default();
        let mut m = Mcp342x::mcp3426(1.0, -0.5);
        m.start(&mut ctx, 0x68, false);
        // Channel 1, continuous, 16 bits, gain 1
        m.write(&mut ctx, 0x18);
        ctx.now = Time::from_ms(50);
        let (code, cfg) = read3(&mut m, &mut ctx);
        assert_eq!((code, cfg & 0x80), (0, 0x80), "first conversion (66.7 ms) not done");
        ctx.now = Time::from_ms(70);
        let (code, cfg) = read3(&mut m, &mut ctx);
        assert_eq!(code, 16000, "1 V at 62.5 uV/LSB");
        assert_eq!(cfg, 0x18, "RDY clear: new result");
        let (_, cfg) = read3(&mut m, &mut ctx);
        assert_eq!(cfg & 0x80, 0x80, "already read");
        // Channel 2, gain 2
        m.write(&mut ctx, 0x39);
        ctx.now += Time::from_ms(70);
        assert_eq!(read3(&mut m, &mut ctx).0, -16000);
    }

    #[test]
    fn resolutions_saturation_and_channel_aliasing() {
        let mut ctx = TestCtx::default();
        let mut m = Mcp342x::mcp3426(3.0, 0.1);
        m.write(&mut ctx, 0x10); // 12 bits
        ctx.now = Time::from_ms(5);
        assert_eq!(read3(&mut m, &mut ctx).0, 2047, "saturates at +2.048 V");
        m.write(&mut ctx, 0x74); // channel 4 -> 2 on the MCP3426, 14 bits
        ctx.now += Time::from_ms(17);
        assert_eq!(read3(&mut m, &mut ctx).0, 400, "0.1 V at 250 uV/LSB");
        // One-shot: RDY = 1 starts a single conversion
        m.write(&mut ctx, 0x80);
        let t0 = ctx.now;
        ctx.now = t0 + Time::from_ms(5);
        let (code, cfg) = read3(&mut m, &mut ctx);
        assert_eq!((code, cfg & 0x90), (2047, 0));
        ctx.now += Time::from_ms(100);
        assert_eq!(read3(&mut m, &mut ctx).1 & 0x80, 0x80, "no further conversions");
    }

    #[test]
    fn c_driver_init_byte_is_a_config_write() {
        let mut ctx = TestCtx::default();
        let mut m = Mcp342x::mcp3426(1.0, 1.0);
        m.write(&mut ctx, 0x06);
        ctx.now = Time::from_ms(500);
        let (_, cfg) = read3(&mut m, &mut ctx);
        assert_eq!(cfg, 0x86, "one-shot, 14 bits, gain 4, never started");
    }
}
