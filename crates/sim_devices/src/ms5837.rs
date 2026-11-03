//! TE MS5837-02BA and MS5837-30BA pressure sensors (datasheets MS5837-02BA01 rev A3 and MS5837-30BA rev B1).
//!
//! Commands: reset (0x1E), PROM read (0xA0 + 2n: calibration words C1-C6 and the CRC-4 in word 0), D1 pressure and
//! D2 temperature conversions at OSR 256-8192 (0x40-0x4A, 0x50-0x5A) and ADC read (0x00, 24-bit result). An ADC read
//! before a conversion has finished, or a second read, returns 0 as the datasheet says.
//!
//! The sensor is given the absolute pressure and temperature as [`Signal`]s; the model finds the raw D1/D2 values
//! that the datasheet's first and second order compensation turns back into them, using the datasheet's example
//! calibration coefficients.

use sim_core::analog::Signal;
use sim_core::time::Time;

use crate::DeviceCtx;
use crate::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x76;
/// Standard atmosphere, Pa.
pub const ATMOSPHERE_PA: f64 = 101_325.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Model {
    Ms5837_02ba,
    Ms5837_30ba,
}

/// Absolute pressure at `depth_m` below the surface of water of `density` kg/m^3.
pub fn pressure_at_depth(surface_pa: f64, depth_m: f64, density: f64) -> f64 {
    surface_pa + density * 9.80665 * depth_m
}

/// The datasheet's CRC-4 over the PROM (word 0's CRC nibble and the eighth word taken as 0).
pub fn crc4(prom: &[u16; 7]) -> u8 {
    let mut w = [0u16; 8];
    w[..7].copy_from_slice(prom);
    w[0] &= 0x0FFF;
    let mut rem: u16 = 0;
    for cnt in 0..16 {
        rem ^= if cnt % 2 == 1 { w[cnt >> 1] & 0xFF } else { w[cnt >> 1] >> 8 };
        for _ in 0..8 {
            rem = if rem & 0x8000 != 0 { (rem << 1) ^ 0x3000 } else { rem << 1 };
        }
    }
    (rem >> 12) as u8 & 0xF
}

/// Datasheet compensation: returns (pressure in Pa, temperature in 0.01 °C).
pub fn compensate(model: Model, c: &[u16; 7], d1: u32, d2: u32) -> (i64, i64) {
    let c = |i: usize| c[i] as i64;
    let (d1, d2) = (d1 as i64, d2 as i64);
    let dt = d2 - c(5) * 256;
    let temp = 2000 + dt * c(6) / (1 << 23);
    let t20 = temp - 2000;
    let (off, sens, ti, offi, sensi) = match model {
        Model::Ms5837_02ba => {
            let off = c(2) * (1 << 17) + c(4) * dt / (1 << 6);
            let sens = c(1) * (1 << 16) + c(3) * dt / (1 << 7);
            let (ti, offi, sensi) = if temp < 2000 {
                (11 * dt * dt / (1i64 << 35), 31 * t20 * t20 / 8, 63 * t20 * t20 / 32)
            } else {
                (0, 0, 0)
            };
            (off, sens, ti, offi, sensi)
        }
        Model::Ms5837_30ba => {
            let off = c(2) * (1 << 16) + c(4) * dt / (1 << 7);
            let sens = c(1) * (1 << 15) + c(3) * dt / (1 << 8);
            let (ti, mut offi, mut sensi) = if temp < 2000 {
                (3 * dt * dt / (1i64 << 33), 3 * t20 * t20 / 2, 5 * t20 * t20 / 8)
            } else {
                (2 * dt * dt / (1i64 << 37), t20 * t20 / 16, 0)
            };
            if temp < -1500 {
                let t = temp + 1500;
                offi += 7 * t * t;
                sensi += 4 * t * t;
            }
            (off, sens, ti, offi, sensi)
        }
    };
    let (off2, sens2) = (off - offi, sens - sensi);
    let temp2 = temp - ti;
    let p = match model {
        Model::Ms5837_02ba => (d1 * sens2 / (1 << 21) - off2) / (1 << 15),
        Model::Ms5837_30ba => (d1 * sens2 / (1 << 21) - off2) / (1 << 13) * 10,
    };
    (p, temp2)
}

/// Smallest 24-bit value for which `f` is at least `target` (`f` increasing).
fn search(target: i64, f: impl Fn(u32) -> i64) -> u32 {
    let (mut lo, mut hi) = (0u32, (1 << 24) - 1);
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if f(mid) < target { lo = mid + 1 } else { hi = mid }
    }
    lo
}

#[derive(Clone, Debug)]
pub struct Ms5837 {
    pub model: Model,
    pub pressure_pa: Signal,
    pub temperature_c: Signal,
    pub prom: [u16; 7],
    /// Conversion in progress or finished: (is pressure, done at); the result is computed at read time.
    conversion: Option<(bool, Time)>,
    command: Option<u8>,
    out: Vec<u8>,
    read_pos: usize,
    pub resets: u32,
    pub conversions: u32,
}

impl Ms5837 {
    pub fn new(model: Model, pressure_pa: impl Into<Signal>, temperature_c: impl Into<Signal>) -> Self {
        let mut prom = match model {
            Model::Ms5837_02ba => [0x0000, 46372, 43981, 29059, 27842, 31553, 28165],
            Model::Ms5837_30ba => [0x0000, 34982, 36352, 20328, 22354, 26646, 26146],
        };
        // Word 0: CRC in bits 15:12, factory defined bits below (the 30BA's product version is in bits 11:5)
        prom[0] = 0x0040;
        prom[0] |= (crc4(&prom) as u16) << 12;
        Ms5837 {
            model,
            pressure_pa: pressure_pa.into(),
            temperature_c: temperature_c.into(),
            prom,
            conversion: None,
            command: None,
            out: Vec::new(),
            read_pos: 0,
            resets: 0,
            conversions: 0,
        }
    }

    /// Conversion time (datasheet maximum) for an OSR index 0-5 (256-8192).
    pub fn conversion_time(osr: u8) -> Time {
        Time::from_us([600, 1170, 2280, 4540, 9040, 18080][osr.min(5) as usize])
    }

    /// Raw (D1, D2) for the given pressure and temperature.
    pub fn raw_for(&self, pressure_pa: f64, temperature_c: f64) -> (u32, u32) {
        let t = (temperature_c * 100.0).round() as i64;
        let d2 = search(t, |d2| compensate(self.model, &self.prom, 0, d2).1);
        let p = pressure_pa.round() as i64;
        let d1 = search(p, |d1| compensate(self.model, &self.prom, d1, d2).0);
        (d1, d2)
    }

    fn command(&mut self, cmd: u8, now: Time) {
        self.out.clear();
        self.read_pos = 0;
        match cmd {
            0x1E => {
                self.resets += 1;
                self.conversion = None;
            }
            0x40..=0x4A | 0x50..=0x5A if cmd & 1 == 0 => {
                self.conversions += 1;
                let osr = (cmd & 0x0F) >> 1;
                self.conversion = Some((cmd < 0x50, now + Self::conversion_time(osr)));
            }
            0x00 => {
                let value = match self.conversion.take() {
                    Some((pressure, done)) if done <= now => {
                        let (d1, d2) = self.raw_for(self.pressure_pa.at(done), self.temperature_c.at(done));
                        if pressure { d1 } else { d2 }
                    }
                    Some(_) => {
                        log::debug!("MS5837: ADC read before the conversion finished");
                        0
                    }
                    None => 0,
                };
                self.out = value.to_be_bytes()[1..].to_vec();
            }
            0xA0..=0xAE if cmd & 1 == 0 => {
                let i = ((cmd - 0xA0) >> 1) as usize;
                self.out = self.prom.get(i).copied().unwrap_or(0).to_be_bytes().to_vec();
            }
            _ => log::debug!("MS5837: unknown command {cmd:#04x}"),
        }
    }
}

impl I2cDevice for Ms5837 {
    fn name(&self) -> String {
        match self.model {
            Model::Ms5837_02ba => "MS5837-02BA".into(),
            Model::Ms5837_30ba => "MS5837-30BA".into(),
        }
    }

    fn responds_to(&self, address: u8) -> bool {
        address == ADDRESS
    }

    fn start(&mut self, _ctx: &mut dyn DeviceCtx, _address: u8, read: bool) -> bool {
        if read {
            self.read_pos = 0;
        } else {
            self.command = None;
        }
        true
    }

    fn write(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        if self.command.is_none() {
            self.command = Some(byte);
            self.command(byte, ctx.now());
        }
        true
    }

    fn read(&mut self, _ctx: &mut dyn DeviceCtx, _ack: bool) -> u8 {
        let b = self.out.get(self.read_pos).copied().unwrap_or(0);
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

    fn cmd(s: &mut Ms5837, ctx: &mut TestCtx, c: u8, n: usize) -> Vec<u8> {
        s.start(ctx, ADDRESS, false);
        s.write(ctx, c);
        s.start(ctx, ADDRESS, true);
        (0..n).map(|_| s.read(ctx, true)).collect()
    }

    #[test]
    fn datasheet_examples() {
        // 02BA datasheet example: D1 = 6465444, D2 = 8077636 -> 20.00 °C... and 1100.02 mbar
        let s = Ms5837::new(Model::Ms5837_02ba, 0.0, 0.0);
        let (p, t) = compensate(Model::Ms5837_02ba, &s.prom, 6_465_444, 8_077_636);
        assert_eq!((p, t), (110_002, 2000));
        // 30BA example: D1 = 4958179, D2 = 6815414 -> 19.81 °C, 3999.8 mbar (first order; second order shifts it)
        let s = Ms5837::new(Model::Ms5837_30ba, 0.0, 0.0);
        let (p, t) = compensate(Model::Ms5837_30ba, &s.prom, 4_958_179, 6_815_414);
        assert!((t - 1981).abs() <= 2, "{t}");
        assert!((p - 399_980).abs() < 300, "{p}");
    }

    #[test]
    fn prom_crc_and_inverse() {
        let s = Ms5837::new(Model::Ms5837_02ba, 0.0, 0.0);
        assert_eq!((s.prom[0] >> 12) as u8, crc4(&s.prom));
        for (p, t) in [(101_325.0, 20.0), (120_000.0, 5.0), (90_000.0, 35.0)] {
            let (d1, d2) = s.raw_for(p, t);
            let (pc, tc) = compensate(s.model, &s.prom, d1, d2);
            assert!((pc as f64 - p).abs() <= 1.0 && (tc as f64 - t * 100.0).abs() <= 1.0, "{p} {t}: {pc} {tc}");
        }
        let s = Ms5837::new(Model::Ms5837_30ba, 0.0, 0.0);
        let (d1, d2) = s.raw_for(pressure_at_depth(ATMOSPHERE_PA, 10.0, 997.0), 12.0);
        let (pc, _) = compensate(s.model, &s.prom, d1, d2);
        assert!((pc as f64 - 199_097.0).abs() < 20.0, "{pc}");
    }

    #[test]
    fn conversion_sequence() {
        let mut ctx = TestCtx::default();
        let mut s = Ms5837::new(Model::Ms5837_02ba, 101_325.0, 21.5);
        cmd(&mut s, &mut ctx, 0x1E, 0);
        let w1 = cmd(&mut s, &mut ctx, 0xA2, 2);
        assert_eq!(u16::from_be_bytes([w1[0], w1[1]]), 46372);
        cmd(&mut s, &mut ctx, 0x4A, 0);
        ctx.now = Time::from_ms(10);
        assert_eq!(cmd(&mut s, &mut ctx, 0x00, 3), [0, 0, 0], "read too early");
        cmd(&mut s, &mut ctx, 0x4A, 0);
        ctx.now = Time::from_ms(30);
        let d1 = cmd(&mut s, &mut ctx, 0x00, 3);
        assert_eq!(cmd(&mut s, &mut ctx, 0x00, 3), [0, 0, 0], "second read");
        cmd(&mut s, &mut ctx, 0x5A, 0);
        ctx.now = Time::from_ms(50);
        let d2 = cmd(&mut s, &mut ctx, 0x00, 3);
        let d1 = u32::from_be_bytes([0, d1[0], d1[1], d1[2]]);
        let d2 = u32::from_be_bytes([0, d2[0], d2[1], d2[2]]);
        assert_eq!(compensate(s.model, &s.prom, d1, d2), (101_325, 2150));
    }
}
