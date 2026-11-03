//! Sensirion SHT4x (SHT40/41/45) temperature and humidity sensor (datasheet v6.6).
//!
//! A one-byte command starts a measurement; while it runs the sensor NAKs its read header. The result is two 16-bit
//! words (temperature, humidity), each followed by a CRC-8 (polynomial 0x31, init 0xFF). The physical temperature
//! and humidity are [`Signal`]s sampled when the measurement ends. Heater commands heat for their pulse and then
//! measure, as specified; the heater's effect on the reading is not modelled.

use sim_core::analog::Signal;
use sim_core::time::Time;

use crate::DeviceCtx;
use crate::i2c::{I2cDevice, crc8};

pub const DEFAULT_ADDRESS: u8 = 0x44;

#[derive(Clone, Debug)]
pub struct Sht41 {
    pub address: u8,
    pub temperature_c: Signal,
    pub humidity_rh: Signal,
    pub serial: u32,
    /// When the measurement in progress ends, and what it reads out.
    busy_until: Time,
    output: Vec<u8>,
    read_pos: usize,
    command: Vec<u8>,
    /// Measurements and commands seen (for tests).
    pub measurements: u32,
    pub naks: u32,
}

/// Datasheet maximum durations.
fn command_duration(cmd: u8) -> Option<Time> {
    Some(match cmd {
        0xFD => Time::from_us(8_300),
        0xF6 => Time::from_us(4_500),
        0xE0 => Time::from_us(1_600),
        // Heater pulses: 1 s or 0.1 s, then a high precision measurement
        0x39 | 0x2F | 0x1E => Time::from_ms(1_100),
        0x32 | 0x24 | 0x15 => Time::from_ms(110),
        0x89 | 0x94 => Time::from_ms(1),
        _ => return None,
    })
}

impl Sht41 {
    pub fn new(temperature_c: impl Into<Signal>, humidity_rh: impl Into<Signal>) -> Self {
        Sht41 {
            address: DEFAULT_ADDRESS,
            temperature_c: temperature_c.into(),
            humidity_rh: humidity_rh.into(),
            serial: 0x1234_5678,
            busy_until: Time::ZERO,
            output: Vec::new(),
            read_pos: 0,
            command: Vec::new(),
            measurements: 0,
            naks: 0,
        }
    }

    fn word(out: &mut Vec<u8>, w: u16) {
        let b = w.to_be_bytes();
        out.extend_from_slice(&b);
        out.push(crc8(&b, 0x31, 0xFF));
    }

    /// Raw ticks for a temperature: `T = -45 + 175 * St / 65535`.
    pub fn temperature_ticks(c: f64) -> u16 {
        ((c + 45.0) / 175.0 * 65535.0).round().clamp(0.0, 65535.0) as u16
    }

    /// Raw ticks for a humidity: `RH = -6 + 125 * Srh / 65535` (the sensor's output may exceed 0-100 %RH).
    pub fn humidity_ticks(rh: f64) -> u16 {
        ((rh + 6.0) / 125.0 * 65535.0).round().clamp(0.0, 65535.0) as u16
    }

    fn execute(&mut self, cmd: u8, now: Time) {
        let Some(d) = command_duration(cmd) else {
            log::debug!("SHT41: unknown command {cmd:#04x}");
            return;
        };
        let done = now + d;
        self.busy_until = done;
        self.read_pos = 0;
        self.output.clear();
        match cmd {
            0x89 => {
                Self::word(&mut self.output, (self.serial >> 16) as u16);
                Self::word(&mut self.output, self.serial as u16);
            }
            0x94 => {}
            _ => {
                self.measurements += 1;
                let t = self.temperature_c.at(done);
                let rh = self.humidity_rh.at(done);
                Self::word(&mut self.output, Self::temperature_ticks(t));
                Self::word(&mut self.output, Self::humidity_ticks(rh));
            }
        }
    }
}

impl I2cDevice for Sht41 {
    fn name(&self) -> String {
        "SHT41".into()
    }

    fn responds_to(&self, address: u8) -> bool {
        address == self.address
    }

    fn start(&mut self, ctx: &mut dyn DeviceCtx, _address: u8, read: bool) -> bool {
        if ctx.now() < self.busy_until {
            // No acknowledge while measuring
            self.naks += 1;
            return false;
        }
        if read {
            self.read_pos = 0;
        }
        self.command.clear();
        true
    }

    fn write(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        self.command.push(byte);
        if self.command.len() == 1 {
            self.execute(byte, ctx.now());
        }
        true
    }

    fn read(&mut self, _ctx: &mut dyn DeviceCtx, _ack: bool) -> u8 {
        let b = self.output.get(self.read_pos).copied().unwrap_or(0xFF);
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

    #[test]
    fn measurement_with_crc_and_busy_nak() {
        let mut ctx = TestCtx::default();
        let mut s = Sht41::new(25.0, 50.0);
        assert!(s.start(&mut ctx, 0x44, false));
        assert!(s.write(&mut ctx, 0xFD));
        ctx.now = Time::from_ms(5);
        assert!(!s.start(&mut ctx, 0x44, true), "NAK while measuring");
        ctx.now = Time::from_ms(10);
        assert!(s.start(&mut ctx, 0x44, true));
        let b: Vec<u8> = (0..6).map(|i| s.read(&mut ctx, i < 5)).collect();
        let st = u16::from_be_bytes([b[0], b[1]]);
        let srh = u16::from_be_bytes([b[3], b[4]]);
        assert_eq!(b[2], crc8(&b[0..2], 0x31, 0xFF));
        assert_eq!(b[5], crc8(&b[3..5], 0x31, 0xFF));
        let t = -45.0 + 175.0 * st as f64 / 65535.0;
        let rh = -6.0 + 125.0 * srh as f64 / 65535.0;
        assert!((t - 25.0).abs() < 0.01 && (rh - 50.0).abs() < 0.01, "{t} {rh}");
    }

    #[test]
    fn serial_number_and_signals() {
        let mut ctx = TestCtx::default();
        let (temp, h) = Signal::shared(10.0);
        let mut s = Sht41::new(temp, 40.0);
        s.start(&mut ctx, 0x44, false);
        s.write(&mut ctx, 0x89);
        ctx.now = Time::from_ms(2);
        s.start(&mut ctx, 0x44, true);
        let b: Vec<u8> = (0..6).map(|_| s.read(&mut ctx, true)).collect();
        assert_eq!([b[0], b[1], b[3], b[4]], [0x12, 0x34, 0x56, 0x78]);
        *h.lock().unwrap() = -10.0;
        s.start(&mut ctx, 0x44, false);
        s.write(&mut ctx, 0xE0);
        ctx.now = Time::from_ms(10);
        s.start(&mut ctx, 0x44, true);
        let st = u16::from_be_bytes([s.read(&mut ctx, true), s.read(&mut ctx, true)]);
        assert_eq!(st, Sht41::temperature_ticks(-10.0));
        assert!(!s.responds_to(0x45));
    }
}
