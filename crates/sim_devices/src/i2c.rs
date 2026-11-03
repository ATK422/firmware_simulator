//! I2C (and SMBus) target device interface.
//!
//! The SoC's I2C controller drives the bus one event at a time: a START (or repeated START) with an address and
//! direction, data bytes either way, and a STOP. The device on that address answers each with an ACK or NAK, or a
//! byte, and may hold SCL low (clock stretching) before a byte.

use crate::DeviceCtx;
use sim_core::time::Time;

pub trait I2cDevice: Send {
    fn name(&self) -> String;
    /// Whether the device answers 7-bit `address`.
    fn responds_to(&self, address: u8) -> bool;
    /// A START or repeated START addressed to the device. Returns whether the device ACKs its address.
    fn start(&mut self, ctx: &mut dyn DeviceCtx, address: u8, read: bool) -> bool;
    /// A byte written by the controller. Returns ACK (true) or NAK.
    fn write(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> bool;
    /// A byte read by the controller. `ack` is whether the controller will ACK it (false on the last byte).
    fn read(&mut self, ctx: &mut dyn DeviceCtx, ack: bool) -> u8;
    /// STOP on the bus after a transfer with this device.
    fn stop(&mut self, ctx: &mut dyn DeviceCtx) {
        let _ = ctx;
    }
    /// How long the device holds SCL low before the next byte (clock stretching).
    fn stretch(&mut self, ctx: &mut dyn DeviceCtx) -> Time {
        let _ = ctx;
        Time::ZERO
    }
    /// Lets the device update its outputs after time passed.
    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        let _ = ctx;
    }
    fn clone_box(&self) -> Box<dyn I2cDevice>;
    /// The model itself, for tests and tools that inspect it (`downcast_ref`).
    fn as_any(&self) -> &dyn std::any::Any;
    /// The fault injector, if this is a device wrapped in one ([`crate::fault::FaultyI2c`]).
    fn faulty(&mut self) -> Option<&mut crate::fault::FaultyI2c> {
        None
    }
    /// Points the shared input values the device reads at their counterparts in `map` (a snapshot's own inputs,
    /// see [`sim_core::analog::Relink`]). Devices that read inputs implement it; the default reads none.
    fn relink_inputs(&mut self, _map: &sim_core::analog::Relink) {}
}

sim_core::clone_box!(I2cDevice);

/// SMBus packet error check: CRC-8, polynomial 0x07, initial 0.
pub fn smbus_pec(data: &[u8]) -> u8 {
    crc8(data, 0x07, 0x00)
}

/// MSB-first CRC-8 with the given polynomial and initial value (no reflection, no final XOR).
pub fn crc8(data: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ poly } else { crc << 1 };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_check_values() {
        // CRC-8/SMBUS check value for "123456789"
        assert_eq!(smbus_pec(b"123456789"), 0xF4);
        // Sensirion CRC-8 (poly 0x31, init 0xFF): datasheet example 0xBEEF -> 0x92
        assert_eq!(crc8(&[0xBE, 0xEF], 0x31, 0xFF), 0x92);
    }
}
