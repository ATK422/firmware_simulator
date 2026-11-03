//! Behavioural models of the external chips on the UWRT boards, attached to the SoC's SPI, I2C, UART or GPIO wires,
//! and the CAN bus they talk on.
//!
//! * [`DeviceCtx`]: what a device sees of the board (time, pins it drives or senses, clocks fed to it)
//! * [`spi`]: the SPI device interface
//! * [`can`]: CAN frames and the in-process, arbitrating CAN bus
//! * [`mcp251xfd`]: the MCP2517FD/MCP2518FD SPI CAN FD controller
//!
//! Models are SoC independent: the machine implements [`DeviceCtx`] for them.

pub mod can;
pub mod mcp251xfd;
pub mod spi;

use sim_core::time::Time;
use sim_core::wire::External;

/// The board as a device sees it.
pub trait DeviceCtx {
    /// Current virtual time.
    fn now(&self) -> Time;
    /// Drives (or releases) one of the SoC's pins from outside, e.g. an interrupt output of the device.
    fn set_pin(&mut self, pin: u8, value: External);
    /// Resolved level of a SoC pin.
    fn pin_level(&self, pin: u8) -> bool;
    /// Frequency of a clock the SoC outputs on `pin` (GPOUT), 0 if the pin carries no clock.
    fn pin_clock_hz(&self, pin: u8) -> u64;
}

/// A [`DeviceCtx`] for unit tests: pins in an array, one clock source.
#[derive(Clone, Debug)]
pub struct TestCtx {
    pub now: Time,
    pub pins: [External; 32],
    pub clocks: [u64; 32],
}

impl Default for TestCtx {
    fn default() -> Self {
        TestCtx { now: Time::ZERO, pins: [External::Floating; 32], clocks: [0; 32] }
    }
}

impl DeviceCtx for TestCtx {
    fn now(&self) -> Time {
        self.now
    }
    fn set_pin(&mut self, pin: u8, value: External) {
        self.pins[pin as usize] = value;
    }
    fn pin_level(&self, pin: u8) -> bool {
        matches!(self.pins[pin as usize], External::Driven(sim_core::wire::Level::High))
    }
    fn pin_clock_hz(&self, pin: u8) -> u64 {
        self.clocks[pin as usize]
    }
}
