//! Behavioural models of the external chips on the UWRT boards, attached to the SoC's SPI, I2C, UART or GPIO wires,
//! and the CAN bus they talk on.
//!
//! * [`DeviceCtx`]: what a device sees of the board (time, pins it drives or senses, clocks fed to it)
//! * [`spi`], [`i2c`], [`uart`], [`pin`]: the SPI, I2C, UART and pin-level device interfaces
//! * [`can`]: CAN frames and the in-process, arbitrating CAN bus
//! * [`mcp251xfd`]: the MCP2517FD/MCP2518FD SPI CAN FD controller
//! * [`sht41`], [`mcp342x`], [`ms5837`], [`ssd1306`]: I2C sensors, ADC and display
//! * [`cd74hc4051`]: an analog multiplexer in front of an ADC input
//! * [`ws2812`]: an LED strip, decoded from its data pin
//!
//! Physical inputs (temperatures, pressures, voltages) are [`sim_core::analog::Signal`]s.
//!
//! Models are SoC independent: the machine implements [`DeviceCtx`] for them.

pub mod bq40z80;
pub mod can;
pub mod cd74hc4051;
pub mod drv8243;
pub mod dshot;
pub mod dynamixel;
pub mod hiwonder;
pub mod i2c;
pub mod i2c_pins;
pub mod mcp251xfd;
pub mod mcp342x;
pub mod ms5837;
pub mod pin;
pub mod png;
pub mod sht41;
pub mod spi;
pub mod ssd1306;
pub mod uart;
pub mod w5500;
pub mod ws2812;

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
    /// Drives (or releases) a pin from outside at a future time `at` (a reply some time after a stimulus). Only
    /// pin-level devices need this; contexts that cannot schedule ignore it.
    fn drive_at(&mut self, at: Time, pin: u8, value: External) {
        let _ = (at, pin, value);
    }
    /// Asks for the device's `update` to be called at `at` (timeouts, end of a gap).
    fn wake_at(&mut self, at: Time) {
        let _ = at;
    }
    /// Reports something a device observed (a decoded frame, a protocol error) to the machine's trace.
    fn report(&mut self, message: String) {
        let _ = message;
    }
}

/// A [`DeviceCtx`] for unit tests: pins in an array, one clock source.
#[derive(Clone, Debug)]
pub struct TestCtx {
    pub now: Time,
    pub pins: [External; 32],
    pub clocks: [u64; 32],
    /// Drives scheduled with `drive_at`, and wake-ups asked for.
    pub scheduled: Vec<(Time, u8, External)>,
    pub wakes: Vec<Time>,
    pub reports: Vec<String>,
}

impl Default for TestCtx {
    fn default() -> Self {
        TestCtx {
            now: Time::ZERO,
            pins: [External::Floating; 32],
            clocks: [0; 32],
            scheduled: Vec::new(),
            wakes: Vec::new(),
            reports: Vec::new(),
        }
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
    fn drive_at(&mut self, at: Time, pin: u8, value: External) {
        self.scheduled.push((at, pin, value));
    }
    fn wake_at(&mut self, at: Time) {
        self.wakes.push(at);
    }
    fn report(&mut self, message: String) {
        self.reports.push(message);
    }
}
