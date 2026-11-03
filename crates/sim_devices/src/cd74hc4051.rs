//! TI CD74HC4051 8:1 analog multiplexer in front of an ADC input.
//!
//! Three select pins (S0 = LSB) and an active-low enable pick which of eight inputs reaches the common pin. The
//! common pin is the [`AnalogSource`] an ADC input is connected to; with the mux disabled it floats (0 V here).
//! The select pins are read from the SoC's GPIO levels at each conversion.

use sim_core::analog::{AnalogSource, Signal};
use sim_core::time::Time;

#[derive(Clone, Debug)]
pub struct Cd74hc4051 {
    /// GPIOs wired to S0, S1, S2.
    pub select_pins: [u8; 3],
    /// GPIO wired to the active-low enable, if it is not tied low.
    pub enable_pin: Option<u8>,
    pub inputs: [Signal; 8],
    /// The input selected at each conversion (0xFF: disabled), when built with [`Cd74hc4051::logging`]. The on
    /// resistance is ignored: the output equals the selected input.
    pub selected_log: Vec<u8>,
    log_selections: bool,
}

impl Cd74hc4051 {
    pub fn new(select_pins: [u8; 3], inputs: [Signal; 8]) -> Self {
        Cd74hc4051 { select_pins, enable_pin: None, inputs, selected_log: Vec::new(), log_selections: false }
    }

    /// Records the selected input at every conversion (for tests).
    pub fn logging(mut self) -> Self {
        self.log_selections = true;
        self
    }

    pub fn selected(&self, pins: u32) -> Option<u8> {
        if self.enable_pin.is_some_and(|p| pins & (1 << p) != 0) {
            return None;
        }
        Some(self.select_pins.iter().enumerate().fold(0, |n, (bit, &p)| n | (((pins >> p) & 1) as u8) << bit))
    }
}

impl AnalogSource for Cd74hc4051 {
    fn voltage(&mut self, t: Time, pins: u32) -> f64 {
        let sel = self.selected(pins);
        if self.log_selections {
            self.selected_log.push(sel.unwrap_or(0xFF));
        }
        sel.map_or(0.0, |s| self.inputs[s as usize].at(t))
    }

    fn clone_box(&self) -> Box<dyn AnalogSource> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selects_by_pins_and_enable() {
        let inputs = std::array::from_fn(|i| Signal::Const(i as f64 * 0.1));
        let mut m = Cd74hc4051::new([10, 11, 12], inputs);
        assert_eq!(m.voltage(Time::ZERO, 0), 0.0);
        assert!((m.voltage(Time::ZERO, 1 << 10 | 1 << 12) - 0.5).abs() < 1e-9);
        m.enable_pin = Some(13);
        assert_eq!(m.selected(1 << 13 | 1 << 11), None);
        assert_eq!(m.selected(1 << 11), Some(2));
    }
}
