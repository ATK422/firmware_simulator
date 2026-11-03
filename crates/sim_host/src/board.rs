//! Board descriptions: `boards/<dir>/<board>.toml`, generated from titan_firmware's board headers by
//! `scripts/gen_boards.py` (plus hand-written overlays).
//!
//! A description says which external chips sit on which bus and pins; [`Board::apply`] attaches the models to a
//! machine. Unknown keys are ignored, so descriptions can carry information for models that arrive later.

use std::collections::BTreeMap;
use std::path::Path;

use rp2040::Machine;
use serde::Deserialize;
use sim_core::wire::{External, Level};
use sim_devices::can::BitRates;
use sim_devices::mcp251xfd::{Mcp251xfd, Mcp251xfdConfig};

#[derive(Debug, Clone, Deserialize)]
pub struct Board {
    pub name: String,
    #[serde(default = "default_mcu")]
    pub mcu: String,
    pub crystal_hz: Option<u64>,
    pub flash_size: Option<usize>,
    pub can: Option<CanSection>,
    #[serde(default)]
    pub spi: Vec<SpiEntry>,
    #[serde(default)]
    pub gpio: Vec<GpioEntry>,
    #[serde(default)]
    pub pins: BTreeMap<String, u8>,
}

fn default_mcu() -> String {
    "rp2040".into()
}

#[derive(Debug, Clone, Deserialize)]
pub struct CanSection {
    /// `internal` (CAN FD 1/5 Mbit/s) or `external` (classic 250 kbit/s) on Talos and Lilboat.
    pub bus: String,
    pub client_id: Option<u8>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpiEntry {
    pub bus: usize,
    pub cs: u8,
    pub mcp2517fd: Option<McpEntry>,
    pub mcp2518fd: Option<McpEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct McpEntry {
    pub int_pin: u8,
    pub clkin_pin: Option<u8>,
    pub crystal_hz: Option<u64>,
    #[serde(default = "yes")]
    pub termination: bool,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, Deserialize)]
pub struct GpioEntry {
    pub pin: u8,
    /// `up` or `down`: an external pull resistor.
    pub pull: Option<String>,
    /// `high` or `low`: driven from outside (a strap, a signal held by another chip).
    pub level: Option<String>,
}

/// Bit rates of the robot buses (`robots/talos.h`).
pub fn bus_rates(bus: &str) -> Option<BitRates> {
    match bus {
        "internal" => Some(BitRates { nominal: 1_000_000, data: Some(5_000_000) }),
        "external" => Some(BitRates { nominal: 250_000, data: None }),
        _ => None,
    }
}

impl Board {
    pub fn load(path: &Path) -> Result<Board, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Board, String> {
        let b: Board = toml::from_str(text).map_err(|e| e.to_string())?;
        if b.mcu != "rp2040" {
            return Err(format!("unsupported mcu {:?}", b.mcu));
        }
        Ok(b)
    }

    /// Attaches the board's external devices and pin conditions to `m`.
    pub fn apply(&self, m: &mut Machine) -> Result<(), String> {
        if let Some(hz) = self.crystal_hz {
            m.sys.clocks.xosc.crystal_hz = hz;
        }
        for s in &self.spi {
            let (entry, mcp2518) = match (&s.mcp2517fd, &s.mcp2518fd) {
                (Some(e), _) => (e, false),
                (None, Some(e)) => (e, true),
                (None, None) => continue,
            };
            let cfg = Mcp251xfdConfig {
                clkin_pin: entry.clkin_pin,
                crystal_hz: entry.crystal_hz,
                int_pin: entry.int_pin,
                gpio0: entry.termination,
                gpio1: false,
                mcp2518,
            };
            if s.bus > 1 {
                return Err(format!("SPI bus {} does not exist", s.bus));
            }
            m.sys.attach_spi(s.bus, s.cs, Box::new(Mcp251xfd::new(cfg)));
        }
        for g in &self.gpio {
            let ext = match (g.level.as_deref(), g.pull.as_deref()) {
                (Some("high"), _) => External::Driven(Level::High),
                (Some("low"), _) => External::Driven(Level::Low),
                (None, Some("up")) => External::Pull(Level::High),
                (None, Some("down")) => External::Pull(Level::Low),
                (None, None) => External::Floating,
                other => return Err(format!("GPIO {}: bad level/pull {other:?}", g.pin)),
            };
            if g.pin as usize >= rp2040::memmap::NUM_GPIOS {
                return Err(format!("GPIO {} does not exist", g.pin));
            }
            m.sys.gpio.external[g.pin as usize] = ext;
        }
        Ok(())
    }

    /// Bit rates of the CAN bus the board sits on.
    pub fn can_rates(&self) -> Option<BitRates> {
        self.can.as_ref().and_then(|c| bus_rates(&c.bus))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_power_board() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../boards/mk2/power_board.toml");
        let b = Board::load(&path).unwrap();
        assert_eq!(b.name, "mk2/power_board");
        assert_eq!(b.can.as_ref().unwrap().client_id, Some(1));
        assert_eq!(b.can_rates(), bus_rates("internal"));
        assert_eq!(b.spi[0].cs, 17);
        assert_eq!(b.pins["fan_tach_pin"], 29);
        let mut m = Machine::new(Default::default());
        b.apply(&mut m).unwrap();
        assert_eq!(m.sys.spi_devices.len(), 1);
        assert_eq!(m.sys.spi_devices[0].dev.name(), "MCP2517FD");
    }

    #[test]
    fn gpio_conditions() {
        let b = Board::parse("name = \"x\"\n[[gpio]]\npin = 29\npull = \"up\"\n[[gpio]]\npin = 6\nlevel = \"low\"\n")
            .unwrap();
        let mut m = Machine::new(Default::default());
        b.apply(&mut m).unwrap();
        assert_eq!(m.sys.gpio.external[29], External::Pull(Level::High));
        assert_eq!(m.sys.gpio.external[6], External::Driven(Level::Low));
        assert!(Board::parse("name = \"x\"\nmcu = \"stm32\"\n").is_err());
    }
}
