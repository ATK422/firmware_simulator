//! Board descriptions: `boards/<dir>/<board>.toml`, generated from titan_firmware's board headers by
//! `scripts/gen_boards.py` (plus hand-written overlays).
//!
//! A description says which external chips sit on which bus and pins; [`Board::apply`] attaches the models to a
//! machine. Unknown keys are ignored, so descriptions can carry information for models that arrive later.
//!
//! Physical inputs (sensor temperatures, rail voltages) are given as starting values. [`Board::apply`] returns them
//! as named [`Inputs`] that a test or host tool changes while the machine runs: `sht41.temperature_c`,
//! `mcp3426.ch1`, or an analog input's `name`.

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::{Arc, Mutex};

use rp2040::Machine;
use serde::Deserialize;
use sim_core::analog::Signal;
use sim_core::wire::{External, Level};
use sim_devices::can::BitRates;
use sim_devices::cd74hc4051::Cd74hc4051;
use sim_devices::i2c::I2cDevice;
use sim_devices::mcp251xfd::{Mcp251xfd, Mcp251xfdConfig};
use sim_devices::mcp342x::Mcp342x;
use sim_devices::ms5837::{self, Ms5837};
use sim_devices::sht41::Sht41;
use sim_devices::ssd1306::Ssd1306;

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
    pub i2c: Vec<I2cEntry>,
    #[serde(default)]
    pub analog: Vec<AnalogEntry>,
    #[serde(default)]
    pub gpio: Vec<GpioEntry>,
    #[serde(default)]
    pub pins: BTreeMap<String, u8>,
    /// The board's acoustic transducer, used when the board is placed in an acoustic environment.
    pub transducer: Option<TransducerEntry>,
    /// Bytes programmed into flash before boot (serial numbers and other per-board data).
    #[serde(default)]
    pub flash: Vec<FlashEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct FlashEntry {
    /// XIP address (0x1000_0000 and up).
    pub address: u32,
    pub bytes: Vec<u8>,
}

/// A transducer and how it is wired: the transducer's specs, plus the receive chain to an ADC input.
#[derive(Debug, Clone, Deserialize)]
pub struct TransducerEntry {
    #[serde(flatten)]
    pub transducer: sim_acoustics::transducer::Transducer,
    pub receive: Option<ReceiveEntry>,
    /// Projector drive (used from M7): the output pin and the driver's supply.
    pub drive: Option<toml::Value>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ReceiveEntry {
    /// ADC input 0-3 the chain feeds.
    pub adc: usize,
    #[serde(flatten)]
    pub chain: sim_acoustics::chain::ChainSpec,
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

/// A device on an I2C bus.
#[derive(Debug, Clone, Deserialize)]
pub struct I2cEntry {
    pub bus: usize,
    /// `sht41`, `mcp3426`, `mcp3428`, `ms5837_02ba`, `ms5837_30ba` or `ssd1306`.
    pub device: String,
    pub address: Option<u8>,
    /// Prefix of the device's input names (default: the device kind).
    pub name: Option<String>,
    // SHT41 and MS5837
    pub temperature_c: Option<f64>,
    pub humidity_rh: Option<f64>,
    // MS5837: absolute pressure, or depth below an atmospheric surface
    pub pressure_pa: Option<f64>,
    pub depth_m: Option<f64>,
    // MCP342x: what each channel measures
    #[serde(default)]
    pub channels: Vec<Measured>,
    // SSD1306: let page-mode positioning commands work in every addressing mode
    #[serde(default)]
    pub page_commands_in_any_mode: bool,
}

/// A voltage measured through an optional resistor divider (`[top, bottom]` ohms).
#[derive(Debug, Clone, Deserialize)]
pub struct Measured {
    pub volts: f64,
    pub divider_ohms: Option<[f64; 2]>,
    pub name: Option<String>,
}

impl Measured {
    fn ratio(&self) -> f64 {
        self.divider_ohms.map_or(1.0, |[top, bottom]| bottom / (top + bottom))
    }
}

/// What drives an ADC input: a voltage (through a divider) or a CD74HC4051 mux.
#[derive(Debug, Clone, Deserialize)]
pub struct AnalogEntry {
    /// ADC input 0-3 (GPIO 26-29).
    pub adc: usize,
    pub name: Option<String>,
    pub volts: Option<f64>,
    pub divider_ohms: Option<[f64; 2]>,
    pub mux: Option<MuxEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MuxEntry {
    pub select_pins: [u8; 3],
    pub enable_pin: Option<u8>,
    pub inputs: Vec<Measured>,
}

/// Named physical inputs of a board, changeable while it runs.
#[derive(Clone, Default, Debug)]
pub struct Inputs(pub BTreeMap<String, Arc<Mutex<f64>>>);

impl Inputs {
    /// Sets input `name`. Panics on an unknown name (a typo in a test should fail loudly).
    pub fn set(&self, name: &str, value: f64) {
        *self.handle(name).lock().unwrap() = value;
    }

    pub fn get(&self, name: &str) -> f64 {
        *self.handle(name).lock().unwrap()
    }

    pub fn handle(&self, name: &str) -> &Arc<Mutex<f64>> {
        self.0.get(name).unwrap_or_else(|| panic!("no input {name:?}; inputs: {:?}", self.0.keys().collect::<Vec<_>>()))
    }

    /// A new input `name` starting at `value`, as a signal scaled by `ratio` (a divider in front of the pin).
    fn add(&mut self, name: String, value: f64, ratio: f64) -> Signal {
        let h = Arc::new(Mutex::new(value));
        self.0.insert(name, h.clone());
        if ratio == 1.0 { Signal::Shared(h) } else { Signal::func(move |_| *h.lock().unwrap() * ratio) }
    }
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

    /// Attaches the board's external devices and pin conditions to `m`. Returns the board's physical inputs.
    pub fn apply(&self, m: &mut Machine) -> Result<Inputs, String> {
        let mut inputs = Inputs::default();
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
        for f in &self.flash {
            for (i, b) in f.bytes.iter().enumerate() {
                let a = f.address + i as u32;
                if !(0x1000_0000..0x1100_0000).contains(&a) || !m.sys.poke(a, sim_core::Size::Byte, *b as u32) {
                    return Err(format!("flash address {a:#010x} is outside flash"));
                }
            }
        }
        for e in &self.i2c {
            if e.bus > 1 {
                return Err(format!("I2C bus {} does not exist", e.bus));
            }
            let dev = i2c_device(e, &mut inputs)?;
            m.sys.attach_i2c(e.bus, dev);
        }
        for a in &self.analog {
            if a.adc > 3 {
                return Err(format!("ADC input {} is not a pin (0-3)", a.adc));
            }
            let name = a.name.clone().unwrap_or(format!("adc{}", a.adc));
            match (&a.mux, a.volts) {
                (Some(mux), _) => {
                    let mut sigs: [Signal; 8] = std::array::from_fn(|_| Signal::Const(0.0));
                    for (i, input) in mux.inputs.iter().take(8).enumerate() {
                        let n = input.name.clone().unwrap_or(format!("{name}.in{i}"));
                        sigs[i] = inputs.add(n, input.volts, input.ratio());
                    }
                    let mut dev = Cd74hc4051::new(mux.select_pins, sigs);
                    dev.enable_pin = mux.enable_pin;
                    m.sys.attach_analog(a.adc, Box::new(dev));
                }
                (None, Some(v)) => {
                    let ratio = Measured { volts: v, divider_ohms: a.divider_ohms, name: None }.ratio();
                    let sig = inputs.add(name, v, ratio);
                    m.sys.attach_analog(a.adc, Box::new(sig));
                }
                (None, None) => return Err(format!("ADC input {}: needs volts or a mux", a.adc)),
            }
        }
        Ok(inputs)
    }

    /// Puts the board's hydrophone at `position` in `env`, feeding its receive chain's ADC input.
    pub fn place_in(
        &self,
        m: &mut Machine,
        env: std::sync::Arc<sim_acoustics::Environment>,
        position: sim_acoustics::geometry::Vec3,
    ) -> Result<(), String> {
        let t = self.transducer.as_ref().ok_or_else(|| format!("{} has no [transducer]", self.name))?;
        let rx =
            t.receive.as_ref().ok_or_else(|| format!("{}: the transducer has no [transducer.receive]", self.name))?;
        if !env.boundaries.contains(position) {
            return Err(format!("{:?} is outside the water", position));
        }
        let h = sim_acoustics::Hydrophone::new(env, position, t.transducer.clone(), rx.chain.clone());
        m.sys.attach_analog(rx.adc, Box::new(h));
        Ok(())
    }

    /// Bit rates of the CAN bus the board sits on.
    pub fn can_rates(&self) -> Option<BitRates> {
        self.can.as_ref().and_then(|c| bus_rates(&c.bus))
    }
}

fn i2c_device(e: &I2cEntry, inputs: &mut Inputs) -> Result<Box<dyn I2cDevice>, String> {
    let name = e.name.clone().unwrap_or_else(|| e.device.clone());
    let n = |q: &str| format!("{name}.{q}");
    Ok(match e.device.as_str() {
        "sht41" => {
            let t = inputs.add(n("temperature_c"), e.temperature_c.unwrap_or(25.0), 1.0);
            let h = inputs.add(n("humidity_rh"), e.humidity_rh.unwrap_or(40.0), 1.0);
            let mut d = Sht41::new(t, h);
            d.address = e.address.unwrap_or(d.address);
            Box::new(d)
        }
        "mcp3426" | "mcp3427" | "mcp3428" => {
            let count = if e.device == "mcp3428" { 4 } else { 2 };
            let sigs = (0..count)
                .map(|i| match e.channels.get(i) {
                    Some(c) => inputs.add(c.name.clone().unwrap_or(n(&format!("ch{}", i + 1))), c.volts, c.ratio()),
                    None => inputs.add(n(&format!("ch{}", i + 1)), 0.0, 1.0),
                })
                .collect();
            let mut d = Mcp342x::with_inputs(sigs);
            d.address = e.address.unwrap_or(d.address);
            Box::new(d)
        }
        "ms5837_02ba" | "ms5837_30ba" => {
            let model = if e.device == "ms5837_02ba" { ms5837::Model::Ms5837_02ba } else { ms5837::Model::Ms5837_30ba };
            let p = e
                .pressure_pa
                .unwrap_or_else(|| ms5837::pressure_at_depth(ms5837::ATMOSPHERE_PA, e.depth_m.unwrap_or(0.0), 997.0));
            let p = inputs.add(n("pressure_pa"), p, 1.0);
            let t = inputs.add(n("temperature_c"), e.temperature_c.unwrap_or(20.0), 1.0);
            Box::new(Ms5837::new(model, p, t))
        }
        "ssd1306" => {
            let mut d = Ssd1306::new();
            d.address = e.address.unwrap_or(d.address);
            d.page_commands_in_any_mode = e.page_commands_in_any_mode;
            Box::new(d)
        }
        other => return Err(format!("unknown I2C device {other:?}")),
    })
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

    #[test]
    fn i2c_devices_analog_inputs_and_handles() {
        let b = Board::parse(
            r#"
name = "x"
[[i2c]]
bus = 0
device = "sht41"
temperature_c = 31.0
[[i2c]]
bus = 1
device = "mcp3426"
channels = [{ volts = 12.0, divider_ohms = [10_000, 1_000], name = "rail12" }]
[[analog]]
adc = 1
name = "battery"
volts = 16.5
divider_ohms = [10_000, 1_000]
[[analog]]
adc = 2
mux = { select_pins = [3, 4, 5], inputs = [{ volts = 1.0 }, { volts = 2.0 }] }
"#,
        )
        .unwrap();
        let mut m = Machine::new(Default::default());
        let inputs = b.apply(&mut m).unwrap();
        assert_eq!(m.sys.i2c_devices.len(), 2);
        assert_eq!(inputs.get("sht41.temperature_c"), 31.0);
        assert_eq!(inputs.get("rail12"), 12.0);
        assert_eq!(inputs.get("mcp3426.ch2"), 0.0);
        assert_eq!(inputs.get("adc2.in1"), 2.0);
        let v = m.sys.analog_inputs[1].as_mut().unwrap().voltage(sim_core::Time::ZERO, 0);
        assert!((v - 1.5).abs() < 1e-9, "divided down: {v}");
        inputs.set("battery", 11.0);
        let v = m.sys.analog_inputs[1].as_mut().unwrap().voltage(sim_core::Time::ZERO, 0);
        assert!((v - 1.0).abs() < 1e-9);
        let v = m.sys.analog_inputs[2].as_mut().unwrap().voltage(sim_core::Time::ZERO, 1 << 3);
        assert_eq!(v, 2.0);
        assert!(Board::parse("name = \"x\"\n[[i2c]]\nbus = 0\ndevice = \"nope\"\n").unwrap().apply(&mut m).is_err());
    }
}
