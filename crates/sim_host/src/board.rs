//! Board descriptions: `boards/<dir>/<board>.toml`, generated from titan_firmware's board headers by
//! `scripts/gen_boards.py` (plus hand-written overlays).
//!
//! A description says which external chips sit on which bus and pins; [`Board::apply`] attaches the models to a
//! machine. Unknown keys are ignored, so descriptions can carry information for models that arrive later.
//!
//! Physical inputs (sensor temperatures, rail voltages) are given as starting values. [`Board::apply`] returns them
//! as named [`Inputs`] that a test or host tool changes while the machine runs: `sht41.temperature_c`,
//! `mcp3426.ch1`, or an analog input's `name`. A snapshot of the machine gets its own copy of them
//! ([`Inputs::deep_copy`] and [`relink_inputs`]).

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::{Arc, Mutex};

use rp2040::Machine;
use serde::Deserialize;
use sim_core::analog::{Relink, Signal};
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
    /// WS2812 LED strips.
    #[serde(default)]
    pub ws2812: Vec<Ws2812Entry>,
    /// DShot ESCs with their motors.
    #[serde(default)]
    pub dshot_esc: Vec<EscEntry>,
    /// Dynamixel servos on a TTL bus pin.
    #[serde(default)]
    pub dynamixel: Vec<DynamixelEntry>,
    /// DRV8243 H-bridge drivers sharing an nSLEEP line.
    #[serde(default)]
    pub drv8243: Vec<Drv8243Entry>,
    /// A W5500 Ethernet controller (Puddles); its cable is `Machine::sys.ethernet`.
    pub w5500: Option<W5500Entry>,
    /// I2C/SMBus devices on bit-banged pins.
    #[serde(default)]
    pub soft_i2c: Vec<SoftI2cEntry>,
    /// Hiwonder bus servos on a half-duplex serial pin.
    #[serde(default)]
    pub hiwonder: Vec<HiwonderEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct HiwonderEntry {
    pub pin: u8,
    pub id: u8,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DynamixelEntry {
    pub pin: u8,
    pub id: u8,
    #[serde(default = "dxl_baud")]
    pub baud: u32,
    /// Where the horn is at power-up (pulses, 4096 per turn).
    pub position: Option<f64>,
    /// Operating mode register (3 position, 4 extended position).
    pub operating_mode: Option<u8>,
}

fn dxl_baud() -> u32 {
    57_600
}

#[derive(Debug, Clone, Deserialize)]
pub struct EscEntry {
    pub name: Option<String>,
    pub signal_pin: u8,
    pub telemetry_pin: Option<u8>,
    /// Mechanical RPM at full throttle.
    pub max_rpm: Option<f64>,
    pub pole_pairs: Option<u32>,
    /// How long after the last DShot frame the ESC stops the motor (250 ms by default, an assumption; see
    /// `sim_devices::dshot`); 0 keeps the last throttle forever.
    pub signal_loss_ms: Option<f64>,
    /// Time constant of the motor's first-order response to a throttle change (60 ms by default, an assumption;
    /// `firmware_simulator calibrate thrusters` fits it from test-stand data).
    pub tau_ms: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Ws2812Entry {
    pub pin: u8,
    pub leds: usize,
    pub name: Option<String>,
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
    /// Projector drive: the output pin and the driver's supply.
    pub drive: Option<sim_acoustics::medium::DriveSpec>,
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
    #[serde(default)]
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
    // BQ40Z80: serial number, pack state, side detect (true = port)
    pub serial: Option<u16>,
    pub voltage_mv: Option<f64>,
    pub current_ma: Option<f64>,
    pub soc_percent: Option<f64>,
    pub side_port: Option<bool>,
}

/// An I2C device on two bit-banged pins (a PIO or software I2C/SMBus).
#[derive(Debug, Clone, Deserialize)]
pub struct SoftI2cEntry {
    pub sda: u8,
    pub scl: u8,
    #[serde(flatten)]
    pub device: I2cEntry,
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
        if ratio == 1.0 { Signal::Shared(h) } else { Signal::Shared(h).scaled(ratio) }
    }

    /// Independent inputs with the same names and current values, and the map from these inputs to them, for
    /// pointing a copy of the machine's devices at the copy ([`relink_inputs`]).
    pub fn deep_copy(&self) -> (Inputs, Relink) {
        let mut map = Relink::new();
        let copy = self
            .0
            .iter()
            .map(|(name, h)| {
                let new = Arc::new(Mutex::new(*h.lock().unwrap()));
                map.insert(h, new.clone());
                (name.clone(), new)
            })
            .collect();
        (Inputs(copy), map)
    }
}

/// Points every device of `m` that reads a shared input in `map` at its replacement: the SPI, UART and I2C
/// devices, the pin devices, the ADC inputs and the die temperature. A machine copied with [`Machine::fork`] still
/// reads the original's inputs until this is done.
pub fn relink_inputs(m: &mut Machine, map: &Relink) {
    if map.is_empty() {
        return;
    }
    let sys = &mut m.sys;
    for a in &mut sys.spi_devices {
        a.dev.relink_inputs(map);
    }
    for d in sys.uart_devices.iter_mut().flatten() {
        d.relink_inputs(map);
    }
    for a in &mut sys.i2c_devices {
        a.dev.relink_inputs(map);
    }
    for d in &mut sys.pin_devices {
        d.relink_inputs(map);
    }
    for a in sys.analog_inputs.iter_mut().flatten() {
        a.relink_inputs(map);
    }
    sys.chip_temp_c.relink(map);
}

/// A W5500 on an SPI bus.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct W5500Entry {
    pub bus: usize,
    pub cs: u8,
    pub int_pin: u8,
}

/// DRV8243 drivers on one nSLEEP line.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Drv8243Entry {
    pub nsleep_pin: u8,
    pub drvoff_pin: Option<u8>,
    pub bridges: Vec<Drv8243Bridge>,
}

/// Where one bridge's nFAULT goes: a GPIO, or a named input of an `[[analog]]` entry (a mux input).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Drv8243Bridge {
    pub nfault_pin: Option<u8>,
    pub nfault_input: Option<String>,
    /// `false` for a bridge that is not fitted (never answers).
    #[serde(default = "yes")]
    pub fitted: bool,
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
        for e in &self.ws2812 {
            let name = e.name.clone().unwrap_or_else(|| "ws2812".into());
            m.sys.attach_pin_device(Box::new(sim_devices::ws2812::Ws2812::new(&name, e.pin, e.leds)));
        }
        for (i, e) in self.dshot_esc.iter().enumerate() {
            let name = e.name.clone().unwrap_or_else(|| format!("esc{}", i + 1));
            let mut esc = sim_devices::dshot::Esc::new(&name, e.signal_pin, e.telemetry_pin);
            esc.max_rpm = e.max_rpm.unwrap_or(esc.max_rpm);
            esc.pole_pairs = e.pole_pairs.unwrap_or(esc.pole_pairs);
            if let Some(ms) = e.tau_ms {
                if !(ms > 0.0 && ms.is_finite()) {
                    return Err(format!("[[dshot_esc]] {name}: tau_ms must be positive"));
                }
                esc.tau = sim_core::Time::from_secs_f64(ms / 1000.0);
            }
            if let Some(ms) = e.signal_loss_ms {
                esc.signal_loss_timeout = (ms > 0.0).then(|| sim_core::Time::from_secs_f64(ms / 1000.0));
            }
            m.sys.attach_pin_device(Box::new(esc));
        }
        for e in &self.soft_i2c {
            let dev = i2c_device(&e.device, &mut inputs)?;
            m.sys.attach_pin_device(Box::new(sim_devices::i2c_pins::I2cPinTarget::new(e.sda, e.scl, dev)));
        }
        for h in &self.hiwonder {
            m.sys.attach_pin_device(Box::new(sim_devices::hiwonder::Hiwonder::new(h.pin, h.id)));
        }
        for d in &self.dynamixel {
            let mut dxl = sim_devices::dynamixel::Dynamixel::xl430(d.pin, d.id, d.baud);
            if let Some(p) = d.position {
                dxl.set_physical_position(p);
            }
            if let Some(mode) = d.operating_mode {
                dxl.table[11] = mode;
            }
            m.sys.attach_pin_device(Box::new(dxl));
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
        if let Some(e) = &self.w5500 {
            if e.bus > 1 {
                return Err(format!("SPI bus {} does not exist", e.bus));
            }
            let wire = sim_devices::w5500::wire();
            m.sys.attach_spi(e.bus, e.cs, Box::new(sim_devices::w5500::W5500::new(wire.clone(), e.int_pin)));
            m.sys.ethernet = Some(wire);
        }
        // After the analog inputs: nFAULT lines can feed them
        for d in &self.drv8243 {
            let mut bridges = Vec::new();
            for b in &d.bridges {
                let out = match (b.nfault_pin, &b.nfault_input) {
                    (Some(p), None) => {
                        if p as usize >= rp2040::memmap::NUM_GPIOS {
                            return Err(format!("DRV8243 nFAULT: GPIO {p} does not exist"));
                        }
                        m.sys.gpio.external[p as usize] = External::Driven(Level::High);
                        sim_devices::drv8243::Nfault::Pin(p)
                    }
                    (None, Some(name)) => {
                        let h = inputs
                            .0
                            .get(name)
                            .ok_or_else(|| format!("DRV8243 nFAULT: no input {name:?} (name a mux input)"))?
                            .clone();
                        *h.lock().unwrap() = 3.3;
                        sim_devices::drv8243::Nfault::Volts(h)
                    }
                    _ => return Err("DRV8243 bridge: give one of nfault_pin and nfault_input".into()),
                };
                bridges.push((out, b.fitted));
            }
            let dev = sim_devices::drv8243::Drv8243::new(d.nsleep_pin, d.drvoff_pin, bridges);
            m.sys.attach_pin_device(Box::new(dev));
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

    /// Puts the board as board `owner` in a medium shared with other boards, moving along `trajectory`: its
    /// projector (if the transducer has a `drive`) radiates what its drive pin outputs, and its hydrophone (if it has
    /// a `receive` chain) hears the environment and the other boards' projectors.
    pub fn place_in_medium(
        &self,
        m: &mut Machine,
        medium: &sim_acoustics::medium::SharedMedium,
        owner: usize,
        trajectory: sim_acoustics::medium::Trajectory,
    ) -> Result<(), String> {
        let t = self.transducer.as_ref().ok_or_else(|| format!("{} has no [transducer]", self.name))?;
        let env = medium.lock().unwrap().env.clone();
        let start = trajectory.at(0.0);
        if !env.boundaries.contains(start) {
            return Err(format!("{:?} is outside the water", start));
        }
        if let Some(rx) = &t.receive {
            let h = sim_acoustics::Hydrophone::new(env, start, t.transducer.clone(), rx.chain.clone())
                .in_medium(medium.clone(), owner)
                .moving(trajectory.clone());
            m.sys.attach_analog(rx.adc, Box::new(h));
        }
        if let Some(drive) = &t.drive {
            let id = medium.lock().unwrap().add_projector(owner, &self.name, t.transducer.clone(), trajectory, drive);
            m.sys.attach_pin_device(Box::new(crate::acoustic::ProjectorPin::new(medium.clone(), id, drive.pin)));
        }
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
        "bq40z80" => {
            let mut g = sim_devices::bq40z80::Bq40z80::new(e.serial.unwrap_or(1));
            g.voltage_mv = inputs.add(n("voltage_mv"), e.voltage_mv.unwrap_or(16_400.0), 1.0);
            g.current_ma = inputs.add(n("current_ma"), e.current_ma.unwrap_or(-2_500.0), 1.0);
            g.soc_percent = inputs.add(n("soc_percent"), e.soc_percent.unwrap_or(80.0), 1.0);
            g.temperature_c = inputs.add(n("temperature_c"), e.temperature_c.unwrap_or(25.0), 1.0);
            g.side_port = e.side_port.unwrap_or(true);
            Box::new(g)
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
    use rp2040::system::I2cAttachment;
    use sim_devices::drv8243::Drv8243;
    use sim_devices::fault::{FaultyAnalog, FaultyI2c};

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

    /// Board-file values that name a GPIO or bus the chip doesn't have are errors, like the other sections'
    /// checks: a DRV8243 `nfault_pin` past GPIO 29 used to panic indexing the pin array, and a W5500 on SPI bus 2
    /// was attached to nothing without a word.
    #[test]
    fn out_of_range_pins_and_buses_are_errors() {
        for text in [
            "name = \"x\"\n[[drv8243]]\nnsleep_pin = 25\nbridges = [{ nfault_pin = 30 }]\n",
            "name = \"x\"\n[w5500]\nbus = 2\ncs = 1\nint_pin = 4\n",
        ] {
            let b = Board::parse(text).unwrap();
            let mut m = Machine::new(Default::default());
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| b.apply(&mut m).map(|_| ())));
            assert!(matches!(r, Ok(Err(_))), "{text}: {r:?}");
        }
    }

    #[test]
    fn esc_motor_time_constant() {
        let b =
            Board::parse("name = \"x\"\n[[dshot_esc]]\nsignal_pin = 1\n[[dshot_esc]]\nsignal_pin = 2\ntau_ms = 35\n")
                .unwrap();
        let mut m = Machine::new(Default::default());
        b.apply(&mut m).unwrap();
        let t: Vec<_> = m
            .sys
            .pin_devices
            .iter()
            .map(|d| d.as_any().downcast_ref::<sim_devices::dshot::Esc>().unwrap().tau)
            .collect();
        assert_eq!(t, [sim_core::Time::from_ms(60), sim_core::Time::from_ms(35)]);
        let bad = Board::parse("name = \"x\"\n[[dshot_esc]]\nsignal_pin = 1\ntau_ms = 0\n").unwrap();
        assert!(bad.apply(&mut Machine::new(Default::default())).is_err());
    }

    #[test]
    fn esc_signal_loss_timeout() {
        let b = Board::parse(
            "name = \"x\"\n[[dshot_esc]]\nsignal_pin = 1\n[[dshot_esc]]\nsignal_pin = 2\nsignal_loss_ms = 100\n\
             [[dshot_esc]]\nsignal_pin = 3\nsignal_loss_ms = 0\n",
        )
        .unwrap();
        let mut m = Machine::new(Default::default());
        b.apply(&mut m).unwrap();
        let t: Vec<_> = m
            .sys
            .pin_devices
            .iter()
            .map(|d| d.as_any().downcast_ref::<sim_devices::dshot::Esc>().unwrap().signal_loss_timeout)
            .collect();
        assert_eq!(
            t,
            [Some(sim_devices::dshot::Esc::DEFAULT_SIGNAL_LOSS_TIMEOUT), Some(sim_core::Time::from_ms(100)), None]
        );
    }

    #[test]
    fn a_copy_gets_inputs_of_its_own() {
        let b = Board::parse(
            r#"
name = "x"
[[i2c]]
bus = 0
device = "sht41"
[[i2c]]
bus = 1
device = "mcp3426"
channels = [{ volts = 12.0, divider_ohms = [10_000, 1_000], name = "rail12" }]
[[soft_i2c]]
sda = 7
scl = 6
device = "bq40z80"
name = "gauge"
[[analog]]
adc = 1
name = "battery"
volts = 16.5
divider_ohms = [10_000, 1_000]
[[analog]]
adc = 2
mux = { select_pins = [3, 4, 5], inputs = [{ volts = 3.3, name = "h0.nfault_v" }, { volts = 2.0 }] }
[[drv8243]]
nsleep_pin = 25
bridges = [{ nfault_input = "h0.nfault_v" }]
"#,
        )
        .unwrap();
        let mut m = Machine::new(Default::default());
        let inputs = b.apply(&mut m).unwrap();
        // A fault wrapper in between (as fault injection leaves it) is looked through
        let sht = m.sys.i2c_devices.remove(0);
        m.sys.i2c_devices.insert(0, I2cAttachment { bus: 0, dev: Box::new(FaultyI2c::new(sht.dev)) });
        let adc1 = m.sys.analog_inputs[1].take().unwrap();
        m.sys.analog_inputs[1] = Some(Box::new(FaultyAnalog::new(adc1, 1)));

        let mut copy = m.fork();
        let (copy_inputs, map) = inputs.deep_copy();
        assert_eq!(map.len(), inputs.0.len());
        relink_inputs(&mut copy, &map);
        assert_eq!(copy_inputs.get("rail12"), 12.0, "same values to start with");
        for (name, v) in [("sht41.temperature_c", 40.0), ("rail12", 11.0), ("gauge.voltage_mv", 15_000.0)] {
            copy_inputs.set(name, v);
        }
        copy_inputs.set("battery", 11.0);
        copy_inputs.set("adc2.in1", 1.0);

        let t = sim_core::Time::ZERO;
        let sht41 =
            |m: &Machine| m.sys.i2c_devices[0].dev.as_any().downcast_ref::<Sht41>().unwrap().temperature_c.at(t);
        let rail = |m: &Machine| m.sys.i2c_devices[1].dev.as_any().downcast_ref::<Mcp342x>().unwrap().inputs[0].at(t);
        let gauge = |m: &Machine| {
            let p = m.sys.pin_devices[0].as_any().downcast_ref::<sim_devices::i2c_pins::I2cPinTarget>().unwrap();
            p.dev.as_any().downcast_ref::<sim_devices::bq40z80::Bq40z80>().unwrap().voltage_mv.at(t)
        };
        assert_eq!((sht41(&m), sht41(&copy)), (25.0, 40.0));
        assert_eq!((rail(&m), rail(&copy)), (12.0 / 11.0, 1.0));
        assert_eq!((gauge(&m), gauge(&copy)), (16_400.0, 15_000.0));
        let adc = |m: &mut Machine, i: usize, pins: u32| m.sys.analog_inputs[i].as_mut().unwrap().voltage(t, pins);
        assert!((adc(&mut m, 1, 0) - 1.5).abs() < 1e-9 && (adc(&mut copy, 1, 0) - 1.0).abs() < 1e-9);
        assert_eq!((adc(&mut m, 2, 1 << 3), adc(&mut copy, 2, 1 << 3)), (2.0, 1.0));
        // The copy's DRV8243 pulls the copy's nFAULT input, not the original's
        let drv = |m: &Machine| match &m.sys.pin_devices[1].as_any().downcast_ref::<Drv8243>().unwrap().bridges[0].0 {
            sim_devices::drv8243::Nfault::Volts(h) => h.clone(),
            _ => unreachable!(),
        };
        assert!(Arc::ptr_eq(&drv(&m), inputs.handle("h0.nfault_v")));
        assert!(Arc::ptr_eq(&drv(&copy), copy_inputs.handle("h0.nfault_v")));
        // The original's inputs are untouched
        assert_eq!(inputs.get("sht41.temperature_c"), 25.0);
    }
}
