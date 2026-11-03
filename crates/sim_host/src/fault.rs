//! Fault injection: what can go wrong around a running board, applied to its session, its CAN bus and the host
//! side of its links.
//!
//! [`Fault`] lists the faults; [`FaultTarget::apply`] applies one to a board (for a single board, a session with
//! its own bus; for a robot, one of its boards with the bus it is on). Each fault is a realistic model of the
//! failure, not a shortcut into firmware state:
//!
//! * **CAN**: bit errors and corrupted frames become error frames that the controllers count (TEC/REC, error
//!   passive, bus-off and its automatic recovery, see [`sim_devices::mcp251xfd`]); a stuck-dominant bus, a board
//!   unplugged from the bus, a flood from a babbling node ([`sim_devices::can::BusFaults`]).
//! * **Parts**: an I2C device that stops answering, answers intermittently, freezes, glitches or holds the clock
//!   ([`sim_devices::fault::I2cFault`]); an ADC input shorted, open or noisy ([`sim_devices::fault::AnalogFault`]).
//! * **Power and the chip**: a brownout of the board's supply (the RP2040 and the chips on its 3.3 V rail lose
//!   power, the RP2040 comes back through a power-on reset); a task that hangs, optionally with interrupts masked,
//!   which starves the safety task so the watchdog fires; a panic; a forced watchdog reset.
//! * **Links**: `titan_bridge` stopping or restarting, the kill switch, the Puddles Ethernet cable unplugged, and
//!   packet loss on it.

use std::ops::Range;

use rp2040::periph::timer::WatchdogReset;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::{CanBus, CanFrame, Flood};
use sim_devices::fault::{AnalogFault, FaultyAnalog, FaultyI2c, I2cFault, Rng};
use sim_devices::i2c_pins::I2cPinTarget;
use sim_devices::mcp251xfd::{ErrorCounters, Mcp251xfd};
use sim_devices::w5500::W5500;

use crate::Session;
use crate::ethernet::PuddlesBridge;
use crate::mock_bridge::MockBridge;

/// Something that goes wrong.
#[derive(Clone, Debug, PartialEq)]
pub enum Fault {
    // ---- CAN bus (the one the board is on)
    /// Every frame on the bus is hit by a bit error with this probability (0 ends it).
    CanErrorRate(f64),
    /// The next `n` frames on the bus are hit by a bit error.
    CanCorrupt(u32),
    /// The bus is held dominant (`true`) or released.
    CanStuckDominant(bool),
    /// The board's CAN controller is unplugged from the bus (`true`) or plugged back in.
    CanUnplugged(bool),
    /// Another node floods the bus with frame `id` (`len` data bytes, CAN FD on an FD bus) at `load` (0 to 1) of
    /// its capacity.
    CanFlood { id: u32, extended: bool, len: usize, load: f64 },
    /// The flooding nodes stop.
    CanFloodStop,

    // ---- Parts on the board
    /// The I2C device at `address` (on either bus, bit-banged ones included) fails, or is repaired (`None`).
    I2c { address: u8, fault: Option<I2cFault> },
    /// ADC input `input` (0-3, GPIO 26-29) fails, or is repaired (`None`).
    Adc { input: usize, fault: Option<AnalogFault> },
    /// GPIO `pin` is driven from outside to `Some(level)` (a switch, a stuck signal, a short), or released
    /// (`None`): the Talos power board's physical kill switch is GPIO 1, low while it asserts kill.
    Gpio { pin: u8, level: Option<bool> },

    // ---- Power and the chip
    /// The board's supply drops out for `off`: the RP2040 and the chips on its 3.3 V rail (CAN controller,
    /// Ethernet controller) lose power; the RP2040 comes back through a power-on reset (the brown-out detector
    /// and the power-on reset both report `CHIP_RESET.HAD_POR`). The board's clock jumps over the time it was off.
    Brownout { off: Time },
    /// Core `core` hangs in a busy loop where it is (a task that never yields), with interrupts masked if
    /// `interrupts_masked` (a critical section that never ends). Nothing else runs on that core, so the safety
    /// task stops feeding the watchdog. The loop is a `b .` from the image ([`hang_address`]), or one placed in the
    /// XIP cache's SRAM for images without one.
    Hang { core: usize, interrupts_masked: bool },
    /// Core 0 calls the firmware's panic path.
    Panic,
    /// The watchdog resets the chip as if its timer had run out (`WATCHDOG_REASON.TIMER`), without the firmware
    /// having hung.
    WatchdogTimeout,

    // ---- Links to the robot computer
    /// `titan_bridge` (the mock bridge on the board's bus or Ethernet) stops (`false`) or starts again (`true`).
    Bridge(bool),
    /// The kill switch the bridge forwards with every ping: `Some(true)` kills, `Some(false)` enables, `None` stops
    /// forwarding (the firmware's kill switch then times out).
    KillSwitch(Option<bool>),
    /// The Ethernet cable is plugged in (`true`) or unplugged: the W5500's PHY link follows it.
    EthernetLink(bool),
    /// Frames between the board's Ethernet cable and the host are lost with these probabilities (0 and 0 end it).
    PacketLoss { to_board: f64, from_board: f64 },
}

fn parse_duration(s: &str) -> Result<Time, String> {
    let (num, scale) = if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1e-3)
    };
    let v: f64 = num.parse().map_err(|_| format!("bad time {s:?}"))?;
    Ok(Time::from_secs_f64(v * scale))
}

fn parse_num<T: std::str::FromStr>(s: Option<&str>, what: &str) -> Result<T, String> {
    let s = s.ok_or_else(|| format!("missing {what}"))?;
    let parsed = match s.strip_prefix("0x") {
        Some(hex) => u64::from_str_radix(hex, 16).map(|v| v.to_string()).map_err(|e| e.to_string())?,
        None => s.to_string(),
    };
    parsed.parse().map_err(|_| format!("bad {what} {s:?}"))
}

impl std::str::FromStr for Fault {
    type Err = String;

    /// Parses `name[:arg...]`, the form the command line's `--fault` takes:
    ///
    /// | Spec | Fault |
    /// |---|---|
    /// | `can-error-rate:P`, `can-corrupt:N` | [`Fault::CanErrorRate`], [`Fault::CanCorrupt`] |
    /// | `can-stuck`, `can-release` | [`Fault::CanStuckDominant`] |
    /// | `can-unplug`, `can-plug` | [`Fault::CanUnplugged`] |
    /// | `can-flood:ID:LOAD[:LEN]`, `can-flood-stop` | [`Fault::CanFlood`] (standard ID, 8 bytes by default) |
    /// | `i2c-absent:ADDR`, `i2c-stuck:ADDR`, `i2c-ok:ADDR` | [`Fault::I2c`] |
    /// | `i2c-intermittent:ADDR:P`, `i2c-glitch:ADDR:P`, `i2c-stretch:ADDR:TIME` | [`Fault::I2c`] |
    /// | `adc-short:IN:VOLTS`, `adc-open:IN`, `adc-noise:IN:SIGMA`, `adc-spikes:IN:P:VOLTS`, `adc-ok:IN` | [`Fault::Adc`] |
    /// | `gpio:PIN:high`, `gpio:PIN:low`, `gpio:PIN:release` | [`Fault::Gpio`] |
    /// | `brownout:TIME`, `hang:CORE`, `hang-masked:CORE`, `panic`, `watchdog` | power and chip faults |
    /// | `bridge-stop`, `bridge-start`, `kill:on`, `kill:off`, `kill:none` | [`Fault::Bridge`], [`Fault::KillSwitch`] |
    /// | `eth-unplug`, `eth-plug`, `packet-loss:TO:FROM` | [`Fault::EthernetLink`], [`Fault::PacketLoss`] |
    ///
    /// Times take `us`, `ms` or `s` (milliseconds without a unit); numbers may be hex (`0x76`).
    fn from_str(spec: &str) -> Result<Fault, String> {
        let mut parts = spec.split(':');
        let name = parts.next().unwrap_or("");
        let mut arg = || parts.next();
        let i2c = |addr: Option<&str>, fault| -> Result<Fault, String> {
            Ok(Fault::I2c { address: parse_num(addr, "I2C address")?, fault })
        };
        Ok(match name {
            "can-error-rate" => Fault::CanErrorRate(parse_num(arg(), "probability")?),
            "can-corrupt" => Fault::CanCorrupt(parse_num(arg(), "frame count")?),
            "can-stuck" => Fault::CanStuckDominant(true),
            "can-release" => Fault::CanStuckDominant(false),
            "can-unplug" => Fault::CanUnplugged(true),
            "can-plug" => Fault::CanUnplugged(false),
            "can-flood" => {
                let id = parse_num(arg(), "identifier")?;
                let load = parse_num(arg(), "load")?;
                let len = arg().map(|l| parse_num(Some(l), "length")).transpose()?.unwrap_or(8);
                Fault::CanFlood { id, extended: id > 0x7FF, len, load }
            }
            "can-flood-stop" => Fault::CanFloodStop,
            "i2c-absent" => i2c(arg(), Some(I2cFault::Absent))?,
            "i2c-stuck" => i2c(arg(), Some(I2cFault::Stuck))?,
            "i2c-ok" => i2c(arg(), None)?,
            "i2c-intermittent" | "i2c-glitch" | "i2c-stretch" => {
                let addr = arg();
                let v = arg();
                let f = match name {
                    "i2c-intermittent" => I2cFault::Intermittent(parse_num(v, "probability")?),
                    "i2c-glitch" => I2cFault::Glitch(parse_num(v, "probability")?),
                    _ => I2cFault::Stretch(parse_duration(v.ok_or("missing time")?)?),
                };
                i2c(addr, Some(f))?
            }
            "adc-short" | "adc-open" | "adc-noise" | "adc-spikes" | "adc-ok" => {
                let input = parse_num(arg(), "ADC input")?;
                let fault = match name {
                    "adc-short" => Some(AnalogFault::Shorted(parse_num(arg(), "volts")?)),
                    "adc-open" => Some(AnalogFault::Open),
                    "adc-noise" => Some(AnalogFault::Noise(parse_num(arg(), "sigma")?)),
                    "adc-spikes" => {
                        let p = parse_num(arg(), "probability")?;
                        Some(AnalogFault::Spikes(p, parse_num(arg(), "volts")?))
                    }
                    _ => None,
                };
                Fault::Adc { input, fault }
            }
            "gpio" => {
                let pin = parse_num(arg(), "pin")?;
                let level = match arg() {
                    Some("high") => Some(true),
                    Some("low") => Some(false),
                    Some("release") => None,
                    other => return Err(format!("gpio level {other:?}: high, low or release")),
                };
                Fault::Gpio { pin, level }
            }
            "brownout" => Fault::Brownout { off: parse_duration(arg().ok_or("missing time")?)? },
            "hang" => Fault::Hang { core: parse_num(arg(), "core")?, interrupts_masked: false },
            "hang-masked" => Fault::Hang { core: parse_num(arg(), "core")?, interrupts_masked: true },
            "panic" => Fault::Panic,
            "watchdog" => Fault::WatchdogTimeout,
            "bridge-stop" => Fault::Bridge(false),
            "bridge-start" => Fault::Bridge(true),
            "kill" => Fault::KillSwitch(match arg() {
                Some("on") => Some(true),
                Some("off") => Some(false),
                Some("none") => None,
                other => return Err(format!("kill {other:?}: on, off or none")),
            }),
            "eth-unplug" => Fault::EthernetLink(false),
            "eth-plug" => Fault::EthernetLink(true),
            "packet-loss" => Fault::PacketLoss {
                to_board: parse_num(arg(), "probability")?,
                from_board: parse_num(arg(), "probability")?,
            },
            other => return Err(format!("unknown fault {other:?}")),
        })
    }
}

/// A stand-in for `titan_bridge` that faults can stop, start and use to forward the kill switch.
pub trait BridgeControl {
    fn stop(&self);
    /// Starts again, pinging from `at`.
    fn start(&self, at: Time);
    fn set_kill(&self, kill: Option<bool>);
}

impl BridgeControl for MockBridge {
    fn stop(&self) {
        MockBridge::stop(self)
    }
    fn start(&self, at: Time) {
        MockBridge::start(self, at)
    }
    fn set_kill(&self, kill: Option<bool>) {
        MockBridge::set_kill(self, kill)
    }
}

impl BridgeControl for PuddlesBridge {
    fn stop(&self) {
        PuddlesBridge::stop(self)
    }
    fn start(&self, at: Time) {
        PuddlesBridge::start(self, at)
    }
    fn set_kill(&self, kill: Option<bool>) {
        PuddlesBridge::set_kill(self, kill)
    }
}

/// Frames lost between a board's Ethernet cable and the host, each independently.
#[derive(Clone, Debug)]
pub struct PacketLoss {
    pub to_board: f64,
    pub from_board: f64,
    pub dropped_to_board: u64,
    pub dropped_from_board: u64,
    rng: Rng,
}

impl PacketLoss {
    pub fn new(to_board: f64, from_board: f64) -> Self {
        PacketLoss { to_board, from_board, dropped_to_board: 0, dropped_from_board: 0, rng: Rng::new(0x5EED_E7E7) }
    }

    pub fn drop_to_board(&mut self) -> bool {
        let d = self.rng.chance(self.to_board);
        self.dropped_to_board += d as u64;
        d
    }

    pub fn drop_from_board(&mut self) -> bool {
        let d = self.rng.chance(self.from_board);
        self.dropped_from_board += d as u64;
        d
    }
}

/// A board to apply a fault to.
pub struct FaultTarget<'a> {
    pub session: &'a mut Session,
    /// The bus the board is on and the indices of the board's CAN controllers among that bus's nodes (for a
    /// single board with its own bus, `0..n`).
    pub bus: Option<(&'a mut CanBus, Range<usize>)>,
    pub bridge: Option<&'a dyn BridgeControl>,
    /// In a robot, the board's power-on time and state ([`crate::robot::Robot::power_on`]): a brownout keeps it off
    /// until then instead of making its clock jump.
    pub power: Option<&'a mut (Time, bool)>,
}

impl FaultTarget<'_> {
    /// Applies `fault` now (at the board's current time).
    pub fn apply(self, fault: &Fault) -> Result<(), String> {
        let now = self.session.machine.now;
        log::info!("[{now}] fault: {fault:?}");
        match fault {
            Fault::CanErrorRate(_)
            | Fault::CanCorrupt(_)
            | Fault::CanStuckDominant(_)
            | Fault::CanUnplugged(_)
            | Fault::CanFlood { .. }
            | Fault::CanFloodStop => {
                let (bus, nodes) = self.bus.ok_or("the board is not on a CAN bus")?;
                apply_can(bus, nodes, fault, now)
            }
            Fault::I2c { address, fault } => set_i2c_fault(self.session, *address, *fault),
            Fault::Adc { input, fault } => set_adc_fault(self.session, *input, *fault),
            Fault::Gpio { pin, level } => {
                use sim_core::wire::{External, Level};
                let m = &mut self.session.machine;
                let ext = m.sys.gpio.external.get_mut(*pin as usize).ok_or(format!("no GPIO {pin}"))?;
                *ext = match level {
                    Some(true) => External::Driven(Level::High),
                    Some(false) => External::Driven(Level::Low),
                    None => External::Floating,
                };
                m.sys.update_pins(&mut *m.trace, now);
                Ok(())
            }
            Fault::Brownout { off } => {
                power_loss(self.session);
                match self.power {
                    Some(p) => *p = (now + *off, false),
                    None => {
                        let m = &mut self.session.machine;
                        let resets = m.resets;
                        m.power_on_at(now + *off);
                        m.resets = resets + 1;
                    }
                }
                Ok(())
            }
            Fault::Hang { core, interrupts_masked } => hang(self.session, *core, *interrupts_masked),
            Fault::Panic => panic(self.session),
            Fault::WatchdogTimeout => {
                self.session.machine.watchdog_reset(WatchdogReset::Timer);
                Ok(())
            }
            Fault::Bridge(running) => {
                let b = self.bridge.ok_or("no bridge to stop or start")?;
                if *running {
                    b.start(now)
                } else {
                    b.stop()
                }
                Ok(())
            }
            Fault::KillSwitch(kill) => {
                self.bridge.ok_or("no bridge to forward the kill switch")?.set_kill(*kill);
                Ok(())
            }
            Fault::EthernetLink(up) => {
                let wire = self.session.machine.sys.ethernet.as_ref().ok_or("the board has no Ethernet")?;
                wire.lock().unwrap().link_up = *up;
                Ok(())
            }
            Fault::PacketLoss { to_board, from_board } => {
                if self.session.machine.sys.ethernet.is_none() {
                    return Err("the board has no Ethernet".into());
                }
                self.session.eth_loss =
                    (*to_board > 0.0 || *from_board > 0.0).then(|| PacketLoss::new(*to_board, *from_board));
                Ok(())
            }
        }
    }
}

/// Board `board` of `robot` as a fault target: the bus it is on (with its CAN controllers' place among that bus's
/// nodes, after those of the boards before it), its power switch, and `bridge`.
pub fn robot_target<'a>(
    robot: &'a mut crate::robot::Robot,
    board: usize,
    bridge: Option<&'a dyn BridgeControl>,
) -> FaultTarget<'a> {
    let j = robot.board_bus[board];
    let first: usize =
        (0..board).filter(|&k| robot.board_bus[k] == j).map(|k| robot.boards[k].machine.can_nodes().len()).sum();
    let n = robot.boards[board].machine.can_nodes().len();
    let bus = j.map(|j| (&mut robot.buses[j].bus, first..first + n));
    FaultTarget { session: &mut robot.boards[board], bus, bridge, power: Some(&mut robot.power_on[board]) }
}

fn apply_can(bus: &mut CanBus, nodes: Range<usize>, fault: &Fault, now: Time) -> Result<(), String> {
    match fault {
        Fault::CanErrorRate(p) => bus.faults.error_rate = *p,
        Fault::CanCorrupt(n) => bus.faults.corrupt_next += n,
        Fault::CanStuckDominant(stuck) => bus.set_stuck_dominant(*stuck, now),
        Fault::CanUnplugged(unplugged) => {
            if nodes.is_empty() {
                return Err("the board has no CAN controller".into());
            }
            for n in nodes {
                bus.set_unplugged(n, *unplugged, now);
            }
        }
        Fault::CanFlood { id, extended, len, load } => {
            let data: Vec<u8> = (0..*len).map(|i| 0x55 ^ i as u8).collect();
            let frame = if bus.rates.data.is_some() {
                let mut data = data;
                data.resize(sim_devices::can::fd_dlc_to_len(sim_devices::can::fd_len_to_dlc(data.len())), 0);
                CanFrame::fd(*id, *extended, &data)
            } else {
                CanFrame::classic(*id, *extended, &data[..data.len().min(8)])
            };
            let rates = bus.rates;
            bus.faults.floods.push(Flood::with_load(frame, *load, rates, now));
        }
        Fault::CanFloodStop => bus.faults.floods.clear(),
        _ => unreachable!(),
    }
    Ok(())
}

/// Breaks (or repairs) the I2C device at `address`, wrapping it in a [`FaultyI2c`] the first time.
pub fn set_i2c_fault(s: &mut Session, address: u8, fault: Option<I2cFault>) -> Result<(), String> {
    let sys = &mut s.machine.sys;
    let slot: &mut Box<dyn sim_devices::i2c::I2cDevice> =
        if let Some(a) = sys.i2c_devices.iter_mut().find(|a| a.dev.responds_to(address)) {
            &mut a.dev
        } else if let Some(t) = sys
            .pin_devices
            .iter_mut()
            .filter_map(|d| d.as_any_mut().and_then(|a| a.downcast_mut::<I2cPinTarget>()))
            .find(|t| t.dev.responds_to(address))
        {
            &mut t.dev
        } else {
            return Err(format!("no I2C device at {address:#04x}"));
        };
    if slot.faulty().is_none() {
        let inner = slot.clone();
        *slot = Box::new(FaultyI2c::new(inner));
    }
    slot.faulty().unwrap().fault = fault;
    Ok(())
}

/// Breaks (or repairs) ADC input `input`, wrapping its source in a [`FaultyAnalog`] the first time. An input
/// with nothing connected gets a 0 V source to break.
pub fn set_adc_fault(s: &mut Session, input: usize, fault: Option<AnalogFault>) -> Result<(), String> {
    let sys = &mut s.machine.sys;
    if input > 3 {
        return Err(format!("ADC input {input} is not a pin (0-3)"));
    }
    let vref = sys.adc_vref;
    let slot = &mut sys.analog_inputs[input];
    let wrapped = slot.as_mut().and_then(|src| src.as_any_mut()).is_some_and(|a| a.is::<FaultyAnalog>());
    if !wrapped {
        let inner = slot.take().unwrap_or_else(|| Box::new(sim_core::analog::Signal::Const(0.0)));
        let mut f = FaultyAnalog::new(inner, 0xADC0 + input as u64);
        f.vref = vref;
        *slot = Some(Box::new(f));
    }
    let f = slot.as_mut().unwrap().as_any_mut().unwrap().downcast_mut::<FaultyAnalog>().unwrap();
    f.fault = fault;
    Ok(())
}

/// What happens to the chips on the board's 3.3 V rail when it drops out: they reset.
pub fn power_loss(s: &mut Session) {
    for a in s.machine.sys.spi_devices.iter_mut() {
        if let Some(any) = a.dev.as_any_mut() {
            if let Some(m) = any.downcast_mut::<Mcp251xfd>() {
                m.power_on_reset();
            } else if let Some(w) = any.downcast_mut::<W5500>() {
                w.power_on_reset();
            }
        }
    }
}

/// An address in the image's code holding `b .` (a branch to itself): the body of cortex-m-rt's `DefaultHandler_`
/// in embassy images, or any other such loop the image has.
pub fn hang_address(image: &Image) -> Option<u32> {
    let halfword = |addr: u32| -> Option<u16> {
        image.segments.iter().find_map(|s| {
            let o = addr.checked_sub(s.addr)? as usize;
            (o + 1 < s.data.len()).then(|| u16::from_le_bytes([s.data[o], s.data[o + 1]]))
        })
    };
    let preferred = ["DefaultHandler_", "HardFault_", "isr_hardfault", "__unhandled_user_irq"];
    let mut syms: Vec<&sim_core::image::Symbol> =
        preferred.iter().filter_map(|n| image.symbol(n)).filter(|s| s.is_function).collect();
    syms.extend(image.symbols.iter().filter(|s| s.is_function));
    syms.into_iter().find_map(|s| {
        let start = s.addr & !1;
        (0..s.size.max(2) / 2).map(|i| start + 2 * i).find(|&a| halfword(a) == Some(0xE7FE))
    })
}

fn hang(s: &mut Session, core: usize, interrupts_masked: bool) -> Result<(), String> {
    if core > 1 || !s.machine.core_enabled[core] {
        return Err(format!("core {core} is not running"));
    }
    let image = s.machine.image.clone().ok_or("no image loaded")?;
    let addr = match hang_address(&image) {
        Some(a) => a,
        None => {
            // No such loop in the image (pico-sdk images have none): put one where the firmware cannot see it, at
            // the end of the XIP cache's SRAM, which an image running from flash with the cache on never uses
            let a = rp2040::memmap::XIP_SRAM_BASE + rp2040::memmap::XIP_SRAM_SIZE as u32 - 2;
            s.machine.sys.poke(a, sim_core::Size::Half, 0xE7FE);
            a
        }
    };
    s.machine.inject_call(core, addr | 1, &[]);
    if interrupts_masked {
        s.machine.cores[core].set_primask(true);
    }
    Ok(())
}

fn panic(s: &mut Session) -> Result<(), String> {
    let image = s.machine.image.clone().ok_or("no image loaded")?;
    // `__defmt_default_panic` is the target of `defmt::panic!` in embassy images
    let sym = image.symbol("__defmt_default_panic").ok_or("no panic entry point found in the image")?;
    s.machine.inject_call(0, sym.addr, &[]);
    Ok(())
}

/// The error counters of each CAN controller on the board.
pub fn can_error_counters(s: &mut Session) -> Vec<ErrorCounters> {
    s.machine
        .sys
        .spi_devices
        .iter_mut()
        .filter_map(|a| a.dev.as_any_mut().and_then(|d| d.downcast_mut::<Mcp251xfd>()).map(|m| m.error_counters()))
        .collect()
}

/// The board's CAN controller (the first one).
pub fn can_controller(s: &mut Session) -> Option<&mut Mcp251xfd> {
    s.machine.sys.spi_devices.iter_mut().find_map(|a| a.dev.as_any_mut().and_then(|d| d.downcast_mut::<Mcp251xfd>()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_fault_specs() {
        let p = |s: &str| s.parse::<Fault>();
        assert_eq!(p("can-stuck"), Ok(Fault::CanStuckDominant(true)));
        assert_eq!(p("can-error-rate:0.25"), Ok(Fault::CanErrorRate(0.25)));
        assert_eq!(p("can-flood:0x10:0.5"), Ok(Fault::CanFlood { id: 0x10, extended: false, len: 8, load: 0.5 }));
        assert_eq!(p("i2c-absent:0x76"), Ok(Fault::I2c { address: 0x76, fault: Some(I2cFault::Absent) }));
        assert_eq!(
            p("i2c-stretch:118:30ms"),
            Ok(Fault::I2c { address: 0x76, fault: Some(I2cFault::Stretch(Time::from_ms(30))) })
        );
        assert_eq!(p("adc-short:0:0"), Ok(Fault::Adc { input: 0, fault: Some(AnalogFault::Shorted(0.0)) }));
        assert_eq!(p("gpio:1:low"), Ok(Fault::Gpio { pin: 1, level: Some(false) }));
        assert_eq!(p("brownout:50ms"), Ok(Fault::Brownout { off: Time::from_ms(50) }));
        assert_eq!(p("hang-masked:0"), Ok(Fault::Hang { core: 0, interrupts_masked: true }));
        assert_eq!(p("kill:none"), Ok(Fault::KillSwitch(None)));
        assert_eq!(p("packet-loss:0.1:0"), Ok(Fault::PacketLoss { to_board: 0.1, from_board: 0.0 }));
        assert!(p("i2c-absent").is_err());
        assert!(p("gpio:1:sideways").is_err());
        assert!(p("teleport").is_err());
    }

    #[test]
    fn packet_loss_drops_about_that_share() {
        let mut l = PacketLoss::new(0.3, 0.0);
        let dropped = (0..1000).filter(|_| l.drop_to_board()).count();
        assert!((250..=350).contains(&dropped), "{dropped}");
        assert!((0..100).all(|_| !l.drop_from_board()));
        assert_eq!(l.dropped_to_board, dropped as u64);
    }

    #[test]
    fn finds_a_busy_loop_in_code() {
        let mut image = Image { segments: Vec::new(), entry: None, symbols: Vec::new(), elf: None };
        image.segments.push(sim_core::image::Segment {
            addr: 0x1000_0000,
            data: vec![0x80, 0xB5, 0x00, 0xAF, 0xFE, 0xE7, 0x00, 0xBF],
        });
        image.symbols.push(sim_core::image::Symbol {
            name: "DefaultHandler_".into(),
            addr: 0x1000_0001,
            size: 6,
            is_function: true,
        });
        assert_eq!(hang_address(&image), Some(0x1000_0004));
    }
}
