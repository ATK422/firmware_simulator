//! Several boards in one process: robots on their CAN buses, optionally in a shared acoustic medium.
//!
//! Every board runs to the same virtual time in short slices; after each slice every bus arbitrates and delivers
//! everything its boards (and host endpoints) sent, the boards' devices update their pins, and each board's
//! progress is reported to the acoustic medium. The slice bounds how late a frame can reach another board (and the
//! shortest acoustic path that is modelled exactly, 15 cm of water at 100 µs), and keeps runs deterministic.
//!
//! A robot file describes the lot:
//!
//! ```toml
//! [environment]                       # optional: the water the boards' transducers are in
//! file = "../environments/pool.toml"
//!
//! [[bus]]
//! name = "talos"                      # boards on the same bus share it
//! rates = "internal"                  # CAN FD 1/5 Mbit/s; "external" is classic 250 kbit/s
//!
//! [[board]]
//! name = "ivc"
//! board = "../boards/talos/ivc_board.toml"
//! elf = "../external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/ivc_board2"
//! bus = "talos"
//! position_m = [2.0, 6.0, 1.0]        # or trajectory = { start_m = [...], velocity_mps = [...] }
//!                                     # or trajectory = [[0.0, [x, y, z]], [10.0, [x, y, z]]]
//! # power_on_ms = 37                   # switched on later than time 0
//! # gpio = [{ pin = 6, level = "high" }]   # straps, replacing the board file's for those pins
//! # [board.transducer]                # replaces the board file's transducer
//! ```
//!
//! Paths are relative to the robot file. Several vehicles go in one file as several buses.
//!
//! A `[[vehicle]]` section (see [`crate::vehicle`]) adds a simulated vehicle that the boards' thrusters move, with
//! the depth sensors and transducers of the boards on it following its motion.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use rp2040::{Machine, MachineConfig, StopReason};
use serde::Deserialize;
use sim_acoustics::medium::{Medium, SharedMedium, Trajectory};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::{CanBus, CanEndpoint, CanNode};

use crate::board::{Board, GpioEntry, Inputs, TransducerEntry, bus_rates};
use crate::session::{CanCallback, Session};
use crate::vehicle::{VehicleSim, VehicleSpec};

/// A CAN bus of the robot with the host endpoints on it.
pub struct RobotBus {
    pub bus: CanBus,
    pub endpoints: Vec<Box<dyn CanEndpoint + Send>>,
    /// Called with every frame that went over the bus.
    pub on_can: Option<CanCallback>,
}

impl RobotBus {
    pub fn new(bus: CanBus) -> Self {
        RobotBus { bus, endpoints: Vec::new(), on_can: None }
    }
}

pub struct Robot {
    pub boards: Vec<Session>,
    /// Board names (from the robot file, or given to [`Robot::add`]).
    pub names: Vec<String>,
    /// Each board's physical inputs (sensor values), from its board file.
    pub inputs: Vec<Inputs>,
    /// The bus each board is on.
    pub board_bus: Vec<Option<usize>>,
    pub buses: Vec<RobotBus>,
    pub bus_names: Vec<String>,
    /// The water the boards' transducers are in.
    pub medium: Option<SharedMedium>,
    pub slice: Time,
    pub now: Time,
    /// Pace virtual time to the wall clock.
    pub realtime: bool,
    /// When each board is switched on (it stays unpowered before), and whether it has been.
    pub power_on: Vec<(Time, bool)>,
    /// Simulated vehicles moved by the boards' thrusters, stepped after every slice.
    pub vehicles: Vec<VehicleSim>,
}

impl Default for Robot {
    fn default() -> Self {
        Robot {
            boards: Vec::new(),
            names: Vec::new(),
            inputs: Vec::new(),
            board_bus: Vec::new(),
            buses: Vec::new(),
            bus_names: Vec::new(),
            medium: None,
            slice: Time::from_us(100),
            now: Time::ZERO,
            realtime: false,
            power_on: Vec::new(),
            vehicles: Vec::new(),
        }
    }
}

impl Robot {
    /// A robot with one bus (index 0) and no boards yet.
    pub fn new(bus: CanBus) -> Robot {
        let mut r = Robot::default();
        r.bus_names.push(bus.name.clone());
        r.buses.push(RobotBus::new(bus));
        r
    }

    /// Adds a board on bus `bus`; returns its index (the owner id in the medium).
    pub fn add(&mut self, name: &str, session: Session, bus: Option<usize>) -> usize {
        self.boards.push(session);
        self.names.push(name.to_string());
        self.inputs.push(Inputs::default());
        self.board_bus.push(bus);
        self.power_on.push((Time::ZERO, true));
        self.boards.len() - 1
    }

    /// The board called `name`.
    pub fn board(&mut self, name: &str) -> Option<&mut Session> {
        let i = self.names.iter().position(|n| n == name)?;
        Some(&mut self.boards[i])
    }

    pub fn index(&self, name: &str) -> Option<usize> {
        self.names.iter().position(|n| n == name)
    }

    /// The vehicle called `name`.
    pub fn vehicle(&mut self, name: &str) -> Option<&mut VehicleSim> {
        self.vehicles.iter_mut().find(|v| v.vehicle.name == name)
    }

    /// The bus called `name`.
    pub fn bus(&mut self, name: &str) -> Option<&mut RobotBus> {
        let i = self.bus_names.iter().position(|n| n == name)?;
        Some(&mut self.buses[i])
    }

    /// Runs every board until `end`. Returns the first board that stopped for another reason, with the reason.
    pub fn run_until(&mut self, end: Time) -> Option<(usize, StopReason)> {
        let wall = std::time::Instant::now();
        let start = self.now;
        // The sensor inputs may have been set by a snapshot of this robot since (they are shared)
        for v in &self.vehicles {
            v.update_outputs();
        }
        while self.now < end {
            let t = (self.now + self.slice).min(end);
            for (i, b) in self.boards.iter_mut().enumerate() {
                let (on_at, on) = &mut self.power_on[i];
                if !*on {
                    if t <= *on_at {
                        continue;
                    }
                    b.machine.power_on_at(*on_at);
                    *on = true;
                }
                match b.machine.run_until(t) {
                    StopReason::TimeLimit => {}
                    other => return Some((i, other)),
                }
                if let Some(m) = &self.medium {
                    m.lock().unwrap().progress(i, t);
                }
            }
            for (j, rb) in self.buses.iter_mut().enumerate() {
                let mut nodes: Vec<&mut dyn CanNode> = Vec::new();
                for (b, on) in self.boards.iter_mut().zip(&self.board_bus) {
                    if *on == Some(j) {
                        nodes.extend(b.machine.can_nodes());
                    }
                }
                let mut eps: Vec<&mut dyn CanEndpoint> =
                    rb.endpoints.iter_mut().map(|e| &mut **e as &mut dyn CanEndpoint).collect();
                let before = rb.bus.record.as_ref().map_or(0, |r| r.len());
                rb.bus.step(&mut nodes, &mut eps, t);
                if let (Some(rec), Some(hook)) = (&mut rb.bus.record, &mut rb.on_can) {
                    for r in &rec[before..] {
                        hook(r);
                    }
                    // The callback has seen the history; keep memory bounded on long runs (`--candump-log`)
                    if rec.len() > 100_000 {
                        rec.drain(..50_000);
                    }
                }
            }
            for b in self.boards.iter_mut() {
                b.step_ethernet();
                b.machine.update_devices();
                b.poll_rtt();
                b.poll_uarts();
            }
            self.now = t;
            for v in &mut self.vehicles {
                v.advance(t, &self.boards);
            }
            if self.realtime {
                let ahead = (self.now - start).as_secs_f64() - wall.elapsed().as_secs_f64();
                if ahead > 0.002 {
                    std::thread::sleep(std::time::Duration::from_secs_f64(ahead));
                }
            }
        }
        None
    }

    /// Puts board `i`'s transducer (projector and hydrophone) on `trajectory`, e.g. at another range between runs
    /// started from a snapshot. Times in the trajectory are virtual seconds since power-on.
    pub fn place(&mut self, i: usize, trajectory: Trajectory) {
        if let Some(m) = &self.medium {
            for p in m.lock().unwrap().projectors.iter_mut().filter(|p| p.owner == i) {
                p.trajectory = trajectory.clone();
            }
        }
        for h in crate::acoustic::hydrophones(&mut self.boards[i].machine) {
            h.set_trajectory(trajectory.clone());
        }
    }

    /// Sets the ambient noise every board's hydrophone hears to white noise of `level_db` re 1 µPa²/Hz.
    pub fn set_noise_db(&mut self, level_db: f64) {
        for b in &mut self.boards {
            for h in crate::acoustic::hydrophones(&mut b.machine) {
                h.set_noise_db(level_db);
            }
        }
    }

    /// A deep copy of the whole robot: every machine, device, bus and the acoustic medium, re-linked so the copy's
    /// hydrophones and projectors use the copy's medium. Host endpoints are not copied (they are often handles to
    /// shared state, like a mock bridge); give the copy its own. Physical inputs stay shared: setting one affects
    /// the original and the copy, so set them after restoring, before running. That includes the pressure inputs
    /// a vehicle drives: copies of a robot with a vehicle can take turns (each run starts by writing its own
    /// vehicle's pressures), but must not run at the same time.
    pub fn snapshot(&self) -> Robot {
        let medium = self.medium.as_ref().map(|m| Arc::new(std::sync::Mutex::new(m.lock().unwrap().clone())));
        let mut boards: Vec<Session> = self.boards.iter().map(Session::snapshot).collect();
        if let (Some(old), Some(new)) = (&self.medium, &medium) {
            for b in &mut boards {
                crate::acoustic::relink(&mut b.machine, old, new);
            }
        }
        // The copy's transducers follow the copy's vehicles
        let mut vehicles = Vec::new();
        for v in &self.vehicles {
            let (copy, pairs) = v.deep_copy();
            for (old, new) in pairs {
                let (old, new) = (Trajectory::Live(old), Trajectory::Live(new));
                if let Some(m) = &medium {
                    for p in m.lock().unwrap().projectors.iter_mut().filter(|p| p.trajectory == old) {
                        p.trajectory = new.clone();
                    }
                }
                for b in &mut boards {
                    for h in crate::acoustic::hydrophones(&mut b.machine).filter(|h| h.trajectory == old) {
                        h.set_trajectory(new.clone());
                    }
                }
            }
            vehicles.push(copy);
        }
        Robot {
            boards,
            names: self.names.clone(),
            inputs: self.inputs.clone(),
            board_bus: self.board_bus.clone(),
            buses: self
                .buses
                .iter()
                .map(|b| RobotBus { bus: b.bus.clone(), endpoints: Vec::new(), on_can: None })
                .collect(),
            bus_names: self.bus_names.clone(),
            medium,
            slice: self.slice,
            now: self.now,
            realtime: self.realtime,
            power_on: self.power_on.clone(),
            vehicles,
        }
    }

    /// Loads a robot file and boots its boards.
    pub fn load(path: &Path) -> Result<Robot, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let spec: RobotSpec = toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        let dir = path.parent().unwrap_or(Path::new("."));
        Robot::from_spec(&spec, dir)
    }

    /// Builds a robot from a parsed robot file; relative paths are taken from `dir`.
    pub fn from_spec(spec: &RobotSpec, dir: &Path) -> Result<Robot, String> {
        let mut robot = Robot::default();
        if let Some(e) = &spec.environment {
            let mut env = sim_acoustics::Environment::load(&dir.join(&e.file))?;
            if let Some(seed) = e.seed {
                env.seed = seed;
            }
            robot.medium = Some(Medium::shared(Arc::new(env)));
        }
        for b in &spec.bus {
            let rates = match &b.rates {
                Some(r) => bus_rates(r).ok_or_else(|| format!("bus {}: unknown rates {r:?}", b.name))?,
                None => bus_rates("internal").unwrap(),
            };
            let mut bus = CanBus::new(&b.name, rates);
            if b.record {
                bus.record = Some(Vec::new());
            }
            robot.bus_names.push(b.name.clone());
            robot.buses.push(RobotBus::new(bus));
        }
        for v in &spec.vehicle {
            if robot.vehicles.iter().any(|o| o.vehicle.name == v.name) {
                return Err(format!("two vehicles called {}", v.name));
            }
            let env = robot.medium.as_ref().map(|m| m.lock().unwrap().env.clone());
            robot.vehicles.push(VehicleSim::new(v, dir, env.as_deref())?);
        }
        for b in &spec.board {
            let board_file = dir.join(&b.board);
            let mut board = Board::load(&board_file)?;
            if let Some(t) = &b.transducer {
                board.transducer = Some(t.clone());
            }
            // Robot-level straps replace the board file's entries for the same pins
            board.gpio.retain(|g| !b.gpio.iter().any(|o| o.pin == g.pin));
            board.gpio.extend(b.gpio.iter().cloned());
            let elf = crate::resolve_image(&dir.join(&b.elf));
            let image = Image::load(&elf).map_err(|e| format!("{}: {e}", elf.display()))?;
            let mut m = Machine::new(MachineConfig {
                bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
                flash_size: board.flash_size.unwrap_or(2 << 20),
                ..Default::default()
            });
            m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
            let inputs = board.apply(&mut m)?;
            let bus = match &b.bus {
                Some(name) => Some(
                    robot
                        .bus_names
                        .iter()
                        .position(|n| n == name)
                        .ok_or_else(|| format!("{}: no bus {name}", b.name))?,
                ),
                None if robot.buses.len() == 1 && board.can.is_some() => Some(0),
                None => None,
            };
            // A bus with no rates given takes those of its first board
            if let Some(j) = bus
                && let Some(want) = board.can_rates()
                && spec.bus.get(j).is_some_and(|b| b.rates.is_none())
                && !robot.board_bus.contains(&Some(j))
            {
                robot.buses[j].bus.rates = want;
            }
            let owner = robot.boards.len();
            if let Some(medium) = &robot.medium
                && board.transducer.is_some()
            {
                let carried =
                    robot.vehicles.iter_mut().zip(&spec.vehicle).find_map(|(v, s)| v.carries(s, &b.name, owner));
                let trajectory = match (carried, &b.trajectory, b.position_m) {
                    (Some(_), Some(_), _) | (Some(_), _, Some(_)) => {
                        return Err(format!("{}: on a vehicle, so it takes no position_m or trajectory", b.name));
                    }
                    (Some(t), None, None) => t,
                    (None, Some(t), _) => t.clone(),
                    (None, None, Some(p)) => Trajectory::Fixed(p),
                    (None, None, None) => {
                        return Err(format!("{}: needs position_m, a trajectory or a vehicle in the water", b.name));
                    }
                };
                board.place_in_medium(&mut m, medium, owner, trajectory)?;
            }
            m.load(&image)?;
            if b.rom_boot {
                m.rom_boot()?;
            } else {
                m.direct_boot()?;
            }
            let session = Session::new(m, &image);
            robot.add(&b.name, session, bus);
            robot.inputs[owner] = inputs;
            if let Some(ms) = b.power_on_ms
                && ms > 0.0
            {
                robot.power_on[owner] = (Time::from_secs_f64(ms / 1000.0), false);
            }
        }
        for (v, s) in robot.vehicles.iter_mut().zip(&spec.vehicle) {
            v.wire(s, &robot.names, &robot.boards, &robot.inputs)?;
        }
        Ok(robot)
    }
}

/// A robot file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotSpec {
    pub environment: Option<EnvironmentRef>,
    #[serde(default)]
    pub bus: Vec<BusSpec>,
    #[serde(default)]
    pub board: Vec<BoardSpec>,
    /// Simulated vehicles the boards are on.
    #[serde(default)]
    pub vehicle: Vec<VehicleSpec>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EnvironmentRef {
    pub file: PathBuf,
    /// Replaces the environment's noise seed.
    pub seed: Option<u64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusSpec {
    pub name: String,
    /// `internal` or `external`; by default the rates of the first board on it.
    pub rates: Option<String>,
    /// Keep a record of every frame (tests, `--candump`).
    #[serde(default)]
    pub record: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardSpec {
    pub name: String,
    /// Board file.
    pub board: PathBuf,
    /// Firmware image (ELF or UF2).
    pub elf: PathBuf,
    pub bus: Option<String>,
    #[serde(default)]
    pub rom_boot: bool,
    /// Switched on this long after the others (boards never power up in perfect lockstep).
    pub power_on_ms: Option<f64>,
    pub position_m: Option<sim_acoustics::geometry::Vec3>,
    pub trajectory: Option<Trajectory>,
    pub transducer: Option<TransducerEntry>,
    /// Pin straps for this board (e.g. a board ID pin), as `[[board.gpio]]` entries of a board file.
    #[serde(default)]
    pub gpio: Vec<GpioEntry>,
}
