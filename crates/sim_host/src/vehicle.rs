//! Vehicles in a robot: the boards' thrusters move a simulated vehicle ([`sim_physics`]), and the sensors and
//! transducers on it follow.
//!
//! After every slice of the robot's run the vehicle is stepped up to the robot's time, with each thruster's
//! propeller speed read from the motor model of the DShot ESC it is wired to (`[[dshot_esc]]` of a board file).
//! Then every pressure sensor wired to the vehicle gets the absolute water pressure at its mounting point
//! (`surface + rho g depth`, the air pressure above the surface) as its board input, and every transducer on the
//! vehicle is moved to its mounting point in the acoustic medium (a [`LivePath`] its hydrophone and projector
//! follow). The vehicle steps at 1 ms on the robot's virtual clock, so it is as deterministic as the boards.
//!
//! In a robot file:
//!
//! ```toml
//! [[vehicle]]
//! name = "talos"
//! file = "../vehicles/talos.toml"     # body and thruster geometry
//! position_m = [5.0, 6.0, 0.0]        # centre of gravity: north, east, depth
//! rpy_deg = [0.0, 0.0, 90.0]          # roll, pitch, yaw
//! # floating = true                   # start at the depth where it floats (instead of position_m's depth)
//! # water = { density_kg_m3 = 1025.0, current_mps = [0.1, 0.0, 0.0] }
//! # log_ms = 10                        # keep a state log (CSV with `--vehicle-csv`)
//! thrusters = [
//!     { thruster = "heave_fl", board = "esc0", esc = "esc1" },   # a thruster of the vehicle file, the board
//!     # ...                                                      # and the ESC (by name in the board file)
//! ]
//!
//! [[vehicle.sensor]]                  # a pressure input of a board, at a point of the hull
//! board = "camera_cage"
//! input = "depth.pressure_pa"
//! mount_m = [0.35, 0.0, -0.12]        # body frame from the centre of gravity: forward, starboard, down
//!
//! [[vehicle.transducer]]              # a board's transducer (in the robot's acoustic environment)
//! board = "ivc"
//! mount_m = [-0.2, 0.0, 0.15]
//! ```

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use serde::Deserialize;
use sim_acoustics::medium::{LivePath, Trajectory};
use sim_core::time::Time;
use sim_devices::dshot::Esc;
use sim_physics::log::StateLog;
use sim_physics::{State, Vec3, Vehicle, VehicleFile, Water};

use crate::Session;
use crate::board::Inputs;

/// A `[[vehicle]]` of a robot file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleSpec {
    pub name: String,
    /// Vehicle file (`vehicles/*.toml`), relative to the robot file.
    pub file: PathBuf,
    /// Where its centre of gravity starts (north, east, depth), m.
    #[serde(default)]
    pub position_m: Vec3,
    /// Starting roll, pitch, yaw, degrees.
    #[serde(default)]
    pub rpy_deg: Vec3,
    /// Start at the depth where it floats at the surface (a positively buoyant vehicle), replacing `position_m`'s.
    #[serde(default)]
    pub floating: bool,
    #[serde(default)]
    pub water: Water,
    /// Each thruster's ESC.
    #[serde(default)]
    pub thrusters: Vec<ThrusterWiring>,
    #[serde(default)]
    pub sensor: Vec<SensorMount>,
    #[serde(default)]
    pub transducer: Vec<TransducerMount>,
    /// Keep a log of the vehicle's state every this many milliseconds.
    pub log_ms: Option<f64>,
}

/// Which ESC spins a thruster.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThrusterWiring {
    /// The thruster's name in the vehicle file.
    pub thruster: String,
    /// The board (by its name in the robot file) and the ESC (by its name in the board file, e.g. `esc1`, `t3`).
    pub board: String,
    pub esc: String,
}

/// A pressure sensor input on the hull.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorMount {
    pub board: String,
    /// The board's input that takes the absolute pressure in pascals (an MS5837's `<name>.pressure_pa`).
    pub input: String,
    /// Where the port is, body frame from the centre of gravity, m.
    pub mount_m: Vec3,
}

/// A board's transducer on the hull.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TransducerMount {
    pub board: String,
    pub mount_m: Vec3,
}

/// A vehicle wired to the boards of a robot.
#[derive(Clone)]
pub struct VehicleSim {
    pub vehicle: Vehicle,
    /// The state log (`log_ms`, or [`VehicleSim::keep_log`]).
    pub log: Option<StateLog>,
    /// Per thruster of the vehicle: the board index and the index of its ESC among the board's pin devices.
    thrusters: Vec<Option<(usize, usize)>>,
    /// Pressure inputs and where they are on the hull.
    sensors: Vec<(Arc<Mutex<f64>>, Vec3)>,
    /// Transducers: board index, mount, and the path its hydrophone and projector follow.
    paths: Vec<(usize, Vec3, LivePath)>,
}

impl VehicleSim {
    /// Loads the vehicle file of `spec` (relative to `dir`) and puts the vehicle at its starting pose. Its wiring
    /// comes later ([`VehicleSim::wire`]), once the boards exist; the paths of the transducers it carries come
    /// first ([`VehicleSim::carries`]), since the boards need them to be placed in the water.
    ///
    /// In an acoustic environment with a bottom (a tank, a lake), the vehicle rests on that bottom unless `water`
    /// says otherwise.
    pub fn new(spec: &VehicleSpec, dir: &Path, env: Option<&sim_acoustics::Environment>) -> Result<VehicleSim, String> {
        let file = VehicleFile::load(&dir.join(&spec.file))?;
        let rpy = spec.rpy_deg.map(f64::to_radians);
        let mut state = State::at_rest(spec.position_m, rpy);
        let mut water = spec.water.clone();
        if water.bottom_m.is_none() {
            water.bottom_m = match env.map(|e| &e.boundaries) {
                Some(sim_acoustics::geometry::Boundaries::Tank { size }) => Some(size[2]),
                Some(sim_acoustics::geometry::Boundaries::Lake { depth }) => Some(*depth),
                _ => None,
            };
        }
        let mut vehicle = Vehicle::new(&file, water, state);
        if spec.floating {
            let depth = vehicle
                .floating_depth()
                .ok_or_else(|| format!("vehicle {}: not positively buoyant, so it doesn't float", spec.name))?;
            state.position[2] = depth;
            vehicle.state = state;
        }
        vehicle.name = spec.name.clone();
        let log = spec.log_ms.map(|ms| StateLog::new(ms / 1000.0, &vehicle));
        let mut sim = VehicleSim { vehicle, log, thrusters: Vec::new(), sensors: Vec::new(), paths: Vec::new() };
        sim.record();
        Ok(sim)
    }

    /// The trajectory of `board`'s transducer if this vehicle carries it (`[[vehicle.transducer]]`), board index
    /// `index` in the robot.
    pub fn carries(&mut self, spec: &VehicleSpec, board: &str, index: usize) -> Option<Trajectory> {
        let m = spec.transducer.iter().find(|t| t.board == board)?;
        let path = LivePath::new(self.vehicle.point(m.mount_m));
        self.paths.push((index, m.mount_m, path.clone()));
        Some(Trajectory::Live(path))
    }

    /// Wires the thrusters and sensors to the robot's boards, by name.
    pub fn wire(
        &mut self,
        spec: &VehicleSpec,
        names: &[String],
        boards: &[Session],
        inputs: &[Inputs],
    ) -> Result<(), String> {
        let ctx = |what: String| format!("vehicle {}: {what}", spec.name);
        let board = |name: &str| names.iter().position(|n| n == name).ok_or_else(|| ctx(format!("no board {name:?}")));
        self.thrusters = vec![None; self.vehicle.thrusters.len()];
        for w in &spec.thrusters {
            let t = self
                .vehicle
                .thrusters
                .iter()
                .position(|t| t.name == w.thruster)
                .ok_or_else(|| ctx(format!("no thruster {:?} in the vehicle file", w.thruster)))?;
            let b = board(&w.board)?;
            let k = boards[b]
                .machine
                .sys
                .pin_devices
                .iter()
                .position(|d| d.as_any().downcast_ref::<Esc>().is_some_and(|e| e.name == w.esc))
                .ok_or_else(|| ctx(format!("board {} has no ESC {:?}", w.board, w.esc)))?;
            if self.thrusters[t].is_some() {
                return Err(ctx(format!("thruster {} is wired twice", w.thruster)));
            }
            self.thrusters[t] = Some((b, k));
        }
        for s in &spec.sensor {
            let b = board(&s.board)?;
            let h = inputs[b]
                .0
                .get(&s.input)
                .ok_or_else(|| ctx(format!("board {} has no input {:?}", s.board, s.input)))?;
            self.sensors.push((h.clone(), s.mount_m));
        }
        for t in &spec.transducer {
            if !self.paths.iter().any(|p| names[p.0] == t.board) {
                return Err(ctx(format!("board {} has no transducer in an acoustic environment", t.board)));
            }
        }
        self.update_outputs();
        Ok(())
    }

    /// Starts keeping a state log every `interval` (unless one is kept already).
    pub fn keep_log(&mut self, interval: Time) {
        if self.log.is_none() {
            self.log = Some(StateLog::new(interval.as_secs_f64(), &self.vehicle));
            self.record();
        }
    }

    /// Each thruster's propeller speed (signed mechanical RPM) at `now`; 0 for one that isn't wired.
    pub fn rpms(&self, boards: &[Session], now: Time) -> Vec<f64> {
        self.thrusters
            .iter()
            .map(|w| {
                w.and_then(|(b, k)| boards[b].machine.sys.pin_devices[k].as_any().downcast_ref::<Esc>())
                    .map_or(0.0, |e| e.rpm_at(now))
            })
            .collect()
    }

    /// Steps the vehicle up to `t` with the ESCs' current speeds, then updates its sensors and transducers.
    pub fn advance(&mut self, t: Time, boards: &[Session]) {
        let end = t.as_secs_f64();
        while self.vehicle.time + self.vehicle.dt <= end + 1e-9 {
            let rpm = self.rpms(boards, Time::from_secs_f64(self.vehicle.time));
            self.vehicle.step(&rpm);
            for (_, mount, path) in &self.paths {
                path.push(self.vehicle.time, self.vehicle.point(*mount));
            }
            self.record();
        }
        self.update_outputs();
    }

    fn record(&mut self) {
        if let Some(l) = &mut self.log {
            l.record(&self.vehicle);
        }
    }

    /// Whether the vehicle drives any board inputs (which robot snapshots share, see [`crate::robot::Robot::snapshot`]).
    pub fn has_sensors(&self) -> bool {
        !self.sensors.is_empty()
    }

    /// Writes the pressure at each sensor's port to its input.
    pub fn update_outputs(&self) {
        for (h, mount) in &self.sensors {
            *h.lock().unwrap() = self.vehicle.pressure_at(*mount);
        }
    }

    /// The paths of the transducers on the vehicle, with their boards.
    pub fn paths(&self) -> impl Iterator<Item = (usize, &LivePath)> {
        self.paths.iter().map(|(b, _, p)| (*b, p))
    }

    /// A copy whose transducer paths are recorded independently of this one's; returns it with the (old, new)
    /// path pairs, for re-linking a snapshot's hydrophones and projectors.
    pub fn deep_copy(&self) -> (VehicleSim, Vec<(LivePath, LivePath)>) {
        let mut v = self.clone();
        let mut pairs = Vec::new();
        for (_, _, p) in &mut v.paths {
            let new = p.deep_copy();
            pairs.push((p.clone(), new.clone()));
            *p = new;
        }
        (v, pairs)
    }
}
