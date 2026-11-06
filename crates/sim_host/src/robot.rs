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
//! # usb_cdc = "pty:/tmp/ivc-log"      # USB plugged into a host that opens CDC port 0 (stdout, pty, pty:PATH)
//! # usb_cdc2 = "pty:/tmp/ivc-ros"     # and port 1 (micro-ROS over titan_dual_stdio_usb); `usb = true` just plugs in
//! # [board.transducer]                # replaces the board file's transducer
//! ```
//!
//! Paths are relative to the robot file; `elf`, `board` and pty paths may start with `~` and contain `$NAME` or
//! `${NAME}` environment variables ([`crate::expand_path`]). Several vehicles go in one file as several buses.
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
use crate::usb::CdcSink;
use crate::vehicle::{VehicleSim, VehicleSpec};

/// Boards in an acoustic medium run on threads in pieces of at most this, each checked for how far apart the water
/// lets them run through it (see [`Robot::run_until`]).
pub const MEDIUM_PIECE: Time = Time::from_secs(1);

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
    /// Each board's named pins (its board file's `[pins]` table).
    pub pins: Vec<std::collections::BTreeMap<String, u8>>,
    /// Each board's CANmore client ID (its board file's `[can] client_id`).
    pub client_ids: Vec<Option<u8>>,
    /// Each board's nano-ROS topic table (the robot file's `ros_toml`: its firmware's `ros.toml`), for a nano-ROS
    /// mock agent ([`crate::nanoros`]).
    pub ros_tomls: Vec<Option<PathBuf>>,
    /// Each board's firmware image file name.
    pub images: Vec<String>,
    /// The bus each board is on.
    pub board_bus: Vec<Option<usize>>,
    /// Each board's USB port: whether it is plugged into a host and where its CDC ports go (see
    /// [`Robot::set_usb_cdc`]).
    pub usb: Vec<UsbPorts>,
    pub buses: Vec<RobotBus>,
    pub bus_names: Vec<String>,
    /// The water the boards' transducers are in.
    pub medium: Option<SharedMedium>,
    pub slice: Time,
    /// Threads the boards may run on (1, the default: the calling thread only; 0: one per core). Boards in an
    /// acoustic medium run on them as far apart as the water lets them (see [`Robot::run_until`]).
    pub threads: usize,
    /// On threads, let the boards run ahead of each other between the slices where they can see each other (on
    /// by default; off to check that it changes nothing): see [`crate::runahead`].
    pub run_ahead: bool,
    /// What running ahead did so far.
    pub run_ahead_stats: crate::runahead::Stats,
    /// Tests: makes running ahead roll boards back as often as it can (each board ignores where the window ends
    /// until it is rolled back, and every window is cut at a random boundary drawn from this seed). The run must
    /// stay the same.
    #[doc(hidden)]
    pub run_ahead_stress: Option<u64>,
    pub now: Time,
    /// Pace virtual time to the wall clock.
    pub realtime: bool,
    /// When each board is switched on (it stays unpowered before), and whether it has been.
    pub power_on: Vec<(Time, bool)>,
    /// Simulated vehicles moved by the boards' thrusters, stepped after every slice.
    pub vehicles: Vec<VehicleSim>,
    /// Real-time pacing, warning when the robot falls behind the wall clock.
    pub(crate) pacing: crate::session::Pacing,
    /// How the boards were spread over threads last time, and what they cost (see [`crate::parallel`]).
    pub(crate) balance: crate::parallel::Balance,
    /// When this is set, [`Robot::run_until`] returns at the next slice boundary, before `end` (the command
    /// line's [`crate::signal::STOP`], set by SIGINT or SIGTERM).
    pub stop_flag: Option<&'static std::sync::atomic::AtomicBool>,
}

impl Default for Robot {
    fn default() -> Self {
        Robot {
            boards: Vec::new(),
            names: Vec::new(),
            inputs: Vec::new(),
            pins: Vec::new(),
            client_ids: Vec::new(),
            ros_tomls: Vec::new(),
            images: Vec::new(),
            board_bus: Vec::new(),
            usb: Vec::new(),
            buses: Vec::new(),
            bus_names: Vec::new(),
            medium: None,
            slice: Time::from_us(100),
            threads: 1,
            run_ahead: true,
            run_ahead_stats: Default::default(),
            run_ahead_stress: None,
            now: Time::ZERO,
            realtime: false,
            power_on: Vec::new(),
            vehicles: Vec::new(),
            pacing: Default::default(),
            balance: Default::default(),
            stop_flag: None,
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
        self.pins.push(Default::default());
        self.client_ids.push(None);
        self.ros_tomls.push(None);
        self.images.push(String::new());
        self.board_bus.push(bus);
        self.usb.push(UsbPorts::default());
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
    /// Returns `None` early, at a slice boundary with [`Robot::now`] before `end`, once [`Robot::stop_flag`] is set.
    pub fn run_until(&mut self, end: Time) -> Option<(usize, StopReason)> {
        // The sensors see a vehicle moved since the last run (a scenario putting it elsewhere) from the start
        for v in &self.vehicles {
            v.update_outputs();
        }
        if self.medium.is_some() {
            return self.run_in_medium(end, None);
        }
        if crate::parallel::worth_it(self, end) {
            if crate::runahead::possible(self) {
                return crate::runahead::run_until(self, end, None, None);
            }
            return crate::parallel::run_until(self, end, None);
        }
        self.run_in_turn(end)
    }

    /// The boards in turn, slice by slice, on the calling thread.
    fn run_in_turn(&mut self, end: Time) -> Option<(usize, StopReason)> {
        while self.now < end && !crate::signal::is_set(self.stop_flag) {
            let t = (self.now + self.slice).min(end);
            for (i, b) in self.boards.iter_mut().enumerate() {
                if !power_on_by(b, &mut self.power_on[i], t) {
                    continue;
                }
                match b.run_machine_through_lockups(t) {
                    StopReason::TimeLimit => {}
                    other => return Some((i, other)),
                }
                if let Some(m) = &self.medium {
                    m.lock().unwrap().progress(i, t);
                }
            }
            let mut boards: Vec<&mut Session> = self.boards.iter_mut().collect();
            end_slice(&mut self.buses, &self.board_bus, &mut boards, &mut self.vehicles, t);
            self.now = t;
            if self.realtime {
                self.pacing.pace(self.now);
            }
        }
        None
    }

    /// Runs every board until `soft` or a little past it, at most until `end`: running its boards ahead of each other
    /// ([`crate::runahead`]), it returns where the first window that ends at or after `soft` ends (where the boards
    /// meet anyway); otherwise at `soft`. Time travel takes its checkpoints this way, so that recording does not cut
    /// the windows short. Stops and [`Robot::stop_flag`] as [`Robot::run_until`].
    pub fn run_until_after(&mut self, soft: Time, end: Time) -> Option<(usize, StopReason)> {
        let soft = soft.min(end);
        if self.medium.is_some() && soft < end {
            for v in &self.vehicles {
                v.update_outputs();
            }
            return self.run_in_medium(end, Some(soft));
        }
        if soft < end && crate::parallel::worth_it(self, end) && crate::runahead::possible(self) {
            for v in &self.vehicles {
                v.update_outputs();
            }
            return crate::runahead::run_until(self, end, Some(soft), None);
        }
        self.run_until(soft)
    }

    /// [`Robot::run_until`] (with `soft`, [`Robot::run_until_after`]) for boards in an acoustic medium: in pieces of
    /// at most [`MEDIUM_PIECE`], on threads where the boards can run at least a slice apart through the whole piece
    /// ([`crate::acoustic::Lookahead`]), running ahead of each other as far as the water lets them; otherwise (one
    /// thread, boards within a slice's sound of each other, a transducer on a simulated vehicle, whose path is not
    /// known ahead) in turn. The same run either way.
    fn run_in_medium(&mut self, end: Time, soft: Option<Time>) -> Option<(usize, StopReason)> {
        let look = match crate::acoustic::Lookahead::of(self) {
            Ok(l) if crate::parallel::max_threads(self) > 1 => l,
            _ => return self.run_in_turn(soft.unwrap_or(end)),
        };
        while self.now < end && !crate::signal::is_set(self.stop_flag) {
            let piece = (self.now + MEDIUM_PIECE).min(end);
            let apart = look.as_ref().is_none_or(|l| l.ahead(self.now, piece - self.now) >= self.slice);
            let stop = if apart && crate::parallel::worth_it(self, piece) {
                if crate::runahead::possible(self) {
                    crate::runahead::run_until(self, piece, soft.map(|s| s.min(piece)), look.as_ref())
                } else {
                    crate::parallel::run_until(self, soft.map_or(piece, |s| s.min(piece)), look.as_ref())
                }
            } else {
                self.run_in_turn(soft.map_or(piece, |s| s.min(piece).max(self.now)))
            };
            if stop.is_some() || soft.is_some_and(|s| self.now >= s) {
                return stop;
            }
        }
        None
    }

    /// Plugs board `i`'s USB port into a host controller model (once; the host enumerates whatever the firmware
    /// presents), like `run --usb`.
    pub fn plug_usb(&mut self, i: usize) {
        let m = &mut self.boards[i].machine;
        if m.sys.usb_host.is_none() {
            let now = m.now;
            m.sys.attach_usb_host(sim_devices::usb::host::HostController::new(), now);
        }
        self.usb[i].plugged = true;
    }

    /// Opens board `i`'s USB CDC port `port` (0, or 1 for titan_dual_stdio_usb's secondary port) from the host
    /// (115200 8N1, DTR and RTS), plugging the board in first, and notes where it goes. The device's output on port
    /// 0 is collected in [`Session::usb_cdc_text`] and passed to [`Session::on_usb_cdc_line`]; ptys are opened by
    /// [`Robot::open_usb_ptys`]. Port 1 carries raw bytes (micro-ROS), so it takes only a pty.
    pub fn set_usb_cdc(&mut self, i: usize, port: usize, sink: CdcSink) -> Result<(), String> {
        let name = &self.names[i];
        match port {
            0 => {}
            1 if sink == CdcSink::Stdout => {
                return Err(format!("{name}: the second CDC port carries raw bytes (micro-ROS): use pty or pty:PATH"));
            }
            1 => {}
            _ => return Err(format!("{name}: no CDC port {port} (0 or 1)")),
        }
        self.plug_usb(i);
        let m = &mut self.boards[i].machine;
        let now = m.now;
        m.sys.with_usb_host(now, |h| h.cdc_open(port, 115_200));
        let ports = &mut self.usb[i];
        *(if port == 0 { &mut ports.cdc } else { &mut ports.cdc2 }) = Some(sink);
        Ok(())
    }

    /// Opens a pseudo terminal for every CDC port set to `pty` or `pty:PATH` that has none yet (a snapshot has
    /// none: ptys are not copied). Returns a line per terminal saying where it is.
    pub fn open_usb_ptys(&mut self) -> Result<Vec<String>, String> {
        let mut opened = Vec::new();
        for i in 0..self.boards.len() {
            let (name, ports, s) = (&self.names[i], &self.usb[i], &mut self.boards[i]);
            for (port, sink) in [(0, &ports.cdc), (1, &ports.cdc2)] {
                let Some(CdcSink::Pty(link)) = sink else { continue };
                let slot = if port == 0 { &mut s.usb_cdc_pty } else { &mut s.usb_cdc2_pty };
                if slot.is_some() {
                    continue;
                }
                let pty = crate::pty::Pty::open_linked(link.as_deref())
                    .map_err(|e| format!("{name}: USB CDC port {port}: {e}"))?;
                opened.push(match link {
                    Some(l) => format!("{name}: USB CDC port {port} on {} (linked from {})", pty.path, l.display()),
                    None => format!("{name}: USB CDC port {port} on {}", pty.path),
                });
                *slot = Some(pty);
            }
        }
        Ok(opened)
    }

    /// The thread each board ran on in the last run on threads (0: the calling thread; see [`crate::parallel`]).
    pub fn thread_of_boards(&self) -> Vec<usize> {
        let mut t = self.balance.threads().to_vec();
        t.resize(self.boards.len(), 0);
        t
    }

    /// Runs board `i` on thread `thread[i]` (0: the calling thread) whenever the boards run on threads, instead of
    /// spreading them by what they cost (tests, experiments). Needs `threads` above every thread number given (or 0
    /// with enough cores); otherwise, or when boards are added, the boards are spread by cost again.
    pub fn fix_threads(&mut self, thread: Vec<usize>) {
        self.balance.fix(thread);
    }

    /// Starts real-time pacing afresh from now (after virtual time was held, e.g. paused from the dashboard).
    pub fn reset_pacing(&mut self) {
        self.pacing.reset();
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

    /// A deep copy of the whole robot: every machine, device, bus, the acoustic medium and the vehicles, re-linked
    /// so the copy's hydrophones and projectors use the copy's medium and vehicles, and its devices (and the sensor
    /// inputs its vehicles drive) read the copy's own physical inputs. The original and any number of copies can
    /// run side by side, each with its own inputs. Host endpoints are not copied (they are often handles to shared
    /// state, like a mock bridge); give the copy its own.
    pub fn snapshot(&self) -> Robot {
        let medium = self.medium.as_ref().map(|m| Arc::new(std::sync::Mutex::new(m.lock().unwrap().clone())));
        let mut boards = Vec::new();
        let mut inputs = Vec::new();
        let mut map = sim_core::analog::Relink::new();
        for (b, i) in self.boards.iter().zip(&self.inputs) {
            let mut s = b.snapshot();
            let (copy, m) = i.deep_copy();
            crate::board::relink_inputs(&mut s.machine, &m);
            map.merge(&m);
            boards.push(s);
            inputs.push(copy);
        }
        if let (Some(old), Some(new)) = (&self.medium, &medium) {
            for b in &mut boards {
                crate::acoustic::relink(&mut b.machine, old, new);
            }
        }
        // The copy's transducers follow the copy's vehicles
        let mut vehicles = Vec::new();
        for v in &self.vehicles {
            let (mut copy, pairs) = v.deep_copy();
            copy.relink_inputs(&map);
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
            inputs,
            pins: self.pins.clone(),
            client_ids: self.client_ids.clone(),
            ros_tomls: self.ros_tomls.clone(),
            images: self.images.clone(),
            board_bus: self.board_bus.clone(),
            usb: self.usb.clone(),
            buses: self
                .buses
                .iter()
                .map(|b| RobotBus { bus: b.bus.clone(), endpoints: Vec::new(), on_can: None })
                .collect(),
            bus_names: self.bus_names.clone(),
            medium,
            slice: self.slice,
            threads: self.threads,
            run_ahead: self.run_ahead,
            run_ahead_stats: self.run_ahead_stats,
            run_ahead_stress: self.run_ahead_stress,
            now: self.now,
            realtime: self.realtime,
            power_on: self.power_on.clone(),
            vehicles,
            pacing: Default::default(),
            balance: self.balance.clone(),
            stop_flag: None,
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
        Robot::from_spec_with_images(spec, dir, &|_, path| path)
    }

    /// [`Robot::from_spec`] with each board's firmware image chosen by `image`: it gets the board's name and the
    /// image path the robot file gives (resolved as [`Robot::from_spec`] does, `$TITAN_RS_RELEASE_DIR` included)
    /// and returns the image to load, used as it is (firmware A/B comparisons put another build in its place).
    pub fn from_spec_with_images(
        spec: &RobotSpec,
        dir: &Path,
        image: &dyn Fn(&str, PathBuf) -> PathBuf,
    ) -> Result<Robot, String> {
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
            let board_file = dir.join(expand(&b.board)?);
            let mut board = Board::load(&board_file)?;
            if let Some(t) = &b.transducer {
                board.transducer = Some(t.clone());
            }
            // Robot-level straps replace the board file's entries for the same pins
            board.gpio.retain(|g| !b.gpio.iter().any(|o| o.pin == g.pin));
            board.gpio.extend(b.gpio.iter().cloned());
            let elf = image(&b.name, crate::resolve_image(&dir.join(expand(&b.elf)?)));
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
            robot.pins[owner] = board.pins.clone();
            robot.client_ids[owner] = board.can.as_ref().and_then(|c| c.client_id);
            robot.ros_tomls[owner] = b.ros_toml.as_ref().map(|p| expand(p).map(|p| dir.join(p))).transpose()?;
            robot.images[owner] = elf.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
            if let Some(ms) = b.power_on_ms
                && ms > 0.0
            {
                robot.power_on[owner] = (Time::from_secs_f64(ms / 1000.0), false);
            }
            if b.usb {
                robot.plug_usb(owner);
            }
            for (port, sink) in [(0, &b.usb_cdc), (1, &b.usb_cdc2)] {
                let sink = match sink {
                    Some(CdcSink::Pty(Some(link))) => CdcSink::Pty(Some(dir.join(expand(link)?))),
                    Some(s) => s.clone(),
                    None => continue,
                };
                robot.set_usb_cdc(owner, port, sink)?;
            }
        }
        for (v, s) in robot.vehicles.iter_mut().zip(&spec.vehicle) {
            v.wire(s, &robot.names, &robot.boards, &robot.inputs)?;
        }
        Ok(robot)
    }
}

/// [`crate::expand_path`] for a path from a robot file.
fn expand(p: &Path) -> Result<PathBuf, String> {
    crate::expand_path(&p.to_string_lossy())
}

/// A board's USB port in a robot: plugged into a host or not, and where the host's CDC ports go.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct UsbPorts {
    pub plugged: bool,
    /// CDC port 0 (pico-sdk stdio_usb, titan_dual_stdio_usb's text log).
    pub cdc: Option<CdcSink>,
    /// CDC port 1 (titan_dual_stdio_usb's secondary port: micro-ROS).
    pub cdc2: Option<CdcSink>,
}

/// Switches `board` on if it is due by `t` (`power_on`: when, and whether it has been); whether it is on.
pub(crate) fn power_on_by(board: &mut Session, power_on: &mut (Time, bool), t: Time) -> bool {
    let (on_at, on) = power_on;
    if !*on {
        if t <= *on_at {
            return false;
        }
        board.machine.power_on_at(*on_at);
        *on = true;
    }
    true
}

/// What happens at the end of every slice, once every board has reached `t`: each bus arbitrates and delivers
/// what its boards and host endpoints sent, the boards' host sides and devices catch up, and the vehicles step.
pub(crate) fn end_slice(
    buses: &mut [RobotBus],
    board_bus: &[Option<usize>],
    boards: &mut [&mut Session],
    vehicles: &mut [VehicleSim],
    t: Time,
) {
    for (j, rb) in buses.iter_mut().enumerate() {
        let mut nodes: Vec<&mut dyn CanNode> = Vec::new();
        for (b, on) in boards.iter_mut().zip(board_bus) {
            if *on == Some(j) {
                nodes.extend(b.machine.can_nodes());
            }
        }
        step_bus(rb, &mut nodes, t);
    }
    for b in boards.iter_mut() {
        b.step_ethernet();
        b.machine.update_devices();
        b.poll_rtt();
        b.poll_uarts();
    }
    for v in vehicles {
        v.advance(t, boards);
    }
}

/// Steps bus `rb` with its nodes `nodes` (its boards' CAN controllers, in board order) up to `t`, and passes what
/// went over it to its callback.
pub(crate) fn step_bus(rb: &mut RobotBus, nodes: &mut [&mut dyn CanNode], t: Time) {
    let mut eps: Vec<&mut dyn CanEndpoint> =
        rb.endpoints.iter_mut().map(|e| &mut **e as &mut dyn CanEndpoint).collect();
    let before = rb.bus.record.as_ref().map_or(0, |r| r.len());
    rb.bus.step(nodes, &mut eps, t);
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
    /// Firmware image (ELF or UF2), relative to the robot file or absolute, with `~` and environment variables
    /// expanded.
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
    /// Plug the board's USB port into a host (implied by `usb_cdc` and `usb_cdc2`).
    #[serde(default)]
    pub usb: bool,
    /// Open the device's (first) USB CDC port: `stdout`, `pty` or `pty:PATH`.
    pub usb_cdc: Option<CdcSink>,
    /// Open its second CDC port (titan_dual_stdio_usb's micro-ROS port): `pty` or `pty:PATH`.
    pub usb_cdc2: Option<CdcSink>,
    /// The board's nano-ROS topics: its firmware's `ros.toml`, which a nano-ROS mock agent for the board
    /// (`--mock-agent nanoros:BOARD`, a scenario's `agents`) reads.
    pub ros_toml: Option<PathBuf>,
}
