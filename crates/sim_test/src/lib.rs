//! Firmware integration tests: boot an ELF on a board, drive its sensors and inputs, inject faults, and assert on
//! what it logs, on its CAN traffic and on the `titan_link` messages a (mock) `titan_bridge` receives.
//!
//! ```no_run
//! use sim_test::{Bench, Time};
//! use sim_test::titan_link::DeviceToHost;
//!
//! let Some(elf) = sim_test::titan_rs("power_board") else { return };
//! let mut b = Bench::builder(&elf).board("mk2/power_board").bridge(1).boot().unwrap();
//! b.wait_for_log("ROS connected", Time::from_secs(1)).unwrap();
//! b.set("sht41.temperature_c", 31.5);
//! b.run_for(Time::from_ms(1500));
//! assert_eq!(b.statuses().last().unwrap().1.faults, 0);
//! let base = b.snapshot(); // fast setup: boot once, run every case from here
//! ```
//!
//! [`RobotBench`] does the same for a robot file (several boards, buses and an acoustic environment), and
//! [`sweep`] runs one scenario from a snapshot over a set of parameters in parallel, e.g. the IVC range and noise
//! sweeps in `tests/ivc_sweep.rs`.

use std::path::{Path, PathBuf};

pub use rp2040::StopReason;
pub use sim_acoustics::medium::Trajectory;
pub use sim_core::time::Time;
pub use sim_host::Session;
pub use sim_host::board::{Board, Inputs};
pub use sim_host::mock_bridge::MockBridge;
pub use sim_host::robot::{Robot, RobotSpec};
pub use titan_link;

use sim_core::image::Image;
use sim_devices::can::{BusRecord, CanBus};
use titan_link::{DeviceToHost, FirmwareStatus, HostToDevice};

/// The repository root.
pub fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A titan_firmware_rs release ELF, if it has been built (`cd external/titan_firmware_rs/firmware && cargo build
/// --workspace --release`). Tests return early (and say so) when it is missing.
pub fn titan_rs(name: &str) -> Option<PathBuf> {
    let p = sim_host::titan_rs_release_dir(&repo()).join(name);
    found(p, "cd external/titan_firmware_rs/firmware && cargo build --workspace --release, or set TITAN_RS_RELEASE_DIR")
}

/// A C titan_firmware build output under `build/c` (`scripts/build_c_firmware.sh`), if present.
pub fn titan_c(rel: &str) -> Option<PathBuf> {
    found(repo().join("build/c").join(rel), "scripts/build_c_firmware.sh")
}

fn found(p: PathBuf, how: &str) -> Option<PathBuf> {
    if p.exists() {
        Some(p)
    } else {
        eprintln!("skipping: {} not built ({how})", p.display());
        None
    }
}

/// `boards/<name>.toml`.
pub fn board_file(name: &str) -> PathBuf {
    repo().join("boards").join(format!("{name}.toml"))
}

/// A change to a board description before it is applied.
pub type BoardEdit = Box<dyn FnOnce(&mut Board)>;

/// Builds a [`Bench`].
pub struct BenchBuilder {
    elf: PathBuf,
    board: Option<String>,
    bridge: Option<(u8, Time, Option<bool>)>,
    rom_boot: bool,
    record_bus: bool,
    edit: Option<BoardEdit>,
}

impl BenchBuilder {
    /// Changes the board description before it is applied (a missing part, different straps, ...).
    pub fn edit_board(mut self, f: impl FnOnce(&mut Board) + 'static) -> Self {
        self.edit = Some(Box::new(f));
        self
    }

    /// The board file (`mk2/power_board`, or a path to a `.toml`).
    pub fn board(mut self, name: &str) -> Self {
        self.board = Some(name.to_string());
        self
    }

    /// Puts a mock `titan_bridge` for CANmore client `client` on the board's bus, pinging from time 0 and enabling
    /// the vehicle (kill switch released).
    pub fn bridge(self, client: u8) -> Self {
        self.bridge_with(client, Time::ZERO, Some(false))
    }

    /// A mock bridge starting at `start`, forwarding `kill` (`None`: no kill switch forwarding).
    pub fn bridge_with(mut self, client: u8, start: Time, kill: Option<bool>) -> Self {
        self.bridge = Some((client, start, kill));
        self
    }

    /// Boot through the bootrom and boot2 (the real flash path) instead of directly.
    pub fn rom_boot(mut self) -> Self {
        self.rom_boot = true;
        self
    }

    /// Keep a record of every CAN frame ([`Bench::frames`]).
    pub fn record_bus(mut self) -> Self {
        self.record_bus = true;
        self
    }

    pub fn boot(self) -> Result<Bench, String> {
        let image = Image::load(&self.elf).map_err(|e| format!("{}: {e}", self.elf.display()))?;
        let mut board = match &self.board {
            Some(b) if b.ends_with(".toml") => Some(Board::load(Path::new(b))?),
            Some(b) => Some(Board::load(&board_file(b))?),
            None => None,
        };
        if let (Some(b), Some(edit)) = (board.as_mut(), self.edit) {
            edit(b);
        }
        let mut m = rp2040::Machine::new(rp2040::MachineConfig {
            bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
            flash_size: board.as_ref().and_then(|b| b.flash_size).unwrap_or(2 << 20),
            ..Default::default()
        });
        m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
        let inputs = match &board {
            Some(b) => b.apply(&mut m)?,
            None => Inputs::default(),
        };
        m.load(&image)?;
        if self.rom_boot {
            m.rom_boot()?;
        } else {
            m.direct_boot()?;
        }
        let mut session = Session::new(m, &image);
        let mut bridge = None;
        if let Some(rates) = board.as_ref().and_then(|b| b.can_rates()) {
            let mut bus = CanBus::new("can0", rates);
            if self.record_bus {
                bus.record = Some(Vec::new());
            }
            session.can_bus = Some(bus);
            if let Some((client, start, kill)) = self.bridge {
                let br = MockBridge::new(client, rates.data.is_some(), start);
                br.set_kill(kill);
                session.can_endpoints.push(Box::new(br.clone()));
                bridge = Some(br);
            }
        } else if self.bridge.is_some() {
            return Err("a bridge needs a board with a CAN bus".into());
        }
        Ok(Bench { session, inputs, bridge, image })
    }
}

/// One board running its firmware.
pub struct Bench {
    pub session: Session,
    /// The board's physical inputs (see its board file).
    pub inputs: Inputs,
    pub bridge: Option<MockBridge>,
    image: Image,
}

impl Bench {
    pub fn builder(elf: impl AsRef<Path>) -> BenchBuilder {
        BenchBuilder {
            elf: elf.as_ref().to_path_buf(),
            board: None,
            bridge: None,
            rom_boot: false,
            record_bus: false,
            edit: None,
        }
    }

    pub fn now(&self) -> Time {
        self.session.machine.now
    }

    /// Runs until virtual time `t`. A stop for any other reason than reaching `t` (lockup, deadlock) is an error.
    pub fn run_until(&mut self, t: Time) -> Result<(), StopReason> {
        match self.session.run_until(t) {
            StopReason::TimeLimit => {
                self.session.poll_uarts();
                Ok(())
            }
            other => Err(other),
        }
    }

    /// Runs for `d` more; panics if the machine stops for another reason.
    pub fn run_for(&mut self, d: Time) {
        let t = self.now() + d;
        if let Err(e) = self.run_until(t) {
            panic!("stopped at {}: {e:?}", self.now());
        }
    }

    /// Sets a physical input (`sht41.temperature_c`, `adc1.volts`, ...).
    pub fn set(&self, input: &str, value: f64) {
        self.inputs.set(input, value);
    }

    pub fn get(&self, input: &str) -> f64 {
        self.inputs.get(input)
    }

    /// Whether a log line (defmt) containing `text` has been seen.
    pub fn logged(&self, text: &str) -> bool {
        self.session.logged(text)
    }

    /// When the first log line containing `text` was read.
    pub fn log_time(&self, text: &str) -> Option<Time> {
        self.session.logs.iter().find(|(_, l)| l.message.contains(text)).map(|(t, _)| *t)
    }

    /// Runs (in 10 ms steps) until a log line containing `text` appears, for at most `limit` more.
    pub fn wait_for_log(&mut self, text: &str, limit: Time) -> Result<Time, String> {
        let end = self.now() + limit;
        loop {
            if let Some(t) = self.log_time(text) {
                return Ok(t);
            }
            if self.now() >= end {
                return Err(format!("no {text:?} by {}; log: {:#?}", self.now(), self.session.logs));
            }
            let t = (self.now() + Time::from_ms(10)).min(end);
            self.run_until(t).map_err(|e| format!("{e:?}"))?;
        }
    }

    /// Text a UART sent so far.
    pub fn uart(&self, n: usize) -> &str {
        &self.session.uart_text[n]
    }

    fn bridge(&self) -> &MockBridge {
        self.bridge.as_ref().expect("the bench has no bridge")
    }

    /// Sends a message from the (mock) bridge.
    pub fn send(&self, msg: HostToDevice) {
        self.bridge().send(msg);
    }

    /// Every titan_link message the bridge received, with when.
    pub fn messages(&self) -> Vec<(Time, DeviceToHost)> {
        self.bridge().state().received.clone()
    }

    /// Received messages picked out by `f`.
    pub fn picked<T>(&self, f: impl Fn(&DeviceToHost) -> Option<T>) -> Vec<(Time, T)> {
        self.bridge().state().received.iter().filter_map(|(t, m)| f(m).map(|v| (*t, v))).collect()
    }

    pub fn statuses(&self) -> Vec<(Time, FirmwareStatus)> {
        self.bridge().statuses()
    }

    /// Problems the bridge had decoding what the board sent.
    pub fn link_errors(&self) -> Vec<String> {
        self.bridge().state().errors.clone()
    }

    /// Every frame on the board's bus (with [`BenchBuilder::record_bus`]).
    pub fn frames(&self) -> &[BusRecord] {
        self.session.can_bus.as_ref().and_then(|b| b.record.as_deref()).unwrap_or(&[])
    }

    /// Makes core 0 call the firmware's panic path now (`__defmt_default_panic`, the target of `defmt::panic!`;
    /// in C images `panic` is called with a message).
    pub fn inject_panic(&mut self) -> Result<(), String> {
        if let Some(s) = self.image.symbol("__defmt_default_panic") {
            let addr = s.addr;
            self.session.machine.inject_call(0, addr, &[]);
            return Ok(());
        }
        Err("no panic entry point found in the image".into())
    }

    /// Resets the chip through the watchdog (scratch registers and RAM kept).
    pub fn watchdog_reset(&mut self) {
        self.session.machine.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
    }

    /// Removes power and applies it again.
    pub fn power_cycle(&mut self) {
        self.session.machine.power_cycle();
    }

    /// A copy that runs on independently from here (machine, devices, bus, logs and bridge state). Physical inputs
    /// stay shared with the original: set them after taking the copy.
    pub fn snapshot(&self) -> Bench {
        let mut session = self.session.snapshot();
        let bridge = self.bridge.as_ref().map(MockBridge::fork);
        if let Some(b) = &bridge {
            session.can_endpoints.push(Box::new(b.clone()));
        }
        Bench { session, inputs: self.inputs.clone(), bridge, image: self.image.clone() }
    }
}

/// Several boards from a robot file, each bus with an optional mock bridge.
pub struct RobotBench {
    pub robot: Robot,
    /// The mock bridge on each bus, by bus index.
    pub bridges: Vec<Option<MockBridge>>,
}

impl RobotBench {
    /// Loads `robots/<file>` (or a path), changed by `edit` first; `bridges` names a CANmore client to put a mock
    /// bridge for on each bus (by bus name).
    pub fn load(file: &str, bridges: &[(&str, u8)], edit: impl FnOnce(&mut RobotSpec)) -> Result<RobotBench, String> {
        let path = if file.ends_with(".toml") && Path::new(file).exists() {
            PathBuf::from(file)
        } else {
            repo().join("robots").join(file)
        };
        let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut spec: RobotSpec = toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        edit(&mut spec);
        let mut robot = Robot::from_spec(&spec, path.parent().unwrap())?;
        let mut out = vec![None; robot.buses.len()];
        for (bus, client) in bridges {
            let j = robot.bus_names.iter().position(|n| n == bus).ok_or_else(|| format!("no bus {bus}"))?;
            let fd = robot.buses[j].bus.rates.data.is_some();
            let br = MockBridge::new(*client, fd, Time::ZERO);
            br.set_kill(Some(false));
            robot.buses[j].endpoints.push(Box::new(br.clone()));
            out[j] = Some(br);
        }
        Ok(RobotBench { robot, bridges: out })
    }

    pub fn now(&self) -> Time {
        self.robot.now
    }

    pub fn run_until(&mut self, t: Time) {
        if let Some((i, why)) = self.robot.run_until(t) {
            panic!("board {} stopped at {}: {why:?}", self.robot.names[i], self.robot.boards[i].machine.now);
        }
    }

    pub fn run_for(&mut self, d: Time) {
        let t = self.now() + d;
        self.run_until(t);
    }

    /// The bridge on bus `name`.
    pub fn bridge(&self, bus: &str) -> &MockBridge {
        let j = self.robot.bus_names.iter().position(|n| n == bus).expect("no such bus");
        self.bridges[j].as_ref().expect("no bridge on that bus")
    }

    pub fn board(&self, name: &str) -> &Session {
        &self.robot.boards[self.robot.index(name).expect("no such board")]
    }

    /// Received messages on bus `bus` picked out by `f`.
    pub fn picked<T>(&self, bus: &str, f: impl Fn(&DeviceToHost) -> Option<T>) -> Vec<(Time, T)> {
        self.bridge(bus).state().received.iter().filter_map(|(t, m)| f(m).map(|v| (*t, v))).collect()
    }

    /// A copy of the robot (boards, buses, acoustic medium and bridges) that runs on independently.
    pub fn snapshot(&self) -> RobotBench {
        let mut robot = self.robot.snapshot();
        let bridges: Vec<Option<MockBridge>> = self.bridges.iter().map(|b| b.as_ref().map(MockBridge::fork)).collect();
        for (j, b) in bridges.iter().enumerate() {
            if let Some(b) = b {
                robot.buses[j].endpoints.push(Box::new(b.clone()));
            }
        }
        RobotBench { robot, bridges }
    }

    /// Moves board `name`'s transducer.
    pub fn place(&mut self, name: &str, trajectory: Trajectory) {
        let i = self.robot.index(name).expect("no such board");
        self.robot.place(i, trajectory);
    }
}

/// Runs `scenario` once per point, each from its own snapshot of `base`, on up to `threads` threads at a time.
/// Results come back in the order of `points`.
pub fn sweep<P: Sync, R: Send>(
    base: &RobotBench,
    points: &[P],
    threads: usize,
    scenario: impl Fn(RobotBench, &P) -> R + Sync,
) -> Vec<R> {
    let mut results: Vec<Option<R>> = (0..points.len()).map(|_| None).collect();
    for (chunk_points, chunk_results) in points.chunks(threads.max(1)).zip(results.chunks_mut(threads.max(1))) {
        let benches: Vec<RobotBench> = chunk_points.iter().map(|_| base.snapshot()).collect();
        std::thread::scope(|s| {
            let handles: Vec<_> = benches
                .into_iter()
                .zip(chunk_points)
                .map(|(bench, p)| {
                    let scenario = &scenario;
                    s.spawn(move || scenario(bench, p))
                })
                .collect();
            for (slot, h) in chunk_results.iter_mut().zip(handles) {
                *slot = Some(h.join().expect("sweep point panicked"));
            }
        });
    }
    results.into_iter().map(Option::unwrap).collect()
}
