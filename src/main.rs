//! `firmware_simulator`: runs RP2040 firmware images in the emulator.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand, ValueEnum};

use rp2040::machine::BkptPolicy;
use rp2040::unmodelled::LogMode;
use rp2040::{Machine, MachineConfig, StopReason};
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::trace::{Category, WriterSink};
use sim_host::Session;

#[derive(Parser)]
#[command(version, about = "RP2040 emulator for UWRT titan firmware")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

// Parsed once; the size difference between the subcommands' arguments doesn't matter
#[allow(clippy::large_enum_variant)]
#[derive(Subcommand)]
enum Command {
    /// Runs one firmware image (ELF or UF2).
    Run(RunArgs),
    /// Runs every board of a robot file on their shared CAN buses (and acoustic environment).
    Robot(RobotArgs),
    /// What `picotool` does, against an emulated board in BOOTSEL mode.
    #[command(subcommand)]
    Picoboot(PicobootCommand),
}

#[derive(Subcommand)]
enum PicobootCommand {
    /// `picotool load -v -x FILE`: powers a board on with BOOTSEL held (blank flash, or `--flash-file`), flashes
    /// FILE over PICOBOOT through the bootrom, reboots into it and runs it for `--time`. Further options are
    /// `run` options, e.g. `--time 2s --board ... --usb-cdc stdout`.
    Load {
        /// Image to flash (ELF or UF2).
        file: PathBuf,
        /// `run` options.
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        run_args: Vec<String>,
    },
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum LoadVia {
    /// The PICOBOOT vendor interface (`picotool load`).
    Picoboot,
    /// Copy the image as a UF2 file to the BOOTSEL mass storage drive.
    Drive,
}

#[derive(clap::Args)]
struct RobotArgs {
    /// Robot file (robots/*.toml).
    robot: PathBuf,
    /// Virtual time to run for.
    #[arg(short, long, default_value = "5s", value_parser = parse_time)]
    time: Time,
    /// Attach a scripted titan_bridge for a client on a bus: `BUS:CLIENT`, e.g. `talos:8` (repeatable).
    #[arg(long = "mock-bridge", value_parser = parse_bus_client)]
    mock_bridges: Vec<(String, u8)>,
    /// Bridge a bus to a SocketCAN interface: `BUS=IFACE`, e.g. `talos=vcan0` (repeatable).
    #[arg(long = "can", value_parser = parse_bus_iface)]
    cans: Vec<(String, String)>,
    /// Print every CAN frame, candump style, with its bus.
    #[arg(long)]
    candump: bool,
    /// Pace virtual time to the wall clock.
    #[arg(long)]
    realtime: bool,
    /// Replace the ambient noise every hydrophone hears with white noise at this level, dB re 1 µPa²/Hz.
    #[arg(long)]
    noise_db: Option<f64>,
    /// Relay an Ethernet board's UDP through a host socket: `BOARD:LOCAL=PEER`, e.g.
    /// `backplane:127.0.0.1:2203=127.0.0.1:2204` (repeatable).
    #[arg(long = "udp", value_parser = parse_board_udp)]
    udps: Vec<(String, String, String)>,
    /// Attach a scripted titan_bridge to an Ethernet board (Puddles' UDP transport), by board name (repeatable).
    #[arg(long = "eth-mock-bridge")]
    eth_mock_bridges: Vec<String>,
    /// Write each vehicle's state every 10 ms to this CSV file (`NAME` in the file name is replaced by the
    /// vehicle's name when there are several).
    #[arg(long)]
    vehicle_csv: Option<PathBuf>,
    /// Inject a fault on a board at a virtual time: `TIME@BOARD:FAULT`, e.g. `1.5s@camera_cage:can-unplug`
    /// (repeatable; see `sim_host::fault::Fault`'s `from_str` for the faults). Bridge faults act on the first mock
    /// bridge on the board's bus.
    #[arg(long = "fault", value_parser = parse_robot_fault)]
    faults: Vec<(Time, String, sim_host::fault::Fault)>,
    /// Profile every board's firmware: write `DIR/BOARD.txt` (functions by time, per core) and `DIR/BOARD.folded`
    /// (folded stacks for flamegraph tools).
    #[arg(long)]
    profile_dir: Option<PathBuf>,
    /// Mean virtual time between profile samples.
    #[arg(long, default_value = "10us", value_parser = parse_time)]
    profile_interval: Time,
}

fn parse_board_udp(s: &str) -> Result<(String, String, String), String> {
    let (board, rest) = s.split_once(':').ok_or("expected BOARD:LOCAL=PEER")?;
    let (local, peer) = rest.split_once('=').ok_or("expected BOARD:LOCAL=PEER")?;
    Ok((board.to_string(), local.to_string(), peer.to_string()))
}

fn parse_robot_fault(s: &str) -> Result<(Time, String, sim_host::fault::Fault), String> {
    let (at, rest) = s.split_once('@').ok_or("expected TIME@BOARD:FAULT")?;
    let (board, fault) = rest.split_once(':').ok_or("expected TIME@BOARD:FAULT")?;
    Ok((parse_time(at)?, board.to_string(), fault.parse()?))
}

fn parse_fault(s: &str) -> Result<(Time, sim_host::fault::Fault), String> {
    let (at, fault) = s.split_once('@').ok_or("expected TIME@FAULT")?;
    Ok((parse_time(at)?, fault.parse()?))
}

/// How a frame on the bus is shown in `--candump`.
fn candump_line(rec: &sim_devices::can::BusRecord) -> String {
    let note = match (rec.error, rec.acked) {
        (Some(e), _) => format!("  (destroyed: {e:?})"),
        (None, false) => "  (no ACK)".into(),
        (None, true) => String::new(),
    };
    format!("{}{note}", sim_devices::can::candump(&rec.frame))
}

fn parse_bus_client(s: &str) -> Result<(String, u8), String> {
    let (bus, client) = s.split_once(':').ok_or("expected BUS:CLIENT")?;
    Ok((bus.to_string(), client.parse().map_err(|e| format!("{client:?}: {e}"))?))
}

fn parse_bus_iface(s: &str) -> Result<(String, String), String> {
    let (bus, iface) = s.split_once('=').ok_or("expected BUS=IFACE")?;
    Ok((bus.to_string(), iface.to_string()))
}

fn robot(args: RobotArgs) -> Result<()> {
    let mut r = sim_host::robot::Robot::load(&args.robot).map_err(anyhow::Error::msg)?;
    log::info!(
        "{} board(s) on {} bus(es){}",
        r.boards.len(),
        r.buses.len(),
        if r.medium.is_some() { ", in an acoustic environment" } else { "" }
    );
    for (i, s) in r.boards.iter_mut().enumerate() {
        let name = r.names[i].clone();
        s.keep_logs = false;
        let n = name.clone();
        s.on_log = Some(Box::new(move |t, line| println!("[{t}] {n}: {line}")));
        s.on_uart_line = Some(Box::new(move |t, u, line| println!("[{t}] {name} uart{u}: {line}")));
    }
    let mut bridges: Vec<Option<sim_host::mock_bridge::MockBridge>> = vec![None; r.buses.len()];
    for (bus, client) in &args.mock_bridges {
        let j = r.bus_names.iter().position(|n| n == bus).with_context(|| format!("no bus {bus:?}"))?;
        let rb = &mut r.buses[j];
        let b = sim_host::mock_bridge::MockBridge::new(*client, rb.bus.rates.data.is_some(), Time::ZERO);
        b.set_kill(Some(false));
        rb.endpoints.push(Box::new(b.clone()));
        bridges[j].get_or_insert(b);
        log::info!("mock bridge for client {client} on {bus}");
    }
    let mut faults = Vec::new();
    for (t, board, fault) in args.faults {
        let i = r.index(&board).with_context(|| format!("--fault: no board {board:?}"))?;
        faults.push((t, i, fault));
    }
    faults.sort_by_key(|f| f.0);
    for (bus, iface) in &args.cans {
        let sock = sim_host::socketcan::SocketCan::open(iface).with_context(|| format!("opening {iface}"))?;
        r.bus(bus).with_context(|| format!("no bus {bus:?}"))?.endpoints.push(Box::new(sock));
        log::info!("bus {bus} bridged to {iface}");
    }
    if args.candump {
        for (j, rb) in r.buses.iter_mut().enumerate() {
            rb.bus.record = Some(Vec::new());
            let name = r.bus_names[j].clone();
            rb.on_can = Some(Box::new(move |rec| println!("[{}] {name} {}", rec.end, candump_line(rec))));
        }
    }
    if let Some(db) = args.noise_db {
        r.set_noise_db(db);
    }
    for name in &args.eth_mock_bridges {
        let s = r.board(name).with_context(|| format!("no board {name:?}"))?;
        anyhow::ensure!(s.machine.sys.ethernet.is_some(), "{name} has no Ethernet");
        let bridge = sim_host::ethernet::PuddlesBridge::new(Time::ZERO);
        bridge.set_kill(Some(false));
        s.eth_endpoints.push(Box::new(bridge));
        log::info!("mock bridge attached to {name} over Ethernet");
    }
    for (name, local, peer) in &args.udps {
        let s = r.board(name).with_context(|| format!("no board {name:?}"))?;
        anyhow::ensure!(s.machine.sys.ethernet.is_some(), "{name} has no Ethernet");
        let relay = sim_host::ethernet::UdpRelay::bind(local.parse()?, peer.parse()?)
            .with_context(|| format!("binding {local}"))?;
        s.eth_endpoints.push(Box::new(relay));
        log::info!("{name} UDP relayed: host {local} <-> {peer}");
    }
    if args.vehicle_csv.is_some() {
        for v in &mut r.vehicles {
            v.keep_log(Time::from_ms(10));
        }
    }
    r.realtime = args.realtime;
    if let Some(dir) = &args.profile_dir {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        for s in r.boards.iter_mut() {
            s.start_profile(args.profile_interval);
        }
    }
    let wall = std::time::Instant::now();
    let mut faults = faults.into_iter().peekable();
    let stopped = loop {
        let next = faults.next_if(|f| f.0 <= args.time);
        let until = next.as_ref().map_or(args.time, |f| f.0);
        if let Some(s) = r.run_until(until) {
            break Some(s);
        }
        let Some((_, i, fault)) = next else { break None };
        let bridge = r.board_bus[i].and_then(|j| bridges[j].clone());
        let ctl = bridge.as_ref().map(|b| b as &dyn sim_host::fault::BridgeControl);
        if let Err(e) = sim_host::fault::robot_target(&mut r, i, ctl).apply(&fault) {
            bail!("fault {fault:?} on {}: {e}", r.names[i]);
        }
    };
    for s in r.boards.iter_mut() {
        s.flush_uarts();
    }
    eprintln!("stopped at {}; {:.2?} wall", r.now, wall.elapsed());
    for v in &r.vehicles {
        let s = &v.vehicle.state;
        let [roll, pitch, yaw] = s.attitude.euler().map(f64::to_degrees);
        eprintln!(
            "vehicle {}: at [{:.2}, {:.2}, {:.2}] m, roll {roll:.1} pitch {pitch:.1} yaw {yaw:.1} deg, {:.2} m/s",
            v.vehicle.name,
            s.position[0],
            s.position[1],
            s.position[2],
            sim_physics::math::norm(s.velocity)
        );
    }
    if let Some(dir) = &args.profile_dir {
        for (s, name) in r.boards.iter().zip(&r.names) {
            let Some(p) = &s.profiler else { continue };
            let path = dir.join(format!("{name}.txt"));
            let folded = p.report().write(&path, name).with_context(|| format!("writing {}", path.display()))?;
            eprintln!("profile of {name}: {} and {}", path.display(), folded.display());
        }
    }
    if let Some(path) = &args.vehicle_csv {
        let several = r.vehicles.len() > 1;
        for v in &r.vehicles {
            let p = if several {
                PathBuf::from(path.to_string_lossy().replace("NAME", &v.vehicle.name))
            } else {
                path.clone()
            };
            anyhow::ensure!(!several || p != *path, "--vehicle-csv needs NAME in the file name for several vehicles");
            let csv = v.log.as_ref().map(|l| l.to_csv()).unwrap_or_default();
            std::fs::write(&p, csv).with_context(|| format!("writing {}", p.display()))?;
        }
    }
    if let Some((i, why)) = stopped {
        bail!("{} stopped: {why:?}", r.names[i]);
    }
    Ok(())
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum TraceKind {
    /// Every executed instruction.
    Instr,
    /// Exception entry and return.
    Exc,
    /// GPIO changes.
    Gpio,
    /// Accesses to unmodelled registers.
    Unmodelled,
    /// Device messages (CAN frames, UART bytes, ...).
    Device,
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum UartSink {
    /// Print complete lines on stdout with their virtual time.
    Stdout,
    /// Create a pseudo terminal (output and input).
    Pty,
    /// Discard.
    None,
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum CdcSink {
    /// Print complete lines on stdout with their virtual time.
    Stdout,
    /// Create a pseudo terminal (output and input), like the device's /dev/ttyACM.
    Pty,
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum UnmodelledLog {
    First,
    Every,
    Off,
}

#[derive(clap::Args)]
struct RunArgs {
    /// Firmware image: ELF or UF2. Optional with `--bootsel` (the flash then starts blank).
    #[arg(required_unless_present = "bootsel")]
    image: Option<PathBuf>,
    /// Virtual time to run for, e.g. `500ms`, `2s`, `100us`.
    #[arg(short, long, default_value = "1s", value_parser = parse_time)]
    time: Time,
    /// Trace categories to print (comma separated).
    #[arg(long, value_delimiter = ',')]
    trace: Vec<TraceKind>,
    /// Write the trace to this file instead of stderr.
    #[arg(long)]
    trace_file: Option<PathBuf>,
    /// Instructions per core before switching cores.
    #[arg(long, default_value_t = 16)]
    quantum: u32,
    /// How to log accesses to unmodelled registers.
    #[arg(long, value_enum, default_value = "first")]
    unmodelled: UnmodelledLog,
    /// Flash size in bytes.
    #[arg(long, default_value_t = 2 * 1024 * 1024)]
    flash_size: usize,
    /// Crystal (XOSC) frequency.
    #[arg(long, default_value_t = 12_000_000)]
    crystal_hz: u64,
    /// Stop on BKPT instructions instead of escalating them to HardFault.
    #[arg(long)]
    stop_on_bkpt: bool,
    /// Where UART0 goes.
    #[arg(long, value_enum, default_value = "stdout")]
    uart0: UartSink,
    /// Where UART1 goes.
    #[arg(long, value_enum, default_value = "stdout")]
    uart1: UartSink,
    /// Board description (boards/<dir>/<board>.toml): attaches the board's external chips.
    #[arg(long)]
    board: Option<PathBuf>,
    /// Bridge the board's CAN bus to this SocketCAN interface (e.g. vcan0).
    #[arg(long)]
    can: Option<String>,
    /// Attach a scripted titan_bridge stand-in for this CANmore client ID (pings, enables the vehicle).
    #[arg(long)]
    mock_bridge: Option<u8>,
    /// Print every CAN frame on the bus, candump style.
    #[arg(long)]
    candump: bool,
    /// Pace virtual time to the wall clock (needed with host tools such as titan_bridge).
    #[arg(long)]
    realtime: bool,
    /// Wait for GDB on this address (e.g. `127.0.0.1:3333`) before running.
    #[arg(long)]
    gdb: Option<String>,
    /// Set one of the board's physical inputs, e.g. `sht41.temperature_c=31.5` (repeatable).
    #[arg(long = "input", value_parser = parse_input)]
    inputs: Vec<(String, f64)>,
    /// Acoustic environment (environments/*.toml) to put the board's transducer in.
    #[arg(long)]
    environment: Option<PathBuf>,
    /// Position of the board's transducer in the environment, `x,y,z` metres (z is depth).
    #[arg(long, value_parser = parse_position, default_value = "0,0,1")]
    position: [f64; 3],
    /// Write what the board's SSD1306 shows to this PNG file at the end.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Boot through the bootrom and boot2 (real flash path) instead of starting the image directly.
    #[arg(long)]
    rom_boot: bool,
    /// Keep the flash in this file: loaded (if it exists) before the image is written over it, saved at the end.
    #[arg(long)]
    flash_file: Option<PathBuf>,
    /// Carry out the bootrom's flash functions directly (high-level emulation) instead of running them against the
    /// QSPI flash model.
    #[arg(long)]
    hle_flash: bool,
    /// Power on with BOOTSEL held: the bootrom enters USB boot mode (PICOBOOT and the RPI-RP2 drive) instead of
    /// booting the flash. Implies `--rom-boot` and `--usb`.
    #[arg(long)]
    bootsel: bool,
    /// Plug the board's USB port into a host, which enumerates whatever the firmware presents.
    #[arg(long)]
    usb: bool,
    /// Open the device's USB CDC ACM serial port (pico-sdk stdio_usb) from the host: print its lines, or attach
    /// it to a pseudo terminal. Implies `--usb`.
    #[arg(long, value_enum)]
    usb_cdc: Option<CdcSink>,
    /// Once the board is in BOOTSEL mode, flash this image (ELF or UF2) into it and reboot into it, then run for
    /// `--time`. Implies `--usb`.
    #[arg(long)]
    picoboot_load: Option<PathBuf>,
    /// Export the board's USB device over USB/IP on this address (e.g. `127.0.0.1:3240`) for the real host's tools:
    /// `sudo usbip attach -r 127.0.0.1 -b 1-1`. Implies `--usb`; use with `--realtime`.
    #[arg(long)]
    usbip: Option<String>,
    /// How `--picoboot-load` gets the image onto the board.
    #[arg(long, value_enum, default_value = "picoboot")]
    load_via: LoadVia,
    /// For a board with Ethernet (Puddles): relay its UDP traffic through a host socket, `LOCAL=PEER`, e.g.
    /// `127.0.0.1:2203=127.0.0.1:2204` (titan_bridge listening on PEER, sending to LOCAL).
    #[arg(long)]
    udp: Option<String>,
    /// Inject a fault at a virtual time: `TIME@FAULT`, e.g. `1s@can-stuck`, `2s@i2c-absent:0x76`,
    /// `3s@bridge-stop` (repeatable; see `sim_host::fault::Fault`'s `from_str` for the faults).
    #[arg(long = "fault", value_parser = parse_fault)]
    faults: Vec<(Time, sim_host::fault::Fault)>,
    /// Profile the firmware: write the functions by time (per core) to FILE and folded stacks for flamegraph tools
    /// next to it (FILE with the extension `.folded`).
    #[arg(long)]
    profile: Option<PathBuf>,
    /// Mean virtual time between profile samples.
    #[arg(long, default_value = "10us", value_parser = parse_time)]
    profile_interval: Time,
}

fn parse_input(s: &str) -> Result<(String, f64), String> {
    let (k, v) = s.split_once('=').ok_or("expected NAME=VALUE")?;
    Ok((k.trim().to_string(), v.trim().parse().map_err(|e| format!("{v:?}: {e}"))?))
}

fn parse_position(s: &str) -> Result<[f64; 3], String> {
    let v: Vec<f64> =
        s.split(',').map(|x| x.trim().parse::<f64>()).collect::<Result<_, _>>().map_err(|e| e.to_string())?;
    v.try_into().map_err(|_| "expected x,y,z".to_string())
}

/// Parses `10ms`, `2s`, `250us`, `100ns` or a bare number of seconds.
fn parse_time(s: &str) -> Result<Time, String> {
    let s = s.trim();
    let (num, mult) = if let Some(n) = s.strip_suffix("ns") {
        (n, 1e-9)
    } else if let Some(n) = s.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1.0)
    };
    let v: f64 = num.trim().parse().map_err(|e| format!("bad time {s:?}: {e}"))?;
    if v < 0.0 {
        return Err("time must be positive".into());
    }
    Ok(Time::from_secs_f64(v * mult))
}

fn load_image(path: &std::path::Path) -> Result<Image> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    Image::parse(&bytes).with_context(|| format!("parsing {}", path.display()))
}

fn run(args: RunArgs) -> Result<()> {
    let image = match &args.image {
        Some(p) => load_image(p)?,
        None => Image::default(),
    };
    let to_load = match &args.picoboot_load {
        Some(p) => Some(load_image(p)?),
        None => None,
    };

    let config = MachineConfig {
        flash_size: args.flash_size,
        bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
        quantum: args.quantum,
        crystal_hz: args.crystal_hz,
    };
    let mut machine = Machine::new(config);
    machine.sys.unmodelled.log_mode = match args.unmodelled {
        UnmodelledLog::First => LogMode::FirstAccess,
        UnmodelledLog::Every => LogMode::Every,
        UnmodelledLog::Off => LogMode::Off,
    };
    if args.stop_on_bkpt {
        machine.bkpt_policy = BkptPolicy::Stop;
    }
    let categories: Vec<Category> = args
        .trace
        .iter()
        .map(|t| match t {
            TraceKind::Instr => Category::Instruction,
            TraceKind::Exc => Category::Exception,
            TraceKind::Gpio => Category::Gpio,
            TraceKind::Unmodelled => Category::Unmodelled,
            TraceKind::Device => Category::Device,
        })
        .collect();
    if !categories.is_empty() {
        let out: Box<dyn std::io::Write + Send> = match &args.trace_file {
            Some(p) => Box::new(std::io::BufWriter::new(
                std::fs::File::create(p).with_context(|| format!("creating {}", p.display()))?,
            )),
            None => Box::new(std::io::stderr()),
        };
        machine.set_trace(Box::new(WriterSink { categories, out, prefix: String::new() }));
    }

    let board = match &args.board {
        Some(p) => Some(sim_host::board::Board::load(p).map_err(anyhow::Error::msg)?),
        None => None,
    };
    if let Some(b) = &board {
        let inputs = b.apply(&mut machine).map_err(anyhow::Error::msg)?;
        log::info!(
            "board {} ({} SPI, {} I2C device(s); inputs: {})",
            b.name,
            machine.sys.spi_devices.len(),
            machine.sys.i2c_devices.len(),
            inputs.0.keys().cloned().collect::<Vec<_>>().join(", ")
        );
        for (k, v) in &args.inputs {
            if !inputs.0.contains_key(k) {
                bail!("{} has no input {k:?}", b.name);
            }
            inputs.set(k, *v);
        }
        if let Some(path) = &args.environment {
            let env = sim_acoustics::Environment::load(path).map_err(anyhow::Error::msg)?;
            log::info!("{} source(s) in {}", env.sources.len(), path.display());
            b.place_in(&mut machine, std::sync::Arc::new(env), args.position).map_err(anyhow::Error::msg)?;
        }
    } else if !args.inputs.is_empty() || args.environment.is_some() {
        bail!("--input and --environment need a --board");
    }
    machine.load(&image).map_err(anyhow::Error::msg)?;
    machine.hle_enabled = args.hle_flash;
    if let Some(f) = &args.flash_file {
        if machine.load_flash_file(f).with_context(|| format!("reading {}", f.display()))? {
            log::info!("flash contents from {}", f.display());
        }
        machine.load(&image).map_err(anyhow::Error::msg)?;
    }
    if args.bootsel {
        machine.sys.io_qspi.bootsel_pressed = true;
        machine.rom_boot().map_err(anyhow::Error::msg)?;
        log::info!("powering on with BOOTSEL held");
    } else if args.rom_boot {
        machine.rom_boot().map_err(anyhow::Error::msg)?;
        log::info!("booting through the bootrom");
    } else {
        let vtor = machine.direct_boot().map_err(anyhow::Error::msg)?;
        log::info!("direct boot: VTOR {vtor:#010x}, entry {:#010x}", machine.cores[0].pc());
    }

    if args.usb || args.bootsel || args.usb_cdc.is_some() || args.picoboot_load.is_some() || args.usbip.is_some() {
        let mut host = sim_devices::usb::host::HostController::new();
        if args.usb_cdc.is_some() {
            host.cdc_open(0, 115_200);
        }
        machine.sys.attach_usb_host(host, Time::ZERO);
        log::info!("USB port plugged into a host");
    }

    // RTT and defmt come from the image that ends up running
    let mut session = Session::new(machine, to_load.as_ref().unwrap_or(&image));
    session.keep_logs = false;
    session.on_log = Some(Box::new(|t, line| println!("[{t}] {line}")));
    let sinks = [args.uart0, args.uart1];
    session.on_uart_line = Some(Box::new(move |t, n, line| {
        if sinks[n] == UartSink::Stdout {
            println!("[{t}] uart{n}: {line}")
        }
    }));
    if let Some(addr) = &args.usbip {
        let server = sim_host::usbip::UsbIpServer::bind(addr.as_str()).with_context(|| format!("binding {addr}"))?;
        eprintln!("USB/IP: exporting bus ID {} on {addr}", sim_host::usbip::BUSID);
        session.usbip = Some(server);
    }
    match args.usb_cdc {
        Some(CdcSink::Stdout) => session.on_usb_cdc_line = Some(Box::new(|t, line| println!("[{t}] usb: {line}"))),
        Some(CdcSink::Pty) => {
            let pty = sim_host::pty::Pty::open().context("opening a pty")?;
            eprintln!("USB CDC port on {}", pty.path);
            session.usb_cdc_pty = Some(pty);
        }
        None => {}
    }
    for (n, sink) in sinks.iter().enumerate() {
        if *sink == UartSink::Pty {
            let pty = sim_host::pty::Pty::open().context("opening a pty")?;
            eprintln!("uart{n} on {}", pty.path);
            session.uart_pty[n] = Some(pty);
        }
    }
    if let Some(rtt) = &session.rtt {
        let defmt = if session.has_defmt() { ", decoding defmt" } else { "" };
        log::info!("RTT control block at {:#010x}{defmt}", rtt.addr);
    }

    let mut bridge_ctl: Option<Box<dyn sim_host::fault::BridgeControl>> = None;
    if let Some(rates) = board.as_ref().and_then(|b| b.can_rates()) {
        let mut bus = sim_devices::can::CanBus::new(args.can.as_deref().unwrap_or("can"), rates);
        if args.candump {
            bus.record = Some(Vec::new());
            session.on_can = Some(Box::new(|r| println!("[{}] {}", r.end, candump_line(r))));
        }
        session.can_bus = Some(bus);
        if let Some(iface) = &args.can {
            let sock = sim_host::socketcan::SocketCan::open(iface).with_context(|| format!("opening {iface}"))?;
            session.can_endpoints.push(Box::new(sock));
            log::info!("CAN bus bridged to {iface}");
        }
        if let Some(client) = args.mock_bridge {
            let bridge = sim_host::mock_bridge::MockBridge::new(client, rates.data.is_some(), sim_core::Time::ZERO);
            bridge.set_kill(Some(false));
            session.can_endpoints.push(Box::new(bridge.clone()));
            bridge_ctl = Some(Box::new(bridge));
            log::info!("mock bridge for client {client} attached");
        }
    } else if session.machine.sys.ethernet.is_some() {
        if args.mock_bridge.is_some() {
            let bridge = sim_host::ethernet::PuddlesBridge::new(Time::ZERO);
            bridge.set_kill(Some(false));
            session.eth_endpoints.push(Box::new(bridge.clone()));
            bridge_ctl = Some(Box::new(bridge));
            log::info!("mock bridge attached over Ethernet (UDP {})", sim_host::ethernet::LINK_PORT);
        }
        if let Some(spec) = &args.udp {
            let (local, peer) = spec.split_once('=').context("--udp LOCAL=PEER")?;
            let relay = sim_host::ethernet::UdpRelay::bind(local.parse()?, peer.parse()?)
                .with_context(|| format!("binding {local}"))?;
            session.eth_endpoints.push(Box::new(relay));
            log::info!("board UDP relayed: host {local} <-> {peer}");
        }
    } else if args.can.is_some() || args.mock_bridge.is_some() || args.udp.is_some() {
        bail!("--can and --mock-bridge need a --board with a CAN bus (or Ethernet), --udp one with Ethernet");
    }
    session.realtime = args.realtime;

    let wall = std::time::Instant::now();
    if let Some(addr) = &args.gdb {
        let listener = std::net::TcpListener::bind(addr).with_context(|| format!("binding {addr}"))?;
        log::info!("waiting for GDB on {addr}");
        if sim_host::gdb::serve(&listener, &mut session)? == sim_host::gdb::Ended::Killed {
            eprintln!("GDB ended the session at {}", session.machine.now);
            return Ok(());
        }
    }
    if let (Some(img), Some(path)) = (&to_load, &args.picoboot_load) {
        picoboot_load(&mut session, img, args.flash_size as u32, args.load_via)
            .with_context(|| format!("loading {}", path.display()))?;
    }
    let end = if to_load.is_some() { session.machine.now + args.time } else { args.time };
    if args.profile.is_some() {
        session.start_profile(args.profile_interval);
    }
    let mut faults = args.faults.clone();
    faults.sort_by_key(|f| f.0);
    let mut faults = faults.into_iter().peekable();
    let reason = loop {
        let next = faults.next_if(|f| f.0 <= end);
        let reason = session.run_until(next.as_ref().map_or(end, |f| f.0));
        let Some((_, fault)) = next.filter(|_| matches!(reason, StopReason::TimeLimit)) else { break reason };
        let n = session.machine.can_nodes().len();
        let mut bus = session.can_bus.take();
        let target = sim_host::fault::FaultTarget {
            session: &mut session,
            bus: bus.as_mut().map(|b| (b, 0..n)),
            bridge: bridge_ctl.as_deref(),
            power: None,
        };
        let applied = target.apply(&fault);
        session.can_bus = bus;
        if let Err(e) = applied {
            bail!("fault {fault:?}: {e}");
        }
    };
    let elapsed = wall.elapsed();
    session.flush_uarts();
    for (ch, bytes) in session.rtt_text.drain(..) {
        print!("[rtt{ch}] {}", String::from_utf8_lossy(&bytes));
    }
    if let Some(f) = &args.flash_file {
        session.machine.save_flash_file(f).with_context(|| format!("writing {}", f.display()))?;
    }
    if let (Some(path), Some(p)) = (&args.profile, &session.profiler) {
        let title = args.image.as_ref().or(args.picoboot_load.as_ref()).and_then(|p| p.file_name());
        let title = title.map_or("firmware".into(), |t| t.to_string_lossy().into_owned());
        let folded = p.report().write(path, &title).with_context(|| format!("writing {}", path.display()))?;
        eprintln!("profile: {} and {}", path.display(), folded.display());
    }
    if let Some(path) = &args.screenshot {
        match session.machine.sys.i2c_device::<sim_devices::ssd1306::Ssd1306>() {
            Some(oled) => {
                std::fs::write(path, oled.to_png(4)).with_context(|| format!("writing {}", path.display()))?;
                print!("{}", oled.render_text());
            }
            None => eprintln!("no SSD1306 on this board"),
        }
    }
    let machine = &mut session.machine;
    let where_ = |pc: u32| match machine.symbolize(pc) {
        Some(s) => {
            // `name+0x12`: demangle the name part
            let (name, off) = s.rsplit_once('+').filter(|(_, o)| o.starts_with("0x")).unwrap_or((&s, ""));
            let off = if off.is_empty() { String::new() } else { format!("+{off}") };
            format!("{pc:#010x} <{}{off}>", sim_host::demangle::demangle(name))
        }
        None => format!("{pc:#010x}"),
    };
    match reason {
        StopReason::TimeLimit => {}
        StopReason::Breakpoint { core, pc } => {
            eprintln!("core{core} stopped at breakpoint {}", where_(pc))
        }
        StopReason::BkptInstruction { core, pc, imm } => {
            eprintln!("core{core} BKPT {imm:#x} at {}", where_(pc))
        }
        StopReason::Lockup { core, pc } => eprintln!("core{core} locked up at {}", where_(pc)),
        StopReason::Deadlock => eprintln!("deadlock: every core sleeps and nothing is scheduled"),
        StopReason::Requested => {}
    }
    eprintln!(
        "stopped at {} ({:?}); core0 {} cycles, pc {}; {:.2?} wall",
        machine.now,
        reason,
        machine.cores[0].cycles(),
        where_(machine.cores[0].pc()),
        elapsed
    );
    if machine.core_enabled[1] {
        eprintln!(
            "core1 {} cycles, pc {}, {:?}",
            machine.cores[1].cycles(),
            where_(machine.cores[1].pc()),
            machine.cores[1].sleep_state()
        );
    }
    let summary = machine.sys.unmodelled.summary();
    if !summary.is_empty() {
        eprintln!("unmodelled registers touched:");
        for (name, reads, writes) in summary {
            eprintln!("  {name:32} {reads:>8} reads {writes:>8} writes");
        }
    }
    if matches!(reason, StopReason::Lockup { .. }) {
        bail!("core locked up");
    }
    Ok(())
}

/// Waits for the board to be in BOOTSEL mode, flashes `image` and reboots into it (BOOTSEL released).
fn picoboot_load(session: &mut Session, image: &Image, flash_size: u32, via: LoadVia) -> Result<()> {
    use sim_host::usb::{MscDrive, Picoboot};
    let t0 = session.machine.now;
    match via {
        LoadVia::Picoboot => {
            let mut pb = Picoboot::open(session, Time::from_secs(5))?;
            session.machine.sys.io_qspi.bootsel_pressed = false;
            let r = pb.load(session, image, flash_size, Some(100))?;
            for (a, n) in &r.ranges {
                log::info!("PICOBOOT: wrote and verified {n} bytes at {a:#010x}");
            }
        }
        LoadVia::Drive => {
            let mut d = MscDrive::open(session, Time::from_secs(5))?;
            session.machine.sys.io_qspi.bootsel_pressed = false;
            let uf2 = image.to_uf2();
            d.copy_uf2(session, &uf2, 0x1000)?;
            log::info!("copied {} UF2 blocks to the RPI-RP2 drive", uf2.len() / 512);
        }
    }
    log::info!("loaded in {} of virtual time; rebooting into it", session.machine.now - t0);
    Ok(())
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).format_timestamp(None).init();
    let cli = Cli::parse();
    match cli.command {
        Command::Run(args) => run(args),
        Command::Robot(args) => robot(args),
        Command::Picoboot(PicobootCommand::Load { file, run_args }) => {
            let mut argv: Vec<std::ffi::OsString> = vec!["firmware_simulator".into(), "run".into(), "--bootsel".into()];
            argv.push("--picoboot-load".into());
            argv.push(file.into());
            argv.extend(run_args.into_iter().map(Into::into));
            match Cli::parse_from(argv).command {
                Command::Run(args) => run(args),
                _ => unreachable!(),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_parsing() {
        assert_eq!(parse_time("10ms").unwrap(), Time::from_ms(10));
        assert_eq!(parse_time("2s").unwrap(), Time::from_secs(2));
        assert_eq!(parse_time("250us").unwrap(), Time::from_us(250));
        assert_eq!(parse_time("100ns").unwrap(), Time::from_ns(100));
        assert_eq!(parse_time("1.5").unwrap(), Time::from_ms(1500));
        assert!(parse_time("x").is_err());
    }

    #[test]
    fn fault_parsing() {
        use sim_host::fault::Fault;
        assert_eq!(parse_fault("1.5s@can-stuck").unwrap(), (Time::from_ms(1500), Fault::CanStuckDominant(true)));
        assert_eq!(
            parse_robot_fault("2s@camera_cage:can-unplug").unwrap(),
            (Time::from_secs(2), "camera_cage".to_string(), Fault::CanUnplugged(true))
        );
        assert!(parse_fault("can-stuck").is_err());
        assert!(parse_robot_fault("1s@can-stuck").is_err(), "the board is missing: `can-stuck` is not a fault");
    }
}
