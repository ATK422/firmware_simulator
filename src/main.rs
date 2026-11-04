//! `firmware_simulator`: runs RP2040 firmware images in the emulator.

use std::path::{Path, PathBuf};

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

mod calib_cli;
mod fuzz_cli;
mod host_tools;
mod mcp;
mod scenario_cli;

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
    /// Compares two CAN logs (hardware vs emulator, or two runs) by CANmore message: missing, extra and changed
    /// messages, and the timing of each message stream. Exits with 1 if they differ.
    CanDiff(host_tools::CanDiffArgs),
    /// Prints the titan_bridge command line for each board of a robot file (or a script or launch file).
    Bridges(host_tools::BridgesArgs),
    /// Connects a SocketCAN interface to an emulator's `--cannelloni` tunnel over UDP.
    CanTunnel(host_tools::TunnelArgs),
    /// What `picotool` does, against an emulated board in BOOTSEL mode.
    #[command(subcommand)]
    Picoboot(PicobootCommand),
    /// Serves the Model Context Protocol on stdin/stdout, for coding agents: tools to run the emulator, read a run's
    /// state, logs and CAN frames, and inject faults, set inputs and pause.
    Mcp(mcp::McpArgs),
    /// Runs YAML scenario files (a robot or board, faults, inputs and checks on logs, statuses, frames and states)
    /// as tests: prints each step with its time and evidence, writes JUnit XML or JSON, exits with 1 on a failure.
    Scenario(scenario_cli::ScenarioArgs),
    /// Runs scenarios with two firmware builds and reports what changed: checks, log lines, CAN traffic, boot and
    /// connect times, faults, resets, ESC speeds (Markdown and JSON).
    Compare(scenario_cli::CompareArgs),
    /// Fits the emulator's models to data recorded on the robot: the acoustic receive chain and environment
    /// (hydrophone WAVs), the thrust curve and ESC motor model (test-stand CSV), the vehicle's drag, added mass and
    /// buoyancy (pool-test logs).
    #[command(subcommand)]
    Calibrate(calib_cli::CalibrateCommand),
    /// Replays a pool test: what the robot computer sent in candump logs goes to the emulated boards at the times
    /// it was recorded, and what they send is compared with what the real boards sent (timing, missing and extra
    /// messages, payloads, faults). Exits with 1 if they differ.
    ReplayPoolTest(calib_cli::ReplayArgs),
    /// Fuzzes one or every board with malformed CAN traffic, watching for crashes, hangs, watchdog resets and
    /// fault states; writes minimised scenario reproducers (docs/fuzzing.md).
    Fuzz(fuzz_cli::FuzzArgs),
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
    /// Attach a scripted host side in the board's protocol: `PROTOCOL[:BUS][:BOARD][=CONFIG]` (repeatable).
    /// `nanoros:BOARD` is a nano-ROS agent (early: nano-ROS is still changing, see docs/nanoros.md) reading the
    /// board's `ros_toml` from the robot file, or `nanoros:BOARD=ros.toml` that file; `titan_link:BOARD` is
    /// `--mock-bridge` for the board's client. Each message from the board is printed with its topic.
    #[arg(long = "mock-agent", value_parser = parse_agent_spec)]
    mock_agents: Vec<sim_host::host_protocol::AgentSpec>,
    /// Bridge a bus to a SocketCAN interface: `BUS=IFACE`, e.g. `talos=vcan0` (repeatable).
    #[arg(long = "can", value_parser = parse_bus_iface)]
    cans: Vec<(String, String)>,
    /// Print every CAN frame, candump style, with its bus.
    #[arg(long)]
    candump: bool,
    /// Pace virtual time to the wall clock.
    #[arg(long)]
    realtime: bool,
    /// Run the boards on up to this many threads (0, the default: up to one per core; 1: in turn on one). Results
    /// are the same with any number; the boards are spread by what they cost, and only as many threads are used as
    /// make the slices shorter. Robots in an acoustic environment always run their boards in turn.
    #[arg(long, default_value_t = 0)]
    threads: usize,
    /// On threads, keep the boards in step slice by slice instead of letting them run ahead of each other between
    /// the slices where they can see each other (the results are the same; this only checks that they are).
    #[arg(long)]
    no_run_ahead: bool,
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
    /// Keep every ADC sample of each board's hydrophone and write them as WAV files at the end (`NAME` in the
    /// file name is replaced by the board's name): -1..1 is the ADC's 0-3.3 V, as `calibrate acoustics` reads it.
    #[arg(long, value_name = "FILE")]
    hydrophone_wav: Option<PathBuf>,
    /// The WAV files' sample rate.
    #[arg(long, default_value_t = 500_000)]
    hydrophone_wav_rate: u32,
    /// Write each vehicle's state every 10 ms to this CSV file (`NAME` in the file name is replaced by the
    /// vehicle's name when there are several).
    #[arg(long)]
    vehicle_csv: Option<PathBuf>,
    /// Let another simulator move this vehicle (repeatable; as `external = true` in the robot file): its own
    /// physics stop, and the dashboard's `POST /api/vehicle` puts it where the other simulator has it, its sensors
    /// and transducers following. `GET /api/thrusters` gives the thrust its ESCs make. Needs `--dashboard`.
    #[arg(long = "external-vehicle")]
    external_vehicles: Vec<String>,
    /// Inject a fault on a board at a virtual time: `TIME@BOARD:FAULT`, e.g. `1.5s@camera_cage:can-unplug`
    /// (repeatable; see `sim_host::fault::Fault`'s `from_str` for the faults). Bridge faults act on the first mock
    /// bridge on the board's bus.
    #[arg(long = "fault", value_parser = parse_robot_fault)]
    faults: Vec<(Time, String, sim_host::fault::Fault)>,
    /// Plug a board's USB port into a host that opens its CDC ACM port (pico-sdk stdio_usb, titan_dual_stdio_usb's
    /// text log): `BOARD=stdout` prints its lines prefixed with the board's name, `BOARD=pty` puts it on a pseudo
    /// terminal, `BOARD=pty:PATH` also links PATH to it (repeatable; replaces the robot file's `usb_cdc`).
    #[arg(long = "usb-cdc", value_parser = parse_board_cdc_sink)]
    usb_cdcs: Vec<(String, CdcSink)>,
    /// Open a board's second CDC port (titan_dual_stdio_usb's secondary port, which titan_firmware's micro-ROS USB
    /// transport uses) on a pseudo terminal: `BOARD=pty` or `BOARD=pty:PATH` (repeatable; replaces the robot file's
    /// `usb_cdc2`).
    #[arg(long = "usb-cdc2", value_parser = parse_board_cdc_sink)]
    usb_cdc2s: Vec<(String, CdcSink)>,
    /// Make an IVC board transmit a byte acoustically at a virtual time: `TIME@BOARD:BYTE`, e.g. `4s@talos_ivc:0x5a`
    /// (repeatable). The board's mock bridge sends `Ivc2(EnqueueData)` to an `ivc_board2` image and `Ivc(Tx)` to an
    /// `ivc_board` image, so the board needs `--mock-bridge` on its bus. Every mock bridge prints the IVC bytes its
    /// board receives (`[t] BOARD: IVC received 0x5a`).
    #[arg(long = "ivc-send", value_parser = parse_ivc_send)]
    ivc_sends: Vec<(Time, String, u8)>,
    #[command(flatten)]
    can_host: host_tools::CanHostArgs,
    /// Trace categories to print for the boards (comma separated), each line prefixed with the board's name.
    #[arg(long, value_delimiter = ',')]
    trace: Vec<TraceKind>,
    /// Only trace these boards (comma separated names; all by default).
    #[arg(long, value_delimiter = ',')]
    trace_board: Vec<String>,
    /// Write the trace to this file instead of stderr.
    #[arg(long)]
    trace_file: Option<PathBuf>,
    /// Profile every board's firmware: write `DIR/BOARD.txt` (functions by time, per core) and `DIR/BOARD.folded`
    /// (folded stacks for flamegraph tools).
    #[arg(long)]
    profile_dir: Option<PathBuf>,
    /// Mean virtual time between profile samples.
    #[arg(long, default_value = "10us", value_parser = parse_time)]
    profile_interval: Time,
    /// Write a waveform (VCD, for GTKWave, Surfer or PulseView) of the GPIOs to this file.
    #[arg(long)]
    vcd: Option<PathBuf>,
    /// GPIOs in the waveform: numbers, ranges and names from the board file's `[pins]` table, e.g.
    /// `0-7,25,status_ledr` (default all).
    #[arg(long, default_value = "all")]
    vcd_pins: String,
    /// Add decoded events to the waveform as string variables: defmt logs, UART lines, CAN frames, device reports
    /// (GTKWave and Surfer show them; PulseView reads only the pins).
    #[arg(long)]
    vcd_events: bool,
    /// Serve a live dashboard on this address (e.g. `127.0.0.1:8080`): logs, firmware status, CAN traffic, pins,
    /// inputs, vehicles, and controls to inject faults, set inputs and pause.
    #[arg(long)]
    dashboard: Option<String>,
    /// Keep serving the dashboard after the run ends (until interrupted).
    #[arg(long, requires = "dashboard")]
    dashboard_hold: bool,
    /// Pause virtual time when it reaches this (`0`: from the start, within the first 10 ms); resume from the
    /// dashboard.
    #[arg(long, requires = "dashboard", value_parser = parse_time)]
    dashboard_pause_at: Option<Time>,
    /// Write every log line, UART line, CAN frame and fault as one JSON object per line to this file, for tools.
    #[arg(long)]
    log_json: Option<PathBuf>,
    /// How to log the boards' accesses to registers the emulator does not model.
    #[arg(long, value_enum, default_value = "summary")]
    unmodelled: UnmodelledLog,
    /// Stop the run when a core of any board locks up, instead of keeping the board running (as the chip does)
    /// until its watchdog resets it.
    #[arg(long)]
    stop_on_lockup: bool,
    /// Debug one board with GDB: `BOARD=ADDR`, e.g. `power=127.0.0.1:3333`. Waits for GDB before running; while it
    /// is attached, continuing runs the whole robot (its boards in turn) and the board's breakpoints stop it all.
    /// `--fault`s due meanwhile are applied once GDB detaches.
    #[arg(long, value_parser = parse_board_addr)]
    gdb: Option<(String, String)>,
    /// Time travel (reverse debugging in GDB, `POST /api/rewind`): record the run with a checkpoint this often in
    /// virtual time; 0 turns it off. On by default (10ms) with `--gdb` and `--dashboard`, off otherwise.
    #[arg(long, value_parser = parse_time)]
    checkpoint_every: Option<Time>,
    /// Memory the time-travel checkpoints may take, in MB; older ones are thinned out beyond it.
    #[arg(long, default_value_t = 512)]
    checkpoint_memory: usize,
    /// Collect firmware code coverage and write it to this lcov file (`genhtml`, VS Code Coverage Gutters) at the
    /// end, with a table of functions and lines hit per crate. The cores run one instruction at a time.
    #[arg(long, value_name = "FILE")]
    coverage: Option<PathBuf>,
}

/// Starts time travel for `target` when asked (`--checkpoint-every` above 0) or on by default (`default_on`: GDB or a
/// dashboard), unless the profiler is on (it cuts runs at its sample times, which replays do not do), in which case
/// only when asked.
fn time_travel(
    target: &mut dyn sim_host::timetravel::Rewindable,
    inputs: Vec<sim_host::board::Inputs>,
    every: Option<Time>,
    memory_mb: usize,
    default_on: bool,
    profiling: bool,
) -> Option<sim_host::timetravel::TimeTravel> {
    let on = match every {
        Some(t) => t > Time::ZERO,
        None if profiling && default_on => {
            log::info!("time travel is off with the profiler (--checkpoint-every turns it on)");
            false
        }
        None => default_on,
    };
    if !on {
        return None;
    }
    let mut tt = sim_host::timetravel::TimeTravel::new(target, inputs);
    tt.every = every.unwrap_or(sim_host::timetravel::DEFAULT_EVERY);
    tt.budget = memory_mb << 20;
    log::info!("time travel: a checkpoint every {} (up to {memory_mb} MB)", tt.every);
    Some(tt)
}

/// What a time-travel run's end means to a run loop.
fn travel_reason(r: sim_host::timetravel::Ran) -> StopReason {
    match r {
        sim_host::timetravel::Ran::Reached => StopReason::TimeLimit,
        sim_host::timetravel::Ran::Interrupted => StopReason::Requested,
        sim_host::timetravel::Ran::Stopped { reason, .. } => reason,
    }
}

/// Writes what ran of each image (`machines`: image name and machine) to the lcov file `path` and prints the
/// table of functions and lines hit.
fn write_coverage<'a>(path: &Path, machines: impl Iterator<Item = (&'a str, &'a Machine)>) -> Result<()> {
    let mut cov = sim_host::coverage::Coverage::new();
    for (name, m) in machines {
        cov.add_machine(name, m);
    }
    let report = cov.report();
    let name = cov.names().join(",");
    report.write_lcov(path, &name).with_context(|| format!("writing {}", path.display()))?;
    eprint!("coverage of {name}:\n{}", report.summary());
    eprintln!("coverage: {}", path.display());
    Ok(())
}

fn parse_board_addr(s: &str) -> Result<(String, String), String> {
    let (board, addr) = s.split_once('=').ok_or("expected BOARD=ADDR")?;
    Ok((board.to_string(), addr.to_string()))
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

fn parse_agent_spec(s: &str) -> Result<sim_host::host_protocol::AgentSpec, String> {
    s.parse()
}

/// Prints each message an agent gets, `[t] NAME <- TOPIC {json}`.
fn print_agent_messages(agent: &dyn sim_host::host_protocol::HostAgent, name: String) {
    agent.on_message(Box::new(move |m| println!("[{}] {name} <- {} {}", m.t, m.topic, m.msg)));
}

/// What a mock agent saw, at the end of a run.
fn agent_summary(name: &str, agent: &dyn sim_host::host_protocol::HostAgent, now: Time) {
    let errors = agent.errors();
    let st = agent.status(now);
    eprintln!(
        "{name}: {} agent {}, {} message(s) from the board, {} sent, {} error(s)",
        agent.protocol(),
        if agent.connected(now) { "connected" } else { "not connected" },
        agent.message_count(),
        st["sent"].as_u64().unwrap_or(0),
        st["decode_errors"].as_u64().unwrap_or(errors.len() as u64)
    );
    for e in errors.iter().take(5) {
        eprintln!("  {e}");
    }
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
        s.machine.sys.unmodelled.log_mode = args.unmodelled.mode();
        s.stop_on_lockup = args.stop_on_lockup;
        s.keep_logs = false;
        let n = name.clone();
        s.on_log = Some(Box::new(move |t, line| println!("[{t}] {n}: {line}")));
        s.on_uart_line = Some(Box::new(move |t, u, line| println!("[{t}] {name} uart{u}: {line}")));
    }
    for (port, sinks) in [(0, &args.usb_cdcs), (1, &args.usb_cdc2s)] {
        for (board, sink) in sinks {
            let opt = if port == 0 { "--usb-cdc" } else { "--usb-cdc2" };
            let i = r.index(board).with_context(|| format!("{opt}: no board {board:?}"))?;
            r.set_usb_cdc(i, port, sink.clone()).map_err(anyhow::Error::msg)?;
        }
    }
    for i in 0..r.boards.len() {
        if r.usb[i].cdc == Some(CdcSink::Stdout) {
            let name = r.names[i].clone();
            r.boards[i].on_usb_cdc_line = Some(Box::new(move |t, line| println!("[{t}] {name} usb: {line}")));
        }
        if r.usb[i].plugged {
            log::info!("{}: USB port plugged into a host", r.names[i]);
        }
    }
    for line in r.open_usb_ptys().map_err(anyhow::Error::msg)? {
        eprintln!("{line}");
    }
    let mut mock_bridges = args.mock_bridges.clone();
    let mut agents = Vec::new();
    for spec in &args.mock_agents {
        if spec.protocol == sim_host::host_protocol::Protocol::TitanLink {
            let p = sim_host::host_protocol::place(&r, spec).map_err(anyhow::Error::msg)?;
            mock_bridges.push((r.bus_names[p.bus].clone(), p.client));
            continue;
        }
        let (p, agent) = sim_host::host_protocol::attach(&mut r, spec, Path::new(".")).map_err(anyhow::Error::msg)?;
        let name = p.board.map_or_else(|| format!("{}:{}", r.bus_names[p.bus], p.client), |i| r.names[i].clone());
        print_agent_messages(&*agent, name.clone());
        log::info!("{} mock agent for client {} on {}", agent.protocol(), p.client, r.bus_names[p.bus]);
        agents.push((p, name, agent));
    }
    let mut bridges: Vec<Option<sim_host::mock_bridge::MockBridge>> = vec![None; r.buses.len()];
    let mut all_bridges = Vec::new();
    for (bus, client) in &mock_bridges {
        let j = r.bus_names.iter().position(|n| n == bus).with_context(|| format!("no bus {bus:?}"))?;
        let b = sim_host::mock_bridge::MockBridge::new(*client, r.buses[j].bus.rates.data.is_some(), Time::ZERO);
        b.set_kill(Some(false));
        // Named after the board it serves
        let name = (0..r.boards.len())
            .find(|&i| r.board_bus[i] == Some(j) && r.client_ids[i] == Some(*client))
            .map_or_else(|| format!("{bus}:{client} bridge"), |i| r.names[i].clone());
        b.on_message(move |t, m| {
            if let Some(byte) = sim_host::mock_bridge::ivc_received(m) {
                println!("[{t}] {name}: IVC received {byte:#04x}");
            }
        });
        let rb = &mut r.buses[j];
        rb.endpoints.push(Box::new(b.clone()));
        all_bridges.push((j, *client, b.clone()));
        bridges[j].get_or_insert(b);
        log::info!("mock bridge for client {client} on {bus}");
    }
    for (t, board, byte) in &args.ivc_sends {
        let i = r.index(board).with_context(|| format!("--ivc-send: no board {board:?}"))?;
        let modem = sim_host::mock_bridge::IvcModem::from_image(&r.images[i]).with_context(|| {
            format!("--ivc-send: {board} runs {}, not an IVC image (ivc_board or ivc_board2)", r.images[i])
        })?;
        let (j, client) = (r.board_bus[i], r.client_ids[i]);
        let bridge = all_bridges.iter().find(|(jj, c, _)| Some(*jj) == j && Some(*c) == client).map(|x| &x.2);
        let Some(bridge) = bridge else {
            let hint = match (j, client) {
                (Some(j), Some(c)) => format!(" (add --mock-bridge {}:{c})", r.bus_names[j]),
                _ => String::new(),
            };
            bail!("--ivc-send: {board} has no mock bridge{hint}");
        };
        bridge.send_at(*t, modem.transmit(*byte));
        log::info!("{board} transmits {byte:#04x} at {t} ({modem:?})");
    }
    let mut faults = Vec::new();
    for (t, board, fault) in args.faults {
        let i = r.index(&board).with_context(|| format!("--fault: no board {board:?}"))?;
        faults.push((t, i, fault));
    }
    faults.sort_by_key(|f| f.0);
    // When each board's last fault that restarts the chip is due: a lockup no watchdog will end lasts until then
    let mut restart_by = vec![None; r.boards.len()];
    for (t, i, f) in &faults {
        if restarts_the_chip(f) {
            restart_by[*i] = Some(*t);
        }
    }
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
    let mut eth_bridges = Vec::new();
    for name in &args.eth_mock_bridges {
        let i = r.index(name).with_context(|| format!("no board {name:?}"))?;
        let s = &mut r.boards[i];
        anyhow::ensure!(s.machine.sys.ethernet.is_some(), "{name} has no Ethernet");
        let bridge = sim_host::ethernet::PuddlesBridge::new(Time::ZERO);
        bridge.set_kill(Some(false));
        s.eth_endpoints.push(Box::new(bridge.clone()));
        eth_bridges.push((i, bridge));
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
    for name in &args.external_vehicles {
        let names: Vec<String> = r.vehicles.iter().map(|v| v.vehicle.name.clone()).collect();
        let v = r.vehicle(name).with_context(|| {
            format!("--external-vehicle: no vehicle {name:?} (the robot has: {})", names.join(", "))
        })?;
        v.external = true;
    }
    for v in &r.vehicles {
        if v.external {
            log::info!("vehicle {} is moved externally (POST /api/vehicle)", v.vehicle.name);
            anyhow::ensure!(
                args.dashboard.is_some(),
                "vehicle {} is external: only the dashboard's POST /api/vehicle can move it, so add --dashboard ADDR",
                v.vehicle.name
            );
        }
    }
    let can_host = host_tools::attach_robot(&args.can_host, &mut r, &args.cans)?;
    if args.coverage.is_some() {
        for b in r.boards.iter_mut() {
            b.machine.enable_coverage();
        }
    }
    r.realtime = args.realtime;
    r.threads = args.threads;
    r.run_ahead = !args.no_run_ahead;
    if !args.trace.is_empty() {
        for b in &args.trace_board {
            anyhow::ensure!(r.index(b).is_some(), "--trace-board: no board {b:?}");
        }
        let out = open_trace(args.trace_file.as_deref())?;
        let categories = trace_categories(&args.trace);
        for (s, name) in r.boards.iter_mut().zip(&r.names) {
            if args.trace_board.is_empty() || args.trace_board.contains(name) {
                let sink = WriterSink { categories: categories.clone(), out: out.clone(), prefix: format!("{name}: ") };
                let old = std::mem::replace(&mut s.machine.trace, Box::new(sim_core::trace::NullSink));
                s.machine.set_trace(sim_core::trace::MultiSink::combine(old, Box::new(sink)));
            }
        }
    }
    let vcd = match &args.vcd {
        Some(path) => {
            let mut scopes = Vec::new();
            for (i, name) in r.names.iter().enumerate() {
                let pins = sim_host::vcd::parse_pins(&args.vcd_pins, &r.pins[i])
                    .map_err(|e| anyhow::anyhow!("{name}: {e}"))?;
                let opts = sim_host::vcd::VcdOptions { pins, events: args.vcd_events };
                scopes.push(sim_host::vcd::Scope::board(name, &r.pins[i], &opts));
            }
            if args.vcd_events {
                scopes.extend(r.bus_names.iter().map(|b| sim_host::vcd::Scope::bus(b)));
            }
            let vcd =
                sim_host::vcd::Vcd::create(path, &scopes).with_context(|| format!("creating {}", path.display()))?;
            for (i, s) in r.boards.iter_mut().enumerate() {
                vcd.attach(i, s);
            }
            if args.vcd_events {
                let n = r.boards.len();
                for (j, rb) in r.buses.iter_mut().enumerate() {
                    vcd.attach_bus(n + j, rb);
                }
            }
            Some(vcd)
        }
        None => None,
    };
    if let Some(dir) = &args.profile_dir {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        for s in r.boards.iter_mut() {
            s.start_profile(args.profile_interval);
        }
    }
    let mut dash = match &args.dashboard {
        Some(addr) => {
            let infos = (0..r.boards.len())
                .map(|i| sim_host::dashboard::BoardInfo {
                    name: r.names[i].clone(),
                    image: r.images[i].clone(),
                    inputs: r.inputs[i].clone(),
                    pins: r.pins[i].clone(),
                    fault_names: Vec::new(),
                })
                .collect();
            let mut d = sim_host::dashboard::Dashboard::start(addr, infos, r.bus_names.clone())
                .with_context(|| format!("binding {addr}"))?;
            d.pause_at(args.dashboard_pause_at);
            for (i, s) in r.boards.iter_mut().enumerate() {
                d.tap_session(i, s);
            }
            for (j, rb) in r.buses.iter_mut().enumerate() {
                d.tap_bus(j, rb);
            }
            for (p, _, a) in &agents {
                d.add_agent(p.board, a.handle());
            }
            for (bus, client) in &mock_bridges {
                let j = r.bus_names.iter().position(|n| n == bus).unwrap();
                let board =
                    (0..r.boards.len()).find(|&i| r.board_bus[i] == Some(j) && r.client_ids[i] == Some(*client));
                if let Some(b) = all_bridges.iter().find(|(jj, c, _)| *jj == j && c == client).map(|x| x.2.clone()) {
                    d.add_link(board, Some(j), sim_host::dashboard::Link::Can(b));
                }
            }
            for (i, b) in &eth_bridges {
                d.add_link(Some(*i), None, sim_host::dashboard::Link::Eth(b.clone()));
            }
            eprintln!("dashboard on http://{}/", d.addr);
            Some(d)
        }
        None => None,
    };
    let json_log = match &args.log_json {
        Some(path) => {
            let log =
                sim_host::jsonlog::JsonLog::create(path).with_context(|| format!("creating {}", path.display()))?;
            for (s, name) in r.boards.iter_mut().zip(&r.names) {
                log.tap_session(name, None, s);
            }
            for (rb, name) in r.buses.iter_mut().zip(&r.bus_names) {
                log.tap_bus(name, rb);
            }
            Some(log)
        }
        None => None,
    };
    if args.hydrophone_wav.is_some() {
        let n: usize = r.boards.iter_mut().map(|s| sim_host::acoustic::record_hydrophones(&mut s.machine)).sum();
        if n == 0 {
            bail!("--hydrophone-wav: no board of this robot has a hydrophone in an environment");
        }
    }
    let wall = std::time::Instant::now();
    let mut faults = faults.into_iter().peekable();
    let mut pending: Option<(Time, usize, sim_host::fault::Fault)> = None;
    r.stop_flag = Some(&sim_host::signal::STOP);
    let inputs = r.inputs.clone();
    let default_on = args.gdb.is_some() || dash.is_some();
    let profiling = args.profile_dir.is_some();
    let mut tt = time_travel(&mut r, inputs, args.checkpoint_every, args.checkpoint_memory, default_on, profiling);
    if let Some((board, addr)) = &args.gdb {
        let i = r.index(board).with_context(|| format!("--gdb: no board {board:?}"))?;
        let listener = std::net::TcpListener::bind(addr).with_context(|| format!("binding {addr}"))?;
        log::info!("waiting for GDB on {addr} (board {board})");
        let ended = match &mut tt {
            Some(tt) => {
                let mut target = sim_host::gdb::Travel { tt, target: &mut r, board: i };
                sim_host::gdb::serve_target(&listener, &mut target)?
            }
            None => {
                let mut target = sim_host::gdb::RobotBoard { robot: &mut r, board: i };
                sim_host::gdb::serve_target(&listener, &mut target)?
            }
        };
        if ended == sim_host::gdb::Ended::Killed {
            eprintln!("GDB ended the session at {}", r.now);
            return Ok(());
        }
    }
    if let Some(d) = &mut dash {
        d.travel = tt.take();
    }
    let stopped = loop {
        if pending.is_none() {
            pending = faults.next_if(|f| f.0 <= args.time);
        }
        let target = pending.as_ref().map_or(args.time, |f| f.0);
        let until = if dash.is_some() { target.min(r.now + DASHBOARD_PIECE) } else { target };
        let stopped = match (&mut dash, &mut tt) {
            (Some(d), _) => d.run_robot(&mut r, until),
            (None, Some(tt)) => match tt.run_until(&mut r, until) {
                sim_host::timetravel::Ran::Stopped { board, reason } => Some((board, reason)),
                _ => None,
            },
            (None, None) => r.run_until(until),
        };
        if let Some(d) = &mut dash {
            d.service_robot(&mut r, &bridges);
            while d.paused() && !sim_host::signal::requested() {
                std::thread::sleep(std::time::Duration::from_millis(20));
                d.service_robot(&mut r, &bridges);
            }
        }
        if let Some((i, StopReason::Lockup { .. })) = stopped
            && !args.stop_on_lockup
            && restart_by[i].is_some_and(|t| t >= r.now)
        {
            // The board stays locked up until a scheduled fault restarts it; the others go on
            log::warn!("[{}] {} stays locked up until a fault restarts it", r.now, r.names[i]);
            continue;
        }
        if let Some(s) = stopped {
            break Some(s);
        }
        if sim_host::signal::requested() {
            break None;
        }
        if until < target {
            continue;
        }
        let Some((_, i, fault)) = pending.take() else { break None };
        let bridge = r.board_bus[i].and_then(|j| bridges[j].clone());
        let ctl = bridge.as_ref().map(|b| b as &dyn sim_host::fault::BridgeControl);
        let applied = sim_host::fault::robot_target(&mut r, i, ctl).apply(&fault);
        if let Some(tt) = dash.as_mut().and_then(|d| d.travel.as_mut()).or(tt.as_mut()) {
            tt.changed(&mut r);
        }
        if let Some(d) = &mut dash {
            let name = r.names[i].clone();
            d.note_fault(r.now, &name, &format!("{fault:?}"), &applied);
        }
        if let Some(l) = &json_log {
            l.fault(r.now, &r.names[i], &format!("{fault:?}"), &applied);
        }
        if let Err(e) = applied {
            bail!("fault {fault:?} on {}: {e}", r.names[i]);
        }
    };
    let why = match &stopped {
        Some((i, why)) => format!("{} stopped: {why:?}", r.names[*i]),
        None if sim_host::signal::requested() => "Interrupted".into(),
        None => "TimeLimit".into(),
    };
    if let Some(d) = &mut dash {
        d.finish_robot(&mut r, &why);
    }
    for s in r.boards.iter_mut() {
        s.flush_uarts();
    }
    can_host.finish()?;
    let interrupted = if why == "Interrupted" { " (Interrupted)" } else { "" };
    eprintln!("stopped at {}{interrupted}; {:.2?} wall", r.now, wall.elapsed());
    for (_, name, a) in &agents {
        agent_summary(name, &**a, r.now);
    }
    if let Some(path) = &args.coverage {
        write_coverage(path, r.images.iter().map(String::as_str).zip(r.boards.iter().map(|b| &b.machine)))?;
    }
    if let (Some(v), Some(path)) = (&vcd, &args.vcd) {
        finish_vcd(v, path, r.now)?;
    }
    if let (Some(l), Some(path)) = (&json_log, &args.log_json) {
        l.finish(r.now, &why).with_context(|| format!("writing {}", path.display()))?;
    }
    for (s, name) in r.boards.iter_mut().zip(&r.names) {
        if s.lockups > 0 {
            eprintln!("{name}: {}", lockup_note(s));
        }
        for note in sim_host::acoustic::clipping_notes(&mut s.machine) {
            eprintln!("{name}: {note}");
        }
    }
    if let Some(path) = &args.hydrophone_wav {
        let several = r
            .boards
            .iter_mut()
            .map(|s| sim_host::acoustic::hydrophones(&mut s.machine).next().is_some())
            .filter(|h| *h)
            .count()
            > 1;
        for (s, name) in r.boards.iter_mut().zip(&r.names) {
            if sim_host::acoustic::hydrophones(&mut s.machine).next().is_none() {
                continue;
            }
            let file = path.to_string_lossy();
            let p = if file.contains("NAME") {
                PathBuf::from(file.replace("NAME", name))
            } else if several {
                let stem = path.file_stem().map(|x| x.to_string_lossy().into_owned()).unwrap_or_default();
                path.with_file_name(format!("{stem}_{name}.wav"))
            } else {
                path.clone()
            };
            let lines = sim_host::acoustic::write_hydrophone_wavs(&mut s.machine, &p, args.hydrophone_wav_rate)
                .with_context(|| format!("writing {}", p.display()))?;
            for l in lines {
                eprintln!("{name}: {l}");
            }
        }
    }
    if args.unmodelled != UnmodelledLog::Off {
        for (s, name) in r.boards.iter().zip(&r.names) {
            let summary = s.machine.sys.unmodelled.summary();
            if !summary.is_empty() {
                let regs: Vec<String> =
                    summary.iter().map(|(reg, reads, writes)| format!("{reg} ({reads}r/{writes}w)")).collect();
                eprintln!("{name}: unmodelled registers touched: {}", regs.join(", "));
            }
        }
    }
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
    if let Some(d) = &mut dash
        && args.dashboard_hold
        && !sim_host::signal::requested()
    {
        eprintln!("run finished; the dashboard keeps serving on http://{}/ (Ctrl-C to quit)", d.addr);
        sim_host::signal::exit_on_signal();
        d.hold();
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

fn trace_categories(kinds: &[TraceKind]) -> Vec<Category> {
    kinds
        .iter()
        .map(|t| match t {
            TraceKind::Instr => Category::Instruction,
            TraceKind::Exc => Category::Exception,
            TraceKind::Gpio => Category::Gpio,
            TraceKind::Unmodelled => Category::Unmodelled,
            TraceKind::Device => Category::Device,
        })
        .collect()
}

/// Where a trace goes: `--trace-file` (buffered, and flushed every 100 ms so it can be followed) or stderr.
fn open_trace(path: Option<&Path>) -> Result<sim_host::flush::SharedWriter> {
    sim_host::flush::SharedWriter::file_or_stderr(path)
        .with_context(|| format!("creating {}", path.map_or_else(String::new, |p| p.display().to_string())))
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

use sim_host::usb::CdcSink;

fn parse_cdc_sink(s: &str) -> Result<CdcSink, String> {
    s.parse()
}

/// `BOARD=SINK` for a robot board's USB CDC port.
fn parse_board_cdc_sink(s: &str) -> Result<(String, CdcSink), String> {
    let (board, sink) = s.split_once('=').ok_or("expected BOARD=SINK (SINK: stdout, pty or pty:PATH)")?;
    Ok((board.to_string(), sink.parse()?))
}

/// `TIME@BOARD:BYTE`, e.g. `4s@talos_ivc:0x5a`.
fn parse_ivc_send(s: &str) -> Result<(Time, String, u8), String> {
    let (at, rest) = s.split_once('@').ok_or("expected TIME@BOARD:BYTE")?;
    let (board, byte) = rest.rsplit_once(':').ok_or("expected TIME@BOARD:BYTE")?;
    let byte = match byte.strip_prefix("0x").or_else(|| byte.strip_prefix("0X")) {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => byte.parse(),
    }
    .map_err(|e| format!("byte {byte:?}: {e} (0-255 or 0x00-0xff)"))?;
    Ok((parse_time(at)?, board.to_string(), byte))
}

/// Opens a pseudo terminal and, if asked, points a symlink at it (replacing an earlier run's symlink).
fn open_pty(link: Option<&Path>, what: &str) -> Result<sim_host::pty::Pty> {
    let pty = sim_host::pty::Pty::open_linked(link).context("opening a pty")?;
    match link {
        Some(link) => eprintln!("{what} on {} (linked from {})", pty.path, link.display()),
        None => eprintln!("{what} on {}", pty.path),
    }
    Ok(pty)
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
enum UnmodelledLog {
    /// One note at the first access, and a table of every unmodelled register touched at the end.
    Summary,
    /// A warning at the first access to each register, and the table.
    First,
    /// A warning at every access, and the table.
    Every,
    /// Nothing.
    Off,
}

impl UnmodelledLog {
    fn mode(self) -> LogMode {
        match self {
            UnmodelledLog::Summary => LogMode::Summary,
            UnmodelledLog::First => LogMode::FirstAccess,
            UnmodelledLog::Every => LogMode::Every,
            UnmodelledLog::Off => LogMode::Off,
        }
    }
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
    /// How to log accesses to registers the emulator does not model.
    #[arg(long, value_enum, default_value = "summary")]
    unmodelled: UnmodelledLog,
    /// Flash size in bytes (default: the board file's `flash_size`, else 2 MiB).
    #[arg(long)]
    flash_size: Option<usize>,
    /// Crystal (XOSC) frequency.
    #[arg(long, default_value_t = 12_000_000)]
    crystal_hz: u64,
    /// Stop on BKPT instructions instead of escalating them to HardFault.
    #[arg(long)]
    stop_on_bkpt: bool,
    /// Stop the run when a core locks up, instead of keeping the board running (as the chip does) until the
    /// watchdog resets it.
    #[arg(long)]
    stop_on_lockup: bool,
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
    /// Attach a scripted host side in the firmware's protocol: `nanoros[:CLIENT][=ros.toml]`, a nano-ROS agent
    /// (early: nano-ROS is still changing, see docs/nanoros.md) for the board file's client (or CLIENT) with the
    /// firmware's `ros.toml`; `titan_link[:CLIENT]` is `--mock-bridge`. Each message from the board is printed.
    #[arg(long, value_parser = parse_agent_spec)]
    mock_agent: Option<sim_host::host_protocol::AgentSpec>,
    /// Print every CAN frame on the bus, candump style.
    #[arg(long)]
    candump: bool,
    /// Pace virtual time to the wall clock (needed with host tools such as titan_bridge).
    #[arg(long)]
    realtime: bool,
    /// Wait for GDB on this address (e.g. `127.0.0.1:3333`) before running.
    #[arg(long)]
    gdb: Option<String>,
    /// Time travel (reverse debugging in GDB, `POST /api/rewind`): record the run with a checkpoint this often in
    /// virtual time; 0 turns it off. On by default (10ms) with `--gdb` and `--dashboard`, off otherwise.
    #[arg(long, value_parser = parse_time)]
    checkpoint_every: Option<Time>,
    /// Memory the time-travel checkpoints may take, in MB; older ones are thinned out beyond it.
    #[arg(long, default_value_t = 512)]
    checkpoint_memory: usize,
    /// Collect firmware code coverage and write it to this lcov file (`genhtml`, VS Code Coverage Gutters) at the
    /// end, with a table of functions and lines hit per crate. The cores run one instruction at a time.
    #[arg(long, value_name = "FILE")]
    coverage: Option<PathBuf>,
    /// Set one of the board's physical inputs, e.g. `sht41.temperature_c=31.5` (repeatable).
    #[arg(long = "input", value_parser = parse_input)]
    inputs: Vec<(String, f64)>,
    /// Acoustic environment (environments/*.toml) to put the board's transducer in.
    #[arg(long)]
    environment: Option<PathBuf>,
    /// Position of the board's transducer in the environment, `x,y,z` metres (z is depth).
    #[arg(long, value_parser = parse_position, default_value = "0,0,1")]
    position: [f64; 3],
    /// Keep every ADC sample of the board's hydrophone (with `--environment`) and write them to this WAV file at
    /// the end: -1..1 is the ADC's 0-3.3 V, as `calibrate acoustics` reads a board recording.
    #[arg(long, value_name = "FILE")]
    hydrophone_wav: Option<PathBuf>,
    /// The WAV file's sample rate (the IVC firmware samples at 500 kS/s).
    #[arg(long, default_value_t = 500_000)]
    hydrophone_wav_rate: u32,
    /// Write what the board's SSD1306 shows to this PNG file at the end.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Boot through the bootrom and boot2 (real flash path) instead of starting the image directly.
    #[arg(long)]
    rom_boot: bool,
    /// Keep the flash in this file: loaded (if it exists) before the image is written over it, saved at the end of
    /// the run (also when SIGINT or SIGTERM ends it).
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
    /// Open the device's USB CDC ACM serial port (pico-sdk stdio_usb) from the host: `stdout` prints its lines,
    /// `pty` attaches it to a pseudo terminal, `pty:PATH` also links PATH to that terminal. Implies `--usb`.
    #[arg(long, value_parser = parse_cdc_sink)]
    usb_cdc: Option<CdcSink>,
    /// Open the device's second CDC port too (titan_dual_stdio_usb's secondary port, which titan_firmware's
    /// micro-ROS USB transport uses) on a pseudo terminal: `pty` or `pty:PATH`. Implies `--usb`.
    #[arg(long, value_parser = parse_cdc_sink)]
    usb_cdc2: Option<CdcSink>,
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
    #[command(flatten)]
    can_host: host_tools::CanHostArgs,
    /// Profile the firmware: write the functions by time (per core) to FILE and folded stacks for flamegraph tools
    /// next to it (FILE with the extension `.folded`).
    #[arg(long)]
    profile: Option<PathBuf>,
    /// Mean virtual time between profile samples.
    #[arg(long, default_value = "10us", value_parser = parse_time)]
    profile_interval: Time,
    /// Write a waveform (VCD, for GTKWave, Surfer or PulseView) of the GPIOs to this file.
    #[arg(long)]
    vcd: Option<PathBuf>,
    /// GPIOs in the waveform: numbers, ranges and names from the board file's `[pins]` table, e.g.
    /// `0-7,25,status_ledr` (default all).
    #[arg(long, default_value = "all")]
    vcd_pins: String,
    /// Add decoded events to the waveform as string variables: defmt logs, UART lines, CAN frames, device reports
    /// (GTKWave and Surfer show them; PulseView reads only the pins).
    #[arg(long)]
    vcd_events: bool,
    /// Serve a live dashboard on this address (e.g. `127.0.0.1:8080`): logs, firmware status, CAN traffic, pins,
    /// inputs, vehicles, and controls to inject faults, set inputs and pause.
    #[arg(long)]
    dashboard: Option<String>,
    /// Keep serving the dashboard after the run ends (until interrupted).
    #[arg(long, requires = "dashboard")]
    dashboard_hold: bool,
    /// Pause virtual time when it reaches this (`0`: from the start, within the first 10 ms); resume from the
    /// dashboard.
    #[arg(long, requires = "dashboard", value_parser = parse_time)]
    dashboard_pause_at: Option<Time>,
    /// Write every log line, UART line, CAN frame and fault as one JSON object per line to this file, for tools.
    #[arg(long)]
    log_json: Option<PathBuf>,
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

    let board = match &args.board {
        Some(p) => Some(sim_host::board::Board::load(p).map_err(anyhow::Error::msg)?),
        None => None,
    };
    let flash_size = args.flash_size.or(board.as_ref().and_then(|b| b.flash_size)).unwrap_or(2 * 1024 * 1024);
    let config = MachineConfig {
        flash_size,
        bootrom: Some(rp2040::BOOTROM_B1.to_vec()),
        quantum: args.quantum,
        crystal_hz: args.crystal_hz,
    };
    let mut machine = Machine::new(config);
    machine.sys.unmodelled.log_mode = args.unmodelled.mode();
    if args.stop_on_bkpt {
        machine.bkpt_policy = BkptPolicy::Stop;
    }
    let categories = trace_categories(&args.trace);
    if !categories.is_empty() {
        let out = open_trace(args.trace_file.as_deref())?;
        machine.set_trace(Box::new(WriterSink { categories, out, prefix: String::new() }));
    }

    let mut board_inputs = sim_host::board::Inputs::default();
    if let Some(b) = &board {
        let inputs = b.apply(&mut machine).map_err(anyhow::Error::msg)?;
        board_inputs = inputs.clone();
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
        if args.hydrophone_wav.is_some() && sim_host::acoustic::record_hydrophones(&mut machine) == 0 {
            bail!("--hydrophone-wav needs a board with a hydrophone in an --environment");
        }
    } else if !args.inputs.is_empty() || args.environment.is_some() {
        bail!("--input and --environment need a --board");
    }
    machine.load(&image).map_err(anyhow::Error::msg)?;
    machine.hle_enabled = args.hle_flash;
    if args.coverage.is_some() {
        machine.enable_coverage();
    }
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

    if args.usb
        || args.bootsel
        || args.usb_cdc.is_some()
        || args.usb_cdc2.is_some()
        || args.picoboot_load.is_some()
        || args.usbip.is_some()
    {
        let mut host = sim_devices::usb::host::HostController::new();
        if args.usb_cdc.is_some() {
            host.cdc_open(0, 115_200);
        }
        if args.usb_cdc2.is_some() {
            host.cdc_open(1, 115_200);
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
    match &args.usb_cdc {
        Some(CdcSink::Stdout) => session.on_usb_cdc_line = Some(Box::new(|t, line| println!("[{t}] usb: {line}"))),
        Some(CdcSink::Pty(link)) => session.usb_cdc_pty = Some(open_pty(link.as_deref(), "USB CDC port")?),
        None => {}
    }
    match &args.usb_cdc2 {
        Some(CdcSink::Stdout) => bail!("--usb-cdc2 carries raw bytes (micro-ROS): use pty or pty:PATH"),
        Some(CdcSink::Pty(link)) => session.usb_cdc2_pty = Some(open_pty(link.as_deref(), "USB CDC port 2")?),
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
    let mut link: Option<sim_host::dashboard::Link> = None;
    let mut agent: Option<Box<dyn sim_host::host_protocol::HostAgent>> = None;
    // A titan_link agent is the mock bridge; any other needs the board's CAN bus
    let mut mock_bridge = args.mock_bridge;
    if let Some(spec) = &args.mock_agent {
        anyhow::ensure!(spec.bus.is_none(), "--mock-agent {spec}: a single board has one bus: PROTOCOL[:CLIENT]");
        let client = match &spec.target {
            Some(t) => t.parse::<u8>().map_err(|_| anyhow::anyhow!("--mock-agent {spec}: {t:?} is not a client ID"))?,
            None => board.as_ref().and_then(|b| b.can.as_ref()?.client_id).with_context(|| {
                format!("--mock-agent {spec}: the board file gives no CANmore client ID; give PROTOCOL:CLIENT")
            })?,
        };
        match (spec.protocol, board.as_ref().and_then(|b| b.can_rates())) {
            (sim_host::host_protocol::Protocol::TitanLink, _) => mock_bridge = Some(client),
            (protocol, Some(rates)) => {
                let a = sim_host::host_protocol::create(
                    protocol,
                    client,
                    rates.data.is_some(),
                    Time::ZERO,
                    spec.config.as_deref(),
                )
                .map_err(|e| anyhow::anyhow!("--mock-agent {spec}: {e}"))?;
                print_agent_messages(&*a, format!("client {client}"));
                session.can_endpoints.push(a.endpoint());
                log::info!("{} mock agent for client {client} attached", a.protocol());
                agent = Some(a);
            }
            (_, None) => bail!("--mock-agent {spec}: needs a --board with a CAN bus"),
        }
    }
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
        if let Some(client) = mock_bridge {
            let bridge = sim_host::mock_bridge::MockBridge::new(client, rates.data.is_some(), sim_core::Time::ZERO);
            bridge.set_kill(Some(false));
            session.can_endpoints.push(Box::new(bridge.clone()));
            link = Some(sim_host::dashboard::Link::Can(bridge.clone()));
            bridge_ctl = Some(Box::new(bridge));
            log::info!("mock bridge for client {client} attached");
        }
    } else if session.machine.sys.ethernet.is_some() {
        if mock_bridge.is_some() {
            let bridge = sim_host::ethernet::PuddlesBridge::new(Time::ZERO);
            bridge.set_kill(Some(false));
            session.eth_endpoints.push(Box::new(bridge.clone()));
            link = Some(sim_host::dashboard::Link::Eth(bridge.clone()));
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
    } else if args.can.is_some() || mock_bridge.is_some() || args.udp.is_some() {
        bail!("--can and --mock-bridge need a --board with a CAN bus (or Ethernet), --udp one with Ethernet");
    }
    session.realtime = args.realtime;
    session.stop_on_lockup = args.stop_on_lockup;
    let can_host = host_tools::attach_run(&args.can_host, &mut session)?;

    let wall = std::time::Instant::now();
    if args.usbip.is_some() && args.checkpoint_every.is_some_and(|t| t > Time::ZERO) {
        log::warn!("time travel does not record USB/IP traffic: going back past it will not see it again");
    }
    let default_on = args.gdb.is_some() || args.dashboard.is_some();
    let inputs = vec![board_inputs.clone()];
    let profiling = args.profile.is_some();
    let mut tt =
        time_travel(&mut session, inputs, args.checkpoint_every, args.checkpoint_memory, default_on, profiling);
    if let Some(addr) = &args.gdb {
        let listener = std::net::TcpListener::bind(addr).with_context(|| format!("binding {addr}"))?;
        log::info!("waiting for GDB on {addr}");
        // Stopped (SIGINT, SIGTERM while GDB was attached): the run below ends at once, with the usual summary
        let ended = match &mut tt {
            Some(tt) => {
                let mut target = sim_host::gdb::Travel { tt, target: &mut session, board: 0 };
                sim_host::gdb::serve_target(&listener, &mut target)?
            }
            None => sim_host::gdb::serve(&listener, &mut session)?,
        };
        if ended == sim_host::gdb::Ended::Killed {
            eprintln!("GDB ended the session at {}", session.machine.now);
            return Ok(());
        }
    }
    if let (Some(img), Some(path)) = (&to_load, &args.picoboot_load) {
        picoboot_load(&mut session, img, flash_size as u32, args.load_via)
            .with_context(|| format!("loading {}", path.display()))?;
    }
    let end = if to_load.is_some() { session.machine.now + args.time } else { args.time };
    let vcd = match &args.vcd {
        Some(path) => {
            let named = board.as_ref().map(|b| b.pins.clone()).unwrap_or_default();
            let pins = sim_host::vcd::parse_pins(&args.vcd_pins, &named).map_err(anyhow::Error::msg)?;
            let opts = sim_host::vcd::VcdOptions { pins, events: args.vcd_events };
            let name = board.as_ref().map_or("board", |b| b.name.as_str());
            let scope = sim_host::vcd::Scope::board(name, &named, &opts);
            let vcd =
                sim_host::vcd::Vcd::create(path, &[scope]).with_context(|| format!("creating {}", path.display()))?;
            vcd.attach(0, &mut session);
            Some(vcd)
        }
        None => None,
    };
    if args.profile.is_some() {
        match &to_load {
            // The machine's image is the one it booted (or none, with --bootsel); name what was flashed
            Some(img) => {
                let m = &session.machine;
                let p = sim_host::profile::Profiler::for_image(img, &m.sys.rom, args.profile_interval, m.now);
                session.profiler = Some(p);
            }
            None => session.start_profile(args.profile_interval),
        }
    }
    let mut dash = match &args.dashboard {
        Some(addr) => {
            let image = args.image.as_ref().or(args.picoboot_load.as_ref()).and_then(|p| p.file_name());
            let image = image.map_or("firmware".into(), |t| t.to_string_lossy().into_owned());
            let info = sim_host::dashboard::BoardInfo {
                name: board.as_ref().map_or_else(|| image.clone(), |b| b.name.clone()),
                image,
                inputs: board_inputs.clone(),
                pins: board.as_ref().map(|b| b.pins.clone()).unwrap_or_default(),
                fault_names: Vec::new(),
            };
            let buses = session.can_bus.as_ref().map(|b| vec![b.name.clone()]).unwrap_or_default();
            let mut d = sim_host::dashboard::Dashboard::start(addr, vec![info], buses)
                .with_context(|| format!("binding {addr}"))?;
            d.pause_at(args.dashboard_pause_at);
            d.travel = tt.take();
            d.tap_session(0, &mut session);
            if let Some(l) = link.take() {
                d.add_link(Some(0), None, l);
            }
            if let Some(a) = &agent {
                d.add_agent(Some(0), a.handle());
            }
            eprintln!("dashboard on http://{}/", d.addr);
            Some(d)
        }
        None => None,
    };
    let json_log = match &args.log_json {
        Some(path) => {
            let log =
                sim_host::jsonlog::JsonLog::create(path).with_context(|| format!("creating {}", path.display()))?;
            let name = board.as_ref().map_or("board", |b| b.name.as_str());
            let bus = session.can_bus.as_ref().map(|b| b.name.clone());
            log.tap_session(name, bus.as_deref(), &mut session);
            Some(log)
        }
        None => None,
    };
    let mut faults = args.faults.clone();
    faults.sort_by_key(|f| f.0);
    // A lockup no watchdog will end lasts until the last fault that restarts the chip, if there is one
    let restart_by = faults.iter().filter(|f| restarts_the_chip(&f.1)).map(|f| f.0).max();
    let mut faults = faults.into_iter().peekable();
    let mut pending: Option<(Time, sim_host::fault::Fault)> = None;
    // From here on SIGINT and SIGTERM end the run cleanly, at the next slice boundary
    session.stop_flag = Some(&sim_host::signal::STOP);
    let reason = loop {
        if pending.is_none() {
            pending = faults.next_if(|f| f.0 <= end);
        }
        let target = pending.as_ref().map_or(end, |f| f.0);
        // With a dashboard, run in short pieces so what it shows and what it is asked to do stay current
        let until = if dash.is_some() { target.min(session.machine.now + DASHBOARD_PIECE) } else { target };
        let reason = match (&mut dash, &mut tt) {
            (Some(d), _) => d.run_session(&mut session, until),
            (None, Some(tt)) => travel_reason(tt.run_until(&mut session, until)),
            (None, None) => session.run_until(until),
        };
        if let Some(d) = &mut dash {
            d.service_session(&mut session);
            while d.paused() && !sim_host::signal::requested() {
                std::thread::sleep(std::time::Duration::from_millis(20));
                d.service_session(&mut session);
            }
        }
        if matches!(reason, StopReason::Lockup { .. })
            && !args.stop_on_lockup
            && restart_by.is_some_and(|t| t >= session.machine.now)
        {
            log::warn!("[{}] the board stays locked up until a fault restarts it", session.machine.now);
            continue;
        }
        if !matches!(reason, StopReason::TimeLimit) {
            break reason;
        }
        if until < target {
            continue;
        }
        let Some((_, fault)) = pending.take() else { break reason };
        let applied = sim_host::fault::apply_to_session(&mut session, bridge_ctl.as_deref(), &fault);
        if let Some(tt) = dash.as_mut().and_then(|d| d.travel.as_mut()).or(tt.as_mut()) {
            tt.changed(&mut session);
        }
        if let Some(d) = &mut dash {
            d.note_fault(session.machine.now, "", &format!("{fault:?}"), &applied);
        }
        if let Some(l) = &json_log {
            let name = board.as_ref().map_or("board", |b| b.name.as_str());
            l.fault(session.machine.now, name, &format!("{fault:?}"), &applied);
        }
        if let Err(e) = applied {
            bail!("fault {fault:?}: {e}");
        }
    };
    let elapsed = wall.elapsed();
    // What the stop line, the JSON log and the dashboard give as the reason
    let why = match reason {
        StopReason::Requested if sim_host::signal::requested() => "Interrupted".to_string(),
        r => format!("{r:?}"),
    };
    session.flush_uarts();
    can_host.finish()?;
    for (ch, bytes) in session.rtt_text.drain(..) {
        print!("[rtt{ch}] {}", String::from_utf8_lossy(&bytes));
    }
    if let Some(f) = &args.flash_file {
        session.machine.save_flash_file(f).with_context(|| format!("writing {}", f.display()))?;
    }
    if let (Some(v), Some(path)) = (&vcd, &args.vcd) {
        finish_vcd(v, path, session.machine.now)?;
    }
    if let (Some(l), Some(path)) = (&json_log, &args.log_json) {
        l.finish(session.machine.now, &why).with_context(|| format!("writing {}", path.display()))?;
    }
    if let (Some(path), Some(p)) = (&args.profile, &session.profiler) {
        let title = args.image.as_ref().or(args.picoboot_load.as_ref()).and_then(|p| p.file_name());
        let title = title.map_or("firmware".into(), |t| t.to_string_lossy().into_owned());
        let folded = p.report().write(path, &title).with_context(|| format!("writing {}", path.display()))?;
        eprintln!("profile: {} and {}", path.display(), folded.display());
    }
    if let Some(path) = &args.coverage {
        let name = args.image.as_ref().or(args.picoboot_load.as_ref()).and_then(|p| p.file_name());
        let name = name.map_or("firmware".into(), |t| t.to_string_lossy().into_owned());
        write_coverage(path, std::iter::once((name.as_str(), &session.machine)))?;
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
    if let Some(path) = &args.hydrophone_wav {
        let lines = sim_host::acoustic::write_hydrophone_wavs(&mut session.machine, path, args.hydrophone_wav_rate)
            .with_context(|| format!("writing {}", path.display()))?;
        for l in lines {
            eprintln!("{l}");
        }
    }
    let lockups = (session.lockups > 0).then(|| lockup_note(&session));
    let clipping = sim_host::acoustic::clipping_notes(&mut session.machine);
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
        StopReason::Lockup { core, pc } if args.stop_on_lockup => eprintln!("core{core} locked up at {}", where_(pc)),
        StopReason::Lockup { core, pc } => eprintln!(
            "core{core} locked up at {} with nothing to reset the chip (the watchdog is not running); stopping",
            where_(pc)
        ),
        StopReason::Deadlock => eprintln!("deadlock: every core sleeps and nothing is scheduled"),
        StopReason::Watchpoint { core, pc, addr, kind, .. } => {
            eprintln!("core{core} stopped at a {kind:?} watchpoint on {addr:#010x}, before {}", where_(pc))
        }
        StopReason::Reached { core, pc } => eprintln!("core{core} stopped at {}", where_(pc)),
        StopReason::Requested => {}
    }
    eprintln!(
        "stopped at {} ({why}); core0 {} cycles, pc {}; {:.2?} wall",
        machine.now,
        machine.cores[0].cycles(),
        where_(machine.cores[0].pc()),
        elapsed
    );
    if let Some(a) = &agent {
        agent_summary(&format!("client {}", a.client_id()), &**a, machine.now);
    }
    if let Some(note) = &lockups {
        eprintln!("{note}");
    }
    for note in &clipping {
        eprintln!("{note}");
    }
    if machine.core_enabled[1] {
        eprintln!(
            "core1 {} cycles, pc {}, {:?}",
            machine.cores[1].cycles(),
            where_(machine.cores[1].pc()),
            machine.cores[1].sleep_state()
        );
    }
    let summary = machine.sys.unmodelled.summary();
    if !summary.is_empty() && args.unmodelled != UnmodelledLog::Off {
        eprintln!("unmodelled registers touched:");
        for (name, reads, writes) in summary {
            eprintln!("  {name:32} {reads:>8} reads {writes:>8} writes");
        }
    }
    if let Some(d) = &mut dash {
        d.finish_session(&mut session, &why);
        if args.dashboard_hold && !sim_host::signal::requested() {
            eprintln!("run finished; the dashboard keeps serving on http://{}/ (Ctrl-C to quit)", d.addr);
            sim_host::signal::exit_on_signal();
            d.hold();
        }
    }
    if matches!(reason, StopReason::Lockup { .. }) {
        bail!("core locked up");
    }
    Ok(())
}

/// Whether a fault restarts the chip (and so ends a lockup).
fn restarts_the_chip(f: &sim_host::fault::Fault) -> bool {
    use sim_host::fault::Fault;
    matches!(f, Fault::Brownout { .. } | Fault::WatchdogTimeout)
}

/// How many times a board that kept running through lockups locked up, how often the chip was reset, and
/// whether a core is locked up now.
fn lockup_note(s: &Session) -> String {
    let m = &s.machine;
    let now: Vec<String> = (0..2)
        .filter(|&c| m.core_enabled[c] && m.cores[c].is_locked_up())
        .map(|c| format!("core{c} is locked up now"))
        .collect();
    let now = if now.is_empty() { String::new() } else { format!("; {}", now.join(", ")) };
    format!("{} lockup(s), {} chip reset(s){now}", s.lockups, m.resets)
}

/// Virtual time run between dashboard updates.
const DASHBOARD_PIECE: Time = Time::from_ms(10);

fn finish_vcd(v: &sim_host::vcd::Vcd, path: &std::path::Path, end: Time) -> Result<()> {
    let late = v.finish(end).with_context(|| format!("writing {}", path.display()))?;
    if late > 0 {
        log::warn!("{late} waveform changes arrived more than 50 ms after their time; they are shown late");
    }
    eprintln!("waveform: {}", path.display());
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
    let cli = Cli::parse();
    // Scenario runs print their own report; the emulator's notes (faults applied, ...) would only interleave with it
    let quiet = matches!(cli.command, Command::Scenario(_) | Command::Compare(_) | Command::Fuzz(_));
    let level = if quiet { "error" } else { "info" };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level)).format_timestamp(None).init();
    if matches!(cli.command, Command::Run(_) | Command::Robot(_) | Command::Picoboot(_)) {
        // SIGINT and SIGTERM end a run the way reaching its time does (flash file saved, logs and trailers written)
        sim_host::signal::install().context("catching SIGINT and SIGTERM")?;
    }
    let result = run_command(cli.command);
    if result.is_ok()
        && let Some(sig) = sim_host::signal::received()
    {
        // Finished cleanly, but because of the signal: exit as a shell expects of a process it ended
        std::process::exit(128 + sig);
    }
    result
}

fn run_command(command: Command) -> Result<()> {
    match command {
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
        Command::CanDiff(args) => match host_tools::can_diff(args) {
            Ok(true) => Ok(()),
            Ok(false) => std::process::exit(1),
            Err(e) => {
                eprintln!("error: {e:#}");
                std::process::exit(2)
            }
        },
        Command::Bridges(args) => host_tools::bridges(args),
        Command::CanTunnel(args) => host_tools::can_tunnel(args),
        Command::Mcp(args) => mcp::serve(args),
        Command::Scenario(args) => std::process::exit(scenario_cli::scenario(args)?),
        Command::Compare(args) => std::process::exit(scenario_cli::compare(args)?),
        Command::Calibrate(cmd) => calib_cli::calibrate(cmd),
        Command::ReplayPoolTest(args) => match calib_cli::replay_pool_test(args) {
            Ok(true) => Ok(()),
            Ok(false) => std::process::exit(1),
            Err(e) => {
                eprintln!("Error: {e:#}");
                std::process::exit(2)
            }
        },
        Command::Fuzz(args) => fuzz_cli::run(args),
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
