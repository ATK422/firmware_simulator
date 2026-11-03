//! Host-side CAN tools of the command line: candump logs of a run (`--candump-log`), replaying recorded traffic
//! (`--replay`), cannelloni tunnels (`--cannelloni`), and the `can-diff`, `bridges` and `can-tunnel` subcommands.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::ValueEnum;

use sim_core::time::Time;
use sim_host::Session;
use sim_host::candiff::{self, Align, DiffOptions};
use sim_host::canlog::{self, CandumpWriter, Filter, LogFrame, Replay, ReplayOptions, SharedReplay};
use sim_host::cannelloni::Cannelloni;
use sim_host::robot::Robot;

fn parse_filter(s: &str) -> Result<Filter, String> {
    Filter::parse(s)
}

/// CAN log, replay and tunnel options of `run` and `robot`.
#[derive(clap::Args, Default)]
pub struct CanHostArgs {
    /// Write every acknowledged CAN frame to this file in `candump -l` format (virtual time as the timestamp, the
    /// bus name as the interface).
    #[arg(long, value_name = "FILE")]
    pub candump_log: Option<PathBuf>,
    /// Replay a candump log (`candump -l` or candump's text output) into the bus as a host endpoint (repeatable).
    #[arg(long, value_name = "FILE")]
    pub replay: Vec<PathBuf>,
    /// Which frames of the log to replay, e.g. `sender=host` (what the robot computer sent), `sender=host,client=1`,
    /// `!client=2`; keys iface, client, sender, type, channel, id (repeatable: a frame any filter selects is replayed).
    #[arg(long = "replay-filter", value_name = "SPEC", value_parser = parse_filter)]
    pub replay_filters: Vec<Filter>,
    /// Virtual time at which the log's first frame is replayed (the rest keep their spacing). By default a log
    /// with Unix timestamps (a hardware recording) starts at time 0, and one with small timestamps (an emulator
    /// run's `--candump-log`) is replayed at its own times.
    #[arg(long, value_name = "TIME", value_parser = super::parse_time)]
    pub replay_offset: Option<Time>,
    /// Interface of the log to replay: `IFACE` with `run`, `IFACE=BUS` with `robot` (repeatable). By default an
    /// interface named like a bus (or like the SocketCAN interface it is bridged to) goes to that bus, and a log
    /// with a single interface goes to a robot's single bus.
    #[arg(long = "replay-iface", value_name = "IFACE[=BUS]")]
    pub replay_ifaces: Vec<String>,
    /// Tunnel the bus to a cannelloni peer over UDP: `LOCAL,PEER` with `run`, `BUS=LOCAL,PEER` with `robot` (e.g.
    /// `internal=0.0.0.0:20000,192.168.1.50:20000`). Use with `--realtime`.
    #[arg(long, value_name = "[BUS=]LOCAL,PEER")]
    pub cannelloni: Vec<String>,
}

impl CanHostArgs {
    fn any(&self) -> bool {
        self.candump_log.is_some() || !self.replay.is_empty() || !self.cannelloni.is_empty()
    }
}

/// What was attached, to report on at the end of the run.
#[derive(Default)]
pub struct Attached {
    writer: Option<(CandumpWriter, PathBuf)>,
    replays: Vec<(String, SharedReplay, usize)>,
}

impl Attached {
    /// Flushes the log and reports what was written and replayed.
    pub fn finish(&self) -> Result<()> {
        if let Some((w, path)) = &self.writer {
            w.flush().with_context(|| format!("writing {}", path.display()))?;
            if let Some(e) = w.error() {
                bail!("writing {}: {e}", path.display());
            }
            let (n, unacked) = w.counts();
            let skipped = if unacked > 0 { format!(" ({unacked} unacknowledged left out)") } else { String::new() };
            eprintln!("candump log: {n} frame(s) in {}{skipped}", path.display());
        }
        for (what, r, total) in &self.replays {
            let r = r.0.lock().unwrap();
            eprintln!("replay {what}: {} of {total} frame(s) sent", r.sent);
        }
        Ok(())
    }
}

/// The log origin and the virtual time it is replayed at.
fn replay_timing(args: &CanHostArgs, log: &[LogFrame]) -> (Option<u64>, Time) {
    let first = log.iter().map(|f| f.time_ns).min();
    match (args.replay_offset, first) {
        (Some(offset), _) => (first, offset),
        (None, Some(t)) if t < canlog::RELATIVE_LIMIT_NS => (Some(0), Time::ZERO),
        (None, _) => (first, Time::ZERO),
    }
}

fn read_logs(paths: &[PathBuf]) -> Result<Vec<(PathBuf, Vec<LogFrame>)>> {
    paths.iter().map(|p| Ok((p.clone(), canlog::read_file(p).map_err(anyhow::Error::msg)?))).collect()
}

/// Attaches the options to a single board's bus.
pub fn attach_run(args: &CanHostArgs, session: &mut Session) -> Result<Attached> {
    let mut out = Attached::default();
    if !args.any() {
        return Ok(out);
    }
    let Some(bus) = session.can_bus.as_mut() else {
        bail!("--candump-log, --replay and --cannelloni need a --board with a CAN bus");
    };
    let bus_name = bus.name.clone();
    if let Some(path) = &args.candump_log {
        let w = CandumpWriter::create(path).with_context(|| format!("creating {}", path.display()))?;
        bus.record.get_or_insert_with(Vec::new);
        session.on_can = Some(w.hook(&bus_name, session.on_can.take()));
        out.writer = Some((w, path.clone()));
    }
    let rates = bus.rates;
    for (path, log) in read_logs(&args.replay)? {
        let ifaces = canlog::interfaces(&log);
        let chosen: Vec<String> = if !args.replay_ifaces.is_empty() {
            args.replay_ifaces.iter().map(|s| s.split('=').next().unwrap().to_string()).collect()
        } else if ifaces.len() <= 1 {
            ifaces.clone()
        } else if ifaces.contains(&bus_name) {
            vec![bus_name.clone()]
        } else {
            bail!("{} has frames on {}: pick one with --replay-iface", path.display(), ifaces.join(", "));
        };
        let (origin_ns, offset) = replay_timing(args, &log);
        let opts = ReplayOptions {
            ifaces: chosen,
            filters: args.replay_filters.clone(),
            offset,
            origin_ns,
            rates: Some(rates),
        };
        let r = Replay::from_log(&log, &opts);
        let total = r.remaining();
        let from = r.first_time().map(|t| format!(" from {t}")).unwrap_or_default();
        log::info!("replaying {total} frame(s) of {}{from}", path.display());
        let shared = SharedReplay::new(r);
        session.can_endpoints.push(Box::new(shared.clone()));
        out.replays.push((path.display().to_string(), shared, total));
    }
    for spec in &args.cannelloni {
        let (local, peer) = Cannelloni::parse_spec(spec).map_err(anyhow::Error::msg)?;
        let t = Cannelloni::bind(local, peer).with_context(|| format!("binding {local}"))?;
        session.can_endpoints.push(Box::new(t));
        log::info!("CAN bus tunnelled (cannelloni over UDP): {local} <-> {peer}");
    }
    Ok(out)
}

/// Attaches the options to a robot's buses; `cans` are its `--can BUS=IFACE` bridges.
pub fn attach_robot(args: &CanHostArgs, r: &mut Robot, cans: &[(String, String)]) -> Result<Attached> {
    let mut out = Attached::default();
    let bus_index = |r: &Robot, name: &str| -> Result<usize> {
        r.bus_names.iter().position(|n| n == name).with_context(|| format!("no bus {name:?}"))
    };
    if let Some(path) = &args.candump_log {
        let w = CandumpWriter::create(path).with_context(|| format!("creating {}", path.display()))?;
        for (j, rb) in r.buses.iter_mut().enumerate() {
            rb.bus.record.get_or_insert_with(Vec::new);
            rb.on_can = Some(w.hook(&r.bus_names[j], rb.on_can.take()));
        }
        out.writer = Some((w, path.clone()));
    }
    for (path, log) in read_logs(&args.replay)? {
        // Which of the log's interfaces go to which bus
        let mut map: Vec<(String, usize)> = Vec::new();
        for spec in &args.replay_ifaces {
            let (iface, bus) = spec.split_once('=').context("--replay-iface IFACE=BUS")?;
            map.push((iface.to_string(), bus_index(r, bus)?));
        }
        let ifaces = canlog::interfaces(&log);
        for iface in &ifaces {
            if map.iter().any(|(i, _)| i == iface) {
                continue;
            }
            let by_name = r.bus_names.iter().position(|n| n == iface);
            let by_can =
                cans.iter().find(|(_, i)| i == iface).and_then(|(b, _)| r.bus_names.iter().position(|n| n == b));
            let single = (ifaces.len() == 1 && r.buses.len() == 1).then_some(0);
            match by_name.or(by_can).or(single) {
                Some(j) => map.push((iface.clone(), j)),
                None if args.replay_ifaces.is_empty() => {
                    bail!("{}: no bus for interface {iface:?}; map it with --replay-iface {iface}=BUS", path.display())
                }
                None => log::warn!("{}: frames on {iface} are not replayed", path.display()),
            }
        }
        // One origin for every bus, keeping the buses' traffic in step
        let (origin_ns, offset) = replay_timing(args, &log);
        for (j, rb) in r.buses.iter_mut().enumerate() {
            let on_bus: Vec<String> = map.iter().filter(|(_, b)| *b == j).map(|(i, _)| i.clone()).collect();
            if on_bus.is_empty() {
                continue;
            }
            let opts = ReplayOptions {
                ifaces: on_bus.clone(),
                filters: args.replay_filters.clone(),
                offset,
                origin_ns,
                rates: Some(rb.bus.rates),
            };
            let replay = Replay::from_log(&log, &opts);
            let total = replay.remaining();
            let shared = SharedReplay::new(replay);
            rb.endpoints.push(Box::new(shared.clone()));
            let what = format!("{} ({} -> {})", path.display(), on_bus.join(", "), r.bus_names[j]);
            log::info!("replaying {total} frame(s) of {what}");
            out.replays.push((what, shared, total));
        }
    }
    for spec in &args.cannelloni {
        let (bus, addrs) = spec.split_once('=').context("--cannelloni BUS=LOCAL,PEER")?;
        let (local, peer) = Cannelloni::parse_spec(addrs).map_err(anyhow::Error::msg)?;
        let j = bus_index(r, bus)?;
        let t = Cannelloni::bind(local, peer).with_context(|| format!("binding {local}"))?;
        r.buses[j].endpoints.push(Box::new(t));
        log::info!("bus {bus} tunnelled (cannelloni over UDP): {local} <-> {peer}");
    }
    Ok(out)
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum AlignArg {
    /// The first frame of each log is time 0.
    Start,
    /// Timestamps as logged.
    None,
    /// Search for the offset that pairs the most events.
    Auto,
}

#[derive(clap::Args)]
pub struct CanDiffArgs {
    /// The reference log (e.g. recorded on hardware), `candump -l` or candump text output.
    a: PathBuf,
    /// The log compared with it (e.g. an emulator run's `--candump-log`).
    b: PathBuf,
    /// How B's timestamps are lined up with A's. By default two emulator logs (timestamps since power-on) are
    /// compared as logged, and otherwise the first frames are lined up.
    #[arg(long, value_enum)]
    align: Option<AlignArg>,
    /// B's timestamps minus A's for the same moment (replaces --align), e.g. `-1.5s`.
    #[arg(long, allow_hyphen_values = true, value_parser = parse_signed_time)]
    offset: Option<i64>,
    /// How far apart paired events may be (at most half a stream's period).
    #[arg(long, default_value = "20ms", value_parser = super::parse_time)]
    tolerance: Time,
    /// Pair events only on the global alignment, without lining each stream up on its own first.
    #[arg(long)]
    no_stream_align: bool,
    /// Also count events outside the time both logs cover.
    #[arg(long)]
    full: bool,
    /// Don't compare the payloads of streams whose name contains this (repeatable), e.g. `FirmwareStatus`.
    #[arg(long, value_name = "STREAM")]
    ignore_payload: Vec<String>,
    /// Only compare streams whose name contains this (repeatable), e.g. `client 8`.
    #[arg(long, value_name = "STREAM")]
    only: Vec<String>,
    /// Compare raw frames by identifier instead of decoding CANmore and titan_link.
    #[arg(long)]
    raw: bool,
    /// Rename an interface in both logs, `OLD=NEW` (repeatable), e.g. `can0=internal`. Two logs with one interface
    /// each are matched without it.
    #[arg(long = "iface-map", value_name = "OLD=NEW")]
    iface_map: Vec<String>,
    /// Examples shown per difference.
    #[arg(long, default_value_t = 5)]
    details: usize,
    /// Also fail if paired events are further apart than this (after the global alignment).
    #[arg(long, value_parser = super::parse_time)]
    max_skew: Option<Time>,
}

fn parse_signed_time(s: &str) -> Result<i64, String> {
    let (neg, rest) = match s.trim().strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s.trim().trim_start_matches('+')),
    };
    let t = super::parse_time(rest)?.as_ns() as i64;
    Ok(if neg { -t } else { t })
}

/// Compares two logs and prints the report; returns whether they agree.
pub fn can_diff(args: CanDiffArgs) -> Result<bool> {
    let mut a = canlog::read_file(&args.a).map_err(anyhow::Error::msg)?;
    let mut b = canlog::read_file(&args.b).map_err(anyhow::Error::msg)?;
    let mut renames = Vec::new();
    for m in &args.iface_map {
        let (old, new) = m.split_once('=').context("--iface-map OLD=NEW")?;
        renames.push((old.to_string(), new.to_string()));
    }
    let (ia, ib) = (canlog::interfaces(&a), canlog::interfaces(&b));
    if renames.is_empty() && ia.len() == 1 && ib.len() == 1 && ia != ib {
        eprintln!("comparing interface {} of A with {} of B", ia[0], ib[0]);
        renames.push((ib[0].clone(), ia[0].clone()));
    }
    for f in a.iter_mut().chain(b.iter_mut()) {
        if let Some((_, new)) = renames.iter().find(|(old, _)| *old == f.iface) {
            f.iface = new.clone();
        }
    }
    let opts = DiffOptions {
        align: match (args.offset, args.align) {
            (Some(o), _) => Align::Offset(o),
            (None, Some(AlignArg::Start)) => Align::Start,
            (None, Some(AlignArg::None)) => Align::None,
            (None, Some(AlignArg::Auto)) => Align::Auto,
            (None, None) => {
                let relative = |l: &[LogFrame]| l.iter().all(|f| f.time_ns < canlog::RELATIVE_LIMIT_NS);
                if relative(&a) && relative(&b) { Align::None } else { Align::Start }
            }
        },
        tolerance_ns: args.tolerance.as_ns() as i64,
        stream_align: !args.no_stream_align,
        full: args.full,
        ignore_payload: args.ignore_payload,
        only: args.only,
    };
    let report = candiff::diff(&candiff::decode(&a, args.raw), &candiff::decode(&b, args.raw), &opts);
    println!("A: {}\nB: {}", args.a.display(), args.b.display());
    print!("{}", report.render(args.details));
    let mut same = report.identical();
    if let Some(max) = args.max_skew {
        let skew = report.max_abs_skew_ns();
        if skew > max.as_ns() as i64 {
            println!("timing: paired events up to {:.3} ms apart (limit {max})", skew as f64 / 1e6);
            same = false;
        }
    }
    Ok(same)
}

#[derive(Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum BridgesFormat {
    /// One command per line.
    Lines,
    /// A bash script starting them all.
    Script,
    /// A ROS 2 Python launch file.
    Launch,
}

#[derive(clap::Args)]
pub struct BridgesArgs {
    /// Robot file (robots/*.toml).
    robot: PathBuf,
    /// The interface a bus is bridged to, `BUS=IFACE` (repeatable); by default the Nth bus of the file is `vcanN`.
    #[arg(long = "can", value_parser = super::parse_bus_iface)]
    cans: Vec<(String, String)>,
    /// ROS namespace for every bridge, instead of each board's default (`talos`).
    #[arg(long)]
    namespace: Option<String>,
    #[arg(long, value_enum, default_value = "lines")]
    format: BridgesFormat,
    /// Write to this file instead of stdout (a script is made executable).
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// The titan_bridge program.
    #[arg(long, default_value = "titan_bridge")]
    program: String,
    /// Boot the robot and take each board's client ID from its CANmore heartbeats.
    #[arg(long)]
    probe: bool,
    /// How long to run when probing.
    #[arg(long, default_value = "1.5s", value_parser = super::parse_time)]
    probe_time: Time,
}

/// Prints (or writes) the titan_bridge commands for a robot's boards.
pub fn bridges(args: BridgesArgs) -> Result<()> {
    let spec = sim_host::bridges::read_spec(&args.robot).map_err(anyhow::Error::msg)?;
    let dir = args.robot.parent().unwrap_or(Path::new("."));
    let mut cmds =
        sim_host::bridges::plan(&spec, dir, &args.cans, args.namespace.as_deref()).map_err(anyhow::Error::msg)?;
    if args.probe {
        let mut robot = Robot::from_spec(&spec, dir).map_err(anyhow::Error::msg)?;
        let probed = sim_host::bridges::probe(&mut robot, args.probe_time).map_err(anyhow::Error::msg)?;
        sim_host::bridges::apply_probe(&mut cmds, &probed);
    }
    let robot_file = args.robot.display().to_string();
    let out = match args.format {
        BridgesFormat::Lines => {
            format!(
                "# Emulator: {}\n{}",
                sim_host::bridges::emulator_command(&robot_file, &cmds),
                sim_host::bridges::render_lines(&cmds, &args.program)
            )
        }
        BridgesFormat::Script => sim_host::bridges::render_script(&cmds, &args.program, &robot_file),
        BridgesFormat::Launch => sim_host::bridges::render_launch(&cmds, &args.program, &robot_file),
    };
    match &args.output {
        Some(p) => {
            std::fs::write(p, &out).with_context(|| format!("writing {}", p.display()))?;
            if args.format == BridgesFormat::Script {
                use std::os::unix::fs::PermissionsExt;
                std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o755))?;
            }
            eprintln!("wrote {}", p.display());
        }
        None => print!("{out}"),
    }
    Ok(())
}

#[derive(clap::Args)]
pub struct TunnelArgs {
    /// SocketCAN interface on this machine (e.g. vcan0).
    iface: String,
    /// `LOCAL,PEER` UDP addresses: this end, and the emulator's `--cannelloni` address.
    addrs: String,
}

/// Runs a SocketCAN <-> cannelloni tunnel until stopped.
pub fn can_tunnel(args: TunnelArgs) -> Result<()> {
    let (local, peer) = Cannelloni::parse_spec(&args.addrs).map_err(anyhow::Error::msg)?;
    sim_host::cannelloni::tunnel(&args.iface, local, peer).with_context(|| format!("tunnel on {}", args.iface))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_times() {
        assert_eq!(parse_signed_time("-1.5s").unwrap(), -1_500_000_000);
        assert_eq!(parse_signed_time("+2ms").unwrap(), 2_000_000);
        assert_eq!(parse_signed_time("250us").unwrap(), 250_000);
        assert!(parse_signed_time("-x").is_err());
    }
}
