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

#[derive(Subcommand)]
enum Command {
    /// Runs one firmware image (ELF or UF2).
    Run(RunArgs),
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
enum UnmodelledLog {
    First,
    Every,
    Off,
}

#[derive(clap::Args)]
struct RunArgs {
    /// Firmware image: ELF or UF2.
    image: PathBuf,
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
    /// Wait for GDB on this address (e.g. `127.0.0.1:3333`) before running.
    #[arg(long)]
    gdb: Option<String>,
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

fn run(args: RunArgs) -> Result<()> {
    let bytes = std::fs::read(&args.image).with_context(|| format!("reading {}", args.image.display()))?;
    let image = Image::parse(&bytes).with_context(|| format!("parsing {}", args.image.display()))?;

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

    machine.load(&image).map_err(anyhow::Error::msg)?;
    let vtor = machine.direct_boot().map_err(anyhow::Error::msg)?;
    log::info!("direct boot: VTOR {vtor:#010x}, entry {:#010x}", machine.cores[0].pc());

    let mut session = Session::new(machine, &image);
    session.keep_logs = false;
    session.on_log = Some(Box::new(|t, line| println!("[{t}] {line}")));
    let sinks = [args.uart0, args.uart1];
    session.on_uart_line = Some(Box::new(move |t, n, line| {
        if sinks[n] == UartSink::Stdout {
            println!("[{t}] uart{n}: {line}")
        }
    }));
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

    let wall = std::time::Instant::now();
    if let Some(addr) = &args.gdb {
        let listener = std::net::TcpListener::bind(addr).with_context(|| format!("binding {addr}"))?;
        log::info!("waiting for GDB on {addr}");
        if sim_host::gdb::serve(&listener, &mut session)? == sim_host::gdb::Ended::Killed {
            eprintln!("GDB ended the session at {}", session.machine.now);
            return Ok(());
        }
    }
    let reason = session.run_until(args.time);
    let elapsed = wall.elapsed();
    session.flush_uarts();
    for (ch, bytes) in session.rtt_text.drain(..) {
        print!("[rtt{ch}] {}", String::from_utf8_lossy(&bytes));
    }
    let machine = &mut session.machine;
    let where_ = |pc: u32| match machine.symbolize(pc) {
        Some(s) => format!("{pc:#010x} <{s}>"),
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

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).format_timestamp(None).init();
    let cli = Cli::parse();
    match cli.command {
        Command::Run(args) => run(args),
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
}
