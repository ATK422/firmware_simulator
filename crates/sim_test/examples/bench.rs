//! Emulator benchmark: runs single boards (on their board files, next to a mock bridge, as in the smoke tests) and
//! robot files for a fixed virtual time and reports how fast the emulator ran them.
//!
//! ```text
//! cargo run --release -p sim_test --example bench -- [--time 2s] [--repeat 3] [--only esc_board,talos]
//! ```
//!
//! For each case: the virtual time run, the best wall time over the repeats, virtual/wall (above 1 keeps up with
//! `--realtime`), instructions executed by both cores and the emulated MIPS. The fingerprint hashes what the run
//! produced (log lines and when they were read, every message the mock bridges received, UART and RTT text, final
//! CPU registers, cycle counts and SRAM) so a change to the emulator can be checked to leave behaviour and timing
//! exactly as they were: the fingerprints before and after must match. They are also checked to match between the
//! repeats.
//!
//! The wall clock is noisy on a shared machine, so compare the best of several repeats, or run under
//! `valgrind --tool=callgrind` with `--repeat 1` and compare instruction counts.

use std::hash::{Hash, Hasher};
use std::time::{Duration, Instant};

use sim_core::cpu::Cpu;
use sim_core::sched::Fnv;
use sim_test::{Bench, RobotBench, Session, Time};

struct Case {
    name: &'static str,
    kind: Kind,
}

enum Kind {
    /// An image on its board file with a mock bridge for a client.
    Board { elf: &'static str, board: &'static str, client: u8 },
    /// A robot file with mock bridges `(bus, client)`.
    Robot { file: &'static str, bridges: &'static [(&'static str, u8)] },
}

const CASES: &[Case] = &[
    Case { name: "template", kind: Kind::Board { elf: "template", board: "mk2/power_board", client: 1 } },
    Case { name: "esc_board", kind: Kind::Board { elf: "esc_board", board: "mk2/esc_board", client: 2 } },
    Case { name: "ivc_board2", kind: Kind::Board { elf: "ivc_board2", board: "talos/ivc_board", client: 8 } },
    Case { name: "talos", kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)] } },
    Case { name: "ivc2_pool", kind: Kind::Robot { file: "ivc2_pool.toml", bridges: &[("talos", 8), ("surface", 8)] } },
];

struct Outcome {
    wall: Duration,
    instructions: u64,
    fingerprint: u64,
}

fn hash_session(h: &mut Fnv, s: &Session) {
    for (t, line) in &s.logs {
        t.hash(h);
        line.to_string().hash(h);
    }
    s.rtt_text.hash(h);
    s.uart_text.hash(h);
    let m = &s.machine;
    m.now.hash(h);
    for (c, core) in m.cores.iter().enumerate() {
        m.core_time(c).hash(h);
        core.cycles().hash(h);
        core.instructions().hash(h);
        for r in 0..core.num_regs() {
            core.read_reg(r).hash(h);
        }
    }
    m.sys.sram.hash(h);
}

fn instructions(s: &Session) -> u64 {
    s.machine.cores.iter().map(|c| c.instructions()).sum()
}

fn run(case: &Case, time: Time) -> Result<Outcome, String> {
    let mut h = Fnv::default();
    match case.kind {
        Kind::Board { elf, board, client } => {
            let elf = sim_test::titan_rs(elf).ok_or("image not built")?;
            let mut b = Bench::builder(&elf).board(board).bridge(client).boot()?;
            let wall = Instant::now();
            b.run_until(time).map_err(|e| format!("stopped at {}: {e:?}", b.now()))?;
            let wall = wall.elapsed();
            hash_session(&mut h, &b.session);
            for (t, m) in b.messages() {
                t.hash(&mut h);
                format!("{m:?}").hash(&mut h);
            }
            Ok(Outcome { wall, instructions: instructions(&b.session), fingerprint: h.finish() })
        }
        Kind::Robot { file, bridges } => {
            let mut r = RobotBench::load(file, bridges, |_| {})?;
            let wall = Instant::now();
            r.run_until(time);
            let wall = wall.elapsed();
            for s in &r.robot.boards {
                hash_session(&mut h, s);
            }
            for b in r.bridges.iter().flatten() {
                for (t, m) in &b.state().received {
                    t.hash(&mut h);
                    format!("{m:?}").hash(&mut h);
                }
            }
            let instructions = r.robot.boards.iter().map(instructions).sum();
            Ok(Outcome { wall, instructions, fingerprint: h.finish() })
        }
    }
}

fn parse_time(s: &str) -> Time {
    let (num, scale) = if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1.0)
    };
    Time::from_secs_f64(num.parse::<f64>().expect("time like 2s or 500ms") * scale)
}

fn main() {
    let mut time = Time::from_secs(2);
    let mut repeat = 3;
    let mut only: Option<Vec<String>> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| panic!("{a} needs a value"));
        match a.as_str() {
            "--time" => time = parse_time(&value()),
            "--repeat" => repeat = value().parse().expect("--repeat N"),
            "--only" => only = Some(value().split(',').map(str::to_string).collect()),
            "--bench" => {} // passed by `cargo bench`-style runners
            other => panic!("unknown argument {other} (--time T, --repeat N, --only a,b)"),
        }
    }
    println!(
        "{:<11} {:>8} {:>9} {:>9} {:>14} {:>8}  fingerprint",
        "case", "virtual", "wall", "virt/wall", "instructions", "MIPS"
    );
    for case in CASES {
        if only.as_ref().is_some_and(|o| !o.iter().any(|n| n == case.name)) {
            continue;
        }
        let mut best: Option<Outcome> = None;
        for _ in 0..repeat.max(1) {
            let o = match run(case, time) {
                Ok(o) => o,
                Err(e) => {
                    println!("{:<11} skipped: {e}", case.name);
                    break;
                }
            };
            if let Some(b) = &best {
                assert_eq!(b.fingerprint, o.fingerprint, "{}: runs differ", case.name);
            }
            if best.as_ref().is_none_or(|b| o.wall < b.wall) {
                best = Some(o);
            }
        }
        let Some(b) = best else { continue };
        let wall = b.wall.as_secs_f64();
        println!(
            "{:<11} {:>8} {:>8.3}s {:>9.2} {:>14} {:>8.1}  {:016x}",
            case.name,
            time.to_string(),
            wall,
            time.as_secs_f64() / wall,
            b.instructions,
            b.instructions as f64 / wall / 1e6,
            b.fingerprint
        );
    }
}
