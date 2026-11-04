//! Emulator benchmark: runs single boards (on their board files, next to a mock bridge, as in the smoke tests) and
//! robot files for a fixed virtual time and reports how fast the emulator ran them.
//!
//! ```text
//! cargo run --release -p sim_test --example bench -- [--time 2s] [--repeat 3] [--only esc_board,talos] [--threads N]
//!     [--checkpoint-every 10ms] [--checkpoint-memory 512] [--coverage]
//! ```
//!
//! `--coverage` collects code coverage (the cores run one instruction at a time): the fingerprints must not change.
//!
//! `--checkpoint-every` records the runs for time travel (`sim_host::timetravel`, as `run --gdb` and the dashboard
//! do), with a checkpoint that often (and `--checkpoint-memory` MB for them), and prints what the checkpoints took;
//! the fingerprints must not change.
//!
//! `talos_mt` is Talos with its boards on up to four threads (`robot --threads 4`), or as many as `--threads N` says
//! (0: one per core); `talos_t2`, `talos_t3` and `talos_t6` on up to two, three and six, and `talos_t3_ls` on three in
//! step slice by slice (without letting the boards run ahead of each other, `Robot::run_ahead`). Their fingerprints
//! match `talos`'s. `--threads` also prints how the boards were spread over the threads. `nereus`, `nereus_t3` and
//! `nereus_t3_ls` run `robots/talos_nereus.toml` as nereus drives it over vcan (every host side behind a live
//! endpoint, the external vehicle's pose set between 10 ms runs) on one thread, three, and three slice by slice;
//! their fingerprints match each other's.
//!
//! For each case: the virtual time run, the best wall time over the repeats, virtual/wall (above 1 keeps up with
//! `--realtime`), the CPU time of that run (less disturbed than the wall clock by a busy machine), instructions
//! executed by both cores (and the share of them in busy-waits that were skipped rather than interpreted) and the
//! emulated MIPS. The fingerprint hashes what the run
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
use sim_devices::can::{CanEndpoint, CanFrame};
use sim_host::socketcan::Live;
use sim_test::{Bench, RobotBench, Session, Time};

struct Case {
    name: &'static str,
    kind: Kind,
}

enum Kind {
    /// An image on its board file with a mock bridge for a client.
    Board { elf: &'static str, board: &'static str, client: u8 },
    /// A robot file with mock bridges `(bus, client)`, its boards run on `threads` threads (see `Robot::threads`;
    /// negative: on that many, slice by slice, without `Robot::run_ahead`).
    /// `live`: the mock bridges behind live endpoints, as titan_bridge is behind a SocketCAN interface
    /// (`sim_host::socketcan::Live`).
    Robot { file: &'static str, bridges: &'static [(&'static str, u8)], threads: isize, live: bool },
    /// `robots/talos_nereus.toml` as nereus drives it over vcan: a mock bridge for each board and the controller's
    /// thruster frames, all behind live endpoints as a SocketCAN interface is (`sim_host::socketcan::Live`), and
    /// the external vehicle's pose set every 10 ms between runs, as the nereus bridge posts it to the dashboard.
    /// `threads` as for `Robot`.
    Nereus { threads: isize },
}

const CASES: &[Case] = &[
    Case { name: "template", kind: Kind::Board { elf: "template", board: "mk2/power_board", client: 1 } },
    Case { name: "esc_board", kind: Kind::Board { elf: "esc_board", board: "mk2/esc_board", client: 2 } },
    Case { name: "ivc_board2", kind: Kind::Board { elf: "ivc_board2", board: "talos/ivc_board", client: 8 } },
    Case {
        name: "talos",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 1, live: false },
    },
    Case {
        name: "talos_mt",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 4, live: false },
    },
    Case {
        name: "talos_t2",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 2, live: false },
    },
    Case {
        name: "talos_t3",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 3, live: false },
    },
    Case {
        name: "talos_t6",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 6, live: false },
    },
    // On three threads in step slice by slice (`Robot::run_ahead` off), for comparison
    Case {
        name: "talos_t3_ls",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: -3, live: false },
    },
    // Talos with its bridge behind a live endpoint (as on vcan); fingerprints match each other's
    Case {
        name: "talos_live_t3",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: 3, live: true },
    },
    Case {
        name: "talos_live_t3_ls",
        kind: Kind::Robot { file: "talos.toml", bridges: &[("internal", 1)], threads: -3, live: true },
    },
    // nereus over vcan (live endpoints, an external vehicle); its fingerprints match each other's
    Case { name: "nereus", kind: Kind::Nereus { threads: 1 } },
    Case { name: "nereus_t3", kind: Kind::Nereus { threads: 3 } },
    Case { name: "nereus_t3_ls", kind: Kind::Nereus { threads: -3 } },
    Case {
        name: "ivc2_pool",
        kind: Kind::Robot { file: "ivc2_pool.toml", bridges: &[("talos", 8), ("surface", 8)], threads: 1, live: false },
    },
];

struct Outcome {
    /// With time travel: checkpoints taken, the time they took, the memory they hold at the end.
    travel: Option<sim_host::timetravel::Stats>,
    wall: Duration,
    /// Of `instructions`, those not interpreted (busy-waits skipped).
    skipped: u64,
    /// CPU time the process used (all threads), when the OS tells.
    cpu: Option<Duration>,
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

fn threads_of(case: &Case) -> usize {
    match case.kind {
        Kind::Robot { threads, .. } | Kind::Nereus { threads } => threads.unsigned_abs(),
        Kind::Board { .. } => 1,
    }
}

fn skipped(s: &Session) -> u64 {
    s.machine.spin_skipped
}

fn instructions(s: &Session) -> u64 {
    s.machine.cores.iter().map(|c| c.instructions()).sum()
}

/// Time travel settings: the time between checkpoints and their memory budget.
#[derive(Clone, Copy)]
struct Travel {
    every: Time,
    budget: usize,
}

fn record(
    target: &mut dyn sim_host::timetravel::Rewindable,
    inputs: Vec<sim_host::board::Inputs>,
    t: Travel,
) -> sim_host::timetravel::TimeTravel {
    let mut tt = sim_host::timetravel::TimeTravel::new(target, inputs);
    tt.every = t.every;
    tt.budget = t.budget;
    tt
}

/// `override_threads`: replaces `talos_mt`'s thread count, and prints where the boards ran.
fn run(
    case: &Case,
    time: Time,
    override_threads: Option<usize>,
    travel: Option<Travel>,
    coverage: bool,
) -> Result<Outcome, String> {
    let mut h = Fnv::default();
    match case.kind {
        Kind::Board { elf, board, client } => {
            let elf = sim_test::titan_rs(elf).ok_or("image not built")?;
            let mut b = Bench::builder(&elf).board(board).bridge(client).boot()?;
            if coverage {
                b.session.machine.enable_coverage();
            }
            let (wall, cpu) = (Instant::now(), cpu_time());
            let mut stats = None;
            match travel {
                None => b.run_until(time).map_err(|e| format!("stopped at {}: {e:?}", b.now()))?,
                Some(t) => {
                    let mut tt = record(&mut b.session, vec![b.inputs.clone()], t);
                    let r = tt.run_until(&mut b.session, time);
                    if r != sim_host::timetravel::Ran::Reached {
                        return Err(format!("stopped at {}: {r:?}", b.now()));
                    }
                    b.session.poll_uarts();
                    stats = Some(tt.stats());
                }
            }
            let (wall, cpu) = (wall.elapsed(), cpu_time().zip(cpu).map(|(b, a)| b - a));
            hash_session(&mut h, &b.session);
            for (t, m) in b.messages() {
                t.hash(&mut h);
                format!("{m:?}").hash(&mut h);
            }
            Ok(Outcome {
                travel: stats,
                wall,
                skipped: skipped(&b.session),
                cpu,
                instructions: instructions(&b.session),
                fingerprint: h.finish(),
            })
        }
        Kind::Robot { file, bridges, live, .. } => {
            let mut r = RobotBench::load(file, bridges, |_| {})?;
            if live {
                for (rb, b) in r.robot.buses.iter_mut().zip(&r.bridges) {
                    rb.endpoints.clear();
                    if let Some(b) = b {
                        rb.endpoints.push(Box::new(Live(b.clone())));
                    }
                }
            }
            r.robot.threads = match override_threads {
                Some(n) if case.name == "talos_mt" => n,
                _ => threads_of(case),
            };
            if coverage {
                for s in r.robot.boards.iter_mut() {
                    s.machine.enable_coverage();
                }
            }
            r.robot.run_ahead = !matches!(case.kind, Kind::Robot { threads, .. } if threads < 0);
            let (wall, cpu) = (Instant::now(), cpu_time());
            let mut stats = None;
            match travel {
                None => r.run_until(time),
                Some(t) => {
                    let inputs = r.robot.inputs.clone();
                    let mut tt = record(&mut r.robot, inputs, t);
                    tt.run_until(&mut r.robot, time);
                    stats = Some(tt.stats());
                }
            }
            let (wall, cpu) = (wall.elapsed(), cpu_time().zip(cpu).map(|(b, a)| b - a));
            if override_threads.is_some() && r.robot.threads != 1 {
                let on: Vec<String> =
                    r.robot.names.iter().zip(r.robot.thread_of_boards()).map(|(n, t)| format!("{n}:{t}")).collect();
                println!("{:<11} threads (0: the calling one): {}", case.name, on.join(" "));
            }
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
            let skipped = r.robot.boards.iter().map(skipped).sum();
            Ok(Outcome { travel: stats, wall, skipped, cpu, instructions, fingerprint: h.finish() })
        }
        Kind::Nereus { threads } => {
            let mut r = RobotBench::load("talos_nereus.toml", &[], |_| {})?;
            r.robot.threads = threads.unsigned_abs();
            r.robot.run_ahead = threads > 0;
            let mut bridges = Vec::new();
            for client in 1..=4 {
                let b = sim_test::MockBridge::new(client, true, Time::ZERO);
                b.set_kill(Some(false));
                r.robot.buses[0].endpoints.push(Box::new(Live(b.clone())));
                bridges.push(b);
            }
            // On its own (wall) clock: off the 10 ms grid of the runs
            r.robot.buses[0].endpoints.push(Box::new(Live(Controller { next: Time::from_us(50_330) })));
            if coverage {
                for s in r.robot.boards.iter_mut() {
                    s.machine.enable_coverage();
                }
            }
            let mut tt = travel.map(|t| {
                let inputs = r.robot.inputs.clone();
                record(&mut r.robot, inputs, t)
            });
            let (wall, cpu) = (Instant::now(), cpu_time());
            while r.now() < time {
                let x = r.now().as_secs_f64();
                let pose = sim_host::vehicle::ExternalPose {
                    position: [x, 0.0, 0.6 + 0.4 * (6.0 * x).sin()],
                    attitude: sim_physics::Quat::from_euler(0.0, 0.0, x),
                    velocity: None,
                    rate: None,
                };
                r.robot.vehicles[0].set_pose(&pose);
                let until = (r.now() + Time::from_ms(10)).min(time);
                match &mut tt {
                    None => r.run_until(until),
                    Some(tt) => {
                        tt.run_until(&mut r.robot, until);
                    }
                }
            }
            let stats = tt.map(|tt| tt.stats());
            let (wall, cpu) = (wall.elapsed(), cpu_time().zip(cpu).map(|(b, a)| b - a));
            for s in &r.robot.boards {
                hash_session(&mut h, s);
            }
            for b in &bridges {
                for (t, m) in &b.state().received {
                    t.hash(&mut h);
                    format!("{m:?}").hash(&mut h);
                }
            }
            let instructions = r.robot.boards.iter().map(instructions).sum();
            let skipped = r.robot.boards.iter().map(skipped).sum();
            Ok(Outcome { travel: stats, wall, skipped, cpu, instructions, fingerprint: h.finish() })
        }
    }
}

/// `complete_controller`'s raw thruster frames (CANmore utility channel 0 of ESC boards 2 and 3), every 20 ms.
struct Controller {
    next: Time,
}

impl CanEndpoint for Controller {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut out = Vec::new();
        while self.next <= until {
            for client in [2u32, 3] {
                let rpm: Vec<u8> = [1200i16, -800, 0, 400].iter().flat_map(|r| r.to_be_bytes()).collect();
                out.push((self.next, CanFrame::classic((client << 6) | (1 << 5) | (1 << 4), false, &rpm)));
            }
            self.next += Time::from_ms(20);
        }
        out
    }

    fn receive(&mut self, _frame: &CanFrame, _at: Time) {}
}

/// User + system CPU time of the process so far (Linux: `/proc/self/stat`, in clock ticks of 10 ms). Less
/// affected than the wall clock by other processes on a busy machine.
fn cpu_time() -> Option<Duration> {
    let stat = std::fs::read_to_string("/proc/self/stat").ok()?;
    // Fields after the command name (which is in parentheses and may contain spaces): utime and stime are the
    // 12th and 13th of them
    let rest = &stat[stat.rfind(')')? + 2..];
    let f: Vec<&str> = rest.split_whitespace().collect();
    let ticks: u64 = f.get(11)?.parse::<u64>().ok()? + f.get(12)?.parse::<u64>().ok()?;
    Some(Duration::from_millis(ticks * 10))
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
    let mut threads = None;
    let mut travel: Option<Travel> = None;
    let mut budget = sim_host::timetravel::DEFAULT_BUDGET;
    let mut coverage = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| panic!("{a} needs a value"));
        match a.as_str() {
            "--time" => time = parse_time(&value()),
            "--repeat" => repeat = value().parse().expect("--repeat N"),
            "--only" => only = Some(value().split(',').map(str::to_string).collect()),
            "--threads" => threads = Some(value().parse().expect("--threads N")),
            "--checkpoint-every" => travel = Some(Travel { every: parse_time(&value()), budget }),
            "--coverage" => coverage = true,
            "--checkpoint-memory" => budget = value().parse::<usize>().expect("--checkpoint-memory MB") << 20,
            "--bench" => {} // passed by `cargo bench`-style runners
            other => panic!("unknown argument {other} (--time T, --repeat N, --only a,b, --threads N)"),
        }
    }
    if let Some(t) = &mut travel {
        t.budget = budget;
    }
    println!(
        "{:<11} {:>8} {:>9} {:>9} {:>8} {:>14} {:>8} {:>8}  fingerprint",
        "case", "virtual", "wall", "virt/wall", "cpu", "instructions", "skipped", "MIPS"
    );
    for case in CASES {
        if only.as_ref().is_some_and(|o| !o.iter().any(|n| n == case.name)) {
            continue;
        }
        let mut best: Option<Outcome> = None;
        for _ in 0..repeat.max(1) {
            let o = match run(case, time, threads, travel, coverage) {
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
        let cpu = b.cpu.map_or("-".to_string(), |c| format!("{:.2}s", c.as_secs_f64()));
        println!(
            "{:<11} {:>8} {:>8.3}s {:>9.2} {:>8} {:>14} {:>7.1}% {:>8.1}  {:016x}",
            case.name,
            time.to_string(),
            wall,
            time.as_secs_f64() / wall,
            cpu,
            b.instructions,
            100.0 * b.skipped as f64 / b.instructions.max(1) as f64,
            b.instructions as f64 / wall / 1e6,
            b.fingerprint
        );
        if let Some(st) = b.travel {
            println!(
                "{:<11} checkpoints: {} taken, {:.3} ms each ({:.1}% of the wall time), {} kept, {:.1} MB",
                "",
                st.taken,
                st.taking_ns as f64 / 1e6 / st.taken.max(1) as f64,
                100.0 * st.taking_ns as f64 / 1e9 / wall,
                st.checkpoints,
                st.bytes as f64 / (1 << 20) as f64
            );
        }
    }
}
