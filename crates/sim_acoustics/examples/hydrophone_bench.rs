//! The hydrophone alone while a packet crosses the pool: another board's projector driven the whole time, heard over
//! the pool's 63 paths (and the pinger's, silent but for its pulses), sampled at 500 kS/s as an IVC board's ADC
//! does, with the path sum in SIMD lanes and step by step (`Hydrophone::lanes`). Prints the time each took and a
//! hash of every sample, which must be the same.
//!
//! ```text
//! cargo run --release -p sim_acoustics --example hydrophone_bench -- [--secs 1] [--repeat 3] [--only fixed,moving]
//!     [--lanes on|off]
//! ```
//!
//! `--lanes` runs only one of the two (for callgrind).
//!
//! Cases: `fixed` (both ends still), `moving` (the hydrophone closing at 0.5 m/s: the taps are interpolated and
//! rebuilt every block), `gaps` (fixed, sampled in 2 ms captures with 3 ms between them, each restarting the chain
//! from its preroll, as the IVC boards do), and `silent`: fixed, the projector never driven, so that the hydrophone
//! leaves its paths out (what the hydrophone costs without the path sum).

use std::hash::Hasher;
use std::sync::Arc;
use std::time::{Duration, Instant};

use sim_acoustics::chain::ChainSpec;
use sim_acoustics::medium::{DriveSpec, Medium, Trajectory};
use sim_acoustics::transducer::Transducer;
use sim_acoustics::{Environment, Hydrophone};
use sim_core::time::Time;

fn setup(secs: u64, rx: Trajectory, driven: bool) -> Hydrophone {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../environments/pool.toml");
    let env = Arc::new(Environment::load(&dir).unwrap());
    let medium = Medium::shared(env.clone());
    let t: Transducer =
        toml::from_str("tvr = { resonance_hz = 51_000, q = 8, peak_db = 140 }\nrvs_db = -190\nbeam = \"omni\"")
            .unwrap();
    {
        let mut m = medium.lock().unwrap();
        let tx = Trajectory::Fixed([10.0, 6.0, 1.5]);
        let id = m.add_projector(0, "tx", t.clone(), tx, &DriveSpec { pin: 7, supply_v: 12.0, nominal_hz: None });
        // A square wave at the resonance the whole time
        let half = Time::from_ps(1_000_000_000_000 / 102_000);
        let (mut at, mut level) = (Time::ZERO, true);
        while driven && at < Time::from_secs(secs + 1) {
            m.edge(id, at, level);
            level = !level;
            at += half;
        }
        m.progress(0, Time::from_secs(secs + 1));
    }
    let chain = ChainSpec { gain_db: 40.0, bandpass_hz: Some([40_000.0, 60_000.0]), ..Default::default() };
    Hydrophone::new(env, [5.0, 6.0, 1.5], t, chain).in_medium(medium, 1).moving(rx)
}

/// Samples `h` for `secs` at 500 kS/s (in captures if `gaps`); returns the time taken and a hash of the samples.
fn run(mut h: Hydrophone, secs: u64, gaps: bool) -> (Duration, Option<Duration>, u64) {
    let mut hash = sim_core::sched::Fnv::default();
    let (wall, cpu) = (Instant::now(), cpu_time());
    let mut t = Time::from_ms(100);
    let end = t + Time::from_secs(secs);
    let mut k = 0u64;
    while t < end {
        hash.write_u64(h.sample(t, 1).to_bits());
        k += 1;
        t += if gaps && k.is_multiple_of(1000) { Time::from_us(3_002) } else { Time::from_us(2) };
    }
    (wall.elapsed(), cpu_time().zip(cpu).map(|(b, a)| b - a), hash.finish())
}

/// User + system CPU time of the process so far (Linux), as the emulator's bench measures it.
fn cpu_time() -> Option<Duration> {
    let stat = std::fs::read_to_string("/proc/self/stat").ok()?;
    let rest = &stat[stat.rfind(')')? + 2..];
    let f: Vec<&str> = rest.split_whitespace().collect();
    let ticks: u64 = f.get(11)?.parse::<u64>().ok()? + f.get(12)?.parse::<u64>().ok()?;
    Some(Duration::from_millis(ticks * 10))
}

fn main() {
    let (mut secs, mut repeat, mut only) = (1u64, 3usize, None::<Vec<String>>);
    let mut which = vec![false, true];
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| panic!("{a} needs a value"));
        match a.as_str() {
            "--secs" => secs = value().parse().expect("--secs N"),
            "--repeat" => repeat = value().parse().expect("--repeat N"),
            "--only" => only = Some(value().split(',').map(str::to_string).collect()),
            "--lanes" => which = vec![value() == "on"],
            "--bench" => {}
            other => panic!("unknown argument {other} (--secs N, --repeat N, --only a,b)"),
        }
    }
    let moving = Trajectory::Linear { start_m: [5.0, 6.0, 1.5], velocity_mps: [0.5, 0.0, 0.0] };
    let cases = [
        ("fixed", Trajectory::Fixed([5.0, 6.0, 1.5]), false),
        ("moving", moving, false),
        ("gaps", Trajectory::Fixed([5.0, 6.0, 1.5]), true),
        ("silent", Trajectory::Fixed([5.0, 6.0, 1.5]), false),
    ];
    println!("{:<8} {:>6} {:>10} {:>10} {:>9}  hash", "case", "lanes", "wall", "cpu", "speed-up");
    for (name, rx, gaps) in cases {
        if only.as_ref().is_some_and(|o| !o.iter().any(|n| n == name)) {
            continue;
        }
        let mut best = [Duration::MAX; 2];
        let mut cpus = [None; 2];
        let mut hashes = [0u64; 2];
        for _ in 0..repeat.max(1) {
            for (i, &lanes) in [false, true].iter().enumerate().filter(|(_, l)| which.contains(l)) {
                let mut h = setup(secs, rx.clone(), name != "silent");
                h.lanes = lanes;
                let (wall, cpu, hash) = run(h, secs, gaps);
                if wall < best[i] {
                    (best[i], cpus[i]) = (wall, cpu);
                }
                hashes[i] = hash;
            }
        }
        if which.len() == 2 {
            assert_eq!(hashes[0], hashes[1], "{name}: the lanes changed a sample");
        }
        for (i, lanes) in ["off", "on"].into_iter().enumerate().filter(|&(i, _)| which.contains(&(i == 1))) {
            let cpu = cpus[i].map_or("-".to_string(), |c| format!("{:.3}s", c.as_secs_f64()));
            let speed = match which.len() {
                2 => format!("{:.2}x", best[0].as_secs_f64() / best[i].as_secs_f64()),
                _ => "-".to_string(),
            };
            println!("{name:<8} {lanes:>6} {:>9.3}s {cpu:>10} {speed:>9}  {:016x}", best[i].as_secs_f64(), hashes[i]);
        }
    }
}
