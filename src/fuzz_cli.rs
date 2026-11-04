//! `firmware_simulator fuzz`: drive many seeded, deterministic cases of malformed CAN traffic at a board (or
//! every board) from one boot, watch the oracles for crashes, hangs, watchdog resets and fault states, and write
//! each distinct finding out as a minimised scenario reproducer and JSON record.

use std::io::Write as _;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use sim_core::time::Time;
use sim_test::fuzz::{self, Campaign, Config};
use sim_test::scenario::parse_time;

#[derive(clap::Args)]
pub struct FuzzArgs {
    /// Boards to fuzz by image name (`esc_board`, `power_board`, ...); none means every board (see
    /// `docs/fuzzing.md`).
    #[arg(value_name = "BOARD")]
    boards: Vec<String>,
    /// Wall-time budget shared out over the boards that boot: `25m`, `90s`, `1h`, `1500ms` (a bare number is
    /// seconds). Give this or `--cases`.
    #[arg(long, value_name = "DURATION")]
    time: Option<String>,
    /// Run this many cases per board instead of a time budget.
    #[arg(long)]
    cases: Option<u64>,
    /// Worker threads per board.
    #[arg(short, long, default_value_t = 2)]
    jobs: usize,
    /// The campaign seed (the whole run is a deterministic function of it).
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// How long to drive fuzz traffic in each case.
    #[arg(long, default_value = "200ms", value_parser = parse_time)]
    window: Time,
    /// Quiet time after the traffic stops, to tell a hang from a slow board.
    #[arg(long, default_value = "1200ms", value_parser = parse_time)]
    settle: Time,
    /// Largest number of operations in a generated case.
    #[arg(long, default_value_t = 16)]
    max_ops: u32,
    /// Write minimised reproducers (a `*.yaml` scenario and a `*.json` record per distinct finding) here.
    #[arg(long, value_name = "DIR")]
    out: Option<PathBuf>,
    /// Confirm each reproducer by running its scenario, and report whether it replayed.
    #[arg(long)]
    confirm: bool,
}

pub fn run(args: FuzzArgs) -> Result<()> {
    if args.time.is_some() && args.cases.is_some() {
        bail!("give --time or --cases, not both");
    }
    let budget = args.time.as_deref().map(parse_wall).transpose().map_err(anyhow::Error::msg)?;
    let targets: Vec<fuzz::Target> = if args.boards.is_empty() {
        fuzz::default_targets()
    } else {
        args.boards
            .iter()
            .map(|b| fuzz::target(b).with_context(|| format!("unknown board {b:?} (try: power_board, esc_board, ...)")))
            .collect::<Result<_>>()?
    };
    if let Some(dir) = &args.out {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }

    let cfg = Config {
        seed: args.seed,
        window: args.window,
        settle: args.settle,
        max_ops: args.max_ops,
        ..Config::default()
    };

    // Share the wall-time budget across the boards that actually boot
    let built: Vec<fuzz::Target> = targets.into_iter().filter(|t| sim_test::titan_rs(&t.image).is_some()).collect();
    if built.is_empty() {
        bail!("no board images built; set TITAN_RS_RELEASE_DIR or build the firmware");
    }
    let per_board = budget.map(|b| Time(b.0 / built.len().max(1) as u64));

    sim_host::signal::install().ok();

    println!("board               cases     corpus   edges   findings  wall");
    let mut total_findings = 0usize;
    let mut all_hits: Vec<(String, fuzz::Hit)> = Vec::new();
    for target in &built {
        let started = Instant::now();
        let campaign = match Campaign::boot(target, &cfg) {
            Ok(c) => c,
            Err(e) => {
                println!("{:<18}  boot failed: {e}", target.image);
                continue;
            }
        };
        let deadline = per_board.map(|d| started + Duration::from_secs_f64(d.as_secs_f64()));
        let stop = move || sim_host::signal::requested() || deadline.is_some_and(|d| Instant::now() >= d);
        let cases = args.cases.unwrap_or(u64::MAX);
        let report = fuzz::run_campaign(&campaign, cases, args.jobs, &stop);
        println!(
            "{:<18}  {:>7}  {:>7}  {:>6}  {:>8}  {:.1}s",
            target.image,
            report.cases,
            report.corpus,
            report.edges,
            report.hits.len(),
            started.elapsed().as_secs_f64(),
        );
        for f in campaign.noise() {
            println!("    (clean board, not counted) [{}] {}", f.kind.word(), f.detail);
        }
        for hit in &report.hits {
            let f = &hit.findings[0];
            println!("    [{}] {}  ({} op(s))", f.kind.word(), f.detail, hit.input.ops.len());
            total_findings += 1;
            if let Some(dir) = &args.out {
                write_reproducer(dir, &campaign, hit, args.confirm)?;
            }
            all_hits.push((target.image.clone(), hit.clone()));
        }
        if sim_host::signal::requested() {
            println!("interrupted");
            break;
        }
    }

    println!("\n{} finding(s) across {} board(s)", total_findings, built.len());
    if let Some(dir) = &args.out {
        println!("reproducers in {}", dir.display());
    }
    Ok(())
}

fn write_reproducer(dir: &std::path::Path, campaign: &Campaign, hit: &fuzz::Hit, confirm: bool) -> Result<()> {
    let target = campaign.target();
    let f = &hit.findings[0];
    let safe_key: String = f.key.chars().map(|c| if c.is_alphanumeric() { c } else { '_' }).collect();
    let stem = format!("fuzz_{}_{}", target.image, safe_key);
    let shared = campaign.shared();
    let yaml = sim_test::fuzz::repro::scenario_yaml(
        target,
        campaign_snap_at(campaign),
        campaign_window(campaign),
        campaign_settle(campaign),
        &hit.input,
        f,
        campaign.params().fd,
    );
    let yaml_path = dir.join(format!("{stem}.yaml"));
    std::fs::write(&yaml_path, &yaml).with_context(|| format!("writing {}", yaml_path.display()))?;
    let record = sim_test::fuzz::repro::record_json(target, &hit.input, f);
    let json_path = dir.join(format!("{stem}.json"));
    std::fs::write(&json_path, serde_json::to_string_pretty(&record)?)
        .with_context(|| format!("writing {}", json_path.display()))?;
    let _ = shared;
    if confirm {
        let scenarios = sim_test::scenario::load(std::slice::from_ref(&yaml_path)).map_err(anyhow::Error::msg)?;
        let opts = sim_test::scenario::Options { skip_missing: false, ..Default::default() };
        let outcomes = sim_test::scenario::run_all(&scenarios, &opts, 1);
        let ok = outcomes.iter().all(|o| o.passed());
        print!("        reproducer {}: {}", yaml_path.display(), if ok { "replays" } else { "DID NOT replay" });
        std::io::stdout().flush().ok();
        println!();
    }
    Ok(())
}

// The campaign keeps its config and snapshot time privately; expose what the reproducer needs through accessors.
fn campaign_snap_at(c: &Campaign) -> Time {
    c.shared().snap_at()
}
fn campaign_window(c: &Campaign) -> Time {
    c.shared().window()
}
fn campaign_settle(c: &Campaign) -> Time {
    c.shared().settle()
}

/// A wall-clock duration: `1h`, `25m`, `90s`, `1500ms`, or a bare number of seconds.
fn parse_wall(s: &str) -> Result<Time, String> {
    let s = s.trim();
    let (num, scale) = if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix('h') {
        (n, 3600.0)
    } else if let Some(n) = s.strip_suffix('m') {
        (n, 60.0)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1.0)
    };
    let v: f64 = num.trim().parse().map_err(|_| format!("bad duration {s:?} (write e.g. 25m, 90s, 1h)"))?;
    if !v.is_finite() || v < 0.0 {
        return Err(format!("bad duration {s:?}"));
    }
    Ok(Time::from_secs_f64(v * scale))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wall_durations() {
        assert_eq!(parse_wall("25m").unwrap(), Time::from_secs(1500));
        assert_eq!(parse_wall("1h").unwrap(), Time::from_secs(3600));
        assert_eq!(parse_wall("90s").unwrap(), Time::from_secs(90));
        assert_eq!(parse_wall("1500ms").unwrap(), Time::from_ms(1500));
        assert_eq!(parse_wall("2").unwrap(), Time::from_secs(2));
        for bad in ["", "m", "x5s", "-1s"] {
            assert!(parse_wall(bad).is_err(), "{bad}");
        }
    }
}
