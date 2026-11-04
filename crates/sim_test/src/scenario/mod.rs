//! Scenario files: firmware integration tests written in YAML instead of Rust.
//!
//! A scenario names a robot file (or one board and its image), the mock bridges, the inputs, how long to run and a
//! list of steps: actions (faults, inputs, titan_link messages, IVC bytes) and checks (`expect`, `never`,
//! `always`) on what the boards log, what the bridges decode, the frames on the buses and the boards' state. The
//! format is in `docs/scenarios.md`; `firmware_simulator scenario FILE...` runs them, and so does
//! [`run_paths`] from a test:
//!
//! ```no_run
//! let opts = sim_test::scenario::Options { skip_missing: true, ..Default::default() };
//! let outcomes = sim_test::scenario::run_paths(&["scenarios".into()], &opts, 4).unwrap();
//! for o in &outcomes {
//!     print!("{}", sim_test::scenario::human(o));
//! }
//! assert!(outcomes.iter().all(|o| o.skipped.is_some() || o.passed()));
//! ```
//!
//! Runs are deterministic: the same files and images give the same steps, times and evidence.

mod output;
mod run;
mod spec;

use std::path::{Path, PathBuf};

pub use output::{human, json, json_one, junit, summary};
pub use run::{
    BoardRecord, ImageOverride, Line, Options, Outcome, Record, Status, StepResult, describe, file_hash,
    override_image, run, short_time,
};
pub use spec::{
    BridgeSpec, Bridges, Case, Condition, End, FaultRef, FrameMatch, PinRef, Range, Scenario, Setup, Start,
    StatusCheck, Step, StepKind, Target, TextMatch, ValueCheck, VehicleQty, What, find_upward, load_file, parse_str,
    parse_time,
};

/// The scenario files under `paths`: files as given, directories searched for `*.yaml` and `*.yml` (recursively,
/// in name order).
pub fn files(paths: &[PathBuf]) -> Result<Vec<PathBuf>, String> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) -> Result<(), String> {
        let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)
            .map_err(|e| format!("{}: {e}", dir.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .collect();
        entries.sort();
        for p in entries {
            if p.is_dir() {
                walk(&p, out)?;
            } else if p.extension().is_some_and(|e| e == "yaml" || e == "yml") {
                out.push(p);
            }
        }
        Ok(())
    }
    let mut out = Vec::new();
    for p in paths {
        if p.is_dir() {
            walk(p, &mut out)?;
        } else if p.exists() {
            out.push(p.clone());
        } else {
            return Err(format!("{}: no such file or directory", p.display()));
        }
    }
    if out.is_empty() {
        return Err("no scenario files (*.yaml) found".into());
    }
    Ok(out)
}

/// Loads every scenario under `paths` ([`files`]).
pub fn load(paths: &[PathBuf]) -> Result<Vec<Scenario>, String> {
    let mut all = Vec::new();
    let mut errors = Vec::new();
    for f in files(paths)? {
        match load_file(&f) {
            Ok(v) => all.extend(v),
            Err(e) => errors.push(e),
        }
    }
    if errors.is_empty() { Ok(all) } else { Err(errors.join("\n")) }
}

/// Runs `scenarios` on up to `jobs` threads (0: one per core), each with `opts`; the outcomes come back in the
/// order of the scenarios (and of their cases), whatever the number of threads.
pub fn run_all(scenarios: &[Scenario], opts: &Options, jobs: usize) -> Vec<Outcome> {
    run_each(scenarios, jobs, |s| run(s, opts)).into_iter().flatten().collect()
}

/// Calls `f` for each scenario on up to `jobs` threads; results in order.
pub fn run_each<T: Send>(scenarios: &[Scenario], jobs: usize, f: impl Fn(&Scenario) -> T + Sync) -> Vec<T> {
    let jobs = match jobs {
        0 => std::thread::available_parallelism().map_or(1, |n| n.get()),
        n => n,
    }
    .min(scenarios.len())
    .max(1);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let slots: Vec<std::sync::Mutex<Option<T>>> = scenarios.iter().map(|_| std::sync::Mutex::new(None)).collect();
    std::thread::scope(|s| {
        for _ in 0..jobs {
            s.spawn(|| {
                loop {
                    let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let Some(sc) = scenarios.get(k) else { break };
                    *slots[k].lock().unwrap() = Some(f(sc));
                }
            });
        }
    });
    slots.into_iter().map(|m| m.into_inner().unwrap().expect("every scenario ran")).collect()
}

/// Loads and runs every scenario under `paths`.
pub fn run_paths(paths: &[PathBuf], opts: &Options, jobs: usize) -> Result<Vec<Outcome>, String> {
    Ok(run_all(&load(paths)?, opts, jobs))
}

/// For a `#[test]`: runs every scenario under `paths` (missing images skip, unless `SIM_REQUIRE_TITAN_RS` is set),
/// prints each one's steps, and panics naming the ones that failed.
pub fn assert_pass(paths: &[PathBuf], jobs: usize) {
    let opts = Options { skip_missing: true, ..Default::default() };
    let outcomes = run_paths(paths, &opts, jobs).unwrap_or_else(|e| panic!("{e}"));
    for o in &outcomes {
        if let Some(why) = &o.skipped {
            eprintln!("skipping: {} ({why})", o.title());
        } else {
            eprint!("{}", human(o));
        }
    }
    let line = summary(&outcomes);
    eprintln!("{line}");
    assert!(outcomes.iter().all(|o| o.skipped.is_some() || o.passed()), "{line}");
}
