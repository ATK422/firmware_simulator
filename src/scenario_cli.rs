//! `firmware_simulator scenario` and `firmware_simulator compare`: YAML scenario files run as tests, and the same
//! scenarios run against two firmware builds and compared.

use std::io::Write as _;
use std::path::PathBuf;

use anyhow::{Context, Result};
use sim_test::scenario::{self, ImageOverride, Options, Outcome};

#[derive(clap::Args)]
pub struct ScenarioArgs {
    /// Scenario files, or directories to run every `*.yaml` in (recursively).
    #[arg(required = true, value_name = "FILE_OR_DIR")]
    paths: Vec<PathBuf>,
    /// Scenarios run side by side on this many threads (0: one per core). The results are the same, and printed in
    /// the same order, on any number.
    #[arg(short, long, default_value_t = 0)]
    jobs: usize,
    /// Writes JUnit XML (a test suite per file, a test case per scenario) for CI.
    #[arg(long, value_name = "FILE")]
    junit: Option<PathBuf>,
    /// Writes the results as JSON (`-`: to stdout, instead of the text).
    #[arg(long, value_name = "FILE")]
    json: Option<PathBuf>,
    /// Runs another firmware build: a directory of images, an image (for every board running one of that file
    /// name) or `BOARD=IMAGE` (repeatable).
    #[arg(long, value_name = "DIR|IMAGE|BOARD=IMAGE", value_parser = parse_override)]
    image: Vec<ImageOverride>,
    /// Prints only the scenarios that failed (and the summary).
    #[arg(short, long)]
    quiet: bool,
    /// Checks the files and lists each scenario's steps, without running them.
    #[arg(long)]
    check: bool,
    /// Scenarios that reach the `pause` step of this name the same way (same setup, bridges, inputs, images, time
    /// and steps up to it) share one run up to there and go on from copies of its state.
    #[arg(long, value_name = "NAME")]
    from_checkpoint: Option<String>,
    /// Collects firmware code coverage over every scenario and writes it to this directory: `lcov.info` (every
    /// image), `IMAGE.info` per image and `summary.txt` (functions and lines hit per crate). The boards run one
    /// instruction at a time.
    #[arg(long, value_name = "DIR")]
    coverage: Option<PathBuf>,
}

fn parse_override(s: &str) -> Result<ImageOverride, String> {
    s.parse()
}

/// Runs the scenarios; the exit status is 0 if all passed, 1 if any failed, 2 if a file could not be read.
pub fn scenario(args: ScenarioArgs) -> Result<i32> {
    let scenarios = match scenario::load(&args.paths) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: {e}");
            return Ok(2);
        }
    };
    if args.check {
        for s in &scenarios {
            println!(
                "{}  ({}): {} for {}",
                s.name,
                s.file.display(),
                setup_text(&s.setup),
                scenario::short_time(s.time)
            );
            for st in s.steps.iter() {
                println!("  {:>3}  {}", st.number, scenario::describe(st));
            }
            for c in &s.cases {
                println!("  case {:?}:", c.name);
                for st in &c.steps {
                    println!("  {:>3}  {}", st.number, scenario::describe(st));
                }
            }
        }
        println!(
            "{} scenario(s) in {} file(s): OK",
            scenarios.len(),
            scenario::files(&args.paths).map_err(anyhow::Error::msg)?.len()
        );
        return Ok(0);
    }
    let opts = Options {
        images: args.image.clone(),
        from_checkpoint: args.from_checkpoint.clone(),
        coverage: args.coverage.is_some(),
        ..Default::default()
    };
    let to_stdout = args.json.as_deref() == Some(std::path::Path::new("-"));
    let print = !to_stdout;
    let quiet = args.quiet;
    let outcomes = run_in_order(
        &scenarios,
        args.jobs,
        |s| scenario::run(s, &opts),
        |o: &Vec<Outcome>| {
            if print {
                for o in o {
                    if !quiet || !o.passed() {
                        print!("{}", scenario::human(o));
                    }
                }
                let _ = std::io::stdout().flush();
            }
        },
    );
    let outcomes: Vec<Outcome> = outcomes.into_iter().flatten().collect();
    if print {
        println!("{}", scenario::summary(&outcomes));
    }
    if let Some(dir) = &args.coverage {
        write_coverage(dir, &outcomes)?;
    }
    if let Some(path) = &args.junit {
        std::fs::write(path, scenario::junit(&outcomes)).with_context(|| format!("writing {}", path.display()))?;
    }
    if let Some(path) = &args.json {
        let text = serde_json::to_string_pretty(&scenario::json(&outcomes))? + "\n";
        if to_stdout {
            print!("{text}");
        } else {
            std::fs::write(path, text).with_context(|| format!("writing {}", path.display()))?;
        }
    }
    Ok(if outcomes.iter().all(Outcome::passed) { 0 } else { 1 })
}

/// Merges the scenarios' coverage and writes it to `dir` (see `--coverage`).
fn write_coverage(dir: &std::path::Path, outcomes: &[Outcome]) -> Result<()> {
    let mut all = sim_host::coverage::Coverage::new();
    for o in outcomes {
        if let Some(c) = &o.coverage {
            all.merge(c);
        }
    }
    std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let report = all.report();
    report.write_lcov(&dir.join("lcov.info"), "scenarios").with_context(|| format!("writing in {}", dir.display()))?;
    let mut summary = String::new();
    for name in all.names() {
        let r = all.report_of(&name).expect("collected");
        r.write_lcov(&dir.join(format!("{name}.info")), &name)?;
        summary.push_str(&format!("{name}:\n{}\n", r.summary()));
    }
    summary.push_str(&format!("every image:\n{}", report.summary()));
    std::fs::write(dir.join("summary.txt"), &summary)?;
    eprint!("{summary}");
    eprintln!("coverage: {}", dir.join("lcov.info").display());
    Ok(())
}

pub fn setup_text(s: &scenario::Setup) -> String {
    match s {
        scenario::Setup::Robot { file, boards: None } => format!("robot {}", file.display()),
        scenario::Setup::Robot { file, boards: Some(b) } => format!("robot {} ({})", file.display(), b.join(", ")),
        scenario::Setup::Board { name, image, .. } => format!("board {name} ({})", image.display()),
    }
}

/// Runs `f` on each scenario on `jobs` threads and hands each result to `done` as soon as it and every one before
/// it are finished, so the output is in order while the runs overlap.
pub fn run_in_order<I: Sync, T: Send>(
    scenarios: &[I],
    jobs: usize,
    f: impl Fn(&I) -> T + Sync,
    mut done: impl FnMut(&T),
) -> Vec<T> {
    let (tx, rx) = std::sync::mpsc::channel::<(usize, T)>();
    let index: Vec<(usize, &I)> = scenarios.iter().enumerate().collect();
    let mut out: Vec<Option<T>> = scenarios.iter().map(|_| None).collect();
    std::thread::scope(|s| {
        let f = &f;
        let index = &index;
        s.spawn(move || {
            let jobs = if jobs == 0 { std::thread::available_parallelism().map_or(1, |n| n.get()) } else { jobs };
            let next = std::sync::atomic::AtomicUsize::new(0);
            std::thread::scope(|inner| {
                for _ in 0..jobs.min(index.len()).max(1) {
                    let tx = tx.clone();
                    let next = &next;
                    inner.spawn(move || {
                        loop {
                            let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some((i, sc)) = index.get(k) else { break };
                            if tx.send((*i, f(sc))).is_err() {
                                break;
                            }
                        }
                    });
                }
            });
        });
        let mut shown = 0;
        for (i, r) in rx {
            out[i] = Some(r);
            while let Some(Some(r)) = out.get(shown) {
                done(r);
                shown += 1;
            }
        }
    });
    out.into_iter().map(|o| o.expect("every scenario ran")).collect()
}

#[derive(clap::Args)]
pub struct CompareArgs {
    /// Firmware build A: a directory of images (e.g. a `TITAN_RS_RELEASE_DIR`), an image, or `BOARD=IMAGE`
    /// (repeatable). Without it, A runs the scenario's own images.
    #[arg(long = "a", value_name = "DIR|IMAGE|BOARD=IMAGE", value_parser = parse_override)]
    a: Vec<ImageOverride>,
    /// Firmware build B, the same way.
    #[arg(long = "b", value_name = "DIR|IMAGE|BOARD=IMAGE", value_parser = parse_override)]
    b: Vec<ImageOverride>,
    /// What to call A in the report (default: what --a gave).
    #[arg(long, value_name = "TEXT")]
    a_label: Option<String>,
    /// What to call B in the report.
    #[arg(long, value_name = "TEXT")]
    b_label: Option<String>,
    /// Scenario files or directories.
    #[arg(required = true, value_name = "SCENARIO")]
    paths: Vec<PathBuf>,
    /// Runs side by side on this many threads (0: one per core).
    #[arg(short, long, default_value_t = 0)]
    jobs: usize,
    /// Writes the Markdown report here instead of to stdout.
    #[arg(short, long, value_name = "FILE")]
    output: Option<PathBuf>,
    /// Writes the comparison as JSON.
    #[arg(long, value_name = "FILE")]
    json: Option<PathBuf>,
    /// Times closer than this count as the same (log lines, boot and connect times, CAN streams).
    #[arg(long, default_value = "20ms", value_parser = parse_time)]
    tolerance: sim_core::time::Time,
    /// Examples shown per kind of difference.
    #[arg(long, default_value_t = 8)]
    details: usize,
    /// Also profiles every board's firmware and compares the CPU load (running share per core, samples per
    /// function). Profiling cuts the runs at the sample times, which can shift how two cores interleave.
    #[arg(long)]
    profile: bool,
    /// The profiler's mean sampling interval.
    #[arg(long, default_value = "50us", value_parser = parse_time)]
    profile_interval: sim_core::time::Time,
    /// Exits with 1 when a step that passes with A does not pass with B.
    #[arg(long)]
    fail_on_regression: bool,
}

fn parse_time(s: &str) -> Result<sim_core::time::Time, String> {
    scenario::parse_time(s)
}

fn label(given: &Option<String>, o: &[ImageOverride]) -> String {
    if let Some(l) = given {
        return l.clone();
    }
    if o.is_empty() {
        return "the scenario's images".into();
    }
    o.iter()
        .map(|x| match x {
            ImageOverride::Dir(d) | ImageOverride::File(d) => format!("`{}`", d.display()),
            ImageOverride::Board(b, p) => format!("`{b}={}`", p.display()),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// Runs every scenario with both builds and writes the report; the exit status is 0 (1 with
/// `--fail-on-regression` and a regression, 2 if a file could not be read).
pub fn compare(args: CompareArgs) -> Result<i32> {
    use sim_test::compare::{CompareOptions, compare};
    let scenarios = match scenario::load(&args.paths) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: {e}");
            return Ok(2);
        }
    };
    let labels = (label(&args.a_label, &args.a), label(&args.b_label, &args.b));
    let side = |images: &[ImageOverride]| Options {
        images: images.to_vec(),
        record: true,
        profile: args.profile.then_some(args.profile_interval),
        skip_missing: false,
        from_checkpoint: None,
        coverage: false,
    };
    let (oa, ob) = (side(&args.a), side(&args.b));
    // Each scenario twice, A and B side by side
    let twice: Vec<(&scenario::Scenario, &Options)> = scenarios.iter().flat_map(|s| [(s, &oa), (s, &ob)]).collect();
    let runs = run_in_order(
        &twice,
        args.jobs,
        |(s, o)| scenario::run(s, o),
        |o: &Vec<Outcome>| {
            for o in o {
                eprintln!("{} {}", if o.passed() { "ran" } else { "ran (failed)" }, o.title());
            }
        },
    );
    let opts = CompareOptions { tolerance: args.tolerance, details: args.details };
    let mut report = String::new();
    let mut all = Vec::new();
    let mut regressions = 0;
    for pair in runs.chunks(2) {
        for (a, b) in pair[0].iter().zip(&pair[1]) {
            let c = compare(a, b, (&labels.0, &labels.1), &opts);
            regressions += c.regressions().len();
            report += &c.markdown();
            all.push(c.json());
        }
    }
    match &args.output {
        Some(p) => std::fs::write(p, &report).with_context(|| format!("writing {}", p.display()))?,
        None => print!("{report}"),
    }
    if let Some(p) = &args.json {
        let text =
            serde_json::to_string_pretty(&serde_json::json!({"a": labels.0, "b": labels.1, "comparisons": all}))?
                + "\n";
        std::fs::write(p, text).with_context(|| format!("writing {}", p.display()))?;
    }
    eprintln!("{} comparison(s), {regressions} regression(s)", all.len());
    Ok(if args.fail_on_regression && regressions > 0 { 1 } else { 0 })
}
