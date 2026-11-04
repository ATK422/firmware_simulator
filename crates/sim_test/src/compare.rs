//! Firmware A/B comparisons: the same scenario run with two builds, and what changed between the runs.
//!
//! [`compare`] takes the [`Outcome`]s of the two runs (with their [`Record`]s, from
//! [`Options::record`](crate::scenario::Options)) and lines up: each step's result; each board's log lines (by
//! content, numbers aside, with their times compared within a tolerance); the CAN traffic on each bus (by
//! [`sim_host::candiff`], stream by stream: counts, rates, first-seen times, payloads); boot and connect times; the
//! faults raised; resets and lockups; ESC speeds; and, when the runs were profiled, CPU load per function.
//! [`Comparison::markdown`] is a report for a pull request comment, [`Comparison::json`] the same for scripts.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use sim_core::time::Time;
use sim_host::candiff;

use crate::scenario::{BoardRecord, Line, Outcome, Record, Status, short_time};

/// How to compare.
#[derive(Clone, Debug)]
pub struct CompareOptions {
    /// Times closer than this are the same (log lines, milestones, first-seen times, CAN pairing).
    pub tolerance: Time,
    /// Examples shown per kind of difference.
    pub details: usize,
}

impl Default for CompareOptions {
    fn default() -> Self {
        CompareOptions { tolerance: Time::from_ms(20), details: 8 }
    }
}

/// One step in both runs.
#[derive(Clone, Debug)]
pub struct StepDiff {
    pub number: usize,
    pub description: String,
    pub a: (Status, Option<Time>, String),
    pub b: (Status, Option<Time>, String),
}

impl StepDiff {
    pub fn status_changed(&self) -> bool {
        self.a.0 != self.b.0
    }
}

/// How the lines of one board compare.
#[derive(Clone, Debug, Default)]
pub struct LineDiff {
    /// Only in B, only in A.
    pub added: Vec<Line>,
    pub removed: Vec<Line>,
    /// The same message with other numbers in it (A's, B's).
    pub changed: Vec<(Line, Line)>,
    /// The same line, logged more than the tolerance apart.
    pub retimed: Vec<(Line, Line)>,
    pub same: usize,
}

impl LineDiff {
    pub fn differs(&self) -> bool {
        !(self.added.is_empty() && self.removed.is_empty() && self.changed.is_empty() && self.retimed.is_empty())
    }
}

/// A moment in a board's run, in both runs.
#[derive(Clone, Debug)]
pub struct Milestone {
    pub what: String,
    pub a: Option<Time>,
    pub b: Option<Time>,
}

/// A fault in both runs: when its "raised" line was first logged, and when a status first carried its bit.
#[derive(Clone, Debug)]
pub struct FaultDiff {
    pub name: String,
    pub logged: (Option<Time>, Option<Time>),
    pub in_status: (Option<Time>, Option<Time>),
    /// Whether the last status of each run had it.
    pub at_end: (bool, bool),
}

/// An ESC's speed over the run: mean, min, max and the last, rpm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Speeds {
    pub mean: f64,
    pub min: f64,
    pub max: f64,
    pub last: f64,
}

impl Speeds {
    fn of(v: &[f64]) -> Option<Speeds> {
        if v.is_empty() {
            return None;
        }
        Some(Speeds {
            mean: v.iter().sum::<f64>() / v.len() as f64,
            min: v.iter().copied().fold(f64::INFINITY, f64::min),
            max: v.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            last: *v.last().unwrap(),
        })
    }

    fn close(a: &Speeds, b: &Speeds) -> bool {
        let near = |x: f64, y: f64| (x - y).abs() <= 5.0_f64.max(0.01 * x.abs().max(y.abs()));
        near(a.mean, b.mean) && near(a.min, b.min) && near(a.max, b.max) && near(a.last, b.last)
    }
}

/// CPU load of one board: per core the share of samples running, and the functions by self samples.
#[derive(Clone, Debug, Default)]
pub struct CpuDiff {
    /// Per core: running share in A and B (0 to 1).
    pub running: Vec<(f64, f64)>,
    /// Function, core, self samples in A and B (the union of each run's top functions).
    pub functions: Vec<(String, usize, u64, u64)>,
}

#[derive(Clone, Debug)]
pub struct BoardDiff {
    pub name: String,
    pub images: (std::path::PathBuf, std::path::PathBuf),
    pub hashes: (u64, u64),
    pub milestones: Vec<Milestone>,
    pub faults: Vec<FaultDiff>,
    pub resets: (Vec<Time>, Vec<Time>),
    pub lockups: (u64, u64),
    pub lines: LineDiff,
    pub escs: Vec<(String, Option<Speeds>, Option<Speeds>)>,
    pub cpu: Option<CpuDiff>,
}

#[derive(Clone, Debug)]
pub struct BusDiff {
    pub bus: String,
    pub report: candiff::Report,
    /// Each stream's first event in A and B.
    pub first_seen: BTreeMap<String, (Option<i64>, Option<i64>)>,
}

/// Everything that differs between two runs of one scenario.
#[derive(Clone, Debug)]
pub struct Comparison {
    pub title: String,
    pub file: String,
    pub labels: (String, String),
    pub errors: (Option<String>, Option<String>),
    pub steps: Vec<StepDiff>,
    pub boards: Vec<BoardDiff>,
    pub buses: Vec<BusDiff>,
    pub end: (Time, Time),
    pub tolerance: Time,
    pub details: usize,
}

/// Lines' text with every number replaced by `#`: lines that differ only in numbers line up.
pub fn shape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut in_num = false;
    for c in text.chars() {
        if c.is_ascii_digit() || (in_num && (c.is_ascii_hexdigit() || c == '.' || c == 'x')) {
            if !in_num {
                out.push('#');
            }
            in_num = true;
        } else {
            out.push(c);
            in_num = false;
        }
    }
    out
}

/// The longest common subsequence of `a` and `b` (Myers' O((N+M)D) algorithm): the index pairs of equal
/// elements, in order. When the sequences differ in more than `max_d` places, only their common start and end
/// are paired.
pub fn lcs<T: PartialEq>(a: &[T], b: &[T], max_d: usize) -> Vec<(usize, usize)> {
    let pre = a.iter().zip(b).take_while(|(x, y)| x == y).count();
    let suf = a[pre..].iter().rev().zip(b[pre..].iter().rev()).take_while(|(x, y)| x == y).count();
    let (ma, mb) = (&a[pre..a.len() - suf], &b[pre..b.len() - suf]);
    let mut pairs: Vec<(usize, usize)> = (0..pre).map(|i| (i, i)).collect();
    let (n, m) = (ma.len() as i64, mb.len() as i64);
    let mut trace: Vec<Vec<i64>> = Vec::new();
    let mut found = None;
    if n == 0 || m == 0 {
        found = Some(0);
        trace.push(vec![0]);
    } else {
        for d in 0..=(n + m).min(max_d as i64) {
            let prev = trace.last();
            let get = |k: i64| -> i64 { prev.map_or(0, |p| p[(k + d - 1) as usize]) };
            let mut v = vec![0i64; (2 * d + 1) as usize];
            for k in (-d..=d).step_by(2) {
                let mut x = if d == 0 {
                    0
                } else if k == -d || (k != d && get(k - 1) < get(k + 1)) {
                    get(k + 1)
                } else {
                    get(k - 1) + 1
                };
                let mut y = x - k;
                while x < n && y < m && ma[x as usize] == mb[y as usize] {
                    x += 1;
                    y += 1;
                }
                v[(k + d) as usize] = x;
                if x >= n && y >= m && found.is_none() {
                    found = Some(d);
                }
            }
            trace.push(v);
            if found.is_some() {
                break;
            }
        }
    }
    let mut mid = Vec::new();
    if let Some(dd) = found {
        let (mut x, mut y) = (n, m);
        for d in (1..=dd).rev() {
            let prev = &trace[(d - 1) as usize];
            let get = |k: i64| prev[(k + d - 1) as usize];
            let k = x - y;
            let down = k == -d || (k != d && get(k - 1) < get(k + 1));
            let pk = if down { k + 1 } else { k - 1 };
            let px = get(pk);
            let py = px - pk;
            let (sx, sy) = if down { (px, py + 1) } else { (px + 1, py) };
            while x > sx && y > sy {
                x -= 1;
                y -= 1;
                mid.push((x as usize, y as usize));
            }
            x = px;
            y = py;
        }
        while x > 0 && y > 0 {
            x -= 1;
            y -= 1;
            mid.push((x as usize, y as usize));
        }
        mid.reverse();
    }
    pairs.extend(mid.into_iter().map(|(x, y)| (x + pre, y + pre)));
    pairs.extend((0..suf).map(|i| (a.len() - suf + i, b.len() - suf + i)));
    pairs
}

/// Lines up two boards' lines by content.
pub fn diff_lines(a: &[Line], b: &[Line], tolerance: Time) -> LineDiff {
    let ka: Vec<(String, String)> = a.iter().map(|l| (l.kind.clone(), shape(&l.text))).collect();
    let kb: Vec<(String, String)> = b.iter().map(|l| (l.kind.clone(), shape(&l.text))).collect();
    let pairs = lcs(&ka, &kb, 4000);
    let mut d = LineDiff::default();
    let (mut i, mut j) = (0, 0);
    for (pi, pj) in pairs.into_iter().chain(std::iter::once((a.len(), b.len()))) {
        d.removed.extend(a[i..pi].iter().cloned());
        d.added.extend(b[j..pj].iter().cloned());
        if pi < a.len() && pj < b.len() {
            let (la, lb) = (&a[pi], &b[pj]);
            if la.text != lb.text {
                d.changed.push((la.clone(), lb.clone()));
            } else if la.t.0.abs_diff(lb.t.0) > tolerance.0 {
                d.retimed.push((la.clone(), lb.clone()));
            } else {
                d.same += 1;
            }
        }
        i = pi + 1;
        j = pj + 1;
    }
    d
}

fn first_line(b: &BoardRecord, text: &str) -> Option<Time> {
    b.lines.iter().find(|l| l.text.contains(text)).map(|l| l.t)
}

fn raised_faults(b: &BoardRecord) -> BTreeMap<String, Time> {
    let mut m = BTreeMap::new();
    for l in &b.lines {
        if let Some(rest) = l.text.strip_prefix("Fault ")
            && let Some((name, tail)) = rest.split_once(' ')
            && tail.starts_with("raised")
        {
            m.entry(name.to_string()).or_insert(l.t);
        }
    }
    m
}

fn status_faults(b: &BoardRecord) -> BTreeMap<String, Time> {
    let mut m = BTreeMap::new();
    for (t, bits) in &b.statuses {
        for bit in 0..32 {
            if bits & (1 << bit) != 0 {
                let name = b.fault_names.get(bit).cloned().unwrap_or_else(|| format!("fault {bit}"));
                m.entry(name).or_insert(*t);
            }
        }
    }
    m
}

fn last_status_faults(b: &BoardRecord) -> Vec<String> {
    let bits = b.statuses.last().map_or(0, |s| s.1);
    (0..32)
        .filter(|k| bits & (1 << k) != 0)
        .map(|k| b.fault_names.get(k).cloned().unwrap_or(format!("fault {k}")))
        .collect()
}

fn cpu(a: &BoardRecord, b: &BoardRecord) -> Option<CpuDiff> {
    let (pa, pb) = (a.profile.as_ref()?, b.profile.as_ref()?);
    let mut d = CpuDiff::default();
    for c in 0..pa.cores.len().min(pb.cores.len()) {
        let share = |p: &sim_host::profile::CoreProfile| {
            let all = p.samples().max(1);
            p.running as f64 / all as f64
        };
        d.running.push((share(&pa.cores[c]), share(&pb.cores[c])));
        let mut names: Vec<String> = Vec::new();
        for p in [&pa.cores[c], &pb.cores[c]] {
            let mut f: Vec<&sim_host::profile::FunctionStat> = p.functions.iter().collect();
            f.sort_by(|x, y| y.self_samples.cmp(&x.self_samples).then(x.name.cmp(&y.name)));
            for s in f.into_iter().take(6) {
                if !names.contains(&s.name) {
                    names.push(s.name.clone());
                }
            }
        }
        for n in names {
            let get = |p: &sim_host::profile::CoreProfile| p.function(&n).map_or(0, |f| f.self_samples);
            d.functions.push((n.clone(), c, get(&pa.cores[c]), get(&pb.cores[c])));
        }
    }
    Some(d)
}

fn compare_boards(a: &BoardRecord, b: &BoardRecord, opts: &CompareOptions) -> BoardDiff {
    let milestones = [
        ("first line", None),
        ("CAN controller online", Some("CAN controller online")),
        ("ROS connected", Some("ROS connected")),
    ]
    .into_iter()
    .map(|(what, text)| Milestone {
        what: what.to_string(),
        a: text.map_or(a.lines.first().map(|l| l.t), |t| first_line(a, t)),
        b: text.map_or(b.lines.first().map(|l| l.t), |t| first_line(b, t)),
    })
    .chain(std::iter::once(Milestone {
        what: "first FirmwareStatus".into(),
        a: a.statuses.first().map(|s| s.0),
        b: b.statuses.first().map(|s| s.0),
    }))
    .collect();
    let (la, lb) = (raised_faults(a), raised_faults(b));
    let (sa, sb) = (status_faults(a), status_faults(b));
    let (ea, eb) = (last_status_faults(a), last_status_faults(b));
    let mut names: Vec<&String> = la.keys().chain(lb.keys()).chain(sa.keys()).chain(sb.keys()).collect();
    names.sort();
    names.dedup();
    let faults = names
        .into_iter()
        .map(|n| FaultDiff {
            name: n.clone(),
            logged: (la.get(n).copied(), lb.get(n).copied()),
            in_status: (sa.get(n).copied(), sb.get(n).copied()),
            at_end: (ea.contains(n), eb.contains(n)),
        })
        .collect();
    let mut escs = Vec::new();
    for (name, va) in &a.escs {
        let vb = b.escs.iter().find(|e| &e.0 == name).map(|e| &e.1[..]).unwrap_or(&[]);
        escs.push((name.clone(), Speeds::of(va), Speeds::of(vb)));
    }
    BoardDiff {
        name: a.name.clone(),
        images: (a.image.clone(), b.image.clone()),
        hashes: (a.image_hash, b.image_hash),
        milestones,
        faults,
        resets: (a.resets.clone(), b.resets.clone()),
        lockups: (a.lockups, b.lockups),
        lines: diff_lines(&a.lines, &b.lines, opts.tolerance),
        escs,
        cpu: cpu(a, b),
    }
}

fn compare_bus(
    name: &str,
    a: &[sim_host::canlog::LogFrame],
    b: &[sim_host::canlog::LogFrame],
    opts: &CompareOptions,
) -> BusDiff {
    let (da, db) = (candiff::decode(a, false), candiff::decode(b, false));
    let dopts = candiff::DiffOptions {
        // Both runs start at power-on: their times line up as they are
        align: candiff::Align::None,
        tolerance_ns: opts.tolerance.as_ns() as i64,
        ..Default::default()
    };
    let mut report = candiff::diff(&da, &db, &dopts);
    // A status whose only change is its uptime, by no more than the tolerance, says the board booted that much
    // earlier or later (shown under boot and connect), not that the message changed
    let tol_ms = opts.tolerance.as_ns() / 1_000_000;
    for st in &mut report.streams {
        st.payload_diffs.retain(|(_, x, y)| !uptime_only(&candiff::payload_change(x, y), tol_ms));
    }
    let mut first_seen: BTreeMap<String, (Option<i64>, Option<i64>)> = BTreeMap::new();
    for e in &da.events {
        let f = first_seen.entry(e.stream.clone()).or_default();
        f.0 = Some(f.0.map_or(e.t_ns, |t| t.min(e.t_ns)));
    }
    for e in &db.events {
        let f = first_seen.entry(e.stream.clone()).or_default();
        f.1 = Some(f.1.map_or(e.t_ns, |t| t.min(e.t_ns)));
    }
    BusDiff { bus: name.to_string(), report, first_seen }
}

/// Whether a payload change (`uptime_ms: 1064 -> 1063, ...`) is only in `uptime_ms`, by at most `tol_ms`.
fn uptime_only(change: &str, tol_ms: u64) -> bool {
    change.split(", ").all(|c| {
        let Some(rest) = c.strip_prefix("uptime_ms: ") else { return false };
        let Some((a, b)) = rest.split_once(" -> ") else { return false };
        matches!((a.parse::<u64>(), b.parse::<u64>()), (Ok(a), Ok(b)) if a.abs_diff(b) <= tol_ms)
    })
}

/// Compares run B of a scenario against run A. Both need their records.
pub fn compare(a: &Outcome, b: &Outcome, labels: (&str, &str), opts: &CompareOptions) -> Comparison {
    let empty = Record::default();
    let (ra, rb) = (a.record.as_ref().unwrap_or(&empty), b.record.as_ref().unwrap_or(&empty));
    let step_of = |o: &Outcome, n: usize| -> (Status, Option<Time>, String) {
        o.steps.iter().find(|s| s.number == n).map_or((Status::NotReached, None, String::new()), |s| {
            (s.status, s.time, s.evidence.iter().find(|e| !e.is_empty()).cloned().unwrap_or_default())
        })
    };
    let mut numbers: Vec<(usize, String)> =
        a.steps.iter().chain(&b.steps).map(|s| (s.number, s.description.clone())).collect();
    numbers.sort();
    numbers.dedup_by_key(|x| x.0);
    let steps = numbers
        .into_iter()
        .map(|(n, d)| StepDiff { number: n, description: d, a: step_of(a, n), b: step_of(b, n) })
        .collect();
    let boards = ra
        .boards
        .iter()
        .filter_map(|ba| rb.boards.iter().find(|bb| bb.name == ba.name).map(|bb| compare_boards(ba, bb, opts)))
        .collect();
    let buses = ra
        .buses
        .iter()
        .map(|(n, fa)| {
            let fb = rb.buses.iter().find(|x| &x.0 == n).map(|x| &x.1[..]).unwrap_or(&[]);
            compare_bus(n, fa, fb, opts)
        })
        .collect();
    Comparison {
        title: a.title(),
        file: a.file.display().to_string(),
        labels: (labels.0.to_string(), labels.1.to_string()),
        errors: (a.error.clone(), b.error.clone()),
        steps,
        boards,
        buses,
        end: (a.end, b.end),
        tolerance: opts.tolerance,
        details: opts.details,
    }
}

fn secs(t: Option<Time>) -> String {
    t.map_or_else(|| "never".into(), |t| format!("{:.3} s", t.as_secs_f64()))
}

fn delta(a: Option<Time>, b: Option<Time>) -> String {
    match (a, b) {
        (Some(a), Some(b)) => format!("{:+.3} s", b.as_secs_f64() - a.as_secs_f64()),
        _ => String::new(),
    }
}

fn differs(a: Option<Time>, b: Option<Time>, tol: Time) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => a.0.abs_diff(b.0) > tol.0,
        (None, None) => false,
        _ => true,
    }
}

fn mark(s: Status) -> &'static str {
    match s {
        Status::Pass => "✓",
        Status::Fail => "✗",
        Status::NotReached => "·",
    }
}

fn md(s: &str) -> String {
    s.replace('|', "\\|").replace('\n', " ")
}

fn short_path(p: &std::path::Path) -> String {
    let comps: Vec<String> = p.components().map(|c| c.as_os_str().to_string_lossy().into_owned()).collect();
    if comps.len() > 3 { format!(".../{}", comps[comps.len() - 3..].join("/")) } else { p.display().to_string() }
}

fn ns_s(t: Option<i64>) -> String {
    t.map_or_else(|| "never".into(), |t| format!("{:.3} s", t as f64 / 1e9))
}

impl Comparison {
    /// Whether the runs differ anywhere this looks.
    pub fn differs(&self) -> bool {
        let tol = self.tolerance;
        self.errors.0 != self.errors.1
            || self.steps.iter().any(StepDiff::status_changed)
            || self.buses.iter().any(|b| !b.report.identical())
            || self.boards.iter().any(|b| {
                b.lines.differs()
                    || b.resets.0.len() != b.resets.1.len()
                    || b.lockups.0 != b.lockups.1
                    || b.milestones.iter().any(|m| differs(m.a, m.b, tol))
                    || b.faults.iter().any(|f| f.logged.0.is_some() != f.logged.1.is_some() || f.at_end.0 != f.at_end.1)
            })
    }

    /// Steps that pass in A and not in B.
    pub fn regressions(&self) -> Vec<&StepDiff> {
        self.steps.iter().filter(|s| s.a.0 == Status::Pass && s.b.0 != Status::Pass).collect()
    }

    /// The report in Markdown, for a pull request comment: only what differs, with the details folded.
    pub fn markdown(&self) -> String {
        let tol = short_time(self.tolerance);
        let n = self.details;
        let mut o = String::new();
        let _ = writeln!(o, "## Firmware A/B: {}\n", md(&self.title));
        let _ = writeln!(o, "`{}` · **A**: {} · **B**: {}\n", self.file, md(&self.labels.0), md(&self.labels.1));
        // Images
        let mut changed: Vec<String> = Vec::new();
        let mut same: Vec<String> = Vec::new();
        for b in &self.boards {
            if b.hashes.0 != b.hashes.1 {
                changed.push(format!(
                    "`{}` ({} `{:016x}` → {} `{:016x}`)",
                    b.name,
                    short_path(&b.images.0),
                    b.hashes.0,
                    short_path(&b.images.1),
                    b.hashes.1
                ));
            } else {
                same.push(format!("`{}`", b.name));
            }
        }
        if !changed.is_empty() {
            let _ = writeln!(o, "**Images that differ:** {}", changed.join(", "));
        }
        if !same.is_empty() {
            let _ = writeln!(o, "**Same image in both:** {}", same.join(", "));
        }
        o.push('\n');
        for (w, e) in [("A", &self.errors.0), ("B", &self.errors.1)] {
            if let Some(e) = e {
                let _ = writeln!(o, "**Run {w} ended with an error:** {}\n", md(e));
            }
        }
        // Checks
        let changed_steps: Vec<&StepDiff> =
            self.steps.iter().filter(|s| s.status_changed() || differs(s.a.1, s.b.1, self.tolerance)).collect();
        let passed = |w: usize| {
            self.steps.iter().filter(|s| if w == 0 { s.a.0 == Status::Pass } else { s.b.0 == Status::Pass }).count()
        };
        let _ = writeln!(o, "### Checks\n");
        let _ = writeln!(
            o,
            "A passed {} of {} steps, B {} of {}.\n",
            passed(0),
            self.steps.len(),
            passed(1),
            self.steps.len()
        );
        if changed_steps.is_empty() {
            let _ = writeln!(o, "Every step has the same result at the same time (within {tol}).\n");
        } else {
            let _ = writeln!(o, "| # | step | A | B |\n|---|---|---|---|");
            for s in changed_steps {
                let cell = |x: &(Status, Option<Time>, String)| {
                    let e =
                        if x.2.is_empty() || x.0 == Status::Pass { String::new() } else { format!("<br>{}", md(&x.2)) };
                    format!("{} {}{e}", mark(x.0), secs(x.1))
                };
                let _ = writeln!(o, "| {} | {} | {} | {} |", s.number, md(&s.description), cell(&s.a), cell(&s.b));
            }
            o.push('\n');
        }
        // Boot and connect
        let rows: Vec<String> = self
            .boards
            .iter()
            .flat_map(|b| b.milestones.iter().filter(|m| differs(m.a, m.b, self.tolerance)).map(move |m| (b, m)))
            .map(|(b, m)| format!("| {} | {} | {} | {} | {} |", b.name, m.what, secs(m.a), secs(m.b), delta(m.a, m.b)))
            .collect();
        let _ = writeln!(o, "### Boot and connect\n");
        if rows.is_empty() {
            let _ = writeln!(o, "Every board booted, came online and connected at the same time (within {tol}).\n");
        } else {
            let _ = writeln!(o, "| board | milestone | A | B | Δ |\n|---|---|---|---|---|\n{}\n", rows.join("\n"));
        }
        // Faults
        let rows: Vec<String> = self
            .boards
            .iter()
            .flat_map(|b| {
                b.faults
                    .iter()
                    .filter(|f| {
                        differs(f.logged.0, f.logged.1, self.tolerance)
                            || differs(f.in_status.0, f.in_status.1, self.tolerance)
                            || f.at_end.0 != f.at_end.1
                    })
                    .map(move |f| (b, f))
            })
            .map(|(b, f)| {
                let cell = |logged: Option<Time>, status: Option<Time>, end: bool| {
                    let mut c = match logged {
                        Some(t) => format!("raised at {}", secs(Some(t))),
                        None => "not raised".into(),
                    };
                    if let Some(t) = status {
                        c += &format!(", in status from {}", secs(Some(t)));
                    }
                    if end {
                        c += ", still raised at the end";
                    }
                    c
                };
                format!(
                    "| {} | `{}` | {} | {} |",
                    b.name,
                    f.name,
                    cell(f.logged.0, f.in_status.0, f.at_end.0),
                    cell(f.logged.1, f.in_status.1, f.at_end.1)
                )
            })
            .collect();
        let _ = writeln!(o, "### Faults raised\n");
        if rows.is_empty() {
            let _ = writeln!(o, "The same faults, raised at the same times.\n");
        } else {
            let _ = writeln!(o, "| board | fault | A | B |\n|---|---|---|---|\n{}\n", rows.join("\n"));
        }
        // Resets
        let rows: Vec<String> = self
            .boards
            .iter()
            .filter(|b| b.resets.0.len() != b.resets.1.len() || b.lockups.0 != b.lockups.1)
            .map(|b| {
                let t = |v: &[Time]| v.iter().map(|t| format!("{:.3}", t.as_secs_f64())).collect::<Vec<_>>().join(", ");
                format!(
                    "| {} | {} ({}) | {} ({}) | {} | {} |",
                    b.name,
                    b.resets.0.len(),
                    t(&b.resets.0),
                    b.resets.1.len(),
                    t(&b.resets.1),
                    b.lockups.0,
                    b.lockups.1
                )
            })
            .collect();
        if !rows.is_empty() {
            let _ = writeln!(
                o,
                "### Resets and lockups\n\n| board | resets A (at s) | resets B (at s) | lockups A | lockups B |\n|---|---|---|---|---|\n{}\n",
                rows.join("\n")
            );
        }
        // Log lines
        let _ = writeln!(o, "### Log lines\n");
        let mut any = false;
        for b in &self.boards {
            let l = &b.lines;
            if !l.differs() {
                continue;
            }
            any = true;
            let _ = writeln!(
                o,
                "<details><summary><b>{}</b>: {} added, {} removed, {} changed, {} retimed (by more than {tol}), {} the same</summary>\n\n```diff",
                b.name,
                l.added.len(),
                l.removed.len(),
                l.changed.len(),
                l.retimed.len(),
                l.same
            );
            for x in l.removed.iter().take(n) {
                let _ = writeln!(o, "- [{}] {}", x.t, x.display);
            }
            more(&mut o, l.removed.len(), n, "-");
            for x in l.added.iter().take(n) {
                let _ = writeln!(o, "+ [{}] {}", x.t, x.display);
            }
            more(&mut o, l.added.len(), n, "+");
            for (x, y) in l.changed.iter().take(n) {
                let _ = writeln!(o, "! [{}] {}\n!   -> [{}] {}", x.t, x.display, y.t, y.display);
            }
            more(&mut o, l.changed.len(), n, "!");
            for (x, y) in l.retimed.iter().take(n) {
                let dt = y.t.as_secs_f64() - x.t.as_secs_f64();
                let _ = writeln!(o, "# [{} -> {}, {dt:+.3} s] {}", x.t, y.t, x.text);
            }
            more(&mut o, l.retimed.len(), n, "#");
            o.push_str("```\n\n</details>\n\n");
        }
        if !any {
            let _ = writeln!(o, "Every board logged the same lines at the same times (within {tol}).\n");
        }
        // CAN
        for bus in self.buses.iter().filter(|b| b.report.frames != (0, 0)) {
            let r = &bus.report;
            let (mi, ex, pd) = r.counts();
            let _ = writeln!(o, "### CAN traffic on `{}`\n", bus.bus);
            let _ = writeln!(o, "Frames: A {}, B {}. ", r.frames.0, r.frames.1);
            let rows: Vec<&candiff::StreamDiff> = r
                .streams
                .iter()
                .filter(|s| s.differs() || differs_ns(bus.first_seen.get(&s.name), self.tolerance))
                .collect();
            if rows.is_empty() {
                let _ = writeln!(o, "Every message stream matches (counts, times within {tol}, payloads).\n");
                continue;
            }
            let _ = writeln!(
                o,
                "{} stream(s) differ: {mi} missing in B, {ex} extra in B, {pd} payload change(s).\n",
                rows.len()
            );
            let _ = writeln!(
                o,
                "| stream | A | B | period A ms | period B ms | first A | first B | missing | extra | payload |"
            );
            let _ = writeln!(o, "|---|---|---|---|---|---|---|---|---|---|");
            for s in &rows {
                let p = |x: &Option<candiff::Period>| x.map_or("".into(), |p| format!("{:.1}", p.mean_ms));
                let f = bus.first_seen.get(&s.name).copied().unwrap_or_default();
                let _ = writeln!(
                    o,
                    "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
                    md(&s.name),
                    s.a_count + s.outside.0,
                    s.b_count + s.outside.1,
                    p(&s.period_a),
                    p(&s.period_b),
                    ns_s(f.0),
                    ns_s(f.1),
                    s.missing.len(),
                    s.extra.len(),
                    s.payload_diffs.len()
                );
            }
            let examples: Vec<String> = rows
                .iter()
                .flat_map(|s| {
                    s.payload_diffs.iter().take(2).map(move |(t, x, y)| {
                        format!("{:.6} {}: {}", *t as f64 / 1e9, s.name, candiff::payload_change(x, y))
                    })
                })
                .take(n)
                .collect();
            if !examples.is_empty() {
                let _ = writeln!(
                    o,
                    "\n<details><summary>Payload changes</summary>\n\n```\n{}\n```\n\n</details>",
                    examples.join("\n")
                );
            }
            o.push('\n');
        }
        // ESCs
        let rows: Vec<String> = self
            .boards
            .iter()
            .flat_map(|b| b.escs.iter().map(move |e| (b, e)))
            .filter(|(_, (_, a, b))| match (a, b) {
                (Some(a), Some(b)) => !Speeds::close(a, b),
                _ => true,
            })
            .map(|(b, (name, sa, sb))| {
                let c = |s: &Option<Speeds>| {
                    s.map_or("-".into(), |s| format!("{:.0} / {:.0} / {:.0} / {:.0}", s.mean, s.min, s.max, s.last))
                };
                format!("| {} | {name} | {} | {} |", b.name, c(sa), c(sb))
            })
            .collect();
        if !rows.is_empty() {
            let _ = writeln!(
                o,
                "### ESC speeds\n\nrpm: mean / min / max / last, sampled every 10 ms.\n\n| board | ESC | A | B |\n|---|---|---|---|\n{}\n",
                rows.join("\n")
            );
        }
        // CPU
        let cpus: Vec<(&BoardDiff, &CpuDiff)> =
            self.boards.iter().filter_map(|b| b.cpu.as_ref().map(|c| (b, c))).collect();
        if !cpus.is_empty() {
            let _ = writeln!(o, "### CPU load\n\n| board | core | running A | running B |\n|---|---|---|---|");
            for (b, c) in &cpus {
                for (k, (x, y)) in c.running.iter().enumerate() {
                    let _ = writeln!(o, "| {} | {k} | {:.1} % | {:.1} % |", b.name, 100.0 * x, 100.0 * y);
                }
            }
            let _ = writeln!(
                o,
                "\n<details><summary>Functions (self samples)</summary>\n\n| board | core | function | A | B |\n|---|---|---|---|---|"
            );
            for (b, c) in &cpus {
                for (f, core, x, y) in &c.functions {
                    if x != y {
                        let short = if f.len() > 80 { format!("{}...", &f[..80]) } else { f.clone() };
                        let _ = writeln!(o, "| {} | {core} | `{}` | {x} | {y} |", b.name, md(&short));
                    }
                }
            }
            o.push_str("\n</details>\n\n");
        }
        o
    }

    /// The comparison as JSON.
    pub fn json(&self) -> serde_json::Value {
        use serde_json::json;
        let t = |t: Option<Time>| t.map(|t| t.as_secs_f64());
        let line = |l: &Line| json!({"t": l.t.as_secs_f64(), "kind": l.kind, "text": l.text});
        json!({
            "title": self.title,
            "file": self.file,
            "a": self.labels.0,
            "b": self.labels.1,
            "differs": self.differs(),
            "errors": [self.errors.0, self.errors.1],
            "end_s": [self.end.0.as_secs_f64(), self.end.1.as_secs_f64()],
            "steps": self.steps.iter().map(|s| json!({
                "number": s.number,
                "description": s.description,
                "a": {"status": s.a.0, "time_s": t(s.a.1), "evidence": s.a.2},
                "b": {"status": s.b.0, "time_s": t(s.b.1), "evidence": s.b.2},
            })).collect::<Vec<_>>(),
            "boards": self.boards.iter().map(|b| json!({
                "name": b.name,
                "images": [b.images.0.display().to_string(), b.images.1.display().to_string()],
                "image_hashes": [format!("{:016x}", b.hashes.0), format!("{:016x}", b.hashes.1)],
                "milestones": b.milestones.iter().map(|m| json!({"what": m.what, "a_s": t(m.a), "b_s": t(m.b)})).collect::<Vec<_>>(),
                "faults": b.faults.iter().map(|f| json!({
                    "name": f.name,
                    "logged_s": [t(f.logged.0), t(f.logged.1)],
                    "in_status_s": [t(f.in_status.0), t(f.in_status.1)],
                    "at_end": [f.at_end.0, f.at_end.1],
                })).collect::<Vec<_>>(),
                "resets_s": [b.resets.0.iter().map(|x| x.as_secs_f64()).collect::<Vec<_>>(), b.resets.1.iter().map(|x| x.as_secs_f64()).collect::<Vec<_>>()],
                "lockups": [b.lockups.0, b.lockups.1],
                "lines": {
                    "same": b.lines.same,
                    "added": b.lines.added.iter().map(line).collect::<Vec<_>>(),
                    "removed": b.lines.removed.iter().map(line).collect::<Vec<_>>(),
                    "changed": b.lines.changed.iter().map(|(x, y)| json!([line(x), line(y)])).collect::<Vec<_>>(),
                    "retimed": b.lines.retimed.iter().map(|(x, y)| json!([line(x), line(y)])).collect::<Vec<_>>(),
                },
                "escs": b.escs.iter().map(|(n, x, y)| {
                    let s = |s: &Option<Speeds>| s.map(|s| json!({"mean": s.mean, "min": s.min, "max": s.max, "last": s.last}));
                    json!({"name": n, "a": s(x), "b": s(y)})
                }).collect::<Vec<_>>(),
                "cpu": b.cpu.as_ref().map(|c| json!({
                    "running": c.running,
                    "functions": c.functions.iter().map(|(f, core, x, y)| json!({"function": f, "core": core, "a": x, "b": y})).collect::<Vec<_>>(),
                })),
            })).collect::<Vec<_>>(),
            "buses": self.buses.iter().map(|bus| json!({
                "bus": bus.bus,
                "frames": [bus.report.frames.0, bus.report.frames.1],
                "identical": bus.report.identical(),
                "streams": bus.report.streams.iter().map(|s| {
                    let f = bus.first_seen.get(&s.name).copied().unwrap_or_default();
                    json!({
                        "name": s.name,
                        "a": s.a_count + s.outside.0,
                        "b": s.b_count + s.outside.1,
                        "period_ms": [s.period_a.map(|p| p.mean_ms), s.period_b.map(|p| p.mean_ms)],
                        "first_s": [f.0.map(|x| x as f64 / 1e9), f.1.map(|x| x as f64 / 1e9)],
                        "missing": s.missing.len(),
                        "extra": s.extra.len(),
                        "payload_changes": s.payload_diffs.iter().map(|(t, x, y)| json!({"t": *t as f64 / 1e9, "change": candiff::payload_change(x, y)})).collect::<Vec<_>>(),
                    })
                }).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
        })
    }
}

fn differs_ns(f: Option<&(Option<i64>, Option<i64>)>, tol: Time) -> bool {
    match f {
        Some((Some(a), Some(b))) => a.abs_diff(*b) > tol.as_ns(),
        Some((None, None)) | None => false,
        _ => true,
    }
}

fn more(o: &mut String, total: usize, shown: usize, sign: &str) {
    if total > shown {
        let _ = writeln!(o, "{sign} ... and {} more", total - shown);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::StepResult;

    fn line(t_ms: u64, kind: &str, text: &str) -> Line {
        Line { t: Time::from_ms(t_ms), kind: kind.into(), text: text.into(), display: format!("{kind} {text}") }
    }

    #[test]
    fn lcs_pairs_the_longest_common_subsequence() {
        let a: Vec<char> = "ABCABBA".chars().collect();
        let b: Vec<char> = "CBABAC".chars().collect();
        let p = lcs(&a, &b, 100);
        assert_eq!(p.len(), 4);
        assert!(p.windows(2).all(|w| w[0].0 < w[1].0 && w[0].1 < w[1].1));
        assert!(p.iter().all(|&(i, j)| a[i] == b[j]));
        assert_eq!(lcs(&a, &a, 0).len(), a.len());
        assert_eq!(lcs::<char>(&[], &b, 10), vec![]);
        // Too different for the limit: only the common ends pair up
        let x: Vec<u32> = (0..100).collect();
        let mut y = x.clone();
        y[50] = 999;
        assert_eq!(lcs(&x, &y, 1).len(), 99, "start and end");
        assert_eq!(lcs(&x, &y, 10).len(), 99);
    }

    #[test]
    fn uptime_jitter_is_not_a_payload_change() {
        assert!(uptime_only("uptime_ms: 1064 -> 1063", 20));
        assert!(!uptime_only("uptime_ms: 1064 -> 1163", 20));
        assert!(!uptime_only("uptime_ms: 11868 -> 11869, faults: 2 -> 0", 20));
        assert!(!uptime_only("port_voltage: 16.2 -> 13.0", 20));
    }

    #[test]
    fn shapes_ignore_numbers() {
        assert_eq!(shape("CAN offline for 10000 ms (tx 128)"), "CAN offline for # ms (tx #)");
        assert_eq!(shape("Fault FAULT_X raised (arg 0x3a981) at a.rs:418"), "Fault FAULT_X raised (arg #) at a.rs:#");
        assert_eq!(shape("esc1 2.5 rpm"), "esc# # rpm");
    }

    #[test]
    fn lines_are_added_removed_changed_and_retimed() {
        let a = vec![
            line(100, "info", "boot"),
            line(200, "info", "ROS connected"),
            line(5600, "info", "Lost connection to ROS"),
            line(5604, "error", "Fault FAULT_WATCHDOG_WARNING raised (arg 0x3a981)"),
            line(8000, "info", "ROS connected"),
            line(14638, "warn", "CAN offline for 10000 ms"),
        ];
        let b = vec![
            line(100, "info", "boot"),
            line(200, "info", "ROS connected"),
            line(5600, "info", "Lost connection to ROS"),
            line(8000, "info", "ROS connected"),
            line(9000, "warn", "CAN offline for 500 ms"),
            line(9500, "info", "new line"),
        ];
        let d = diff_lines(&a, &b, Time::from_ms(20));
        assert_eq!(d.same, 4);
        assert_eq!(
            d.removed.iter().map(|l| l.text.as_str()).collect::<Vec<_>>(),
            ["Fault FAULT_WATCHDOG_WARNING raised (arg 0x3a981)"]
        );
        assert_eq!(d.added.iter().map(|l| l.text.as_str()).collect::<Vec<_>>(), ["new line"]);
        assert_eq!(d.changed.len(), 1);
        assert_eq!((d.changed[0].0.t, d.changed[0].1.t), (Time::from_ms(14638), Time::from_ms(9000)));
        assert!(d.retimed.is_empty());
        // The same text later than the tolerance is retimed; within it, the same
        let c = vec![line(100, "info", "boot"), line(260, "info", "ROS connected")];
        let d = diff_lines(&a[..2], &c, Time::from_ms(20));
        assert_eq!((d.same, d.retimed.len()), (1, 1));
        assert!(!diff_lines(&a, &a, Time::ZERO).differs());
    }

    fn outcome(steps: Vec<StepResult>, record: Record) -> Outcome {
        Outcome {
            name: "synthetic".into(),
            file: "x.yaml".into(),
            document: 0,
            case: None,
            steps,
            error: None,
            skipped: None,
            end: Time::from_secs(10),
            wall: Default::default(),
            images: Vec::new(),
            record: Some(record),
        }
    }

    fn step(n: usize, status: Status, t_ms: u64) -> StepResult {
        StepResult {
            number: n,
            kind: "expect",
            description: format!("step {n}"),
            status,
            time: Some(Time::from_ms(t_ms)),
            evidence: vec![format!("evidence {n}")],
        }
    }

    /// Two hand-made runs that differ in each way the comparison looks at.
    #[test]
    fn a_synthetic_comparison_finds_every_kind_of_difference() {
        use sim_devices::can::CanFrame;
        let board =
            |lines: Vec<Line>, statuses: Vec<(Time, u32)>, resets: Vec<Time>, rpm: Vec<f64>, hash| BoardRecord {
                name: "esc0".into(),
                image: format!("/fw/{hash}/esc_board").into(),
                image_hash: hash,
                lines,
                resets,
                lockups: 0,
                fault_names: ["FAULT_WATCHDOG_RESET", "FAULT_WATCHDOG_WARNING"].map(String::from).to_vec(),
                statuses,
                escs: vec![("esc1".into(), rpm)],
                profile: None,
            };
        // A raw (non-CANmore) frame every 100 ms; B sends it every 50 ms from 1 s, and changes its payload
        let frames = |period: u64, data: u8| -> Vec<sim_host::canlog::LogFrame> {
            (0..20)
                .map(|k| sim_host::canlog::LogFrame {
                    time_ns: (1000 + k * period) * 1_000_000,
                    iface: "can".into(),
                    frame: CanFrame::classic(0x123, false, &[data]),
                })
                .collect()
        };
        let a = Record {
            boards: vec![board(
                vec![
                    line(130, "info", "CAN controller online as client 2"),
                    line(200, "info", "ROS connected"),
                    line(3600, "error", "Fault FAULT_WATCHDOG_WARNING raised (arg 0x1) at safety.rs:418"),
                ],
                vec![(Time::from_secs(1), 0), (Time::from_secs(4), 2)],
                vec![],
                vec![0.0, 800.0, 800.0],
                1,
            )],
            buses: vec![("can".into(), frames(100, 1))],
            end: Time::from_secs(10),
        };
        let b = Record {
            boards: vec![board(
                vec![line(130, "info", "CAN controller online as client 2"), line(700, "info", "ROS connected")],
                vec![(Time::from_secs(1), 0), (Time::from_secs(4), 0)],
                vec![Time::from_secs(5)],
                vec![0.0, 400.0, 400.0],
                2,
            )],
            buses: vec![("can".into(), frames(50, 2))],
            end: Time::from_secs(10),
        };
        let oa = outcome(vec![step(1, Status::Pass, 200), step(2, Status::Pass, 3600)], a);
        let ob = outcome(vec![step(1, Status::Pass, 700), step(2, Status::Fail, 5000)], b);
        let c = compare(&oa, &ob, ("A build", "B build"), &CompareOptions::default());
        assert!(c.differs());
        assert_eq!(c.regressions().iter().map(|s| s.number).collect::<Vec<_>>(), [2]);
        let bd = &c.boards[0];
        assert_eq!(bd.lines.removed.len(), 1, "the fault line");
        assert_eq!(bd.lines.retimed.len(), 1, "ROS connected 500 ms later");
        let m = bd.milestones.iter().find(|m| m.what == "ROS connected").unwrap();
        assert_eq!((m.a, m.b), (Some(Time::from_ms(200)), Some(Time::from_ms(700))));
        let f = bd.faults.iter().find(|f| f.name == "FAULT_WATCHDOG_WARNING").unwrap();
        assert_eq!((f.logged.0.is_some(), f.logged.1.is_some(), f.at_end), (true, false, (true, false)));
        assert_eq!((bd.resets.0.len(), bd.resets.1.len()), (0, 1));
        assert!(matches!(bd.escs[0], (_, Some(x), Some(y)) if x.max == 800.0 && y.max == 400.0));
        let bus = &c.buses[0];
        assert!(!bus.report.identical());
        assert_eq!(bus.report.frames, (20, 20));
        let md = c.markdown();
        for want in [
            "## Firmware A/B: synthetic",
            "Images that differ",
            "| 2 | step 2 | ✓ 3.600 s | ✗ 5.000 s<br>evidence 2 |",
            "| esc0 | ROS connected | 0.200 s | 0.700 s | +0.500 s |",
            "`FAULT_WATCHDOG_WARNING` | raised at 3.600 s, in status from 4.000 s, still raised at the end | not raised |",
            "### Resets and lockups",
            "- [3.600000s] error Fault FAULT_WATCHDOG_WARNING",
            "# [0.200000s -> 0.700000s, +0.500 s] ROS connected",
            "### CAN traffic on `can`",
            "### ESC speeds",
            "| esc0 | esc1 | 533 / 0 / 800 / 800 | 267 / 0 / 400 / 400 |",
        ] {
            assert!(md.contains(want), "no {want:?} in\n{md}");
        }
        let j = c.json();
        assert_eq!(j["differs"], true);
        assert_eq!(j["steps"][1]["b"]["status"], "fail");
        assert_eq!(j["boards"][0]["lines"]["removed"].as_array().unwrap().len(), 1);
        // Identical runs: nothing differs, and the report says so
        let same = compare(&oa, &oa, ("A", "A"), &CompareOptions::default());
        assert!(!same.differs() && same.regressions().is_empty());
        let md = same.markdown();
        assert!(md.contains("Every step has the same result") && md.contains("Every message stream matches"), "{md}");
        // Deterministic
        assert_eq!(c.markdown(), compare(&oa, &ob, ("A build", "B build"), &CompareOptions::default()).markdown());
    }
}
