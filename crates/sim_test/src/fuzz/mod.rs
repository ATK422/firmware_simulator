//! Fuzzing the firmware through the emulator: boot a board once, then drive many seeded, deterministic cases of
//! malformed and adversarial CAN traffic from that snapshot, watching the oracles in [`oracle`] for crashes,
//! hangs, watchdog resets and fault states. New branch-edge coverage ([`cpu_armv6m::edges`]) keeps an input in the
//! corpus, AFL style. Failing cases are minimised and written out as a scenario reproducer (plus a JSON record),
//! so `firmware_simulator scenario reproducer.yaml` replays them.
//!
//! The engine is firmware-agnostic: it asserts nothing about what any board does, only that bus traffic alone
//! should not crash, hang or fault it. What it finds is written up in `docs/faults.md` and pinned by
//! `firmware_finding_*` tests, not judged here.

pub mod cases;
pub mod oracle;
pub mod repro;

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use sim_core::time::Time;

use crate::Bench;
use cases::{FuzzInput, Op, Params};
pub use oracle::{Finding, Kind};

/// A board to fuzz.
#[derive(Clone, Debug)]
pub struct Target {
    /// The image name (`esc_board`), looked up under `$TITAN_RS_RELEASE_DIR`.
    pub image: String,
    /// The board file (`mk2/esc_board`).
    pub board: String,
    /// The CANmore client ID a mock bridge serves it as.
    pub client: u8,
    /// A log line that marks the board as up before the snapshot (`ROS connected`).
    pub boot_log: String,
}

/// The default fuzz targets: the titan_firmware_rs boards on a CAN bus (the Puddles backplane is Ethernet only and
/// left out). Matches the smoke test's board/client assignments.
pub fn default_targets() -> Vec<Target> {
    let t = |image: &str, board: &str, client| Target {
        image: image.into(),
        board: board.into(),
        client,
        boot_log: "ROS connected".into(),
    };
    vec![
        t("power_board", "mk2/power_board", 1),
        t("esc_board", "mk2/esc_board", 2),
        t("camera_cage_bb", "mk2/camera_cage_bb", 4),
        t("actuator_board", "mk2/actuator_board", 5),
        t("led_board", "talos/led_board", 6),
        t("sbc_board", "talos/sbc_board", 7),
        t("ivc_board2", "talos/ivc_board", 8),
        t("smart_battery", "mk2/sbh_mcu", 21),
        t("poac_board", "talos/poac_board", 1),
        t("hbridge_board", "lilboat/hbridge_board", 2),
        t("po_half_board", "lilboat/po_half", 1),
    ]
}

/// A target found by name in [`default_targets`], or by `image`/`board`/`client` when not listed.
pub fn target(name: &str) -> Option<Target> {
    default_targets().into_iter().find(|t| t.image == name)
}

/// How to run a campaign.
#[derive(Clone, Debug)]
pub struct Config {
    pub seed: u64,
    /// Virtual time to drive fuzz traffic for in each case.
    pub window: Time,
    /// Quiet time after the traffic stops, to tell a hang from a slow board.
    pub settle: Time,
    /// Largest number of ops in a generated case.
    pub max_ops: u32,
    /// Time to wait for `boot_log` before snapshotting.
    pub boot_time: Time,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            seed: 0,
            window: Time::from_ms(200),
            settle: Time::from_ms(1200),
            max_ops: 16,
            boot_time: Time::from_secs(2),
        }
    }
}

/// What one case did.
#[derive(Clone, Debug)]
pub struct CaseResult {
    pub findings: Vec<Finding>,
    /// New edges this case reached that no earlier case had (coverage feedback).
    pub new_edges: usize,
}

/// The immutable, thread-safe part of a campaign: the clean-boot baseline, the generator parameters, the config,
/// and the pooled coverage. Shared by reference across workers (a `Bench` is not `Sync`, so each worker keeps its
/// own snapshot of the base board).
pub struct Shared {
    baseline: oracle::Baseline,
    snap_at: Time,
    params: Params,
    cfg: Config,
    seen: Arc<Mutex<cpu_armv6m::EdgeMap>>,
    pub target: Target,
}

impl Shared {
    /// Runs one case on a copy of `base` and judges it. Does not touch the shared coverage.
    /// The virtual time the board was snapshotted at (op offsets are added to it).
    pub fn snap_at(&self) -> Time {
        self.snap_at
    }

    /// The fuzz window.
    pub fn window(&self) -> Time {
        self.cfg.window
    }

    /// The settle time after the window.
    pub fn settle(&self) -> Time {
        self.cfg.settle
    }

    pub fn judge(&self, base: &Bench, input: &FuzzInput) -> (Vec<Finding>, cpu_armv6m::EdgeMap) {
        let mut b = base.snapshot();
        let bridge = b.bridge.clone().expect("a CAN bridge");
        // Schedule every op on the bridge / as a fault, at its absolute time on the slice grid
        let mut faults = Vec::new();
        for op in &input.ops {
            let at = self.snap_at + grid(Time::from_us(op.at_us() as u64));
            match op {
                Op::Raw { id, extended, fd, brs, data, .. } => {
                    let mut f = if *fd {
                        sim_devices::can::CanFrame::fd(*id, *extended, data)
                    } else {
                        sim_devices::can::CanFrame::classic(*id, *extended, data)
                    };
                    f.brs = *brs && *fd;
                    bridge.send_frame_at(at, f);
                }
                Op::Encoded { bytes, .. } => {
                    for f in cases::encoded_frames(self.params.client, self.params.fd, bytes) {
                        bridge.send_frame_at(at, f);
                    }
                }
                Op::Host { msg, .. } => bridge.send_at(at, msg.clone()),
                Op::Fault { fault, .. } => faults.push((at, fault.clone())),
            }
        }
        faults.sort_by_key(|f| f.0);
        for (at, fault) in faults {
            b.inject_at(at, fault);
        }
        // Drive the traffic
        let end = self.snap_at + self.cfg.window;
        let _ = b.run_until(end);
        // Stop the adversarial bus conditions so a healthy board can answer again, then listen for a heartbeat
        for f in [
            crate::Fault::CanStuckDominant(false),
            crate::Fault::CanUnplugged(false),
            crate::Fault::CanFloodStop,
            crate::Fault::CanErrorRate(0.0),
            crate::Fault::KillSwitch(Some(false)),
        ] {
            let _ = b.inject(f);
        }
        let settle_start = b.now();
        let _ = b.run_until(settle_start + self.cfg.settle);

        let edges = b.session.machine.edges().unwrap_or_default();
        // Logs after the snapshot (the clean boot's lines are the baseline)
        let logs: Vec<(Time, Option<String>, String)> = b
            .session
            .logs
            .iter()
            .filter(|(t, _)| *t >= self.snap_at)
            .map(|(t, l)| (*t, l.level.clone(), l.message.clone()))
            .collect();
        let answered_after = {
            let s = bridge.state();
            s.heartbeats.iter().any(|(t, _)| *t > settle_start) || s.received.iter().any(|(t, _)| *t > settle_start)
        };
        let unmodelled = unmodelled_set(&b);
        let out = oracle::Outcome {
            logs: &logs,
            faults: b.faults().unwrap_or(self.baseline.faults),
            unmodelled: &unmodelled,
            resets: b.session.machine.resets,
            lockups: b.session.lockups,
            answered_after,
            locked_up_now: (0..2).any(|c| b.session.machine.cores[c].is_locked_up()),
        };
        (oracle::check(&self.baseline, &out), edges)
    }

    /// New edges this case added to the shared coverage (and folds them in).
    fn fold_edges(&self, edges: &cpu_armv6m::EdgeMap) -> usize {
        let mut seen = self.seen.lock().unwrap();
        let n = edges.new_bits(&seen);
        if n > 0 {
            seen.merge(edges);
        }
        n
    }

    /// Shrinks `input` on `base` while it still produces a finding with `key`: drop ops, then shrink each op's
    /// data to a prefix. A deterministic ddmin-style reduction over the op list.
    pub fn minimize(&self, base: &Bench, input: &FuzzInput, key: &str) -> FuzzInput {
        let hits = |i: &FuzzInput| self.judge(base, i).0.iter().any(|f| f.key == key);
        let mut best = input.clone();
        let mut changed = true;
        while changed {
            changed = false;
            let mut i = 0;
            while i < best.ops.len() {
                let mut trial = best.clone();
                trial.ops.remove(i);
                if hits(&trial) {
                    best = trial;
                    changed = true;
                } else {
                    i += 1;
                }
            }
        }
        for i in 0..best.ops.len() {
            loop {
                let len = op_data_len(&best.ops[i]);
                if len == 0 {
                    break;
                }
                let mut trial = best.clone();
                truncate_op(&mut trial.ops[i], len - 1);
                if hits(&trial) {
                    best = trial;
                } else {
                    break;
                }
            }
        }
        best
    }
}

/// A booted board the cases branch from.
pub struct Campaign {
    base: Bench,
    shared: Shared,
    noise: Vec<Finding>,
}

impl Campaign {
    /// Boots `target`'s image on its board next to a mock bridge, runs until it is up, and snapshots it.
    pub fn boot(target: &Target, cfg: &Config) -> Result<Campaign, String> {
        let elf = crate::titan_rs(&target.image).ok_or_else(|| format!("{} not built", target.image))?;
        let mut base = Bench::builder(&elf).board(&target.board).bridge(target.client).record_bus().boot()?;
        base.session.stop_on_lockup = false;
        base.session.keep_logs = true;
        base.session.machine.enable_edges(&[]);
        base.wait_for_log(&target.boot_log, cfg.boot_time)?;
        // Settle a moment so the baseline fault word and unmodelled set are the steady state
        base.run_for(Time::from_ms(300));
        let fd = base.session.can_bus.as_ref().and_then(|b| b.rates.data).is_some();
        let baseline = oracle::Baseline {
            faults: base.faults().unwrap_or(0),
            unmodelled: unmodelled_set(&base),
            resets: base.session.machine.resets,
            lockups: base.session.lockups,
            was_connected: !base.bridge.as_ref().unwrap().state().heartbeats.is_empty(),
            fault_names: fault_names(&base),
            noise: Default::default(),
        };
        let snap_at = grid(base.now());
        let seen = base.session.machine.edges().unwrap_or_default();
        let params = Params {
            client: target.client,
            fd,
            window_us: cfg.window.as_ps().div_ceil(1_000_000) as u32,
            max_ops: cfg.max_ops,
        };
        let shared = Shared {
            baseline,
            snap_at,
            params,
            cfg: cfg.clone(),
            seen: Arc::new(Mutex::new(seen)),
            target: target.clone(),
        };
        let mut campaign = Campaign { base, shared, noise: Vec::new() };
        // A control case with no traffic: whatever it trips is the clean board's own behaviour, not the input's
        let (control, _) = campaign.run_once(&FuzzInput { ops: Vec::new() });
        campaign.shared.baseline.noise = control.iter().map(|f| f.key.clone()).collect();
        campaign.noise = control;
        Ok(campaign)
    }

    /// What the control case (no fuzz traffic) tripped: the clean board's own behaviour, left out of the findings.
    pub fn noise(&self) -> &[Finding] {
        &self.noise
    }

    pub fn params(&self) -> Params {
        self.shared.params
    }

    pub fn shared(&self) -> &Shared {
        &self.shared
    }

    pub fn target(&self) -> &Target {
        &self.shared.target
    }

    /// Runs one case from the snapshot and judges it (single-threaded API).
    pub fn run_once(&self, input: &FuzzInput) -> (Vec<Finding>, cpu_armv6m::EdgeMap) {
        self.shared.judge(&self.base, input)
    }
}

/// Rounds a time up to the 100 µs slice grid, so a case's op times match the scenario runner's grid when it is
/// replayed as a reproducer.
fn grid(t: Time) -> Time {
    let s = 100_000u64; // 100 µs in ps
    Time(t.0.div_ceil(s) * s)
}

fn unmodelled_set(b: &Bench) -> BTreeSet<String> {
    b.session.machine.sys.unmodelled.summary().into_iter().map(|(n, _, _)| n).collect()
}

fn fault_names(b: &Bench) -> Vec<String> {
    let mut names = b.session.machine.image.as_ref().map(|i| sim_host::dashboard::fault_names(i)).unwrap_or_default();
    for (k, std) in [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
    ]
    .iter()
    .enumerate()
    {
        if names.len() <= k {
            names.push(std.to_string());
        }
    }
    names
}

/// A failing case kept for reporting: the input, its findings, and whether its reproducer replays.
#[derive(Clone, Debug)]
pub struct Hit {
    pub input: FuzzInput,
    pub findings: Vec<Finding>,
}

/// The result of a campaign on one board.
#[derive(Debug, Default)]
pub struct Report {
    pub cases: u64,
    pub corpus: usize,
    pub edges: usize,
    /// One hit per distinct finding key (the first, minimised).
    pub hits: Vec<Hit>,
}

/// Runs `cases` fuzz cases against `campaign` on `jobs` threads, coverage-guided, and returns a report with one
/// minimised hit per distinct finding. `stop` ends it early. Each worker keeps its own snapshot of the base board
/// (a `Bench` is not `Sync`); the corpus, the hits and the coverage are pooled.
pub fn run_campaign(campaign: &Campaign, cases: u64, jobs: usize, stop: &(dyn Fn() -> bool + Sync)) -> Report {
    let jobs = jobs.max(1);
    let shared = &campaign.shared;
    let p = shared.params;
    // One base per worker, snapshotted up front on this thread (snapshot needs the single original)
    let bases: Vec<Bench> = (0..jobs).map(|_| campaign.base.snapshot()).collect();
    let corpus: Mutex<Vec<FuzzInput>> = Mutex::new(Vec::new());
    let hits: Mutex<Vec<Hit>> = Mutex::new(Vec::new());
    let next = AtomicU64::new(0);
    let done = AtomicU64::new(0);

    std::thread::scope(|s| {
        for (w, base) in bases.into_iter().enumerate() {
            let (corpus, hits, next, done) = (&corpus, &hits, &next, &done);
            s.spawn(move || {
                let base = base;
                let mut rng = cases::Rng::new(shared.cfg.seed ^ (w as u64).wrapping_mul(0x1234_5678_9ABC_DEFF));
                loop {
                    let k = next.fetch_add(1, Ordering::Relaxed);
                    if k >= cases || stop() {
                        break;
                    }
                    // Generate fresh, or mutate a corpus entry
                    let input = {
                        let c = corpus.lock().unwrap();
                        if c.is_empty() || rng.chance(1, 3) {
                            FuzzInput::generate(shared.cfg.seed ^ k.wrapping_mul(0x9E37_79B9), p)
                        } else {
                            let mut seed = c[rng.below(c.len() as u32) as usize].clone();
                            cases::mutate(&mut seed, &mut rng, p, &c);
                            seed
                        }
                    };
                    let (findings, edges) = shared.judge(&base, &input);
                    let new = shared.fold_edges(&edges);
                    done.fetch_add(1, Ordering::Relaxed);
                    if new > 0 {
                        let mut c = corpus.lock().unwrap();
                        if c.len() < 4096 {
                            c.push(input.clone());
                        }
                    }
                    for f in &findings {
                        let h = hits.lock().unwrap();
                        if h.iter().any(|x| x.findings.iter().any(|g| g.key == f.key)) {
                            continue;
                        }
                        drop(h);
                        // Minimise against this finding's key (may take a while) without holding the lock
                        let min = shared.minimize(&base, &input, &f.key);
                        let mut h = hits.lock().unwrap();
                        if !h.iter().any(|x| x.findings.iter().any(|g| g.key == f.key)) {
                            h.push(Hit { input: min, findings: vec![f.clone()] });
                        }
                    }
                }
            });
        }
    });

    Report {
        cases: done.load(Ordering::Relaxed),
        corpus: corpus.lock().unwrap().len(),
        edges: shared.seen.lock().unwrap().count(),
        hits: hits.into_inner().unwrap(),
    }
}

fn op_data_len(op: &Op) -> usize {
    match op {
        Op::Raw { data, .. } | Op::Encoded { bytes: data, .. } => data.len(),
        _ => 0,
    }
}

fn truncate_op(op: &mut Op, len: usize) {
    match op {
        Op::Raw { data, .. } | Op::Encoded { bytes: data, .. } => data.truncate(len),
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_campaign() -> Option<Campaign> {
        let t = target("power_board")?;
        if crate::titan_rs(&t.image).is_none() {
            eprintln!("skipping: {} not built", t.image);
            return None;
        }
        Campaign::boot(&t, &Config { window: Time::from_ms(100), settle: Time::from_ms(800), ..Config::default() }).ok()
    }

    #[test]
    fn a_short_campaign_runs_and_is_deterministic() {
        let Some(c) = a_campaign() else { return };
        let r1 = run_campaign(&c, 40, 2, &|| false);
        assert_eq!(r1.cases, 40);
        // A clean boot touches some registers and reaches some edges
        assert!(r1.edges > 0);
        // Determinism: the same seed and inputs give the same hits
        let Some(c2) = a_campaign() else { return };
        let r2 = run_campaign(&c2, 40, 1, &|| false);
        let keys = |r: &Report| {
            let mut k: Vec<String> = r.hits.iter().flat_map(|h| h.findings.iter().map(|f| f.key.clone())).collect();
            k.sort();
            k
        };
        assert_eq!(keys(&r1), keys(&r2), "same findings on 1 and 2 threads");
    }

    #[test]
    fn run_once_is_deterministic_and_clean_input_is_clean() {
        let Some(c) = a_campaign() else { return };
        // An empty input does nothing adversarial: no finding
        let empty = FuzzInput { ops: vec![] };
        assert!(c.run_once(&empty).0.is_empty(), "a board left alone is fine");
        let input = FuzzInput::generate(123, c.params());
        let (f1, e1) = c.run_once(&input);
        let (f2, e2) = c.run_once(&input);
        assert_eq!(e1.count(), e2.count());
        assert_eq!(f1.iter().map(|f| &f.key).collect::<Vec<_>>(), f2.iter().map(|f| &f.key).collect::<Vec<_>>());
    }
}
