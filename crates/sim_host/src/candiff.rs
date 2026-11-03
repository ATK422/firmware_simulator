//! Comparing two CAN logs (a hardware recording and an emulator run, or two emulator runs) by what the frames mean.
//!
//! Each log is first decoded into *events*: CANmore messages are re-assembled per client and direction, and those
//! carrying `titan_link` are decoded into the protocol's messages (`DeviceToHost::FirmwareStatus`,
//! `HostToDevice::Ping`, ...); utility frames are taken one by one (the heartbeat is decoded, without its rolling
//! counter, whose phase depends on when the board started). Each event belongs to a *stream*, named by interface,
//! client, direction and message kind, e.g. `internal client 1 board->host FirmwareStatus`. Frames that are not
//! CANmore (or every frame, with `raw`) form one stream per identifier.
//!
//! The two logs are then aligned in time (see [`Align`]), and each stream is compared on its own: its events are
//! paired in time order within a tolerance, which shows events missing from the second log, extra ones in it,
//! paired events whose payloads differ, the offset between the streams and the timing of each (period, jitter).
//! Only the time both logs cover is compared, so a hardware log started mid-run can be held against an emulator
//! run from power-on.

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;

use canmore::id::{Direction, Fields, FrameType};
use canmore::{Heartbeat, MsgDecoder};

use crate::canlog::LogFrame;

/// One decoded message or frame of a log.
#[derive(Clone, PartialEq, Debug)]
pub struct Event {
    /// Time of the (last) frame, ns.
    pub t_ns: i64,
    pub stream: String,
    /// What is compared between the logs.
    pub payload: String,
    /// What is shown for it (the payload, plus fields left out of the comparison).
    pub detail: String,
}

/// CANmore re-assembly errors per sender: how many, and the first one.
pub type DecodeErrors = BTreeMap<String, (usize, String)>;

/// A log's events and the problems decoding it.
#[derive(Clone, Debug, Default)]
pub struct Decoded {
    pub events: Vec<Event>,
    /// CANmore re-assembly errors per sender (`internal client 1 board->host`): count and the first error.
    pub errors: DecodeErrors,
    pub frames: usize,
}

fn dir_name(d: Direction) -> &'static str {
    match d {
        Direction::ClientToAgent => "board->host",
        Direction::AgentToClient => "host->board",
    }
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02X}")).collect()
}

fn mode_name(mode: u8) -> String {
    match mode {
        0b000 => "none".into(),
        0b001 => "normal".into(),
        0b010 => "bootloader".into(),
        0b011 => "linux".into(),
        0b101 => "boot_delay".into(),
        m => format!("{m:#05b}"),
    }
}

fn channel_name(ch: u8) -> String {
    match ch {
        canmore::CHAN_HEARTBEAT => "heartbeat".into(),
        canmore::CHAN_CONTROL_INTERFACE => "util ch14 (control interface)".into(),
        canmore::CHAN_REMOTE_TTY => "util ch13 (remote tty)".into(),
        canmore::CHAN_CAMERA_FEED => "util ch12 (camera feed)".into(),
        canmore::CHAN_THRUSTER_CMDS => "util ch0 (thruster commands)".into(),
        c => format!("util ch{c}"),
    }
}

/// The variant path of a `Debug`-printed enum value: `Esc(Telemetry { .. })` is `Esc::Telemetry`, `Ping { .. }`
/// is `Ping`.
pub fn variant_path(debug: &str) -> String {
    let mut parts = Vec::new();
    let mut rest = debug;
    loop {
        let end = rest.find(|c: char| !(c.is_alphanumeric() || c == '_')).unwrap_or(rest.len());
        if end == 0 {
            break;
        }
        // A tuple variant wrapping a struct of the same name (`FirmwareStatus(FirmwareStatus { .. })`) is one level
        if parts.last() != Some(&&rest[..end]) {
            parts.push(&rest[..end]);
        }
        match rest[end..].strip_prefix('(') {
            Some(r) if r.starts_with(|c: char| c.is_ascii_uppercase()) => rest = r,
            _ => break,
        }
    }
    parts.join("::")
}

/// Decodes a log into events. With `raw`, frames are not interpreted as CANmore.
pub fn decode(log: &[LogFrame], raw: bool) -> Decoded {
    let mut out = Decoded { frames: log.len(), ..Default::default() };
    let mut decoders: HashMap<(String, u8, bool, bool), MsgDecoder> = HashMap::new();
    for lf in log {
        let f = &lf.frame;
        let t_ns = lf.time_ns as i64;
        let fields = Fields::decode(f.id, f.extended);
        if raw || f.rtr {
            let id = if f.extended { format!("{:08X}", f.id) } else { format!("{:03X}", f.id) };
            let kind = if f.rtr {
                "remote"
            } else if f.fd {
                "fd"
            } else {
                "classic"
            };
            let payload = format!("{kind} {}", hex(&f.data));
            out.events.push(Event { t_ns, stream: format!("{} id {id}", lf.iface), detail: payload.clone(), payload });
            continue;
        }
        let sender = format!("{} client {} {}", lf.iface, fields.client_id, dir_name(fields.direction));
        if fields.frame_type == FrameType::Util {
            if f.extended {
                // CANmore utility frames are always standard; anything else is someone else's protocol
                let payload = hex(&f.data);
                out.events.push(Event {
                    t_ns,
                    stream: format!("{} id {:08X}", lf.iface, f.id),
                    detail: payload.clone(),
                    payload,
                });
                continue;
            }
            let (payload, detail) = match (fields.noc, f.data.as_slice()) {
                (canmore::CHAN_HEARTBEAT, [b]) => {
                    let h = Heartbeat::from_byte(*b);
                    let p = format!(
                        "mode={} error={} term_valid={} term_enabled={}",
                        mode_name(h.mode),
                        h.error,
                        h.term_valid,
                        h.term_enabled
                    );
                    let d = format!("{p} (cnt={})", h.cnt);
                    (p, d)
                }
                (_, data) => (hex(data), hex(data)),
            };
            out.events.push(Event { t_ns, stream: format!("{sender} {}", channel_name(fields.noc)), payload, detail });
            continue;
        }
        let key = (lf.iface.clone(), fields.client_id, fields.direction == Direction::AgentToClient, f.fd);
        let dec = decoders.entry(key).or_insert_with(|| MsgDecoder::new(f.fd));
        match dec.decode_frame(f.id, f.extended, &f.data) {
            Ok(None) => {}
            Ok(Some(m)) => {
                let (kind, payload) = match m.subtype {
                    canmore::MSG_SUBTYPE_TITAN_LINK => {
                        let text = match fields.direction {
                            Direction::ClientToAgent => {
                                titan_link::decode::<titan_link::DeviceToHost>(m.data).map(|v| format!("{v:?}"))
                            }
                            Direction::AgentToClient => {
                                titan_link::decode::<titan_link::HostToDevice>(m.data).map(|v| format!("{v:?}"))
                            }
                        };
                        match text {
                            Ok(t) => (variant_path(&t), t),
                            Err(_) => ("titan_link (undecodable)".to_string(), hex(m.data)),
                        }
                    }
                    canmore::MSG_SUBTYPE_XRCE_DDS => ("xrce-dds".to_string(), hex(m.data)),
                    s => (format!("msg subtype {s}"), hex(m.data)),
                };
                out.events.push(Event { t_ns, stream: format!("{sender} {kind}"), detail: payload.clone(), payload });
            }
            Err(e) => {
                let entry = out.errors.entry(sender).or_insert((0, format!("{e:?} at {}", seconds(t_ns))));
                entry.0 += 1;
            }
        }
    }
    out
}

/// How the second log's times are lined up with the first's.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Align {
    /// The first frame of each log is time 0 (both started at power-on, or at the same moment).
    Start,
    /// Timestamps are taken as they are.
    None,
    /// The second log is this many ns later than the first.
    Offset(i64),
    /// Searched for: the offset that pairs the most events of the same streams, preferring pairs with equal
    /// payloads. A heuristic, for logs started at unrelated times; give the offset when it is known.
    Auto,
}

#[derive(Clone, Debug)]
pub struct DiffOptions {
    pub align: Align,
    /// How far apart (after alignment) two events may be and still pair up, ns. Never more than half a stream's
    /// period, so neighbouring events of a periodic stream are not confused.
    pub tolerance_ns: i64,
    /// Line each stream up on its own (after the global alignment) before pairing, and report its offset. Boards
    /// that booted at different moments then compare by content, with the difference shown as the stream offset.
    pub stream_align: bool,
    /// Also count events outside the time both logs cover as missing or extra.
    pub full: bool,
    /// Streams (by substring of their name) whose payloads are not compared, only their presence and timing.
    pub ignore_payload: Vec<String>,
    /// Compare only streams whose names contain one of these (all if empty).
    pub only: Vec<String>,
}

impl Default for DiffOptions {
    fn default() -> Self {
        DiffOptions {
            align: Align::Start,
            tolerance_ns: 20_000_000,
            stream_align: true,
            full: false,
            ignore_payload: Vec::new(),
            only: Vec::new(),
        }
    }
}

/// Interval statistics of a stream in one log, ms.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Period {
    pub intervals: usize,
    pub mean_ms: f64,
    /// Standard deviation of the intervals (jitter).
    pub std_ms: f64,
    pub min_ms: f64,
    pub max_ms: f64,
}

impl Period {
    fn of(times: &[i64]) -> Option<Period> {
        if times.len() < 2 {
            return None;
        }
        let d: Vec<f64> = times.windows(2).map(|w| (w[1] - w[0]) as f64 / 1e6).collect();
        let mean = d.iter().sum::<f64>() / d.len() as f64;
        let var = d.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / d.len() as f64;
        Some(Period {
            intervals: d.len(),
            mean_ms: mean,
            std_ms: var.sqrt(),
            min_ms: d.iter().copied().fold(f64::INFINITY, f64::min),
            max_ms: d.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        })
    }

    fn median_ns(times: &[i64]) -> Option<i64> {
        let mut d: Vec<i64> = times.windows(2).map(|w| w[1] - w[0]).collect();
        if d.is_empty() {
            return None;
        }
        d.sort_unstable();
        Some(d[d.len() / 2])
    }
}

/// How one stream compares.
#[derive(Clone, Debug, Default)]
pub struct StreamDiff {
    pub name: String,
    /// Events of each log in the compared window.
    pub a_count: usize,
    pub b_count: usize,
    pub matched: usize,
    /// Events of A with no partner in B (time in A, detail).
    pub missing: Vec<(i64, String)>,
    /// Events of B with no partner in A (time in A's timeline, detail).
    pub extra: Vec<(i64, String)>,
    /// Paired events whose payloads differ: time in A, A's detail, B's detail.
    pub payload_diffs: Vec<(i64, String, String)>,
    pub payload_compared: bool,
    /// The stream's own offset after the global alignment (B later than A), ns.
    pub offset_ns: i64,
    /// Of paired events, after the global alignment: the mean and largest time difference (B - A), ns.
    pub mean_skew_ns: f64,
    pub max_abs_skew_ns: i64,
    pub period_a: Option<Period>,
    pub period_b: Option<Period>,
    /// Events left out because they fall outside the window both logs cover.
    pub outside: (usize, usize),
}

impl StreamDiff {
    pub fn differs(&self) -> bool {
        !self.missing.is_empty() || !self.extra.is_empty() || !self.payload_diffs.is_empty()
    }
}

/// The comparison of two logs.
#[derive(Clone, Debug)]
pub struct Report {
    /// B's timestamps minus A's for the same moment, ns.
    pub offset_ns: i64,
    pub align: Align,
    /// The compared window in A's timeline (ns since A's first frame).
    pub window: (i64, i64),
    pub streams: Vec<StreamDiff>,
    pub frames: (usize, usize),
    pub errors: (DecodeErrors, DecodeErrors),
    /// Times in the report are A's timestamps minus this: 0 for a log of times since power-on, A's first frame
    /// for a log of Unix times.
    pub time_origin: i64,
}

fn seconds(ns: i64) -> String {
    let sign = if ns < 0 { "-" } else { "" };
    let a = ns.unsigned_abs();
    format!("{sign}{}.{:06}", a / 1_000_000_000, (a % 1_000_000_000) / 1_000)
}

fn by_stream(events: &[Event]) -> BTreeMap<&str, Vec<&Event>> {
    let mut m: BTreeMap<&str, Vec<&Event>> = BTreeMap::new();
    for e in events {
        m.entry(e.stream.as_str()).or_default().push(e);
    }
    for v in m.values_mut() {
        v.sort_by_key(|e| e.t_ns);
    }
    m
}

/// Pairs `a` and `b` (sorted times, `b` already shifted onto `a`'s timeline) in order within `tol`. Returns the
/// pairs and the unpaired indices of each.
fn pair(a: &[i64], b: &[i64], tol: i64) -> (Vec<(usize, usize)>, Vec<usize>, Vec<usize>) {
    let (mut i, mut j) = (0, 0);
    let (mut pairs, mut only_a, mut only_b) = (Vec::new(), Vec::new(), Vec::new());
    while i < a.len() && j < b.len() {
        let d = b[j] - a[i];
        if d.abs() <= tol {
            pairs.push((i, j));
            i += 1;
            j += 1;
        } else if d < 0 {
            only_b.push(j);
            j += 1;
        } else {
            only_a.push(i);
            i += 1;
        }
    }
    only_a.extend(i..a.len());
    only_b.extend(j..b.len());
    (pairs, only_a, only_b)
}

/// The pairing tolerance of a stream: the global one, but under half its period.
fn stream_tolerance(a: &[i64], b: &[i64], tol: i64) -> i64 {
    let p = [Period::median_ns(a), Period::median_ns(b)].into_iter().flatten().min();
    match p {
        Some(p) if p > 0 => tol.min((p - 1) / 2).max(0),
        _ => tol,
    }
}

/// Searches for the offset of B against A that pairs the most events (see [`Align::Auto`]).
fn auto_offset(a: &BTreeMap<&str, Vec<&Event>>, b: &BTreeMap<&str, Vec<&Event>>, tol: i64) -> i64 {
    // Candidate offsets from pairs of events of the same stream, sampled evenly over each log
    let bin = tol.max(1_000);
    let mut votes: HashMap<i64, (u64, i128, u64)> = HashMap::new();
    const SAMPLES: usize = 48;
    let sample = |v: &[&Event]| -> Vec<(i64, u64)> {
        let step = v.len().div_ceil(SAMPLES).max(1);
        v.iter().step_by(step).map(|e| (e.t_ns, hash(&e.payload))).collect()
    };
    for (name, ea) in a {
        let Some(eb) = b.get(name) else { continue };
        let (sa, sb) = (sample(ea), sample(eb));
        // Rare streams say more about the alignment than frequent ones, whose pairs repeat every period
        let weight = 1 + 64 / (ea.len().min(eb.len()) as u64).max(1);
        for (ta, pa) in &sa {
            for (tb, pb) in &sb {
                let d = tb - ta;
                let w = if pa == pb { 4 * weight } else { weight };
                let v = votes.entry(d.div_euclid(bin)).or_default();
                v.0 += w;
                v.1 += d as i128 * w as i128;
                v.2 += w;
            }
        }
    }
    let mut cands: Vec<(u64, i64)> = votes.values().map(|v| (v.0, (v.1 / v.2.max(1) as i128) as i64)).collect();
    cands.sort_by(|x, y| y.0.cmp(&x.0).then(x.1.abs().cmp(&y.1.abs())));
    cands.truncate(24);
    // Score each candidate by the events it pairs over every stream
    let mut best = (0usize, 0i64, 0usize);
    for &(_, off) in &cands {
        let mut score = 0;
        let mut same = 0;
        for (name, ea) in a {
            let Some(eb) = b.get(name) else { continue };
            let ta: Vec<i64> = ea.iter().map(|e| e.t_ns).collect();
            let tb: Vec<i64> = eb.iter().map(|e| e.t_ns - off).collect();
            let (pairs, _, _) = pair(&ta, &tb, stream_tolerance(&ta, &tb, tol));
            score += pairs.len();
            same += pairs.iter().filter(|(i, j)| ea[*i].payload == eb[*j].payload).count();
        }
        if (score + same, same) > (best.0, best.2) {
            best = (score + same, off, same);
        }
    }
    best.1
}

fn hash(s: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    s.hash(&mut h);
    h.finish()
}

/// The stream's own offset: the median difference to the nearest event of the other log, within half a period.
fn stream_offset(a: &[i64], b: &[i64], tol: i64) -> i64 {
    let window = match [Period::median_ns(a), Period::median_ns(b)].into_iter().flatten().min() {
        Some(p) => p / 2,
        None => tol,
    };
    let mut d = Vec::new();
    let mut j = 0;
    for &t in a {
        while j + 1 < b.len() && (b[j + 1] - t).abs() <= (b[j] - t).abs() {
            j += 1;
        }
        if let Some(&tb) = b.get(j)
            && (tb - t).abs() <= window
        {
            d.push(tb - t);
        }
    }
    if d.is_empty() {
        return 0;
    }
    d.sort_unstable();
    d[d.len() / 2]
}

/// Compares log B against log A.
pub fn diff(a: &Decoded, b: &Decoded, opts: &DiffOptions) -> Report {
    let keep = |e: &&Event| opts.only.is_empty() || opts.only.iter().any(|s| e.stream.contains(s.as_str()));
    let ea: Vec<Event> = a.events.iter().filter(keep).cloned().collect();
    let eb: Vec<Event> = b.events.iter().filter(keep).cloned().collect();
    let sa = by_stream(&ea);
    let sb = by_stream(&eb);
    let first = |e: &[Event]| e.iter().map(|e| e.t_ns).min();
    let last = |e: &[Event]| e.iter().map(|e| e.t_ns).max();
    let origin_a = first(&ea).unwrap_or(0);
    // Report times: an emulator log's own (virtual) times, or seconds since the first frame of a Unix-time log
    let shown = if origin_a < crate::canlog::RELATIVE_LIMIT_NS as i64 { 0 } else { origin_a };
    let offset = match opts.align {
        Align::Start => first(&eb).unwrap_or(0) - origin_a,
        Align::None => 0,
        Align::Offset(o) => o,
        Align::Auto => auto_offset(&sa, &sb, opts.tolerance_ns),
    };
    // The window both logs cover, in A's timeline
    let window = match (first(&ea), last(&ea), first(&eb), last(&eb)) {
        (Some(a0), Some(a1), Some(b0), Some(b1)) => (a0.max(b0 - offset), a1.min(b1 - offset)),
        _ => (0, -1),
    };
    let mut names: Vec<&str> = sa.keys().chain(sb.keys()).copied().collect();
    names.sort_unstable();
    names.dedup();
    let empty = Vec::new();
    let mut streams = Vec::new();
    for name in names {
        let va = sa.get(name).unwrap_or(&empty);
        let vb = sb.get(name).unwrap_or(&empty);
        let ta_all: Vec<i64> = va.iter().map(|e| e.t_ns).collect();
        let tb_all: Vec<i64> = vb.iter().map(|e| e.t_ns - offset).collect();
        let tol = stream_tolerance(&ta_all, &tb_all, opts.tolerance_ns);
        let own = if opts.stream_align { stream_offset(&ta_all, &tb_all, opts.tolerance_ns) } else { 0 };
        let tb_own: Vec<i64> = tb_all.iter().map(|t| t - own).collect();
        let (pairs, only_a, only_b) = pair(&ta_all, &tb_own, tol);
        // Unpaired events near the edges of the window may have their partner just outside it
        let inside = |t: i64| opts.full || (t >= window.0 + tol && t <= window.1 - tol);
        let in_window = |t: i64| opts.full || (t >= window.0 && t <= window.1);
        let compare_payload = !opts.ignore_payload.iter().any(|s| name.contains(s.as_str()));
        let mut d = StreamDiff {
            name: name.to_string(),
            a_count: ta_all.iter().filter(|t| in_window(**t)).count(),
            b_count: tb_own.iter().filter(|t| in_window(**t)).count(),
            matched: pairs.len(),
            payload_compared: compare_payload,
            offset_ns: own,
            period_a: Period::of(&ta_all),
            period_b: Period::of(&vb.iter().map(|e| e.t_ns).collect::<Vec<_>>()),
            ..Default::default()
        };
        for i in only_a {
            if inside(ta_all[i]) {
                d.missing.push((ta_all[i] - shown, va[i].detail.clone()));
            } else {
                d.outside.0 += 1;
            }
        }
        for j in only_b {
            if inside(tb_own[j]) {
                d.extra.push((tb_own[j] - shown, vb[j].detail.clone()));
            } else {
                d.outside.1 += 1;
            }
        }
        let mut skew_sum = 0f64;
        for &(i, j) in &pairs {
            let s = tb_all[j] - ta_all[i];
            skew_sum += s as f64;
            d.max_abs_skew_ns = d.max_abs_skew_ns.max(s.abs());
            if compare_payload && va[i].payload != vb[j].payload {
                d.payload_diffs.push((ta_all[i] - shown, va[i].detail.clone(), vb[j].detail.clone()));
            }
        }
        if !pairs.is_empty() {
            d.mean_skew_ns = skew_sum / pairs.len() as f64;
        }
        streams.push(d);
    }
    Report {
        offset_ns: offset,
        align: opts.align,
        window: (window.0 - shown, window.1 - shown),
        streams,
        frames: (a.frames, b.frames),
        errors: (a.errors.clone(), b.errors.clone()),
        time_origin: shown,
    }
}

/// Where two payloads differ: `field: old -> new` for each differing token when both have the same shape,
/// otherwise the differing middle part with some context.
pub fn payload_change(a: &str, b: &str) -> String {
    fn tokens(s: &str) -> Vec<&str> {
        let mut out = Vec::new();
        let mut start = None;
        for (i, c) in s.char_indices() {
            let word = c.is_alphanumeric() || matches!(c, '_' | '.' | '-' | '"');
            match (word, start) {
                (true, None) => start = Some(i),
                (false, Some(st)) => {
                    out.push(&s[st..i]);
                    start = None;
                }
                _ => {}
            }
            if !word && !c.is_whitespace() {
                out.push(&s[i..i + c.len_utf8()]);
            }
        }
        if let Some(st) = start {
            out.push(&s[st..]);
        }
        out
    }
    let (ta, tb) = (tokens(a), tokens(b));
    if ta.len() == tb.len() {
        let mut changes = Vec::new();
        for k in 0..ta.len() {
            if ta[k] != tb[k] {
                // Name the change by the closest `field:` before it
                let field = (0..k).rev().find(|&m| ta.get(m + 1) == Some(&":") && m + 1 < k).map(|m| ta[m]);
                changes.push(match field {
                    Some(f) => format!("{f}: {} -> {}", ta[k], tb[k]),
                    None => format!("{} -> {}", ta[k], tb[k]),
                });
            }
        }
        if !changes.is_empty() {
            return changes.join(", ");
        }
    }
    let pre = a.chars().zip(b.chars()).take_while(|(x, y)| x == y).count();
    let (ra, rb): (Vec<char>, Vec<char>) = (a.chars().skip(pre).collect(), b.chars().skip(pre).collect());
    let suf = ra.iter().rev().zip(rb.iter().rev()).take_while(|(x, y)| x == y).count();
    let mid = |r: &[char]| r[..r.len() - suf.min(r.len())].iter().collect::<String>();
    let ctx: String = a.chars().take(pre).collect::<Vec<_>>().iter().rev().take(24).rev().collect();
    format!("...{ctx}[{} -> {}]", mid(&ra), mid(&rb))
}

fn truncate(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { s.chars().take(n).collect::<String>() + "..." }
}

impl Report {
    /// Whether the logs agree: no missing or extra events and no payload differences.
    pub fn identical(&self) -> bool {
        !self.streams.iter().any(StreamDiff::differs)
    }

    /// The largest time difference of paired events (after the global alignment), ns.
    pub fn max_abs_skew_ns(&self) -> i64 {
        self.streams.iter().map(|s| s.max_abs_skew_ns).max().unwrap_or(0)
    }

    /// Number of missing, extra and payload-differing events over all streams.
    pub fn counts(&self) -> (usize, usize, usize) {
        self.streams
            .iter()
            .fold((0, 0, 0), |c, s| (c.0 + s.missing.len(), c.1 + s.extra.len(), c.2 + s.payload_diffs.len()))
    }

    /// The report as text: a table of streams, then up to `details` examples of each difference.
    pub fn render(&self, details: usize) -> String {
        let mut o = String::new();
        let align = match self.align {
            Align::Start => "first frames",
            Align::None => "timestamps as logged",
            Align::Offset(_) => "given offset",
            Align::Auto => "searched",
        };
        let _ = writeln!(
            o,
            "frames: A {}, B {}; aligned on {align}: B = A {:+.6} s; compared {} s to {} s of A",
            self.frames.0,
            self.frames.1,
            self.offset_ns as f64 / 1e9,
            seconds(self.window.0),
            seconds(self.window.1)
        );
        for (which, errs) in [("A", &self.errors.0), ("B", &self.errors.1)] {
            for (sender, (n, first)) in errs {
                let _ = writeln!(o, "{which}: {n} CANmore decode error(s) from {sender}, first {first}");
            }
        }
        let w = self.streams.iter().map(|s| s.name.len()).max().unwrap_or(6).max(6);
        let period = |p: &Option<Period>| match p {
            Some(p) => format!("{:.2} ± {:.3}", p.mean_ms, p.std_ms),
            None => "-".into(),
        };
        let _ = writeln!(
            o,
            "\n{:w$}  {:>6} {:>6} {:>6} {:>6} {:>6} {:>7}  {:>18}  {:>18}  {:>10}  {:>10}",
            "stream",
            "A",
            "B",
            "paired",
            "miss",
            "extra",
            "payload",
            "period A ms",
            "period B ms",
            "offset ms",
            "max skew"
        );
        for s in &self.streams {
            let payload = if s.payload_compared { s.payload_diffs.len().to_string() } else { "-".into() };
            let _ = writeln!(
                o,
                "{:w$}  {:>6} {:>6} {:>6} {:>6} {:>6} {:>7}  {:>18}  {:>18}  {:>+10.3}  {:>10.3}",
                s.name,
                s.a_count,
                s.b_count,
                s.matched,
                s.missing.len(),
                s.extra.len(),
                payload,
                period(&s.period_a),
                period(&s.period_b),
                s.offset_ns as f64 / 1e6,
                s.max_abs_skew_ns as f64 / 1e6,
            );
        }
        let mut any = false;
        for s in self.streams.iter().filter(|s| s.differs()) {
            if !any {
                let _ = writeln!(
                    o,
                    "\ndifferences (times in A's timeline{}):",
                    if self.time_origin == 0 { "" } else { ", s since its first frame" }
                );
                any = true;
            }
            let _ = writeln!(o, "  {}:", s.name);
            for (label, list) in [("missing in B", &s.missing), ("extra in B", &s.extra)] {
                if list.is_empty() {
                    continue;
                }
                let _ = writeln!(o, "    {} {label}", list.len());
                for (t, d) in list.iter().take(details) {
                    let _ = writeln!(o, "      {}  {}", seconds(*t), truncate(d, 160));
                }
            }
            if !s.payload_diffs.is_empty() {
                let _ = writeln!(o, "    {} with different payloads", s.payload_diffs.len());
                for (t, a, b) in s.payload_diffs.iter().take(details) {
                    let _ = writeln!(o, "      {}  {}", seconds(*t), truncate(&payload_change(a, b), 200));
                }
            }
        }
        let (m, e, p) = self.counts();
        let outside: usize = self.streams.iter().map(|s| s.outside.0 + s.outside.1).sum();
        let note = if outside > 0 {
            format!(" ({outside} event(s) outside the common window not compared)")
        } else {
            String::new()
        };
        if self.identical() {
            let _ = writeln!(o, "\nidentical: every event paired with equal payloads{note}");
        } else {
            let _ = writeln!(o, "\nDIFFERENT: {m} missing, {e} extra, {p} payload difference(s){note}");
        }
        o
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::canlog::{LogFrame, parse};
    use canmore::MsgEncoder;
    use sim_devices::can::CanFrame;
    use titan_link::{DeviceToHost, HostToDevice};

    /// The frames of a titan_link message as a board (or the host) sends them.
    fn msg_frames<T: serde::Serialize>(client: u8, dir: Direction, fd: bool, msg: &T) -> Vec<CanFrame> {
        let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
        let enc = titan_link::encode(msg, &mut buf).unwrap();
        MsgEncoder::new(client, dir, fd, canmore::MSG_SUBTYPE_TITAN_LINK, enc)
            .map(|f| {
                if fd {
                    CanFrame::fd(f.id, f.extended, f.data())
                } else {
                    CanFrame::classic(f.id, f.extended, f.data())
                }
            })
            .collect()
    }

    fn status(uptime_ms: u32, faults: u32) -> DeviceToHost {
        DeviceToHost::FirmwareStatus(titan_link::FirmwareStatus {
            board_name: "mk2/power_board".try_into().unwrap(),
            client_id: 1,
            uptime_ms,
            faults,
            ..Default::default()
        })
    }

    /// A board (client 1) and its bridge for `secs`: pings every 100 ms, link heartbeats every 100 ms (5 ms after
    /// each ping), CANmore heartbeats every 500 ms, status every second.
    fn session(secs: u64, start_ns: u64) -> Vec<LogFrame> {
        let mut log = Vec::new();
        let mut push = |t_ms: u64, frames: Vec<CanFrame>| {
            for (k, frame) in frames.into_iter().enumerate() {
                log.push(LogFrame {
                    time_ns: start_ns + t_ms * 1_000_000 + k as u64 * 50_000,
                    iface: "can0".into(),
                    frame,
                });
            }
        };
        let ping = HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION };
        for k in 0..secs * 10 {
            push(k * 100, msg_frames(1, Direction::AgentToClient, true, &ping));
            push(k * 100 + 5, msg_frames(1, Direction::ClientToAgent, true, &DeviceToHost::Heartbeat));
            if k % 5 == 0 {
                let hb = Heartbeat { cnt: (k / 5 % 4) as u8, mode: 1, ..Default::default() }.frame(1);
                push(k * 100 + 7, vec![CanFrame::classic(hb.id, hb.extended, hb.data())]);
            }
            if k % 10 == 0 {
                push(k * 100 + 9, msg_frames(1, Direction::ClientToAgent, true, &status(k as u32 * 100, 0)));
            }
        }
        log.sort_by_key(|f| f.time_ns);
        log
    }

    fn stream<'a>(r: &'a Report, suffix: &str) -> &'a StreamDiff {
        r.streams.iter().find(|s| s.name.ends_with(suffix)).unwrap_or_else(|| panic!("no stream {suffix}: {r:#?}"))
    }

    #[test]
    fn decodes_canmore_streams() {
        let d = decode(&session(1, 0), false);
        let names: std::collections::BTreeSet<&str> = d.events.iter().map(|e| e.stream.as_str()).collect();
        assert_eq!(
            names.into_iter().collect::<Vec<_>>(),
            [
                "can0 client 1 board->host FirmwareStatus",
                "can0 client 1 board->host Heartbeat",
                "can0 client 1 board->host heartbeat",
                "can0 client 1 host->board Ping",
            ]
        );
        let hb = d.events.iter().find(|e| e.stream.ends_with("heartbeat")).unwrap();
        assert_eq!(hb.payload, "mode=normal error=false term_valid=false term_enabled=false");
        assert!(hb.detail.ends_with("(cnt=0)"));
        assert!(d.errors.is_empty());
        let raw = decode(&session(1, 0), true);
        assert!(raw.events.iter().all(|e| e.stream.starts_with("can0 id ")));
    }

    #[test]
    fn reassembles_long_messages_and_reports_errors() {
        // A status over classic CAN takes several frames; drop a middle frame of the second one
        let one = msg_frames(3, Direction::ClientToAgent, false, &status(5, 0));
        assert!(one.len() > 2);
        let mut log: Vec<LogFrame> = Vec::new();
        for (k, f) in one.iter().chain(one.iter().enumerate().filter(|(i, _)| *i != 1).map(|(_, f)| f)).enumerate() {
            log.push(LogFrame { time_ns: k as u64 * 1000, iface: "can1".into(), frame: f.clone() });
        }
        let d = decode(&log, false);
        assert_eq!(d.events.len(), 1);
        assert!(d.events[0].payload.contains("uptime_ms: 5"), "{:?}", d.events[0]);
        assert!(d.errors["can1 client 3 board->host"].1.starts_with("BadSeqNum"), "{:?}", d.errors);
    }

    #[test]
    fn identical_logs_compare_identical() {
        let a = decode(&session(3, 0), false);
        let r = diff(&a, &a.clone(), &DiffOptions::default());
        assert!(r.identical(), "{}", r.render(5));
        assert_eq!(r.max_abs_skew_ns(), 0);
        let ping = stream(&r, "Ping");
        assert_eq!((ping.a_count, ping.matched), (30, 30));
        let p = ping.period_a.unwrap();
        assert!((p.mean_ms - 100.0).abs() < 1e-9 && p.std_ms < 1e-9);
        assert!(r.render(5).contains("identical"));
    }

    #[test]
    fn missing_extra_and_payload_differences() {
        let a = session(3, 0);
        let mut b = a.clone();
        // Drop the CANmore heartbeat at 1.007 s, change the status at 2.009 s, add a stray utility frame
        b.retain(|f| f.time_ns != 1_007_000_000);
        for f in b.iter_mut().filter(|f| f.time_ns == 2_009_000_000) {
            f.frame = msg_frames(1, Direction::ClientToAgent, true, &status(2000, 4)).remove(0);
        }
        let stray = canmore::id::util_id(1, Direction::ClientToAgent, 3);
        b.push(LogFrame { time_ns: 1_500_000_000, iface: "can0".into(), frame: CanFrame::classic(stray, false, &[1]) });
        b.sort_by_key(|f| f.time_ns);
        let r = diff(&decode(&a, false), &decode(&b, false), &DiffOptions::default());
        assert!(!r.identical());
        assert_eq!(r.counts(), (1, 1, 1));
        let hb = stream(&r, "board->host heartbeat");
        assert_eq!(hb.missing[0].0, 1_007_000_000);
        let st = stream(&r, "FirmwareStatus");
        assert_eq!(payload_change(&st.payload_diffs[0].1, &st.payload_diffs[0].2), "faults: 0 -> 4");
        assert_eq!(stream(&r, "util ch3").extra.len(), 1);
        let text = r.render(5);
        assert!(text.contains("faults: 0 -> 4") && text.contains("1 missing in B"), "{text}");
        // The payload of an ignored stream is not compared
        let opts = DiffOptions { ignore_payload: vec!["FirmwareStatus".into()], ..Default::default() };
        assert_eq!(diff(&decode(&a, false), &decode(&b, false), &opts).counts(), (1, 1, 0));
    }

    #[test]
    fn logs_of_different_lengths_compare_on_their_overlap() {
        // B is a hardware log started 1.2 s into A's run, at a Unix timestamp, and stopped earlier
        let a = session(4, 0);
        let b: Vec<LogFrame> = a
            .iter()
            .filter(|f| (1_200_000_000..3_000_000_000).contains(&f.time_ns))
            .map(|f| LogFrame { time_ns: f.time_ns + 1_700_000_000_000_000_000, ..f.clone() })
            .collect();
        let (da, db) = (decode(&a, false), decode(&b, false));
        let r = diff(&da, &db, &DiffOptions { align: Align::Offset(1_700_000_000_000_000_000), ..Default::default() });
        assert!(r.identical(), "{}", r.render(3));
        assert_eq!(r.window, (1_200_000_000, 2_905_000_000));
        assert!(stream(&r, "Ping").outside.0 > 0);
        // The offset can be searched for
        let auto = diff(&da, &db, &DiffOptions { align: Align::Auto, ..Default::default() });
        assert_eq!(auto.offset_ns, 1_700_000_000_000_000_000);
        assert!(auto.identical(), "{}", auto.render(3));
        // Lining up the first frames is wrong here
        let start = diff(&da, &db, &DiffOptions::default());
        assert_eq!(start.offset_ns, 1_700_000_000_000_000_000 + 1_200_000_000);
        // Counting everything flags what B never recorded
        let full = diff(&da, &db, &DiffOptions { align: Align::Auto, full: true, ..Default::default() });
        assert!(full.counts().0 > 0);
    }

    #[test]
    fn timing_differences_and_stream_offsets() {
        let a = session(3, 0);
        // In B the board's CANmore heartbeat comes 30 ms later and its link heartbeat jitters by ±2 ms
        let mut b = a.clone();
        let hb_id = canmore::id::util_id(1, Direction::ClientToAgent, canmore::CHAN_HEARTBEAT);
        let link_hb = msg_frames(1, Direction::ClientToAgent, true, &DeviceToHost::Heartbeat).remove(0);
        let mut k = 0i64;
        for f in &mut b {
            if f.frame.id == hb_id && !f.frame.extended {
                f.time_ns += 30_000_000;
            } else if f.frame == link_hb {
                k += 1;
                f.time_ns = (f.time_ns as i64 + if k % 2 == 0 { 2_000_000 } else { -2_000_000 }) as u64;
            }
        }
        b.sort_by_key(|f| f.time_ns);
        let (da, db) = (decode(&a, false), decode(&b, false));
        let r = diff(&da, &db, &DiffOptions::default());
        assert!(r.identical(), "{}", r.render(3));
        let hb = stream(&r, "board->host heartbeat");
        assert_eq!(hb.offset_ns, 30_000_000);
        assert_eq!(hb.max_abs_skew_ns, 30_000_000);
        let link = stream(&r, "board->host Heartbeat");
        let p = link.period_b.unwrap();
        assert!((p.mean_ms - 100.0).abs() < 0.2 && (p.std_ms - 4.0).abs() < 0.2, "{p:?}");
        assert!(link.period_a.unwrap().std_ms < 1e-9);
        // Without stream alignment the late heartbeats do not pair within 20 ms
        let strict = diff(&da, &db, &DiffOptions { stream_align: false, ..Default::default() });
        assert_eq!(stream(&strict, "board->host heartbeat").matched, 0);
    }

    #[test]
    fn parses_and_diffs_text_logs() {
        let a = parse("(1.000000) can0 123#01\n(1.100000) can0 123#02\n(1.200000) can0 123#03\n").unwrap();
        let b = parse("(5.000000) can0 123#01\n(5.100000) can0 123#07\n").unwrap();
        let r = diff(&decode(&a, true), &decode(&b, true), &DiffOptions { full: true, ..Default::default() });
        assert_eq!(r.counts(), (1, 0, 1));
        assert_eq!(payload_change("classic 02", "classic 07"), "02 -> 07");
    }

    #[test]
    fn variant_paths() {
        assert_eq!(variant_path("Esc(Telemetry { rpm: 3 })"), "Esc::Telemetry");
        assert_eq!(variant_path("Ping { protocol_version: 2 }"), "Ping");
        assert_eq!(variant_path("Heartbeat"), "Heartbeat");
        assert_eq!(variant_path("Power(ElectricalReadings(ElectricalReadings { a: 1 }))"), "Power::ElectricalReadings");
        assert_eq!(variant_path("Led(SetColor(1, 2))"), "Led::SetColor");
    }

    #[test]
    fn payload_change_falls_back_to_context() {
        assert_eq!(payload_change("Foo { a: [1, 2] }", "Foo { a: [1, 2, 3] }"), "...Foo { a: [1, 2[ -> , 3]");
    }
}
