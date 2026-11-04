//! candump log files: reading them (`candump -l` logs and candump's default text output), writing simulated
//! traffic as `candump -l` logs, and replaying recorded traffic into a simulated bus as a host endpoint.
//!
//! The `candump -l` format is one frame per line: `(1699999999.123456) can0 123#DEADBEEF`, with `ID##F...` for CAN
//! FD (`F` is the flags nibble: 1 = BRS, 2 = ESI) and `ID#R` for remote frames. A three digit identifier is a
//! standard one, eight digits an extended one. candump's text output (`candump can0`, with or without `-t a`, `-x`
//! or `-a`) looks like `(1699999999.123456)  can0  123   [4]  DE AD BE EF`, with a two digit length (`[08]`) for
//! CAN FD frames. Both are read; error frames and CAN XL frames are skipped.
//!
//! A simulated log uses the virtual time as its timestamp (`(0000000001.250000)` is 1.25 s after power-on) and the
//! bus name as the interface, and has the frames a listening interface would have received: acknowledged ones,
//! stamped with the time their last bit was sent.

use std::collections::VecDeque;
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, Mutex};

use canmore::id::{Direction, Fields, FrameType};
use sim_core::time::Time;
use sim_devices::can::{BitRates, BusRecord, CanEndpoint, CanFrame};

use crate::session::CanCallback;

/// Timestamps below this (about 11.6 days) are times since power-on (an emulator log, or `candump -t z`) rather
/// than Unix times.
pub const RELATIVE_LIMIT_NS: u64 = 1_000_000 * 1_000_000_000;

/// One frame of a log.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct LogFrame {
    /// Timestamp in nanoseconds (since the Unix epoch for a hardware log, since power-on for a simulated one).
    pub time_ns: u64,
    /// Interface (`can0`) or, in a simulated log, bus name (`internal`).
    pub iface: String,
    pub frame: CanFrame,
}

/// A parse error with its line number (1-based).
#[derive(Clone, PartialEq, Eq, Debug, thiserror::Error)]
#[error("line {line}: {msg}")]
pub struct ParseError {
    pub line: usize,
    pub msg: String,
}

/// Reads a log file (see the module documentation for the formats).
pub fn read_file(path: &Path) -> Result<Vec<LogFrame>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Parses the text of a log. Blank lines and lines starting with `#` (comments in hand-written logs) are skipped,
/// as are error frames and CAN XL frames. Frames come back in file order.
pub fn parse(text: &str) -> Result<Vec<LogFrame>, ParseError> {
    let mut out = Vec::new();
    // A log without timestamps (plain `candump can0`) gets increasing ones 1 µs apart, keeping the order
    let mut untimed = 0u64;
    for (i, raw) in text.lines().enumerate() {
        let err = |msg: String| ParseError { line: i + 1, msg };
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (time_ns, rest) = match line.strip_prefix('(') {
            Some(r) => {
                let (ts, rest) = r.split_once(')').ok_or_else(|| err("unterminated timestamp".into()))?;
                (parse_timestamp(ts).map_err(err)?, rest.trim_start())
            }
            None => {
                untimed += 1_000;
                (untimed, line)
            }
        };
        let mut tokens = rest.split_whitespace();
        let iface = tokens.next().ok_or_else(|| err("no interface".into()))?.to_string();
        let tokens: Vec<&str> = tokens.collect();
        let frame = match tokens.first() {
            Some(t) if t.contains('#') => parse_compact(t).map_err(err)?,
            Some(_) => parse_long(&tokens).map_err(err)?,
            None => return Err(err("no frame".into())),
        };
        if let Some(frame) = frame {
            out.push(LogFrame { time_ns, iface, frame });
        }
    }
    Ok(out)
}

/// `1699999999.123456` (any number of fraction digits) or candump's `-t A` date form `2023-11-03 12:00:00.123456`
/// (taken as UTC: only differences between timestamps matter here).
fn parse_timestamp(ts: &str) -> Result<u64, String> {
    let ts = ts.trim();
    let bad = || format!("bad timestamp {ts:?}");
    let (date, clock) = match ts.split_once(' ') {
        Some((d, c)) => (Some(d), c),
        None => (None, ts),
    };
    let (whole, frac) = clock.split_once('.').unwrap_or((clock, ""));
    let mut frac_ns = 0u64;
    for (k, c) in frac.chars().take(9).enumerate() {
        frac_ns += c.to_digit(10).ok_or_else(bad)? as u64 * 10u64.pow(8 - k as u32);
    }
    let secs: u64 = match date {
        None => whole.parse().map_err(|_| bad())?,
        Some(d) => {
            let ymd: Vec<i64> = d.split('-').map(|x| x.parse().map_err(|_| bad())).collect::<Result<_, _>>()?;
            let hms: Vec<u64> = whole.split(':').map(|x| x.parse().map_err(|_| bad())).collect::<Result<_, _>>()?;
            let (&[y, m, day], &[h, min, s]) = (&ymd[..], &hms[..]) else { return Err(bad()) };
            // (Years this far out are past what the nanoseconds below hold anyway; this keeps the day count in
            // range)
            if y.abs() > 1_000_000_000 || m.abs() > 1_000_000_000 || day.abs() > 1_000_000_000 {
                return Err(bad());
            }
            let days = u64::try_from(days_from_civil(y, m, day)).map_err(|_| bad())?;
            let parts = [(days, 86_400), (h, 3600), (min, 60), (s, 1)];
            parts.iter().try_fold(0u64, |acc, &(v, k)| v.checked_mul(k)?.checked_add(acc)).ok_or_else(bad)?
        }
    };
    secs.checked_mul(1_000_000_000).and_then(|ns| ns.checked_add(frac_ns)).ok_or_else(bad)
}

/// Days since 1970-01-01 of a proleptic Gregorian date (Howard Hinnant's algorithm).
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// An identifier as candump prints it: three digits are a standard identifier, eight an extended one (with the
/// kernel's flag bits, so an error frame has bit 29 set). Returns `None` for an error frame.
fn parse_id(s: &str) -> Result<Option<(u32, bool)>, String> {
    let v = u32::from_str_radix(s, 16).map_err(|_| format!("bad identifier {s:?}"))?;
    if s.len() > 3 || v > 0x7FF {
        if v & 0x2000_0000 != 0 {
            return Ok(None);
        }
        Ok(Some((v & 0x1FFF_FFFF, true)))
    } else {
        Ok(Some((v, false)))
    }
}

fn parse_hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let digits: Vec<u8> = s.bytes().filter(|&b| b != b'.').collect();
    if !digits.len().is_multiple_of(2) {
        return Err(format!("odd number of hex digits in {s:?}"));
    }
    digits
        .chunks(2)
        .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).map_err(|_| format!("bad data {s:?}")))
        .collect()
}

/// `123#11223344`, `123#R`, `123#R4`, `12345678##1AABB` (FD), `123#1122334455667788_9` (classic DLC suffix).
fn parse_compact(tok: &str) -> Result<Option<CanFrame>, String> {
    if tok.contains("###") {
        return Ok(None); // CAN XL
    }
    let (id, rest) = tok.split_once('#').unwrap();
    let Some((id, extended)) = parse_id(id)? else { return Ok(None) };
    if let Some(fd) = rest.strip_prefix('#') {
        let mut chars = fd.chars();
        let flags = chars.next().and_then(|c| c.to_digit(16)).ok_or_else(|| format!("no FD flags in {tok:?}"))?;
        let data = parse_hex_bytes(chars.as_str())?;
        if data.len() > 64 {
            return Err(format!("{} bytes in a CAN FD frame", data.len()));
        }
        let mut f = CanFrame::fd(id, extended, &data);
        f.brs = flags & 1 != 0;
        f.esi = flags & 2 != 0;
        return Ok(Some(f));
    }
    if let Some(r) = rest.strip_prefix(['R', 'r']) {
        let len = r.chars().next().and_then(|c| c.to_digit(16)).unwrap_or(0) as usize;
        let mut f = CanFrame::classic(id, extended, &vec![0; len.min(8)]);
        f.rtr = true;
        return Ok(Some(f));
    }
    let data = parse_hex_bytes(rest.split('_').next().unwrap())?;
    if data.len() > 8 {
        return Err(format!("{} bytes in a classic frame", data.len()));
    }
    Ok(Some(CanFrame::classic(id, extended, &data)))
}

/// candump's text output after the interface: `[RX|TX F F] ID [N] DATA...` or `ID [N] remote request`.
fn parse_long(tokens: &[&str]) -> Result<Option<CanFrame>, String> {
    let mut t = tokens;
    // `-x` adds the direction and two flag columns (BRS `B`, ESI `E`, `-` for unset)
    let mut x_flags = None;
    if matches!(t.first(), Some(&"RX") | Some(&"TX")) && t.len() >= 3 {
        x_flags = Some((t[1] == "B", t[2] == "E"));
        t = &t[3..];
    }
    let [id, len, rest @ ..] = t else { return Err("expected `ID [LEN] DATA`".into()) };
    let Some((id, extended)) = parse_id(id)? else { return Ok(None) };
    let digits =
        len.strip_prefix('[').and_then(|l| l.strip_suffix(']')).ok_or_else(|| format!("bad length {len:?}"))?;
    let n: usize = digits.parse().map_err(|_| format!("bad length {len:?}"))?;
    // candump prints a CAN FD length with two digits, a classic one with one
    let fd = digits.len() == 2;
    if n > if fd { 64 } else { 8 } {
        return Err(format!("{n} bytes in a {} frame", if fd { "CAN FD" } else { "classic" }));
    }
    if !fd && rest.first() == Some(&"remote") {
        let mut f = CanFrame::classic(id, extended, &vec![0; n.min(8)]);
        f.rtr = true;
        return Ok(Some(f));
    }
    let data: Vec<u8> = rest
        .iter()
        .take(n)
        .map(|b| u8::from_str_radix(b, 16).map_err(|_| format!("bad data byte {b:?}")))
        .collect::<Result<_, _>>()?;
    if data.len() != n {
        return Err(format!("{n} bytes announced, {} found", data.len()));
    }
    Ok(Some(if fd {
        let mut f = CanFrame::fd(id, extended, &data);
        if let Some((brs, esi)) = x_flags {
            f.brs = brs;
            f.esi = esi;
        }
        f
    } else {
        CanFrame::classic(id, extended, &data)
    }))
}

/// A frame in `candump -l` notation: `123#DEADBEEF`, `12345678##1DEADBEEF` (FD, flags nibble), `123#R`.
pub fn compact(frame: &CanFrame) -> String {
    let id = if frame.extended { format!("{:08X}", frame.id) } else { format!("{:03X}", frame.id) };
    let data: String = frame.data.iter().map(|b| format!("{b:02X}")).collect();
    if frame.fd {
        format!("{id}##{:X}{data}", frame.brs as u8 | (frame.esi as u8) << 1)
    } else if frame.rtr {
        if frame.data.is_empty() { format!("{id}#R") } else { format!("{id}#R{}", frame.data.len()) }
    } else {
        format!("{id}#{data}")
    }
}

/// One `candump -l` line.
pub fn format_line(time_ns: u64, iface: &str, frame: &CanFrame) -> String {
    let secs = time_ns / 1_000_000_000;
    let us = (time_ns % 1_000_000_000) / 1_000;
    format!("({secs:010}.{us:06}) {iface} {}", compact(frame))
}

/// [`format_line`] to the nanosecond (the log's reader takes up to nine decimals), for times that must come back
/// exactly ([`Arrivals`]).
pub fn format_line_ns(time_ns: u64, iface: &str, frame: &CanFrame) -> String {
    let secs = time_ns / 1_000_000_000;
    let ns = time_ns % 1_000_000_000;
    format!("({secs:010}.{ns:09}) {iface} {}", compact(frame))
}

/// Writes a `candump -l` log of one or more buses to a file. Clone it to hand a hook to each bus; it flushes at
/// least every half second of wall time, so a run stopped with Ctrl-C leaves a usable log, and when dropped.
#[derive(Clone)]
pub struct CandumpWriter(Arc<Mutex<WriterState>>);

struct WriterState {
    out: Box<dyn Write + Send>,
    written: u64,
    /// Frames not written because nobody acknowledged them (a listening interface would have).
    unacked: u64,
    error: Option<String>,
}

impl Drop for WriterState {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

impl CandumpWriter {
    pub fn create(path: &Path) -> std::io::Result<CandumpWriter> {
        let f = std::fs::File::create(path)?;
        Ok(Self::new(Box::new(std::io::BufWriter::new(f))))
    }

    /// A log written to `out`, flushed every [`crate::flush::FLUSH_PERIOD`] (the last frames too, when the bus
    /// goes quiet).
    pub fn new(out: Box<dyn Write + Send>) -> CandumpWriter {
        let state = Arc::new(Mutex::new(WriterState { out, written: 0, unacked: 0, error: None }));
        crate::flush::flush_periodically(&state, crate::flush::FLUSH_PERIOD, |m| {
            if let Ok(mut s) = m.lock() {
                let _ = s.out.flush();
            }
        });
        CandumpWriter(state)
    }

    /// Writes one bus record (skipped if it was not acknowledged).
    pub fn record(&self, iface: &str, rec: &BusRecord) {
        let mut s = self.0.lock().unwrap();
        if !rec.acked {
            s.unacked += 1;
            return;
        }
        let line = format_line(rec.end.as_ns(), iface, &rec.frame);
        if let Err(e) = writeln!(s.out, "{line}") {
            s.error.get_or_insert(e.to_string());
            return;
        }
        s.written += 1;
    }

    /// Writes a frame a live endpoint handed in at `at`, to the nanosecond ([`Arrivals`]).
    pub fn arrival(&self, iface: &str, at: Time, frame: &CanFrame) {
        let mut s = self.0.lock().unwrap();
        if let Err(e) = writeln!(s.out, "{}", format_line_ns(at.as_ns(), iface, frame)) {
            s.error.get_or_insert(e.to_string());
            return;
        }
        s.written += 1;
    }

    /// A bus callback writing that bus's frames as `iface`, after calling `then` (an existing callback, such as
    /// `--candump` printing) if given.
    pub fn hook(&self, iface: &str, mut then: Option<CanCallback>) -> CanCallback {
        let w = self.clone();
        let iface = iface.to_string();
        Box::new(move |rec| {
            if let Some(f) = &mut then {
                f(rec);
            }
            w.record(&iface, rec);
        })
    }

    pub fn flush(&self) -> std::io::Result<()> {
        self.0.lock().unwrap().out.flush()
    }

    /// Frames written and frames skipped for want of an acknowledgement.
    pub fn counts(&self) -> (u64, u64) {
        let s = self.0.lock().unwrap();
        (s.written, s.unacked)
    }

    /// The first write error, if any.
    pub fn error(&self) -> Option<String> {
        self.0.lock().unwrap().error.clone()
    }
}

/// Which of two ways a CANmore frame travels, by who sends it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sender {
    /// The robot computer (CANmore agent: `titan_bridge`, `canmore_cli`, `upload_tool`).
    Host,
    /// A board (CANmore client).
    Board,
}

#[derive(Clone, PartialEq, Eq, Debug)]
enum Term {
    Iface(Vec<String>),
    Client(Vec<u8>),
    Sender(Sender),
    Type(FrameType),
    /// Utility channel (NOC of a utility frame).
    Channel(Vec<u8>),
    /// Raw identifier and mask.
    Id(u32, u32),
}

/// Selects frames of a log by CANmore fields: `key=value` terms joined by `,`, all of which must hold; a term
/// starting with `!` must not. Several values for one key are separated by `|`.
///
/// * `sender=host` (what the robot computer sent) or `sender=board`;
/// * `client=1|8`: CANmore client ID;
/// * `type=msg` or `type=util`; `channel=15` (utility channel, implies `type=util`);
/// * `iface=can0|can1`;
/// * `id=0x123` or `id=0x100/0x700`: raw identifier with an optional mask.
///
/// A list of filters (repeated `--replay-filter` options) selects a frame if any of them does; an empty list selects
/// every frame. For example `sender=host` replays what the bridges sent, and `sender=host` plus `!client=2` replays
/// everything except what board 2 sent, for an emulated board 2 to take its place.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Filter {
    terms: Vec<(bool, Term)>,
}

impl Filter {
    pub fn parse(spec: &str) -> Result<Filter, String> {
        let mut terms = Vec::new();
        for part in spec.split(',').map(str::trim).filter(|p| !p.is_empty()) {
            let (negate, part) = match part.strip_prefix('!') {
                Some(p) => (true, p.trim()),
                None => (false, part),
            };
            let (key, value) = part.split_once('=').ok_or_else(|| format!("{part:?}: expected KEY=VALUE"))?;
            let values: Vec<&str> = value.split('|').map(str::trim).collect();
            let num = |v: &str| -> Result<u32, String> {
                let r = match v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
                    Some(h) => u32::from_str_radix(h, 16),
                    None => v.parse(),
                };
                r.map_err(|_| format!("{v:?} is not a number"))
            };
            let small = |v: &str, max: u32| -> Result<u8, String> {
                let n = num(v)?;
                if n > max { Err(format!("{n} is out of range (0-{max})")) } else { Ok(n as u8) }
            };
            let term = match key.trim() {
                "iface" => Term::Iface(values.iter().map(|v| v.to_string()).collect()),
                "client" => Term::Client(values.iter().map(|v| small(v, 31)).collect::<Result<_, _>>()?),
                "channel" => Term::Channel(values.iter().map(|v| small(v, 15)).collect::<Result<_, _>>()?),
                "sender" => Term::Sender(match value.trim() {
                    "host" | "agent" => Sender::Host,
                    "board" | "client" => Sender::Board,
                    v => return Err(format!("sender={v}: expected host or board")),
                }),
                "type" => Term::Type(match value.trim() {
                    "msg" => FrameType::Msg,
                    "util" => FrameType::Util,
                    v => return Err(format!("type={v}: expected msg or util")),
                }),
                "id" => {
                    let (id, mask) = match value.split_once('/') {
                        Some((i, m)) => (num(i.trim())?, num(m.trim())?),
                        None => (num(value.trim())?, 0x1FFF_FFFF),
                    };
                    Term::Id(id, mask)
                }
                k => return Err(format!("unknown filter key {k:?} (iface, client, sender, type, channel, id)")),
            };
            terms.push((negate, term));
        }
        Ok(Filter { terms })
    }

    pub fn matches(&self, iface: &str, frame: &CanFrame) -> bool {
        let f = Fields::decode(frame.id, frame.extended);
        self.terms.iter().all(|(negate, term)| {
            let hit = match term {
                Term::Iface(names) => names.iter().any(|n| n == iface),
                Term::Client(ids) => ids.contains(&f.client_id),
                Term::Sender(s) => {
                    *s == if f.direction == Direction::AgentToClient { Sender::Host } else { Sender::Board }
                }
                Term::Type(t) => f.frame_type == *t,
                Term::Channel(chs) => f.frame_type == FrameType::Util && chs.contains(&f.noc),
                Term::Id(id, mask) => frame.id & mask == id & mask,
            };
            hit != *negate
        })
    }

    /// Whether any of `filters` selects the frame (every frame if there are none).
    pub fn any(filters: &[Filter], iface: &str, frame: &CanFrame) -> bool {
        filters.is_empty() || filters.iter().any(|f| f.matches(iface, frame))
    }
}

/// How log timestamps become virtual times when replaying.
#[derive(Clone, Debug)]
pub struct ReplayOptions {
    /// Interfaces of the log to take frames from (all if empty).
    pub ifaces: Vec<String>,
    /// Frames to replay (see [`Filter::any`]).
    pub filters: Vec<Filter>,
    /// Virtual time at which the log's origin is replayed.
    pub offset: Time,
    /// Log timestamp (ns) that maps to `offset`; by default the log's first frame (on any interface, selected or
    /// not), which keeps the selected frames where they were relative to everything else in the log.
    pub origin_ns: Option<u64>,
    /// The bus rates: a frame is made ready its on-wire duration before its logged time (when its last bit was
    /// sent), so on a free bus it ends when it ended in the log.
    pub rates: Option<BitRates>,
}

impl Default for ReplayOptions {
    fn default() -> Self {
        ReplayOptions { ifaces: Vec::new(), filters: Vec::new(), offset: Time::ZERO, origin_ns: None, rates: None }
    }
}

/// A host endpoint that sends recorded frames at their (time shifted) logged times: the in-process `canplayer`.
#[derive(Clone, Debug, Default)]
pub struct Replay {
    queue: VecDeque<(Time, CanFrame)>,
    /// Frames handed to the bus so far.
    pub sent: u64,
    /// Frames that would have been replayed before virtual time 0 (an origin after them) and were dropped.
    pub dropped: u64,
}

impl Replay {
    /// Frames to send at the given virtual times (sorted here).
    pub fn new(mut frames: Vec<(Time, CanFrame)>) -> Replay {
        frames.sort_by_key(|(t, _)| *t);
        Replay { queue: frames.into(), sent: 0, dropped: 0 }
    }

    /// The selected frames of a log, timed as `opts` says.
    pub fn from_log(log: &[LogFrame], opts: &ReplayOptions) -> Replay {
        let Some(origin) = opts.origin_ns.or_else(|| log.iter().map(|f| f.time_ns).min()) else {
            return Replay::default();
        };
        let mut frames = Vec::new();
        let mut dropped = 0;
        for lf in log {
            if !opts.ifaces.is_empty() && !opts.ifaces.contains(&lf.iface) {
                continue;
            }
            if !Filter::any(&opts.filters, &lf.iface, &lf.frame) {
                continue;
            }
            let Some(rel) = lf.time_ns.checked_sub(origin) else {
                dropped += 1;
                continue;
            };
            // Saturating: a log spanning more than the ~213 days a `Time` holds replays the rest at the end of time
            let mut at = opts.offset + Time::from_ps(rel.saturating_mul(1_000));
            if let Some(r) = opts.rates {
                at = at.saturating_sub(lf.frame.duration(r.nominal, r.data.unwrap_or(r.nominal)));
            }
            frames.push((at, lf.frame.clone()));
        }
        let mut r = Replay::new(frames);
        r.dropped = dropped;
        r
    }

    /// Frames still to send.
    pub fn remaining(&self) -> usize {
        self.queue.len()
    }

    /// When the next frame is due, if any are left.
    pub fn first_time(&self) -> Option<Time> {
        self.queue.front().map(|(t, _)| *t)
    }

    /// When the last frame is due, if any are left.
    pub fn last_time(&self) -> Option<Time> {
        self.queue.back().map(|(t, _)| *t)
    }
}

impl CanEndpoint for Replay {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut out = Vec::new();
        while self.queue.front().is_some_and(|(t, _)| *t <= until) {
            out.push(self.queue.pop_front().unwrap());
        }
        self.sent += out.len() as u64;
        out
    }

    fn receive(&mut self, _frame: &CanFrame, _at: Time) {}

    /// Its next frame's time: a robot runs its boards ahead of each other up to there.
    fn idle_until(&self) -> Option<Time> {
        Some(self.first_time().unwrap_or(Time::MAX))
    }
}

/// Records when a live endpoint's frames entered the bus ([`CanEndpoint::live`]: what reached a SocketCAN interface
/// or a cannelloni tunnel, handed in at the bus step that read it), as a log ([`CandumpWriter::arrival`]) with
/// each frame at that step's time, to the nanosecond, as `iface`. [`arrivals_replay`] hands the same frames in at
/// the same steps again, so a run with real host tools can be repeated exactly, taking turns or running ahead.
pub struct Arrivals {
    pub inner: Box<dyn CanEndpoint + Send>,
    pub iface: String,
    pub log: CandumpWriter,
}

impl CanEndpoint for Arrivals {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let out = self.inner.take_tx(until);
        for (at, f) in &out {
            self.log.arrival(&self.iface, *at, f);
        }
        out
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        self.inner.receive(frame, at);
    }

    fn tx_queue_limit(&self) -> Option<usize> {
        self.inner.tx_queue_limit()
    }

    fn idle_until(&self) -> Option<Time> {
        self.inner.idle_until()
    }

    fn live(&self) -> bool {
        self.inner.live()
    }
}

/// The frames of `iface` in a log of arrivals ([`Arrivals`]), handed in at the times logged, as they were then.
pub fn arrivals_replay(log: &[LogFrame], iface: &str) -> Replay {
    let opts = ReplayOptions { ifaces: vec![iface.to_string()], origin_ns: Some(0), ..Default::default() };
    Replay::from_log(log, &opts)
}

/// A [`Replay`] shared with whoever wants to watch its progress after handing it to a bus.
#[derive(Clone, Default)]
pub struct SharedReplay(pub Arc<Mutex<Replay>>);

impl SharedReplay {
    pub fn new(r: Replay) -> Self {
        SharedReplay(Arc::new(Mutex::new(r)))
    }
}

impl CanEndpoint for SharedReplay {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.0.lock().unwrap().take_tx(until)
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        self.0.lock().unwrap().receive(frame, at)
    }

    fn idle_until(&self) -> Option<Time> {
        self.0.lock().unwrap().idle_until()
    }
}

/// A bus record as the log [`CandumpWriter`] would have written it (acknowledged frames, stamped with their end
/// to the microsecond), for tests that record with `CanBus::record` and compare afterwards.
pub fn from_records(records: &[BusRecord], iface: &str) -> Vec<LogFrame> {
    records
        .iter()
        .filter(|r| r.acked)
        .map(|r| LogFrame { time_ns: r.end.as_ns() / 1000 * 1000, iface: iface.to_string(), frame: r.frame.clone() })
        .collect()
}

/// A log as `candump -l` text.
pub fn to_text(log: &[LogFrame]) -> String {
    log.iter().map(|f| format_line(f.time_ns, &f.iface, &f.frame) + "\n").collect()
}

/// The interfaces a log has frames on, in order of first appearance.
pub fn interfaces(log: &[LogFrame]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for f in log {
        if !out.contains(&f.iface) {
            out.push(f.iface.clone());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_devices::can::{CanBus, Monitor};

    /// Arrivals are logged to the nanosecond and replayed at exactly those times; a replay tells when it sends.
    #[test]
    fn arrivals_replay_at_their_times() {
        #[derive(Clone, Default)]
        struct Buf(Arc<Mutex<Vec<u8>>>);
        impl Write for Buf {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                self.0.lock().unwrap().extend_from_slice(b);
                Ok(b.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let buf = Buf::default();
        let log = CandumpWriter::new(Box::new(buf.clone()));
        let (a, b) = (CanFrame::fd(0x123, true, &[1, 2]), CanFrame::classic(0x6F, false, &[0xC0]));
        let inner =
            Monitor { outbox: vec![(Time(1_234_567_891), a.clone()), (Time(5), b.clone())], ..Default::default() };
        let mut ep = Arrivals { inner: Box::new(crate::socketcan::Live(inner)), iface: "internal".into(), log };
        assert!(ep.live() && ep.idle_until().is_none());
        let at = Time::from_ns(1_234_567_900);
        assert_eq!(ep.take_tx(at).len(), 2);
        ep.log.flush().unwrap();
        let text = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
        assert_eq!(text.lines().next(), Some("(0000000001.234567900) internal 00000123##10102"));
        let mut r = arrivals_replay(&parse(&text).unwrap(), "internal");
        assert!(arrivals_replay(&parse(&text).unwrap(), "external").remaining() == 0);
        assert_eq!(r.idle_until(), Some(at));
        assert!(r.take_tx(at - Time(1)).is_empty());
        assert_eq!(r.take_tx(at), [(at, a), (at, b)]);
        assert_eq!(r.idle_until(), Some(Time::MAX));
        let shared = SharedReplay::new(Replay::new(vec![(Time::from_us(7), CanFrame::classic(1, false, &[]))]));
        assert_eq!(shared.idle_until(), Some(Time::from_us(7)));
    }

    #[test]
    fn parses_candump_l() {
        let log = parse(
            "(1699999999.123456) can0 06F#C0\n\
             (1699999999.200000) can0 01400801##100AABB\n\
             (1699999999.300000) can1 123#R\n\
             (1699999999.300001) can1 123#R3\n\
             (1699999999.400000) can0 12345678##3\n\
             (1699999999.500000) can0 20000080#0000000000000000\n\
             (1699999999.600000) can0 7FF#11.22.33\n\
             (1699999999.700000) can0 001#1122334455667788_9\n\
             # a comment\n\n",
        )
        .unwrap();
        assert_eq!(log.len(), 7, "the error frame is skipped");
        assert_eq!(log[0].time_ns, 1_699_999_999_123_456_000);
        assert_eq!((log[0].iface.as_str(), log[0].frame.clone()), ("can0", CanFrame::classic(0x6F, false, &[0xC0])));
        assert_eq!(log[1].frame, CanFrame::fd(0x1400801, true, &[0x00, 0xAA, 0xBB]));
        assert!(log[2].frame.rtr && log[2].frame.data.is_empty());
        assert_eq!(log[3].frame.data.len(), 3);
        assert!(log[4].frame.fd && log[4].frame.brs && log[4].frame.esi && log[4].frame.data.is_empty());
        assert_eq!(log[5].frame.data, [0x11, 0x22, 0x33]);
        assert_eq!(log[6].frame.data.len(), 8);
        assert_eq!(interfaces(&log), ["can0", "can1"]);
    }

    #[test]
    fn parses_candump_text_output() {
        let log = parse(
            " (1699999999.000100)  vcan0  06F   [1]  C0\n\
             \x20 vcan0  01400801  [08]  02 00 01 00 00 00 00 00\n\
             (2023-11-14 22:13:20.000001)  can1  RX B -  123  [02]  AA BB   '..'\n\
             \x20 can1  456   [3]  remote request\n\
             \x20 can0  20000004   [8]  00 04 00 00 00 00 00 00   ERRORFRAME\n",
        )
        .unwrap();
        assert_eq!(log.len(), 4);
        assert_eq!(log[0].frame, CanFrame::classic(0x6F, false, &[0xC0]));
        assert_eq!(log[0].time_ns, 1_699_999_999_000_100_000);
        assert!(log[1].frame.fd && log[1].frame.extended && log[1].frame.data.len() == 8);
        assert_eq!(log[1].time_ns, 1_000, "an untimed line keeps its place");
        // 2023-11-14 22:13:20 UTC is 1700000000
        assert_eq!(log[2].time_ns, 1_700_000_000_000_001_000);
        assert!(log[2].frame.fd && log[2].frame.brs && !log[2].frame.esi);
        assert_eq!(log[2].frame.data, [0xAA, 0xBB]);
        assert!(log[3].frame.rtr && log[3].frame.data.len() == 3);
    }

    #[test]
    fn parse_errors_name_the_line() {
        let e = parse("(1.0) can0 06F#C0\n(2.0) can0 XYZ#00\n").unwrap_err();
        assert_eq!(e.line, 2);
        assert!(parse("(1.0) can0 06F#C").is_err(), "odd digits");
        assert!(parse("(1.0 can0 06F#C0").is_err(), "unterminated timestamp");
        assert!(parse("(1.0) can0 06F#001122334455667788").is_err(), "nine bytes in a classic frame");
        assert!(parse("(1.0) can0 06F  [2]  01").is_err(), "short data");
        // candump's text output is held to the same limits as `-l` lines (a 65 byte frame would overrun a SocketCAN
        // frame's 64 byte buffer when replayed onto a bridged interface)
        let bytes = |n: usize| vec!["00"; n].join(" ");
        assert!(parse(&format!("(1.0) can0 06F  [65]  {}", bytes(65))).is_err(), "65 bytes in a CAN FD frame");
        assert!(parse(&format!("(1.0) can0 06F  [9]  {}", bytes(9))).is_err(), "nine bytes in a classic frame");
        assert!(parse(&format!("(1.0) can0 06F  [64]  {}", bytes(64))).is_ok());
        // Timestamps past what nanoseconds in a u64 hold are errors, not wrapped (or, with overflow checks, a panic)
        assert!(parse("(18446744074.000000) can0 06F#C0").is_err(), "584 years of nanoseconds");
        assert!(parse("(18446744073.709551) can0 06F#C0").is_ok());
        assert!(parse("(2023-11-14 5124095576030432:00:00.0) can0 06F#C0").is_err(), "hours");
        assert!(parse("(9223372036854775807-01-01 00:00:00.0) can0 06F#C0").is_err(), "years");
    }

    #[test]
    fn writes_what_it_reads() {
        let frames = [
            CanFrame::classic(0x6F, false, &[0xC0]),
            CanFrame::fd(0x1400801, true, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
            CanFrame { esi: true, brs: false, ..CanFrame::fd(0x10, false, &[]) },
            CanFrame { rtr: true, ..CanFrame::classic(0x7FF, false, &[0, 0]) },
            CanFrame::classic(0x5, true, &[]),
        ];
        let text: String = frames
            .iter()
            .enumerate()
            .map(|(i, f)| format_line(1_500_000_000 + i as u64 * 1_000_000_123, "internal", f) + "\n")
            .collect();
        assert!(text.starts_with("(0000000001.500000) internal 06F#C0\n"), "{text}");
        assert!(text.contains(" 01400801##1010203"), "{text}");
        assert!(text.contains(" 010##2\n"), "{text}");
        assert!(text.contains(" 7FF#R2\n"), "{text}");
        assert!(text.contains(" 00000005#\n"), "an extended identifier keeps eight digits: {text}");
        let back = parse(&text).unwrap();
        assert_eq!(back.iter().map(|f| f.frame.clone()).collect::<Vec<_>>(), frames);
        assert_eq!(back[1].time_ns, 2_500_000_000, "microsecond resolution");
    }

    #[test]
    fn writer_records_acknowledged_frames() {
        let buf = SharedBuf::default();
        let w = CandumpWriter::new(Box::new(buf.clone()));
        let rec = |acked, us| BusRecord {
            start: Time::ZERO,
            end: Time::from_us(us),
            sender: sim_devices::can::Sender::Node(0),
            frame: CanFrame::classic(0x6F, false, &[0xC0]),
            acked,
            error: None,
        };
        let mut printed = 0;
        {
            let counter = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let c = counter.clone();
            let mut hook = w.hook(
                "can0",
                Some(Box::new(move |_| {
                    c.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                })),
            );
            hook(&rec(true, 135));
            hook(&rec(false, 270));
            printed += counter.load(std::sync::atomic::Ordering::Relaxed);
        }
        w.flush().unwrap();
        assert_eq!(printed, 2, "the chained callback sees every frame");
        assert_eq!(w.counts(), (1, 1));
        assert_eq!(buf.text(), "(0000000000.000135) can0 06F#C0\n");
    }

    #[derive(Clone, Default)]
    struct SharedBuf(Arc<Mutex<Vec<u8>>>);
    impl SharedBuf {
        fn text(&self) -> String {
            String::from_utf8(self.0.lock().unwrap().clone()).unwrap()
        }
    }
    impl Write for SharedBuf {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    fn util(client: u8, dir: Direction, ch: u8) -> CanFrame {
        CanFrame::classic(canmore::id::util_id(client, dir, ch), false, &[0])
    }

    #[test]
    fn filters_by_canmore_fields() {
        let hb1 = util(1, Direction::ClientToAgent, canmore::CHAN_HEARTBEAT);
        let ctl2 = util(2, Direction::AgentToClient, canmore::CHAN_CONTROL_INTERFACE);
        let msg8 = CanFrame::fd(
            canmore::id::ext_id(
                8,
                FrameType::Msg,
                Direction::AgentToClient,
                0,
                canmore::id::msg_first_extra(true, 3, 1),
            ),
            true,
            &[0; 3],
        );
        let f = |s: &str| Filter::parse(s).unwrap();
        assert!(f("sender=board").matches("can0", &hb1));
        assert!(!f("sender=host").matches("can0", &hb1));
        assert!(f("sender=host,client=2|8").matches("can0", &ctl2));
        assert!(f("sender=host,client=2|8").matches("can0", &msg8));
        assert!(f("type=util,channel=15").matches("can0", &hb1));
        assert!(!f("channel=15").matches("can0", &ctl2));
        assert!(!f("channel=0").matches("can0", &msg8), "a message frame has no channel");
        assert!(f("type=msg").matches("can0", &msg8));
        assert!(f("!client=1").matches("can0", &ctl2) && !f("!client=1").matches("can0", &hb1));
        assert!(f("iface=can1|can0").matches("can0", &hb1) && !f("iface=can1").matches("can0", &hb1));
        assert!(f("id=0x06F").matches("x", &hb1) && f("id=0x040/0x7C0").matches("x", &hb1));
        assert!(f("").matches("x", &hb1), "an empty filter selects everything");
        // Repeated filters are alternatives: the bridges' traffic plus everything except board 2
        let both = [f("sender=host"), f("!client=2")];
        assert!(Filter::any(&both, "x", &ctl2) && Filter::any(&both, "x", &hb1));
        assert!(!Filter::any(&both, "x", &util(2, Direction::ClientToAgent, 15)));
        for bad in ["client", "client=32", "sender=me", "type=x", "foo=1", "channel=16", "id=zz"] {
            assert!(Filter::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn replay_time_shifts_and_filters() {
        let log = parse(
            "(100.000000) can0 06F#C0\n\
             (100.100000) can0 070#01\n\
             (100.250000) can1 070#02\n\
             (100.300000) can0 0AF#C1\n",
        )
        .unwrap();
        let opts = ReplayOptions {
            ifaces: vec!["can0".into()],
            filters: vec![Filter::parse("sender=board").unwrap()],
            offset: Time::from_ms(5),
            ..Default::default()
        };
        let mut r = Replay::from_log(&log, &opts);
        assert_eq!(r.remaining(), 2, "host frames and can1 are left out");
        assert_eq!(r.last_time(), Some(Time::from_ms(305)));
        assert_eq!(r.take_tx(Time::from_ms(4)).len(), 0);
        let first = r.take_tx(Time::from_ms(5));
        assert_eq!(first, vec![(Time::from_ms(5), CanFrame::classic(0x6F, false, &[0xC0]))]);
        assert_eq!(r.take_tx(Time::from_secs(1)).len(), 1);
        assert_eq!(r.sent, 2);
        // An origin after some frames drops them
        let late = Replay::from_log(&log, &ReplayOptions { origin_ns: Some(100_200_000_000), ..Default::default() });
        assert_eq!((late.remaining(), late.dropped), (2, 2));
    }

    /// A log spanning more than the ~213 days of picoseconds a `Time` holds (untimed lines among Unix-time ones,
    /// an early origin): the frames beyond are replayed at the end of time, after everything else, instead of
    /// wrapping round to an unrelated earlier time.
    #[test]
    fn replay_times_past_the_end_of_time_saturate() {
        let log = parse("(1.000000) can0 06F#C0\n(20000000.000000) can0 123#00\n(19.000000) can0 124#00\n").unwrap();
        let mut r = Replay::from_log(&log, &ReplayOptions::default());
        assert_eq!(r.first_time(), Some(Time::ZERO));
        assert_eq!(r.last_time(), Some(Time::MAX), "231 days after the origin");
        let sent: Vec<u32> = r.take_tx(Time::from_secs(100)).iter().map(|(_, f)| f.id).collect();
        assert_eq!(sent, [0x6F, 0x124]);
        assert_eq!(r.remaining(), 1);
    }

    #[test]
    fn replayed_frames_end_when_they_did_in_the_log() {
        let rates = sim_host_rates();
        let log = parse("(10.000500) can0 06F#C0\n(10.001000) can0 123#00112233\n").unwrap();
        let replay = Replay::from_log(
            &log,
            &ReplayOptions { origin_ns: Some(10_000_000_000), rates: Some(rates), ..Default::default() },
        );
        let mut bus = CanBus::new("can0", rates);
        bus.record = Some(Vec::new());
        let mut r = replay;
        let mut mon = Monitor::default();
        bus.step(&mut [], &mut [&mut r, &mut mon], Time::from_ms(2));
        let rec = bus.record.unwrap();
        assert_eq!(rec.len(), 2);
        assert_eq!(rec[0].end, Time::from_us(500));
        assert_eq!(rec[1].end, Time::from_us(1000));
        assert_eq!(mon.received.len(), 2, "the other endpoint hears the replay");
    }

    fn sim_host_rates() -> BitRates {
        crate::board::bus_rates("external").unwrap()
    }

    #[test]
    fn the_last_frame_reaches_the_file_when_the_bus_goes_quiet() {
        let path = crate::flush::tests::temp_path("candump.log");
        let w = CandumpWriter::create(&path).unwrap();
        let rec = BusRecord {
            start: Time::from_us(10),
            end: Time::from_us(120),
            sender: sim_devices::can::Sender::Node(1),
            frame: CanFrame::classic(0x6F, false, &[0xC0]),
            acked: true,
            error: None,
        };
        w.record("can0", &rec);
        let text = crate::flush::tests::wait_for_file(&path, |t| t.ends_with('\n'));
        assert!(text.ends_with("can0 06F#C0\n"), "{text:?}");
        drop(w);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
