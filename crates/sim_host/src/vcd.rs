//! Waveform export as a VCD file (IEEE 1364 value change dump) for GTKWave, Surfer or PulseView.
//!
//! Each board is a scope holding a 1-bit wire per selected GPIO, from the machine's GPIO trace: the resolved pin
//! level, whatever drives it (SIO, PWM, PIO, an external device, a fault). Optionally ([`VcdOptions::events`]) each
//! board also gets string variables with decoded traffic: defmt log lines, UART lines, CAN frames (candump
//! notation) and device reports (WS2812 frames, DShot commands, ...), and a robot gets one per CAN bus. String
//! variables are a GTKWave extension that Surfer also reads; PulseView imports only the wires.
//!
//! The time unit is 1 ns, which resolves a 125 MHz `clk_sys` (8 ns) and PIO edges. Only changes are written, as
//! they happen, through a small reordering window: boards of a robot run one after the other in 100 µs slices, so
//! events reach the writer up to a slice out of order; they are sorted before they are written.

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::{Arc, Mutex};

use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, TraceSink};

use crate::Session;

/// Number of GPIOs.
pub const PINS: usize = 30;

/// Events within this many nanoseconds of the newest one are held back for sorting. Besides the robot's slices, a
/// PIO block whose state machines were stalled is only advanced when something next looks at it, and then reports
/// its pin changes at the (earlier) cycles they happened.
const WINDOW_NS: u64 = 50_000_000;
/// Buffered changes before a flush is attempted (at least; it grows with what the window holds).
const FLUSH_AT: usize = 16_384;

/// What goes into the file for each board.
#[derive(Clone, Debug, Default)]
pub struct VcdOptions {
    /// GPIOs to record (all 30 if empty).
    pub pins: Vec<u8>,
    /// Add the decoded string variables (logs, UART, CAN, device reports).
    pub events: bool,
}

/// Parses a pin list: comma separated GPIO numbers, ranges (`0-29`), `all`, or names from a board file's `[pins]`
/// table (with or without its `_pin` suffix, e.g. `status_ledr` for `status_ledr_pin`).
pub fn parse_pins(spec: &str, named: &BTreeMap<String, u8>) -> Result<Vec<u8>, String> {
    let mut pins = Vec::new();
    for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        if item == "all" {
            pins.extend(0..PINS as u8);
        } else if let Some((a, b)) = item.split_once('-')
            && let (Ok(a), Ok(b)) = (a.trim().parse::<u8>(), b.trim().parse::<u8>())
        {
            if a > b || b as usize >= PINS {
                return Err(format!("bad pin range {item:?} (GPIOs are 0-29)"));
            }
            pins.extend(a..=b);
        } else if let Ok(n) = item.parse::<u8>() {
            if n as usize >= PINS {
                return Err(format!("no GPIO {n} (GPIOs are 0-29)"));
            }
            pins.push(n);
        } else if let Some(&n) = named.get(item).or_else(|| named.get(&format!("{item}_pin"))) {
            pins.push(n);
        } else {
            let names: Vec<&str> = named.keys().map(String::as_str).collect();
            return Err(format!("no pin {item:?}; the board names: {}", names.join(", ")));
        }
    }
    pins.sort_unstable();
    pins.dedup();
    Ok(pins)
}

/// The variable name of each GPIO: `gpio25`, or `gpio13_status_ledr` when the board file names the pin (its first
/// name in alphabetical order, without the `_pin` suffix).
pub fn pin_labels(named: &BTreeMap<String, u8>) -> Vec<String> {
    (0..PINS as u8)
        .map(|p| match named.iter().find(|(_, n)| **n == p) {
            Some((name, _)) => format!("gpio{p}_{}", name.strip_suffix("_pin").unwrap_or(name)),
            None => format!("gpio{p}"),
        })
        .collect()
}

/// A scope of the file: a board, or a CAN bus.
#[derive(Clone, Debug)]
pub struct Scope {
    pub name: String,
    /// GPIOs with their variable names.
    pub pins: Vec<(u8, String)>,
    /// String variables.
    pub strings: Vec<String>,
}

impl Scope {
    /// A board's scope: `options.pins` (all by default) labelled from the board's pin names, plus the event
    /// variables if asked for.
    pub fn board(name: &str, named: &BTreeMap<String, u8>, options: &VcdOptions) -> Scope {
        let labels = pin_labels(named);
        let pins: Vec<u8> = if options.pins.is_empty() { (0..PINS as u8).collect() } else { options.pins.clone() };
        let strings = if options.events {
            ["log", "uart0", "uart1", "can", "device"].map(String::from).to_vec()
        } else {
            Vec::new()
        };
        Scope {
            name: sanitize(name),
            pins: pins.into_iter().map(|p| (p, labels[p as usize].clone())).collect(),
            strings,
        }
    }

    /// A CAN bus's scope: one string variable with every frame.
    pub fn bus(name: &str) -> Scope {
        Scope { name: format!("bus_{}", sanitize(name)), pins: Vec::new(), strings: vec!["frames".into()] }
    }
}

fn sanitize(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect()
}

#[derive(Clone, Debug)]
enum Value {
    Bit(bool),
    Text(String),
}

struct Writer {
    out: Box<dyn Write + Send>,
    /// Identifier codes per (scope, pin).
    pin_ids: Vec<[Option<usize>; PINS]>,
    /// Identifier codes per (scope, string variable name).
    string_ids: Vec<BTreeMap<String, usize>>,
    /// Pending changes: (time in ns, sequence, variable, value).
    pending: Vec<(u64, u64, usize, Value)>,
    flush_at: usize,
    seq: u64,
    newest: u64,
    /// Everything up to this time has been written.
    written: u64,
    last_stamp: Option<u64>,
    last: Vec<Option<Value>>,
    /// Changes that arrived after their time had been written (placed at the earliest time still open).
    pub late: u64,
    error: Option<std::io::Error>,
}

/// VCD identifier code for variable `n`: printable ASCII `!`..`~`, base 94.
fn code(mut n: usize) -> String {
    let mut s = String::new();
    loop {
        s.push((33 + (n % 94) as u8) as char);
        n /= 94;
        if n == 0 {
            return s;
        }
        n -= 1;
    }
}

/// A string value as VCD carries it: no whitespace (written as `_`), printable ASCII.
fn vcd_string(text: &str) -> String {
    let s: String = text
        .chars()
        .map(|c| {
            if c.is_whitespace() {
                '_'
            } else if c.is_ascii_graphic() {
                c
            } else {
                '?'
            }
        })
        .collect();
    if s.is_empty() { "_".into() } else { s }
}

impl Writer {
    fn push(&mut self, t_ns: u64, var: usize, value: Value) {
        self.seq += 1;
        self.pending.push((t_ns, self.seq, var, value));
        self.newest = self.newest.max(t_ns);
        if self.pending.len() >= self.flush_at {
            self.flush_before(self.newest.saturating_sub(WINDOW_NS));
            // Sorting again only once as much has come in keeps this linear when the window holds a lot
            self.flush_at = FLUSH_AT.max(2 * self.pending.len());
        }
    }

    /// Writes the pending changes before `t` in time order.
    fn flush_before(&mut self, t: u64) {
        // Changes at the same time go by variable (then in the order they came), so boards running on several
        // threads, whose changes arrive in any order within a slice, still give the same file
        self.pending.sort_by_key(|p| (p.0, p.2, p.1));
        let n = self.pending.partition_point(|p| p.0 < t);
        let batch: Vec<_> = self.pending.drain(..n).collect();
        for (time, _, var, value) in batch {
            self.write_change(time, var, value);
        }
    }

    fn write_change(&mut self, time: u64, var: usize, value: Value) {
        if self.error.is_some() {
            return;
        }
        let time = if time < self.written {
            self.late += 1;
            self.written
        } else {
            time
        };
        let same = match (&self.last[var], &value) {
            (Some(Value::Bit(a)), Value::Bit(b)) => a == b,
            (Some(Value::Text(_)), Value::Text(_)) => false,
            _ => false,
        };
        if same {
            return;
        }
        let r = (|| -> std::io::Result<()> {
            if self.last_stamp != Some(time) {
                writeln!(self.out, "#{time}")?;
                self.last_stamp = Some(time);
            }
            match &value {
                Value::Bit(b) => writeln!(self.out, "{}{}", *b as u8, code(var)),
                Value::Text(s) => writeln!(self.out, "s{} {}", vcd_string(s), code(var)),
            }
        })();
        if let Err(e) = r {
            self.error = Some(e);
        }
        self.written = time;
        self.last[var] = Some(value);
    }
}

/// A VCD file being written; clone the handle to give it to several boards.
#[derive(Clone)]
pub struct Vcd(Arc<Mutex<Writer>>);

impl Vcd {
    /// Creates `path` with these scopes.
    pub fn create(path: &std::path::Path, scopes: &[Scope]) -> std::io::Result<Vcd> {
        let f = std::fs::File::create(path)?;
        Vcd::new(Box::new(std::io::BufWriter::with_capacity(1 << 16, f)), scopes)
    }

    /// Writes the header for `scopes` to `out`; values follow as the boards run.
    pub fn new(mut out: Box<dyn Write + Send>, scopes: &[Scope]) -> std::io::Result<Vcd> {
        writeln!(out, "$version firmware_simulator {} $end", env!("CARGO_PKG_VERSION"))?;
        writeln!(out, "$comment virtual time of the emulated boards $end")?;
        writeln!(out, "$timescale 1ns $end")?;
        let mut n = 0;
        let mut pin_ids = Vec::new();
        let mut string_ids = Vec::new();
        for s in scopes {
            writeln!(out, "$scope module {} $end", s.name)?;
            let mut pins = [None; PINS];
            for (p, label) in &s.pins {
                writeln!(out, "$var wire 1 {} {} $end", code(n), sanitize(label))?;
                pins[*p as usize] = Some(n);
                n += 1;
            }
            let mut strings = BTreeMap::new();
            for name in &s.strings {
                writeln!(out, "$var string 1 {} {} $end", code(n), sanitize(name))?;
                strings.insert(name.clone(), n);
                n += 1;
            }
            writeln!(out, "$upscope $end")?;
            pin_ids.push(pins);
            string_ids.push(strings);
        }
        writeln!(out, "$enddefinitions $end")?;
        // Every wire starts unknown until its board reports its level
        writeln!(out, "#0")?;
        writeln!(out, "$dumpvars")?;
        for v in 0..n {
            if pin_ids.iter().any(|p| p.contains(&Some(v))) {
                writeln!(out, "x{}", code(v))?;
            }
        }
        writeln!(out, "$end")?;
        Ok(Vcd(Arc::new(Mutex::new(Writer {
            out,
            pin_ids,
            string_ids,
            pending: Vec::new(),
            flush_at: FLUSH_AT,
            seq: 0,
            newest: 0,
            written: 0,
            last_stamp: Some(0),
            last: vec![None; n],
            late: 0,
            error: None,
        }))))
    }

    /// Records a pin level of scope `scope` at `t`.
    pub fn pin(&self, scope: usize, pin: u8, t: Time, level: bool) {
        let mut w = self.0.lock().unwrap();
        if let Some(var) = w.pin_ids.get(scope).and_then(|p| p.get(pin as usize).copied().flatten()) {
            w.push(t.as_ns(), var, Value::Bit(level));
        }
    }

    /// Records a string event on variable `name` of scope `scope` (ignored if the scope has no such variable).
    pub fn text(&self, scope: usize, name: &str, t: Time, text: &str) {
        let mut w = self.0.lock().unwrap();
        if let Some(&var) = w.string_ids.get(scope).and_then(|s| s.get(name)) {
            w.push(t.as_ns(), var, Value::Text(text.to_string()));
        }
    }

    /// Whether scope `scope` has string variable `name`.
    pub fn has_text(&self, scope: usize, name: &str) -> bool {
        self.0.lock().unwrap().string_ids.get(scope).is_some_and(|s| s.contains_key(name))
    }

    /// The pins scope `scope` records, as a bit mask.
    pub fn pin_mask(&self, scope: usize) -> u32 {
        let w = self.0.lock().unwrap();
        w.pin_ids.get(scope).map_or(0, |p| (0..PINS).filter(|&i| p[i].is_some()).fold(0, |m, i| m | 1 << i))
    }

    /// A trace sink for the machine of scope `scope`, which starts with its pins at `levels` at `now`.
    pub fn sink(&self, scope: usize, now: Time, levels: u32) -> Box<dyn TraceSink> {
        let mask = self.pin_mask(scope);
        for p in 0..PINS as u8 {
            if mask & (1 << p) != 0 {
                self.pin(scope, p, now, levels & (1 << p) != 0);
            }
        }
        Box::new(VcdSink { vcd: self.clone(), scope, mask, device: self.has_text(scope, "device") })
    }

    /// Records a session's pins (and, if the scope has them, its logs, UART lines, CAN frames and device reports)
    /// in scope `scope`, alongside whatever the session already traces and does with them.
    pub fn attach(&self, scope: usize, s: &mut Session) {
        let m = &mut s.machine;
        let old = std::mem::replace(&mut m.trace, Box::new(sim_core::trace::NullSink));
        let traces_gpio = old.wants(Category::Gpio);
        let sink = self.sink(scope, m.now, m.sys.gpio.level);
        m.set_trace(sim_core::trace::MultiSink::combine(old, sink));
        if !traces_gpio {
            // Only the recorded pins' PWM slices need their edges observed
            let mask = self.pin_mask(scope);
            let slices = (0..PINS).filter(|p| mask & (1 << p) != 0).fold(0u32, |a, p| a | 1 << ((p / 2) % 8));
            m.sys.pwm.observed = m.sys.pwm_device_slices | slices;
        }
        if self.has_text(scope, "log") {
            let v = self.clone();
            let mut prev = s.on_log.take();
            s.on_log = Some(Box::new(move |t, line| {
                if let Some(p) = &mut prev {
                    p(t, line);
                }
                v.text(scope, "log", t, &line.to_string());
            }));
            let v = self.clone();
            let mut prev = s.on_uart_line.take();
            s.on_uart_line = Some(Box::new(move |t, n, line| {
                if let Some(p) = &mut prev {
                    p(t, n, line);
                }
                v.text(scope, if n == 0 { "uart0" } else { "uart1" }, t, line);
            }));
        }
        if self.has_text(scope, "can")
            && let Some(bus) = &mut s.can_bus
        {
            bus.record.get_or_insert_with(Vec::new);
            let v = self.clone();
            let mut prev = s.on_can.take();
            s.on_can = Some(Box::new(move |r| {
                if let Some(p) = &mut prev {
                    p(r);
                }
                v.text(scope, "can", r.end, &can_text(r));
            }));
        }
    }

    /// Records every frame of a robot bus in the bus's scope (`frames`).
    pub fn attach_bus(&self, scope: usize, bus: &mut crate::robot::RobotBus) {
        bus.bus.record.get_or_insert_with(Vec::new);
        let v = self.clone();
        let mut prev = bus.on_can.take();
        bus.on_can = Some(Box::new(move |r| {
            if let Some(p) = &mut prev {
                p(r);
            }
            v.text(scope, "frames", r.end, &can_text(r));
        }));
    }

    /// Writes everything still pending and the final time, and flushes the file. Returns how many changes came in
    /// too late to be placed at their own time (0 normally).
    pub fn finish(&self, end: Time) -> std::io::Result<u64> {
        let mut w = self.0.lock().unwrap();
        w.flush_before(u64::MAX);
        let end = end.as_ns().max(w.written);
        if w.last_stamp != Some(end) {
            let r = writeln!(w.out, "#{end}");
            if let Err(e) = r {
                w.error.get_or_insert(e);
            }
            w.last_stamp = Some(end);
        }
        if let Some(e) = w.error.take() {
            return Err(e);
        }
        w.out.flush()?;
        Ok(w.late)
    }
}

/// A CAN frame as `candump` shows it, with errors and missing ACKs noted.
fn can_text(r: &sim_devices::can::BusRecord) -> String {
    let mut s = sim_devices::can::candump(&r.frame);
    if let Some(e) = &r.error {
        s.push_str(&format!(" (destroyed: {e:?})"));
    } else if !r.acked {
        s.push_str(" (no ACK)");
    }
    s
}

struct VcdSink {
    vcd: Vcd,
    scope: usize,
    mask: u32,
    device: bool,
}

impl TraceSink for VcdSink {
    fn wants(&self, category: Category) -> bool {
        category == Category::Gpio || (self.device && category == Category::Device)
    }

    fn event(&mut self, time: Time, event: TraceEvent) {
        match event {
            TraceEvent::Gpio { pin, level } if self.mask & (1 << pin) != 0 => {
                self.vcd.pin(self.scope, pin, time, level)
            }
            TraceEvent::Device { source, message } if self.device => {
                self.vcd.text(self.scope, "device", time, &format!("{source}: {message}"))
            }
            _ => {}
        }
    }
}

/// A parsed VCD file (what the tests read back): variables by scope and name, and every change.
#[derive(Clone, Debug, Default)]
pub struct Parsed {
    pub timescale: String,
    /// (scope, name) to identifier code.
    pub vars: BTreeMap<(String, String), String>,
    /// Changes per identifier code: (time, value), the value `0`/`1`/`x` for wires and the text for strings.
    pub changes: BTreeMap<String, Vec<(u64, String)>>,
}

impl Parsed {
    /// The changes of variable `name` in scope `scope`.
    pub fn signal(&self, scope: &str, name: &str) -> Option<&[(u64, String)]> {
        let code = self.vars.get(&(scope.to_string(), name.to_string()))?;
        Some(self.changes.get(code).map_or(&[][..], |v| v.as_slice()))
    }
}

/// Reads a VCD file (enough of the format for what [`Vcd`] writes and common tools produce). Fails on times
/// that go backwards.
pub fn parse(text: &str) -> Result<Parsed, String> {
    let mut p = Parsed::default();
    let mut scope: Vec<String> = Vec::new();
    let mut tokens = text.split_whitespace().peekable();
    let mut now: u64 = 0;
    let mut in_defs = true;
    while let Some(tok) = tokens.next() {
        if in_defs {
            match tok {
                "$timescale" => {
                    let mut ts = String::new();
                    for t in tokens.by_ref() {
                        if t == "$end" {
                            break;
                        }
                        ts.push_str(t);
                    }
                    p.timescale = ts;
                }
                "$scope" => {
                    let _kind = tokens.next();
                    scope.push(tokens.next().ok_or("bad $scope")?.to_string());
                    tokens.next();
                }
                "$upscope" => {
                    scope.pop();
                    tokens.next();
                }
                "$var" => {
                    let _kind = tokens.next();
                    let _size = tokens.next();
                    let code = tokens.next().ok_or("bad $var")?.to_string();
                    let name = tokens.next().ok_or("bad $var")?.to_string();
                    for t in tokens.by_ref() {
                        if t == "$end" {
                            break;
                        }
                    }
                    p.vars.insert((scope.join("."), name), code);
                }
                "$enddefinitions" => {
                    tokens.next();
                    in_defs = false;
                }
                _ if tok.starts_with('$') => {
                    for t in tokens.by_ref() {
                        if t == "$end" {
                            break;
                        }
                    }
                }
                _ => return Err(format!("unexpected {tok:?} in the header")),
            }
            continue;
        }
        match tok.as_bytes()[0] {
            b'#' => {
                let t: u64 = tok[1..].parse().map_err(|_| format!("bad time {tok}"))?;
                if t < now {
                    return Err(format!("time goes backwards: #{t} after #{now}"));
                }
                now = t;
            }
            b'$' => {}
            b'0' | b'1' | b'x' | b'z' | b'X' | b'Z' => {
                p.changes.entry(tok[1..].to_string()).or_default().push((now, tok[..1].to_string()));
            }
            b's' | b'r' | b'b' => {
                let code = tokens.next().ok_or("value without identifier")?;
                p.changes.entry(code.to_string()).or_default().push((now, tok[1..].to_string()));
            }
            _ => return Err(format!("unexpected {tok:?}")),
        }
    }
    Ok(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn named() -> BTreeMap<String, u8> {
        [("status_ledr_pin", 13), ("fault_led_pin", 13), ("can_int", 22)].map(|(k, v)| (k.to_string(), v)).into()
    }

    #[test]
    fn pin_lists() {
        assert_eq!(parse_pins("0-3,25", &named()).unwrap(), vec![0, 1, 2, 3, 25]);
        assert_eq!(parse_pins("status_ledr,can_int,13", &named()).unwrap(), vec![13, 22]);
        assert_eq!(parse_pins("all", &named()).unwrap().len(), 30);
        assert!(parse_pins("30", &named()).is_err());
        assert!(parse_pins("5-2", &named()).is_err());
        let e = parse_pins("nope", &named()).unwrap_err();
        assert!(e.contains("can_int"), "{e}");
    }

    #[test]
    fn labels_use_the_board_names() {
        let l = pin_labels(&named());
        assert_eq!(l[13], "gpio13_fault_led");
        assert_eq!(l[22], "gpio22_can_int");
        assert_eq!(l[0], "gpio0");
    }

    #[test]
    fn identifier_codes_are_unique() {
        let codes: std::collections::BTreeSet<String> = (0..20_000).map(code).collect();
        assert_eq!(codes.len(), 20_000);
        assert!(codes.iter().all(|c| c.bytes().all(|b| (33..127).contains(&b))));
    }

    #[derive(Clone, Default)]
    struct Shared(Arc<Mutex<Vec<u8>>>);
    impl Write for Shared {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn out_of_order_changes_are_sorted_and_repeats_dropped() {
        let buf = Shared::default();
        let opts = VcdOptions { pins: vec![1, 2], events: true };
        let scopes = [Scope::board("a", &BTreeMap::new(), &opts), Scope::board("b board", &BTreeMap::new(), &opts)];
        let v = Vcd::new(Box::new(buf.clone()), &scopes).unwrap();
        v.pin(0, 1, Time::from_ns(500), true);
        v.pin(1, 2, Time::from_ns(100), true);
        v.pin(0, 1, Time::from_ns(600), true); // no change
        v.pin(0, 1, Time::from_ns(700), false);
        v.pin(0, 5, Time::from_ns(700), false); // not recorded
        v.text(1, "log", Time::from_ns(300), "hello world");
        assert_eq!(v.finish(Time::from_us(1)).unwrap(), 0);
        let text = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
        let p = parse(&text).unwrap();
        assert_eq!(p.timescale, "1ns");
        let a1: Vec<(u64, String)> = p.signal("a", "gpio1").unwrap().to_vec();
        assert_eq!(a1, vec![(0, "x".into()), (500, "1".into()), (700, "0".into())]);
        assert_eq!(p.signal("b_board", "gpio2").unwrap().last().unwrap(), &(100, "1".to_string()));
        assert_eq!(p.signal("b_board", "log").unwrap(), &[(300, "hello_world".to_string())]);
        assert!(p.signal("a", "gpio5").is_none());
        assert!(text.trim_end().ends_with("#1000"), "{text}");
    }

    #[test]
    fn the_parser_rejects_time_going_backwards() {
        let bad = "$timescale 1ns $end $scope module a $end $var wire 1 ! x $end $upscope $end $enddefinitions $end #5 1! #3 0!";
        assert!(parse(bad).unwrap_err().contains("backwards"));
    }
}
