//! Oracles: what counts as a finding after a fuzz case has run, and how to tell one finding from another.
//!
//! A finding is behaviour the firmware should not show in response to bus traffic alone: a panic, a HardFault or
//! lockup, a watchdog reset, a hang (the board stops answering after the traffic stops), a fault bit raised and
//! never cleared, an unmodelled-register access that did not happen on a clean boot (often a wild pointer), or an
//! ERROR log line. Each has a short, stable [`Finding::key`] so repeated hits of the same bug collapse to one.

use std::collections::BTreeSet;

use sim_core::time::Time;

/// What a clean boot of the board looks like, captured once at the snapshot the cases branch from.
#[derive(Clone, Debug)]
pub struct Baseline {
    /// Fault bits set in the last status before fuzzing (normally 0).
    pub faults: u32,
    /// Unmodelled registers touched during boot (by name): only new ones are interesting.
    pub unmodelled: BTreeSet<String>,
    pub resets: u32,
    pub lockups: u64,
    /// Whether the board was answering link heartbeats before fuzzing.
    pub was_connected: bool,
    /// The board's fault bit names (`FAULT_*`), for reporting.
    pub fault_names: Vec<String>,
}

/// A kind of finding. Ordered by how serious it is, worst first.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// A firmware panic (panic handler, crash record).
    Panic,
    /// A core locked up (a fault inside a fault handler, a branch to a bad address).
    Lockup,
    /// The watchdog reset the chip.
    WatchdogReset,
    /// The board stopped answering after the traffic stopped (a hang that the watchdog did not catch).
    Hang,
    /// A fault bit was raised and never cleared.
    FaultLatched,
    /// The firmware touched a register the emulator does not model and a clean boot did not (a wild pointer, or a
    /// gap in the model: investigate).
    UnmodelledAccess,
    /// An ERROR-level log line.
    ErrorLog,
}

impl Kind {
    pub fn word(self) -> &'static str {
        match self {
            Kind::Panic => "panic",
            Kind::Lockup => "lockup",
            Kind::WatchdogReset => "watchdog_reset",
            Kind::Hang => "hang",
            Kind::FaultLatched => "fault_latched",
            Kind::UnmodelledAccess => "unmodelled_access",
            Kind::ErrorLog => "error_log",
        }
    }
}

/// One thing an oracle noticed.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Finding {
    pub kind: Kind,
    /// A short, stable identifier so the same bug from different inputs collapses to one (the kind plus the fault
    /// name, the register, or the gist of the log line).
    pub key: String,
    /// A human-readable line.
    pub detail: String,
    #[serde(serialize_with = "ser_time")]
    pub at: Option<Time>,
}

fn ser_time<S: serde::Serializer>(t: &Option<Time>, s: S) -> Result<S::Ok, S::Error> {
    match t {
        Some(t) => s.serialize_f64(t.as_secs_f64()),
        None => s.serialize_none(),
    }
}

/// What a run left behind, for the oracles to judge against the baseline.
pub struct Outcome<'a> {
    /// `(time, level, message)` of every log line during and after the fuzz window.
    pub logs: &'a [(Time, Option<String>, String)],
    pub faults: u32,
    pub unmodelled: &'a BTreeSet<String>,
    pub resets: u32,
    pub lockups: u64,
    /// Whether the board answered a link heartbeat in the settle window after the traffic stopped.
    pub answered_after: bool,
    /// Whether a core is locked up right now.
    pub locked_up_now: bool,
}

fn fault_list(names: &[String], bits: u32) -> String {
    let mut out = Vec::new();
    for b in 0..32 {
        if bits & (1 << b) != 0 {
            out.push(names.get(b).cloned().unwrap_or_else(|| format!("bit{b}")));
        }
    }
    out.join("|")
}

/// Every finding in `o` versus `base`. Deduplicated by key, worst kind first.
pub fn check(base: &Baseline, o: &Outcome) -> Vec<Finding> {
    let mut out: Vec<Finding> = Vec::new();
    let mut push = |f: Finding| {
        if !out.iter().any(|g| g.key == f.key) {
            out.push(f);
        }
    };

    // Panic / crash record / HardFault, by log line
    for (t, _level, msg) in o.logs {
        let m = msg.to_ascii_uppercase();
        if m.contains("PANIC") || msg.contains("Reset by panic") || m.contains("HARDFAULT") {
            push(Finding { kind: Kind::Panic, key: "panic".into(), detail: msg.clone(), at: Some(*t) });
        }
        if msg.contains("Watchdog timeout") {
            push(Finding { kind: Kind::WatchdogReset, key: "watchdog_log".into(), detail: msg.clone(), at: Some(*t) });
        }
    }

    if o.lockups > base.lockups || o.locked_up_now {
        push(Finding {
            kind: Kind::Lockup,
            key: "lockup".into(),
            detail: format!("{} lockup(s) (was {})", o.lockups, base.lockups),
            at: None,
        });
    }

    if o.resets > base.resets {
        push(Finding {
            kind: Kind::WatchdogReset,
            key: "reset".into(),
            detail: format!("{} chip reset(s) (was {})", o.resets, base.resets),
            at: None,
        });
    }

    // A fault bit raised that the baseline did not have and that is still set at the end
    // One finding per bit, so a case that latches two faults is not a third, combined finding
    let new_faults = o.faults & !base.faults;
    for b in (0..32).filter(|b| new_faults & (1 << b) != 0) {
        let name = fault_list(&base.fault_names, 1 << b);
        push(Finding {
            kind: Kind::FaultLatched,
            key: format!("fault:{name}"),
            detail: format!("{name} raised and still set at the end (faults {:#x})", o.faults),
            at: None,
        });
    }

    // The board went quiet (connected before, no heartbeat after the traffic stopped) without a reset explaining
    // it: a hang the watchdog did not catch
    if base.was_connected && !o.answered_after && o.resets == base.resets && !o.locked_up_now {
        push(Finding {
            kind: Kind::Hang,
            key: "hang".into(),
            detail: "no link heartbeat after the traffic stopped".into(),
            at: None,
        });
    }

    for reg in o.unmodelled.difference(&base.unmodelled) {
        push(Finding {
            kind: Kind::UnmodelledAccess,
            key: format!("unmodelled:{reg}"),
            detail: format!("touched {reg}, not touched on a clean boot"),
            at: None,
        });
    }

    for (t, level, msg) in o.logs {
        // defmt levels come as `error`, `warn`, ... (defmt_decoder's `Level::as_str`)
        if level.as_deref().is_some_and(|l| l.eq_ignore_ascii_case("error")) {
            // Collapse the numbers out of the key so the same message with different arguments is one finding
            push(Finding {
                kind: Kind::ErrorLog,
                key: format!("error:{}", gist(msg)),
                detail: msg.clone(),
                at: Some(*t),
            });
        }
    }

    out.sort_by_key(|f| f.kind);
    out
}

/// The gist of a log line: its text with every number (a run starting with a digit: `172`, `0x3a981`) replaced
/// by `#`, so the same message with other arguments collapses to one key. Words stay as they are.
pub fn gist(msg: &str) -> String {
    let mut out = String::new();
    let mut chars = msg.chars().peekable();
    while let Some(c) = chars.next() {
        if c.is_ascii_digit() {
            while chars.peek().is_some_and(|n| n.is_ascii_hexdigit() || *n == 'x' || *n == 'X') {
                chars.next();
            }
            out.push('#');
        } else {
            out.push(c);
        }
    }
    out
}

/// A regex (Rust syntax) matching a log line with `msg`'s gist: the text escaped, each number any number.
pub fn gist_regex(msg: &str) -> String {
    gist(msg).split('#').map(regex::escape).collect::<Vec<_>>().join("[0-9A-Fa-fxX]+")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> Baseline {
        Baseline {
            faults: 0,
            unmodelled: ["UART0.UARTIBRD".to_string()].into_iter().collect(),
            resets: 0,
            lockups: 0,
            was_connected: true,
            fault_names: vec![
                "FAULT_WATCHDOG_RESET".into(),
                "FAULT_WATCHDOG_WARNING".into(),
                "FAULT_CAN_INTERNAL_ERROR".into(),
            ],
        }
    }

    #[test]
    fn a_clean_run_finds_nothing() {
        let o = Outcome {
            logs: &[(Time::ZERO, Some("INFO".into()), "all well".into())],
            faults: 0,
            unmodelled: &base().unmodelled,
            resets: 0,
            lockups: 0,
            answered_after: true,
            locked_up_now: false,
        };
        assert!(check(&base(), &o).is_empty());
    }

    #[test]
    fn each_oracle_fires_and_dedups() {
        let unmodelled: BTreeSet<String> =
            ["UART0.UARTIBRD".to_string(), "SPI0.SSPCR0".to_string()].into_iter().collect();
        let o = Outcome {
            logs: &[
                (Time::from_ms(1), Some("error".into()), "PANIC: wild".into()),
                (Time::from_ms(2), Some("error".into()), "Fault raised arg 0x12".into()),
                (Time::from_ms(3), Some("error".into()), "Fault raised arg 0x34".into()),
            ],
            faults: 0b1100,
            unmodelled: &unmodelled,
            resets: 1,
            lockups: 0,
            answered_after: false,
            locked_up_now: false,
        };
        let fs = check(&base(), &o);
        let kinds: Vec<Kind> = fs.iter().map(|f| f.kind).collect();
        assert!(kinds.contains(&Kind::Panic));
        assert!(kinds.contains(&Kind::WatchdogReset));
        // Two latched bits are two findings, not one combined key
        assert_eq!(fs.iter().filter(|f| f.kind == Kind::FaultLatched).count(), 2);
        assert!(kinds.contains(&Kind::UnmodelledAccess));
        // The two "Fault raised arg 0x.." ERROR lines collapse to one key (plus the PANIC one)
        assert_eq!(fs.iter().filter(|f| f.kind == Kind::ErrorLog).count(), 2);
        // A reset explains the silence, so no separate hang finding
        assert!(!kinds.contains(&Kind::Hang));
        // Panic sorts before the rest
        assert_eq!(fs[0].kind, Kind::Panic);
    }

    #[test]
    fn hang_only_without_a_reset() {
        let o = Outcome {
            logs: &[],
            faults: 0,
            unmodelled: &base().unmodelled,
            resets: 0,
            lockups: 0,
            answered_after: false,
            locked_up_now: false,
        };
        assert_eq!(check(&base(), &o).iter().filter(|f| f.kind == Kind::Hang).count(), 1);
    }

    #[test]
    fn gist_collapses_numbers() {
        assert_eq!(gist("Fault raised arg 0x12"), gist("Fault raised arg 0x9ab"));
        assert_ne!(gist("Depth error"), gist("Comm error"));
        // Words with hex letters stay words; only numbers are masked
        assert_eq!(
            gist("Bridge speaks titan_link v172, firmware speaks v2"),
            "Bridge speaks titan_link v#, firmware speaks v#"
        );
        let re = regex::Regex::new(&gist_regex("Bridge speaks v172 (arg 0x3a981)")).unwrap();
        assert!(re.is_match("Bridge speaks v9 (arg 0xff)"));
        assert!(!re.is_match("Bridge speaks v (arg 0xff)"));
    }
}
