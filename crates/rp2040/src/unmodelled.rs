//! Fallback for registers the emulator does not model yet.
//!
//! Every access is reported (named from the SVD) so silent divergence from hardware shows up in the log. Writes are
//! stored, honouring the atomic XOR/SET/CLR aliases, and reads return the stored value or the SVD reset value, so
//! firmware that only configures a block and never depends on its behaviour keeps running.

use std::collections::{BTreeMap, HashMap};

use sim_core::bus::WriteMode;
use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, TraceSink};

use crate::svd;

/// How unmodelled accesses are logged (they are always traced to the `Unmodelled` trace category).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum LogMode {
    /// A warning the first time each register is touched, plus a summary on request.
    #[default]
    FirstAccess,
    /// A warning for every access.
    Every,
    /// Nothing in the log (still counted).
    Off,
}

#[derive(Clone, Default, Debug)]
pub struct Unmodelled {
    values: HashMap<u32, u32>,
    /// Access counts per register address: (reads, writes).
    counts: BTreeMap<u32, (u64, u64)>,
    pub log_mode: LogMode,
}

/// The register an address refers to, with alias bits removed where the block has aliases.
fn canonical(addr: u32) -> u32 {
    let word = addr & !3;
    match svd::peripheral_at(word) {
        Some(p) if svd::has_atomic_aliases(p.base) => p.base + ((word - p.base) & 0xFFF),
        _ => word,
    }
}

fn alias_mode(addr: u32) -> WriteMode {
    match svd::peripheral_at(addr) {
        Some(p) if svd::has_atomic_aliases(p.base) => match (addr - p.base) >> 12 & 3 {
            1 => WriteMode::Xor,
            2 => WriteMode::Set,
            3 => WriteMode::Clear,
            _ => WriteMode::Normal,
        },
        _ => WriteMode::Normal,
    }
}

impl Unmodelled {
    fn report(&mut self, trace: &mut dyn TraceSink, now: Time, core: u8, write: bool, addr: u32, value: u32) {
        let reg = canonical(addr);
        let entry = self.counts.entry(reg).or_default();
        let first = entry.0 + entry.1 == 0;
        if write {
            entry.1 += 1;
        } else {
            entry.0 += 1;
        }
        let log_it = match self.log_mode {
            LogMode::FirstAccess => first,
            LogMode::Every => true,
            LogMode::Off => false,
        };
        if log_it || trace.wants(Category::Unmodelled) {
            let name = svd::describe(addr);
            if log_it {
                let dir = if write { "write" } else { "read" };
                log::warn!("[{now}] core{core} unmodelled {dir} {name} ({addr:#010x}) = {value:#010x}");
            }
            if trace.wants(Category::Unmodelled) {
                trace.event(now, TraceEvent::Unmodelled { core, write, addr, value, name });
            }
        }
    }

    /// Reads a word from an unmodelled register.
    pub fn read(&mut self, trace: &mut dyn TraceSink, now: Time, core: u8, addr: u32) -> u32 {
        let reg = canonical(addr);
        let value = *self.values.get(&reg).unwrap_or(&svd::reset_value(reg));
        self.report(trace, now, core, false, addr, value);
        value
    }

    /// Writes a word to an unmodelled register through whichever alias `addr` names.
    pub fn write(&mut self, trace: &mut dyn TraceSink, now: Time, core: u8, addr: u32, value: u32) {
        let reg = canonical(addr);
        let old = *self.values.get(&reg).unwrap_or(&svd::reset_value(reg));
        let new = alias_mode(addr).apply(old, value);
        self.values.insert(reg, new);
        self.report(trace, now, core, true, addr, value);
    }

    /// Value currently held, without counting an access.
    pub fn peek(&self, addr: u32) -> u32 {
        let reg = canonical(addr);
        *self.values.get(&reg).unwrap_or(&svd::reset_value(reg))
    }

    /// `(register name, reads, writes)` for every unmodelled register touched so far.
    pub fn summary(&self) -> Vec<(String, u64, u64)> {
        self.counts.iter().map(|(&a, &(r, w))| (svd::describe(a), r, w)).collect()
    }

    /// Forgets stored values of the block at `base` (it was reset through RESETS).
    pub fn reset_block(&mut self, base: u32) {
        let size = if base >= 0x5000_0000 { 0x10_0000 } else { 0x4000 };
        self.values.retain(|&a, _| !(base..base + size).contains(&a));
    }

    /// Forgets stored values (a chip reset). Access counts are kept.
    pub fn reset(&mut self) {
        self.values.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::trace::{NullSink, VecSink};

    #[test]
    fn stores_with_aliases_and_reset_values() {
        let mut u = Unmodelled { log_mode: LogMode::Off, ..Default::default() };
        let mut t = NullSink;
        // UART0.UARTIBRD resets to 0
        let ibrd = 0x4003_4024;
        assert_eq!(u.read(&mut t, Time::ZERO, 0, ibrd), 0);
        u.write(&mut t, Time::ZERO, 0, ibrd, 0x00F0);
        u.write(&mut t, Time::ZERO, 0, ibrd + 0x2000, 0x000F); // SET
        assert_eq!(u.read(&mut t, Time::ZERO, 0, ibrd), 0x00FF);
        u.write(&mut t, Time::ZERO, 0, ibrd + 0x3000, 0x0011); // CLR
        assert_eq!(u.peek(ibrd), 0x00EE);
        u.write(&mut t, Time::ZERO, 0, ibrd + 0x1000, 0x00FF); // XOR
        assert_eq!(u.peek(ibrd), 0x0011);
        // A register with a non-zero reset value: UART0.UARTCR resets to 0x300
        assert_eq!(u.read(&mut t, Time::ZERO, 0, 0x4003_4030), 0x300);
        let s = u.summary();
        assert!(s.iter().any(|(n, r, w)| n == "UART0.UARTIBRD" && *r == 2 && *w == 4), "{s:?}");
    }

    #[test]
    fn traces_named_accesses() {
        let mut u = Unmodelled { log_mode: LogMode::Off, ..Default::default() };
        let sink = VecSink::new(&[Category::Unmodelled]);
        let mut t = sink.clone();
        u.write(&mut t, Time::ZERO, 1, 0x4003_C000, 7);
        let ev = sink.take();
        assert_eq!(ev[0].1.to_string(), "core1 unmodelled write SPI0.SSPCR0 (4003c000) = 00000007");
    }
}
