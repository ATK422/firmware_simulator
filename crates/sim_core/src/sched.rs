//! Keyed event scheduler.
//!
//! Peripherals never poll. Each one schedules the next moment something happens to it (a timer alarm, an SPI byte
//! finishing) under a key it owns. Scheduling a key that is already pending moves it; [`Scheduler::cancel`] removes
//! it. Events at the same time fire in the order they were scheduled, which keeps runs deterministic.

use std::hash::{Hash, Hasher};

/// FNV-1a: a small, fast hash for short keys (also used to fingerprint runs).
#[derive(Default)]
pub struct Fnv(u64);

impl Hasher for Fnv {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        let mut h = if self.0 == 0 { 0xcbf2_9ce4_8422_2325 } else { self.0 };
        for &b in bytes {
            h = (h ^ b as u64).wrapping_mul(0x100_0000_01b3);
        }
        self.0 = h;
    }
}

use crate::time::Time;

/// The pending events, one per key, ordered by `(time, seq)` where `seq` counts calls to
/// [`Scheduler::schedule`].
///
/// A machine has a handful of keys (one per peripheral activity), so the events are kept in a small unsorted list
/// with the earliest one remembered: scheduling is a short scan, and the run loop's frequent "when is the next
/// event" is a field read. (A heap with a key map cost more than that here, mostly in the stale heap entries left by
/// keys that are moved on every access.)
#[derive(Clone, Debug)]
pub struct Scheduler<K: Copy + Eq + Hash> {
    events: Vec<Entry<K>>,
    /// Index of the earliest event in `events`, if known.
    earliest: Option<usize>,
    seq: u64,
}

#[derive(Clone, Copy, Debug)]
struct Entry<K> {
    time: Time,
    seq: u64,
    key: K,
}

impl<K> Entry<K> {
    fn order(&self) -> (Time, u64) {
        (self.time, self.seq)
    }
}

impl<K: Copy + Eq + Hash> Default for Scheduler<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Copy + Eq + Hash> Scheduler<K> {
    pub fn new() -> Self {
        Scheduler { events: Vec::new(), earliest: None, seq: 0 }
    }

    /// Schedules `key` at `time`, replacing any pending event with the same key.
    pub fn schedule(&mut self, key: K, time: Time) {
        self.seq += 1;
        let entry = Entry { time, seq: self.seq, key };
        let i = match self.events.iter().position(|e| e.key == key) {
            Some(i) => {
                self.events[i] = entry;
                if self.earliest == Some(i) {
                    // It moved, possibly later: find the earliest again when asked
                    self.earliest = None;
                    return;
                }
                i
            }
            None => {
                self.events.push(entry);
                self.events.len() - 1
            }
        };
        if let Some(e) = self.earliest
            && entry.order() < self.events[e].order()
        {
            self.earliest = Some(i);
        }
    }

    /// Removes a pending event. Returns whether one was pending.
    pub fn cancel(&mut self, key: K) -> bool {
        match self.events.iter().position(|e| e.key == key) {
            Some(i) => {
                self.remove(i);
                true
            }
            None => false,
        }
    }

    fn remove(&mut self, i: usize) {
        self.events.swap_remove(i);
        self.earliest = None;
    }

    /// When `key` is due, if pending.
    pub fn due(&self, key: K) -> Option<Time> {
        self.events.iter().find(|e| e.key == key).map(|e| e.time)
    }

    pub fn is_pending(&self, key: K) -> bool {
        self.events.iter().any(|e| e.key == key)
    }

    fn earliest(&mut self) -> Option<usize> {
        if self.earliest.is_none() {
            self.earliest = (0..self.events.len()).min_by_key(|&i| self.events[i].order());
        }
        self.earliest
    }

    /// Time of the earliest pending event, or [`Time::MAX`] if there is none.
    pub fn next_time(&mut self) -> Time {
        match self.earliest() {
            Some(i) => self.events[i].time,
            None => Time::MAX,
        }
    }

    /// Removes and returns the earliest event if it is due at or before `now`.
    pub fn pop_due(&mut self, now: Time) -> Option<(Time, K)> {
        let i = self.earliest()?;
        let e = self.events[i];
        if e.time > now {
            return None;
        }
        self.remove(i);
        Some((e.time, e.key))
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ordering the scheduler must keep: by time, then by when the key was last scheduled.
    #[test]
    fn matches_a_reference_model() {
        let mut s = Scheduler::new();
        // Reference: key -> (time, seq)
        let mut model: std::collections::HashMap<u8, (u64, u64)> = Default::default();
        let mut seq = 0;
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let mut now = 0u64;
        for _ in 0..20_000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let key = (x % 12) as u8;
            match (x >> 8) % 8 {
                0..=3 => {
                    // Often the same time as others, to exercise the tie-break
                    let t = now + (x >> 16) % 4 * 10;
                    seq += 1;
                    model.insert(key, (t, seq));
                    s.schedule(key, Time(t));
                }
                4 => assert_eq!(s.cancel(key), model.remove(&key).is_some()),
                5 => assert_eq!(s.due(key), model.get(&key).map(|&(t, _)| Time(t))),
                _ => {
                    now += (x >> 20) % 15;
                    loop {
                        let expect =
                            model.iter().min_by_key(|(_, v)| **v).filter(|(_, v)| v.0 <= now).map(|(k, v)| (*k, v.0));
                        let got = s.pop_due(Time(now)).map(|(t, k)| (k, t.0));
                        assert_eq!(got, expect);
                        match got {
                            Some((k, _)) => {
                                model.remove(&k);
                            }
                            None => break,
                        }
                    }
                    let next = model.values().min().map_or(Time::MAX, |v| Time(v.0));
                    assert_eq!(s.next_time(), next);
                }
            }
            assert_eq!(s.len(), model.len());
        }
    }

    #[test]
    fn order_and_replace() {
        let mut s = Scheduler::new();
        s.schedule('a', Time(30));
        s.schedule('b', Time(10));
        s.schedule('c', Time(20));
        s.schedule('a', Time(5)); // moves a
        assert_eq!(s.next_time(), Time(5));
        assert_eq!(s.pop_due(Time(100)), Some((Time(5), 'a')));
        assert_eq!(s.pop_due(Time(100)), Some((Time(10), 'b')));
        assert_eq!(s.pop_due(Time(15)), None);
        assert_eq!(s.pop_due(Time(20)), Some((Time(20), 'c')));
        assert_eq!(s.pop_due(Time(100)), None);
        assert!(s.is_empty());
    }

    #[test]
    fn cancel() {
        let mut s = Scheduler::new();
        s.schedule(1, Time(10));
        s.schedule(2, Time(20));
        assert!(s.cancel(1));
        assert!(!s.cancel(1));
        assert_eq!(s.next_time(), Time(20));
        assert_eq!(s.due(2), Some(Time(20)));
        assert_eq!(s.due(1), None);
    }

    #[test]
    fn same_time_fifo() {
        let mut s = Scheduler::new();
        for k in 0..10 {
            s.schedule(k, Time(7));
        }
        let order: Vec<_> = std::iter::from_fn(|| s.pop_due(Time(7)).map(|(_, k)| k)).collect();
        assert_eq!(order, (0..10).collect::<Vec<_>>());
    }

    #[test]
    fn empty_is_max() {
        let mut s: Scheduler<u8> = Scheduler::new();
        assert_eq!(s.next_time(), Time::MAX);
    }
}
