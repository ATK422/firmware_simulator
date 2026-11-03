//! Keyed event scheduler.
//!
//! Peripherals never poll. Each one schedules the next moment something happens to it (a timer alarm, an SPI byte
//! finishing) under a key it owns. Scheduling a key that is already pending moves it; [`Scheduler::cancel`] removes
//! it. Events at the same time fire in the order they were scheduled, which keeps runs deterministic.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::hash::{BuildHasherDefault, Hash, Hasher};

/// FNV-1a: the scheduler's keys are small, and SipHash's DoS resistance buys nothing here.
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

type FnvMap<K, V> = HashMap<K, V, BuildHasherDefault<Fnv>>;

use crate::time::Time;

#[derive(Clone, Debug)]
pub struct Scheduler<K: Copy + Eq + Hash> {
    heap: BinaryHeap<Reverse<(Time, u64, KeyOrd<K>)>>,
    pending: FnvMap<K, (Time, u64)>,
    seq: u64,
}

/// Wrapper so keys need no `Ord`; heap ordering only ever looks at `(time, seq)`, which is unique.
#[derive(Clone, Copy, Debug)]
struct KeyOrd<K>(K);
impl<K> PartialEq for KeyOrd<K> {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}
impl<K> Eq for KeyOrd<K> {}
impl<K> PartialOrd for KeyOrd<K> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl<K> Ord for KeyOrd<K> {
    fn cmp(&self, _: &Self) -> std::cmp::Ordering {
        std::cmp::Ordering::Equal
    }
}

impl<K: Copy + Eq + Hash> Default for Scheduler<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Copy + Eq + Hash> Scheduler<K> {
    pub fn new() -> Self {
        Scheduler { heap: BinaryHeap::new(), pending: FnvMap::default(), seq: 0 }
    }

    /// Schedules `key` at `time`, replacing any pending event with the same key.
    pub fn schedule(&mut self, key: K, time: Time) {
        self.seq += 1;
        self.pending.insert(key, (time, self.seq));
        self.heap.push(Reverse((time, self.seq, KeyOrd(key))));
    }

    /// Removes a pending event. Returns whether one was pending.
    pub fn cancel(&mut self, key: K) -> bool {
        self.pending.remove(&key).is_some()
    }

    /// When `key` is due, if pending.
    pub fn due(&self, key: K) -> Option<Time> {
        self.pending.get(&key).map(|&(t, _)| t)
    }

    pub fn is_pending(&self, key: K) -> bool {
        self.pending.contains_key(&key)
    }

    fn discard_stale(&mut self) {
        while let Some(Reverse((t, seq, KeyOrd(k)))) = self.heap.peek() {
            match self.pending.get(k) {
                Some(&(pt, pseq)) if pt == *t && pseq == *seq => return,
                _ => {
                    self.heap.pop();
                }
            }
        }
    }

    /// Time of the earliest pending event, or [`Time::MAX`] if there is none.
    pub fn next_time(&mut self) -> Time {
        self.discard_stale();
        self.heap.peek().map(|Reverse((t, _, _))| *t).unwrap_or(Time::MAX)
    }

    /// Removes and returns the earliest event if it is due at or before `now`.
    pub fn pop_due(&mut self, now: Time) -> Option<(Time, K)> {
        self.discard_stale();
        match self.heap.peek() {
            Some(Reverse((t, _, _))) if *t <= now => {
                let Reverse((t, _, KeyOrd(k))) = self.heap.pop().unwrap();
                self.pending.remove(&k);
                Some((t, k))
            }
            _ => None,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    pub fn len(&self) -> usize {
        self.pending.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
