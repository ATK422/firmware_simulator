//! Branch edge coverage: a bitmap of the taken branches a core executed, AFL style, cheap enough to leave on while
//! the core runs batched (decoded blocks, busy-wait skipping), unlike instruction coverage.
//!
//! Every branch that the program takes (`B`, a taken `B<cond>`, `BL`, `BX`, `BLX`, `POP {..., pc}` and any other
//! write of the PC by an instruction) marks the bit of the hash of its own address and its target. Exception
//! entries and returns are left out: where they land depends on when an interrupt came, not on the path the
//! program took. A fuzzer compares maps to tell whether an input reached new code.
//!
//! A map can also watch address ranges: a branch landing in one sets its bit in [`EdgeMap::watched`] (the
//! firmware's `HardFault` handler, which spins on a branch to itself, for example).

/// Bits in the map (a power of two).
pub const EDGE_MAP_BITS: usize = 1 << 16;

/// Taken branches, by the hash of their source and target.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EdgeMap {
    words: Box<[u64]>,
    /// Address ranges `[lo, hi)` whose branch targets are watched, at most 32.
    pub watch: Vec<(u32, u32)>,
    /// Bit `k` set: a branch landed in `watch[k]`.
    pub watched: u32,
}

impl Default for EdgeMap {
    fn default() -> Self {
        EdgeMap::new()
    }
}

impl EdgeMap {
    pub fn new() -> EdgeMap {
        EdgeMap { words: vec![0; EDGE_MAP_BITS / 64].into_boxed_slice(), watch: Vec::new(), watched: 0 }
    }

    /// The bit of the branch from `from` to `to`.
    #[inline(always)]
    pub fn index(from: u32, to: u32) -> usize {
        let h = (from >> 1).wrapping_mul(0x9E37_79B1) ^ (to >> 1).wrapping_mul(0x85EB_CA77).rotate_left(13);
        (h >> (32 - EDGE_MAP_BITS.trailing_zeros())) as usize
    }

    /// Notes the taken branch from `from` to `to`.
    #[inline(always)]
    pub fn mark(&mut self, from: u32, to: u32) {
        let i = EdgeMap::index(from, to);
        self.words[i / 64] |= 1 << (i % 64);
        if !self.watch.is_empty() {
            self.check_watch(to);
        }
    }

    #[inline(never)]
    fn check_watch(&mut self, to: u32) {
        for (k, (lo, hi)) in self.watch.iter().enumerate().take(32) {
            if (*lo..*hi).contains(&to) {
                self.watched |= 1 << k;
            }
        }
    }

    /// Whether the branch from `from` to `to` was taken (or another with the same bit).
    pub fn hit(&self, from: u32, to: u32) -> bool {
        let i = EdgeMap::index(from, to);
        self.words[i / 64] & (1 << (i % 64)) != 0
    }

    /// Bits set.
    pub fn count(&self) -> usize {
        self.words.iter().map(|w| w.count_ones() as usize).sum()
    }

    /// Adds `other`'s bits (and watch hits).
    pub fn merge(&mut self, other: &EdgeMap) {
        for (a, b) in self.words.iter_mut().zip(other.words.iter()) {
            *a |= b;
        }
        self.watched |= other.watched;
    }

    /// Bits set in `self` that are not set in `seen`.
    pub fn new_bits(&self, seen: &EdgeMap) -> usize {
        self.words.iter().zip(seen.words.iter()).map(|(a, b)| (a & !b).count_ones() as usize).sum()
    }

    /// Clears the bits and the watch hits (the watched ranges stay).
    pub fn clear(&mut self) {
        self.words.iter_mut().for_each(|w| *w = 0);
        self.watched = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marks_merges_and_counts_new_bits() {
        let mut a = EdgeMap::new();
        a.mark(0x1000_0100, 0x1000_0200);
        assert!(a.hit(0x1000_0100, 0x1000_0200));
        assert!(!a.hit(0x1000_0200, 0x1000_0100), "edges have a direction");
        assert_eq!(a.count(), 1);
        let mut b = EdgeMap::new();
        b.mark(0x1000_0100, 0x1000_0200);
        b.mark(0x1000_0300, 0x1000_0100);
        assert_eq!(b.new_bits(&a), 1);
        a.merge(&b);
        assert_eq!(a.count(), 2);
        assert_eq!(b.new_bits(&a), 0);
        a.clear();
        assert_eq!(a.count(), 0);
    }

    #[test]
    fn watched_ranges() {
        let mut m = EdgeMap::new();
        m.watch = vec![(0x1000_0400, 0x1000_0410), (0x2000_0000, 0x2000_0004)];
        m.mark(0x1000_0000, 0x1000_0410);
        assert_eq!(m.watched, 0);
        m.mark(0x1000_0400, 0x1000_0400);
        assert_eq!(m.watched, 1);
        m.mark(0x1000_0000, 0x2000_0002);
        assert_eq!(m.watched, 3);
    }
}
