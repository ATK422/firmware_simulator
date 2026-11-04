//! Code coverage: which instructions the cores executed, one bit per halfword of ROM, flash (through any XIP alias)
//! and SRAM. [`Machine::coverage`](crate::Machine::coverage) turns it on; while it is on the cores run one
//! instruction at a time (as with a breakpoint set), so every instruction is seen, including those of busy-waits
//! that a batched run would skip over (they ran in the reference semantics). The run is otherwise the same.

use crate::memmap::{ROM_SIZE, SRAM_BASE, SRAM_SIZE};

/// Executed instructions, a bit per halfword.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CoverageMap {
    rom: Vec<u64>,
    flash: Vec<u64>,
    sram: Vec<u64>,
    flash_size: usize,
}

fn bits(bytes: usize) -> Vec<u64> {
    vec![0; (bytes / 2).div_ceil(64)]
}

impl CoverageMap {
    /// An empty map for a flash of `flash_size` bytes.
    pub fn new(flash_size: usize) -> CoverageMap {
        CoverageMap { rom: bits(ROM_SIZE), flash: bits(flash_size), sram: bits(SRAM_SIZE), flash_size }
    }

    /// The bitmap and the index of `addr`'s halfword in it.
    fn slot(&self, addr: u32) -> Option<(&[u64], usize)> {
        match addr >> 28 {
            0x0 if (addr as usize) < ROM_SIZE => Some((&self.rom, addr as usize / 2)),
            // Every XIP alias reads the same flash
            0x1 if (0x10..=0x13).contains(&(addr >> 24)) => {
                let off = (addr - (addr & 0xFF00_0000)) as usize;
                (off < self.flash_size).then_some((&self.flash, off / 2))
            }
            0x2 if addr >= SRAM_BASE && ((addr - SRAM_BASE) as usize) < SRAM_SIZE => {
                Some((&self.sram, (addr - SRAM_BASE) as usize / 2))
            }
            _ => None,
        }
    }

    /// Notes that the instruction at `addr` ran.
    #[inline]
    pub fn mark(&mut self, addr: u32) {
        let (map, i) = match addr >> 28 {
            0x0 if (addr as usize) < ROM_SIZE => (&mut self.rom, addr as usize / 2),
            0x1 if (0x10..=0x13).contains(&(addr >> 24)) => {
                let off = (addr & 0x00FF_FFFF) as usize;
                if off >= self.flash_size {
                    return;
                }
                (&mut self.flash, off / 2)
            }
            0x2 if addr >= SRAM_BASE && ((addr - SRAM_BASE) as usize) < SRAM_SIZE => {
                (&mut self.sram, (addr - SRAM_BASE) as usize / 2)
            }
            _ => return,
        };
        map[i / 64] |= 1 << (i % 64);
    }

    /// Whether an instruction at `addr` ran (flash addresses through any XIP alias).
    pub fn hit(&self, addr: u32) -> bool {
        self.slot(addr).is_some_and(|(map, i)| map[i / 64] & (1 << (i % 64)) != 0)
    }

    /// Whether any instruction in `addr..addr + len` ran.
    pub fn any(&self, addr: u32, len: u32) -> bool {
        (0..len.div_ceil(2)).any(|i| self.hit(addr.wrapping_add(2 * i)))
    }

    /// Adds what `other` saw (another board running the same image, another run).
    pub fn merge(&mut self, other: &CoverageMap) {
        for (a, b) in [(&mut self.rom, &other.rom), (&mut self.flash, &other.flash), (&mut self.sram, &other.sram)] {
            if a.len() < b.len() {
                a.resize(b.len(), 0);
            }
            for (x, y) in a.iter_mut().zip(b) {
                *x |= y;
            }
        }
        self.flash_size = self.flash_size.max(other.flash_size);
    }

    /// Halfwords marked.
    pub fn count(&self) -> usize {
        [&self.rom, &self.flash, &self.sram].iter().flat_map(|m| m.iter()).map(|w| w.count_ones() as usize).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marks_by_halfword_through_every_alias() {
        let mut c = CoverageMap::new(2 << 20);
        c.mark(0x1000_0100);
        assert!(c.hit(0x1000_0100) && c.hit(0x1100_0100) && c.hit(0x1300_0101));
        assert!(!c.hit(0x1000_0102));
        c.mark(0x2000_0010);
        c.mark(0x0000_00f0);
        c.mark(0x5000_0000);
        c.mark(0x1020_0000);
        assert_eq!(c.count(), 3);
        assert!(c.any(0x2000_000c, 6) && !c.any(0x2000_0000, 16));
        let mut d = CoverageMap::new(2 << 20);
        d.mark(0x1000_0200);
        d.merge(&c);
        assert_eq!(d.count(), 4);
    }
}
