//! SEGGER RTT host side: finds the control block and drains the up (target to host) channels, as a debug probe
//! would.
//!
//! Control block layout: `"SEGGER RTT"` id (16 bytes), number of up and down channels, then one 24-byte descriptor
//! per channel: name pointer, buffer pointer, size, write offset, read offset, flags.

use crate::target::TargetMemory;

const ID: &[u8] = b"SEGGER RTT\0";
const DESC_SIZE: u32 = 24;
/// Channel mode: block the target when the buffer is full (what probe-rs sets for defmt).
pub const MODE_BLOCK_IF_FULL: u32 = 2;

#[derive(Clone, Debug)]
pub struct RttReader {
    /// Address of the control block (`_SEGGER_RTT`).
    pub addr: u32,
    /// Mode to put the up channels in once the target has initialised them; `None` leaves them alone.
    pub mode: Option<u32>,
}

impl RttReader {
    pub fn new(addr: u32) -> Self {
        RttReader { addr, mode: Some(MODE_BLOCK_IF_FULL) }
    }

    /// Finds the control block from the ELF symbol table.
    pub fn from_image(image: &sim_core::Image) -> Option<Self> {
        image.symbol("_SEGGER_RTT").map(|s| Self::new(s.addr))
    }

    fn valid(&self, mem: &mut dyn TargetMemory) -> bool {
        mem.read_bytes(self.addr, ID.len()).is_some_and(|b| b == ID)
    }

    /// Reads everything new on every up channel. Returns `(channel, bytes)` for channels with data.
    pub fn poll(&self, mem: &mut dyn TargetMemory) -> Vec<(usize, Vec<u8>)> {
        let mut out = Vec::new();
        if !self.valid(mem) {
            return out;
        }
        let Some(num_up) = mem.read_u32(self.addr + 16) else { return out };
        for ch in 0..num_up.min(16) {
            let desc = self.addr + 24 + ch * DESC_SIZE;
            let (Some(buf), Some(size), Some(wr), Some(rd), Some(flags)) = (
                mem.read_u32(desc + 4),
                mem.read_u32(desc + 8),
                mem.read_u32(desc + 12),
                mem.read_u32(desc + 16),
                mem.read_u32(desc + 20),
            ) else {
                continue;
            };
            if let Some(mode) = self.mode
                && buf != 0
                && flags & 3 != mode
            {
                mem.write_u32(desc + 20, (flags & !3) | mode);
            }
            if size == 0 || wr >= size || rd >= size || wr == rd {
                continue;
            }
            let mut data = Vec::new();
            let mut pos = rd;
            while pos != wr {
                match mem.read_u8(buf + pos) {
                    Some(b) => data.push(b),
                    None => break,
                }
                pos = (pos + 1) % size;
            }
            mem.write_u32(desc + 16, wr);
            out.push((ch as usize, data));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::target::FlatMemory;

    fn setup() -> (FlatMemory, RttReader) {
        let mut m = FlatMemory { base: 0x2000_0000, data: vec![0; 0x1000] };
        let cb = 0x2000_0100;
        for (i, b) in ID.iter().enumerate() {
            m.write_u8(cb + i as u32, *b);
        }
        m.write_u32(cb + 16, 1);
        m.write_u32(cb + 20, 0);
        let desc = cb + 24;
        m.write_u32(desc + 4, 0x2000_0800); // buffer
        m.write_u32(desc + 8, 8); // size
        (m, RttReader::new(cb))
    }

    #[test]
    fn drains_with_wraparound_and_sets_mode() {
        let (mut m, rtt) = setup();
        let desc = rtt.addr + 24;
        for (i, b) in b"abcdefgh".iter().enumerate() {
            m.write_u8(0x2000_0800 + i as u32, *b);
        }
        m.write_u32(desc + 16, 6); // rd
        m.write_u32(desc + 12, 3); // wr (wrapped)
        let got = rtt.poll(&mut m);
        assert_eq!(got, vec![(0, b"ghabc".to_vec())]);
        assert_eq!(m.read_u32(desc + 16), Some(3), "read offset advanced");
        assert_eq!(m.read_u32(desc + 20).unwrap() & 3, MODE_BLOCK_IF_FULL);
        assert!(rtt.poll(&mut m).is_empty());
    }

    #[test]
    fn ignores_uninitialised_block() {
        let mut m = FlatMemory { base: 0x2000_0000, data: vec![0; 0x1000] };
        assert!(RttReader::new(0x2000_0100).poll(&mut m).is_empty());
    }
}
