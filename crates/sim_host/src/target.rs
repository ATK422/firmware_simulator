//! Debug-probe style access to a simulated target's memory.

use rp2040::Machine;
use sim_core::bus::Size;

/// What a debug probe can do to target memory: side-effect free reads and direct writes.
pub trait TargetMemory {
    fn read_u8(&mut self, addr: u32) -> Option<u8>;
    fn write_u8(&mut self, addr: u32, v: u8) -> bool;

    fn read_u32(&mut self, addr: u32) -> Option<u32> {
        let mut b = [0u8; 4];
        for (i, byte) in b.iter_mut().enumerate() {
            *byte = self.read_u8(addr.wrapping_add(i as u32))?;
        }
        Some(u32::from_le_bytes(b))
    }

    fn write_u32(&mut self, addr: u32, v: u32) -> bool {
        v.to_le_bytes().iter().enumerate().all(|(i, &b)| self.write_u8(addr.wrapping_add(i as u32), b))
    }

    fn read_bytes(&mut self, addr: u32, len: usize) -> Option<Vec<u8>> {
        (0..len).map(|i| self.read_u8(addr.wrapping_add(i as u32))).collect()
    }
}

impl TargetMemory for Machine {
    fn read_u8(&mut self, addr: u32) -> Option<u8> {
        self.sys.peek(addr, Size::Byte).map(|v| v as u8)
    }
    fn write_u8(&mut self, addr: u32, v: u8) -> bool {
        self.sys.poke(addr, Size::Byte, v as u32)
    }
    fn read_u32(&mut self, addr: u32) -> Option<u32> {
        if addr & 3 == 0 {
            self.sys.peek(addr, Size::Word)
        } else {
            (0..4).try_fold(0u32, |acc, i| Some(acc | (self.read_u8(addr + i)? as u32) << (8 * i)))
        }
    }
}

/// A flat memory for tests.
#[cfg(test)]
pub struct FlatMemory {
    pub base: u32,
    pub data: Vec<u8>,
}

#[cfg(test)]
impl TargetMemory for FlatMemory {
    fn read_u8(&mut self, addr: u32) -> Option<u8> {
        self.data.get(addr.checked_sub(self.base)? as usize).copied()
    }
    fn write_u8(&mut self, addr: u32, v: u8) -> bool {
        match addr.checked_sub(self.base).and_then(|o| self.data.get_mut(o as usize)) {
            Some(b) => {
                *b = v;
                true
            }
            None => false,
        }
    }
}
