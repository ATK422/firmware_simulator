//! The bus a CPU executes against, and the trait memory mapped peripherals implement.

use std::fmt;

/// Access width.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Size {
    Byte = 1,
    Half = 2,
    Word = 4,
}

impl Size {
    pub const fn bytes(self) -> u32 {
        self as u32
    }

    /// Mask of the bits an access of this size carries.
    pub const fn mask(self) -> u32 {
        match self {
            Size::Byte => 0xFF,
            Size::Half => 0xFFFF,
            Size::Word => 0xFFFF_FFFF,
        }
    }
}

/// A failed bus access. The CPU turns it into a HardFault.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BusFault {
    /// Nothing is mapped at the address.
    Unmapped(u32),
    /// The region rejects this access (e.g. a write to ROM, an access size it does not support).
    Rejected(u32),
}

impl fmt::Display for BusFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusFault::Unmapped(a) => write!(f, "unmapped address {a:#010x}"),
            BusFault::Rejected(a) => write!(f, "access rejected at {a:#010x}"),
        }
    }
}

impl std::error::Error for BusFault {}

/// What a CPU sees: byte addressed, little endian, 32-bit address space.
///
/// Implementations attach the accessing core's identity and current time; the CPU does not need to know about
/// either.
pub trait Bus {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault>;
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault>;

    /// Instruction fetch of one halfword. Defaults to a data read; machines may give it a fast path.
    fn fetch16(&mut self, addr: u32) -> Result<u16, BusFault> {
        self.read(addr, Size::Half).map(|v| v as u16)
    }

    /// Instruction fetch from memory that needs no checks (no faults, no side effects), or `None` to go through
    /// [`Bus::fetch16`]. Lets an interpreter keep its fast path free of error handling.
    #[inline]
    fn fetch16_fast(&mut self, addr: u32) -> Option<u16> {
        let _ = addr;
        None
    }

    /// The memory that [`Bus::fetch16_fast`] serves `addr` from, from `addr` to its end, with the same side effects
    /// as a fast fetch at `addr`; `None` where `fetch16_fast` would return `None`. Lets an interpreter decode a run
    /// of instructions at once (and see later whether they are still the same). Every fast fetch from the returned
    /// bytes must be one that `fetch16_fast` would have served (with no further side effects) until an access
    /// reports [`Bus::take_slow_access`]. A bus that serves code must also answer [`Bus::take_attention`] for
    /// accesses only: an interpreter running decoded instructions does not ask it after those that access nothing.
    #[inline]
    fn code_at(&mut self, addr: u32) -> Option<&[u8]> {
        let _ = addr;
        None
    }

    /// Whether an access since the last call went anywhere but plain memory (a peripheral register, a checked flash
    /// access): it may have changed what [`Bus::code_at`] returned, or whether fast fetches are still allowed. The
    /// default says yes after every access.
    #[inline]
    fn take_slow_access(&mut self) -> bool {
        true
    }

    /// Plain memory at `addr..addr + len` (word aligned) that a CPU may read or (with `write`) write directly, as
    /// word accesses through [`Bus::read`]/[`Bus::write`] would, in address order and with no other effect than
    /// they would have; `None` to go through them. For multi-word transfers (`PUSH`, `POP`, `LDM`, `STM`).
    #[inline]
    fn ram(&mut self, addr: u32, len: u32, write: bool) -> Option<&mut [u8]> {
        let _ = (addr, len, write);
        None
    }

    /// Extra stall cycles accumulated by accesses since the last call (e.g. XIP cache misses).
    fn take_stall_cycles(&mut self) -> u32 {
        0
    }

    /// Called by a CPU running a batch of instructions (see `CortexM0Plus::run_batch`) before each instruction,
    /// with the cycles the batch has taken so far, so that accesses can be stamped with the time they happen at.
    #[inline]
    fn set_elapsed(&mut self, cycles: u64) {
        let _ = cycles;
    }

    /// Whether an access since the last call reached something other than plain memory (a peripheral register),
    /// after which the machine must look at the system before the CPU goes on: interrupt lines, clocks, resets may
    /// have changed. A batch of instructions ends after such an access. The default ends every batch after one
    /// instruction.
    #[inline]
    fn take_attention(&mut self) -> bool {
        true
    }

    /// Ticks of SysTick's reference clock (CLKSOURCE clear) since SysTick was last fed them, up to the current
    /// instruction's start, which a CPU running a batch feeds it before an access to a SysTick register: a machine
    /// that lets a batch run across ticks (feeding SysTick between batches) answers here, so that the access sees
    /// the counter as it is then. The default (a machine that feeds SysTick before every instruction) has none.
    #[inline]
    fn take_reference_ticks(&mut self) -> u64 {
        0
    }

    /// Reads without side effects, for debuggers and tracing. `None` if the location cannot be peeked.
    fn peek(&mut self, addr: u32, size: Size) -> Option<u32> {
        let _ = (addr, size);
        None
    }
}

/// How a register write combines with the current value. RP2040 peripherals expose XOR/SET/CLR aliases of every
/// register at offsets `+0x1000/+0x2000/+0x3000`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WriteMode {
    Normal,
    Xor,
    Set,
    Clear,
}

impl WriteMode {
    /// The value a plain read/write register ends up holding.
    pub fn apply(self, old: u32, value: u32) -> u32 {
        match self {
            WriteMode::Normal => value,
            WriteMode::Xor => old ^ value,
            WriteMode::Set => old | value,
            WriteMode::Clear => old & !value,
        }
    }

    /// Updates `reg` in place, only touching bits in `writable`.
    pub fn update(self, reg: &mut u32, value: u32, writable: u32) {
        let new = self.apply(*reg, value);
        *reg = (*reg & !writable) | (new & writable);
    }

    /// For write-1-to-clear bits: the bits this write clears.
    pub fn w1c_bits(self, value: u32) -> u32 {
        match self {
            WriteMode::Normal | WriteMode::Set => value,
            WriteMode::Xor | WriteMode::Clear => 0,
        }
    }
}

/// A memory mapped register block, addressed by word-aligned offset within its window.
///
/// `Ctx` is the machine's context (scheduler, current time, IRQ lines, …), so this trait stays SoC independent.
/// Accesses narrower than a word reach the peripheral as whole-word accesses: the machine replicates write data
/// across byte lanes as the RP2040 APB bridge does and extracts the lane from the read value.
pub trait Peripheral<Ctx> {
    /// Name used in logs, e.g. `"TIMER"`.
    fn name(&self) -> &'static str;
    /// Reads the register at `offset`. `None` means the register is not modelled.
    fn read(&mut self, ctx: &mut Ctx, offset: u32) -> Option<u32>;
    /// Writes the register at `offset`. `false` means the register is not modelled.
    fn write(&mut self, ctx: &mut Ctx, offset: u32, value: u32, mode: WriteMode) -> bool;
    /// Side-effect free read for debuggers. Defaults to nothing.
    fn peek(&self, offset: u32) -> Option<u32> {
        let _ = offset;
        None
    }
}

/// Little endian access to a byte buffer, shared by every RAM/ROM model.
pub mod mem {
    use super::Size;

    #[inline(always)]
    pub fn read(buf: &[u8], off: usize, size: Size) -> u32 {
        match size {
            Size::Byte => buf[off] as u32,
            // One bounds check and one load (indexing each byte checks each)
            Size::Half => u16::from_le_bytes(*chunk(buf, off)) as u32,
            Size::Word => u32::from_le_bytes(*chunk(buf, off)),
        }
    }

    /// The `N` bytes at `off`; panics out of range, as indexing does.
    #[inline(always)]
    fn chunk<const N: usize>(buf: &[u8], off: usize) -> &[u8; N] {
        match buf.get(off..).and_then(|b| b.first_chunk::<N>()) {
            Some(c) => c,
            None => panic!("{N} bytes at {off} out of range for a slice of {}", buf.len()),
        }
    }

    #[inline(always)]
    pub fn write(buf: &mut [u8], off: usize, size: Size, value: u32) {
        match size {
            Size::Byte => buf[off] = value as u8,
            Size::Half => buf[off..off + 2].copy_from_slice(&(value as u16).to_le_bytes()),
            Size::Word => buf[off..off + 4].copy_from_slice(&value.to_le_bytes()),
        }
    }

    /// Spreads a narrow write across all byte lanes of a word, as the RP2040 APB/AHB bridges do.
    #[inline]
    pub fn replicate(value: u32, size: Size) -> u32 {
        match size {
            Size::Byte => (value & 0xFF) * 0x0101_0101,
            Size::Half => (value & 0xFFFF) * 0x0001_0001,
            Size::Word => value,
        }
    }

    /// Extracts the lane of a narrow read at byte address `addr` from a word value.
    #[inline]
    pub fn lane(word: u32, addr: u32, size: Size) -> u32 {
        (word >> ((addr & 3) * 8)) & size.mask()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_modes() {
        assert_eq!(WriteMode::Normal.apply(0xF0, 0x0F), 0x0F);
        assert_eq!(WriteMode::Xor.apply(0xFF, 0x0F), 0xF0);
        assert_eq!(WriteMode::Set.apply(0xF0, 0x0F), 0xFF);
        assert_eq!(WriteMode::Clear.apply(0xFF, 0x0F), 0xF0);
        let mut r = 0xAAAA_0000;
        WriteMode::Normal.update(&mut r, 0xFFFF_FFFF, 0x0000_00FF);
        assert_eq!(r, 0xAAAA_00FF);
    }

    #[test]
    fn mem_helpers() {
        let mut b = [0u8; 8];
        mem::write(&mut b, 0, Size::Word, 0x1234_5678);
        assert_eq!(b[..4], [0x78, 0x56, 0x34, 0x12]);
        assert_eq!(mem::read(&b, 1, Size::Half), 0x3456);
        assert_eq!(mem::read(&b, 3, Size::Byte), 0x12);
        assert_eq!(mem::replicate(0xAB, Size::Byte), 0xABAB_ABAB);
        assert_eq!(mem::replicate(0xABCD, Size::Half), 0xABCD_ABCD);
        assert_eq!(mem::lane(0x1234_5678, 0x4000_0002, Size::Half), 0x1234);
        assert_eq!(mem::lane(0x1234_5678, 0x4000_0001, Size::Byte), 0x56);
    }
}
