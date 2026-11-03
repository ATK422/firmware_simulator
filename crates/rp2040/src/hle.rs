//! High-level emulation of the bootrom's flash functions (PLAN.md M2).
//!
//! The ROM's flash routines drive the SSI and the QSPI flash chip, which are only modelled from M6 on. Until then
//! their entry points, found through the ROM function table exactly as firmware finds them, are trapped: when a
//! core is about to execute one, the operation is carried out directly on the flash array and the core returns to
//! its caller. Erase and program take the time a W25Q16JV typically needs, charged to the calling core, so
//! firmware that paces its flash work against the watchdog sees realistic timing.

use std::collections::BTreeMap;

use sim_core::time::Time;

use crate::memmap::ROM_SIZE;

/// A trapped ROM function.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RomFn {
    ConnectInternalFlash,
    FlashExitXip,
    FlashRangeErase,
    FlashRangeProgram,
    FlashFlushCache,
    FlashEnterCmdXip,
    ResetToUsbBoot,
}

impl RomFn {
    pub const ALL: [(RomFn, [u8; 2]); 7] = [
        (RomFn::ConnectInternalFlash, *b"IF"),
        (RomFn::FlashExitXip, *b"EX"),
        (RomFn::FlashRangeErase, *b"RE"),
        (RomFn::FlashRangeProgram, *b"RP"),
        (RomFn::FlashFlushCache, *b"FC"),
        (RomFn::FlashEnterCmdXip, *b"CX"),
        (RomFn::ResetToUsbBoot, *b"UB"),
    ];
}

/// Typical W25Q16JV timings (datasheet 9.6): sector erase 45 ms, 64 KiB block erase 150 ms, page program 0.4 ms.
pub const SECTOR_ERASE: Time = Time::from_ms(45);
pub const BLOCK_ERASE: Time = Time::from_ms(150);
pub const PAGE_PROGRAM: Time = Time::from_us(400);
pub const SECTOR_SIZE: u32 = 4096;
pub const BLOCK_SIZE: u32 = 65536;
pub const PAGE_SIZE: u32 = 256;

/// Reads the ROM function table: `0x14` holds a halfword pointer to `(code, address)` halfword pairs ending in a
/// zero code. Returns entry address (without the Thumb bit) for each trapped function present.
pub fn rom_functions(rom: &[u8]) -> BTreeMap<u32, RomFn> {
    let mut out = BTreeMap::new();
    if rom.len() < 0x20 {
        return out;
    }
    let half = |a: usize| -> Option<u16> { Some(u16::from_le_bytes([*rom.get(a)?, *rom.get(a + 1)?])) };
    let Some(table) = half(0x14) else { return out };
    let mut p = table as usize;
    while p + 4 <= ROM_SIZE.min(rom.len()) {
        let (Some(code), Some(addr)) = (half(p), half(p + 2)) else { break };
        if code == 0 {
            break;
        }
        let code = code.to_le_bytes();
        if let Some((f, _)) = RomFn::ALL.iter().find(|(_, c)| *c == code) {
            out.insert(addr as u32 & !1, *f);
        }
        p += 4;
    }
    out
}

/// Erases `count` bytes at flash offset `offset` the way `flash_range_erase` does (whole sectors, or 64 KiB blocks
/// when the caller asks for them and alignment allows). Returns the time it takes.
pub fn erase(flash: &mut [u8], offset: u32, count: u32, block_size: u32) -> Time {
    let mut t = Time::ZERO;
    let mut addr = offset & !(SECTOR_SIZE - 1);
    let end = offset.saturating_add(count);
    while addr < end {
        let (size, cost) = if block_size == BLOCK_SIZE && addr.is_multiple_of(BLOCK_SIZE) && end - addr >= BLOCK_SIZE {
            (BLOCK_SIZE, BLOCK_ERASE)
        } else {
            (SECTOR_SIZE, SECTOR_ERASE)
        };
        for i in addr..addr + size {
            if let Some(b) = flash.get_mut(i as usize) {
                *b = 0xFF;
            }
        }
        t += cost;
        addr += size;
    }
    t
}

/// Programs `data` at flash offset `offset` (NOR semantics: bits can only be cleared). Returns the time it takes.
pub fn program(flash: &mut [u8], offset: u32, data: &[u8]) -> Time {
    for (i, &d) in data.iter().enumerate() {
        if let Some(b) = flash.get_mut(offset as usize + i) {
            *b &= d;
        }
    }
    let first_page = offset / PAGE_SIZE;
    let last_page = (offset + data.len().max(1) as u32 - 1) / PAGE_SIZE;
    Time(PAGE_PROGRAM.as_ps() * (last_page - first_page + 1) as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_flash_functions_in_the_b1_rom() {
        let f = rom_functions(crate::BOOTROM_B1);
        let names: Vec<RomFn> = f.values().copied().collect();
        for want in [RomFn::FlashRangeErase, RomFn::FlashRangeProgram, RomFn::ConnectInternalFlash, RomFn::FlashExitXip]
        {
            assert!(names.contains(&want), "{want:?} in {f:x?}");
        }
        assert!(f.keys().all(|&a| (a as usize) < ROM_SIZE));
    }

    #[test]
    fn erase_and_program() {
        let mut flash = vec![0u8; 3 * SECTOR_SIZE as usize];
        let t = erase(&mut flash, SECTOR_SIZE, SECTOR_SIZE, BLOCK_SIZE);
        assert_eq!(t, SECTOR_ERASE);
        assert!(flash[..SECTOR_SIZE as usize].iter().all(|&b| b == 0));
        assert!(flash[SECTOR_SIZE as usize..2 * SECTOR_SIZE as usize].iter().all(|&b| b == 0xFF));
        let t = program(&mut flash, SECTOR_SIZE + 10, &[0x0F, 0xF0]);
        assert_eq!(t, PAGE_PROGRAM);
        assert_eq!(&flash[SECTOR_SIZE as usize + 10..SECTOR_SIZE as usize + 12], &[0x0F, 0xF0]);
        // Programming over programmed bits only clears more
        program(&mut flash, SECTOR_SIZE + 10, &[0xF0, 0xFF]);
        assert_eq!(&flash[SECTOR_SIZE as usize + 10..SECTOR_SIZE as usize + 12], &[0x00, 0xF0]);
        // A page-crossing program counts two pages
        assert_eq!(program(&mut flash, SECTOR_SIZE + 250, &[0; 10]), Time(PAGE_PROGRAM.as_ps() * 2));
    }

    #[test]
    fn block_erase_when_aligned() {
        let mut flash = vec![0u8; 2 * BLOCK_SIZE as usize];
        assert_eq!(erase(&mut flash, 0, BLOCK_SIZE, BLOCK_SIZE), BLOCK_ERASE);
        assert_eq!(erase(&mut flash, 0, BLOCK_SIZE, SECTOR_SIZE), Time(SECTOR_ERASE.as_ps() * 16));
    }
}
