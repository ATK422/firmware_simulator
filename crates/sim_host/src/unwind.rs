//! Stack unwinding for Cortex-M0+ firmware from the outside, as a debug probe does it: the ELF's call frame
//! information (`.debug_frame`) says, for every instruction, where the caller's frame and return address are;
//! exception frames (an `EXC_RETURN` value as return address) are unwound from the eight words the hardware stacked.
//! Functions without call frame information (bootrom code, hand-written assembly) are unwound through LR when they
//! are the innermost frame, which is right for leaf functions.
//!
//! Every return address is checked before it is followed: it must have the Thumb bit, point into code and follow a
//! `BL` or `BLX`, so a wrong guess ends the walk (marked as truncated) instead of producing a made-up caller.

use gimli::{BaseAddresses, CfaRule, CieOrFde, DebugFrame, LittleEndian, RegisterRule, UnwindContext, UnwindSection};

/// How a register of the caller is recovered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Rule {
    /// Not recoverable (for the return address: the outermost frame).
    Undefined,
    /// Unchanged by the callee (still in the register).
    Same,
    /// Saved at CFA + offset.
    Offset(i32),
}

/// One row of the unwind table: how to unwind from an address range.
#[derive(Clone, Debug)]
struct Row {
    start: u32,
    end: u32,
    cfa_reg: u8,
    cfa_off: i32,
    /// The return address (LR).
    ra: Rule,
    /// Callee-saved registers (other than LR) saved on the stack.
    saved: Vec<(u8, i32)>,
    /// Start of the function (its FDE).
    func: u32,
    /// The function calls others but never saves LR (it never returns: a reset handler, an executor loop), so
    /// LR is only its return address at its first instruction; anywhere else it is left over from a callee.
    lr_stale: bool,
}

/// The unwind rows of an image, sorted by address.
#[derive(Clone, Debug, Default)]
pub struct UnwindTable {
    rows: Vec<Row>,
}

impl UnwindTable {
    /// Reads `.debug_frame` from an ELF file. An image without one (or with one this cannot read) gets an empty
    /// table, so every walk falls back to LR.
    pub fn from_elf(elf: &[u8]) -> UnwindTable {
        use object::{Object, ObjectSection, SectionFlags};
        let Ok(file) = object::File::parse(elf) else { return UnwindTable::default() };
        let Some(data) = file.section_by_name(".debug_frame").and_then(|s| s.data().ok()) else {
            return UnwindTable::default();
        };
        // The code of a function, from the allocated section that holds it
        let code = |start: u32, end: u32| -> Option<Vec<u8>> {
            let s = file.sections().find(|s| {
                let alloc = matches!(s.flags(), SectionFlags::Elf { sh_flags } if sh_flags & object::elf::SHF_ALLOC as u64 != 0);
                alloc && s.address() <= start as u64 && end as u64 <= s.address() + s.size()
            })?;
            let d = s.data().ok()?;
            let o = (start as u64 - s.address()) as usize;
            d.get(o..o + (end - start) as usize).map(<[u8]>::to_vec)
        };
        match Self::from_debug_frame(data, &code) {
            Ok(t) => t,
            Err(e) => {
                log::warn!("could not read .debug_frame: {e}");
                UnwindTable::default()
            }
        }
    }

    fn from_debug_frame(data: &[u8], code: &dyn Fn(u32, u32) -> Option<Vec<u8>>) -> Result<UnwindTable, gimli::Error> {
        let mut section = DebugFrame::new(data, LittleEndian);
        section.set_address_size(4);
        let bases = BaseAddresses::default();
        let mut ctx = Box::new(UnwindContext::new());
        let mut rows = Vec::new();
        let mut entries = section.entries(&bases);
        while let Some(entry) = entries.next()? {
            let CieOrFde::Fde(partial) = entry else { continue };
            let Ok(fde) = partial.parse(DebugFrame::cie_from_offset) else { continue };
            // Functions the linker discarded keep their entries, at address 0 (where the bootrom is)
            if (fde.initial_address() as usize) < ROM_END {
                continue;
            }
            let ra_reg = fde.cie().return_address_register().0;
            // gimli reports a register nobody mentioned and one marked `DW_CFA_undefined` alike; only the CIE's
            // explicit undefined return address (the outermost frame) means the end of the stack
            let mut cie_undefined_ra = false;
            let mut ins = fde.cie().instructions(&section, &bases);
            while let Some(i) = ins.next()? {
                if let gimli::CallFrameInstruction::Undefined { register } = i
                    && register.0 == ra_reg
                {
                    cie_undefined_ra = true;
                }
            }
            let first = rows.len();
            let func = fde.initial_address() as u32;
            let mut table = fde.rows(&section, &bases, &mut ctx)?;
            while let Some(r) = table.next_row()? {
                let (cfa_reg, cfa_off) = match r.cfa() {
                    CfaRule::RegisterAndOffset { register, offset } if register.0 < 16 => {
                        (register.0 as u8, *offset as i32)
                    }
                    _ => continue,
                };
                let rule = |reg: u16| match r.register(gimli::Register(reg)) {
                    RegisterRule::Undefined => Rule::Undefined,
                    RegisterRule::Offset(o) => Rule::Offset(o as i32),
                    // An unmentioned register (or one marked so) keeps its value
                    _ => Rule::Same,
                };
                let ra = match rule(ra_reg) {
                    Rule::Undefined if !cie_undefined_ra => Rule::Same,
                    other => other,
                };
                let saved = r
                    .registers()
                    .filter(|(reg, _)| reg.0 < 16 && reg.0 != ra_reg && reg.0 != 13)
                    .filter_map(|(reg, rule)| match rule {
                        RegisterRule::Offset(o) => Some((reg.0 as u8, *o as i32)),
                        _ => None,
                    })
                    .collect();
                rows.push(Row {
                    start: r.start_address() as u32,
                    end: r.end_address() as u32,
                    cfa_reg,
                    cfa_off,
                    ra,
                    saved,
                    func,
                    lr_stale: false,
                });
            }
            let saves_lr = rows[first..].iter().any(|r| matches!(r.ra, Rule::Offset(_)));
            if !saves_lr {
                let end = fde.initial_address() + fde.len();
                let calls = code(func, end as u32).is_some_and(|c| contains_call(&c));
                for r in &mut rows[first..] {
                    r.lr_stale = calls;
                }
            }
        }
        rows.sort_by_key(|r| r.start);
        Ok(UnwindTable { rows })
    }

    /// Whether the table has rows at all.
    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    fn find(&self, addr: u32) -> Option<&Row> {
        let i = self.rows.partition_point(|r| r.start <= addr);
        let r = self.rows[..i].last()?;
        (addr < r.end).then_some(r)
    }
}

/// Whether Thumb code contains a `BL` or `BLX Rm`.
fn contains_call(code: &[u8]) -> bool {
    let h: Vec<u16> = code.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    let mut i = 0;
    while i < h.len() {
        if h[i] & 0xF800 == 0xF000 && h.get(i + 1).is_some_and(|n| n & 0xD000 == 0xD000) {
            return true;
        }
        if h[i] & 0xFF87 == 0x4780 {
            return true;
        }
        // Other 32-bit encodings (MSR, MRS, barriers) take two halfwords
        i += if h[i] >> 11 >= 0b11101 { 2 } else { 1 };
    }
    false
}

/// What the unwinder may read of the target.
pub trait Memory {
    fn word(&self, addr: u32) -> Option<u32>;
    fn half(&self, addr: u32) -> Option<u16>;
    /// Whether instructions can be fetched from `addr`.
    fn is_code(&self, addr: u32) -> bool;
    /// The start of the function containing `addr`, from the symbol table, for code without call frame
    /// information (whose prologue is then decoded).
    fn function_start(&self, _addr: u32) -> Option<u32> {
        None
    }
}

/// What a function's prologue did up to some address: how far below the CFA (the stack pointer at entry) the stack
/// pointer is, and where it saved LR and the low registers.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Prologue {
    /// Bytes pushed or allocated.
    depth: u32,
    /// LR's slot, as an offset from the CFA.
    lr: Option<i32>,
    /// r4-r7 saved by the first push, as offsets from the CFA.
    saved: Vec<(u8, i32)>,
}

/// Decodes the Thumb prologue of the function at `start` up to `pc`: `PUSH`, `SUB SP, #imm`, and the large-frame
/// form `LDR Rx, =-imm; ADD SP, Rx`. Other instructions that leave SP alone are skipped; the decoding stops at the
/// first branch or call (the prologue is over by then). `None` if something else changes SP.
fn prologue(mem: &dyn Memory, start: u32, pc: u32) -> Option<Prologue> {
    let mut p = Prologue::default();
    let mut lit: [Option<u32>; 8] = [None; 8];
    let mut a = start;
    // Prologues are short; give up on anything longer
    let end = pc.min(start + 96);
    while a < end {
        let h = mem.half(a)?;
        let wide = h >> 11 >= 0b11101;
        if wide {
            // BL (or another 32-bit instruction, none of which touch SP in a prologue)
            if h & 0xF800 == 0xF000 && mem.half(a + 2).is_some_and(|n| n & 0xD000 == 0xD000) {
                break;
            }
            a += 4;
            continue;
        }
        if h & 0xFE00 == 0xB400 {
            // PUSH {list[, LR]}: lowest register at the lowest address, LR at the top
            let n = (h & 0xFF).count_ones() + ((h >> 8) & 1) as u32;
            let top = p.depth;
            p.depth += 4 * n;
            if h & 0x100 != 0 && p.lr.is_none() {
                p.lr = Some(-(top as i32) - 4);
            }
            if p.saved.is_empty() {
                let mut slot = -(p.depth as i32);
                for r in 0..8u8 {
                    if h & (1 << r) != 0 {
                        if r >= 4 {
                            p.saved.push((r, slot));
                        }
                        slot += 4;
                    }
                }
            }
        } else if h & 0xFF80 == 0xB080 {
            p.depth += 4 * (h & 0x7F) as u32;
        } else if h & 0xFF80 == 0xB000 {
            p.depth = p.depth.checked_sub(4 * (h & 0x7F) as u32)?;
        } else if h & 0xF800 == 0x4800 {
            // LDR Rt, [PC, #imm]
            let rt = ((h >> 8) & 7) as usize;
            lit[rt] = mem.word(((a + 4) & !3) + 4 * (h & 0xFF) as u32);
        } else if h & 0xFFC0 == 0x4240 && (h & 7) == ((h >> 3) & 7) {
            // NEGS Rd, Rd
            let r = (h & 7) as usize;
            lit[r] = lit[r].map(u32::wrapping_neg);
        } else if h & 0xFF87 == 0x4485 {
            // ADD SP, Rm with a (negative) constant loaded before
            let v = lit[((h >> 3) & 0xF) as usize & 7]?;
            p.depth = p.depth.checked_add(v.wrapping_neg())?;
        } else if h & 0xF000 == 0xD000 && h & 0x0F00 < 0x0E00
            || h & 0xF800 == 0xE000
            || h & 0xFF00 == 0x4700
            || h & 0xFE00 == 0xBC00
        {
            // A branch, BX/BLX, POP: the prologue is over
            break;
        } else if h & 0xFF87 == 0x4685 {
            // MOV SP, Rm: not understood
            return None;
        }
        // Other instructions leave SP alone; one that writes a register forgets the literal it held
        if h & 0xF800 != 0x4800
            && h & 0xFFC0 != 0x4240
            && let Some(rd) = low_dest(h)
        {
            lit[rd] = None;
        }
        a += 2;
    }
    if p.depth > 0x10000 {
        return None;
    }
    Some(p)
}

/// The frame left at a function's final `POP {list, PC}`: just the popped registers.
fn epilogue_pop(h: u16) -> Prologue {
    let n = (h & 0xFF).count_ones() + 1;
    let depth = 4 * n;
    let mut slot = -(depth as i32);
    let mut saved = Vec::new();
    for r in 0..8u8 {
        if h & (1 << r) != 0 {
            if r >= 4 {
                saved.push((r, slot));
            }
            slot += 4;
        }
    }
    Prologue { depth, lr: Some(-4), saved }
}

/// The low register a 16-bit data instruction writes, roughly (enough to forget stale literals).
fn low_dest(h: u16) -> Option<usize> {
    match h >> 11 {
        // shifts, add/sub, mov/cmp/add/sub immediate
        0b00000..=0b00011 => Some((h & 7) as usize),
        0b00100 | 0b00110 | 0b00111 => Some(((h >> 8) & 7) as usize),
        // data processing (Rd in bits 0-2), loads
        0b01000 => Some((h & 7) as usize),
        0b01011 | 0b01101 | 0b01111 | 0b10001 => Some((h & 7) as usize),
        0b10011..=0b10101 => Some(((h >> 8) & 7) as usize),
        _ => None,
    }
}

/// A core's registers at the moment of the sample.
#[derive(Clone, Copy, Debug)]
pub struct Registers {
    /// r0-r15 (r13 the active stack pointer, r15 the address of the next instruction).
    pub r: [u32; 16],
    /// The process stack pointer, for an exception taken from thread mode on PSP.
    pub psp: u32,
}

/// A walked stack.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Stack {
    /// Innermost first: the PC, then for each caller the address of its call instruction (inside the caller, so
    /// that it symbolizes to the caller even when the call is the last instruction of a function).
    pub frames: Vec<u32>,
    /// The walk ended on something it could not unwind rather than at the outermost frame.
    pub truncated: bool,
    /// An LR guess was used for a frame without call frame information.
    pub guessed: bool,
}

/// End of the bootrom.
const ROM_END: usize = 0x4000;

/// The deepest stack followed.
pub const MAX_DEPTH: usize = 64;

fn is_exc_return(v: u32) -> bool {
    v & 0xFFFF_FFF0 == 0xFFFF_FFF0
}

/// Whether `ra` is a plausible return address: Thumb, in code, right after a `BL` or `BLX Rm`.
fn plausible_return(mem: &dyn Memory, ra: u32) -> bool {
    if ra & 1 == 0 {
        return false;
    }
    let a = ra & !1;
    if !mem.is_code(a.wrapping_sub(4)) {
        return false;
    }
    let bl = matches!((mem.half(a.wrapping_sub(4)), mem.half(a.wrapping_sub(2))),
        (Some(h1), Some(h2)) if h1 & 0xF800 == 0xF000 && h2 & 0xD000 == 0xD000);
    let blx = matches!(mem.half(a.wrapping_sub(2)), Some(h) if h & 0xFF87 == 0x4780);
    bl || blx
}

/// Walks the stack of a core stopped with `regs`.
pub fn unwind(table: &UnwindTable, mem: &dyn Memory, regs: &Registers) -> Stack {
    let mut r: [Option<u32>; 16] = regs.r.map(Some);
    let mut stack = Stack::default();
    let mut pc = regs.r[15] & !1;
    // The address looked up in the table: the PC itself for the innermost frame and for interrupted code, the call
    // instruction for callers
    let mut lookup = pc;
    stack.frames.push(pc);
    // LR holds this frame's return address only while nothing has been called since entry: the innermost frame, or
    // code interrupted by an exception
    let mut live_lr = true;
    loop {
        if stack.frames.len() >= MAX_DEPTH {
            stack.truncated = true;
            break;
        }
        let Some(sp) = r[13] else {
            stack.truncated = true;
            break;
        };
        let ra = match table.find(lookup) {
            Some(row) => {
                let Some(base) = r[row.cfa_reg as usize] else {
                    stack.truncated = true;
                    break;
                };
                let cfa = base.wrapping_add(row.cfa_off as u32);
                let read = |o: i32| mem.word(cfa.wrapping_add(o as u32));
                let ra = match row.ra {
                    Rule::Undefined => break,
                    Rule::Same if live_lr && (!row.lr_stale || lookup == row.func) => r[14],
                    // A function that has called something without saving LR can never return: it is the root of
                    // the stack (a reset handler, `main` and other `-> !` entry points)
                    Rule::Same => break,
                    Rule::Offset(o) => read(o),
                };
                for &(reg, o) in &row.saved {
                    r[reg as usize] = read(o);
                }
                if cfa < sp {
                    // A caller's frame is never below its callee's
                    stack.truncated = true;
                    break;
                }
                r[13] = Some(cfa);
                ra
            }
            // The bootrom has no symbols or call frame information; its own code calling into the firmware (core 1's
            // launch, the boot path) is the root of what runs there
            None if !live_lr && (pc as usize) < ROM_END => break,
            None => {
                // No call frame information: decode the function's prologue, if its start is known (or, at the
                // function's final `POP {..., PC}`, that)
                let start = mem.function_start(lookup);
                let frame = match mem.half(lookup) {
                    Some(h) if live_lr && h & 0xFF00 == 0xBD00 => Some(epilogue_pop(h)),
                    _ => start.and_then(|f| prologue(mem, f, lookup)),
                };
                match frame {
                    Some(p) if p.lr.is_some() || p.depth > 0 || !live_lr => {
                        stack.guessed = true;
                        let cfa = sp.wrapping_add(p.depth);
                        let ra = match p.lr {
                            Some(o) => mem.word(cfa.wrapping_add(o as u32)),
                            None if live_lr => r[14],
                            None => None,
                        };
                        for &(reg, o) in &p.saved {
                            r[reg as usize] = mem.word(cfa.wrapping_add(o as u32));
                        }
                        r[13] = Some(cfa);
                        ra
                    }
                    // Nothing pushed (yet): a leaf, or the start of a function
                    _ if live_lr => {
                        stack.guessed = true;
                        r[14]
                    }
                    _ => None,
                }
            }
        };
        if (pc as usize) < ROM_END && ra.is_some_and(|a| (a as usize) < ROM_END) {
            // Within the bootrom: the rest is its own boot path
            break;
        }
        let Some(ra) = ra else {
            stack.truncated = true;
            break;
        };
        r[14] = None;
        if is_exc_return(ra) {
            // The hardware stacked r0-r3, r12, LR, PC and xPSR on the stack the interrupted code was using
            let frame_sp = if ra & 4 != 0 { regs.psp } else { r[13].unwrap_or(0) };
            let words: Option<Vec<u32>> = (0..8).map(|k| mem.word(frame_sp.wrapping_add(4 * k))).collect();
            let Some(w) = words else {
                stack.truncated = true;
                break;
            };
            for (k, reg) in [0, 1, 2, 3, 12].into_iter().enumerate() {
                r[reg] = Some(w[k]);
            }
            r[14] = Some(w[5]);
            let aligned = w[7] & (1 << 9) != 0;
            r[13] = Some(frame_sp + 32 + if aligned { 4 } else { 0 });
            pc = w[6] & !1;
            if !mem.is_code(pc) {
                stack.truncated = true;
                break;
            }
            lookup = pc;
            live_lr = true;
            stack.frames.push(pc);
            continue;
        }
        if !plausible_return(mem, ra) {
            // The reset handler's caller (LR 0xFFFFFFFF after reset) ends the walk normally
            if ra != 0xFFFF_FFFF {
                stack.truncated = true;
            }
            break;
        }
        let call = (ra & !1) - 2;
        if call == pc && r[13] == Some(sp) {
            // No progress: a guessed LR pointing back into the same frame
            stack.truncated = true;
            break;
        }
        pc = call;
        lookup = call;
        live_lr = false;
        stack.frames.push(call);
    }
    stack
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// Memory for tests: words and halfwords at addresses, code everywhere in 0x1000_0000..0x1100_0000.
    #[derive(Default)]
    struct Fake {
        mem: BTreeMap<u32, u8>,
    }

    impl Fake {
        fn put_word(&mut self, a: u32, v: u32) {
            for (i, b) in v.to_le_bytes().into_iter().enumerate() {
                self.mem.insert(a + i as u32, b);
            }
        }
        fn put_half(&mut self, a: u32, v: u16) {
            for (i, b) in v.to_le_bytes().into_iter().enumerate() {
                self.mem.insert(a + i as u32, b);
            }
        }
        /// A `BL` ending at `ra`.
        fn bl_before(&mut self, ra: u32) {
            let a = ra & !1;
            self.put_half(a - 4, 0xF000);
            self.put_half(a - 2, 0xF800);
        }
    }

    impl Memory for Fake {
        fn word(&self, a: u32) -> Option<u32> {
            Some(u32::from_le_bytes([
                *self.mem.get(&a)?,
                *self.mem.get(&(a + 1))?,
                *self.mem.get(&(a + 2))?,
                *self.mem.get(&(a + 3))?,
            ]))
        }
        fn half(&self, a: u32) -> Option<u16> {
            Some(u16::from_le_bytes([*self.mem.get(&a)?, *self.mem.get(&(a + 1))?]))
        }
        fn is_code(&self, a: u32) -> bool {
            (0x1000_0000..0x1100_0000).contains(&a)
        }
    }

    fn row(start: u32, end: u32, cfa_off: i32, ra: Rule, saved: Vec<(u8, i32)>) -> Row {
        Row { start, end, cfa_reg: 13, cfa_off, ra, saved, func: start & !0xFF, lr_stale: false }
    }

    #[test]
    fn lr_is_left_over_in_functions_that_call_without_saving_it() {
        // A noreturn loop at 0x100 that calls 0x200 and never saves LR; LR still points after its last call
        let mut table = UnwindTable {
            rows: vec![
                row(0x1000_0100, 0x1000_0200, 0, Rule::Same, vec![]),
                row(0x1000_0200, 0x1000_0300, 0, Rule::Same, vec![]),
            ],
        };
        table.rows[0].lr_stale = true;
        let mut m = Fake::default();
        m.bl_before(0x1000_0151);
        let s = unwind(&table, &m, &regs(0x1000_0152, 0x2000_0ff8, 0x1000_0151));
        assert_eq!(s.frames, vec![0x1000_0152]);
        assert!(!s.truncated);
        // At its first instruction LR is still its caller's
        m.bl_before(0x1000_0221);
        let s = unwind(&table, &m, &regs(0x1000_0100, 0x2000_0ff8, 0x1000_0221));
        assert_eq!(s.frames, vec![0x1000_0100, 0x1000_021e]);
    }

    #[test]
    fn prologues_without_cfi_are_decoded() {
        // __aeabi_ldivmod: push {r4, lr}; sub sp, #16; add r4, sp, #8; str r4, [sp]; bl ...
        let mut m = Fake::default();
        for (i, h) in [0xb510u16, 0xb084, 0xac02, 0x9400, 0xf000, 0xf800].into_iter().enumerate() {
            m.put_half(0x1000_0400 + 2 * i as u32, h);
        }
        let p = prologue(&m, 0x1000_0400, 0x1000_0408).unwrap();
        assert_eq!(p, Prologue { depth: 24, lr: Some(-4), saved: vec![(4, -8)] });
        // Half way: only the push
        assert_eq!(prologue(&m, 0x1000_0400, 0x1000_0402).unwrap().depth, 8);
        // Large frames: push {r4-r7, lr}; ldr r6, =-4096; add sp, r6
        let mut m = Fake::default();
        for (i, h) in [0xb5f0u16, 0x4e01, 0x44b5, 0xe7fe].into_iter().enumerate() {
            m.put_half(0x1000_0500 + 2 * i as u32, h);
        }
        m.put_word(0x1000_0508, (-4096i32) as u32);
        let p = prologue(&m, 0x1000_0500, 0x1000_0506).unwrap();
        assert_eq!(p.depth, 20 + 4096);
        assert_eq!(p.lr, Some(-4));
        assert_eq!(p.saved, vec![(4, -20), (5, -16), (6, -12), (7, -8)]);
    }

    #[test]
    fn an_outer_function_without_cfi_is_unwound_through_its_prologue() {
        struct WithSymbols(Fake);
        impl Memory for WithSymbols {
            fn word(&self, a: u32) -> Option<u32> {
                self.0.word(a)
            }
            fn half(&self, a: u32) -> Option<u16> {
                self.0.half(a)
            }
            fn is_code(&self, a: u32) -> bool {
                self.0.is_code(a)
            }
            fn function_start(&self, a: u32) -> Option<u32> {
                (0x1000_0400..0x1000_0440).contains(&a).then_some(0x1000_0400)
            }
        }
        // main (CFI, root) calls asm (no CFI: push {r4, lr}; sub sp, #16; ... bl) calls leaf (CFI)
        let table = UnwindTable {
            rows: vec![
                row(0x1000_0000, 0x1000_0100, 0, Rule::Undefined, vec![]),
                row(0x1000_0200, 0x1000_0300, 0, Rule::Same, vec![]),
            ],
        };
        let mut m = Fake::default();
        for (i, h) in [0xb510u16, 0xb084, 0xac02, 0x9400, 0xf000, 0xf800].into_iter().enumerate() {
            m.put_half(0x1000_0400 + 2 * i as u32, h);
        }
        m.bl_before(0x1000_0041);
        // asm's frame: 16 bytes, then r4 and lr
        m.put_word(0x2000_0fe0 + 20, 0x1000_0041);
        let s = unwind(&table, &WithSymbols(m), &regs(0x1000_0210, 0x2000_0fe0, 0x1000_040d));
        assert_eq!(s.frames, vec![0x1000_0210, 0x1000_040a, 0x1000_003e]);
        assert!(!s.truncated, "{s:?}");
    }

    #[test]
    fn the_final_pop_of_a_function_without_cfi() {
        assert_eq!(epilogue_pop(0xbd10), Prologue { depth: 8, lr: Some(-4), saved: vec![(4, -8)] });
        assert_eq!(epilogue_pop(0xbd00), Prologue { depth: 4, lr: Some(-4), saved: vec![] });
    }

    #[test]
    fn calls_are_found_in_code() {
        let bl = [0x00, 0xF0, 0x00, 0xF8];
        assert!(contains_call(&bl));
        assert!(contains_call(&[0x00, 0xbf, 0x98, 0x47])); // nop; blx r3
        assert!(!contains_call(&[0x01, 0x38, 0xfd, 0xd1, 0x70, 0x47])); // subs; bne; bx lr
    }

    fn regs(pc: u32, sp: u32, lr: u32) -> Registers {
        let mut r = [0; 16];
        r[15] = pc;
        r[13] = sp;
        r[14] = lr;
        Registers { r, psp: 0 }
    }

    #[test]
    fn follows_saved_return_addresses_and_stops_at_the_outermost_frame() {
        // main (0x1000_0000..0x100) calls outer (0x100..0x200) calls leaf (0x200..0x300)
        let table = UnwindTable {
            rows: vec![
                row(0x1000_0000, 0x1000_0100, 0, Rule::Undefined, vec![]),
                row(0x1000_0100, 0x1000_0102, 0, Rule::Same, vec![]),
                // after `push {r4, lr}`
                row(0x1000_0102, 0x1000_0200, 8, Rule::Offset(-4), vec![(4, -8)]),
                row(0x1000_0200, 0x1000_0300, 0, Rule::Same, vec![]),
            ],
        };
        let mut m = Fake::default();
        m.bl_before(0x1000_0021);
        m.bl_before(0x1000_0151);
        // outer's frame at 0x2000_0ff8: r4, lr
        m.put_word(0x2000_0ff8, 0x1234);
        m.put_word(0x2000_0ffc, 0x1000_0021);
        let s = unwind(&table, &m, &regs(0x1000_0210, 0x2000_0ff8, 0x1000_0151));
        assert_eq!(s.frames, vec![0x1000_0210, 0x1000_014e, 0x1000_001e]);
        assert!(!s.truncated && !s.guessed, "{s:?}");
    }

    #[test]
    fn a_return_address_without_a_call_before_it_ends_the_walk() {
        let table = UnwindTable { rows: vec![row(0x1000_0200, 0x1000_0300, 0, Rule::Same, vec![])] };
        let m = Fake::default();
        let s = unwind(&table, &m, &regs(0x1000_0210, 0x2000_0ff8, 0x1000_0151));
        assert_eq!(s.frames, vec![0x1000_0210]);
        assert!(s.truncated);
    }

    #[test]
    fn exception_frames_continue_into_the_interrupted_code() {
        // A handler at 0x300 (leaf, LR = EXC_RETURN to thread mode on MSP) interrupted `leaf` at 0x220, which was
        // called from main
        let table = UnwindTable {
            rows: vec![
                row(0x1000_0000, 0x1000_0100, 0, Rule::Undefined, vec![]),
                row(0x1000_0200, 0x1000_0300, 0, Rule::Same, vec![]),
                row(0x1000_0300, 0x1000_0400, 0, Rule::Same, vec![]),
            ],
        };
        let mut m = Fake::default();
        m.bl_before(0x1000_0041);
        let frame = [0, 1, 2, 3, 12, 0x1000_0041, 0x1000_0220, 0x0100_0000];
        for (k, w) in frame.into_iter().enumerate() {
            m.put_word(0x2000_0fe0 + 4 * k as u32, w);
        }
        let s = unwind(&table, &m, &regs(0x1000_0310, 0x2000_0fe0, 0xFFFF_FFF9));
        assert_eq!(s.frames, vec![0x1000_0310, 0x1000_0220, 0x1000_003e]);
        assert!(!s.truncated, "{s:?}");
    }

    #[test]
    fn functions_without_cfi_are_unwound_through_lr_when_innermost() {
        let table = UnwindTable { rows: vec![row(0x1000_0000, 0x1000_0100, 0, Rule::Undefined, vec![])] };
        let mut m = Fake::default();
        m.bl_before(0x1000_0041);
        let s = unwind(&table, &m, &regs(0x1000_0810, 0x2000_0fe0, 0x1000_0041));
        assert_eq!(s.frames, vec![0x1000_0810, 0x1000_003e]);
        assert!(s.guessed && !s.truncated, "{s:?}");
        // but not beyond: a caller without CFI has no live LR
        let s = unwind(&UnwindTable::default(), &m, &regs(0x1000_0810, 0x2000_0fe0, 0x1000_0041));
        assert_eq!(s.frames, vec![0x1000_0810, 0x1000_003e]);
        assert!(s.truncated);
    }
}
