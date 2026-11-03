//! Decoded blocks: runs of instructions decoded once and executed from a per-core cache by
//! [`CortexM0Plus::run_batch`].
//!
//! A block is decoded from the memory the bus fetches instructions from without checks ([`Bus::code_at`]) and
//! starts at any address the PC reaches. Conditional branches leave it when taken. It ends after any other branch,
//! an instruction left to the interpreter
//! ([`K::Step`]: exception entry and return, system registers, sleep, `SEV`, `BKPT`, undefined encodings, anything
//! that reads the PC), [`MAX_OPS`] instructions or the end of that memory. A block keeps the halfwords it was decoded
//! from and is only used while memory still holds them, which is checked each time it is looked up, so nothing has
//! to tell the cache about writes (SRAM, flash programming, a new image). Within a block, a store that reaches the
//! block's own code or an access that went anywhere but plain memory ([`Bus::take_slow_access`]) ends it, and the
//! next lookup looks at the code again.
//!
//! Executing an op does exactly what [`CortexM0Plus::execute`] does for its encoding: same results, flags, cycles,
//! accesses in the same order, faults (the `tests` module checks every 16-bit encoding against it).

use sim_core::bus::{Bus, Size};

use crate::exec::{Next, sext};
use crate::{CortexM0Plus, Fault};

/// Most instructions in a block.
pub(crate) const MAX_OPS: usize = 16;
/// Most bytes of code in a block (16 instructions; a `BL` ends one).
const MAX_BYTES: usize = 2 * MAX_OPS;
/// Cache entries per core (direct mapped).
const ENTRIES: usize = 2048;

/// What an op does. The ops before [`K::Ldr`] neither access the bus nor change anything but registers, flags and
/// the PC; the ones from [`K::B`] on end a block.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub(crate) enum K {
    // a = rd, b = rm, imm = shift (1..=31)
    LslI,
    LsrI,
    AsrI,
    // a = rd, b = rm: LSLS #0 (MOVS Rd, Rm), LSRS #32, ASRS #32
    Movs,
    Lsr32,
    Asr32,
    // a = rd, b = rn, imm = rm
    AddR,
    SubR,
    // a = rd, b = rn, imm = imm3
    AddI3,
    SubI3,
    // a = rd/rn, imm = imm8
    MovI,
    CmpI,
    AddI8,
    SubI8,
    // Data processing: a = rdn, b = rm
    And,
    Eor,
    LslR,
    LsrR,
    AsrR,
    Adc,
    Sbc,
    Ror,
    Tst,
    Rsb,
    CmpR,
    Cmn,
    Orr,
    Mul,
    Bic,
    Mvn,
    // High registers, neither the PC: a = rd/rn, b = rm
    AddHi,
    CmpHi,
    MovHi,
    // a = rd, imm = value (ADR)
    Set,
    // a = rd, imm = offset (ADD Rd, SP, #imm)
    AddSp,
    // imm = signed offset (ADD/SUB SP, SP, #imm)
    SpAdd,
    // a = rd, b = rm
    Sxth,
    Sxtb,
    Uxth,
    Uxtb,
    Rev,
    Rev16,
    Revsh,
    Nop,
    // ---- accesses the bus
    // a = rt, imm = address (LDR literal)
    Ldr,
    // a = rt, b = rn, imm = rm
    StrR,
    StrhR,
    StrbR,
    LdrsbR,
    LdrR,
    LdrhR,
    LdrbR,
    LdrshR,
    // a = rt, b = rn, imm = offset
    StrI,
    LdrI,
    StrbI,
    LdrbI,
    StrhI,
    LdrhI,
    // a = rn, imm = list
    Stm,
    Ldm,
    // imm = list (bit 14 LR / bit 15 PC)
    Push,
    Pop,
    // ---- sets the PC (and leaves the block when it branches)
    // a = cond, imm = target
    BCond,
    // ---- ends a block
    // imm = target
    B,
    // imm = target, b: unused; the return address is the op's address + 5
    Bl,
    // a = rm
    Bx,
    Blx,
    // imm = list including the PC
    PopPc,
    /// Left to the interpreter.
    Step,
}

impl K {
    /// Whether the op may touch the bus (or the exception state): the batch then checks what it left to do.
    #[inline(always)]
    pub(crate) fn uses_bus(self) -> bool {
        self as u8 >= K::Ldr as u8
    }

    #[inline(always)]
    pub(crate) fn ends_block(self) -> bool {
        self as u8 >= K::B as u8
    }

    /// Whether the op sets the PC (else the caller moves it on).
    #[inline(always)]
    pub(crate) fn sets_pc(self) -> bool {
        self as u8 >= K::BCond as u8
    }
}

/// One decoded instruction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct Op {
    pub(crate) kind: K,
    a: u8,
    b: u8,
    /// Offset of the instruction from the start of the block, in bytes.
    pub(crate) off: u8,
    imm: u32,
}

impl Op {
    /// The cycles an op that does not end a block takes.
    fn fixed_cycles(self) -> u32 {
        match self.kind {
            K::Stm | K::Ldm | K::Push | K::Pop => 1 + self.imm.count_ones(),
            // Not taken (taken, it is the last op run)
            K::BCond => 1,
            k if k.uses_bus() => 2,
            _ => 1,
        }
    }

    const fn new(kind: K, a: u32, b: u32, imm: u32) -> Op {
        Op { kind, a: a as u8, b: b as u8, off: 0, imm }
    }
}

/// A cached block.
#[derive(Clone, Copy)]
pub(crate) struct Entry {
    /// Address of the first instruction; odd for an unused entry.
    pc: u32,
    /// Ops and code bytes used.
    n: u8,
    bytes: u8,
    /// `lead[i]`: the cycles ops `0..i` take (only an op that ends a block, the last, has no fixed count).
    lead: [u8; MAX_OPS],
    /// The code, as little endian words masked by `mask` (to the first `bytes` bytes).
    code: [u64; MAX_BYTES / 8],
    mask: [u64; MAX_BYTES / 8],
    pub(crate) ops: [Op; MAX_OPS],
}

impl Entry {
    const EMPTY: Entry = Entry {
        pc: 1,
        n: 0,
        bytes: 0,
        lead: [0; MAX_OPS],
        code: [0; MAX_BYTES / 8],
        mask: [0; MAX_BYTES / 8],
        ops: [Op::new(K::Nop, 0, 0, 0); MAX_OPS],
    };

    #[inline(always)]
    pub(crate) fn ops(&self) -> &[Op] {
        &self.ops[..self.n as usize]
    }

    /// Address of the first instruction.
    #[inline(always)]
    pub(crate) fn start(&self) -> u32 {
        self.pc
    }

    /// The cycles the ops before op `i` take.
    #[inline(always)]
    pub(crate) fn lead_cycles(&self, i: usize) -> u64 {
        self.lead[i] as u64
    }

    /// Code bytes the block was decoded from.
    #[inline(always)]
    pub(crate) fn len(&self) -> u32 {
        self.bytes as u32
    }
}

/// The decoded block cache of one core. Cloning gives an empty cache (it is only a cache).
pub struct BlockCache {
    entries: Option<Box<[Entry]>>,
    /// Where the last batch stopped in the middle of a block (at its step limit): the PC, the entry and the op
    /// there, so that the next batch goes on with that block rather than decode a new one from there.
    resume: (u32, usize, usize),
}

impl Clone for BlockCache {
    fn clone(&self) -> Self {
        BlockCache::default()
    }
}

impl std::fmt::Debug for BlockCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("BlockCache")
    }
}

impl Default for BlockCache {
    fn default() -> Self {
        BlockCache { entries: None, resume: (1, 0, 0) }
    }
}

impl BlockCache {
    /// The block to run from `pc`, as the entry it is in and the index of the op at `pc`: the block the last
    /// batch stopped in, or the one starting at `pc`, decoded now unless the cached one was decoded from the same
    /// bytes. `None` if the bus has no code at `pc` ([`Bus::code_at`]).
    #[inline(always)]
    pub(crate) fn lookup<B: Bus + ?Sized>(&mut self, bus: &mut B, pc: u32) -> Option<(usize, usize)> {
        let entries = self.entries.get_or_insert_with(|| vec![Entry::EMPTY; ENTRIES].into_boxed_slice());
        let (at, slot, i) = self.resume;
        if at == pc {
            self.resume.0 = 1;
            let e = &entries[slot];
            if i < e.n as usize
                && e.pc.wrapping_add(e.ops[i].off as u32) == pc
                && bus.code_at(e.pc).is_some_and(|code| e.same_code(code))
            {
                return Some((slot, i));
            }
        }
        let code = bus.code_at(pc)?;
        let slot = (pc.wrapping_mul(0x9E37_79B1) >> (32 - ENTRIES.trailing_zeros())) as usize;
        let e = &mut entries[slot];
        if e.pc != pc || !e.same_code(code) {
            decode_block(e, pc, code);
        }
        Some((slot, 0))
    }

    /// A block [`BlockCache::lookup`] returned.
    #[inline(always)]
    pub(crate) fn entry(&self, slot: usize) -> &Entry {
        match &self.entries {
            Some(e) => &e[slot],
            None => &Entry::EMPTY,
        }
    }

    /// The batch stops before op `i` of the block in `slot`, at `pc`.
    #[inline(always)]
    pub(crate) fn stop_at(&mut self, pc: u32, slot: usize, i: usize) {
        self.resume = (pc, slot, i);
    }
}

impl Entry {
    /// Whether `code` starts with the bytes the block was decoded from.
    #[inline(always)]
    fn same_code(&self, code: &[u8]) -> bool {
        match code.first_chunk::<MAX_BYTES>() {
            Some(c) => {
                let mut diff = 0;
                for (k, chunk) in c.chunks_exact(8).enumerate() {
                    let word = u64::from_le_bytes(chunk.try_into().unwrap_or([0; 8]));
                    diff |= (word ^ self.code[k]) & self.mask[k];
                }
                diff == 0
            }
            None => code.len() >= self.bytes as usize && words(code, self.bytes as usize).0 == self.code,
        }
    }
}

/// The first `n` bytes of `code` as masked little endian words, and the masks.
fn words(code: &[u8], n: usize) -> ([u64; MAX_BYTES / 8], [u64; MAX_BYTES / 8]) {
    let mut bytes = [0; MAX_BYTES];
    bytes[..n].copy_from_slice(&code[..n]);
    let (mut w, mut m) = ([0; MAX_BYTES / 8], [0; MAX_BYTES / 8]);
    for k in 0..MAX_BYTES / 8 {
        let mut chunk = [0; 8];
        chunk.copy_from_slice(&bytes[8 * k..8 * k + 8]);
        w[k] = u64::from_le_bytes(chunk);
        let used = n.saturating_sub(8 * k).min(8);
        m[k] = if used == 8 { u64::MAX } else { (1u64 << (8 * used)) - 1 };
    }
    (w, m)
}

#[cold]
#[inline(never)]
fn decode_block(e: &mut Entry, pc: u32, code: &[u8]) {
    e.pc = pc;
    let mut n = 0;
    let mut at = 0;
    let half = |at: usize| code.get(at..at + 2).map(|b| u16::from_le_bytes([b[0], b[1]]) as u32);
    while n < MAX_OPS && at + 2 <= MAX_BYTES {
        let Some(ins) = half(at) else { break };
        let addr = pc.wrapping_add(at as u32);
        let (op, width) = if ins >> 11 >= 0b11101 {
            match half(at + 2) {
                Some(hw2) if at + 4 <= MAX_BYTES => (decode32(ins, hw2, addr), 4),
                // The rest of a 32-bit instruction is out of reach: the next block (or the interpreter) gets it
                _ => break,
            }
        } else {
            (decode16(ins, addr), 2)
        };
        e.ops[n] = Op { off: at as u8, ..op };
        n += 1;
        at += width;
        if op.kind.ends_block() {
            break;
        }
    }
    e.n = n as u8;
    e.bytes = at as u8;
    let mut lead = 0;
    for i in 0..n {
        e.lead[i] = lead as u8;
        lead += e.ops[i].fixed_cycles();
    }
    (e.code, e.mask) = words(code, at);
}

/// The op for a 32-bit encoding at `pc`: `BL`, or the interpreter.
pub(crate) fn decode32(hw1: u32, hw2: u32, pc: u32) -> Op {
    if hw1 >> 11 == 0b11110 && hw2 & 0xD000 == 0xD000 {
        let s = (hw1 >> 10) & 1;
        let j1 = (hw2 >> 13) & 1;
        let j2 = (hw2 >> 11) & 1;
        let i1 = !(j1 ^ s) & 1;
        let i2 = !(j2 ^ s) & 1;
        let imm = s << 24 | i1 << 23 | i2 << 22 | (hw1 & 0x3FF) << 12 | (hw2 & 0x7FF) << 1;
        let next_pc = pc.wrapping_add(4);
        return Op::new(K::Bl, 0, 0, next_pc.wrapping_add(sext(imm, 25)));
    }
    Op::new(K::Step, 0, 0, 0)
}

/// The op for a 16-bit encoding at `pc`.
pub(crate) fn decode16(ins: u32, pc: u32) -> Op {
    let pcv = pc.wrapping_add(4);
    let lo = |s: u32| (ins >> s) & 7;
    let step = Op::new(K::Step, 0, 0, 0);
    match ins >> 11 {
        0b00000 => match (ins >> 6) & 31 {
            0 => Op::new(K::Movs, lo(0), lo(3), 0),
            imm => Op::new(K::LslI, lo(0), lo(3), imm),
        },
        0b00001 => match (ins >> 6) & 31 {
            0 => Op::new(K::Lsr32, lo(0), lo(3), 0),
            imm => Op::new(K::LsrI, lo(0), lo(3), imm),
        },
        0b00010 => match (ins >> 6) & 31 {
            0 => Op::new(K::Asr32, lo(0), lo(3), 0),
            imm => Op::new(K::AsrI, lo(0), lo(3), imm),
        },
        0b00011 => {
            let kind = match (ins >> 9) & 3 {
                0 => K::AddR,
                1 => K::SubR,
                2 => K::AddI3,
                _ => K::SubI3,
            };
            Op::new(kind, lo(0), lo(3), lo(6))
        }
        0b00100 => Op::new(K::MovI, lo(8), 0, ins & 0xFF),
        0b00101 => Op::new(K::CmpI, lo(8), 0, ins & 0xFF),
        0b00110 => Op::new(K::AddI8, lo(8), 0, ins & 0xFF),
        0b00111 => Op::new(K::SubI8, lo(8), 0, ins & 0xFF),
        0b01000 if ins & (1 << 10) == 0 => {
            const DP: [K; 16] = [
                K::And,
                K::Eor,
                K::LslR,
                K::LsrR,
                K::AsrR,
                K::Adc,
                K::Sbc,
                K::Ror,
                K::Tst,
                K::Rsb,
                K::CmpR,
                K::Cmn,
                K::Orr,
                K::Mul,
                K::Bic,
                K::Mvn,
            ];
            Op::new(DP[((ins >> 6) & 0xF) as usize], lo(0), lo(3), 0)
        }
        0b01000 => {
            let rm = (ins >> 3) & 0xF;
            let rd = (ins & 7) | ((ins >> 4) & 8);
            match (ins >> 8) & 3 {
                0 if rd != 15 && rm != 15 => Op::new(K::AddHi, rd, rm, 0),
                1 if rd != 15 && rm != 15 => Op::new(K::CmpHi, rd, rm, 0),
                2 if rd != 15 && rm != 15 => Op::new(K::MovHi, rd, rm, 0),
                3 if ins & 7 == 0 && rm != 15 => {
                    let kind = if ins & (1 << 7) != 0 { K::Blx } else { K::Bx };
                    Op::new(kind, rm, 0, 0)
                }
                _ => step,
            }
        }
        0b01001 => Op::new(K::Ldr, lo(8), 0, (pcv & !3).wrapping_add((ins & 0xFF) * 4)),
        0b01010 | 0b01011 => {
            const LS: [K; 8] = [K::StrR, K::StrhR, K::StrbR, K::LdrsbR, K::LdrR, K::LdrhR, K::LdrbR, K::LdrshR];
            Op::new(LS[((ins >> 9) & 7) as usize], lo(0), lo(3), lo(6))
        }
        0b01100..=0b01111 => {
            let imm = (ins >> 6) & 31;
            match (ins >> 11) & 3 {
                0 => Op::new(K::StrI, lo(0), lo(3), imm * 4),
                1 => Op::new(K::LdrI, lo(0), lo(3), imm * 4),
                2 => Op::new(K::StrbI, lo(0), lo(3), imm),
                _ => Op::new(K::LdrbI, lo(0), lo(3), imm),
            }
        }
        0b10000 => Op::new(K::StrhI, lo(0), lo(3), ((ins >> 6) & 31) * 2),
        0b10001 => Op::new(K::LdrhI, lo(0), lo(3), ((ins >> 6) & 31) * 2),
        0b10010 => Op::new(K::StrI, lo(8), 13, (ins & 0xFF) * 4),
        0b10011 => Op::new(K::LdrI, lo(8), 13, (ins & 0xFF) * 4),
        0b10100 => Op::new(K::Set, lo(8), 0, (pcv & !3).wrapping_add((ins & 0xFF) * 4)),
        0b10101 => Op::new(K::AddSp, lo(8), 0, (ins & 0xFF) * 4),
        0b10110 | 0b10111 => match (ins >> 8) & 0xF {
            0x0 => {
                let imm = (ins & 0x7F) * 4;
                Op::new(K::SpAdd, 0, 0, if ins & (1 << 7) != 0 { imm.wrapping_neg() } else { imm })
            }
            0x2 => {
                const EXT: [K; 4] = [K::Sxth, K::Sxtb, K::Uxth, K::Uxtb];
                Op::new(EXT[((ins >> 6) & 3) as usize], lo(0), lo(3), 0)
            }
            0x4 | 0x5 => match (ins & 0xFF) | ((ins & 0x100) << 6) {
                0 => step,
                list => Op::new(K::Push, 0, 0, list),
            },
            0xA => match (ins >> 6) & 3 {
                0 => Op::new(K::Rev, lo(0), lo(3), 0),
                1 => Op::new(K::Rev16, lo(0), lo(3), 0),
                3 => Op::new(K::Revsh, lo(0), lo(3), 0),
                _ => step,
            },
            0xC | 0xD => match (ins & 0xFF) | ((ins & 0x100) << 7) {
                0 => step,
                list if list & (1 << 15) != 0 => Op::new(K::PopPc, 0, 0, list),
                list => Op::new(K::Pop, 0, 0, list),
            },
            // Hints other than WFE, WFI and SEV execute as NOP
            0xF if ins & 0xF == 0 && !(2..=4).contains(&((ins >> 4) & 0xF)) => Op::new(K::Nop, 0, 0, 0),
            _ => step,
        },
        0b11000 if ins & 0xFF != 0 => Op::new(K::Stm, lo(8), 0, ins & 0xFF),
        0b11001 if ins & 0xFF != 0 => Op::new(K::Ldm, lo(8), 0, ins & 0xFF),
        0b11010 | 0b11011 => match (ins >> 8) & 0xF {
            0xE | 0xF => step,
            cond => Op::new(K::BCond, cond, 0, pcv.wrapping_add(sext(ins & 0xFF, 8) << 1)),
        },
        0b11100 => Op::new(K::B, 0, 0, pcv.wrapping_add(sext(ins & 0x7FF, 11) << 1)),
        _ => step,
    }
}

/// `AddWithCarry` (as `exec::add_with_carry`) without 64-bit arithmetic.
#[inline(always)]
pub(crate) fn adc(x: u32, y: u32, carry: bool) -> (u32, bool, bool) {
    let (r, c1) = x.overflowing_add(y);
    let (r, c2) = r.overflowing_add(carry as u32);
    // Signed overflow: both operands have the sign the result does not
    (r, c1 | c2, ((x ^ r) & (y ^ r)) >> 31 != 0)
}

/// Why [`CortexM0Plus::run_blocks`] stopped.
pub(crate) enum Exit {
    /// The next instruction is the interpreter loop's to look at: the step limit was reached, the exception state
    /// changed, or the next block cannot run on its own (none there, or the budget is close).
    Next,
    /// The batch must end (the machine needs to look, or the budget was reached).
    Batch,
    /// The instruction at the address faulted.
    Fault(Fault, u32),
}

impl CortexM0Plus {
    /// Runs decoded blocks from the PC, `cycles` into a batch that may take `max_steps` more steps and ends at
    /// `budget` cycles, as long as each block's ops can run without the checks the interpreter loop makes before
    /// an instruction: no step limit or budget can end the batch before the last op of a block run, the exception
    /// state did not change and the PC is not below `trap_below`. Not with SysTick counting the core clock.
    ///
    /// Returns the cycles the batch has then taken, the steps run (with the one that faulted) and why it stopped,
    /// with the PC at the next instruction (or the one that faulted).
    #[inline(never)]
    pub(crate) fn run_blocks<B: Bus + ?Sized>(
        &mut self,
        bus: &mut B,
        cache: &mut BlockCache,
        max_steps: u32,
        mut cycles: u64,
        budget: u64,
        trap_below: u32,
    ) -> (u64, u32, Exit) {
        let mut steps = 0;
        loop {
            let pc = self.r[15];
            if pc < trap_below {
                return (cycles, steps, Exit::Next);
            }
            let Some((slot, first)) = cache.lookup(bus, pc) else {
                return (cycles, steps, Exit::Next);
            };
            let block = cache.entry(slot);
            let (lo, len) = (block.start(), block.len());
            let ops = &block.ops()[first..];
            let run = &ops[..ops.len().min((max_steps - steps) as usize)];
            if run.is_empty() || cycles + block.lead_cycles(first + run.len() - 1) - block.lead_cycles(first) >= budget
            {
                return (cycles, steps, Exit::Next);
            }
            let cut = run.len() < ops.len();
            let mut leave = false;
            let mut last = run[run.len() - 1];
            let mut ran = run.len();
            let mut ops = run.iter();
            while let Some(&op) = ops.next() {
                // Ops run so far, this one included
                let i = run.len() - ops.len() - 1;
                match self.exec_op(bus, op, lo, len, cycles, &mut leave) {
                    Ok(c) => cycles += c as u64,
                    Err(f) => return (cycles, steps + i as u32 + 1, Exit::Fault(f, lo.wrapping_add(op.off as u32))),
                }
                if op.kind.uses_bus() {
                    let stall = bus.take_stall_cycles();
                    cycles += stall as u64;
                    let attention = bus.take_attention() || std::mem::take(&mut self.batch_break);
                    if attention || bus.take_slow_access() || leave || stall != 0 {
                        if !op.kind.sets_pc() {
                            self.r[15] = lo.wrapping_add(op.off as u32 + 2);
                        }
                        if attention {
                            return (cycles, steps + i as u32 + 1, Exit::Batch);
                        }
                        (last, ran) = (op, i + 1);
                        break;
                    }
                }
            }
            steps += ran as u32;
            if !last.kind.sets_pc() {
                self.r[15] = lo.wrapping_add(last.off as u32 + 2);
                if cut && ran == run.len() {
                    cache.stop_at(self.r[15], slot, first + ran);
                }
            }
            if cycles >= budget {
                return (cycles, steps, Exit::Batch);
            }
            if steps == max_steps || self.exc_changed {
                return (cycles, steps, Exit::Next);
            }
        }
    }

    // The longer ops, out of line so that the loop running blocks stays small

    #[inline(never)]
    fn op_stm<B: Bus + ?Sized>(&mut self, bus: &mut B, rn: u32, list: u32) -> Result<u32, Fault> {
        self.stm(bus, rn, list)
    }

    #[inline(never)]
    fn op_ldm<B: Bus + ?Sized>(&mut self, bus: &mut B, rn: u32, list: u32) -> Result<u32, Fault> {
        self.ldm(bus, rn, list)
    }

    #[inline(never)]
    fn op_push<B: Bus + ?Sized>(&mut self, bus: &mut B, list: u32) -> Result<u32, Fault> {
        self.push(bus, list)
    }

    #[inline(never)]
    fn op_pop<B: Bus + ?Sized>(&mut self, bus: &mut B, list: u32) -> Result<(Next, u32), Fault> {
        self.pop(bus, list)
    }

    #[inline(never)]
    fn op_pop_pc<B: Bus + ?Sized>(&mut self, bus: &mut B, list: u32) -> Result<u32, Fault> {
        let (n, c) = self.pop(bus, list)?;
        if let Next::Branch(t) = n {
            self.r[15] = t;
        }
        Ok(c)
    }

    #[inline(never)]
    fn op_bx<B: Bus + ?Sized>(&mut self, bus: &mut B, target: u32) -> Result<u32, Fault> {
        let (n, c) = self.bx_write_pc(bus, target)?;
        if let Next::Branch(t) = n {
            self.r[15] = t;
        }
        Ok(2 + c)
    }

    #[inline(never)]
    fn op_step<B: Bus + ?Sized>(&mut self, bus: &mut B, pc: u32) -> Result<u32, Fault> {
        self.r[15] = pc;
        self.execute::<B, false>(bus)
    }

    #[inline(always)]
    fn load<B: Bus + ?Sized>(&mut self, bus: &mut B, addr: u32, size: Size) -> Result<u32, Fault> {
        self.read_mem(bus, addr, size)
    }

    /// A store at `addr`; `code` is set if it may have reached the block's code (`len` bytes from `lo`).
    #[inline(always)]
    fn store<B: Bus + ?Sized>(
        &mut self,
        bus: &mut B,
        addr: u32,
        (size, v): (Size, u32),
        (lo, len): (u32, u32),
        code: &mut bool,
    ) -> Result<(), Fault> {
        self.write_mem(bus, addr, size, v)?;
        *code |= addr.wrapping_sub(lo.wrapping_sub(4)) < len + 4;
        Ok(())
    }

    /// Executes `op`, an instruction of the block of `len` bytes from `lo`, `elapsed` cycles into the batch. Ops that
    /// end a block, and conditional branches, set the PC; the others leave it alone (the caller moves it on).
    /// Returns the cycles taken. Sets `leave` after a store that may have changed the block's code and after a
    /// conditional branch that was taken: the block must not go on.
    #[inline(always)]
    pub(crate) fn exec_op<B: Bus + ?Sized>(
        &mut self,
        bus: &mut B,
        op: Op,
        lo: u32,
        len: u32,
        elapsed: u64,
        leave: &mut bool,
    ) -> Result<u32, Fault> {
        let code = leave;
        let pc = lo.wrapping_add(op.off as u32);
        // Accesses happen at the instruction's start time
        macro_rules! timed {
            () => {
                bus.set_elapsed(elapsed)
            };
        }
        // Register numbers are below 16 (the masks spare the bounds checks)
        let a = (op.a & 15) as usize;
        let b = (op.b & 15) as usize;
        let m = (op.imm & 15) as usize;
        let imm = op.imm;
        macro_rules! arith {
            ($x:expr, $y:expr, $c:expr) => {{
                let (r, c, v) = adc($x, $y, $c);
                self.set_nz(r);
                self.c = c;
                self.v = v;
                r
            }};
        }
        macro_rules! logic {
            ($r:expr) => {{
                let r = $r;
                self.set_nz(r);
                self.r[a] = r;
            }};
        }
        match op.kind {
            K::LslI => {
                let v = self.r[b];
                self.c = (v >> (32 - imm)) & 1 != 0;
                logic!(v << imm);
            }
            K::LsrI => {
                let v = self.r[b];
                self.c = (v >> (imm - 1)) & 1 != 0;
                logic!(v >> imm);
            }
            K::AsrI => {
                let v = self.r[b];
                self.c = (v >> (imm - 1)) & 1 != 0;
                logic!(((v as i32) >> imm) as u32);
            }
            K::Movs => logic!(self.r[b]),
            K::Lsr32 => {
                self.c = self.r[b] >> 31 != 0;
                logic!(0);
            }
            K::Asr32 => {
                let v = self.r[b];
                self.c = v >> 31 != 0;
                logic!(((v as i32) >> 31) as u32);
            }
            K::AddR => self.r[a] = arith!(self.r[b], self.r[m], false),
            K::SubR => self.r[a] = arith!(self.r[b], !self.r[m], true),
            K::AddI3 => self.r[a] = arith!(self.r[b], imm, false),
            K::SubI3 => self.r[a] = arith!(self.r[b], !imm, true),
            K::MovI => logic!(imm),
            K::CmpI => {
                arith!(self.r[a], !imm, true);
            }
            K::AddI8 => self.r[a] = arith!(self.r[a], imm, false),
            K::SubI8 => self.r[a] = arith!(self.r[a], !imm, true),
            K::And => logic!(self.r[a] & self.r[b]),
            K::Eor => logic!(self.r[a] ^ self.r[b]),
            K::LslR => {
                let (dn, s) = (self.r[a], self.r[b] & 0xFF);
                let r = match s {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (32 - s)) & 1 != 0;
                        dn << s
                    }
                    32 => {
                        self.c = dn & 1 != 0;
                        0
                    }
                    _ => {
                        self.c = false;
                        0
                    }
                };
                logic!(r);
            }
            K::LsrR => {
                let (dn, s) = (self.r[a], self.r[b] & 0xFF);
                let r = match s {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (s - 1)) & 1 != 0;
                        dn >> s
                    }
                    32 => {
                        self.c = dn >> 31 != 0;
                        0
                    }
                    _ => {
                        self.c = false;
                        0
                    }
                };
                logic!(r);
            }
            K::AsrR => {
                let (dn, s) = (self.r[a], self.r[b] & 0xFF);
                let r = match s {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (s - 1)) & 1 != 0;
                        ((dn as i32) >> s) as u32
                    }
                    _ => {
                        self.c = dn >> 31 != 0;
                        ((dn as i32) >> 31) as u32
                    }
                };
                logic!(r);
            }
            K::Adc => self.r[a] = arith!(self.r[a], self.r[b], self.c),
            K::Sbc => self.r[a] = arith!(self.r[a], !self.r[b], self.c),
            K::Ror => {
                let (dn, s) = (self.r[a], self.r[b] & 0xFF);
                let r = if s == 0 {
                    dn
                } else {
                    let r = dn.rotate_right(s & 31);
                    self.c = r >> 31 != 0;
                    r
                };
                logic!(r);
            }
            K::Tst => self.set_nz(self.r[a] & self.r[b]),
            K::Rsb => self.r[a] = arith!(!self.r[b], 0, true),
            K::CmpR => {
                arith!(self.r[a], !self.r[b], true);
            }
            K::Cmn => {
                arith!(self.r[a], self.r[b], false);
            }
            K::Orr => logic!(self.r[a] | self.r[b]),
            K::Mul => logic!(self.r[a].wrapping_mul(self.r[b])),
            K::Bic => logic!(self.r[a] & !self.r[b]),
            K::Mvn => logic!(!self.r[b]),
            K::AddHi => {
                let r = self.r[a].wrapping_add(self.r[b]);
                self.r[a] = if a == 13 { r & !3 } else { r };
            }
            K::CmpHi => {
                arith!(self.r[a], !self.r[b], true);
            }
            K::MovHi => {
                let r = self.r[b];
                self.r[a] = if a == 13 { r & !3 } else { r };
            }
            K::Set => self.r[a] = imm,
            K::AddSp => self.r[a] = self.r[13].wrapping_add(imm),
            K::SpAdd => self.r[13] = self.r[13].wrapping_add(imm),
            K::Sxth => self.r[a] = sext(self.r[b] & 0xFFFF, 16),
            K::Sxtb => self.r[a] = sext(self.r[b] & 0xFF, 8),
            K::Uxth => self.r[a] = self.r[b] & 0xFFFF,
            K::Uxtb => self.r[a] = self.r[b] & 0xFF,
            K::Rev => self.r[a] = self.r[b].swap_bytes(),
            K::Rev16 => {
                let m = self.r[b];
                self.r[a] = ((m & 0x00FF_00FF) << 8) | ((m & 0xFF00_FF00) >> 8);
            }
            K::Revsh => {
                let m = self.r[b];
                self.r[a] = sext(((m & 0xFF) << 8) | ((m >> 8) & 0xFF), 16);
            }
            K::Nop => {}
            K::Ldr => {
                timed!();
                self.r[a] = self.load(bus, imm, Size::Word)?;
                return Ok(2);
            }
            K::StrR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.store(bus, addr, (Size::Word, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::StrhR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.store(bus, addr, (Size::Half, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::StrbR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.store(bus, addr, (Size::Byte, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::LdrsbR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.r[a] = sext(self.load(bus, addr, Size::Byte)?, 8);
                return Ok(2);
            }
            K::LdrR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.r[a] = self.load(bus, addr, Size::Word)?;
                return Ok(2);
            }
            K::LdrhR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.r[a] = self.load(bus, addr, Size::Half)?;
                return Ok(2);
            }
            K::LdrbR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.r[a] = self.load(bus, addr, Size::Byte)?;
                return Ok(2);
            }
            K::LdrshR => {
                timed!();
                let addr = self.r[b].wrapping_add(self.r[m]);
                self.r[a] = sext(self.load(bus, addr, Size::Half)?, 16);
                return Ok(2);
            }
            K::StrI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.store(bus, addr, (Size::Word, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::LdrI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.r[a] = self.load(bus, addr, Size::Word)?;
                return Ok(2);
            }
            K::StrbI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.store(bus, addr, (Size::Byte, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::LdrbI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.r[a] = self.load(bus, addr, Size::Byte)?;
                return Ok(2);
            }
            K::StrhI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.store(bus, addr, (Size::Half, self.r[a]), (lo, len), code)?;
                return Ok(2);
            }
            K::LdrhI => {
                timed!();
                let addr = self.r[b].wrapping_add(imm);
                self.r[a] = self.load(bus, addr, Size::Half)?;
                return Ok(2);
            }
            K::Stm => {
                timed!();
                let base = self.r[a];
                let r = self.op_stm(bus, a as u32, imm);
                // Conservatively: any part of the block within the stored words
                *code |= base.wrapping_sub(lo.wrapping_sub(64)) < len + 64;
                return r;
            }
            K::Ldm => {
                timed!();
                return self.op_ldm(bus, a as u32, imm);
            }
            K::Push => {
                timed!();
                let r = self.op_push(bus, imm);
                *code |= self.r[13].wrapping_sub(lo.wrapping_sub(64)) < len + 64;
                return r;
            }
            K::Pop => {
                timed!();
                return self.op_pop(bus, imm).map(|(_, c)| c);
            }
            K::B => {
                self.r[15] = imm;
                return Ok(2);
            }
            K::BCond => {
                if self.condition_passed(a as u32) {
                    self.r[15] = imm;
                    *code = true;
                    return Ok(2);
                }
                self.r[15] = pc.wrapping_add(2);
            }
            K::Bl => {
                self.r[14] = pc.wrapping_add(4) | 1;
                self.r[15] = imm;
                return Ok(3);
            }
            K::Bx | K::Blx => {
                timed!();
                let target = self.r[a];
                if op.kind == K::Blx {
                    self.r[14] = pc.wrapping_add(2) | 1;
                }
                return self.op_bx(bus, target);
            }
            K::PopPc => {
                timed!();
                return self.op_pop_pc(bus, imm);
            }
            K::Step => {
                timed!();
                return self.op_step(bus, pc);
            }
        }
        Ok(1)
    }
}
