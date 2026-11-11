//! Random Thumb programs for the differential tests.
//!
//! A program is a main loop of chunks (each a run of random instructions ending in a branch to the next, with its
//! literal pool after it), a few functions the chunks call, and handlers for every exception. Instructions come from
//! every ARMv6-M class: data processing with every flag-setting form and shifts by immediate and by register
//! (including 0, 32 and more), high register operations (with the PC and SP as operands), loads and stores of every
//! size to SRAM, flash, ROM, the test registers, the PPB, unmapped and unaligned addresses, `LDM`/`STM`/`PUSH`/`POP`
//! (with the PC, and to an address with bit 0 clear), `BL`/`BX`/`BLX`, `MOV`/`ADD` to the PC, conditional branches
//! and counted loops, `SVC`, `BKPT`, `UDF`, `CPS`, `MSR`/`MRS`, barriers, hints, sleep, `SEV`, random 16- and 32-bit
//! encodings, and stores into the code being run (when it is in SRAM). Faults are taken by a HardFault handler that
//! resumes the main loop at a chunk start, so a program runs on whatever it does; the driver bounds it by steps.
//!
//! r8 and r9 are loop counters (random instructions never write them), so loops end.

use super::bus::{self, DiffBus, FLASH_BASE, REGS_BASE, ROM_BASE, SRAM_BASE, reg};
use crate::CortexM0Plus;
use crate::ppb;
use crate::tests::enc::*;

/// splitmix64: small, deterministic, good enough.
#[derive(Clone)]
pub(crate) struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }
    pub fn next64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn u32(&mut self) -> u32 {
        (self.next64() >> 32) as u32
    }
    /// Uniform in `0..n`.
    pub fn below(&mut self, n: u32) -> u32 {
        ((self.u32() as u64 * n as u64) >> 32) as u32
    }
    pub fn range(&mut self, lo: u32, hi: u32) -> u32 {
        lo + self.below(hi - lo + 1)
    }
    pub fn chance(&mut self, percent: u32) -> bool {
        self.below(100) < percent
    }
    pub fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len() as u32) as usize]
    }
    /// A value that is often an edge case for shifts, carries and overflows.
    pub fn interesting(&mut self) -> u32 {
        match self.below(4) {
            0 => self.pick(&[
                0,
                1,
                2,
                31,
                32,
                33,
                63,
                64,
                255,
                256,
                0x7FFF_FFFF,
                0x8000_0000,
                0x8000_0001,
                0xFFFF_FFFF,
                0xFFFF_FFFE,
                0xFFFF,
                0x8000,
                0x7F,
                0x80,
                0xFF00,
            ]),
            1 => self.below(70),
            _ => self.u32(),
        }
    }
}

const FAULT_COUNT: u32 = SRAM_BASE + 0x3FF0;
const RESUME: u32 = ROM_BASE + 0x1000;
const HARDFAULT_AT: u32 = ROM_BASE + 0x100;
const MSP_TOP: u32 = SRAM_BASE + 0xF000;
const PSP_TOP: u32 = SRAM_BASE + 0xC000;
/// Data areas the pointers aim at.
const SRAM_DATA: (u32, u32) = (SRAM_BASE, 0x3F00);
const FLASH_DATA: (u32, u32) = (FLASH_BASE, 0x0F00);
const ROM_DATA: (u32, u32) = (ROM_BASE + 0x1100, 0x0300);

/// A label: an address known once bound.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Label(usize);

#[derive(Clone, Copy)]
enum Lit {
    Word(u32),
    /// A label's address, with bit 0 set for a Thumb branch target.
    Addr(Label, bool),
}

#[derive(Clone, Copy)]
enum Fix {
    /// A conditional branch (cond) or `B` (None) at the halfword index.
    B(usize, Option<u16>, Label),
    Bl(usize, Label),
    Adr(usize, u16, Label),
    /// A literal word at the halfword index.
    Lit(usize, Lit),
}

/// Emits code at `org`, with labels, fixups and per-chunk literal pools.
struct Asm {
    org: u32,
    hw: Vec<u16>,
    labels: Vec<Option<u32>>,
    fixes: Vec<Fix>,
    /// The current pool: the `LDR`s (halfword index, rt) waiting for each literal.
    pool: Vec<(Lit, Vec<(usize, u16)>)>,
    /// Instruction starts in the current chunk (backward branch targets) and labels to bind a few instructions on.
    starts: Vec<u32>,
    ahead: Vec<(Label, u32)>,
}

impl Asm {
    fn new(org: u32) -> Self {
        Asm {
            org,
            hw: Vec::new(),
            labels: Vec::new(),
            fixes: Vec::new(),
            pool: Vec::new(),
            starts: Vec::new(),
            ahead: Vec::new(),
        }
    }

    fn here(&self) -> u32 {
        self.org + 2 * self.hw.len() as u32
    }

    fn label(&mut self) -> Label {
        self.labels.push(None);
        Label(self.labels.len() - 1)
    }

    fn bind(&mut self, l: Label) {
        self.labels[l.0] = Some(self.here());
    }

    /// Starts an instruction: binds labels whose time has come.
    fn start(&mut self) {
        let here = self.here();
        self.starts.push(here);
        let mut k = 0;
        while k < self.ahead.len() {
            if self.ahead[k].1 == 0 {
                let (l, _) = self.ahead.swap_remove(k);
                self.labels[l.0] = Some(here);
            } else {
                self.ahead[k].1 -= 1;
                k += 1;
            }
        }
    }

    fn emit(&mut self, h: u16) {
        self.start();
        self.hw.push(h);
    }

    fn emit32(&mut self, h: [u16; 2]) {
        self.start();
        self.hw.extend(h);
    }

    /// A label bound at the start of an instruction `n` instructions on (or at the end of the chunk).
    fn ahead(&mut self, n: u32) -> Label {
        let l = self.label();
        self.ahead.push((l, n));
        l
    }

    fn b_to(&mut self, cond: Option<u16>, l: Label) {
        self.start();
        self.fixes.push(Fix::B(self.hw.len(), cond, l));
        self.hw.push(0);
    }

    fn bl_to(&mut self, l: Label) {
        self.start();
        self.fixes.push(Fix::Bl(self.hw.len(), l));
        self.hw.extend([0, 0]);
    }

    fn adr_to(&mut self, rd: u16, l: Label) {
        self.start();
        self.fixes.push(Fix::Adr(self.hw.len(), rd, l));
        self.hw.push(0);
    }

    /// `LDR rt, =lit` from the current pool.
    fn ldr_lit(&mut self, rt: u16, lit: Lit) {
        self.start();
        let at = self.hw.len();
        self.hw.push(0);
        let same = |p: &Lit| match (p, &lit) {
            (Lit::Word(a), Lit::Word(b)) => a == b,
            (Lit::Addr(a, x), Lit::Addr(b, y)) => a == b && x == y,
            _ => false,
        };
        match self.pool.iter_mut().find(|(p, _)| same(p)) {
            Some((_, users)) => users.push((at, rt)),
            None => self.pool.push((lit, vec![(at, rt)])),
        }
    }

    fn word(&mut self, rt: u16, v: u32) {
        self.ldr_lit(rt, Lit::Word(v));
    }

    /// Ends a chunk: binds labels still ahead, places the pool (after `tail`, which must not fall through).
    fn end_chunk(&mut self) {
        for (l, _) in std::mem::take(&mut self.ahead) {
            self.labels[l.0] = Some(self.here());
        }
        self.starts.clear();
    }

    fn flush_pool(&mut self) {
        if self.pool.is_empty() {
            return;
        }
        if self.here() & 2 != 0 {
            self.hw.push(NOP);
        }
        for (lit, users) in std::mem::take(&mut self.pool) {
            let addr = self.here();
            for (at, rt) in users {
                let pc = self.org + 2 * at as u32;
                let off = addr - ((pc + 4) & !3);
                assert!(off <= 1020, "literal out of reach");
                self.hw[at] = ldr_lit(rt, (off / 4) as u16);
            }
            self.fixes.push(Fix::Lit(self.hw.len(), lit));
            self.hw.extend([0, 0]);
        }
    }

    fn resolve(&self, l: Label) -> u32 {
        self.labels[l.0].expect("unbound label")
    }

    /// Patches every fixup and writes the code to `bus`.
    fn finish(mut self, bus: &mut DiffBus) {
        for f in std::mem::take(&mut self.fixes) {
            match f {
                Fix::B(at, cond, l) => {
                    let pc = self.org + 2 * at as u32;
                    let off = (self.resolve(l) as i64 - (pc as i64 + 4)) / 2;
                    self.hw[at] = match cond {
                        Some(c) if (-128..128).contains(&off) => bcond(c, off as i16),
                        None if (-1024..1024).contains(&off) => b(off as i16),
                        _ => NOP,
                    };
                }
                Fix::Bl(at, l) => {
                    let pc = self.org + 2 * at as u32;
                    let [a, b] = bl(pc, self.resolve(l));
                    (self.hw[at], self.hw[at + 1]) = (a, b);
                }
                Fix::Adr(at, rd, l) => {
                    let pc = self.org + 2 * at as u32;
                    let off = self.resolve(l) as i64 - ((pc as i64 + 4) & !3);
                    self.hw[at] =
                        if (0..=1020).contains(&off) && off % 4 == 0 { adr(rd, (off / 4) as u16) } else { NOP };
                }
                Fix::Lit(at, lit) => {
                    let v = match lit {
                        Lit::Word(v) => v,
                        Lit::Addr(l, thumb) => self.resolve(l) | thumb as u32,
                    };
                    (self.hw[at], self.hw[at + 1]) = (v as u16, (v >> 16) as u16);
                }
            }
        }
        for (i, h) in self.hw.iter().enumerate() {
            bus.poke16(self.org + 2 * i as u32, *h);
        }
    }
}

/// A generated program: memory, the core's initial state and how the driver runs it.
pub(crate) struct Program {
    pub bus: DiffBus,
    pub cpu: CortexM0Plus,
    pub quantum: u32,
    pub trap_below: u32,
    pub steps: u64,
    /// Where the main loop is, for messages.
    pub code: (u32, u32),
}

struct Gen<'a> {
    rng: &'a mut Rng,
    /// Where the main program runs (stores there change it).
    code: (u32, u32),
    funcs: Vec<Label>,
    /// Functions callable from the code being generated (fewer from within a function: no recursion).
    callable: usize,
}

const LOW: [u16; 8] = [0, 1, 2, 3, 4, 5, 6, 7];

impl Gen<'_> {
    fn lo(&mut self) -> u16 {
        self.rng.pick(&LOW)
    }

    /// A pointer for loads and stores: mostly somewhere plain, sometimes anywhere interesting.
    fn pointer(&mut self, align: u32) -> u32 {
        let r = &mut *self.rng;
        let in_ = |r: &mut Rng, (base, len): (u32, u32)| (base + r.below(len)) & !(align - 1);
        match r.below(20) {
            0..=6 => in_(r, SRAM_DATA),
            7 => in_(r, (MSP_TOP - 0x200, 0x200)),
            8 | 9 => in_(r, (self.code.0, self.code.1 - self.code.0)),
            10 | 11 => in_(r, FLASH_DATA),
            12 => in_(r, ROM_DATA),
            13 => REGS_BASE + 4 * r.pick(&[0, 1, 2, 3, 4, 5, 6, 8, 9, 10]),
            14 => r.pick(&[0x3000_0000, 0x5000_0000, 0x6000_0100, 0xD000_0000, 0x1FFF_FFF0]),
            15 => r.pick(&[bus::SRAM_BASE + bus::SRAM_SIZE - 8, bus::ROM_SIZE - 4, FLASH_BASE + bus::FLASH_SIZE - 4]),
            16 => r.pick(&[ppb::SYST_CSR, ppb::SYST_CVR, ppb::NVIC_ISPR, ppb::NVIC_IPR0, ppb::ICSR, ppb::CPUID]),
            _ => in_(r, SRAM_DATA) | r.range(1, 3),
        }
    }

    /// Instructions that keep to the registers: every data processing form.
    fn alu(&mut self, a: &mut Asm) {
        let (d, m, n) = (self.lo(), self.lo(), self.lo());
        let r = &mut *self.rng;
        let ins = match r.below(24) {
            0 => lsls_i(d, m, r.below(32) as u16),
            1 => lsrs_i(d, m, r.below(32) as u16),
            2 => asrs_i(d, m, r.below(32) as u16),
            3 => adds_r(d, n, m),
            4 => subs_r(d, n, m),
            5 => adds_i3(d, n, r.below(8) as u16),
            6 => subs_i3(d, n, r.below(8) as u16),
            7 => movs(d, r.below(256) as u16),
            8 => cmp_i(d, r.below(256) as u16),
            9 => adds_i8(d, r.below(256) as u16),
            10 => subs_i8(d, r.below(256) as u16),
            11..=15 => dp(r.below(16) as u16, d, m),
            16 => ext(r.below(4) as u16, d, m),
            17 => rev(r.pick(&[0, 1, 3]), d, m),
            18 => add_rd_sp(d, r.below(256) as u16),
            // High registers: r8/r9 (loop counters) are never written, nor the SP or PC
            19 => add_hi(r.pick(&[0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12]), r.below(15) as u16),
            20 => cmp_hi(r.below(16) as u16, r.below(16) as u16),
            21 => mov_hi(r.pick(&[0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12]), r.below(16) as u16),
            22 => {
                // A value for shifts by register: 0..=40 or an edge case
                let v = r.pick(&[0, 1, 31, 32, 33, 255, 0x80, 0x7F]);
                movs(d, v)
            }
            _ => {
                let v = r.interesting();
                a.word(d, v);
                return;
            }
        };
        a.emit(ins);
    }

    /// A pointer in a register, then loads and stores through it.
    fn memory(&mut self, a: &mut Asm) {
        let base = self.lo();
        let align = self.rng.pick(&[1, 2, 4, 4]);
        let p = self.pointer(align);
        a.word(base, p);
        for _ in 0..self.rng.range(1, 4) {
            let t = self.lo();
            let r = &mut *self.rng;
            let imm = if r.chance(70) { r.below(4) as u16 } else { r.below(32) as u16 };
            let ins = match r.below(12) {
                0 => str_i(t, base, imm),
                1 => ldr_i(t, base, imm),
                2 => strb_i(t, base, imm),
                3 => ldrb_i(t, base, imm),
                4 => strh_i(t, base, imm),
                5 => ldrh_i(t, base, imm),
                _ => {
                    // Register offset: an index set just before, small or anything
                    let i = (base + 1 + r.below(7) as u16) % 8;
                    if r.chance(80) {
                        a.emit(movs(i, (r.below(16) * align) as u16));
                    }
                    let r = &mut *self.rng;
                    ldst_r(r.below(8) as u16, t, base, i)
                }
            };
            a.emit(ins);
        }
    }

    /// Writes to the test registers and the PPB with values that do something.
    fn device(&mut self, a: &mut Asm) {
        let p = self.lo();
        let v = (p + 1 + self.rng.below(7) as u16) % 8;
        let r = &mut *self.rng;
        let (addr, value) = match r.below(10) {
            0 => (REGS_BASE + reg::IRQ_SET, 1 << r.below(10)),
            1 => (REGS_BASE + reg::IRQ_CLR, 1 << r.below(10)),
            2 | 3 => (REGS_BASE + reg::SCHED, r.below(10) | r.below(200) << 5),
            4 => (REGS_BASE + reg::XIP, r.below(2)),
            5 => (REGS_BASE + reg::ATTN, 0),
            6 => (ppb::ICSR, r.pick(&[1 << 28, 1 << 27, 1 << 26, 1 << 25, 1 << 31])),
            7 => (r.pick(&[ppb::NVIC_ISPR, ppb::NVIC_ICPR, ppb::NVIC_ISER, ppb::NVIC_ICER]), 1 << r.below(10)),
            8 => (r.pick(&[ppb::SHPR2, ppb::SHPR3, ppb::NVIC_IPR0, ppb::NVIC_IPR0 + 4]), r.u32()),
            _ => (ppb::SCR, r.pick(&[0, ppb::SCR_SEVONPEND, ppb::SCR_SLEEPONEXIT])),
        };
        a.word(p, addr);
        a.word(v, value);
        let r = &mut *self.rng;
        a.emit(if r.chance(80) { str_i(v, p, 0) } else { ldr_i(v, p, 0) });
        if r.chance(30) {
            // Read the time or stall
            a.emit(ldr_i(v, p, 0));
        }
    }

    fn stack(&mut self, a: &mut Asm) {
        let t = self.lo();
        let r = &mut *self.rng;
        let ins = match r.below(8) {
            0 => str_sp(t, r.below(64) as u16),
            1 => ldr_sp(t, r.below(64) as u16),
            2 => sub_sp(r.below(16) as u16),
            3 => add_sp(r.below(16) as u16),
            4 | 5 => push(r.below(256) as u16 | 1 << r.below(8), r.chance(50)),
            _ => pop(r.below(256) as u16 | 1 << r.below(8), false),
        };
        a.emit(ins);
    }

    fn multiple(&mut self, a: &mut Asm) {
        let base = self.lo();
        let p = self.pointer(4);
        a.word(base, p);
        let r = &mut *self.rng;
        let list = r.below(256) as u16 | 1 << r.below(8);
        a.emit(if r.chance(50) { stm(base, list) } else { ldm(base, list) });
    }

    fn branch(&mut self, a: &mut Asm) {
        let r = &mut *self.rng;
        match r.below(10) {
            // Forward, conditional or not
            0..=3 => {
                // (to the instruction n - 1 after the branch)
                let l = a.ahead(r.range(1, 6));
                let cond = r.chance(80).then(|| r.below(14) as u16);
                a.b_to(cond, l);
            }
            // Back to an instruction of this chunk (loops until the flags change, or for ever)
            4 if !a.starts.is_empty() => {
                let to = r.pick(&a.starts);
                let l = a.label();
                a.labels[l.0] = Some(to);
                a.b_to(Some(r.below(14) as u16), l);
            }
            // A counted loop on r8 or r9: the body runs n times
            _ => {
                let (counter, t) = (if r.chance(50) { 8 } else { 9 }, self.lo());
                let n = self.rng.range(1, 40);
                a.emit(movs(t, n as u16));
                a.emit(mov_hi(counter, t));
                let top = a.label();
                a.bind(top);
                for _ in 0..self.rng.range(1, 6) {
                    match self.rng.below(4) {
                        0 => self.memory(a),
                        _ => self.alu(a),
                    }
                }
                let t = self.lo();
                a.emit(mov_hi(t, counter));
                a.emit(subs_i8(t, 1));
                a.emit(mov_hi(counter, t));
                a.b_to(Some(1), top);
            }
        }
    }

    fn call(&mut self, a: &mut Asm) {
        let r = &mut *self.rng;
        if self.callable == 0 || r.chance(20) {
            // Interworking branches to the next instruction: to an address with bit 0 set, or clear (faults there)
            let rm = self.lo();
            let thumb = self.rng.chance(85);
            // The instruction after the pattern's (`ahead(n)` is bound n instructions on)
            match self.rng.below(5) {
                0 => {
                    let l = a.label();
                    a.adr_to(rm, l);
                    a.emit(adds_i8(rm, thumb as u16));
                    a.emit(bx(rm));
                    if a.here() & 2 != 0 {
                        a.emit(NOP);
                    }
                    a.bind(l);
                }
                1 => {
                    let l = a.ahead(2);
                    a.ldr_lit(rm, Lit::Addr(l, thumb));
                    a.emit(bx(rm));
                }
                2 => {
                    let l = a.ahead(3);
                    a.ldr_lit(rm, Lit::Addr(l, thumb));
                    a.emit(push(1 << rm, false));
                    a.emit(pop(0, true));
                }
                3 => {
                    // MOV PC, Rm (bit 0 ignored)
                    let l = a.ahead(2);
                    a.ldr_lit(rm, Lit::Addr(l, thumb));
                    a.emit(mov_hi(15, rm));
                }
                _ => {
                    // MOV Rd, PC / ADD Rd, PC (the interpreter's)
                    a.emit(mov_hi(rm, 15));
                    a.emit(add_hi(rm, 15));
                }
            }
            return;
        }
        let f = self.funcs[self.rng.below(self.callable as u32) as usize];
        if self.rng.chance(70) {
            a.bl_to(f);
        } else {
            let rm = self.lo();
            a.ldr_lit(rm, Lit::Addr(f, true));
            a.emit(blx(rm));
        }
    }

    fn system(&mut self, a: &mut Asm) {
        let (d, s) = (self.lo(), self.lo());
        let r = &mut *self.rng;
        match r.below(22) {
            0 | 1 => a.emit(svc(r.below(256) as u16)),
            2 => a.emit(bkpt(r.below(256) as u16)),
            3 => a.emit(0xDE00 | r.below(256) as u16),
            4 => a.emit32([0xF7F0 | r.below(16) as u16, 0xA000 | r.below(0x1000) as u16]),
            5 => a.emit(CPSID),
            6 | 7 => a.emit(CPSIE),
            8 => a.emit32(msr(r.pick(&[0, 1, 2, 3, 16, 16]), s)),
            9 => {
                // CONTROL: SPSEL and nPRIV from a small value
                a.emit(movs(s, r.below(4) as u16));
                a.emit32(msr(20, s));
            }
            10 | 11 => a.emit32(mrs(d, r.pick(&[0, 1, 2, 3, 5, 6, 7, 8, 9, 16, 20]))),
            12 => a.emit32(r.pick(&[DMB, DSB, ISB])),
            13 => a.emit(r.pick(&[NOP, 0xBF10, 0xBF50, 0xBF60])),
            14 => a.emit(r.pick(&[WFI, WFE, SEV])),
            15 => a.emit(SEV),
            // Anything at all
            16 | 17 => a.emit(r.u32() as u16),
            18 => a.emit32([0xE800 | r.below(0x1800) as u16, r.u32() as u16]),
            // MSR MSP/PSP with what MRS read
            19 => {
                let sysm = r.pick(&[8, 9]);
                a.emit32(mrs(s, sysm));
                a.emit32(msr(sysm, s));
            }
            _ => a.emit(r.pick(&[CPSID, CPSIE, NOP])),
        }
    }

    /// Changes an instruction a few on, in this run of code (only does anything where the code is in SRAM).
    fn self_modify(&mut self, a: &mut Asm) {
        let p = self.lo();
        let v = (p + 1 + self.rng.below(7) as u16) % 8;
        let r = &mut *self.rng;
        let mov = movs(r.below(8) as u16, r.below(256) as u16);
        let new = r.pick(&[mov, NOP, adds_i8(0, 1), b(-2), 0xDE00]) as u32;
        // (Pushing to ROM or flash would fault on the stacking too, and lock up)
        let in_sram = self.code.0 >= SRAM_BASE;
        match r.below(if in_sram { 4 } else { 3 }) {
            // A store at or a few after the instruction following it (three on)
            0 | 1 => {
                let target = a.ahead(r.range(3, 7));
                let wide = r.below(3);
                a.ldr_lit(p, Lit::Addr(target, false));
                a.word(v, new | new << 16);
                a.emit(match wide {
                    0 => strb_i(v, p, 0),
                    1 => strh_i(v, p, 0),
                    _ => str_i(v, p, 0),
                });
            }
            // STM of new code over the instructions after it (from a word below them)
            2 => {
                let target = a.ahead(r.range(3, 6));
                let k = r.below(3);
                a.ldr_lit(p, Lit::Addr(target, false));
                a.emit(subs_i8(p, (4 + 2 * k) as u16));
                a.word(v, new | new << 16);
                a.emit(stm(p, 1 << v));
            }
            // PUSH with the SP just past the instructions after it; the SP is put back after
            _ => {
                let target = a.ahead(r.range(4, 8));
                let list = 1 << v | r.below(256) as u16 & !(1 << p);
                a.emit(mov_hi(p, 13));
                a.ldr_lit(v, Lit::Addr(target, false));
                a.emit(adds_i8(v, 8));
                a.emit(mov_hi(13, v));
                a.word(v, new | new << 16);
                a.emit(push(list, false));
                a.emit(mov_hi(13, p));
            }
        }
    }

    /// A run of random instructions.
    fn body(&mut self, a: &mut Asm, n: usize, calls: bool) {
        let start = a.hw.len();
        while a.hw.len() - start < n {
            match self.rng.below(100) {
                0..=39 => self.alu(a),
                40..=59 => self.memory(a),
                60..=63 => self.device(a),
                64..=69 => self.stack(a),
                70..=73 => self.multiple(a),
                74..=83 => self.branch(a),
                84..=88 if calls => self.call(a),
                84..=88 => self.alu(a),
                89..=95 => self.system(a),
                _ => self.self_modify(a),
            }
        }
    }
}

/// Where a handler or the main program goes.
#[derive(Clone, Copy)]
enum Place {
    Rom,
    Flash,
    Sram,
}

fn org(place: Place) -> (u32, u32) {
    match place {
        Place::Rom => (ROM_BASE + 0x2000, ROM_BASE + 0x3800),
        Place::Flash => (FLASH_BASE + 0x1000, FLASH_BASE + 0x5800),
        Place::Sram => (SRAM_BASE + 0x4000, SRAM_BASE + 0x8000),
    }
}

/// Generates program `seed`.
pub(crate) fn generate(seed: u64) -> Program {
    let mut rng = Rng::new(seed);
    let mut bus = DiffBus::new();
    for (base, len) in [SRAM_DATA, FLASH_DATA, ROM_DATA, (SRAM_BASE + 0x8000, 0x8000)] {
        for k in 0..len / 4 {
            bus.poke32(base + 4 * k, rng.interesting());
        }
    }
    let place = rng.pick(&[Place::Rom, Place::Flash, Place::Sram, Place::Sram]);
    let (lo, hi) = org(place);
    let mut a = Asm::new(lo);
    let mut g = Gen { rng: &mut rng, code: (lo, hi), funcs: Vec::new(), callable: 0 };

    // The main loop: chunks, then back to the first; then the functions
    let chunks = g.rng.range(2, 6);
    let starts: Vec<Label> = (0..=chunks).map(|_| a.label()).collect();
    let nfuncs = g.rng.range(0, 4) as usize;
    g.funcs = (0..nfuncs).map(|_| a.label()).collect();
    g.callable = nfuncs;
    for k in 0..chunks as usize {
        a.bind(starts[k]);
        let n = g.rng.range(10, 70) as usize;
        g.body(&mut a, n, true);
        a.end_chunk();
        a.b_to(None, starts[k + 1]);
        a.flush_pool();
    }
    // (Out of a B's reach from here)
    a.bind(starts[chunks as usize]);
    a.ldr_lit(7, Lit::Addr(starts[0], true));
    a.emit(bx(7));
    a.flush_pool();
    for k in 0..nfuncs {
        a.bind(g.funcs[k]);
        // Functions call only those before them
        g.callable = k;
        let list = g.rng.below(256) as u16 & 0xF0;
        let leaf = g.rng.chance(25);
        if !leaf {
            a.emit(push(list, true));
        }
        let n = g.rng.range(2, 30) as usize;
        g.body(&mut a, n, true);
        a.end_chunk();
        a.emit(if leaf { bx(14) } else { pop(list, true) });
        a.flush_pool();
    }
    assert!(a.here() <= hi, "program too long");
    let resume: Vec<u32> = starts[..chunks as usize].iter().map(|l| a.resolve(*l)).collect();
    a.finish(&mut bus);
    for k in 0..16 {
        bus.poke32(RESUME + 4 * k, resume[k as usize % resume.len()]);
    }

    // HardFault: count, resume the main loop at a chunk start (one after another), return
    let mut h = Asm::new(HARDFAULT_AT);
    let msp = h.label();
    let got = h.label();
    h.emit(movs(1, 4));
    h.emit(mov_hi(2, 14));
    h.emit(dp(8, 2, 1)); // tst r2, r1
    h.b_to(Some(0), msp);
    h.emit32(mrs(0, 9));
    h.b_to(None, got);
    h.bind(msp);
    h.emit32(mrs(0, 8));
    h.bind(got);
    h.word(3, FAULT_COUNT);
    h.emit(ldr_i(1, 3, 0));
    h.emit(adds_i8(1, 1));
    h.emit(str_i(1, 3, 0));
    h.emit(lsls_i(1, 1, 28));
    h.emit(lsrs_i(1, 1, 26));
    h.word(2, RESUME);
    h.emit(ldst_r(4, 2, 2, 1)); // ldr r2, [r2, r1]
    h.emit(str_i(2, 0, 6));
    h.emit(bx(14));
    h.flush_pool();
    h.finish(&mut bus);

    // The other handlers: NMI, SVCall, PendSV, SysTick and IRQ 0-9, each in ROM, flash or SRAM
    let mut vectors = [HARDFAULT_AT | 1; 48];
    let mut at = [(ROM_BASE + 0x400, 0), (FLASH_BASE + 0x6000, 0), (SRAM_BASE + 0x9000, 0)];
    for exc in [2, 11, 14, 15].into_iter().chain(16..26) {
        let k = g.rng.below(3) as usize;
        let mut h = Asm::new(at[k].0);
        vectors[exc] = at[k].0 | 1;
        g.code = (at[k].0, at[k].0 + 0x80);
        if exc >= 16 {
            // Level sensitive: lower the line, or the interrupt is taken again on return
            h.word(0, REGS_BASE + reg::IRQ_CLR);
            h.word(1, 1 << (exc - 16));
            h.emit(str_i(1, 0, 0));
        }
        for _ in 0..g.rng.below(6) {
            match g.rng.below(6) {
                0 => g.device(&mut h),
                1 => g.memory(&mut h),
                _ => g.alu(&mut h),
            }
        }
        h.end_chunk();
        h.emit(bx(14));
        h.flush_pool();
        at[k].0 = (h.here() + 15) & !15;
        at[k].1 += 1;
        h.finish(&mut bus);
    }
    vectors[0] = MSP_TOP;
    vectors[1] = lo | 1;
    for (e, v) in vectors.iter().enumerate() {
        bus.poke32(ROM_BASE + 4 * e as u32, *v);
    }

    // The core, as after reset, then random registers and exception configuration
    let mut cpu = CortexM0Plus::new();
    sim_core::cpu::Cpu::reset(&mut cpu, &mut bus, ROM_BASE);
    for r in 0..8 {
        cpu.r[r] = rng.interesting();
    }
    for r in 10..13 {
        cpu.r[r] = rng.interesting();
    }
    cpu.other_sp = PSP_TOP;
    let f = rng.below(16);
    (cpu.n, cpu.z, cpu.c, cpu.v) = (f & 1 != 0, f & 2 != 0, f & 4 != 0, f & 8 != 0);
    cpu.nvic.enabled = rng.below(1 << 10);
    for p in cpu.nvic.priority.iter_mut() {
        *p = rng.pick(&[0x00, 0x40, 0x80, 0xC0]);
    }
    cpu.scb.shpr2 = rng.pick(&[0, 0x4000_0000, 0x8000_0000, 0xC000_0000]);
    cpu.scb.shpr3 = rng.below(4) << 22 << 8 | rng.below(4) << 22;
    match rng.below(4) {
        0 => {}
        1 => {
            cpu.systick.rvr = rng.range(20, 600);
            cpu.systick.csr = 0b111;
        }
        _ => {
            cpu.systick.rvr = rng.range(1, 60);
            cpu.systick.csr = 0b011;
            bus.tick_period = rng.range(2, 40) as u64;
        }
    }
    let steps = 4000;
    for _ in 0..rng.below(8) {
        let t = rng.below(steps as u32 * 2) as u64;
        bus.events.push((t, rng.below(10)));
    }
    let quantum = match rng.below(10) {
        0 => 1,
        1 => 1000,
        _ => rng.range(2, 48),
    };
    let trap_below = match rng.below(10) {
        0 => HARDFAULT_AT + 0x100,
        1 if matches!(place, Place::Rom) => lo + rng.below(0x200),
        _ => 0,
    };
    Program { bus, cpu, quantum, trap_below, steps, code: (lo, hi) }
}
