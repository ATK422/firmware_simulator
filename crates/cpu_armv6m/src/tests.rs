//! Per-instruction and exception tests. Programs are hand encoded with the helpers in [`enc`] and run from a flat
//! test bus: code at `0x0000_0000`, RAM at `0x2000_0000`.

#![allow(clippy::type_complexity)]

use sim_core::bus::{Bus, BusFault, Size, mem};
use sim_core::cpu::{Cpu, StepOutcome};

use crate::{CortexM0Plus, Fault, Sleep, exception, reg};

const CODE: u32 = 0x0000_0100;
const RAM: u32 = 0x2000_0000;
const RAM_SIZE: usize = 0x1_0000;
const STACK_TOP: u32 = RAM + 0x8000;
const VECTORS: u32 = 0x0000_0000;
const HANDLER: u32 = 0x0000_1000;

struct TestBus {
    rom: Vec<u8>,
    ram: Vec<u8>,
    writes: Vec<(u32, Size, u32)>,
    /// RAM handed out for writing (`Bus::ram`) and not yet logged in `writes`: logged at the next access.
    pending: Option<(u32, u32)>,
}

impl TestBus {
    fn new() -> Self {
        TestBus { rom: vec![0; 0x1_0000], ram: vec![0; RAM_SIZE], writes: Vec::new(), pending: None }
    }
    fn region(&mut self, addr: u32) -> Option<(&mut Vec<u8>, usize)> {
        if (addr as usize) < self.rom.len() {
            Some((&mut self.rom, addr as usize))
        } else if (RAM..RAM + RAM_SIZE as u32).contains(&addr) {
            Some((&mut self.ram, (addr - RAM) as usize))
        } else {
            None
        }
    }
    fn load_code(&mut self, at: u32, code: &[u16]) {
        for (i, hw) in code.iter().enumerate() {
            mem::write(&mut self.rom, at as usize + 2 * i, Size::Half, *hw as u32);
        }
    }
    fn word(&mut self, addr: u32) -> u32 {
        self.read(addr, Size::Word).unwrap()
    }
    fn set_word(&mut self, addr: u32, v: u32) {
        let (buf, off) = self.region(addr).unwrap();
        mem::write(buf, off, Size::Word, v);
    }
}

impl Bus for TestBus {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        let (buf, off) = self.region(addr).ok_or(BusFault::Unmapped(addr))?;
        Ok(mem::read(buf, off, size))
    }
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.writes.push((addr, size, value));
        let (buf, off) = self.region(addr).ok_or(BusFault::Unmapped(addr))?;
        mem::write(buf, off, size, value);
        Ok(())
    }
}

/// Instruction encoders for the tests.
mod enc {
    pub fn lsls_i(rd: u16, rm: u16, imm: u16) -> u16 {
        imm << 6 | rm << 3 | rd
    }
    pub fn lsrs_i(rd: u16, rm: u16, imm: u16) -> u16 {
        0x0800 | imm << 6 | rm << 3 | rd
    }
    pub fn asrs_i(rd: u16, rm: u16, imm: u16) -> u16 {
        0x1000 | imm << 6 | rm << 3 | rd
    }
    pub fn adds_r(rd: u16, rn: u16, rm: u16) -> u16 {
        0x1800 | rm << 6 | rn << 3 | rd
    }
    pub fn subs_r(rd: u16, rn: u16, rm: u16) -> u16 {
        0x1A00 | rm << 6 | rn << 3 | rd
    }
    pub fn adds_i3(rd: u16, rn: u16, imm: u16) -> u16 {
        0x1C00 | imm << 6 | rn << 3 | rd
    }
    pub fn subs_i3(rd: u16, rn: u16, imm: u16) -> u16 {
        0x1E00 | imm << 6 | rn << 3 | rd
    }
    pub fn movs(rd: u16, imm: u16) -> u16 {
        0x2000 | rd << 8 | imm
    }
    pub fn cmp_i(rn: u16, imm: u16) -> u16 {
        0x2800 | rn << 8 | imm
    }
    pub fn adds_i8(rdn: u16, imm: u16) -> u16 {
        0x3000 | rdn << 8 | imm
    }
    pub fn subs_i8(rdn: u16, imm: u16) -> u16 {
        0x3800 | rdn << 8 | imm
    }
    pub fn dp(op: u16, rdn: u16, rm: u16) -> u16 {
        0x4000 | op << 6 | rm << 3 | rdn
    }
    pub fn add_hi(rdn: u16, rm: u16) -> u16 {
        0x4400 | (rdn & 8) << 4 | rm << 3 | (rdn & 7)
    }
    pub fn cmp_hi(rn: u16, rm: u16) -> u16 {
        0x4500 | (rn & 8) << 4 | rm << 3 | (rn & 7)
    }
    pub fn mov_hi(rd: u16, rm: u16) -> u16 {
        0x4600 | (rd & 8) << 4 | rm << 3 | (rd & 7)
    }
    pub fn bx(rm: u16) -> u16 {
        0x4700 | rm << 3
    }
    pub fn blx(rm: u16) -> u16 {
        0x4780 | rm << 3
    }
    pub fn ldr_lit(rt: u16, imm8: u16) -> u16 {
        0x4800 | rt << 8 | imm8
    }
    pub fn ldst_r(op: u16, rt: u16, rn: u16, rm: u16) -> u16 {
        0x5000 | op << 9 | rm << 6 | rn << 3 | rt
    }
    pub fn str_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x6000 | imm5 << 6 | rn << 3 | rt
    }
    pub fn ldr_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x6800 | imm5 << 6 | rn << 3 | rt
    }
    pub fn strb_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x7000 | imm5 << 6 | rn << 3 | rt
    }
    pub fn ldrb_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x7800 | imm5 << 6 | rn << 3 | rt
    }
    pub fn strh_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x8000 | imm5 << 6 | rn << 3 | rt
    }
    pub fn ldrh_i(rt: u16, rn: u16, imm5: u16) -> u16 {
        0x8800 | imm5 << 6 | rn << 3 | rt
    }
    pub fn str_sp(rt: u16, imm8: u16) -> u16 {
        0x9000 | rt << 8 | imm8
    }
    pub fn ldr_sp(rt: u16, imm8: u16) -> u16 {
        0x9800 | rt << 8 | imm8
    }
    pub fn adr(rd: u16, imm8: u16) -> u16 {
        0xA000 | rd << 8 | imm8
    }
    pub fn add_rd_sp(rd: u16, imm8: u16) -> u16 {
        0xA800 | rd << 8 | imm8
    }
    pub fn add_sp(imm7: u16) -> u16 {
        0xB000 | imm7
    }
    pub fn sub_sp(imm7: u16) -> u16 {
        0xB080 | imm7
    }
    pub fn ext(op: u16, rd: u16, rm: u16) -> u16 {
        0xB200 | op << 6 | rm << 3 | rd
    }
    pub fn push(list: u16, lr: bool) -> u16 {
        0xB400 | (lr as u16) << 8 | list
    }
    pub fn pop(list: u16, pc: bool) -> u16 {
        0xBC00 | (pc as u16) << 8 | list
    }
    pub fn rev(op: u16, rd: u16, rm: u16) -> u16 {
        0xBA00 | op << 6 | rm << 3 | rd
    }
    pub fn stm(rn: u16, list: u16) -> u16 {
        0xC000 | rn << 8 | list
    }
    pub fn ldm(rn: u16, list: u16) -> u16 {
        0xC800 | rn << 8 | list
    }
    pub fn bcond(cond: u16, off_halfwords: i16) -> u16 {
        0xD000 | cond << 8 | (off_halfwords as u16 & 0xFF)
    }
    pub fn b(off_halfwords: i16) -> u16 {
        0xE000 | (off_halfwords as u16 & 0x7FF)
    }
    /// BL from `from` to `to`.
    pub fn bl(from: u32, to: u32) -> [u16; 2] {
        let off = to.wrapping_sub(from + 4) as i32;
        let imm = (off >> 1) as u32;
        let s = (imm >> 23) & 1;
        let i1 = (imm >> 22) & 1;
        let i2 = (imm >> 21) & 1;
        let j1 = (!i1 ^ s) & 1;
        let j2 = (!i2 ^ s) & 1;
        [(0xF000 | s << 10 | (imm >> 11) & 0x3FF) as u16, (0xD000 | j1 << 13 | j2 << 11 | imm & 0x7FF) as u16]
    }
    pub fn msr(sysm: u16, rn: u16) -> [u16; 2] {
        [0xF380 | rn, 0x8800 | sysm]
    }
    pub fn mrs(rd: u16, sysm: u16) -> [u16; 2] {
        [0xF3EF, 0x8000 | rd << 8 | sysm]
    }
    pub const NOP: u16 = 0xBF00;
    pub const WFI: u16 = 0xBF30;
    pub const WFE: u16 = 0xBF20;
    pub const SEV: u16 = 0xBF40;
    pub const CPSID: u16 = 0xB672;
    pub const CPSIE: u16 = 0xB662;
    pub const DMB: [u16; 2] = [0xF3BF, 0x8F5F];
    pub const DSB: [u16; 2] = [0xF3BF, 0x8F4F];
    pub const ISB: [u16; 2] = [0xF3BF, 0x8F6F];
    pub fn svc(imm: u16) -> u16 {
        0xDF00 | imm
    }
    pub fn bkpt(imm: u16) -> u16 {
        0xBE00 | imm
    }
}
use enc::*;

struct Rig {
    cpu: CortexM0Plus,
    bus: TestBus,
}

impl Rig {
    fn new(code: &[u16]) -> Rig {
        let mut bus = TestBus::new();
        bus.set_word(VECTORS, STACK_TOP);
        bus.set_word(VECTORS + 4, CODE | 1);
        for e in 2..48 {
            bus.set_word(VECTORS + 4 * e, HANDLER | 1);
        }
        bus.load_code(CODE, code);
        let mut cpu = CortexM0Plus::new();
        cpu.reset(&mut bus, VECTORS);
        Rig { cpu, bus }
    }

    fn handler(&mut self, exc: u32, at: u32, code: &[u16]) {
        self.bus.set_word(VECTORS + 4 * exc, at | 1);
        self.bus.load_code(at, code);
    }

    fn step(&mut self) -> StepOutcome {
        self.cpu.step(&mut self.bus)
    }

    /// Runs `n` steps, asserting each executes.
    fn run(&mut self, n: usize) -> u32 {
        let mut total = 0;
        for _ in 0..n {
            match self.step() {
                StepOutcome::Ran(c) => total += c,
                other => panic!("unexpected {other:?} at {:#x}", self.cpu.pc()),
            }
        }
        total
    }

    fn r(&self, n: usize) -> u32 {
        self.cpu.r[n]
    }

    fn flags(&self) -> (bool, bool, bool, bool) {
        (self.cpu.n, self.cpu.z, self.cpu.c, self.cpu.v)
    }
}

/// Runs one instruction with registers preset; returns the rig for inspection.
fn one(ins: u16, regs: &[(usize, u32)]) -> Rig {
    let mut rig = Rig::new(&[ins]);
    for &(r, v) in regs {
        rig.cpu.r[r] = v;
    }
    rig.run(1);
    rig
}

fn one_with_c(ins: u16, regs: &[(usize, u32)], carry: bool) -> Rig {
    let mut rig = Rig::new(&[ins]);
    for &(r, v) in regs {
        rig.cpu.r[r] = v;
    }
    rig.cpu.c = carry;
    rig.run(1);
    rig
}

const N: bool = true;
const F: bool = false;

#[test]
fn reset_loads_sp_and_pc() {
    let rig = Rig::new(&[NOP]);
    assert_eq!(rig.r(13), STACK_TOP);
    assert_eq!(rig.cpu.pc(), CODE);
    assert_eq!(rig.cpu.ipsr(), 0);
    assert_eq!(rig.cpu.xpsr() & (1 << 24), 1 << 24, "Thumb bit");
}

#[test]
fn shift_immediate() {
    // (instruction, input, carry in, expected result, expected flags N Z C)
    let cases: &[(u16, u32, bool, u32, (bool, bool, bool))] = &[
        (lsls_i(0, 1, 0), 0x8000_0001, N, 0x8000_0001, (N, F, N)), // LSL #0 is MOVS: C unchanged
        (lsls_i(0, 1, 1), 0x8000_0001, F, 0x0000_0002, (F, F, N)),
        (lsls_i(0, 1, 31), 0x0000_0003, F, 0x8000_0000, (N, F, N)),
        (lsls_i(0, 1, 4), 0x0F00_0000, N, 0xF000_0000, (N, F, F)),
        (lsrs_i(0, 1, 1), 0x0000_0003, F, 0x0000_0001, (F, F, N)),
        (lsrs_i(0, 1, 0), 0x8000_0000, F, 0, (F, N, N)), // LSR #32
        (lsrs_i(0, 1, 0), 0x7FFF_FFFF, N, 0, (F, N, F)),
        (asrs_i(0, 1, 4), 0x8000_0010, F, 0xF800_0001, (N, F, F)),
        (asrs_i(0, 1, 0), 0x8000_0000, F, 0xFFFF_FFFF, (N, F, N)), // ASR #32
        (asrs_i(0, 1, 0), 0x4000_0000, N, 0, (F, N, F)),
    ];
    for (i, &(ins, input, cin, res, (n, z, c))) in cases.iter().enumerate() {
        let rig = one_with_c(ins, &[(1, input)], cin);
        assert_eq!(rig.r(0), res, "case {i}");
        let (fn_, fz, fc, _) = rig.flags();
        assert_eq!((fn_, fz, fc), (n, z, c), "case {i} flags");
    }
}

#[test]
fn add_sub_register_and_imm3() {
    // (instruction, rn, rm, result, N Z C V)
    let cases: &[(u16, u32, u32, u32, (bool, bool, bool, bool))] = &[
        (adds_r(0, 1, 2), 1, 2, 3, (F, F, F, F)),
        (adds_r(0, 1, 2), 0xFFFF_FFFF, 1, 0, (F, N, N, F)),
        (adds_r(0, 1, 2), 0x7FFF_FFFF, 1, 0x8000_0000, (N, F, F, N)),
        (adds_r(0, 1, 2), 0x8000_0000, 0x8000_0000, 0, (F, N, N, N)),
        (subs_r(0, 1, 2), 5, 3, 2, (F, F, N, F)),
        (subs_r(0, 1, 2), 3, 5, 0xFFFF_FFFE, (N, F, F, F)),
        (subs_r(0, 1, 2), 0x8000_0000, 1, 0x7FFF_FFFF, (F, F, N, N)),
        (subs_r(0, 1, 2), 7, 7, 0, (F, N, N, F)),
        (adds_i3(0, 1, 7), 1, 0, 8, (F, F, F, F)),
        (subs_i3(0, 1, 7), 7, 0, 0, (F, N, N, F)),
    ];
    for (i, &(ins, rn, rm, res, flags)) in cases.iter().enumerate() {
        let rig = one(ins, &[(1, rn), (2, rm)]);
        assert_eq!(rig.r(0), res, "case {i}");
        assert_eq!(rig.flags(), flags, "case {i} flags");
    }
}

#[test]
fn imm8_forms() {
    let rig = one_with_c(movs(3, 0), &[(3, 99)], true);
    assert_eq!(rig.r(3), 0);
    assert_eq!(rig.flags(), (F, N, N, F), "MOVS leaves C and V alone");
    let rig = one(movs(7, 255), &[]);
    assert_eq!(rig.r(7), 255);
    let rig = one(cmp_i(2, 10), &[(2, 10)]);
    assert_eq!(rig.flags(), (F, N, N, F));
    assert_eq!(rig.r(2), 10, "CMP does not write");
    let rig = one(cmp_i(2, 11), &[(2, 10)]);
    assert_eq!(rig.flags(), (N, F, F, F));
    let rig = one(adds_i8(4, 200), &[(4, 0xFFFF_FF00)]);
    assert_eq!(rig.r(4), 0xFFFF_FFC8);
    assert_eq!(rig.flags(), (N, F, F, F));
    let rig = one(adds_i8(4, 1), &[(4, 0xFFFF_FFFF)]);
    assert_eq!((rig.r(4), rig.flags()), (0, (F, N, N, F)));
    let rig = one(subs_i8(5, 1), &[(5, 0)]);
    assert_eq!((rig.r(5), rig.flags()), (0xFFFF_FFFF, (N, F, F, F)));
}

#[test]
fn data_processing_logic() {
    let a = 0xF0F0_1234;
    let b = 0x0FF0_00FF;
    let cases: &[(u16, u32, u32)] = &[
        (0x0, a & b, 0),  // ANDS
        (0x1, a ^ b, 0),  // EORS
        (0xC, a | b, 0),  // ORRS
        (0xE, a & !b, 0), // BICS
        (0xF, !b, 0),     // MVNS
        (0xD, a.wrapping_mul(b), 0),
    ];
    for &(op, expect, _) in cases {
        let rig = one_with_c(dp(op, 0, 1), &[(0, a), (1, b)], true);
        assert_eq!(rig.r(0), expect, "op {op:#x}");
        let (n, z, c, v) = rig.flags();
        assert_eq!((n, z), (expect >> 31 != 0, expect == 0), "op {op:#x}");
        assert!(c && !v, "op {op:#x} leaves C and V");
    }
    // TST only sets flags
    let rig = one(dp(0x8, 0, 1), &[(0, 0xF0), (1, 0x0F)]);
    assert_eq!(rig.r(0), 0xF0);
    assert!(rig.cpu.z);
}

#[test]
fn data_processing_arith() {
    // ADCS with carry in
    let rig = one_with_c(dp(0x5, 0, 1), &[(0, 1), (1, 2)], true);
    assert_eq!(rig.r(0), 4);
    let rig = one_with_c(dp(0x5, 0, 1), &[(0, 0xFFFF_FFFF), (1, 0)], true);
    assert_eq!((rig.r(0), rig.flags()), (0, (F, N, N, F)));
    // SBCS: 5 - 3 - !C
    let rig = one_with_c(dp(0x6, 0, 1), &[(0, 5), (1, 3)], false);
    assert_eq!(rig.r(0), 1);
    let rig = one_with_c(dp(0x6, 0, 1), &[(0, 5), (1, 3)], true);
    assert_eq!(rig.r(0), 2);
    // RSBS (NEG)
    let rig = one(dp(0x9, 0, 1), &[(1, 5)]);
    assert_eq!((rig.r(0), rig.flags()), (0xFFFF_FFFB, (N, F, F, F)));
    let rig = one(dp(0x9, 0, 1), &[(1, 0)]);
    assert_eq!((rig.r(0), rig.flags()), (0, (F, N, N, F)));
    let rig = one(dp(0x9, 0, 1), &[(1, 0x8000_0000)]);
    assert_eq!((rig.r(0), rig.flags()), (0x8000_0000, (N, F, F, N)));
    // CMP / CMN
    let rig = one(dp(0xA, 0, 1), &[(0, 1), (1, 2)]);
    assert_eq!(rig.flags(), (N, F, F, F));
    assert_eq!(rig.r(0), 1);
    let rig = one(dp(0xB, 0, 1), &[(0, 1), (1, 0xFFFF_FFFF)]);
    assert_eq!(rig.flags(), (F, N, N, F));
}

#[test]
fn shift_by_register() {
    // (op, value, amount, carry in, result, carry out)
    let cases: &[(u16, u32, u32, bool, u32, bool)] = &[
        (0x2, 0x8000_0001, 0, N, 0x8000_0001, N),
        (0x2, 0x8000_0001, 1, F, 2, N),
        (0x2, 0x0000_0001, 32, F, 0, N),
        (0x2, 0x0000_0001, 33, N, 0, F),
        (0x2, 0x0000_0001, 0x100, F, 1, F), // only the bottom byte counts
        (0x3, 0x8000_0001, 1, F, 0x4000_0000, N),
        (0x3, 0x8000_0000, 32, F, 0, N),
        (0x3, 0x8000_0000, 40, N, 0, F),
        (0x4, 0x8000_0000, 4, F, 0xF800_0000, F),
        (0x4, 0x8000_0000, 32, F, 0xFFFF_FFFF, N),
        (0x4, 0x4000_0000, 200, N, 0, F),
        (0x7, 0x0000_0001, 1, F, 0x8000_0000, N), // ROR
        (0x7, 0x0000_0010, 36, N, 0x0000_0001, F),
        (0x7, 0x8000_0000, 32, F, 0x8000_0000, N),
        (0x7, 0x1234_5678, 0, N, 0x1234_5678, N),
    ];
    for (i, &(op, val, amt, cin, res, cout)) in cases.iter().enumerate() {
        let rig = one_with_c(dp(op, 0, 1), &[(0, val), (1, amt)], cin);
        assert_eq!(rig.r(0), res, "case {i}");
        assert_eq!(rig.cpu.c, cout, "case {i} carry");
        assert_eq!(rig.cpu.z, res == 0, "case {i} Z");
    }
}

#[test]
fn high_register_ops() {
    let rig = one(add_hi(8, 1), &[(8, 10), (1, 5)]);
    assert_eq!(rig.r(8), 15);
    let rig = one(add_hi(13, 1), &[(1, 6)]);
    assert_eq!(rig.r(13), (STACK_TOP + 6) & !3, "SP bits 1:0 are ignored");
    let rig = one(mov_hi(12, 2), &[(2, 0xABCD)]);
    assert_eq!(rig.r(12), 0xABCD);
    let rig = one(mov_hi(0, 15), &[]);
    assert_eq!(rig.r(0), CODE + 4, "reading PC gives the instruction address + 4");
    let rig = one(cmp_hi(9, 10), &[(9, 3), (10, 3)]);
    assert!(rig.cpu.z && rig.cpu.c);
    // MOV PC, Rm branches (bit 0 ignored)
    let rig = one(mov_hi(15, 1), &[(1, 0x201)]);
    assert_eq!(rig.cpu.pc(), 0x200);
    // ADD PC, Rm
    let rig = one(add_hi(15, 1), &[(1, 0x10)]);
    assert_eq!(rig.cpu.pc(), CODE + 4 + 0x10);
}

#[test]
fn bx_and_blx() {
    let rig = one(bx(1), &[(1, 0x301)]);
    assert_eq!(rig.cpu.pc(), 0x300);
    let rig = one(blx(2), &[(2, 0x401)]);
    assert_eq!(rig.cpu.pc(), 0x400);
    assert_eq!(rig.r(14), (CODE + 2) | 1);
    // BX to an ARM-state address faults
    let mut rig = Rig::new(&[bx(1)]);
    rig.cpu.r[1] = 0x300;
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
    assert_eq!(rig.cpu.last_fault.unwrap().fault, Fault::InvalidState(0x300));
}

#[test]
fn ldr_literal_is_word_aligned() {
    // At CODE (aligned): base = CODE + 4
    let mut rig = Rig::new(&[ldr_lit(0, 1), NOP, NOP, NOP]);
    rig.bus.set_word(CODE + 8, 0xDEAD_BEEF);
    rig.run(1);
    assert_eq!(rig.r(0), 0xDEAD_BEEF);
    // At CODE + 2: PC = CODE + 6, aligned down to CODE + 4
    let mut rig = Rig::new(&[NOP, ldr_lit(1, 1), NOP, NOP]);
    rig.bus.set_word(CODE + 8, 0x1234_5678);
    rig.run(2);
    assert_eq!(rig.r(1), 0x1234_5678);
}

#[test]
fn load_store_register_offset() {
    let base = RAM + 0x100;
    let mut rig = Rig::new(&[
        ldst_r(0, 0, 1, 2), // STR r0, [r1, r2]
        ldst_r(1, 3, 1, 4), // STRH r3, [r1, r4]
        ldst_r(2, 5, 1, 6), // STRB r5, [r1, r6]
        ldst_r(4, 7, 1, 2), // LDR r7, [r1, r2]
        ldst_r(3, 0, 1, 6), // LDRSB r0, [r1, r6]
        ldst_r(7, 3, 1, 4), // LDRSH r3, [r1, r4]
        ldst_r(5, 2, 1, 4), // LDRH r2, [r1, r4]
        ldst_r(6, 4, 1, 6), // LDRB r4, [r1, r6]
    ]);
    let regs = [(0, 0x1122_3344), (1, base), (2, 0), (3, 0xFFFF_8001), (4, 4), (5, 0x0000_0080), (6, 6)];
    for (r, v) in regs {
        rig.cpu.r[r] = v;
    }
    rig.run(3);
    assert_eq!(rig.bus.word(base), 0x1122_3344);
    assert_eq!(rig.bus.word(base + 4), 0x0080_8001, "halfword at +4 and byte at +6");
    rig.run(1);
    assert_eq!(rig.r(7), 0x1122_3344);
    rig.run(1);
    assert_eq!(rig.r(0), 0xFFFF_FF80, "LDRSB sign extends");
    rig.run(1);
    assert_eq!(rig.r(3), 0xFFFF_8001, "LDRSH sign extends");
    rig.run(1);
    assert_eq!(rig.r(2), 0x8001, "LDRH zero extends");
    rig.run(1);
    assert_eq!(rig.r(4), 0x80, "LDRB zero extends");
}

#[test]
fn load_store_immediate_offset() {
    let base = RAM + 0x200;
    let mut rig = Rig::new(&[
        str_i(0, 1, 31),  // STR r0, [r1, #124]
        ldr_i(2, 1, 31),  // LDR r2, [r1, #124]
        strb_i(0, 1, 31), // STRB r0, [r1, #31]
        ldrb_i(3, 1, 31), // LDRB r3, [r1, #31]
        strh_i(0, 1, 31), // STRH r0, [r1, #62]
        ldrh_i(4, 1, 31), // LDRH r4, [r1, #62]
    ]);
    rig.cpu.r[0] = 0xCAFE_BABE;
    rig.cpu.r[1] = base;
    let cycles = rig.run(6);
    assert_eq!(rig.bus.word(base + 124), 0xCAFE_BABE);
    assert_eq!(rig.r(2), 0xCAFE_BABE);
    assert_eq!(rig.r(3), 0xBE);
    assert_eq!(rig.r(4), 0xBABE);
    assert_eq!(rig.bus.read(base + 62, Size::Half).unwrap(), 0xBABE);
    assert_eq!(cycles, 12, "loads and stores take 2 cycles");
}

#[test]
fn sp_relative_adr_and_sp_adjust() {
    let mut rig = Rig::new(&[
        str_sp(1, 2),    // STR r1, [sp, #8]
        ldr_sp(2, 2),    // LDR r2, [sp, #8]
        add_rd_sp(3, 4), // ADD r3, sp, #16
        sub_sp(4),       // SUB sp, #16
        add_sp(127),     // ADD sp, #508
        adr(4, 3),       // ADR r4, PC+12 (aligned)
    ]);
    rig.cpu.r[1] = 77;
    rig.run(5);
    assert_eq!(rig.bus.word(STACK_TOP + 8), 77);
    assert_eq!(rig.r(2), 77);
    assert_eq!(rig.r(3), STACK_TOP + 16);
    assert_eq!(rig.r(13), STACK_TOP - 16 + 508);
    rig.run(1);
    // ADR at CODE+10: Align(CODE+14, 4) = CODE+12, + 12
    assert_eq!(rig.r(4), CODE + 12 + 12);
}

#[test]
fn extend_and_reverse() {
    let v = 0x1234_8586;
    let cases: &[(u16, u32)] = &[
        (ext(0, 0, 1), 0xFFFF_8586), // SXTH
        (ext(1, 0, 1), 0xFFFF_FF86), // SXTB
        (ext(2, 0, 1), 0x0000_8586), // UXTH
        (ext(3, 0, 1), 0x0000_0086), // UXTB
        (rev(0, 0, 1), 0x8685_3412), // REV
        (rev(1, 0, 1), 0x3412_8685), // REV16
        (rev(3, 0, 1), 0xFFFF_8685), // REVSH
    ];
    for &(ins, expect) in cases {
        let rig = one(ins, &[(1, v)]);
        assert_eq!(rig.r(0), expect, "{ins:#06x}");
    }
}

#[test]
fn push_pop() {
    let mut rig = Rig::new(&[push(0b1001_0001, true), pop(0b1001_0001, false)]);
    rig.cpu.r[0] = 10;
    rig.cpu.r[4] = 14;
    rig.cpu.r[7] = 17;
    rig.cpu.r[14] = 0xEEEE;
    let c = rig.run(1);
    assert_eq!(c, 5, "PUSH is 1 + N cycles");
    assert_eq!(rig.r(13), STACK_TOP - 16);
    let sp = rig.r(13);
    assert_eq!(
        [rig.bus.word(sp), rig.bus.word(sp + 4), rig.bus.word(sp + 8), rig.bus.word(sp + 12)],
        [10, 14, 17, 0xEEEE]
    );
    rig.cpu.r[0] = 0;
    rig.cpu.r[4] = 0;
    rig.cpu.r[7] = 0;
    rig.run(1);
    assert_eq!((rig.r(0), rig.r(4), rig.r(7), rig.r(13)), (10, 14, 17, STACK_TOP - 4));
}

#[test]
fn pop_pc_interworks() {
    let mut rig = Rig::new(&[pop(0b1, true)]);
    rig.cpu.r[13] = STACK_TOP - 8;
    rig.bus.set_word(STACK_TOP - 8, 5);
    rig.bus.set_word(STACK_TOP - 4, 0x501);
    rig.run(1);
    assert_eq!(rig.r(0), 5);
    assert_eq!(rig.cpu.pc(), 0x500);
    assert_eq!(rig.r(13), STACK_TOP);
}

#[test]
fn stm_ldm() {
    let base = RAM + 0x400;
    let mut rig = Rig::new(&[stm(0, 0b0000_1110), ldm(4, 0b0011_0000), ldm(5, 0b0110_0000)]);
    rig.cpu.r[0] = base;
    rig.cpu.r[1] = 1;
    rig.cpu.r[2] = 2;
    rig.cpu.r[3] = 3;
    let c = rig.run(1);
    assert_eq!(c, 4);
    assert_eq!(rig.r(0), base + 12, "STM writes back");
    assert_eq!([rig.bus.word(base), rig.bus.word(base + 4), rig.bus.word(base + 8)], [1, 2, 3]);
    // LDM r4!, {r4, r5}: base in list, no writeback; r4 gets the loaded value
    rig.cpu.r[4] = base;
    rig.run(1);
    assert_eq!((rig.r(4), rig.r(5)), (1, 2));
    // LDM r5, {r5, r6} with r5 = base + 4 (now 2): reload
    rig.cpu.r[5] = base + 4;
    rig.run(1);
    assert_eq!((rig.r(5), rig.r(6)), (2, 3));
    // LDM r0!, {r1} writes back when base not in list
    let mut rig = Rig::new(&[ldm(0, 0b10)]);
    rig.cpu.r[0] = base;
    rig.bus.set_word(base, 42);
    rig.run(1);
    assert_eq!((rig.r(0), rig.r(1)), (base + 4, 42));
}

#[test]
fn conditional_branches() {
    // (cond, N Z C V, taken)
    let cases: &[(u16, (bool, bool, bool, bool), bool)] = &[
        (0x0, (F, N, F, F), true), // EQ
        (0x0, (F, F, F, F), false),
        (0x1, (F, F, F, F), true),  // NE
        (0x2, (F, F, N, F), true),  // CS
        (0x3, (F, F, N, F), false), // CC
        (0x4, (N, F, F, F), true),  // MI
        (0x5, (N, F, F, F), false), // PL
        (0x6, (F, F, F, N), true),  // VS
        (0x7, (F, F, F, N), false), // VC
        (0x8, (F, F, N, F), true),  // HI
        (0x8, (F, N, N, F), false),
        (0x9, (F, N, N, F), true), // LS
        (0xA, (N, F, F, N), true), // GE
        (0xA, (N, F, F, F), false),
        (0xB, (N, F, F, F), true), // LT
        (0xC, (F, F, F, F), true), // GT
        (0xC, (F, N, F, F), false),
        (0xD, (F, N, F, F), true), // LE
        (0xD, (N, F, F, N), false),
    ];
    for (i, &(cond, (n, z, c, v), taken)) in cases.iter().enumerate() {
        let mut rig = Rig::new(&[bcond(cond, -2)]);
        (rig.cpu.n, rig.cpu.z, rig.cpu.c, rig.cpu.v) = (n, z, c, v);
        let cycles = rig.run(1);
        if taken {
            assert_eq!(rig.cpu.pc(), CODE, "case {i}: B<cond> . is a loop");
            assert_eq!(cycles, 2, "case {i}");
        } else {
            assert_eq!(rig.cpu.pc(), CODE + 2, "case {i}");
            assert_eq!(cycles, 1, "case {i}");
        }
    }
}

#[test]
fn unconditional_branch_and_bl() {
    let rig = one(b(10), &[]);
    assert_eq!(rig.cpu.pc(), CODE + 4 + 20);
    let rig = one(b(-1024), &[]);
    assert_eq!(rig.cpu.pc(), (CODE + 4).wrapping_sub(2048));
    // BL forward and backward
    for target in [0x0040_0000u32, 0x80, CODE + 4] {
        let mut rig = Rig::new(&bl(CODE, target));
        let c = rig.run(1);
        assert_eq!(rig.cpu.pc(), target, "BL to {target:#x}");
        assert_eq!(rig.r(14), (CODE + 4) | 1);
        assert_eq!(c, 3);
    }
}

#[test]
fn msr_mrs() {
    let [m1, m2] = msr(0, 0); // APSR
    let [r1, r2] = mrs(1, 3); // xPSR
    let mut rig = Rig::new(&[m1, m2, r1, r2]);
    rig.cpu.r[0] = 0xF000_0000;
    let c = rig.run(1);
    assert_eq!(c, 3);
    assert_eq!(rig.flags(), (N, N, N, N));
    rig.run(1);
    assert_eq!(rig.r(1), 0xF000_0000, "xPSR read: flags, IPSR 0, T bit reads as zero");

    // PRIMASK
    let [a, b] = msr(16, 0);
    let [c1, c2] = mrs(2, 16);
    let mut rig = Rig::new(&[a, b, c1, c2]);
    rig.cpu.r[0] = 1;
    rig.run(2);
    assert!(rig.cpu.primask());
    assert_eq!(rig.r(2), 1);

    // CONTROL.SPSEL switches thread mode to the PSP
    let [a, b] = msr(9, 0); // PSP = r0
    let [c1, c2] = msr(20, 1); // CONTROL = r1
    let [d1, d2] = mrs(3, 8); // r3 = MSP
    let mut rig = Rig::new(&[a, b, c1, c2, d1, d2]);
    rig.cpu.r[0] = RAM + 0x1000;
    rig.cpu.r[1] = 2;
    rig.run(2);
    assert_eq!(rig.r(13), RAM + 0x1000, "SP is now the PSP");
    assert_eq!(rig.cpu.control(), 2);
    rig.run(1);
    assert_eq!(rig.r(3), STACK_TOP, "MSP kept its value");
}

#[test]
fn unprivileged_cannot_mask() {
    let [a, b] = msr(20, 0);
    let [c, d] = msr(16, 1);
    let mut rig = Rig::new(&[a, b, c, d, CPSID]);
    rig.cpu.r[0] = 1; // nPRIV
    rig.cpu.r[1] = 1;
    rig.run(3);
    assert!(!rig.cpu.primask(), "unprivileged writes to PRIMASK are ignored");
}

#[test]
fn cps_and_barriers() {
    let mut rig = Rig::new(&[CPSID, CPSIE, DMB[0], DMB[1], DSB[0], DSB[1], ISB[0], ISB[1]]);
    rig.run(1);
    assert!(rig.cpu.primask());
    rig.run(1);
    assert!(!rig.cpu.primask());
    assert_eq!(rig.run(3), 9, "barriers take 3 cycles");
    assert_eq!(rig.cpu.pc(), CODE + 16);
}

#[test]
fn undefined_instructions_fault() {
    for code in [&[0xDE00u16, 0][..], &[0xBF10 | 1, 0], &[0xB100, 0], &[0xF7F0, 0xA000], &[0xE800, 0x0000]] {
        let mut rig = Rig::new(code);
        rig.run(1);
        assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT, "{code:x?}");
        assert!(matches!(rig.cpu.last_fault.unwrap().fault, Fault::Undefined(_)));
    }
}

#[test]
fn unaligned_access_faults() {
    let mut rig = Rig::new(&[ldr_i(0, 1, 0)]);
    rig.cpu.r[1] = RAM + 2;
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
    assert_eq!(rig.cpu.last_fault.unwrap().fault, Fault::Unaligned(RAM + 2));
    // Stacked PC is the faulting instruction
    assert_eq!(rig.bus.word(rig.r(13) + 24), CODE);
}

#[test]
fn bus_error_faults_and_double_fault_locks_up() {
    let mut rig = Rig::new(&[ldr_i(0, 1, 0)]);
    rig.handler(exception::HARD_FAULT, HANDLER, &[ldr_i(0, 1, 0)]);
    rig.cpu.r[1] = 0x4000_0000;
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
    assert_eq!(rig.step(), StepOutcome::Lockup);
    assert!(rig.cpu.is_locked_up());
}

#[test]
fn bkpt_reports_breakpoint() {
    let mut rig = Rig::new(&[bkpt(0xAB)]);
    assert_eq!(rig.step(), StepOutcome::Breakpoint(0xAB));
    assert_eq!(rig.cpu.pc(), CODE);
    rig.cpu.escalate_breakpoint(&mut rig.bus);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
}

#[test]
fn exception_entry_stack_frame_and_return() {
    // Thread code: NOPs. IRQ 3 handler: MOVS r0, #42; BX LR
    let mut rig = Rig::new(&[NOP, NOP, NOP]);
    rig.handler(16 + 3, HANDLER, &[movs(0, 42), bx(14)]);
    for (i, v) in [(0usize, 100u32), (1, 101), (2, 102), (3, 103), (12, 112), (14, 0x1111)] {
        rig.cpu.r[i] = v;
    }
    rig.cpu.n = true;
    rig.run(1);
    rig.cpu.nvic.enabled = 1 << 3;
    rig.cpu.set_irq_lines(1 << 3);
    let c = rig.run(1);
    assert_eq!(c, 15);
    assert_eq!(rig.cpu.ipsr(), 19);
    assert_eq!(rig.cpu.pc(), HANDLER);
    assert_eq!(rig.r(14), 0xFFFF_FFF9, "EXC_RETURN: thread mode, MSP");
    let sp = rig.r(13);
    assert_eq!(sp, STACK_TOP - 32);
    let frame: Vec<u32> = (0..8).map(|i| rig.bus.word(sp + 4 * i)).collect();
    assert_eq!(frame, vec![100, 101, 102, 103, 112, 0x1111, CODE + 2, 0x8100_0000]);
    assert_eq!(rig.cpu.nvic.pending, 0, "pending cleared on entry");
    // Line still asserted while active: not re-pended
    rig.run(1);
    assert_eq!(rig.cpu.nvic.pending, 0);
    rig.cpu.set_irq_lines(0);
    rig.run(1); // BX LR
    assert_eq!(rig.cpu.ipsr(), 0);
    assert_eq!(rig.cpu.pc(), CODE + 2);
    assert_eq!(rig.r(0), 100, "r0 restored");
    assert_eq!(rig.r(13), STACK_TOP);
    assert!(rig.cpu.n, "flags restored");
}

#[test]
fn exception_stack_alignment() {
    let mut rig = Rig::new(&[NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.r[13] = STACK_TOP - 4; // misaligned to 8
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    let sp = rig.r(13);
    assert_eq!(sp, (STACK_TOP - 4 - 32) & !7);
    assert_eq!(rig.bus.word(sp + 28) & (1 << 9), 1 << 9, "stacked xPSR bit 9 records the realignment");
    rig.cpu.set_irq_lines(0);
    rig.run(1);
    assert_eq!(rig.r(13), STACK_TOP - 4, "original SP restored");
}

#[test]
fn level_interrupt_repends_after_return() {
    let mut rig = Rig::new(&[NOP, NOP, NOP, NOP]);
    rig.handler(16, HANDLER, &[NOP, bx(14)]);
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(3); // entry, NOP, BX LR
    assert_eq!(rig.cpu.ipsr(), 0);
    rig.run(1); // re-entered since the line is still high
    assert_eq!(rig.cpu.ipsr(), 16);
}

#[test]
fn primask_blocks_and_priorities_preempt() {
    let mut rig = Rig::new(&[CPSID, NOP, CPSIE, NOP, NOP]);
    rig.handler(16, HANDLER, &[NOP, NOP, NOP, bx(14)]);
    rig.handler(17, HANDLER + 0x100, &[bx(14)]);
    rig.cpu.nvic.enabled = 0b11;
    rig.cpu.nvic.priority[0] = 0x80;
    rig.cpu.nvic.priority[1] = 0x40; // more urgent
    rig.run(1); // CPSID
    rig.cpu.set_irq_lines(0b01);
    rig.run(1); // NOP, IRQ blocked
    assert_eq!(rig.cpu.ipsr(), 0);
    rig.run(1); // CPSIE
    rig.run(1); // IRQ 0 taken
    assert_eq!(rig.cpu.ipsr(), 16);
    rig.cpu.set_irq_lines(0b10);
    rig.run(1); // IRQ 1 preempts
    assert_eq!(rig.cpu.ipsr(), 17);
    assert_eq!(rig.r(14), 0xFFFF_FFF1, "EXC_RETURN: handler mode");
    rig.cpu.set_irq_lines(0);
    rig.run(1); // return to IRQ 0
    assert_eq!(rig.cpu.ipsr(), 16);
    // Lower priority does not preempt
    rig.cpu.nvic.priority[1] = 0xC0;
    rig.cpu.set_irq_lines(0b10);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 16);
}

#[test]
fn equal_priority_lowest_number_first() {
    let mut rig = Rig::new(&[NOP]);
    rig.cpu.nvic.enabled = 0b1100;
    rig.cpu.set_irq_lines(0b1100);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 18);
}

#[test]
fn psp_thread_exception_uses_psp_frame() {
    let mut rig = Rig::new(&[NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.write_reg(reg::PSP, RAM + 0x2000);
    rig.cpu.write_reg(reg::CONTROL, 2);
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    assert_eq!(rig.r(14), 0xFFFF_FFFD);
    assert_eq!(rig.cpu.psp(), RAM + 0x2000 - 32);
    assert_eq!(rig.r(13), STACK_TOP, "handler runs on the MSP");
    assert_eq!(rig.cpu.control() & 2, 0);
    rig.cpu.set_irq_lines(0);
    rig.run(1);
    assert_eq!(rig.r(13), RAM + 0x2000);
    assert_eq!(rig.cpu.control() & 2, 2);
}

#[test]
fn svc_pendsv_systick() {
    let mut rig = Rig::new(&[svc(5), NOP]);
    rig.handler(exception::SVCALL, HANDLER, &[bx(14)]);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::SVCALL);
    assert_eq!(rig.bus.word(rig.r(13) + 24), CODE + 2, "SVC returns to the next instruction");
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 0);

    let mut rig = Rig::new(&[NOP, NOP]);
    rig.cpu.scb.pendsv_pending = true;
    rig.cpu.scb.systick_pending = true;
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::PENDSV, "same priority: lower exception number first");
}

#[test]
fn svc_at_insufficient_priority_escalates() {
    let mut rig = Rig::new(&[NOP]);
    rig.handler(16, HANDLER, &[svc(1)]);
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
}

#[test]
fn systick_fires_exception() {
    let mut rig = Rig::new(&[NOP; 32]);
    rig.handler(exception::SYSTICK, HANDLER, &[NOP; 8]);
    rig.cpu.systick.rvr = 9;
    rig.cpu.systick.cvr = 5;
    rig.cpu.systick.csr = 0b111;
    rig.run(4);
    assert_eq!(rig.cpu.ipsr(), 0);
    rig.run(1); // 5th cycle wraps
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::SYSTICK);
}

#[test]
fn wfi_sleeps_until_interrupt() {
    let mut rig = Rig::new(&[WFI, movs(0, 1)]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.nvic.enabled = 1;
    rig.run(1);
    assert_eq!(rig.cpu.sleep_state(), Some(Sleep::Wfi));
    assert_eq!(rig.step(), StepOutcome::Sleeping);
    assert!(!rig.cpu.would_wake());
    rig.cpu.set_irq_lines(1);
    assert!(rig.cpu.would_wake());
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 16);
}

#[test]
fn wfi_wakes_with_primask_set_without_taking() {
    let mut rig = Rig::new(&[CPSID, WFI, movs(0, 1)]);
    rig.cpu.nvic.enabled = 1;
    rig.run(2);
    assert_eq!(rig.step(), StepOutcome::Sleeping);
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 0, "interrupt stays pending under PRIMASK");
    assert_eq!(rig.r(0), 1);
}

#[test]
fn wfi_ignores_disabled_interrupts() {
    let mut rig = Rig::new(&[WFI, NOP]);
    rig.run(1);
    rig.cpu.set_irq_lines(1);
    assert_eq!(rig.step(), StepOutcome::Sleeping);
}

#[test]
fn wfe_event_register() {
    // SEV sets the local event register, so the first WFE falls through and the second sleeps
    let mut rig = Rig::new(&[SEV, WFE, WFE, NOP]);
    rig.run(1);
    assert!(rig.cpu.take_sev());
    rig.run(1);
    assert_eq!(rig.cpu.sleep_state(), None);
    rig.run(1);
    assert_eq!(rig.cpu.sleep_state(), Some(Sleep::Wfe));
    assert_eq!(rig.step(), StepOutcome::Sleeping);
    rig.cpu.signal_event();
    rig.run(1);
    assert_eq!(rig.cpu.pc(), CODE + 8);
    assert!(!rig.cpu.event_reg, "wake-up consumes the event");
}

#[test]
fn sevonpend_wakes_wfe_on_disabled_interrupt() {
    let mut rig = Rig::new(&[WFE, NOP]);
    rig.cpu.scb.scr = crate::ppb::SCR_SEVONPEND;
    rig.run(1);
    assert_eq!(rig.step(), StepOutcome::Sleeping);
    rig.cpu.set_irq_lines(1 << 7);
    rig.run(1);
    assert_eq!(rig.cpu.pc(), CODE + 4);
}

#[test]
fn sleep_on_exit() {
    let mut rig = Rig::new(&[NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.scb.scr = crate::ppb::SCR_SLEEPONEXIT;
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    rig.cpu.set_irq_lines(0);
    rig.run(1);
    assert_eq!(rig.cpu.sleep_state(), Some(Sleep::Wfi));
}

#[test]
fn tail_chain_via_return() {
    let mut rig = Rig::new(&[NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.handler(17, HANDLER + 0x100, &[bx(14)]);
    rig.cpu.nvic.enabled = 0b11;
    rig.cpu.set_irq_lines(0b11);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 16);
    rig.cpu.set_irq_lines(0b10);
    rig.run(1); // return from 16
    rig.run(1); // enter 17
    assert_eq!(rig.cpu.ipsr(), 17);
    rig.cpu.set_irq_lines(0);
    rig.run(1);
    assert_eq!((rig.cpu.ipsr(), rig.cpu.pc(), rig.r(13)), (0, CODE, STACK_TOP));
}

#[test]
fn invalid_exc_return_faults() {
    let mut rig = Rig::new(&[NOP]);
    rig.handler(16, HANDLER, &[bx(1)]);
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    rig.cpu.r[1] = 0xFFFF_FFF5;
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
    assert_eq!(rig.cpu.last_fault.unwrap().fault, Fault::InvalidReturn(0xFFFF_FFF5));
}

#[test]
fn debugger_register_view() {
    let mut rig = Rig::new(&[NOP]);
    rig.cpu.write_reg(3, 33);
    assert_eq!(rig.cpu.read_reg(3), 33);
    assert_eq!(rig.cpu.read_reg(reg::MSP), STACK_TOP);
    rig.cpu.write_reg(reg::PC, 0x201);
    assert_eq!(rig.cpu.pc(), 0x200);
    rig.cpu.write_reg(reg::XPSR, 0x4000_0000);
    assert!(rig.cpu.z);
    assert_eq!(rig.cpu.num_regs(), reg::COUNT);
    let _ = rig.step();
}

// ------------------------------------------------------------------ batches

/// A bus whose accesses never need the machine's attention, so batches only end for CPU reasons.
struct QuietBus<'a>(&'a mut TestBus);

impl Bus for QuietBus<'_> {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.0.read(addr, size)
    }
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.0.write(addr, size, value)
    }
    fn take_attention(&mut self) -> bool {
        false
    }
}

impl Rig {
    fn batch(&mut self, max_steps: u32, budget: u64) -> crate::Batch {
        self.cpu.run_batch(&mut QuietBus(&mut self.bus), max_steps, budget, 0)
    }
}

/// A loop of loads, stores, arithmetic and branches: r0 counts down from 10, r1 accumulates through RAM.
fn loop_program() -> Vec<u16> {
    vec![
        movs(0, 10),
        ldr_lit(2, 4), // r2 = RAM (the literal at CODE + 0x14)
        movs(1, 0),
        // loop:
        str_i(0, 2, 0),
        ldr_i(3, 2, 0),
        dp(0xC, 1, 3), // orrs r1, r3
        subs_i8(0, 1),
        bcond(1, -6), // bne loop
        b(-2),        // spin
        0,
        (RAM & 0xFFFF) as u16,
        (RAM >> 16) as u16,
    ]
}

#[test]
fn batch_runs_as_steps_would() {
    let mut stepped = Rig::new(&loop_program());
    let mut batched = Rig::new(&loop_program());
    let mut cycles = 0;
    for _ in 0..40 {
        cycles += stepped.run(1);
    }
    let b = batched.batch(40, u64::MAX);
    assert_eq!(b, crate::Batch { steps: 40, cycles: cycles as u64, stop: None });
    assert_eq!(batched.cpu.r, stepped.cpu.r);
    assert_eq!(batched.cpu.cycles(), stepped.cpu.cycles());
    assert_eq!(batched.cpu.instructions(), 40);
    assert_eq!(batched.flags(), stepped.flags());
    assert_eq!(batched.bus.writes, stepped.bus.writes);
}

#[test]
fn batch_ends_at_its_budget_and_step_limit() {
    let mut rig = Rig::new(&loop_program());
    // movs (1) + ldr literal (2) + movs (1) = 4 cycles: a budget of 3 ends after the load
    let b = rig.batch(100, 3);
    assert_eq!((b.steps, b.cycles), (2, 3));
    let b = rig.batch(5, u64::MAX);
    assert_eq!(b.steps, 5);
    // A zero budget still runs one instruction, as a step would before the time check
    let b = rig.batch(100, 0);
    assert_eq!(b.steps, 1);
}

#[test]
fn batch_stops_before_an_exception_is_taken() {
    let mut rig = Rig::new(&[CPSID, NOP, NOP, CPSIE, NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.nvic.enabled = 1;
    rig.run(1);
    rig.cpu.set_irq_lines(1);
    // Masked: both NOPs and CPSIE run; the IRQ then preempts before the next NOP
    let b = rig.batch(100, u64::MAX);
    assert_eq!(b.steps, 3);
    assert_eq!(rig.cpu.ipsr(), 0);
    assert_eq!(rig.cpu.pc(), CODE + 8);
    // A batch does not take exceptions; a step does
    assert_eq!(rig.batch(100, u64::MAX).steps, 0);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), 16);
}

#[test]
fn batch_stops_when_systick_pends() {
    let mut rig = Rig::new(&[NOP; 32]);
    rig.handler(exception::SYSTICK, HANDLER, &[NOP; 8]);
    rig.cpu.systick.rvr = 9;
    rig.cpu.systick.cvr = 5;
    rig.cpu.systick.csr = 0b111;
    // The 5th cycle wraps; the exception is taken before the 6th instruction
    assert_eq!(rig.batch(100, u64::MAX).steps, 5);
    rig.run(1);
    assert_eq!(rig.cpu.ipsr(), exception::SYSTICK);
}

#[test]
fn batch_ends_after_sleep_sev_and_ppb_accesses() {
    let mut rig = Rig::new(&[NOP, SEV, NOP, WFI, NOP]);
    assert_eq!(rig.batch(100, u64::MAX).steps, 2, "SEV must reach the other core");
    assert!(rig.cpu.take_sev());
    assert_eq!(rig.batch(100, u64::MAX).steps, 2, "WFI");
    assert_eq!(rig.cpu.sleep_state(), Some(Sleep::Wfi));
    assert_eq!(rig.batch(100, u64::MAX).steps, 0, "asleep");

    // LDR r1, [r0] from the PPB (SysTick CSR)
    let mut rig = Rig::new(&[ldr_i(1, 0, 0), NOP, NOP, WFI]);
    rig.cpu.r[0] = 0xE000_E010;
    assert_eq!(rig.batch(100, u64::MAX).steps, 1);
    assert_eq!(rig.batch(100, u64::MAX).steps, 3);
}

#[test]
fn batch_ends_after_bus_attention() {
    // The default `take_attention` asks for attention after every instruction
    let mut rig = Rig::new(&[NOP, NOP, NOP]);
    assert_eq!(rig.cpu.run_batch(&mut rig.bus, 100, u64::MAX, 0).steps, 1);
}

#[test]
fn batch_stops_at_breakpoints_faults_and_trapped_addresses() {
    let mut rig = Rig::new(&[NOP, bkpt(7), NOP]);
    let b = rig.batch(100, u64::MAX);
    assert_eq!((b.steps, b.stop), (2, Some(StepOutcome::Breakpoint(7))));
    assert_eq!(rig.cpu.pc(), CODE + 2, "the PC stays on the BKPT");
    assert_eq!((rig.cpu.last_pc, rig.cpu.last_opcode), (CODE + 2, 0xBE07), "for escalate_breakpoint");

    // An undefined instruction is taken as a HardFault, which ends the batch
    let mut rig = Rig::new(&[NOP, 0xDE00, NOP]);
    let b = rig.batch(100, u64::MAX);
    assert_eq!((b.steps, b.stop), (2, None));
    assert_eq!(rig.cpu.ipsr(), exception::HARD_FAULT);
    assert_eq!(b.cycles, 1 + 15);

    // Below `trap_below` the batch hands back to single steps
    let mut rig = Rig::new(&[NOP, NOP]);
    assert_eq!(rig.cpu.run_batch(&mut QuietBus(&mut rig.bus), 100, u64::MAX, CODE + 1).steps, 0);
}

// ------------------------------------------------------------------ decoded blocks

/// A quiet bus that also serves code (ROM and RAM), so batches run decoded blocks. Counts slow accesses (none:
/// everything is plain memory) as the default does not.
struct CodeBus<'a>(&'a mut TestBus);

impl CodeBus<'_> {
    /// Logs what was written through `ram`.
    fn flush(&mut self) {
        if let Some((addr, len)) = self.0.pending.take() {
            for k in 0..len / 4 {
                let v = self.0.word(addr + 4 * k);
                self.0.writes.push((addr + 4 * k, Size::Word, v));
            }
        }
    }
}

impl Drop for CodeBus<'_> {
    fn drop(&mut self) {
        self.flush();
    }
}

impl Bus for CodeBus<'_> {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.flush();
        self.0.read(addr, size)
    }
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.flush();
        self.0.write(addr, size, value)
    }
    fn fetch16_fast(&mut self, addr: u32) -> Option<u16> {
        self.0.region(addr).filter(|(b, o)| o + 2 <= b.len()).map(|(b, o)| mem::read(b, o, Size::Half) as u16)
    }
    fn code_at(&mut self, addr: u32) -> Option<&[u8]> {
        self.0.region(addr).map(|(b, o)| &b[o..]).filter(|b| b.len() >= 2)
    }
    fn ram(&mut self, addr: u32, len: u32, write: bool) -> Option<&mut [u8]> {
        // RAM only, and logged as the word writes would be
        self.flush();
        if !(RAM..RAM + RAM_SIZE as u32).contains(&addr) || (addr - RAM + len) as usize > RAM_SIZE {
            return None;
        }
        if write {
            self.0.pending = Some((addr, len));
        }
        let off = (addr - RAM) as usize;
        Some(&mut self.0.ram[off..off + len as usize])
    }
    fn take_slow_access(&mut self) -> bool {
        false
    }
    fn take_attention(&mut self) -> bool {
        false
    }
}

/// Small deterministic generator for the tests below.
struct XorShift(u64);

impl XorShift {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 16) as u32
    }
}

/// Everything about a core but its block cache.
fn core_state(cpu: &CortexM0Plus) -> String {
    let mut c = cpu.clone();
    c.decode_blocks = true;
    format!("{c:?}")
}

/// Runs a batch on `rig` with or without decoded blocks.
fn batch_on(rig: &mut Rig, blocks: bool, max_steps: u32, budget: u64) -> crate::Batch {
    rig.cpu.decode_blocks = blocks;
    rig.cpu.run_batch(&mut CodeBus(&mut rig.bus), max_steps, budget, 0)
}

/// Random register state: plain random values, or (`mem`) addresses in RAM for loads, stores and the stack.
fn random_state(cpu: &mut CortexM0Plus, rng: &mut XorShift, mem: bool) {
    for r in 0..13 {
        cpu.r[r] =
            if mem && !rng.next().is_multiple_of(4) { RAM + 0x1000 + (rng.next() % 0x6000) * 2 } else { rng.next() };
        if rng.next().is_multiple_of(8) {
            cpu.r[r] = rng.next() % 40;
        }
    }
    cpu.r[13] = if mem { STACK_TOP - 0x100 - (rng.next() % 0x40) * 4 } else { rng.next() & !3 };
    cpu.r[14] = rng.next();
    let f = rng.next();
    (cpu.n, cpu.z, cpu.c, cpu.v) = (f & 1 != 0, f & 2 != 0, f & 4 != 0, f & 8 != 0);
}

#[test]
fn every_16_bit_encoding_runs_as_decoded_as_interpreted() {
    // One rig per way of running; both see the same programs and states, and must stay identical
    let mut interp = Rig::new(&[NOP]);
    let mut decoded = Rig::new(&[NOP]);
    interp.handler(exception::SVCALL, HANDLER, &[NOP]);
    decoded.handler(exception::SVCALL, HANDLER, &[NOP]);
    let start = interp.cpu.clone();
    let mut rng = XorShift(0x1234_5678_9ABC_DEF1);
    for ins in 0..=0xFFFFu32 {
        for trial in 0..6 {
            let mut cpu = start.clone();
            random_state(&mut cpu, &mut rng, trial % 2 == 0);
            if trial == 5 {
                // Handler mode, so that BX and POP {pc} can return from an exception
                cpu.ipsr = 16;
                cpu.active |= 1 << 16;
                cpu.r[1] = 0xFFFF_FFF9;
            }
            // The instruction, a second halfword for 32-bit encodings, then more code
            let code = [ins as u16, rng.next() as u16, movs(0, 1), adds_i8(0, 1)];
            for rig in [&mut interp, &mut decoded] {
                rig.bus.load_code(CODE, &code);
                rig.cpu = cpu.clone();
                rig.cpu.r[15] = CODE;
                rig.bus.writes.clear();
            }
            let a = batch_on(&mut interp, false, 1, u64::MAX);
            let b = batch_on(&mut decoded, true, 1, u64::MAX);
            assert_eq!(a, b, "{ins:#06x} trial {trial}");
            assert_eq!(core_state(&interp.cpu), core_state(&decoded.cpu), "{ins:#06x} trial {trial}");
            assert_eq!(interp.bus.writes, decoded.bus.writes, "{ins:#06x} trial {trial}");
        }
    }
    assert_eq!(interp.bus.ram, decoded.bus.ram);
}

#[test]
fn bl_runs_as_decoded_as_interpreted() {
    let mut rng = XorShift(99);
    for _ in 0..2000 {
        let from = CODE + (rng.next() % 0x100) * 2;
        let to = (rng.next() % 0xF000) & !1;
        let mut interp = Rig::new(&[NOP]);
        interp.bus.load_code(from, &bl(from, to));
        interp.cpu.r[15] = from;
        let mut decoded = Rig { cpu: interp.cpu.clone(), bus: TestBus { ..TestBus::new() } };
        decoded.bus.rom = interp.bus.rom.clone();
        decoded.bus.ram = interp.bus.ram.clone();
        assert_eq!(batch_on(&mut interp, false, 1, u64::MAX), batch_on(&mut decoded, true, 1, u64::MAX));
        assert_eq!(core_state(&interp.cpu), core_state(&decoded.cpu));
        assert_eq!(decoded.cpu.pc(), to);
    }
}

/// A random program of instructions that keep to RAM (registers point there) and branch around in it.
fn random_program(rng: &mut XorShift, len: usize) -> Vec<u16> {
    let mut code = Vec::with_capacity(len);
    while code.len() < len {
        let ins = match rng.next() % 16 {
            // Short branches, forward and back
            0 => bcond((rng.next() % 14) as u16, (rng.next() % 16) as i16 - 8),
            1 => b((rng.next() % 16) as i16 - 6),
            // Loads and stores relative to r6/r7 (kept pointing into RAM by not being written: rd/rt < 6)
            2 => ldr_i((rng.next() % 6) as u16, 6 + (rng.next() % 2) as u16, (rng.next() % 32) as u16),
            3 => str_i((rng.next() % 6) as u16, 6 + (rng.next() % 2) as u16, (rng.next() % 32) as u16),
            4 => strb_i((rng.next() % 6) as u16, 7, (rng.next() % 32) as u16),
            5 => ldrh_i((rng.next() % 6) as u16, 6, (rng.next() % 32) as u16),
            6 => push((rng.next() % 64) as u16 | 1, rng.next().is_multiple_of(2)),
            7 => pop((rng.next() % 64) as u16 | 1, false),
            // Data processing and the rest of the arithmetic on r0-r5
            8 => dp((rng.next() % 16) as u16, (rng.next() % 6) as u16, (rng.next() % 8) as u16),
            9 => adds_r((rng.next() % 6) as u16, (rng.next() % 8) as u16, (rng.next() % 8) as u16),
            10 => lsls_i((rng.next() % 6) as u16, (rng.next() % 8) as u16, (rng.next() % 32) as u16),
            11 => lsrs_i((rng.next() % 6) as u16, (rng.next() % 8) as u16, (rng.next() % 32) as u16),
            12 => cmp_i((rng.next() % 8) as u16, (rng.next() % 256) as u16),
            13 => subs_i8((rng.next() % 6) as u16, (rng.next() % 256) as u16),
            14 => mov_hi((rng.next() % 6) as u16, 8 + (rng.next() % 5) as u16),
            _ => add_hi(8 + (rng.next() % 5) as u16, (rng.next() % 8) as u16),
        };
        code.push(ins);
    }
    code
}

#[test]
fn random_programs_run_as_decoded_as_interpreted() {
    let mut rng = XorShift(0xDEC0_DEB1_0C55);
    for program in 0..300 {
        let code = random_program(&mut rng, 64);
        // Half the programs run from RAM, where their stores can overwrite them
        let at = if program % 2 == 0 { CODE } else { RAM + 0x800 };
        let mut interp = Rig::new(&[NOP]);
        random_state(&mut interp.cpu, &mut rng, true);
        interp.cpu.r[6] = RAM + 0x7C0 + (rng.next() % 32) * 4;
        interp.cpu.r[7] = at + (rng.next() % 32) * 2 - 0x20;
        for (i, hw) in code.iter().enumerate() {
            let (buf, off) = interp.bus.region(at + 2 * i as u32).unwrap();
            mem::write(buf, off, Size::Half, *hw as u32);
        }
        // Past the program: spin
        let end = at + 2 * code.len() as u32;
        for i in 0..16 {
            let (buf, off) = interp.bus.region(end + 2 * i).unwrap();
            mem::write(buf, off, Size::Half, b(-2) as u32);
        }
        interp.cpu.r[15] = at;
        let mut decoded = Rig { cpu: interp.cpu.clone(), bus: TestBus::new() };
        decoded.bus.rom = interp.bus.rom.clone();
        decoded.bus.ram = interp.bus.ram.clone();
        for batch in 0..60 {
            let max_steps = 1 + rng.next() % 24;
            let budget = (rng.next() % 40) as u64;
            let x = batch_on(&mut interp, false, max_steps, budget);
            let y = batch_on(&mut decoded, true, max_steps, budget);
            assert_eq!(x, y, "program {program} batch {batch}");
            assert_eq!(core_state(&interp.cpu), core_state(&decoded.cpu), "program {program} batch {batch}");
            assert_eq!(interp.bus.writes, decoded.bus.writes, "program {program} batch {batch}");
            if interp.cpu.ipsr() != 0 {
                break;
            }
        }
        assert_eq!(interp.bus.ram, decoded.bus.ram, "program {program}");
    }
}

#[test]
fn a_store_into_the_running_block_takes_effect() {
    // In RAM: r0 points at the MOVS r2, #1 two instructions on; r1 holds MOVS r2, #7
    let code = [strh_i(1, 0, 0), NOP, movs(2, 1), adds_i8(2, 1), b(-2)];
    for blocks in [false, true] {
        let mut rig = Rig::new(&[NOP]);
        for (i, hw) in code.iter().enumerate() {
            let (buf, off) = rig.bus.region(RAM + 2 * i as u32).unwrap();
            mem::write(buf, off, Size::Half, *hw as u32);
        }
        rig.cpu.r[15] = RAM;
        rig.cpu.r[0] = RAM + 4;
        // First a run that stores what is already there, so that the block is decoded and cached
        rig.cpu.r[1] = movs(2, 1) as u32;
        assert_eq!(batch_on(&mut rig, blocks, 4, u64::MAX).steps, 4);
        assert_eq!(rig.r(2), 2);
        rig.cpu.r[15] = RAM;
        rig.cpu.r[1] = movs(2, 7) as u32;
        let b = batch_on(&mut rig, blocks, 4, u64::MAX);
        assert_eq!(b.steps, 4);
        assert_eq!(rig.r(2), 8, "blocks {blocks}");
    }
}

#[test]
fn changed_code_is_decoded_again() {
    let mut rig = Rig::new(&[movs(0, 1), movs(1, 2), b(-2)]);
    rig.cpu.decode_blocks = true;
    let start = rig.cpu.clone();
    rig.cpu.run_batch(&mut CodeBus(&mut rig.bus), 3, u64::MAX, 0);
    assert_eq!((rig.r(0), rig.r(1)), (1, 2));
    // Same core (and cache), new code at the same address
    rig.bus.load_code(CODE + 2, &[movs(1, 9)]);
    let blocks = std::mem::take(&mut rig.cpu.blocks);
    rig.cpu = start;
    rig.cpu.blocks = blocks;
    rig.cpu.run_batch(&mut CodeBus(&mut rig.bus), 3, u64::MAX, 0);
    assert_eq!((rig.r(0), rig.r(1)), (1, 9));
}

#[test]
fn block_add_with_carry_matches_the_reference() {
    let edges = [0, 1, 2, 0x7FFF_FFFE, 0x7FFF_FFFF, 0x8000_0000, 0x8000_0001, 0xFFFF_FFFE, 0xFFFF_FFFF];
    let mut rng = XorShift(7);
    let mut values: Vec<u32> = edges.to_vec();
    values.extend((0..200).map(|_| rng.next()));
    for &x in &values {
        for &y in &values {
            for c in [false, true] {
                assert_eq!(crate::block::adc(x, y, c), crate::exec::add_with_carry(x, y, c), "{x:#x} {y:#x} {c}");
            }
        }
    }
}

#[test]
fn stacking_wraps_around_the_address_space() {
    // An MSP just above zero puts the frame at the top of the address space (the PPB, which takes the writes) and
    // past 0xFFFF_FFFF to address 0: the frame addresses wrap, as the core's address arithmetic does
    let mut rig = Rig::new(&[NOP, NOP]);
    rig.handler(16, HANDLER, &[bx(14)]);
    rig.cpu.r[13] = 0x10;
    rig.cpu.nvic.enabled = 1;
    rig.cpu.set_irq_lines(1);
    rig.run(1);
    assert_eq!((rig.cpu.ipsr(), rig.r(13)), (16, 0xFFFF_FFF0));
    assert_eq!(rig.bus.word(0x8), CODE, "stacked return address at frame + 24, wrapped to 0x8");
    rig.cpu.set_irq_lines(0);
    rig.run(1);
    assert_eq!((rig.cpu.ipsr(), rig.cpu.pc(), rig.r(13)), (0, CODE, 0x10));
}
