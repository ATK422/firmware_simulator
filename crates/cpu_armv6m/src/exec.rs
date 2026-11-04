//! ARMv6-M Thumb instruction execution.
//!
//! Encodings follow the ARMv6-M Architecture Reference Manual (DDI 0419), section A6. Cycle counts follow the
//! Cortex-M0+ TRM (DDI 0484), with the single cycle multiplier the RP2040 implements.

use sim_core::bus::{Bus, Size};

use crate::{CortexM0Plus, Fault, Sleep, exception};

/// `AddWithCarry` from the ARM ARM: result, carry out, overflow.
#[inline]
pub(crate) fn add_with_carry(x: u32, y: u32, carry: bool) -> (u32, bool, bool) {
    let unsigned = x as u64 + y as u64 + carry as u64;
    let result = unsigned as u32;
    let signed = x as i32 as i64 + y as i32 as i64 + carry as i64;
    (result, unsigned >> 32 != 0, signed != result as i32 as i64)
}

#[inline]
pub(crate) fn sext(value: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    ((value << shift) as i32 >> shift) as u32
}

/// Where execution continues after an instruction.
pub(crate) enum Next {
    /// The following instruction (PC + 2 or + 4).
    Sequential,
    /// A branch target, already with bit 0 cleared.
    Branch(u32),
    /// The PC was already set (exception entry or return).
    AlreadySet,
}

impl CortexM0Plus {
    #[inline]
    pub(crate) fn set_nz(&mut self, v: u32) {
        self.n = v & 0x8000_0000 != 0;
        self.z = v == 0;
    }

    #[inline]
    fn rd_reg(&self, n: u32, pcv: u32) -> u32 {
        if n == 15 { pcv } else { self.r[n as usize] }
    }

    #[inline]
    fn wr_low(&mut self, n: u32, v: u32) {
        self.r[n as usize] = v;
    }

    fn privileged(&self) -> bool {
        self.ipsr != 0 || !self.npriv
    }

    #[inline(always)]
    pub(crate) fn condition_passed(&self, cond: u32) -> bool {
        let r = match cond >> 1 {
            0 => self.z,
            1 => self.c,
            2 => self.n,
            3 => self.v,
            4 => self.c && !self.z,
            5 => self.n == self.v,
            6 => !self.z && self.n == self.v,
            _ => true,
        };
        if cond & 1 == 1 && cond != 0xF { !r } else { r }
    }

    /// Notes a taken branch in the edge map, if it is on.
    #[inline(always)]
    pub(crate) fn note_edge(&mut self, from: u32, to: u32) {
        if let Some(e) = self.edges.as_deref_mut() {
            e.mark(from, to);
        }
    }

    /// `BXWritePC`: interworking branch that also performs exception return from handler mode.
    pub(crate) fn bx_write_pc<B: Bus + ?Sized>(&mut self, bus: &mut B, target: u32) -> Result<(Next, u32), Fault> {
        if self.ipsr != 0 && target & 0xF000_0000 == 0xF000_0000 {
            let c = self.exception_return(bus, target)?;
            return Ok((Next::AlreadySet, c));
        }
        if target & 1 == 0 {
            return Err(Fault::InvalidState(target));
        }
        Ok((Next::Branch(target & !1), 0))
    }

    /// Executes the instruction at the PC. Returns the cycles it took. With `TRACK` the instruction's address and
    /// encoding are kept in `last_pc`/`last_opcode` (for tracing); batches leave them alone.
    pub(crate) fn execute<B: Bus + ?Sized, const TRACK: bool>(&mut self, bus: &mut B) -> Result<u32, Fault> {
        let pc = self.r[15];
        let ins = match bus.fetch16_fast(pc) {
            Some(v) => v,
            None => bus.fetch16(pc)?,
        } as u32;
        if TRACK {
            self.last_pc = pc;
            self.last_opcode = ins;
        }
        let pcv = pc.wrapping_add(4);
        let mut width = 2;
        let mut cycles = 1;
        let mut next = Next::Sequential;

        match ins >> 11 {
            // LSLS Rd, Rm, #imm5
            0b00000 => {
                let imm = (ins >> 6) & 31;
                let v = self.r[((ins >> 3) & 7) as usize];
                let res = if imm == 0 {
                    v
                } else {
                    self.c = (v >> (32 - imm)) & 1 != 0;
                    v << imm
                };
                self.set_nz(res);
                self.wr_low(ins & 7, res);
            }
            // LSRS Rd, Rm, #imm5 (0 means 32)
            0b00001 => {
                let imm = (ins >> 6) & 31;
                let v = self.r[((ins >> 3) & 7) as usize];
                let res = if imm == 0 {
                    self.c = v >> 31 != 0;
                    0
                } else {
                    self.c = (v >> (imm - 1)) & 1 != 0;
                    v >> imm
                };
                self.set_nz(res);
                self.wr_low(ins & 7, res);
            }
            // ASRS Rd, Rm, #imm5 (0 means 32)
            0b00010 => {
                let imm = (ins >> 6) & 31;
                let v = self.r[((ins >> 3) & 7) as usize];
                let res = if imm == 0 {
                    self.c = v >> 31 != 0;
                    ((v as i32) >> 31) as u32
                } else {
                    self.c = (v >> (imm - 1)) & 1 != 0;
                    ((v as i32) >> imm) as u32
                };
                self.set_nz(res);
                self.wr_low(ins & 7, res);
            }
            // ADDS/SUBS Rd, Rn, Rm / #imm3
            0b00011 => {
                let rn = self.r[((ins >> 3) & 7) as usize];
                let field = (ins >> 6) & 7;
                let op2 = if ins & (1 << 10) != 0 { field } else { self.r[field as usize] };
                let (res, c, v) =
                    if ins & (1 << 9) != 0 { add_with_carry(rn, !op2, true) } else { add_with_carry(rn, op2, false) };
                self.set_nz(res);
                self.c = c;
                self.v = v;
                self.wr_low(ins & 7, res);
            }
            // MOVS Rd, #imm8
            0b00100 => {
                let imm = ins & 0xFF;
                self.set_nz(imm);
                self.wr_low((ins >> 8) & 7, imm);
            }
            // CMP Rn, #imm8
            0b00101 => {
                let (res, c, v) = add_with_carry(self.r[((ins >> 8) & 7) as usize], !(ins & 0xFF), true);
                self.set_nz(res);
                self.c = c;
                self.v = v;
            }
            // ADDS Rdn, #imm8
            0b00110 => {
                let rdn = (ins >> 8) & 7;
                let (res, c, v) = add_with_carry(self.r[rdn as usize], ins & 0xFF, false);
                self.set_nz(res);
                self.c = c;
                self.v = v;
                self.wr_low(rdn, res);
            }
            // SUBS Rdn, #imm8
            0b00111 => {
                let rdn = (ins >> 8) & 7;
                let (res, c, v) = add_with_carry(self.r[rdn as usize], !(ins & 0xFF), true);
                self.set_nz(res);
                self.c = c;
                self.v = v;
                self.wr_low(rdn, res);
            }
            0b01000 => {
                if ins & (1 << 10) == 0 {
                    self.data_processing(ins);
                } else {
                    let (n, c) = self.special_data(bus, ins, pcv)?;
                    next = n;
                    cycles = c;
                }
            }
            // LDR Rt, [PC, #imm8*4]
            0b01001 => {
                let addr = (pcv & !3).wrapping_add((ins & 0xFF) * 4);
                let v = self.read_mem(bus, addr, Size::Word)?;
                self.wr_low((ins >> 8) & 7, v);
                cycles = 2;
            }
            // Load/store with register offset
            0b01010 | 0b01011 => {
                let addr = self.r[((ins >> 3) & 7) as usize].wrapping_add(self.r[((ins >> 6) & 7) as usize]);
                let rt = ins & 7;
                cycles = 2;
                match (ins >> 9) & 7 {
                    0 => self.write_mem(bus, addr, Size::Word, self.r[rt as usize])?,
                    1 => self.write_mem(bus, addr, Size::Half, self.r[rt as usize])?,
                    2 => self.write_mem(bus, addr, Size::Byte, self.r[rt as usize])?,
                    3 => {
                        let v = self.read_mem(bus, addr, Size::Byte)?;
                        self.wr_low(rt, sext(v, 8));
                    }
                    4 => {
                        let v = self.read_mem(bus, addr, Size::Word)?;
                        self.wr_low(rt, v);
                    }
                    5 => {
                        let v = self.read_mem(bus, addr, Size::Half)?;
                        self.wr_low(rt, v);
                    }
                    6 => {
                        let v = self.read_mem(bus, addr, Size::Byte)?;
                        self.wr_low(rt, v);
                    }
                    _ => {
                        let v = self.read_mem(bus, addr, Size::Half)?;
                        self.wr_low(rt, sext(v, 16));
                    }
                }
            }
            // STR/LDR/STRB/LDRB Rt, [Rn, #imm5]
            0b01100..=0b01111 => {
                let imm = (ins >> 6) & 31;
                let base = self.r[((ins >> 3) & 7) as usize];
                let rt = ins & 7;
                cycles = 2;
                match (ins >> 11) & 3 {
                    0 => self.write_mem(bus, base.wrapping_add(imm * 4), Size::Word, self.r[rt as usize])?,
                    1 => {
                        let v = self.read_mem(bus, base.wrapping_add(imm * 4), Size::Word)?;
                        self.wr_low(rt, v);
                    }
                    2 => self.write_mem(bus, base.wrapping_add(imm), Size::Byte, self.r[rt as usize])?,
                    _ => {
                        let v = self.read_mem(bus, base.wrapping_add(imm), Size::Byte)?;
                        self.wr_low(rt, v);
                    }
                }
            }
            // STRH/LDRH Rt, [Rn, #imm5*2]
            0b10000 | 0b10001 => {
                let addr = self.r[((ins >> 3) & 7) as usize].wrapping_add(((ins >> 6) & 31) * 2);
                let rt = ins & 7;
                cycles = 2;
                if ins & (1 << 11) == 0 {
                    self.write_mem(bus, addr, Size::Half, self.r[rt as usize])?;
                } else {
                    let v = self.read_mem(bus, addr, Size::Half)?;
                    self.wr_low(rt, v);
                }
            }
            // STR/LDR Rt, [SP, #imm8*4]
            0b10010 | 0b10011 => {
                let addr = self.r[13].wrapping_add((ins & 0xFF) * 4);
                let rt = (ins >> 8) & 7;
                cycles = 2;
                if ins & (1 << 11) == 0 {
                    self.write_mem(bus, addr, Size::Word, self.r[rt as usize])?;
                } else {
                    let v = self.read_mem(bus, addr, Size::Word)?;
                    self.wr_low(rt, v);
                }
            }
            // ADR Rd, label
            0b10100 => self.wr_low((ins >> 8) & 7, (pcv & !3).wrapping_add((ins & 0xFF) * 4)),
            // ADD Rd, SP, #imm8*4
            0b10101 => self.wr_low((ins >> 8) & 7, self.r[13].wrapping_add((ins & 0xFF) * 4)),
            0b10110 | 0b10111 => {
                let (n, c) = self.misc(bus, ins)?;
                next = n;
                cycles = c;
            }
            // STM Rn!, {list}
            0b11000 => {
                let list = ins & 0xFF;
                if list == 0 {
                    return Err(Fault::Undefined(ins));
                }
                cycles = self.stm(bus, (ins >> 8) & 7, list)?;
            }
            // LDM Rn{!}, {list}
            0b11001 => {
                let list = ins & 0xFF;
                if list == 0 {
                    return Err(Fault::Undefined(ins));
                }
                cycles = self.ldm(bus, (ins >> 8) & 7, list)?;
            }
            // B<cond>, UDF, SVC
            0b11010 | 0b11011 => {
                let cond = (ins >> 8) & 0xF;
                match cond {
                    0xE => return Err(Fault::Undefined(ins)),
                    0xF => {
                        let ret = pc.wrapping_add(2);
                        if self.exception_priority(exception::SVCALL) < self.execution_priority() {
                            cycles = self.exception_entry(bus, exception::SVCALL, ret)?;
                            next = Next::AlreadySet;
                        } else {
                            // SVC at insufficient priority escalates to HardFault
                            return Err(Fault::Undefined(ins));
                        }
                    }
                    _ => {
                        if self.condition_passed(cond) {
                            next = Next::Branch(pcv.wrapping_add(sext(ins & 0xFF, 8) << 1));
                            cycles = 2;
                        }
                    }
                }
            }
            // B label
            0b11100 => {
                next = Next::Branch(pcv.wrapping_add(sext(ins & 0x7FF, 11) << 1));
                cycles = 2;
            }
            // 32-bit encodings
            0b11101..=0b11111 => {
                let hw2 = bus.fetch16(pc.wrapping_add(2))? as u32;
                if TRACK {
                    self.last_opcode = ins << 16 | hw2;
                }
                width = 4;
                let (n, c) = self.execute32(ins, hw2, pc)?;
                next = n;
                cycles = c;
            }
            _ => unreachable!(),
        }

        match next {
            Next::Sequential => self.r[15] = pc.wrapping_add(width),
            Next::Branch(t) => {
                self.r[15] = t;
                self.note_edge(pc, t);
            }
            Next::AlreadySet => {}
        }
        Ok(cycles)
    }

    #[inline(always)]
    fn data_processing(&mut self, ins: u32) {
        let rdn = ins & 7;
        let rm = self.r[((ins >> 3) & 7) as usize];
        let dn = self.r[rdn as usize];
        let shift_amount = rm & 0xFF;
        match (ins >> 6) & 0xF {
            // ANDS
            0x0 => {
                let r = dn & rm;
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // EORS
            0x1 => {
                let r = dn ^ rm;
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // LSLS (register)
            0x2 => {
                let r = match shift_amount {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (32 - shift_amount)) & 1 != 0;
                        dn << shift_amount
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
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // LSRS (register)
            0x3 => {
                let r = match shift_amount {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (shift_amount - 1)) & 1 != 0;
                        dn >> shift_amount
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
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // ASRS (register)
            0x4 => {
                let r = match shift_amount {
                    0 => dn,
                    1..=31 => {
                        self.c = (dn >> (shift_amount - 1)) & 1 != 0;
                        ((dn as i32) >> shift_amount) as u32
                    }
                    _ => {
                        self.c = dn >> 31 != 0;
                        ((dn as i32) >> 31) as u32
                    }
                };
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // ADCS
            0x5 => {
                let (r, c, v) = add_with_carry(dn, rm, self.c);
                self.set_nz(r);
                self.c = c;
                self.v = v;
                self.wr_low(rdn, r);
            }
            // SBCS
            0x6 => {
                let (r, c, v) = add_with_carry(dn, !rm, self.c);
                self.set_nz(r);
                self.c = c;
                self.v = v;
                self.wr_low(rdn, r);
            }
            // RORS
            0x7 => {
                let r = if shift_amount == 0 {
                    dn
                } else {
                    let r = dn.rotate_right(shift_amount & 31);
                    self.c = r >> 31 != 0;
                    r
                };
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // TST
            0x8 => self.set_nz(dn & rm),
            // RSBS Rd, Rn, #0
            0x9 => {
                let (r, c, v) = add_with_carry(!rm, 0, true);
                self.set_nz(r);
                self.c = c;
                self.v = v;
                self.wr_low(rdn, r);
            }
            // CMP
            0xA => {
                let (r, c, v) = add_with_carry(dn, !rm, true);
                self.set_nz(r);
                self.c = c;
                self.v = v;
            }
            // CMN
            0xB => {
                let (r, c, v) = add_with_carry(dn, rm, false);
                self.set_nz(r);
                self.c = c;
                self.v = v;
            }
            // ORRS
            0xC => {
                let r = dn | rm;
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // MULS
            0xD => {
                let r = dn.wrapping_mul(rm);
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // BICS
            0xE => {
                let r = dn & !rm;
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
            // MVNS
            _ => {
                let r = !rm;
                self.set_nz(r);
                self.wr_low(rdn, r);
            }
        }
    }

    /// ADD/CMP/MOV with high registers, BX and BLX.
    fn special_data<B: Bus + ?Sized>(&mut self, bus: &mut B, ins: u32, pcv: u32) -> Result<(Next, u32), Fault> {
        let rm = (ins >> 3) & 0xF;
        let rd = (ins & 7) | ((ins >> 4) & 8);
        match (ins >> 8) & 3 {
            // ADD Rdn, Rm
            0 => {
                let r = self.rd_reg(rd, pcv).wrapping_add(self.rd_reg(rm, pcv));
                if rd == 15 {
                    return Ok((Next::Branch(r & !1), 2));
                }
                self.write_any(rd, r);
                Ok((Next::Sequential, 1))
            }
            // CMP Rn, Rm
            1 => {
                let (r, c, v) = add_with_carry(self.rd_reg(rd, pcv), !self.rd_reg(rm, pcv), true);
                self.set_nz(r);
                self.c = c;
                self.v = v;
                Ok((Next::Sequential, 1))
            }
            // MOV Rd, Rm
            2 => {
                let r = self.rd_reg(rm, pcv);
                if rd == 15 {
                    return Ok((Next::Branch(r & !1), 2));
                }
                self.write_any(rd, r);
                Ok((Next::Sequential, 1))
            }
            // BX Rm / BLX Rm
            _ => {
                if ins & 7 != 0 {
                    return Err(Fault::Undefined(ins));
                }
                let target = self.rd_reg(rm, pcv);
                if ins & (1 << 7) != 0 {
                    if rm == 15 {
                        return Err(Fault::Undefined(ins));
                    }
                    self.r[14] = (pcv - 2) | 1;
                }
                let (n, c) = self.bx_write_pc(bus, target)?;
                Ok((n, 2 + c))
            }
        }
    }

    /// `STM Rn!, {list}` (`list` not empty): the cycles taken.
    #[inline(always)]
    pub(crate) fn stm<B: Bus + ?Sized>(&mut self, bus: &mut B, rn: u32, list: u32) -> Result<u32, Fault> {
        let rn = rn as usize;
        let mut addr = self.r[rn];
        for i in 0..8 {
            if list & (1 << i) != 0 {
                self.write_mem(bus, addr, Size::Word, self.r[i])?;
                addr = addr.wrapping_add(4);
            }
        }
        self.r[rn] = addr;
        Ok(1 + list.count_ones())
    }

    /// `LDM Rn{!}, {list}` (`list` not empty): the cycles taken.
    #[inline(always)]
    pub(crate) fn ldm<B: Bus + ?Sized>(&mut self, bus: &mut B, rn: u32, list: u32) -> Result<u32, Fault> {
        let rn = rn as usize;
        let mut addr = self.r[rn];
        let mut vals = [0u32; 8];
        for (i, val) in vals.iter_mut().enumerate() {
            if list & (1 << i) != 0 {
                *val = self.read_mem(bus, addr, Size::Word)?;
                addr = addr.wrapping_add(4);
            }
        }
        for (i, val) in vals.iter().enumerate() {
            if list & (1 << i) != 0 {
                self.r[i] = *val;
            }
        }
        if list & (1 << rn) == 0 {
            self.r[rn] = addr;
        }
        Ok(1 + list.count_ones())
    }

    /// `PUSH {list}` (bit 14 for LR, `list` not empty): the cycles taken.
    #[inline(always)]
    pub(crate) fn push<B: Bus + ?Sized>(&mut self, bus: &mut B, list: u32) -> Result<u32, Fault> {
        let count = list.count_ones();
        let mut addr = self.r[13].wrapping_sub(4 * count);
        let start = addr;
        for i in 0..15 {
            if list & (1 << i) != 0 {
                self.write_mem(bus, addr, Size::Word, self.r[i])?;
                addr = addr.wrapping_add(4);
            }
        }
        self.r[13] = start;
        Ok(1 + count)
    }

    /// `POP {list}` (bit 15 for the PC, `list` not empty).
    #[inline(always)]
    pub(crate) fn pop<B: Bus + ?Sized>(&mut self, bus: &mut B, list: u32) -> Result<(Next, u32), Fault> {
        let count = list.count_ones();
        let mut addr = self.r[13];
        let mut vals = [0u32; 16];
        for (i, val) in vals.iter_mut().enumerate() {
            if list & (1 << i) != 0 {
                *val = self.read_mem(bus, addr, Size::Word)?;
                addr = addr.wrapping_add(4);
            }
        }
        for (i, val) in vals.iter().enumerate().take(8) {
            if list & (1 << i) != 0 {
                self.r[i] = *val;
            }
        }
        self.r[13] = addr;
        if list & (1 << 15) != 0 {
            let (n, c) = self.bx_write_pc(bus, vals[15])?;
            return Ok((n, 3 + count + c));
        }
        Ok((Next::Sequential, 1 + count))
    }

    pub(crate) fn write_any(&mut self, n: u32, v: u32) {
        if n == 13 {
            self.r[13] = v & !3;
        } else {
            self.r[n as usize] = v;
        }
    }

    /// The miscellaneous 16-bit instructions (`1011 xxxx xxxx xxxx`).
    fn misc<B: Bus + ?Sized>(&mut self, bus: &mut B, ins: u32) -> Result<(Next, u32), Fault> {
        match (ins >> 8) & 0xF {
            // ADD/SUB SP, SP, #imm7*4
            0x0 => {
                let imm = (ins & 0x7F) * 4;
                self.r[13] =
                    if ins & (1 << 7) != 0 { self.r[13].wrapping_sub(imm) } else { self.r[13].wrapping_add(imm) };
            }
            // SXTH, SXTB, UXTH, UXTB
            0x2 => {
                let m = self.r[((ins >> 3) & 7) as usize];
                let r = match (ins >> 6) & 3 {
                    0 => sext(m & 0xFFFF, 16),
                    1 => sext(m & 0xFF, 8),
                    2 => m & 0xFFFF,
                    _ => m & 0xFF,
                };
                self.wr_low(ins & 7, r);
            }
            // PUSH {list, lr}
            0x4 | 0x5 => {
                let list = (ins & 0xFF) | ((ins & 0x100) << 6);
                if list == 0 {
                    return Err(Fault::Undefined(ins));
                }
                return Ok((Next::Sequential, self.push(bus, list)?));
            }
            // CPSIE i / CPSID i
            0x6 => {
                if ins & 0xFFEF != 0xB662 {
                    return Err(Fault::Undefined(ins));
                }
                if self.privileged() {
                    self.primask = ins & (1 << 4) != 0;
                    self.exc_changed = true;
                }
            }
            // REV, REV16, REVSH
            0xA => {
                let m = self.r[((ins >> 3) & 7) as usize];
                let r = match (ins >> 6) & 3 {
                    0 => m.swap_bytes(),
                    1 => ((m & 0x00FF_00FF) << 8) | ((m & 0xFF00_FF00) >> 8),
                    3 => sext(((m & 0xFF) << 8) | ((m >> 8) & 0xFF), 16),
                    _ => return Err(Fault::Undefined(ins)),
                };
                self.wr_low(ins & 7, r);
            }
            // POP {list, pc}
            0xC | 0xD => {
                let list = (ins & 0xFF) | ((ins & 0x100) << 7);
                if list == 0 {
                    return Err(Fault::Undefined(ins));
                }
                return self.pop(bus, list);
            }
            // BKPT #imm8
            0xE => return Err(Fault::Breakpoint(ins as u8)),
            // Hints
            0xF => {
                if ins & 0xF != 0 {
                    // IT is not part of ARMv6-M
                    return Err(Fault::Undefined(ins));
                }
                match (ins >> 4) & 0xF {
                    // NOP, YIELD
                    0 | 1 => {}
                    // WFE
                    2 => {
                        if self.event_reg {
                            self.event_reg = false;
                        } else {
                            self.sleep = Some(Sleep::Wfe);
                            self.batch_break = true;
                        }
                        return Ok((Next::Sequential, 2));
                    }
                    // WFI
                    3 => {
                        self.sleep = Some(Sleep::Wfi);
                        self.batch_break = true;
                        return Ok((Next::Sequential, 2));
                    }
                    // SEV
                    4 => {
                        self.sev_out = true;
                        self.batch_break = true;
                        self.event_reg = true;
                    }
                    // Other hints execute as NOP
                    _ => {}
                }
            }
            _ => return Err(Fault::Undefined(ins)),
        }
        Ok((Next::Sequential, 1))
    }

    fn execute32(&mut self, hw1: u32, hw2: u32, pc: u32) -> Result<(Next, u32), Fault> {
        let ins = hw1 << 16 | hw2;
        if hw1 >> 11 == 0b11110 && hw2 & 0xD000 == 0xD000 {
            // BL label
            let s = (hw1 >> 10) & 1;
            let j1 = (hw2 >> 13) & 1;
            let j2 = (hw2 >> 11) & 1;
            let i1 = !(j1 ^ s) & 1;
            let i2 = !(j2 ^ s) & 1;
            let imm = s << 24 | i1 << 23 | i2 << 22 | (hw1 & 0x3FF) << 12 | (hw2 & 0x7FF) << 1;
            let next_pc = pc.wrapping_add(4);
            self.r[14] = next_pc | 1;
            return Ok((Next::Branch(next_pc.wrapping_add(sext(imm, 25))), 3));
        }
        if hw1 & 0xFFF0 == 0xF380 && hw2 & 0xFF00 == 0x8800 {
            // MSR spec_reg, Rn
            let rn = (hw1 & 0xF) as usize;
            if rn == 13 || rn == 15 {
                return Err(Fault::Undefined(ins));
            }
            let v = self.r[rn];
            let sysm = hw2 & 0xFF;
            match sysm {
                0..=7 => {
                    if sysm & 4 == 0 {
                        self.set_apsr(v);
                    }
                }
                8 if self.privileged() => self.set_msp(v),
                9 if self.privileged() => self.set_psp(v),
                16 if self.privileged() => {
                    self.primask = v & 1 != 0;
                    self.exc_changed = true;
                }
                20 if self.privileged() => {
                    self.npriv = v & 1 != 0;
                    if self.ipsr == 0 {
                        self.set_mode(0, v & 2 != 0);
                    }
                }
                8 | 9 | 16 | 20 => {}
                _ => return Err(Fault::Undefined(ins)),
            }
            return Ok((Next::Sequential, 3));
        }
        if hw1 == 0xF3EF && hw2 & 0xF000 == 0x8000 {
            // MRS Rd, spec_reg
            let rd = ((hw2 >> 8) & 0xF) as usize;
            if rd == 13 || rd == 15 {
                return Err(Fault::Undefined(ins));
            }
            let sysm = hw2 & 0xFF;
            let v = match sysm {
                0..=7 => {
                    let mut v = 0;
                    if sysm & 1 != 0 {
                        v |= self.ipsr;
                    }
                    if sysm & 4 == 0 {
                        v |= self.xpsr() & 0xF000_0000;
                    }
                    v
                }
                8 => self.msp(),
                9 => self.psp(),
                16 => self.primask as u32,
                20 => self.control(),
                _ => return Err(Fault::Undefined(ins)),
            };
            self.r[rd] = v;
            return Ok((Next::Sequential, 3));
        }
        if hw1 == 0xF3BF && hw2 & 0xFF00 == 0x8F00 {
            // DSB (4), DMB (5), ISB (6); barriers are no-ops for an in-order interpreter
            return match (hw2 >> 4) & 0xF {
                4..=6 => Ok((Next::Sequential, 3)),
                _ => Err(Fault::Undefined(ins)),
            };
        }
        // UDF.W and everything else
        Err(Fault::Undefined(ins))
    }
}
