//! Single-cycle IO block (SIO, datasheet 2.3.1): per-core CPUID, GPIO registers, the inter-core FIFOs, the 32
//! spinlocks, the per-core hardware dividers and interpolators.
//!
//! The divider returns its results immediately (READY is always set); programs that respect the 8-cycle latency
//! see the same values as on hardware.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;

pub const CPUID: u32 = 0x000;
pub const GPIO_IN: u32 = 0x004;
pub const GPIO_HI_IN: u32 = 0x008;
pub const GPIO_OUT: u32 = 0x010;
pub const GPIO_OUT_SET: u32 = 0x014;
pub const GPIO_OUT_CLR: u32 = 0x018;
pub const GPIO_OUT_XOR: u32 = 0x01C;
pub const GPIO_OE: u32 = 0x020;
pub const GPIO_OE_SET: u32 = 0x024;
pub const GPIO_OE_CLR: u32 = 0x028;
pub const GPIO_OE_XOR: u32 = 0x02C;
pub const GPIO_HI_OUT: u32 = 0x030;
pub const GPIO_HI_OUT_SET: u32 = 0x034;
pub const GPIO_HI_OUT_CLR: u32 = 0x038;
pub const GPIO_HI_OUT_XOR: u32 = 0x03C;
pub const GPIO_HI_OE: u32 = 0x040;
pub const GPIO_HI_OE_SET: u32 = 0x044;
pub const GPIO_HI_OE_CLR: u32 = 0x048;
pub const GPIO_HI_OE_XOR: u32 = 0x04C;
pub const FIFO_ST: u32 = 0x050;
pub const FIFO_WR: u32 = 0x054;
pub const FIFO_RD: u32 = 0x058;
pub const SPINLOCK_ST: u32 = 0x05C;
pub const DIV_UDIVIDEND: u32 = 0x060;
pub const DIV_UDIVISOR: u32 = 0x064;
pub const DIV_SDIVIDEND: u32 = 0x068;
pub const DIV_SDIVISOR: u32 = 0x06C;
pub const DIV_QUOTIENT: u32 = 0x070;
pub const DIV_REMAINDER: u32 = 0x074;
pub const DIV_CSR: u32 = 0x078;
pub const INTERP0: u32 = 0x080;
pub const INTERP1: u32 = 0x0C0;
pub const SPINLOCK0: u32 = 0x100;
pub const SPINLOCK31: u32 = 0x17C;

const BANK0_MASK: u32 = (1 << 30) - 1;
const QSPI_MASK: u32 = (1 << 6) - 1;
const FIFO_DEPTH: usize = 8;

const ST_VLD: u32 = 1 << 0;
const ST_RDY: u32 = 1 << 1;
const ST_WOF: u32 = 1 << 2;
const ST_ROE: u32 = 1 << 3;

#[derive(Clone, Debug, Default)]
pub struct Divider {
    dividend: u32,
    divisor: u32,
    quotient: u32,
    remainder: u32,
    dirty: bool,
}

impl Divider {
    fn compute(&mut self, signed: bool) {
        let (n, d) = (self.dividend, self.divisor);
        if d == 0 {
            // Division by zero: quotient all ones for non-negative (or unsigned) dividends, 1 for negative ones
            self.quotient = if signed && (n as i32) < 0 { 1 } else { 0xFFFF_FFFF };
            self.remainder = n;
        } else if signed {
            self.quotient = (n as i32).wrapping_div(d as i32) as u32;
            self.remainder = (n as i32).wrapping_rem(d as i32) as u32;
        } else {
            self.quotient = n / d;
            self.remainder = n % d;
        }
        self.dirty = true;
    }
}

/// Lane control register fields.
#[derive(Clone, Copy, Debug)]
struct LaneCtrl(u32);

impl LaneCtrl {
    fn shift(self) -> u32 {
        self.0 & 31
    }
    fn mask_lsb(self) -> u32 {
        (self.0 >> 5) & 31
    }
    fn mask_msb(self) -> u32 {
        (self.0 >> 10) & 31
    }
    fn signed(self) -> bool {
        self.0 & (1 << 15) != 0
    }
    fn cross_input(self) -> bool {
        self.0 & (1 << 16) != 0
    }
    fn cross_result(self) -> bool {
        self.0 & (1 << 17) != 0
    }
    fn add_raw(self) -> bool {
        self.0 & (1 << 18) != 0
    }
    fn force_msb(self) -> u32 {
        (self.0 >> 19) & 3
    }
    fn blend(self) -> bool {
        self.0 & (1 << 21) != 0
    }
    fn clamp(self) -> bool {
        self.0 & (1 << 22) != 0
    }
}

/// One interpolator (datasheet 2.3.1.6). `index` selects the interpolator-specific features: blend mode on
/// interpolator 0, clamp mode on interpolator 1.
#[derive(Clone, Debug, Default)]
pub struct Interp {
    index: u8,
    pub accum: [u32; 2],
    pub base: [u32; 3],
    pub ctrl: [u32; 2],
    result: [u32; 3],
    overflow: (bool, bool),
}

impl Interp {
    fn new(index: u8) -> Self {
        let mut i = Interp { index, ..Default::default() };
        i.update();
        i
    }

    fn update(&mut self) {
        let c0 = LaneCtrl(self.ctrl[0]);
        let c1 = LaneCtrl(self.ctrl[1]);
        let do_clamp = c0.clamp() && self.index == 1;
        let do_blend = c0.blend() && self.index == 0;
        let input0 = if c0.cross_input() { self.accum[1] } else { self.accum[0] };
        let input1 = if c1.cross_input() { self.accum[0] } else { self.accum[1] };

        let lane = |c: LaneCtrl, input: u32| -> (u32, bool) {
            let msbmask = if c.mask_msb() == 31 { u32::MAX } else { (1u32 << (c.mask_msb() + 1)) - 1 };
            let mask = msbmask & !((1u32 << c.mask_lsb()) - 1);
            let shifted = input >> c.shift();
            let u = shifted & mask;
            let overflow = shifted & !msbmask != 0;
            let s = if u & (1 << c.mask_msb()) != 0 { u | (u32::MAX << c.mask_msb()) } else { u };
            (if c.signed() { s } else { u }, overflow)
        };
        let (r0, of0) = lane(c0, input0);
        let (r1, of1) = lane(c1, input1);
        self.overflow = (of0, of1);

        let add0 = self.base[0].wrapping_add(if c0.add_raw() { input0 } else { r0 });
        let add1 = self.base[1].wrapping_add(if c1.add_raw() { input1 } else { r1 });
        let add2 = self.base[2].wrapping_add(r0).wrapping_add(if do_blend { 0 } else { r1 });

        let clamp0 = if c0.signed() {
            (r0 as i32).clamp(self.base[0] as i32, (self.base[1] as i32).max(self.base[0] as i32)) as u32
        } else {
            r0.clamp(self.base[0], self.base[1].max(self.base[0]))
        };
        let alpha = (r1 & 0xFF) as i64;
        let blend1 = if c1.signed() {
            let (b0, b1) = (self.base[0] as i32 as i64, self.base[1] as i32 as i64);
            (b0 + (alpha * (b1 - b0)).div_euclid(256)) as u32
        } else {
            let (b0, b1) = (self.base[0] as i64, self.base[1] as i64);
            (b0 + (alpha * (b1 - b0)).div_euclid(256)) as u32
        };
        let msb = c0.force_msb() << 28;
        self.result[0] = if do_blend { alpha as u32 } else { (if do_clamp { clamp0 } else { add0 }) | msb };
        self.result[1] = (if do_blend { blend1 } else { add1 }) | msb;
        self.result[2] = add2;
    }

    fn writeback(&mut self) {
        let c0 = LaneCtrl(self.ctrl[0]);
        let c1 = LaneCtrl(self.ctrl[1]);
        let (r0, r1) = (self.result[0], self.result[1]);
        self.accum[0] = if c0.cross_result() { r1 } else { r0 };
        self.accum[1] = if c1.cross_result() { r0 } else { r1 };
        self.update();
    }

    fn read(&mut self, off: u32) -> u32 {
        match off {
            0x00 => self.accum[0],
            0x04 => self.accum[1],
            0x08 => self.base[0],
            0x0C => self.base[1],
            0x10 => self.base[2],
            0x14 | 0x18 | 0x1C => {
                let v = self.result[((off - 0x14) / 4) as usize];
                self.writeback();
                v
            }
            0x20 | 0x24 | 0x28 => self.result[((off - 0x20) / 4) as usize],
            0x2C => {
                let (o0, o1) = self.overflow;
                self.ctrl[0] | (o0 as u32) << 23 | (o1 as u32) << 24 | ((o0 || o1) as u32) << 25
            }
            0x30 => self.ctrl[1],
            0x34 => self.accum[0],
            0x38 => self.accum[1],
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, v: u32) {
        match off {
            0x00 => self.accum[0] = v,
            0x04 => self.accum[1] = v,
            0x08 => self.base[0] = v,
            0x0C => self.base[1] = v,
            0x10 => self.base[2] = v,
            0x2C => self.ctrl[0] = v & 0x007F_FFFF,
            0x30 => self.ctrl[1] = v & 0x001F_FFFF,
            0x34 => self.accum[0] = self.accum[0].wrapping_add(v),
            0x38 => self.accum[1] = self.accum[1].wrapping_add(v),
            0x3C => {
                // BASE_1AND0: two 16-bit values, sign-extended per the lanes' SIGNED flags
                let c0 = LaneCtrl(self.ctrl[0]);
                let c1 = LaneCtrl(self.ctrl[1]);
                let do_blend = c0.blend() && self.index == 0;
                let lo = v & 0xFFFF;
                let hi = v >> 16;
                let sx = |x: u32| x as u16 as i16 as i32 as u32;
                let base0_signed = if do_blend { c1.signed() } else { c0.signed() };
                self.base[0] = if base0_signed { sx(lo) } else { lo };
                self.base[1] = if c1.signed() { sx(hi) } else { hi };
            }
            _ => {}
        }
        self.update();
    }
}

#[derive(Clone, Debug)]
pub struct Sio {
    pub gpio_out: u32,
    pub gpio_oe: u32,
    pub gpio_hi_out: u32,
    pub gpio_hi_oe: u32,
    /// `fifo[n]` is written by core n and read by the other core.
    pub fifo: [VecDeque<u32>; 2],
    /// Sticky WOF/ROE flags per reading/writing core.
    fifo_err: [u32; 2],
    pub spinlocks: u32,
    pub div: [Divider; 2],
    pub interp: [[Interp; 2]; 2],
}

impl Default for Sio {
    fn default() -> Self {
        Sio {
            gpio_out: 0,
            gpio_oe: 0,
            gpio_hi_out: 0,
            gpio_hi_oe: 0,
            fifo: [VecDeque::new(), VecDeque::new()],
            fifo_err: [0; 2],
            spinlocks: 0,
            div: Default::default(),
            interp: [[Interp::new(0), Interp::new(1)], [Interp::new(0), Interp::new(1)]],
        }
    }
}

/// Applies a write through one of the four `REG`, `REG_SET`, `REG_CLR`, `REG_XOR` addresses (in that order).
fn update(reg: &mut u32, which: u32, value: u32, mask: u32) {
    let mode = match which {
        0 => WriteMode::Normal,
        1 => WriteMode::Set,
        2 => WriteMode::Clear,
        _ => WriteMode::Xor,
    };
    *reg = mode.apply(*reg, value) & mask;
}

impl Sio {
    fn fifo_st(&self, core: usize) -> u32 {
        let mut st = self.fifo_err[core];
        if !self.fifo[1 - core].is_empty() {
            st |= ST_VLD;
        }
        if self.fifo[core].len() < FIFO_DEPTH {
            st |= ST_RDY;
        }
        st
    }

    /// SIO_IRQ_PROCn: the core's RX FIFO has data or an error flag is set.
    pub fn fifo_irq(&self, core: usize) -> bool {
        self.fifo_st(core) & (ST_VLD | ST_WOF | ST_ROE) != 0
    }

    /// Resets the SIO (PSM force-off): FIFOs, spinlocks, dividers and interpolators. GPIO outputs too.
    pub fn reset(&mut self) {
        *self = Sio::default();
    }

    /// Reads a register. `gpio_in` / `gpio_hi_in` are the current input levels of the two banks.
    pub fn read(&mut self, core: u8, offset: u32, gpio_in: u32, gpio_hi_in: u32) -> Option<u32> {
        let c = core as usize & 1;
        Some(match offset {
            CPUID => core as u32,
            GPIO_IN => gpio_in & BANK0_MASK,
            GPIO_HI_IN => gpio_hi_in & QSPI_MASK,
            GPIO_OUT..=GPIO_OUT_XOR => self.gpio_out,
            GPIO_OE..=GPIO_OE_XOR => self.gpio_oe,
            GPIO_HI_OUT..=GPIO_HI_OUT_XOR => self.gpio_hi_out,
            GPIO_HI_OE..=GPIO_HI_OE_XOR => self.gpio_hi_oe,
            FIFO_ST => self.fifo_st(c),
            FIFO_WR => 0,
            FIFO_RD => match self.fifo[1 - c].pop_front() {
                Some(v) => v,
                None => {
                    self.fifo_err[c] |= ST_ROE;
                    0
                }
            },
            SPINLOCK_ST => self.spinlocks,
            DIV_UDIVIDEND | DIV_SDIVIDEND => self.div[c].dividend,
            DIV_UDIVISOR | DIV_SDIVISOR => self.div[c].divisor,
            DIV_QUOTIENT => {
                self.div[c].dirty = false;
                self.div[c].quotient
            }
            DIV_REMAINDER => self.div[c].remainder,
            DIV_CSR => 1 | (self.div[c].dirty as u32) << 1,
            INTERP0..=0x0FC => {
                let which = ((offset - INTERP0) / 0x40) as usize;
                self.interp[c][which].read((offset - INTERP0) % 0x40)
            }
            SPINLOCK0..=SPINLOCK31 => {
                let bit = 1 << ((offset - SPINLOCK0) / 4);
                if self.spinlocks & bit != 0 {
                    0
                } else {
                    self.spinlocks |= bit;
                    bit
                }
            }
            _ => return None,
        })
    }

    /// Reads a register without side effects (no FIFO pop, spinlock claim, interpolator pop or flag clearing).
    pub fn peek(&self, core: u8, offset: u32, gpio_in: u32) -> Option<u32> {
        let c = core as usize & 1;
        match offset {
            FIFO_RD => Some(self.fifo[1 - c].front().copied().unwrap_or(0)),
            SPINLOCK0..=SPINLOCK31 => Some(self.spinlocks & (1 << ((offset - SPINLOCK0) / 4))),
            DIV_QUOTIENT => Some(self.div[c].quotient),
            INTERP0..=0x0FC => {
                let which = ((offset - INTERP0) / 0x40) as usize;
                let off = (offset - INTERP0) % 0x40;
                let mut i = self.interp[c][which].clone();
                // POP registers read the same value as the PEEK ones
                Some(i.read(if (0x14..=0x1C).contains(&off) { off + 0xC } else { off }))
            }
            _ => self.clone().read(core, offset, gpio_in, 0),
        }
    }

    /// Writes a register. Returns false for offsets not handled here.
    pub fn write(&mut self, core: u8, offset: u32, value: u32) -> bool {
        let c = core as usize & 1;
        let which = (offset >> 2) & 3;
        match offset {
            GPIO_OUT..=GPIO_OUT_XOR => update(&mut self.gpio_out, which, value, BANK0_MASK),
            GPIO_OE..=GPIO_OE_XOR => update(&mut self.gpio_oe, which, value, BANK0_MASK),
            GPIO_HI_OUT..=GPIO_HI_OUT_XOR => update(&mut self.gpio_hi_out, which, value, QSPI_MASK),
            GPIO_HI_OE..=GPIO_HI_OE_XOR => update(&mut self.gpio_hi_oe, which, value, QSPI_MASK),
            FIFO_ST => self.fifo_err[c] &= !(value & (ST_WOF | ST_ROE)),
            FIFO_WR => {
                if self.fifo[c].len() < FIFO_DEPTH {
                    self.fifo[c].push_back(value);
                } else {
                    self.fifo_err[c] |= ST_WOF;
                }
            }
            DIV_UDIVIDEND | DIV_SDIVIDEND => {
                self.div[c].dividend = value;
                self.div[c].compute(offset == DIV_SDIVIDEND);
            }
            DIV_UDIVISOR | DIV_SDIVISOR => {
                self.div[c].divisor = value;
                self.div[c].compute(offset == DIV_SDIVISOR);
            }
            DIV_QUOTIENT => {
                self.div[c].quotient = value;
                self.div[c].dirty = true;
            }
            DIV_REMAINDER => {
                self.div[c].remainder = value;
                self.div[c].dirty = true;
            }
            INTERP0..=0x0FC => {
                let which = ((offset - INTERP0) / 0x40) as usize;
                self.interp[c][which].write((offset - INTERP0) % 0x40, value);
            }
            SPINLOCK0..=SPINLOCK31 => self.spinlocks &= !(1 << ((offset - SPINLOCK0) / 4)),
            // CPUID, the input registers, FIFO_RD, SPINLOCK_ST and DIV_CSR are read-only
            CPUID | GPIO_IN | GPIO_HI_IN | FIFO_RD | SPINLOCK_ST | DIV_CSR => {}
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpuid_per_core() {
        let mut s = Sio::default();
        assert_eq!(s.read(0, CPUID, 0, 0), Some(0));
        assert_eq!(s.read(1, CPUID, 0, 0), Some(1));
    }

    #[test]
    fn gpio_out_set_clr_xor() {
        let mut s = Sio::default();
        s.write(0, GPIO_OUT, 0xFFFF_FFFF);
        assert_eq!(s.gpio_out, BANK0_MASK, "only 30 GPIOs");
        s.write(0, GPIO_OUT_CLR, 0xF);
        assert_eq!(s.gpio_out, BANK0_MASK & !0xF);
        s.write(0, GPIO_OUT_SET, 0x1);
        assert_eq!(s.gpio_out & 0xF, 0x1);
        s.write(0, GPIO_OUT_XOR, 0x3);
        assert_eq!(s.gpio_out & 0xF, 0x2);
        assert_eq!(s.read(0, GPIO_OUT_XOR, 0, 0), Some(s.gpio_out), "aliases read the register");
        s.write(0, GPIO_OE_SET, 1 << 25);
        s.write(0, GPIO_OE_XOR, 1 << 24 | 1 << 25);
        assert_eq!(s.gpio_oe, 1 << 24);
        s.write(0, GPIO_HI_OUT, 0xFF);
        assert_eq!(s.gpio_hi_out, 0x3F);
    }

    #[test]
    fn inputs() {
        let mut s = Sio::default();
        assert_eq!(s.read(0, GPIO_IN, 0xFFFF_FFFF, 0), Some(BANK0_MASK));
        assert_eq!(s.read(0, GPIO_HI_IN, 0, 0xFF), Some(QSPI_MASK));
        assert!(s.write(0, GPIO_IN, 5), "read-only writes are accepted and ignored");
        assert!(!s.write(0, 0x200, 0));
    }

    #[test]
    fn fifos_between_cores() {
        let mut s = Sio::default();
        assert_eq!(s.read(1, FIFO_ST, 0, 0), Some(ST_RDY));
        s.write(0, FIFO_WR, 0xAA);
        s.write(0, FIFO_WR, 0xBB);
        assert_eq!(s.read(1, FIFO_ST, 0, 0), Some(ST_RDY | ST_VLD));
        assert!(s.fifo_irq(1));
        assert!(!s.fifo_irq(0));
        assert_eq!(s.read(1, FIFO_RD, 0, 0), Some(0xAA));
        assert_eq!(s.read(1, FIFO_RD, 0, 0), Some(0xBB));
        assert_eq!(s.read(1, FIFO_RD, 0, 0), Some(0));
        assert_eq!(s.read(1, FIFO_ST, 0, 0), Some(ST_RDY | ST_ROE), "read underflow is sticky");
        s.write(1, FIFO_ST, ST_ROE);
        assert_eq!(s.read(1, FIFO_ST, 0, 0), Some(ST_RDY));
        for i in 0..9 {
            s.write(1, FIFO_WR, i);
        }
        assert_eq!(s.read(1, FIFO_ST, 0, 0), Some(ST_WOF), "full and overflowed");
        assert_eq!(s.read(0, FIFO_RD, 0, 0), Some(0));
        assert_eq!(s.fifo[1].len(), 7);
    }

    #[test]
    fn spinlocks() {
        let mut s = Sio::default();
        assert_eq!(s.read(0, SPINLOCK31, 0, 0), Some(1 << 31));
        assert_eq!(s.read(1, SPINLOCK31, 0, 0), Some(0), "already claimed");
        assert_eq!(s.read(1, SPINLOCK_ST, 0, 0), Some(1 << 31));
        s.write(0, SPINLOCK31, 0);
        assert_eq!(s.read(1, SPINLOCK31, 0, 0), Some(1 << 31));
        assert_eq!(s.read(0, SPINLOCK0, 0, 0), Some(1));
        s.reset();
        assert_eq!(s.spinlocks, 0);
    }

    #[test]
    fn divider() {
        let mut s = Sio::default();
        s.write(0, DIV_UDIVIDEND, 100);
        s.write(0, DIV_UDIVISOR, 7);
        assert_eq!(s.read(0, DIV_CSR, 0, 0), Some(3), "ready and dirty");
        assert_eq!(s.read(0, DIV_REMAINDER, 0, 0), Some(2));
        assert_eq!(s.read(0, DIV_QUOTIENT, 0, 0), Some(14));
        assert_eq!(s.read(0, DIV_CSR, 0, 0), Some(1), "quotient read clears dirty");
        s.write(0, DIV_SDIVIDEND, (-100i32) as u32);
        s.write(0, DIV_SDIVISOR, 7);
        assert_eq!(s.read(0, DIV_QUOTIENT, 0, 0), Some((-14i32) as u32));
        assert_eq!(s.read(0, DIV_REMAINDER, 0, 0), Some((-2i32) as u32));
        // Divide by zero
        s.write(0, DIV_UDIVIDEND, 5);
        s.write(0, DIV_UDIVISOR, 0);
        assert_eq!((s.read(0, DIV_QUOTIENT, 0, 0), s.read(0, DIV_REMAINDER, 0, 0)), (Some(u32::MAX), Some(5)));
        s.write(0, DIV_SDIVIDEND, (-5i32) as u32);
        s.write(0, DIV_SDIVISOR, 0);
        assert_eq!(s.read(0, DIV_QUOTIENT, 0, 0), Some(1));
        // Overflow case
        s.write(0, DIV_SDIVIDEND, i32::MIN as u32);
        s.write(0, DIV_SDIVISOR, (-1i32) as u32);
        assert_eq!(s.read(0, DIV_QUOTIENT, 0, 0), Some(i32::MIN as u32));
        // Per-core dividers, and save/restore through the result registers
        s.write(1, DIV_QUOTIENT, 77);
        assert_eq!(s.read(1, DIV_QUOTIENT, 0, 0), Some(77));
        assert_eq!(s.read(0, DIV_DIVIDEND_CHECK, 0, 0), Some(i32::MIN as u32));
    }

    const DIV_DIVIDEND_CHECK: u32 = DIV_UDIVIDEND;

    #[test]
    fn interp_lane_shift_mask_and_pop() {
        let mut s = Sio::default();
        // Lane 0: shift 4, mask bits 0..7; lane 1: raw
        s.write(0, INTERP0 + 0x2C, 4 | 7 << 10);
        s.write(0, INTERP0 + 0x30, 31 << 10 | 1 << 18);
        s.write(0, INTERP0, 0x1234);
        s.write(0, INTERP0 + 0x04, 10);
        s.write(0, INTERP0 + 0x08, 0x1000);
        s.write(0, INTERP0 + 0x0C, 5);
        s.write(0, INTERP0 + 0x10, 0);
        assert_eq!(s.read(0, INTERP0 + 0x20, 0, 0), Some(0x1000 + 0x23), "PEEK_LANE0");
        assert_eq!(s.read(0, INTERP0 + 0x24, 0, 0), Some(15), "PEEK_LANE1 add raw");
        assert_eq!(s.read(0, INTERP0 + 0x28, 0, 0), Some(0x23 + 10), "PEEK_FULL");
        assert!(s.read(0, INTERP0 + 0x2C, 0, 0).unwrap() & (1 << 23) != 0, "lane 0 discarded bits overflow");
        // POP writes results back into the accumulators
        assert_eq!(s.read(0, INTERP0 + 0x14, 0, 0), Some(0x1023));
        assert_eq!(s.read(0, INTERP0, 0, 0), Some(0x1023));
        assert_eq!(s.read(0, INTERP0 + 0x04, 0, 0), Some(15));
        // ACCUM0_ADD
        s.write(0, INTERP0 + 0x34, 1);
        assert_eq!(s.read(0, INTERP0, 0, 0), Some(0x1024));
        // Core 1 has its own interpolators
        assert_eq!(s.read(1, INTERP0, 0, 0), Some(0));
    }

    #[test]
    fn interp_signed_and_clamp() {
        let mut s = Sio::default();
        // Interp1 lane 0: signed, mask 0..7, clamp to [-10, 10]
        s.write(0, INTERP1 + 0x2C, 7 << 10 | 1 << 15 | 1 << 22);
        s.write(0, INTERP1 + 0x08, (-10i32) as u32);
        s.write(0, INTERP1 + 0x0C, 10);
        s.write(0, INTERP1, 0xF0); // -16 as a signed byte
        assert_eq!(s.read(0, INTERP1 + 0x20, 0, 0), Some((-10i32) as u32));
        s.write(0, INTERP1, 0x05);
        assert_eq!(s.read(0, INTERP1 + 0x20, 0, 0), Some(5));
    }

    #[test]
    fn interp_blend() {
        let mut s = Sio::default();
        s.write(0, INTERP0 + 0x2C, 1 << 21); // blend on interp0
        s.write(0, INTERP0 + 0x30, 7 << 10); // lane 1: alpha in bits 0..7
        s.write(0, INTERP0 + 0x3C, 200 << 16 | 100); // BASE_1AND0
        s.write(0, INTERP0 + 0x04, 128);
        assert_eq!(s.read(0, INTERP0 + 0x24, 0, 0), Some(150), "halfway between 100 and 200");
    }
}
