//! The core-private peripherals on the Private Peripheral Bus (`0xE000_0000`): SysTick, NVIC, SCB and the MPU.

use sim_core::bus::Size;

use crate::CortexM0Plus;

pub const PPB_BASE: u32 = 0xE000_0000;

pub const SYST_CSR: u32 = 0xE000_E010;
pub const SYST_RVR: u32 = 0xE000_E014;
pub const SYST_CVR: u32 = 0xE000_E018;
pub const SYST_CALIB: u32 = 0xE000_E01C;
pub const NVIC_ISER: u32 = 0xE000_E100;
pub const NVIC_ICER: u32 = 0xE000_E180;
pub const NVIC_ISPR: u32 = 0xE000_E200;
pub const NVIC_ICPR: u32 = 0xE000_E280;
pub const NVIC_IPR0: u32 = 0xE000_E400;
pub const NVIC_IPR7: u32 = 0xE000_E41C;
pub const CPUID: u32 = 0xE000_ED00;
pub const ICSR: u32 = 0xE000_ED04;
pub const VTOR: u32 = 0xE000_ED08;
pub const AIRCR: u32 = 0xE000_ED0C;
pub const SCR: u32 = 0xE000_ED10;
pub const CCR: u32 = 0xE000_ED14;
pub const SHPR2: u32 = 0xE000_ED1C;
pub const SHPR3: u32 = 0xE000_ED20;
pub const SHCSR: u32 = 0xE000_ED24;
pub const MPU_TYPE: u32 = 0xE000_ED90;
pub const MPU_CTRL: u32 = 0xE000_ED94;
pub const MPU_RNR: u32 = 0xE000_ED98;
pub const MPU_RBAR: u32 = 0xE000_ED9C;
pub const MPU_RASR: u32 = 0xE000_EDA0;

pub const SCR_SLEEPONEXIT: u32 = 1 << 1;
pub const SCR_SLEEPDEEP: u32 = 1 << 2;
pub const SCR_SEVONPEND: u32 = 1 << 4;

const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2;
const CSR_COUNTFLAG: u32 = 1 << 16;

/// Nested vectored interrupt controller: 32 external interrupts, 2 priority bits each.
#[derive(Clone, Debug, Default)]
pub struct Nvic {
    pub enabled: u32,
    pub pending: u32,
    /// Priority byte of each IRQ (only bits 7:6 are implemented).
    pub priority: [u8; 32],
}

/// System control block state.
#[derive(Clone, Debug, Default)]
pub struct Scb {
    pub vtor: u32,
    pub scr: u32,
    pub shpr2: u32,
    pub shpr3: u32,
    pub nmi_pending: bool,
    pub pendsv_pending: bool,
    pub systick_pending: bool,
    /// Set by a write of `SYSRESETREQ` to AIRCR; collected by the machine.
    pub sysresetreq: bool,
}

/// The 24-bit SysTick down counter.
#[derive(Clone, Debug, Default)]
pub struct SysTick {
    pub csr: u32,
    pub rvr: u32,
    pub cvr: u32,
}

impl SysTick {
    fn enabled(&self) -> bool {
        self.csr & CSR_ENABLE != 0
    }

    /// Counts `n` clocks. Returns true if the exception should be pended.
    fn count(&mut self, mut n: u64) -> bool {
        if !self.enabled() || n == 0 {
            return false;
        }
        let mut wrapped = false;
        if self.cvr != 0 {
            if n < self.cvr as u64 {
                self.cvr -= n as u32;
                return false;
            }
            n -= self.cvr as u64;
            self.cvr = 0;
            wrapped = true;
        }
        // Counter at zero with `n` clocks left: each period of RVR+1 clocks reloads then counts down to zero again
        if n > 0 && self.rvr != 0 {
            let period = self.rvr as u64 + 1;
            if n >= period {
                wrapped = true;
            }
            let r = n % period;
            self.cvr = if r == 0 { 0 } else { self.rvr - (r as u32 - 1) };
        }
        if wrapped {
            self.csr |= CSR_COUNTFLAG;
        }
        wrapped && self.csr & CSR_TICKINT != 0
    }

    /// Core clock cycles (only counted when CLKSOURCE selects the processor clock).
    pub fn tick_core(&mut self, cycles: u64) -> bool {
        self.csr & CSR_CLKSOURCE != 0 && self.count(cycles)
    }

    /// Reference clock ticks (only counted when CLKSOURCE selects the external reference).
    pub fn tick_reference(&mut self, ticks: u64) -> bool {
        self.csr & CSR_CLKSOURCE == 0 && self.count(ticks)
    }

    /// Clocks until the next exception, if SysTick counts the core clock and will interrupt.
    pub fn cycles_until_wrap(&self) -> Option<u64> {
        if !self.enabled() || self.csr & (CSR_TICKINT | CSR_CLKSOURCE) != CSR_TICKINT | CSR_CLKSOURCE {
            return None;
        }
        if self.cvr != 0 {
            Some(self.cvr as u64)
        } else if self.rvr != 0 {
            Some(self.rvr as u64 + 1)
        } else {
            None
        }
    }
}

/// MPU registers. Stored so firmware reads back what it wrote; protection is not enforced.
#[derive(Clone, Debug, Default)]
pub struct Mpu {
    pub ctrl: u32,
    pub rnr: u32,
    pub rbar: [u32; 8],
    pub rasr: [u32; 8],
}

impl CortexM0Plus {
    fn ppb_read_word(&mut self, addr: u32) -> u32 {
        match addr {
            SYST_CSR => {
                let v = self.systick.csr;
                self.systick.csr &= !CSR_COUNTFLAG;
                v
            }
            SYST_RVR => self.systick.rvr,
            SYST_CVR => self.systick.cvr,
            SYST_CALIB => 0,
            NVIC_ISER | NVIC_ICER => self.nvic.enabled,
            NVIC_ISPR | NVIC_ICPR => self.nvic.pending,
            NVIC_IPR0..=NVIC_IPR7 => {
                let i = ((addr - NVIC_IPR0) / 4) as usize * 4;
                u32::from_le_bytes(self.nvic.priority[i..i + 4].try_into().unwrap())
            }
            CPUID => self.cpuid,
            ICSR => self.icsr(),
            VTOR => self.scb.vtor,
            AIRCR => 0xFA05_0000,
            SCR => self.scb.scr,
            CCR => 0x0000_0208,
            SHPR2 => self.scb.shpr2,
            SHPR3 => self.scb.shpr3,
            SHCSR => 0, // debug access only on ARMv6-M
            MPU_TYPE => 0x0000_0800,
            MPU_CTRL => self.mpu.ctrl,
            MPU_RNR => self.mpu.rnr,
            MPU_RBAR => self.mpu.rbar[(self.mpu.rnr & 7) as usize] | (self.mpu.rnr & 0xF),
            MPU_RASR => self.mpu.rasr[(self.mpu.rnr & 7) as usize],
            _ => {
                log::trace!("unmodelled PPB read {addr:#010x}");
                0
            }
        }
    }

    fn icsr(&self) -> u32 {
        let mut v = self.ipsr & 0x1FF;
        if let Some((e, _)) = self.highest_pending() {
            v |= (e & 0x1FF) << 12;
        }
        if self.nvic.pending & self.nvic.enabled != 0 {
            v |= 1 << 22;
        }
        if self.scb.pendsv_pending {
            v |= 1 << 28;
        }
        if self.scb.systick_pending {
            v |= 1 << 26;
        }
        if self.scb.nmi_pending {
            v |= 1 << 31;
        }
        v
    }

    /// Writes `value` to the bits in `mask` of the word register at `addr`.
    fn ppb_write_word(&mut self, addr: u32, value: u32, mask: u32) {
        let merge = |old: u32| (old & !mask) | (value & mask);
        let set_bits = value & mask;
        match addr {
            SYST_CSR => self.systick.csr = merge(self.systick.csr) & (CSR_ENABLE | CSR_TICKINT | CSR_CLKSOURCE),
            SYST_RVR => self.systick.rvr = merge(self.systick.rvr) & 0x00FF_FFFF,
            SYST_CVR => {
                self.systick.cvr = 0;
                self.systick.csr &= !CSR_COUNTFLAG;
            }
            NVIC_ISER => self.nvic.enabled |= set_bits,
            NVIC_ICER => self.nvic.enabled &= !set_bits,
            NVIC_ISPR => self.nvic.pending |= set_bits,
            NVIC_ICPR => self.nvic.pending &= !set_bits,
            NVIC_IPR0..=NVIC_IPR7 => {
                let i = ((addr - NVIC_IPR0) / 4) as usize * 4;
                let old = u32::from_le_bytes(self.nvic.priority[i..i + 4].try_into().unwrap());
                let new = merge(old) & 0xC0C0_C0C0;
                self.nvic.priority[i..i + 4].copy_from_slice(&new.to_le_bytes());
            }
            ICSR => {
                if set_bits & (1 << 31) != 0 {
                    self.scb.nmi_pending = true;
                }
                if set_bits & (1 << 28) != 0 {
                    self.scb.pendsv_pending = true;
                }
                if set_bits & (1 << 27) != 0 {
                    self.scb.pendsv_pending = false;
                }
                if set_bits & (1 << 26) != 0 {
                    self.scb.systick_pending = true;
                }
                if set_bits & (1 << 25) != 0 {
                    self.scb.systick_pending = false;
                }
            }
            VTOR => self.scb.vtor = merge(self.scb.vtor) & 0xFFFF_FF00,
            AIRCR => {
                if mask == 0xFFFF_FFFF && value >> 16 == 0x05FA {
                    if value & (1 << 2) != 0 {
                        self.scb.sysresetreq = true;
                    }
                    if value & (1 << 1) != 0 {
                        // VECTCLRACTIVE (debug use): clear all active state
                        self.active = 0;
                    }
                }
            }
            SCR => self.scb.scr = merge(self.scb.scr) & (SCR_SLEEPONEXIT | SCR_SLEEPDEEP | SCR_SEVONPEND),
            SHPR2 => self.scb.shpr2 = merge(self.scb.shpr2) & 0xC000_0000,
            SHPR3 => self.scb.shpr3 = merge(self.scb.shpr3) & 0xC0C0_0000,
            MPU_CTRL => self.mpu.ctrl = merge(self.mpu.ctrl) & 7,
            MPU_RNR => self.mpu.rnr = merge(self.mpu.rnr) & 7,
            MPU_RBAR => {
                let v = merge(self.mpu.rbar[(self.mpu.rnr & 7) as usize]);
                if v & (1 << 4) != 0 {
                    self.mpu.rnr = v & 7;
                }
                self.mpu.rbar[(self.mpu.rnr & 7) as usize] = v & 0xFFFF_FF00;
            }
            MPU_RASR => {
                let i = (self.mpu.rnr & 7) as usize;
                self.mpu.rasr[i] = merge(self.mpu.rasr[i]);
            }
            _ => log::trace!("unmodelled PPB write {addr:#010x} = {value:#010x}"),
        }
    }

    pub(crate) fn ppb_read(&mut self, addr: u32, size: Size) -> u32 {
        let word = self.ppb_read_word(addr & !3);
        sim_core::bus::mem::lane(word, addr, size)
    }

    pub(crate) fn ppb_write(&mut self, addr: u32, size: Size, value: u32) {
        let shift = (addr & 3) * 8;
        self.ppb_write_word(addr & !3, (value & size.mask()) << shift, size.mask() << shift);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn st(rvr: u32, cvr: u32) -> SysTick {
        SysTick { csr: CSR_ENABLE | CSR_TICKINT | CSR_CLKSOURCE, rvr, cvr }
    }

    #[test]
    fn systick_counts_down_and_reloads() {
        let mut s = st(9, 5);
        assert!(!s.tick_core(4));
        assert_eq!(s.cvr, 1);
        assert!(s.tick_core(1));
        assert_eq!(s.cvr, 0);
        assert!(s.csr & CSR_COUNTFLAG != 0);
        // Next clock reloads
        assert!(!s.tick_core(1));
        assert_eq!(s.cvr, 9);
        // A full period (10 clocks) later it is at 9 again, having wrapped
        assert!(s.tick_core(10));
        assert_eq!(s.cvr, 9);
    }

    #[test]
    fn systick_long_skip() {
        let mut s = st(999, 0);
        assert!(s.tick_core(1_000_000));
        assert_eq!(s.cvr, 0);
        let mut s = st(999, 0);
        assert!(s.tick_core(1_000_001));
        assert_eq!(s.cvr, 999);
    }

    #[test]
    fn systick_clock_source_and_disable() {
        let mut s = st(9, 5);
        assert!(!s.tick_reference(100), "reference ticks ignored when counting the core clock");
        s.csr &= !CSR_CLKSOURCE;
        assert!(s.tick_reference(5));
        s.csr = 0;
        assert!(!s.tick_core(100));
    }

    #[test]
    fn systick_until_wrap() {
        assert_eq!(st(9, 5).cycles_until_wrap(), Some(5));
        assert_eq!(st(9, 0).cycles_until_wrap(), Some(10));
        let mut s = st(9, 5);
        s.csr &= !CSR_TICKINT;
        assert_eq!(s.cycles_until_wrap(), None);
    }

    #[test]
    fn nvic_registers() {
        let mut c = CortexM0Plus::new();
        c.ppb_write(NVIC_ISER, Size::Word, 0b1010);
        assert_eq!(c.ppb_read(NVIC_ISER, Size::Word), 0b1010);
        c.ppb_write(NVIC_ICER, Size::Word, 0b0010);
        assert_eq!(c.ppb_read(NVIC_ICER, Size::Word), 0b1000);
        c.ppb_write(NVIC_ISPR, Size::Word, 1 << 5);
        assert_eq!(c.ppb_read(NVIC_ISPR, Size::Word), 1 << 5);
        c.ppb_write(NVIC_ICPR, Size::Word, 1 << 5);
        assert_eq!(c.nvic.pending, 0);
        c.ppb_write(NVIC_IPR0 + 4, Size::Word, 0xFFFF_FF40);
        assert_eq!(c.nvic.priority[4..8], [0x40, 0xC0, 0xC0, 0xC0]);
        // Byte write to one priority field
        c.ppb_write(NVIC_IPR0 + 5, Size::Byte, 0x80);
        assert_eq!(c.nvic.priority[5], 0x80);
        assert_eq!(c.ppb_read(NVIC_IPR0 + 4, Size::Word), 0xC0C0_8040);
    }

    #[test]
    fn scb_registers() {
        let mut c = CortexM0Plus::new();
        assert_eq!(c.ppb_read(CPUID, Size::Word), 0x410C_C601);
        c.ppb_write(VTOR, Size::Word, 0x1000_0100);
        assert_eq!(c.scb.vtor, 0x1000_0100);
        c.ppb_write(VTOR, Size::Word, 0x2000_0012);
        assert_eq!(c.scb.vtor, 0x2000_0000, "low bits of VTOR are fixed");
        // AIRCR needs the key
        c.ppb_write(AIRCR, Size::Word, 0x0000_0004);
        assert!(!c.scb.sysresetreq);
        c.ppb_write(AIRCR, Size::Word, 0x05FA_0004);
        assert!(c.take_reset_request());
        c.ppb_write(ICSR, Size::Word, 1 << 28);
        assert!(c.scb.pendsv_pending);
        assert!(c.ppb_read(ICSR, Size::Word) & (1 << 28) != 0);
        assert_eq!((c.ppb_read(ICSR, Size::Word) >> 12) & 0x1FF, 14, "VECTPENDING");
        c.ppb_write(ICSR, Size::Word, 1 << 27);
        assert!(!c.scb.pendsv_pending);
        c.ppb_write(SHPR3, Size::Word, 0xFFFF_FFFF);
        assert_eq!(c.scb.shpr3, 0xC0C0_0000);
        assert_eq!(c.exception_priority(crate::exception::PENDSV), 0xC0);
        c.ppb_write(SCR, Size::Word, 0xFF);
        assert_eq!(c.scb.scr, 0x16);
        assert_eq!(c.ppb_read(MPU_TYPE, Size::Word), 0x800);
    }

    #[test]
    fn systick_registers() {
        let mut c = CortexM0Plus::new();
        c.ppb_write(SYST_RVR, Size::Word, 0xFFFF_FFFF);
        assert_eq!(c.systick.rvr, 0x00FF_FFFF);
        c.systick.cvr = 1234;
        c.systick.csr |= CSR_COUNTFLAG;
        c.ppb_write(SYST_CVR, Size::Word, 99);
        assert_eq!(c.systick.cvr, 0, "any write clears CVR");
        assert_eq!(c.systick.csr & CSR_COUNTFLAG, 0);
        c.systick.csr = CSR_ENABLE | CSR_COUNTFLAG;
        assert!(c.ppb_read(SYST_CSR, Size::Word) & CSR_COUNTFLAG != 0);
        assert!(c.ppb_read(SYST_CSR, Size::Word) & CSR_COUNTFLAG == 0, "COUNTFLAG clears on read");
    }
}
