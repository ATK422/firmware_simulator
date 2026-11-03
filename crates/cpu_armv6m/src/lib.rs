//! Cortex-M0+ (ARMv6-M) interpreter.
//!
//! * The whole ARMv6-M Thumb instruction set: every 16-bit encoding plus the 32-bit `BL`, `MRS`, `MSR`, `DMB`,
//!   `DSB`, `ISB` and `UDF.W` ([`exec`]).
//! * Exception entry, return and tail-chaining with the ARMv6-M stack frame and 8-byte alignment, PRIMASK, the
//!   banked MSP/PSP and the CONTROL register.
//! * The core-private peripherals that live on each core's PPB ([`ppb`]): NVIC (32 level sensitive IRQ lines, 2
//!   priority bits), SysTick, SCB (VTOR, AIRCR with `SYSRESETREQ`, SCR with `SLEEPONEXIT`/`SEVONPEND`, SHPR2/3,
//!   ICSR) and the MPU registers (stored, not enforced).
//! * `WFI`/`WFE`/`SEV` with the event register; the machine decides what a sleeping core costs.
//! * Cycle counts from the Cortex-M0+ TRM (single cycle multiplier, as on RP2040).
//!
//! The core knows nothing about the SoC: it executes against a [`sim_core::Bus`] and gets its interrupt lines from
//! [`CortexM0Plus::set_irq_lines`].

pub mod disasm;
mod exec;
pub mod ppb;

#[cfg(test)]
mod tests;

use sim_core::bus::{Bus, BusFault, Size};
use sim_core::cpu::{Cpu, StepOutcome};

pub use ppb::{Mpu, Nvic, Scb, SysTick};

/// Exception numbers.
pub mod exception {
    pub const RESET: u32 = 1;
    pub const NMI: u32 = 2;
    pub const HARD_FAULT: u32 = 3;
    pub const SVCALL: u32 = 11;
    pub const PENDSV: u32 = 14;
    pub const SYSTICK: u32 = 15;
    /// Exception number of external interrupt 0.
    pub const IRQ0: u32 = 16;
}

/// Debugger register numbers, matching the target description served by the GDB stub.
pub mod reg {
    pub const SP: usize = 13;
    pub const LR: usize = 14;
    pub const PC: usize = 15;
    pub const XPSR: usize = 16;
    pub const MSP: usize = 17;
    pub const PSP: usize = 18;
    pub const PRIMASK: usize = 19;
    pub const CONTROL: usize = 20;
    pub const COUNT: usize = 21;
}

/// Why an instruction could not complete. Turned into an exception by [`CortexM0Plus::step`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Fault {
    Bus(BusFault),
    /// Undefined or unpredictable encoding.
    Undefined(u32),
    /// Unaligned data access (always faults on ARMv6-M).
    Unaligned(u32),
    /// `BKPT #imm`.
    Breakpoint(u8),
    /// Branch to an address with bit 0 clear (would leave Thumb state).
    InvalidState(u32),
    /// Illegal EXC_RETURN value.
    InvalidReturn(u32),
}

impl From<BusFault> for Fault {
    fn from(f: BusFault) -> Fault {
        Fault::Bus(f)
    }
}

/// Why a core is asleep.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sleep {
    Wfi,
    Wfe,
}

/// Information about the last fault that led to a HardFault, for logging.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FaultRecord {
    pub pc: u32,
    pub fault: Fault,
}

/// One Cortex-M0+ core.
#[derive(Clone, Debug)]
pub struct CortexM0Plus {
    /// r0-r15. `r[13]` is the active stack pointer; `r[15]` is the address of the instruction being executed.
    pub(crate) r: [u32; 16],
    /// The stack pointer not currently selected (PSP while on MSP and vice versa).
    pub(crate) other_sp: u32,
    pub(crate) n: bool,
    pub(crate) z: bool,
    pub(crate) c: bool,
    pub(crate) v: bool,
    /// Current exception number, 0 in thread mode.
    pub(crate) ipsr: u32,
    pub(crate) primask: bool,
    pub(crate) spsel: bool,
    pub(crate) npriv: bool,
    /// Bitmask of active exceptions, indexed by exception number.
    pub(crate) active: u64,
    pub nvic: Nvic,
    pub scb: Scb,
    pub systick: SysTick,
    pub mpu: Mpu,
    pub(crate) irq_lines: u32,
    pub(crate) sleep: Option<Sleep>,
    pub(crate) event_reg: bool,
    pub(crate) cycles: u64,
    pub(crate) lockup: bool,
    /// Set when the last instruction was `SEV`; the machine forwards the event to the other cores.
    pub(crate) sev_out: bool,
    /// CPUID value (`0x410CC601` for r0p1).
    pub cpuid: u32,
    /// Address and encoding of the last executed instruction, for tracing.
    pub last_pc: u32,
    pub last_opcode: u32,
    pub last_fault: Option<FaultRecord>,
    /// Exception entries (number) and returns since the last [`CortexM0Plus::take_exception_log`], for tracing.
    pub(crate) exception_log: Vec<(bool, u32, u32)>,
    pub log_exceptions: bool,
}

impl Default for CortexM0Plus {
    fn default() -> Self {
        Self::new()
    }
}

impl CortexM0Plus {
    pub fn new() -> Self {
        CortexM0Plus {
            r: [0; 16],
            other_sp: 0,
            n: false,
            z: false,
            c: false,
            v: false,
            ipsr: 0,
            primask: false,
            spsel: false,
            npriv: false,
            active: 0,
            nvic: Nvic::default(),
            scb: Scb::default(),
            systick: SysTick::default(),
            mpu: Mpu::default(),
            irq_lines: 0,
            sleep: None,
            event_reg: false,
            cycles: 0,
            lockup: false,
            sev_out: false,
            cpuid: 0x410C_C601,
            last_pc: 0,
            last_opcode: 0,
            last_fault: None,
            exception_log: Vec::new(),
            log_exceptions: false,
        }
    }

    // ------------------------------------------------------------------ state access

    pub fn reg(&self, n: usize) -> u32 {
        self.r[n]
    }

    pub fn set_reg(&mut self, n: usize, v: u32) {
        self.r[n] = v;
    }

    pub fn set_pc(&mut self, pc: u32) {
        self.r[15] = pc & !1;
    }

    pub fn xpsr(&self) -> u32 {
        (self.n as u32) << 31
            | (self.z as u32) << 30
            | (self.c as u32) << 29
            | (self.v as u32) << 28
            | 1 << 24
            | self.ipsr
    }

    pub(crate) fn set_apsr(&mut self, v: u32) {
        self.n = v & (1 << 31) != 0;
        self.z = v & (1 << 30) != 0;
        self.c = v & (1 << 29) != 0;
        self.v = v & (1 << 28) != 0;
    }

    pub fn ipsr(&self) -> u32 {
        self.ipsr
    }

    pub fn primask(&self) -> bool {
        self.primask
    }

    pub fn control(&self) -> u32 {
        (self.spsel as u32) << 1 | self.npriv as u32
    }

    pub(crate) fn using_psp(&self) -> bool {
        self.ipsr == 0 && self.spsel
    }

    pub fn msp(&self) -> u32 {
        if self.using_psp() { self.other_sp } else { self.r[13] }
    }

    pub fn psp(&self) -> u32 {
        if self.using_psp() { self.r[13] } else { self.other_sp }
    }

    pub fn set_msp(&mut self, v: u32) {
        if self.using_psp() {
            self.other_sp = v & !3;
        } else {
            self.r[13] = v & !3;
        }
    }

    pub fn set_psp(&mut self, v: u32) {
        if self.using_psp() {
            self.r[13] = v & !3;
        } else {
            self.other_sp = v & !3;
        }
    }

    /// Changes mode/SPSEL, swapping the banked stack pointer if the selection changes.
    pub(crate) fn set_mode(&mut self, ipsr: u32, spsel: bool) {
        let before = self.using_psp();
        self.ipsr = ipsr;
        self.spsel = spsel;
        if before != self.using_psp() {
            std::mem::swap(&mut self.r[13], &mut self.other_sp);
        }
    }

    pub fn is_locked_up(&self) -> bool {
        self.lockup
    }

    pub fn sleep_state(&self) -> Option<Sleep> {
        self.sleep
    }

    /// The level of each external interrupt line (bit n = IRQ n).
    pub fn set_irq_lines(&mut self, lines: u32) {
        self.irq_lines = lines;
    }

    /// Sets the event register (an `SEV` from another core, or an external event), waking a `WFE`.
    pub fn signal_event(&mut self) {
        self.event_reg = true;
    }

    /// True (once) if the last step executed `SEV`.
    pub fn take_sev(&mut self) -> bool {
        std::mem::take(&mut self.sev_out)
    }

    /// True (once) if software requested a system reset through AIRCR.SYSRESETREQ.
    pub fn take_reset_request(&mut self) -> bool {
        std::mem::take(&mut self.scb.sysresetreq)
    }

    /// Exception entries (`true`) and returns (`false`) since the last call, as `(entry, exception, pc)`.
    pub fn take_exception_log(&mut self) -> Vec<(bool, u32, u32)> {
        std::mem::take(&mut self.exception_log)
    }

    // ------------------------------------------------------------------ exceptions

    /// Priority of an exception; lower numbers are more urgent. Only the top two bits are implemented.
    pub fn exception_priority(&self, exc: u32) -> i32 {
        match exc {
            exception::RESET => -3,
            exception::NMI => -2,
            exception::HARD_FAULT => -1,
            exception::SVCALL => ((self.scb.shpr2 >> 24) & 0xC0) as i32,
            exception::PENDSV => ((self.scb.shpr3 >> 16) & 0xC0) as i32,
            exception::SYSTICK => ((self.scb.shpr3 >> 24) & 0xC0) as i32,
            e if e >= exception::IRQ0 => (self.nvic.priority[(e - 16) as usize] & 0xC0) as i32,
            _ => 256,
        }
    }

    /// Priority the core currently executes at, before PRIMASK boosting.
    fn active_priority(&self) -> i32 {
        let mut prio = 256;
        let mut act = self.active;
        while act != 0 {
            let e = act.trailing_zeros();
            act &= act - 1;
            prio = prio.min(self.exception_priority(e));
        }
        prio
    }

    pub fn execution_priority(&self) -> i32 {
        let p = self.active_priority();
        if self.primask { p.min(0) } else { p }
    }

    /// Latches asserted IRQ lines into the NVIC pending bits. Level sensitive: an active interrupt is not
    /// re-pended until its handler returns.
    fn latch_irqs(&mut self) {
        let active_irqs = (self.active >> 16) as u32;
        let newly = self.irq_lines & !active_irqs & !self.nvic.pending;
        if newly != 0 {
            self.nvic.pending |= newly;
            if self.scb.scr & ppb::SCR_SEVONPEND != 0 {
                self.event_reg = true;
            }
        }
    }

    /// Most urgent pending exception that is allowed to run (enabled), with its priority.
    pub fn highest_pending(&self) -> Option<(u32, i32)> {
        let mut best: Option<(u32, i32)> = None;
        let mut consider = |e: u32, p: i32| {
            if best.is_none_or(|(_, bp)| p < bp) {
                best = Some((e, p));
            }
        };
        if self.scb.nmi_pending {
            consider(exception::NMI, -2);
        }
        if self.scb.pendsv_pending {
            consider(exception::PENDSV, self.exception_priority(exception::PENDSV));
        }
        if self.scb.systick_pending {
            consider(exception::SYSTICK, self.exception_priority(exception::SYSTICK));
        }
        let mut irqs = self.nvic.pending & self.nvic.enabled;
        while irqs != 0 {
            let n = irqs.trailing_zeros();
            irqs &= irqs - 1;
            consider(16 + n, self.exception_priority(16 + n));
        }
        // Ties go to the lowest exception number: candidates are considered in increasing number and only a
        // strictly more urgent one replaces the current best.
        best
    }

    /// The exception that would be taken now, ignoring PRIMASK if `ignore_primask` (used for WFI/WFE wake-up).
    fn preempting(&self, ignore_primask: bool) -> Option<u32> {
        let (e, p) = self.highest_pending()?;
        let limit = if ignore_primask { self.active_priority() } else { self.execution_priority() };
        if p < limit { Some(e) } else { None }
    }

    fn clear_pending(&mut self, e: u32) {
        match e {
            exception::NMI => self.scb.nmi_pending = false,
            exception::PENDSV => self.scb.pendsv_pending = false,
            exception::SYSTICK => self.scb.systick_pending = false,
            e if e >= 16 => self.nvic.pending &= !(1 << (e - 16)),
            _ => {}
        }
    }

    /// Pushes the stack frame and branches to the handler of `exc`. `return_addr` is stacked as the PC.
    fn exception_entry(&mut self, bus: &mut dyn Bus, exc: u32, return_addr: u32) -> Result<u32, Fault> {
        let sp = self.r[13];
        let align = (sp >> 2) & 1;
        let frame = sp.wrapping_sub(0x20) & !4;
        let xpsr = self.xpsr() & !(1 << 9) | align << 9;
        let vals = [self.r[0], self.r[1], self.r[2], self.r[3], self.r[12], self.r[14], return_addr, xpsr];
        for (i, v) in vals.iter().enumerate() {
            self.write_mem(bus, frame + 4 * i as u32, Size::Word, *v)?;
        }
        self.r[13] = frame;
        self.r[14] = if self.ipsr != 0 {
            0xFFFF_FFF1
        } else if self.spsel {
            0xFFFF_FFFD
        } else {
            0xFFFF_FFF9
        };
        self.set_mode(exc, false);
        self.active |= 1 << exc;
        self.clear_pending(exc);
        let vector = self.read_mem(bus, self.scb.vtor.wrapping_add(4 * exc), Size::Word)?;
        self.r[15] = vector & !1;
        self.event_reg = true;
        if self.log_exceptions {
            self.exception_log.push((true, exc, return_addr));
        }
        // 15 cycles for entry on the M0+ (stacking + vector fetch), counted as in the TRM
        Ok(15)
    }

    /// Handles a branch to an EXC_RETURN value in handler mode.
    pub(crate) fn exception_return(&mut self, bus: &mut dyn Bus, exc_return: u32) -> Result<u32, Fault> {
        let (to_thread, use_psp) = match exc_return & 0xF {
            0x1 => (false, false),
            0x9 => (true, false),
            0xD => (true, true),
            _ => return Err(Fault::InvalidReturn(exc_return)),
        };
        let returning = self.ipsr;
        self.active &= !(1 << returning);
        if self.log_exceptions {
            self.exception_log.push((false, returning, 0));
        }
        // Select the stack the frame was pushed on
        self.set_mode(if to_thread { 0 } else { self.ipsr }, use_psp);
        let frame = self.r[13];
        let mut vals = [0u32; 8];
        for (i, v) in vals.iter_mut().enumerate() {
            *v = self.read_mem(bus, frame + 4 * i as u32, Size::Word)?;
        }
        self.r[0] = vals[0];
        self.r[1] = vals[1];
        self.r[2] = vals[2];
        self.r[3] = vals[3];
        self.r[12] = vals[4];
        self.r[14] = vals[5];
        self.r[15] = vals[6] & !1;
        let xpsr = vals[7];
        self.r[13] = frame.wrapping_add(0x20) | ((xpsr >> 9) & 1) << 2;
        self.set_apsr(xpsr);
        let new_ipsr = xpsr & 0x3F;
        // Mode comes from EXC_RETURN; a mismatched stacked IPSR is unpredictable, follow the stacked value in
        // handler mode only.
        let spsel = self.spsel;
        self.set_mode(if to_thread { 0 } else { new_ipsr }, spsel);
        self.event_reg = true;
        if to_thread && self.scb.scr & ppb::SCR_SLEEPONEXIT != 0 && self.preempting(false).is_none() {
            self.sleep = Some(Sleep::Wfi);
        }
        Ok(3)
    }

    /// Escalates a fault to HardFault (or locks up if already in HardFault or NMI).
    fn take_fault(&mut self, bus: &mut dyn Bus, fault: Fault) -> StepOutcome {
        let pc = self.r[15];
        self.last_fault = Some(FaultRecord { pc, fault });
        log::debug!("core fault at {pc:#010x}: {fault:?}");
        if self.ipsr == exception::HARD_FAULT || self.ipsr == exception::NMI {
            self.lockup = true;
            return StepOutcome::Lockup;
        }
        match self.exception_entry(bus, exception::HARD_FAULT, pc) {
            Ok(c) => {
                self.cycles += c as u64;
                StepOutcome::Ran(c)
            }
            Err(_) => {
                self.lockup = true;
                StepOutcome::Lockup
            }
        }
    }

    /// Turns a `BKPT` into a HardFault, as on a core with no debugger attached.
    pub fn escalate_breakpoint(&mut self, bus: &mut dyn Bus) -> StepOutcome {
        let imm = (self.last_opcode & 0xFF) as u8;
        self.take_fault(bus, Fault::Breakpoint(imm))
    }

    /// Requests an exception from outside (NMI from the SoC, or a test).
    pub fn set_nmi_pending(&mut self) {
        self.scb.nmi_pending = true;
    }

    // ------------------------------------------------------------------ memory

    pub(crate) fn read_mem(&mut self, bus: &mut dyn Bus, addr: u32, size: Size) -> Result<u32, Fault> {
        if addr & (size.bytes() - 1) != 0 {
            return Err(Fault::Unaligned(addr));
        }
        if addr >= ppb::PPB_BASE {
            return Ok(self.ppb_read(addr, size));
        }
        Ok(bus.read(addr, size)?)
    }

    pub(crate) fn write_mem(&mut self, bus: &mut dyn Bus, addr: u32, size: Size, v: u32) -> Result<(), Fault> {
        if addr & (size.bytes() - 1) != 0 {
            return Err(Fault::Unaligned(addr));
        }
        if addr >= ppb::PPB_BASE {
            self.ppb_write(addr, size, v);
            return Ok(());
        }
        Ok(bus.write(addr, size, v)?)
    }

    /// Debugger memory read: PPB registers come from this core, the rest from `bus` without side effects where
    /// the bus supports peeking.
    pub fn debug_read(&mut self, bus: &mut dyn Bus, addr: u32, size: Size) -> Option<u32> {
        if addr >= ppb::PPB_BASE {
            return Some(self.ppb_read(addr, size));
        }
        bus.peek(addr, size)
    }

    // ------------------------------------------------------------------ stepping

    /// Advances SysTick by `cycles` core clock cycles without executing (used while the core sleeps).
    pub fn advance_idle(&mut self, cycles: u64) {
        self.cycles += cycles;
        if self.systick.tick_core(cycles) {
            self.scb.systick_pending = true;
        }
    }

    /// SysTick reference clock ticks (the RP2040 feeds the 1 µs watchdog tick).
    pub fn tick_systick_reference(&mut self, ticks: u64) {
        if self.systick.tick_reference(ticks) {
            self.scb.systick_pending = true;
        }
    }

    /// Core clock cycles until SysTick next pends its exception, if it is counting the core clock.
    pub fn cycles_until_systick(&self) -> Option<u64> {
        self.systick.cycles_until_wrap()
    }

    /// Whether a sleeping core would wake now.
    pub fn would_wake(&mut self) -> bool {
        self.latch_irqs();
        match self.sleep {
            None => true,
            Some(Sleep::Wfi) => self.preempting(true).is_some(),
            Some(Sleep::Wfe) => self.event_reg || self.preempting(true).is_some(),
        }
    }

    fn step_inner(&mut self, bus: &mut dyn Bus) -> StepOutcome {
        if self.lockup {
            return StepOutcome::Lockup;
        }
        self.latch_irqs();

        if let Some(sleep) = self.sleep {
            let wake = match sleep {
                Sleep::Wfi => self.preempting(true).is_some(),
                Sleep::Wfe => {
                    if self.event_reg || self.preempting(true).is_some() {
                        self.event_reg = false;
                        true
                    } else {
                        false
                    }
                }
            };
            if !wake {
                return StepOutcome::Sleeping;
            }
            self.sleep = None;
        }

        if let Some(exc) = self.preempting(false) {
            let ret = self.r[15];
            return match self.exception_entry(bus, exc, ret) {
                Ok(c) => {
                    self.cycles += c as u64;
                    self.tick(c);
                    StepOutcome::Ran(c)
                }
                Err(_) => {
                    self.lockup = true;
                    StepOutcome::Lockup
                }
            };
        }

        let pc = self.r[15];
        match self.execute(bus) {
            Ok(cycles) => {
                let cycles = cycles + bus.take_stall_cycles();
                self.cycles += cycles as u64;
                self.tick(cycles);
                StepOutcome::Ran(cycles)
            }
            Err(Fault::Breakpoint(imm)) => {
                self.r[15] = pc;
                StepOutcome::Breakpoint(imm)
            }
            Err(f) => {
                self.r[15] = pc;
                self.take_fault(bus, f)
            }
        }
    }

    fn tick(&mut self, cycles: u32) {
        if self.systick.tick_core(cycles as u64) {
            self.scb.systick_pending = true;
        }
    }

    /// Ends the current sleep, for debuggers that halt a sleeping core.
    pub fn wake(&mut self) {
        self.sleep = None;
    }
}

impl Cpu for CortexM0Plus {
    fn step(&mut self, bus: &mut dyn Bus) -> StepOutcome {
        self.step_inner(bus)
    }

    fn reset(&mut self, bus: &mut dyn Bus, vtor: u32) {
        let cpuid = self.cpuid;
        let log = self.log_exceptions;
        *self = CortexM0Plus::new();
        self.cpuid = cpuid;
        self.log_exceptions = log;
        self.scb.vtor = vtor;
        let sp = bus.read(vtor, Size::Word).unwrap_or(0);
        let pc = bus.read(vtor + 4, Size::Word).unwrap_or(0);
        self.r[13] = sp & !3;
        self.r[14] = 0xFFFF_FFFF;
        self.r[15] = pc & !1;
    }

    fn pc(&self) -> u32 {
        self.r[15]
    }

    fn num_regs(&self) -> usize {
        reg::COUNT
    }

    fn read_reg(&self, n: usize) -> u32 {
        match n {
            0..=15 => self.r[n],
            reg::XPSR => self.xpsr(),
            reg::MSP => self.msp(),
            reg::PSP => self.psp(),
            reg::PRIMASK => self.primask as u32,
            reg::CONTROL => self.control(),
            _ => 0,
        }
    }

    fn write_reg(&mut self, n: usize, v: u32) {
        match n {
            0..=12 | 14 => self.r[n] = v,
            13 => self.r[13] = v & !3,
            15 => self.r[15] = v & !1,
            reg::XPSR => {
                self.set_apsr(v);
                let spsel = self.spsel;
                self.set_mode(v & 0x3F, spsel);
            }
            reg::MSP => self.set_msp(v),
            reg::PSP => self.set_psp(v),
            reg::PRIMASK => self.primask = v & 1 != 0,
            reg::CONTROL => {
                self.npriv = v & 1 != 0;
                let ipsr = self.ipsr;
                self.set_mode(ipsr, v & 2 != 0);
            }
            _ => {}
        }
    }

    fn is_sleeping(&self) -> bool {
        self.sleep.is_some()
    }

    fn cycles(&self) -> u64 {
        self.cycles
    }
}
