//! The RP2040 machine: two Cortex-M0+ cores, the system bus and the deterministic run loop.
//!
//! # Scheduling
//!
//! Each core keeps its own virtual time. The loop always runs the core that is furthest behind, for up to
//! [`Machine::quantum`] instructions and never past the next scheduled event, then fires every event both running
//! cores have reached. Peripheral accesses are stamped with the accessing core's time. A sleeping core (`WFI`/`WFE`
//! with nothing to wake it) stops constraining time; when every core sleeps, time jumps straight to the next event
//! ("idle fast-forward").

use std::collections::BTreeSet;
use std::sync::Arc;

use cpu_armv6m::CortexM0Plus;
use cpu_armv6m::disasm;
use sim_core::bus::Size;
use sim_core::cpu::{Cpu, StepOutcome};
use sim_core::image::Image;
use sim_core::time::{ClockPeriod, CycleClock, Time};
use sim_core::trace::{Category, NullSink, TraceEvent, TraceSink};

use crate::memmap::*;
use crate::system::{CoreBus, Event, System};

/// Machine configuration.
#[derive(Clone, Debug)]
pub struct MachineConfig {
    /// Size of the QSPI flash chip in bytes.
    pub flash_size: usize,
    /// Bootrom image mapped at address 0. `None` leaves the ROM empty (direct boot only).
    pub bootrom: Option<Vec<u8>>,
    /// Instructions a core runs before the other gets a turn.
    pub quantum: u32,
    /// `clk_sys` frequency used until the clock tree is modelled.
    pub clk_sys_hz: u64,
}

impl Default for MachineConfig {
    fn default() -> Self {
        MachineConfig { flash_size: 2 * 1024 * 1024, bootrom: None, quantum: 16, clk_sys_hz: 125_000_000 }
    }
}

/// Why [`Machine::run_until`] returned.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopReason {
    /// Reached the requested time.
    TimeLimit,
    /// A core is about to execute an instruction at a breakpoint address.
    Breakpoint { core: u8, pc: u32 },
    /// A core executed `BKPT`.
    BkptInstruction { core: u8, pc: u32, imm: u8 },
    /// A core locked up.
    Lockup { core: u8, pc: u32 },
    /// Every core sleeps and nothing is scheduled: nothing will ever happen again.
    Deadlock,
    /// [`Machine::request_stop`] was called (e.g. a GDB interrupt).
    Requested,
}

/// What to do with a `BKPT` instruction.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BkptPolicy {
    /// Stop the machine (a debugger is attached).
    Stop,
    /// Escalate to HardFault, as on a chip with no debugger.
    #[default]
    HardFault,
}

pub struct Machine {
    pub cores: [CortexM0Plus; 2],
    clocks: [CycleClock; 2],
    /// Whether each core executes at all. Core 1 is parked until something launches it.
    pub core_enabled: [bool; 2],
    pub sys: System,
    pub trace: Box<dyn TraceSink>,
    /// Events have been processed up to this time.
    pub now: Time,
    pub quantum: u32,
    clk_sys: ClockPeriod,
    pub image: Option<Arc<Image>>,
    pub breakpoints: BTreeSet<u32>,
    pub bkpt_policy: BkptPolicy,
    stop_requested: bool,
    /// Skip the breakpoint check for this core's next instruction (resuming from a breakpoint).
    skip_breakpoint: [bool; 2],
    trace_instructions: bool,
}

/// A snapshot of a machine: everything except the trace sink.
#[derive(Clone)]
pub struct Snapshot {
    cores: [CortexM0Plus; 2],
    clocks: [CycleClock; 2],
    core_enabled: [bool; 2],
    sys: System,
    now: Time,
    clk_sys: ClockPeriod,
}

impl Machine {
    pub fn new(config: MachineConfig) -> Machine {
        let mut cores = [CortexM0Plus::new(), CortexM0Plus::new()];
        cores[1].cpuid = 0x410C_C601;
        Machine {
            cores,
            clocks: [CycleClock::default(); 2],
            core_enabled: [true, false],
            sys: System::new(config.flash_size, config.bootrom.as_deref()),
            trace: Box::new(NullSink),
            now: Time::ZERO,
            quantum: config.quantum.max(1),
            clk_sys: ClockPeriod::from_hz(config.clk_sys_hz),
            image: None,
            breakpoints: BTreeSet::new(),
            bkpt_policy: BkptPolicy::default(),
            stop_requested: false,
            skip_breakpoint: [false; 2],
            trace_instructions: false,
        }
    }

    pub fn set_trace(&mut self, trace: Box<dyn TraceSink>) {
        self.trace_instructions = trace.wants(Category::Instruction);
        let exceptions = trace.wants(Category::Exception);
        for c in &mut self.cores {
            c.log_exceptions = exceptions;
        }
        self.trace = trace;
    }

    /// Current time of a core.
    pub fn core_time(&self, core: usize) -> Time {
        self.clocks[core].now
    }

    pub fn clk_sys(&self) -> ClockPeriod {
        self.clk_sys
    }

    /// Copies an image's segments into flash/SRAM/ROM at their load addresses.
    pub fn load(&mut self, image: &Image) -> Result<(), String> {
        for seg in &image.segments {
            for (i, &b) in seg.data.iter().enumerate() {
                let addr = seg.addr.wrapping_add(i as u32);
                if !self.sys.poke(addr, Size::Byte, b as u32) {
                    return Err(format!("image byte at {addr:#010x} is outside flash, SRAM and ROM"));
                }
            }
        }
        self.image = Some(Arc::new(image.clone()));
        Ok(())
    }

    /// Starts core 0 as if the bootrom and boot2 had run: VTOR at the application's vector table (flash offset
    /// `0x100`, after boot2, or flash offset 0 for images without boot2), SP and PC from it. Core 1 stays parked.
    pub fn direct_boot(&mut self) -> Result<u32, String> {
        let vtor = [XIP_BASE + 0x100, XIP_BASE, SRAM_BASE]
            .into_iter()
            .find(|&base| self.looks_like_vector_table(base))
            .ok_or("no vector table found at 0x10000100, 0x10000000 or 0x20000000")?;
        let mut trace = NullSink;
        let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: 0, now: Time::ZERO };
        self.cores[0].reset(&mut bus, vtor);
        self.core_enabled = [true, false];
        Ok(vtor)
    }

    fn looks_like_vector_table(&self, base: u32) -> bool {
        let (Some(sp), Some(pc)) = (self.sys.peek(base, Size::Word), self.sys.peek(base + 4, Size::Word)) else {
            return false;
        };
        let sp_ok = (SRAM_BASE..=SRAM_END).contains(&sp) || (XIP_SRAM_BASE..=XIP_SRAM_BASE + 0x4000).contains(&sp);
        let pc_ok = pc & 1 == 1
            && ((XIP_BASE..XIP_BASE + self.sys.flash.len() as u32).contains(&pc)
                || (SRAM_BASE..SRAM_END).contains(&pc));
        sp_ok && pc_ok
    }

    /// Asks [`Machine::run_until`] to return at the next instruction boundary.
    pub fn request_stop(&mut self) {
        self.stop_requested = true;
    }

    fn refresh_irqs(&mut self) {
        if self.sys.irq_dirty {
            self.sys.irq_dirty = false;
            for c in 0..2 {
                self.cores[c].set_irq_lines(self.sys.irq_lines(c as u8));
            }
        }
    }

    fn fire_events(&mut self, until: Time) {
        while let Some((t, ev)) = self.sys.sched.pop_due(until) {
            self.now = self.now.max(t);
            match ev {
                Event::External(_) => {}
            }
        }
        self.now = self.now.max(until);
    }

    /// Whether core `c` can make progress now (running, and awake or wakeable).
    fn runnable(&mut self, c: usize) -> bool {
        self.core_enabled[c] && !self.cores[c].is_locked_up() && self.cores[c].would_wake()
    }

    /// Brings a core that slept up to the current time, letting SysTick count the idle cycles.
    fn catch_up(&mut self, c: usize) {
        if self.clocks[c].now < self.now {
            let idle = self.now - self.clocks[c].now;
            let cycles = self.clk_sys.cycles_in(idle);
            self.cores[c].advance_idle(cycles);
            self.clocks[c].jump_to(self.now);
        }
    }

    /// Executes one instruction (or exception entry) on `core`.
    fn step_core(&mut self, c: usize) -> StepOutcome {
        let now = self.clocks[c].now;
        let mut bus = CoreBus { sys: &mut self.sys, trace: &mut *self.trace, core: c as u8, now };
        let outcome = self.cores[c].step(&mut bus);
        match outcome {
            StepOutcome::Ran(cycles) => {
                self.clocks[c].advance(&self.clk_sys, cycles as u64);
                if self.trace_instructions {
                    self.trace_instruction(c, now);
                }
                if self.cores[c].log_exceptions {
                    for (entry, exception, pc) in self.cores[c].take_exception_log() {
                        let ev = if entry {
                            TraceEvent::ExceptionEntry { core: c as u8, exception, return_pc: pc }
                        } else {
                            TraceEvent::ExceptionReturn { core: c as u8, exception }
                        };
                        self.trace.event(now, ev);
                    }
                }
                if self.cores[c].take_sev() {
                    self.cores[1 - c].signal_event();
                }
            }
            StepOutcome::Breakpoint(_) if self.bkpt_policy == BkptPolicy::HardFault => {
                let mut bus = CoreBus { sys: &mut self.sys, trace: &mut *self.trace, core: c as u8, now };
                self.cores[c].escalate_breakpoint(&mut bus);
            }
            _ => {}
        }
        outcome
    }

    fn trace_instruction(&mut self, c: usize, now: Time) {
        let cpu = &self.cores[c];
        let pc = cpu.last_pc;
        let op = cpu.last_opcode;
        let text = if op > 0xFFFF {
            disasm::disassemble(pc, (op >> 16) as u16, op as u16)
        } else {
            disasm::disassemble(pc, op as u16, 0)
        };
        self.trace.event(now, TraceEvent::Instruction { core: c as u8, pc, opcode: op, text });
    }

    /// Runs until virtual time `end` (or something stops the machine).
    pub fn run_until(&mut self, end: Time) -> StopReason {
        self.stop_requested = false;
        loop {
            if self.stop_requested {
                self.stop_requested = false;
                return StopReason::Requested;
            }
            self.refresh_irqs();
            let next_event = self.sys.sched.next_time();

            // Pick the runnable core furthest behind
            let mut pick: Option<usize> = None;
            for c in 0..2 {
                if self.runnable(c) {
                    self.catch_up(c);
                    if pick.is_none_or(|p| self.clocks[c].now < self.clocks[p].now) {
                        pick = Some(c);
                    }
                }
            }

            let Some(c) = pick else {
                // Every core sleeps: fast-forward to the next event, bounded by SysTick wake-ups
                let mut target = next_event.min(end);
                for c in 0..2 {
                    if self.core_enabled[c]
                        && let Some(cycles) = self.cores[c].cycles_until_systick()
                    {
                        let t = self.clocks[c].now.max(self.now) + self.clk_sys.cycles(cycles);
                        target = target.min(t);
                    }
                }
                if target == Time::MAX {
                    return StopReason::Deadlock;
                }
                if target >= end && next_event > end {
                    self.now = self.now.max(end);
                    for c in 0..2 {
                        if self.core_enabled[c] {
                            self.catch_up(c);
                        }
                    }
                    return StopReason::TimeLimit;
                }
                self.fire_events(target);
                for c in 0..2 {
                    if self.core_enabled[c] {
                        self.catch_up(c);
                    }
                }
                continue;
            };

            if self.clocks[c].now >= end {
                self.fire_events(end.min(next_event));
                if next_event > end {
                    return StopReason::TimeLimit;
                }
                continue;
            }

            // Run core c for a quantum, stopping at the next event or the end
            let limit = next_event.min(end);
            for _ in 0..self.quantum {
                let pc = self.cores[c].pc();
                if !self.breakpoints.is_empty() && self.breakpoints.contains(&pc) && !self.cores[c].is_sleeping() {
                    if self.skip_breakpoint[c] {
                        self.skip_breakpoint[c] = false;
                    } else {
                        self.skip_breakpoint[c] = true;
                        return StopReason::Breakpoint { core: c as u8, pc };
                    }
                }
                match self.step_core(c) {
                    StepOutcome::Ran(_) => {}
                    StepOutcome::Sleeping => break,
                    StepOutcome::Breakpoint(imm) => {
                        if self.bkpt_policy == BkptPolicy::Stop {
                            return StopReason::BkptInstruction { core: c as u8, pc, imm };
                        }
                    }
                    StepOutcome::Lockup => {
                        log::error!("core{c} locked up at {:#010x}", self.cores[c].pc());
                        return StopReason::Lockup { core: c as u8, pc: self.cores[c].pc() };
                    }
                }
                self.refresh_irqs();
                if self.clocks[c].now >= limit || self.cores[c].is_sleeping() || self.stop_requested {
                    break;
                }
            }

            // Fire events every running core has reached
            let mut horizon = Time::MAX;
            for c in 0..2 {
                if self.core_enabled[c] && !self.cores[c].is_sleeping() && !self.cores[c].is_locked_up() {
                    horizon = horizon.min(self.clocks[c].now);
                }
            }
            if horizon == Time::MAX {
                continue;
            }
            if next_event <= horizon {
                self.fire_events(horizon);
            } else {
                self.now = self.now.max(horizon.min(next_event));
            }
        }
    }

    /// Runs for `d` of virtual time from now.
    pub fn run_for(&mut self, d: Time) -> StopReason {
        let end = self.now + d;
        self.run_until(end)
    }

    /// Executes exactly one instruction on `core` (debugger single step), ignoring breakpoints.
    pub fn single_step(&mut self, core: usize) -> StepOutcome {
        self.refresh_irqs();
        self.cores[core].wake();
        let outcome = self.step_core(core);
        let t = self.clocks[core].now;
        if self.sys.sched.next_time() <= t {
            self.fire_events(t);
        }
        self.skip_breakpoint[core] = false;
        outcome
    }

    /// Takes a snapshot of the machine's state.
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            cores: self.cores.clone(),
            clocks: self.clocks,
            core_enabled: self.core_enabled,
            sys: self.sys.clone(),
            now: self.now,
            clk_sys: self.clk_sys,
        }
    }

    /// Restores a snapshot taken with [`Machine::snapshot`]. The trace sink and breakpoints are kept.
    pub fn restore(&mut self, s: &Snapshot) {
        self.cores = s.cores.clone();
        self.clocks = s.clocks;
        self.core_enabled = s.core_enabled;
        self.sys = s.sys.clone();
        self.now = s.now;
        self.clk_sys = s.clk_sys;
        self.sys.irq_dirty = true;
    }

    /// Debugger memory read from `core`'s point of view (its PPB registers included).
    pub fn debug_read(&mut self, core: usize, addr: u32, size: Size) -> Option<u32> {
        let mut trace = NullSink;
        let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: core as u8, now: self.now };
        self.cores[core].debug_read(&mut bus, addr, size)
    }

    /// Symbol name and offset for an address, if the loaded image has one.
    pub fn symbolize(&self, addr: u32) -> Option<String> {
        let image = self.image.as_ref()?;
        let (sym, off) = image.symbolize(addr)?;
        Some(if off == 0 { sym.name.clone() } else { format!("{}+{off:#x}", sym.name) })
    }
}
