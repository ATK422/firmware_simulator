//! The RP2040 machine: two Cortex-M0+ cores, the system bus and the deterministic run loop.
//!
//! # Scheduling
//!
//! Each core keeps its own virtual time. The loop always runs the core that is furthest behind, for up to
//! [`Machine::quantum`] instructions and never past the next scheduled event, then fires every event both running
//! cores have reached. Peripheral accesses are stamped with the accessing core's time. A sleeping core (`WFI`/`WFE`
//! with nothing to wake it) stops constraining time; when every core sleeps, time jumps straight to the next event
//! ("idle fast-forward").

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use cpu_armv6m::CortexM0Plus;
use cpu_armv6m::disasm;
use sim_core::bus::Size;
use sim_core::cpu::{Cpu, StepOutcome};
use sim_core::image::Image;
use sim_core::time::{ClockPeriod, CycleClock, Time};
use sim_core::trace::{Category, NullSink, TraceEvent, TraceSink};

use crate::hle::{self, RomFn};
use crate::memmap::*;
use crate::periph::sysctl::psm;
use crate::periph::timer::WatchdogReset;
use crate::system::{CoreBus, Event, ResetRequest, System};

/// Time from releasing core 1's PSM force-off until it executes its first instruction (a few `clk_ref` cycles of
/// PSM sequencing). Firmware relies on this: the bootrom's core 1 handshake starts with an unconditional FIFO read
/// that would set the sticky ROE flag if core 0's first command had not arrived yet.
pub const CORE1_RELEASE_LATENCY: Time = Time::from_us(1);

/// Machine configuration.
#[derive(Clone, Debug)]
pub struct MachineConfig {
    /// Size of the QSPI flash chip in bytes.
    pub flash_size: usize,
    /// Bootrom image mapped at address 0. `None` leaves the ROM empty (direct boot only).
    pub bootrom: Option<Vec<u8>>,
    /// Instructions a core runs before the other gets a turn.
    pub quantum: u32,
    /// Crystal frequency (XOSC).
    pub crystal_hz: u64,
}

impl Default for MachineConfig {
    fn default() -> Self {
        MachineConfig { flash_size: 2 * 1024 * 1024, bootrom: None, quantum: 16, crystal_hz: 12_000_000 }
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
    /// The core that ran the last quantum (tie-breaker for scheduling).
    last_ran: usize,
    /// Vector table used by direct boot, reused after resets.
    boot_vtor: Option<u32>,
    /// Boot through the bootrom and boot2 (see [`Machine::rom_boot`]) rather than directly.
    rom_boot: bool,
    /// Number of chip resets since power-on.
    pub resets: u32,
    /// Trapped ROM functions by entry address (see [`crate::hle`]).
    pub hle: BTreeMap<u32, RomFn>,
    /// Whether ROM flash functions are trapped. Off once the flash chip is modelled at the SSI level.
    pub hle_enabled: bool,
    /// Firmware asked to reboot into BOOTSEL mode.
    pub usb_boot_requested: bool,
    /// `External` events that fired, as `(time, id)`, for the host to collect.
    pub external_fired: Vec<(Time, u32)>,
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
        let mut m = Machine {
            cores,
            clocks: [CycleClock::default(); 2],
            core_enabled: [true, false],
            sys: {
                let mut sys = System::new(config.flash_size, config.bootrom.as_deref());
                sys.clocks.xosc.crystal_hz = config.crystal_hz;
                sys
            },
            trace: Box::new(NullSink),
            now: Time::ZERO,
            quantum: config.quantum.max(1),
            clk_sys: ClockPeriod::from_hz(1),
            image: None,
            breakpoints: BTreeSet::new(),
            bkpt_policy: BkptPolicy::default(),
            stop_requested: false,
            last_ran: 1,
            boot_vtor: None,
            rom_boot: false,
            resets: 0,
            external_fired: Vec::new(),
            hle: BTreeMap::new(),
            hle_enabled: false,
            usb_boot_requested: false,
            skip_breakpoint: [false; 2],
            trace_instructions: false,
        };
        m.clk_sys = m.sys.clk_sys;
        m.sys.clk_sys_changed = false;
        m.hle = hle::rom_functions(&m.sys.rom);
        m
    }

    pub fn set_trace(&mut self, trace: Box<dyn TraceSink>) {
        self.trace_instructions = trace.wants(Category::Instruction);
        // Tracing GPIO makes every PWM edge visible
        self.sys.pwm.observed = if trace.wants(Category::Gpio) { 0xFF } else { 0 };
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
        self.boot_vtor = Some(vtor);
        self.rom_boot = false;
        self.reboot();
        Ok(vtor)
    }

    /// Starts both cores in the bootrom, as after power-on: the bootrom loads boot2 from flash through the SSI and
    /// the QSPI flash model, boot2 sets up XIP and enters the application. Needs the bootrom mapped. Resets reboot
    /// the same way.
    pub fn rom_boot(&mut self) -> Result<(), String> {
        if !self.has_bootrom() {
            return Err("ROM boot needs the bootrom".into());
        }
        self.rom_boot = true;
        self.boot_vtor = None;
        self.reboot();
        Ok(())
    }

    /// Whether the machine boots through the bootrom.
    pub fn boots_from_rom(&self) -> bool {
        self.rom_boot
    }

    /// Whether a bootrom with a plausible vector table is mapped.
    pub fn has_bootrom(&self) -> bool {
        let sp = self.sys.peek(0, Size::Word).unwrap_or(0);
        let pc = self.sys.peek(4, Size::Word).unwrap_or(0);
        (SRAM_BASE..=SRAM_END).contains(&sp) && pc & 1 == 1 && (pc as usize) < ROM_SIZE
    }

    /// Starts core 1 in the bootrom, where it waits for the launch sequence on the SIO FIFO (as after reset).
    fn boot_core1_rom(&mut self) {
        let now = self.clocks[0].now.max(self.now);
        self.boot_core1_rom_at(now);
    }

    fn boot_core1_rom_at(&mut self, now: Time) {
        if !self.has_bootrom() {
            self.core_enabled[1] = false;
            return;
        }
        let log = self.cores[1].log_exceptions;
        self.cores[1] = CortexM0Plus::new();
        self.cores[1].log_exceptions = log;
        self.clocks[1] = CycleClock::new(now);
        let mut trace = NullSink;
        let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: 1, now };
        self.cores[1].reset(&mut bus, 0);
        self.core_enabled[1] = true;
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
            self.sys.fire_event(&mut *self.trace, t, ev);
            match ev {
                Event::External(id) => self.external_fired.push((t, id)),
                Event::Core1Boot => self.boot_core1_rom_at(t),
                _ => {}
            }
            self.after_system_change();
        }
        self.now = self.now.max(until);
    }

    /// Picks up effects of peripheral activity that concern the machine: a new `clk_sys` period, resets, PSM.
    fn after_system_change(&mut self) {
        if self.sys.clk_sys_changed {
            self.sys.clk_sys_changed = false;
            self.clk_sys = self.sys.clk_sys;
        }
        if let Some(req) = self.sys.reset_request.take() {
            match req {
                ResetRequest::Watchdog(kind) => self.watchdog_reset(kind),
            }
        }
        if self.sys.psm_forced_off != 0 || self.sys.psm_released != 0 {
            let off = std::mem::take(&mut self.sys.psm_forced_off);
            let released = std::mem::take(&mut self.sys.psm_released);
            if off & (1 << psm::PROC1) != 0 {
                // Core 1 is held in reset while forced off
                self.core_enabled[1] = false;
                self.cores[1] = CortexM0Plus::new();
            }
            if released & (1 << psm::PROC1) != 0 {
                // Out of reset again after the PSM sequences the domain back on: core 1 then runs the bootrom and
                // waits for the launch sequence
                let t = self.clocks[0].now.max(self.clocks[1].now).max(self.now);
                self.sys.sched.schedule(Event::Core1Boot, t + CORE1_RELEASE_LATENCY);
            }
        }
    }

    /// Resets the chip from the watchdog: peripherals and cores restart, RAM and scratch registers survive.
    pub fn watchdog_reset(&mut self, kind: WatchdogReset) {
        let t = self.clocks[0].now.max(self.clocks[1].now).max(self.now);
        log::info!("[{t}] watchdog reset ({kind:?})");
        self.resets += 1;
        self.sys.watchdog.record_reset(kind);
        self.sys.watchdog_reset(t);
        self.clk_sys = self.sys.clk_sys;
        self.sys.clk_sys_changed = false;
        self.now = t;
        for c in 0..2 {
            let log = self.cores[c].log_exceptions;
            self.cores[c] = CortexM0Plus::new();
            self.cores[c].log_exceptions = log;
            self.clocks[c] = CycleClock::new(t);
        }
        self.reboot();
    }

    /// Loads the flash contents from a file (a raw image of the whole flash), if it exists. Returns whether it did.
    pub fn load_flash_file(&mut self, path: &std::path::Path) -> std::io::Result<bool> {
        match std::fs::read(path) {
            Ok(data) => {
                let n = data.len().min(self.sys.flash.len());
                self.sys.flash[..n].copy_from_slice(&data[..n]);
                Ok(true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Writes the flash contents to a file.
    pub fn save_flash_file(&self, path: &std::path::Path) -> std::io::Result<()> {
        std::fs::write(path, &self.sys.flash)
    }

    /// Removes power and applies it again: every block resets, including the watchdog's scratch registers and
    /// REASON, CHIP_RESET reports a power-on reset, and SRAM comes up with arbitrary contents. The flash keeps its
    /// data and its non-volatile status bits; attached devices stay attached.
    pub fn power_cycle(&mut self) {
        self.chip_reset(crate::periph::sysctl::ChipResetCause::PowerOn);
    }

    /// Pulls the RUN pin low and releases it: a chip reset like power-on, except that SRAM keeps its contents.
    pub fn run_pin_reset(&mut self) {
        self.chip_reset(crate::periph::sysctl::ChipResetCause::RunPin);
    }

    fn chip_reset(&mut self, cause: crate::periph::sysctl::ChipResetCause) {
        use crate::periph::sysctl::ChipResetCause;
        let t = self.clocks[0].now.max(self.clocks[1].now).max(self.now);
        log::info!("[{t}] chip reset ({cause:?})");
        self.resets += 1;
        self.sys.watchdog_reset(t);
        let ref_hz = self.sys.freqs.clk_ref;
        self.sys.watchdog = crate::periph::timer::Watchdog::new(ref_hz);
        self.sys.timer.restart(&self.sys.watchdog.tick, t);
        self.sys.vreg.record(cause);
        self.sys.ssi.chip.power_up();
        self.sys.refresh_xip();
        if cause == ChipResetCause::PowerOn {
            // Arbitrary but reproducible power-up contents
            let mut x = 0x9E37_79B9_7F4A_7C15u64 ^ t.as_ps();
            for b in self.sys.sram.iter_mut().chain(self.sys.xip_sram.iter_mut()) {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                *b = x as u8;
            }
        }
        self.clk_sys = self.sys.clk_sys;
        self.sys.clk_sys_changed = false;
        self.now = t;
        for c in 0..2 {
            let log = self.cores[c].log_exceptions;
            self.cores[c] = CortexM0Plus::new();
            self.cores[c].log_exceptions = log;
            self.clocks[c] = CycleClock::new(t);
        }
        self.reboot();
    }

    /// Carries out a trapped ROM function for core `c` and returns to its caller.
    fn run_hle(&mut self, c: usize, f: RomFn) -> StepOutcome {
        let cpu = &self.cores[c];
        let (r0, r1, r2) = (cpu.reg(0), cpu.reg(1), cpu.reg(2));
        let lr = cpu.reg(14);
        let now = self.clocks[c].now;
        let duration = match f {
            RomFn::FlashRangeErase => {
                log::debug!("[{now}] core{c} flash_range_erase({r0:#x}, {r1:#x}, {r2:#x})");
                hle::erase(&mut self.sys.flash, r0, r1, r2)
            }
            RomFn::FlashRangeProgram => {
                log::debug!("[{now}] core{c} flash_range_program({r0:#x}, {r1:#010x}, {r2:#x})");
                let data: Vec<u8> =
                    (0..r2).map(|i| self.sys.peek(r1.wrapping_add(i), Size::Byte).unwrap_or(0xFF) as u8).collect();
                hle::program(&mut self.sys.flash, r0, &data)
            }
            RomFn::ResetToUsbBoot => {
                log::warn!("[{now}] core{c} called reset_usb_boot (BOOTSEL mode is not emulated)");
                self.usb_boot_requested = true;
                self.request_stop();
                Time::from_us(1)
            }
            RomFn::ConnectInternalFlash | RomFn::FlashExitXip => {
                // Leave the SSI as the ROM does, so firmware can send its own commands (pico-sdk's flash_do_cmd)
                self.sys.ssi.enter_command_mode();
                self.sys.refresh_xip();
                Time::from_us(2)
            }
            RomFn::FlashFlushCache | RomFn::FlashEnterCmdXip => Time::from_us(2),
        };
        self.cores[c].set_pc(lr & !1);
        let cycles = self.clk_sys.cycles_in(duration);
        self.cores[c].advance_idle(cycles);
        self.clocks[c].advance(&self.clk_sys, cycles);
        StepOutcome::Ran(0)
    }

    /// AIRCR.SYSRESETREQ: on the RP2040 this resets the requesting processor only (its PSM domain), not the
    /// peripherals or SIO.
    pub fn core_reset(&mut self, c: usize) {
        log::info!("[{}] core{c} SYSRESETREQ", self.clocks[c].now);
        let log = self.cores[c].log_exceptions;
        self.cores[c] = CortexM0Plus::new();
        self.cores[c].log_exceptions = log;
        if c == 0 && self.rom_boot {
            let mut trace = NullSink;
            let now = self.clocks[0].now;
            let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: 0, now };
            self.cores[0].reset(&mut bus, 0);
        } else if c == 0 {
            if let Some(vtor) = self.boot_vtor {
                let mut trace = NullSink;
                let now = self.clocks[0].now;
                let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: 0, now };
                self.cores[0].reset(&mut bus, vtor);
            }
        } else {
            self.core_enabled[1] = false;
        }
    }

    /// Starts the cores again after a reset.
    fn reboot(&mut self) {
        if self.rom_boot {
            // Every block starts in reset; the bootrom releases what it needs. Both cores run the bootrom; core 1
            // parks itself waiting for the launch sequence.
            self.sys.resets = crate::periph::sysctl::Resets::power_on();
            let now = self.now;
            for c in 0..2 {
                let mut trace = NullSink;
                let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: c as u8, now };
                self.cores[c].reset(&mut bus, 0);
                self.clocks[c] = CycleClock::new(now.max(self.clocks[c].now));
            }
            self.core_enabled = [true, true];
            return;
        }
        if self.boot_vtor.is_some() {
            // As if the bootrom and boot2 had run
            self.sys.preset_xip();
        }
        match self.boot_vtor {
            Some(vtor) => {
                let mut trace = NullSink;
                let now = self.now;
                let mut bus = CoreBus { sys: &mut self.sys, trace: &mut trace, core: 0, now };
                self.cores[0].reset(&mut bus, vtor);
                self.core_enabled = [true, false];
                self.boot_core1_rom();
            }
            None => self.core_enabled = [false, false],
        }
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
        let pc = self.cores[c].pc();
        if (pc as usize) < ROM_SIZE
            && self.hle_enabled
            && !self.cores[c].is_sleeping()
            && let Some(f) = self.hle.get(&pc).copied()
        {
            return self.run_hle(c, f);
        }
        let mut bus = CoreBus { sys: &mut self.sys, trace: &mut *self.trace, core: c as u8, now };
        let outcome = self.cores[c].step(&mut bus);
        match outcome {
            StepOutcome::Ran(cycles) => {
                self.clocks[c].advance(&self.clk_sys, cycles as u64);
                if self.sys.clk_sys_changed
                    || self.sys.reset_request.is_some()
                    || self.sys.psm_forced_off != 0
                    || self.sys.psm_released != 0
                {
                    self.after_system_change();
                }
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
                if self.cores[c].take_reset_request() {
                    self.core_reset(c);
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
                    // Ties go to the core that did not run last, so a core woken at the other core's current
                    // time is not starved by a busy-waiting partner
                    let earlier = |p: usize| {
                        self.clocks[c].now < self.clocks[p].now
                            || (self.clocks[c].now == self.clocks[p].now && c != self.last_ran)
                    };
                    if pick.is_none_or(earlier) {
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
            self.last_ran = c;
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

    /// Lets attached devices update their outputs (after the host delivered something to them).
    pub fn update_devices(&mut self) {
        let now = self.now;
        self.sys.update_devices(&mut *self.trace, now);
    }

    /// The CAN controllers among the attached devices.
    pub fn can_nodes(&mut self) -> Vec<&mut dyn sim_devices::can::CanNode> {
        self.sys.spi_devices.iter_mut().filter_map(|a| a.dev.can_node()).collect()
    }

    /// Debugger memory read from `core`'s point of view (its PPB registers included).
    pub fn debug_read(&mut self, core: usize, addr: u32, size: Size) -> Option<u32> {
        self.sys.peek_time = self.clocks[core].now.max(self.now);
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
