//! Differential testing: the same programs run on two engines, compared after every quantum.
//!
//! An [`Engine`] is a way of running the core: the reference interpreter one instruction at a time ([`Engine::Step`],
//! as the machine runs while anything needs to see each instruction), or batches ([`CortexM0Plus::run_batch`]) that
//! interpret ([`Engine::Interpreter`]) or run decoded blocks ([`Engine::Blocks`]). A driver ([`Sim`]) runs the core
//! as the machine's run loop does: quanta of a few instructions that stop at the next event, events raising interrupt
//! lines between quanta, interrupt lines handed over after each step or batch, `BKPT` taken as a HardFault, idle
//! time skipped to the next event while the core sleeps, SysTick on the reference clock fed lazily with batches ending
//! before it pends its exception. What the driver does depends only on the core's and the bus's state, so two engines
//! that agree go through the same quanta; after each, everything is compared: every register and flag, the exception
//! state, NVIC, SysTick and SCB, cycle and instruction counts, the bus's state and every data access since the last
//! quantum (address, size, value, read or write, and the time it happened at). Memory is compared at the end.
//!
//! The programs ([`programs`]) are random Thumb code from every instruction class, faulting and taking interrupts that
//! arrive mid-block at chosen cycles and from its own stores to the test registers ([`bus`]).

pub(crate) mod bus;
pub(crate) mod programs;

use sim_core::cpu::StepOutcome;

use crate::CortexM0Plus;
use crate::disasm;
use bus::DiffBus;

/// A way of running the core.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Engine {
    /// [`CortexM0Plus::step_on`] for every instruction: the reference.
    Step,
    /// Batches, every instruction decoded as it runs.
    Interpreter,
    /// Batches running decoded blocks.
    Blocks,
    /// Batches running regions of decoded blocks compiled to native code as soon as their first block runs whole
    /// ([`crate::jit`]), accessing SRAM, flash and ROM directly ([`sim_core::Bus::direct_memory`]).
    Jit,
    /// The same, compiling after a region's first block ran whole three times (so that blocks also run interpreted,
    /// and compiled code is entered from interpreted code and the other way round), every access through the bus.
    JitWarm,
}

impl Engine {
    fn configure(self, cpu: &mut CortexM0Plus) {
        cpu.decode_blocks = matches!(self, Engine::Blocks | Engine::Jit | Engine::JitWarm);
        cpu.jit = matches!(self, Engine::Jit | Engine::JitWarm);
        cpu.jit_threshold = if self == Engine::JitWarm { 3 } else { 0 };
        cpu.jit_sync = true;
    }

    fn batched(self) -> bool {
        self != Engine::Step
    }

    /// Whether compiled code accesses memory directly: its accesses there are not logged.
    fn direct(self) -> bool {
        self == Engine::Jit
    }
}

/// Whether an access is to memory a bus may let compiled code access directly (not logged then).
fn plain_memory(a: &bus::Access) -> bool {
    let in_ = |base: u32, size: u32| a.addr.wrapping_sub(base) < size;
    in_(bus::SRAM_BASE, bus::SRAM_SIZE) || in_(bus::ROM_BASE, bus::ROM_SIZE) || in_(bus::FLASH_BASE, bus::FLASH_SIZE)
}

/// A core and its bus, run as the machine runs a core.
pub(crate) struct Sim {
    pub cpu: CortexM0Plus,
    pub bus: DiffBus,
    engine: Engine,
    quantum: u32,
    trap_below: u32,
    /// Steps taken (instructions, exception entries, sleeping steps).
    pub steps: u64,
    pub quanta: u64,
    /// Times idle time was skipped with nothing to wake the core, which then is woken.
    idle: u32,
    pub done: bool,
    /// Print every step and batch.
    pub trace: bool,
}

impl Sim {
    pub fn new(p: &programs::Program, engine: Engine) -> Sim {
        let mut cpu = p.cpu.clone();
        engine.configure(&mut cpu);
        let mut bus = p.bus.clone();
        bus.direct = engine.direct();
        Sim {
            cpu,
            bus,
            engine,
            quantum: p.quantum,
            trap_below: p.trap_below,
            steps: 0,
            quanta: 0,
            idle: 0,
            done: false,
            trace: false,
        }
    }

    fn now(&self) -> u64 {
        self.cpu.cycles
    }

    /// Feeds SysTick the reference ticks up to now (the machine's `sync_systick_reference`).
    fn sync_reference(&mut self) {
        if !self.cpu.systick.counts_reference_clock() {
            self.bus.ref_seen = None;
            return;
        }
        let ticks = self.bus.ticks(self.now());
        match self.bus.ref_seen {
            None => self.bus.ref_seen = Some(ticks),
            Some(seen) if ticks > seen => {
                self.cpu.tick_systick_reference(ticks - seen);
                self.bus.ref_seen = Some(ticks);
            }
            Some(_) => {}
        }
    }

    /// When SysTick on the reference clock wraps and pends its exception, if it will.
    fn reference_wrap(&self) -> Option<u64> {
        let seen = self.bus.ref_seen?;
        let n = self.cpu.systick.reference_ticks_until_wrap()?;
        Some((seen + n) * self.bus.tick_period)
    }

    /// What the machine does after a step or batch: hand the core its lines, collect `SEV` and reset requests.
    fn after_step(&mut self) {
        self.cpu.set_irq_lines(self.bus.lines);
        self.cpu.take_sev();
        self.cpu.take_reset_request();
        if self.cpu.lockup {
            self.done = true;
        }
    }

    /// Runs one quantum, or skips idle time while the core sleeps.
    pub fn quantum(&mut self) {
        self.quanta += 1;
        let now = self.now();
        self.bus.fire(now);
        self.cpu.set_irq_lines(self.bus.lines);
        if self.cpu.lockup {
            self.done = true;
            return;
        }
        if self.cpu.sleep.is_some() {
            self.sync_reference();
            if !self.cpu.would_wake() {
                let mut target = self.bus.next_event();
                if let Some(c) = self.cpu.cycles_until_systick() {
                    target = Some(target.map_or(now + c, |t| t.min(now + c)));
                }
                if let Some(t) = self.reference_wrap() {
                    target = Some(target.map_or(t, |x| x.min(t)));
                }
                match target {
                    Some(t) if t > now => self.cpu.advance_idle(t - now),
                    _ => {
                        // Nothing would ever wake it (or something already should have): wake it, as a debugger
                        self.idle += 1;
                        self.cpu.advance_idle(16);
                        if self.idle.is_multiple_of(2) {
                            self.cpu.signal_event();
                        } else {
                            self.cpu.wake();
                        }
                    }
                }
                return;
            }
        }
        let limit = self.bus.next_event().unwrap_or(u64::MAX).min(now + 2000);
        let mut left = self.quantum;
        while left > 0 && !self.done {
            self.sync_reference();
            let now = self.now();
            self.bus.begin(now);
            if self.engine.batched() && self.cpu.r[15] >= self.trap_below {
                let mut budget = limit.saturating_sub(now);
                if let Some(t) = self.reference_wrap() {
                    // From a moment before the tick that pends it (`Machine::systick_reference_pend`)
                    budget = budget.min(t.saturating_sub(now));
                }
                let pc = self.cpu.r[15];
                let b = self.cpu.run_batch(&mut self.bus, left, budget, self.trap_below);
                if self.trace {
                    eprintln!("batch at {pc:#x} t {now}: {b:?} -> pc {:#x} t {}", self.cpu.r[15], self.now());
                }
                if b.steps > 0 {
                    assert!(b.steps <= left);
                    left -= b.steps;
                    self.steps += b.steps as u64;
                    match b.stop {
                        Some(StepOutcome::Breakpoint(_)) => {
                            self.bus.begin(self.now());
                            self.cpu.escalate_breakpoint(&mut self.bus);
                        }
                        Some(StepOutcome::Lockup) => self.done = true,
                        _ => {}
                    }
                    self.after_step();
                    if self.now() >= limit || self.cpu.sleep.is_some() {
                        break;
                    }
                    continue;
                }
            }
            left -= 1;
            self.steps += 1;
            let pc = self.cpu.r[15];
            let out = self.cpu.step_on(&mut self.bus);
            if self.trace {
                eprintln!("step at {pc:#x} t {now}: {out:?} -> pc {:#x} t {}", self.cpu.r[15], self.now());
            }
            match out {
                StepOutcome::Breakpoint(_) => {
                    self.bus.begin(self.now());
                    self.cpu.escalate_breakpoint(&mut self.bus);
                }
                StepOutcome::Lockup => self.done = true,
                StepOutcome::Sleeping => {
                    self.after_step();
                    break;
                }
                StepOutcome::Ran(_) => {}
            }
            self.after_step();
            if self.now() >= limit || self.cpu.sleep.is_some() {
                break;
            }
        }
        // (SysTick is fed lazily in batches: as fed up to here, to compare)
        self.sync_reference();
        self.bus.flush();
    }

    /// Everything about the core that a run determines ([`CortexM0Plus::comparable_state`]).
    pub fn core_state(&self) -> String {
        self.cpu.comparable_state()
    }

    /// The instructions around `pc`, for messages.
    fn code_around(&mut self, pc: u32) -> String {
        let mut s = String::new();
        for k in 0..12u32 {
            let at = pc.wrapping_sub(12).wrapping_add(2 * k);
            let (Some(h1), Some(h2)) = (self.bus.peek16(at), self.bus.peek16(at.wrapping_add(2))) else { continue };
            let mark = if at == pc { "=>" } else { "  " };
            s += &format!("{mark} {at:#010x}: {:04x} {}\n", h1, disasm::disassemble(at, h1, h2));
        }
        s
    }
}

/// The first line where two multi-line strings differ, with both versions.
fn first_difference(a: &str, b: &str) -> String {
    let (x, y): (Vec<&str>, Vec<&str>) = (a.split(", ").collect(), b.split(", ").collect());
    match x.iter().zip(&y).position(|(p, q)| p != q) {
        Some(i) => format!("{}\n    vs {}", x[i], y[i]),
        None if x.len() == y.len() => "(the same)".into(),
        None => format!("{} vs {} fields", x.len(), y.len()),
    }
}

/// Runs program `seed` on engines `a` and `b`, comparing after every quantum; panics with a report at the first
/// difference. Returns the steps run.
pub(crate) fn compare_program(seed: u64, a: Engine, b: Engine) -> u64 {
    let p = programs::generate(seed);
    let mut x = Sim::new(&p, a);
    let mut y = Sim::new(&p, b);
    let mut history: Vec<u32> = Vec::new();
    while !x.done && x.steps < p.steps && x.quanta < 4 * p.steps {
        let pc = x.cpu.r[15];
        history.push(pc);
        x.quantum();
        y.quantum();
        let (sx, sy) = (x.core_state(), y.core_state());
        let (bx, by) = (x.bus.state(), y.bus.state());
        if a.direct() || b.direct() {
            // What compiled code reads and writes directly goes unlogged: memory is compared instead
            x.bus.log.retain(|a| !plain_memory(a));
            y.bus.log.retain(|a| !plain_memory(a));
        }
        let memory = (a.direct() || b.direct()) && x.bus.sram != y.bus.sram;
        let same = !memory && sx == sy && bx == by && x.bus.log == y.bus.log && x.steps == y.steps && x.done == y.done;
        if !same {
            let log_diff = x
                .bus
                .log
                .iter()
                .zip(&y.bus.log)
                .position(|(p, q)| p != q)
                .map_or(format!("{} vs {} accesses", x.bus.log.len(), y.bus.log.len()), |i| {
                    format!("access {i}: {:?}\n    vs {:?}", x.bus.log[i], y.bus.log[i])
                });
            let code = x.code_around(pc);
            let recent: Vec<String> = history.iter().rev().take(4).map(|pc| format!("{pc:#x}")).collect();
            panic!(
                "program {seed:#x} (main loop at {:#x}..{:#x}, quantum {}, trap below {:#x}): {a:?} and {b:?} differ \
                 after quantum {} ({} / {} steps), which started at pc {pc:#x} (quanta before at {})\n\
                 core: {}\nbus: {}\nlog: {log_diff}\nSRAM differs: {memory}\nsteps {} vs {}, done {} vs {}\n\
                 {a:?}:\n  {sx}\n{b:?}:\n  {sy}\ncode at the start of the quantum:\n{}",
                p.code.0,
                p.code.1,
                p.quantum,
                p.trap_below,
                x.quanta,
                x.steps,
                y.steps,
                recent.join(", "),
                first_difference(&sx, &sy),
                first_difference(&bx, &by),
                x.steps,
                y.steps,
                x.done,
                y.done,
                code,
            );
        }
        x.bus.log.clear();
        y.bus.log.clear();
    }
    assert!(x.bus.sram == y.bus.sram, "program {seed:#x}: {a:?} and {b:?} leave different SRAM");
    x.steps
}

/// Runs `n` programs from `seed` on both engines (more with `SIM_DIFF_PROGRAMS`, a count).
pub(crate) fn compare_engines(seed: u64, n: u64, a: Engine, b: Engine) {
    let n = std::env::var("SIM_DIFF_PROGRAMS").ok().and_then(|v| v.parse().ok()).unwrap_or(n);
    let mut steps = 0;
    for k in 0..n {
        steps += compare_program(seed.wrapping_add(k.wrapping_mul(0x9E37_79B9_7F4A_7C15)), a, b);
    }
    eprintln!("{a:?} vs {b:?}: {n} programs, {steps} steps");
}

#[test]
fn blocks_run_as_single_steps() {
    compare_engines(0xB10C_5000, 1500, Engine::Step, Engine::Blocks);
}

#[test]
fn interpreted_batches_run_as_single_steps() {
    compare_engines(0x1A7E_5000, 600, Engine::Step, Engine::Interpreter);
}

#[test]
fn blocks_run_as_interpreted_batches() {
    compare_engines(0xB1A7_C400, 600, Engine::Interpreter, Engine::Blocks);
}

#[test]
fn compiled_blocks_run_as_single_steps() {
    compare_engines(0x717_5000, 1500, Engine::Step, Engine::Jit);
}

#[test]
fn compiled_blocks_run_as_decoded_blocks() {
    compare_engines(0x717_B10C, 1500, Engine::Blocks, Engine::JitWarm);
}

#[test]
fn programs_do_what_they_are_meant_to() {
    // The generator's programs must exercise what they are for: across a few hundred programs, faults are taken and
    // resumed from, interrupts are taken, code in SRAM is written while it runs, the test registers are used
    let (mut faults, mut irqs, mut steps, mut locked) = (0, 0, 0, 0);
    let (mut code_writes, mut raised, mut scheduled, mut xip, mut stalls) = (0, 0, 0, 0, 0);
    for seed in 0..200u64 {
        let p = programs::generate(seed);
        let mut s = Sim::new(&p, Engine::Blocks);
        s.cpu.log_exceptions = true;
        while !s.done && s.steps < p.steps && s.quanta < 4 * p.steps {
            s.quantum();
            for a in std::mem::take(&mut s.bus.log) {
                let reg = a.addr.wrapping_sub(bus::REGS_BASE);
                code_writes += (a.write && (p.code.0..p.code.1).contains(&a.addr) && p.code.0 >= bus::SRAM_BASE) as u32;
                raised += (a.write && reg == bus::reg::IRQ_SET) as u32;
                scheduled += (a.write && reg == bus::reg::SCHED) as u32;
                xip += (a.write && reg == bus::reg::XIP) as u32;
                stalls += (!a.write && reg == bus::reg::STALL) as u32;
            }
            for (entry, exc, _) in s.cpu.take_exception_log() {
                if entry && exc == crate::exception::HARD_FAULT {
                    faults += 1;
                }
                if entry && exc >= 16 {
                    irqs += 1;
                }
            }
        }
        steps += s.steps;
        locked += s.cpu.lockup as u32;
    }
    eprintln!(
        "{steps} steps, {faults} faults, {irqs} interrupts, {locked} lockups, {code_writes} stores to running code, \
         {raised} lines raised, {scheduled} scheduled, {xip} XIP switches, {stalls} stalls"
    );
    assert!(code_writes > 1000 && raised > 100 && scheduled > 200 && xip > 100 && stalls > 20);
    assert!(steps > 200 * 2000, "{steps} steps");
    assert!(faults > 2000, "{faults} faults");
    assert!(irqs > 500, "{irqs} interrupts");
    assert!(locked < 40, "{locked} of 200 programs locked up");
}

/// Replays one program for investigating a difference: `SIM_DIFF_SEED=<hex> SIM_DIFF_QUANTA=<n> cargo test --release
/// -p cpu_armv6m replay_program -- --ignored --nocapture` prints every step and batch of its first quanta on the
/// reference and on decoded blocks, then compares it on every engine.
#[test]
#[ignore]
fn replay_program() {
    let seed = std::env::var("SIM_DIFF_SEED").expect("SIM_DIFF_SEED");
    let seed = u64::from_str_radix(seed.trim_start_matches("0x"), 16).unwrap();
    let quanta = std::env::var("SIM_DIFF_QUANTA").ok().and_then(|q| q.parse().ok()).unwrap_or(10);
    let p = programs::generate(seed);
    for engine in [Engine::Step, Engine::Blocks] {
        eprintln!("---- {engine:?}");
        let mut s = Sim::new(&p, engine);
        s.trace = true;
        for _ in 0..quanta {
            s.quantum();
        }
    }
    compare_program(seed, Engine::Step, Engine::Interpreter);
    compare_program(seed, Engine::Step, Engine::Blocks);
}
