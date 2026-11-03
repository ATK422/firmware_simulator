//! The CPU interface a machine steps.

use crate::bus::Bus;

/// Result of executing one instruction (or taking one exception).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StepOutcome {
    /// Executed normally, taking this many cycles.
    Ran(u32),
    /// The core is asleep (`WFI`/`WFE`) and did nothing. Wake it with an interrupt or event.
    Sleeping,
    /// Hit a `BKPT` instruction (the PC points at it). Cycles taken: none.
    Breakpoint(u8),
    /// The core locked up (fault inside the HardFault handler) and will not execute again until reset.
    Lockup,
}

/// A CPU core the machine can step, inspect and debug.
pub trait Cpu {
    /// Executes one instruction or exception entry against `bus`.
    fn step(&mut self, bus: &mut dyn Bus) -> StepOutcome;
    /// Resets the core, loading SP and PC from the vector table at `vtor`.
    fn reset(&mut self, bus: &mut dyn Bus, vtor: u32);
    fn pc(&self) -> u32;
    /// Number of general purpose registers exposed to debuggers.
    fn num_regs(&self) -> usize;
    /// Debugger view of register `n` (architecture specific numbering).
    fn read_reg(&self, n: usize) -> u32;
    fn write_reg(&mut self, n: usize, value: u32);
    fn is_sleeping(&self) -> bool;
    /// Total cycles executed since reset.
    fn cycles(&self) -> u64;
}
