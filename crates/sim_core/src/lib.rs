//! Core abstractions of the simulator, shared by every machine and free of any SoC knowledge.
//!
//! * [`time`]: virtual time ([`Time`], picoseconds) and clock periods
//! * [`sched`]: the keyed event [`Scheduler`]
//! * [`bus`]: the [`Bus`] a CPU executes against and the [`Peripheral`] trait for memory mapped blocks
//! * [`cpu`]: the [`Cpu`] trait a machine steps
//! * [`wire`]: IRQ lines and digital pin levels
//! * [`trace`]: structured trace events and sinks
//! * [`image`]: ELF and UF2 firmware images
//! * [`snapshot`]: whole-machine snapshot support

pub mod bus;
pub mod cpu;
pub mod image;
pub mod sched;
pub mod snapshot;
pub mod time;
pub mod trace;
pub mod wire;

pub use bus::{Bus, BusFault, Peripheral, Size, WriteMode};
pub use cpu::{Cpu, StepOutcome};
pub use image::Image;
pub use sched::Scheduler;
pub use time::{ClockPeriod, Time};
pub use trace::{TraceEvent, TraceSink};
