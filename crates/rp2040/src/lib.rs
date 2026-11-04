//! RP2040 SoC model.
//!
//! * [`memmap`]: the address map; [`irq`]: interrupt numbers
//! * [`svd`]: register names/offsets/reset values generated from the pico-sdk SVD
//! * [`system`]: memories, bus fabric and peripheral dispatch ([`System`], [`system::CoreBus`])
//! * [`periph`]: the peripheral models
//! * [`unmodelled`]: the logging fallback for registers not modelled yet
//! * [`machine`]: two Cortex-M0+ cores plus the system, and the deterministic run loop ([`Machine`])

pub mod hle;
pub mod irq;
pub mod machine;
pub mod memmap;
pub mod periph;
pub mod svd;
pub mod system;
pub mod unmodelled;

pub use machine::{Machine, MachineConfig, RunCarry, StopReason};
pub use system::System;

/// The B1 bootrom image bundled with the emulator (see `bootrom/README.md`).
pub const BOOTROM_B1: &[u8; 16384] = include_bytes!("../bootrom/b1.bin");
