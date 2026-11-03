//! Helpers shared by the firmware tests.
#![allow(dead_code)]

use std::path::PathBuf;

use rp2040::{Machine, MachineConfig};
use sim_core::image::Image;
use sim_core::trace::TraceSink;
use sim_host::Session;

pub fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A titan_firmware_rs release ELF, if it has been built (see CLAUDE.md); tests skip when it is missing.
pub fn titan_elf(name: &str) -> Option<PathBuf> {
    let p = repo().join("external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release").join(name);
    if p.exists() {
        Some(p)
    } else {
        eprintln!(
            "skipping: {} not built (cd external/titan_firmware_rs/firmware && cargo build --workspace --release)",
            p.display()
        );
        None
    }
}

/// Boots an image like the CLI does: B1 bootrom mapped, direct boot.
pub fn boot(path: &std::path::Path, trace: Option<Box<dyn TraceSink>>) -> Session {
    let image = Image::load(path).unwrap();
    let mut m = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    if let Some(t) = trace {
        m.set_trace(t);
    }
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    Session::new(m, &image)
}
