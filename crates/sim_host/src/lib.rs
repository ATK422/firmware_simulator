//! Host bridges: RTT and defmt (M1), the GDB remote protocol (M1), SocketCAN and UDP (M3, M7), UART ptys.
//!
//! Bridges see the target through [`TargetMemory`] (debug-probe style memory access) so they work with any machine.

pub mod acoustic;

/// Where the titan_firmware_rs release images are: `$TITAN_RS_RELEASE_DIR` if set (a worktree, another checkout of
/// titan_firmware_rs, CI), else `repo/external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release`.
pub fn titan_rs_release_dir(repo: &std::path::Path) -> std::path::PathBuf {
    match std::env::var_os("TITAN_RS_RELEASE_DIR") {
        Some(d) => d.into(),
        None => repo.join(RS_RELEASE_SUFFIX),
    }
}

const RS_RELEASE_SUFFIX: &str = "external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release";

/// An image path from a robot file, with the titan_firmware_rs release directory replaced by
/// `$TITAN_RS_RELEASE_DIR` when that is set.
pub fn resolve_image(path: &std::path::Path) -> std::path::PathBuf {
    let s = path.to_string_lossy();
    match (std::env::var("TITAN_RS_RELEASE_DIR"), s.find(RS_RELEASE_SUFFIX)) {
        (Ok(dir), Some(i)) => std::path::Path::new(&dir).join(s[i + RS_RELEASE_SUFFIX.len()..].trim_start_matches('/')),
        _ => path.to_path_buf(),
    }
}
pub mod board;
pub mod defmt;
pub mod ethernet;
pub mod gdb;
pub mod mock_bridge;
pub mod pty;
pub mod robot;
pub mod rtt;
pub mod session;
pub mod socketcan;
pub mod target;
pub mod upload;
pub mod vehicle;

pub use session::Session;
pub use target::TargetMemory;
