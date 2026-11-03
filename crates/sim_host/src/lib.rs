//! Host bridges: RTT and defmt (M1), the GDB remote protocol (M1), SocketCAN and UDP (M3, M7), UART ptys.
//!
//! Bridges see the target through [`TargetMemory`] (debug-probe style memory access) so they work with any machine.

pub mod acoustic;
pub mod board;
pub mod defmt;
pub mod gdb;
pub mod mock_bridge;
pub mod pty;
pub mod robot;
pub mod rtt;
pub mod session;
pub mod socketcan;
pub mod target;
pub mod upload;

pub use session::Session;
pub use target::TargetMemory;
