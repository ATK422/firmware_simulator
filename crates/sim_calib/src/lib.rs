//! Calibration: fitting the emulator's models to data recorded on the robot, and replaying a recorded pool test
//! into the emulated boards.
//!
//! * [`fit`]: deterministic least squares (Levenberg–Marquardt, Nelder–Mead) on named, bounded parameters
//! * [`thrusters`]: thrust curve and DShot ESC motor model from test-stand CSV
//! * [`vehicle`]: drag, added mass and buoyancy from a pool-test log (thruster speeds against IMU, DVL and depth)
//! * [`acoustics`]: the receive chain and the environment from hydrophone recordings (WAV)
//! * [`pool`]: replaying the robot computer's side of a candump log into the emulated boards and comparing what
//!   they send with what the real boards sent
//! * [`table`]: CSV in, Markdown tables and SVG plots out
//!
//! Every fit is tested on data the emulator's own models generate with known parameters: perturb, fit, recover.
//! See `docs/calibration.md`.

pub mod acoustics;
pub mod fit;
pub mod pool;
pub mod table;
pub mod thrusters;
pub mod vehicle;
