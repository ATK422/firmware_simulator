//! Vehicle physics: thrusters push a rigid body through the water, and the sensors on it see where it went.
//!
//! * [`math`]: vectors, quaternions and the frames (world north-east-down, body forward-starboard-down)
//! * [`vehicle`]: the 6-DOF rigid body with added mass, drag, gravity, buoyancy and the surface; vehicle files
//! * [`thruster`]: thruster geometry and thrust from propeller speed
//! * [`log`]: a record of the vehicle's state over time, as CSV
//!
//! Nothing here knows about boards or firmware: the caller reads the propeller speeds (e.g. from emulated ESCs),
//! steps the vehicle on its virtual clock and puts the results where the sensors read them.

pub mod log;
pub mod math;
pub mod thruster;
pub mod vehicle;

pub use math::{Quat, Vec3};
pub use thruster::{ThrustCurve, Thruster, ThrusterSpec};
pub use vehicle::{Imu, Params, State, Vehicle, VehicleFile, Water, Wrench};
