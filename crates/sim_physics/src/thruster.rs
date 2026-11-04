//! Thrusters: where they are on the vehicle, which way they push, and how hard for a given propeller speed.
//!
//! A propeller's thrust grows with the square of its speed (`T = k rpm |rpm|`, the thrust coefficient of
//! momentum theory at a fixed advance ratio), and is weaker in reverse, where the propeller and duct work against
//! their design. The coefficients come from one point of a thrust curve in each direction, as datasheets give
//! them: e.g. the Blue Robotics T200 at 16 V makes 51.4 N forward at its top speed of 3533 RPM and 39.9 N in reverse
//! at 3465 RPM (its published performance data).
//!
//! Thrusters with a counter-rotating propeller (fitted so that the torques of neighbouring thrusters cancel) are
//! `inverted`: their motor turns backwards for the same push, and forwards gives their weaker reverse thrust. The
//! Talos ESC boards' RPM controller knows which ones those are (its invert mask, thrusters 1 and 4 of each board by
//! default) and turns them the other way; raw throttle commands don't.

use serde::Deserialize;

use crate::math::{Vec3, unit};

/// Thrust at one speed in each direction.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ThrustCurve {
    /// Thrust at `at_rpm` forwards, N.
    pub forward_n: f64,
    /// Thrust at `at_rpm` in reverse, N (a positive number).
    pub reverse_n: f64,
    pub at_rpm: f64,
}

/// A thruster as written in a vehicle file.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ThrusterSpec {
    pub name: String,
    /// Where the propeller is, body frame, from the centre of gravity.
    pub position_m: Vec3,
    /// The direction it pushes the vehicle at positive RPM (normalized when loaded).
    pub direction: Vec3,
    /// Its thrust curve, when it differs from the vehicle's `thruster_model`.
    pub curve: Option<ThrustCurve>,
    /// A counter-rotating propeller: negative motor RPM pushes along `direction`.
    #[serde(default)]
    pub inverted: bool,
}

/// A thruster ready to use: unit direction and thrust coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct Thruster {
    pub name: String,
    pub position: Vec3,
    pub direction: Vec3,
    /// N/RPM² forwards and in reverse.
    pub k_forward: f64,
    pub k_reverse: f64,
    pub inverted: bool,
}

impl Thruster {
    /// From its spec, with `default` as the curve when it has none of its own.
    pub fn new(spec: &ThrusterSpec, default: Option<&ThrustCurve>) -> Thruster {
        let c = spec.curve.as_ref().or(default).expect("a thrust curve (checked when the file is loaded)");
        Thruster {
            name: spec.name.clone(),
            position: spec.position_m,
            direction: unit(spec.direction),
            k_forward: c.forward_n / (c.at_rpm * c.at_rpm),
            k_reverse: c.reverse_n / (c.at_rpm * c.at_rpm),
            inverted: spec.inverted,
        }
    }

    /// Thrust along `direction` at a signed mechanical motor RPM, N.
    pub fn thrust(&self, rpm: f64) -> f64 {
        let rpm = if self.inverted { -rpm } else { rpm };
        if rpm >= 0.0 { self.k_forward * rpm * rpm } else { -self.k_reverse * rpm * rpm }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thrust_is_quadratic_and_weaker_in_reverse() {
        let spec = ThrusterSpec {
            name: "t".into(),
            position_m: [0.0; 3],
            direction: [0.0, 0.0, 2.0],
            curve: Some(ThrustCurve { forward_n: 51.5, reverse_n: 40.2, at_rpm: 3800.0 }),
            inverted: false,
        };
        let t = Thruster::new(&spec, None);
        assert_eq!(t.direction, [0.0, 0.0, 1.0]);
        assert!((t.thrust(3800.0) - 51.5).abs() < 1e-9);
        assert!((t.thrust(1900.0) - 51.5 / 4.0).abs() < 1e-9);
        assert!((t.thrust(-3800.0) + 40.2).abs() < 1e-9);
        assert_eq!(t.thrust(0.0), 0.0);
        // The vehicle's model is used when the thruster has none
        let model = ThrustCurve { forward_n: 10.0, reverse_n: 5.0, at_rpm: 1000.0 };
        let t = Thruster::new(&ThrusterSpec { curve: None, ..spec.clone() }, Some(&model));
        assert!((t.thrust(-1000.0) + 5.0).abs() < 1e-9);
        // A counter-rotating propeller pushes forwards with the motor turning backwards
        let t = Thruster::new(&ThrusterSpec { inverted: true, ..spec }, None);
        assert!((t.thrust(-3800.0) - 51.5).abs() < 1e-9);
        assert!((t.thrust(3800.0) + 40.2).abs() < 1e-9);
    }
}
