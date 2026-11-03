//! Transducers described by their datasheet numbers.

use serde::Deserialize;

use crate::geometry::Vec3;

/// A frequency response in dB: constant, a table (linear interpolation in dB over log frequency), or a resonance.
#[derive(Clone, Debug, Deserialize)]
#[serde(untagged)]
pub enum Response {
    Const(f64),
    Table(Vec<(f64, f64)>),
    /// Second-order resonance: `peak_db` at `resonance_hz`, falling off with quality factor `q`.
    Resonance {
        resonance_hz: f64,
        q: f64,
        peak_db: f64,
    },
}

impl Response {
    pub fn at(&self, hz: f64) -> f64 {
        match self {
            Response::Const(db) => *db,
            Response::Table(points) => {
                let Some(first) = points.first() else { return f64::NEG_INFINITY };
                if hz <= first.0 {
                    return first.1;
                }
                for w in points.windows(2) {
                    let ((f0, d0), (f1, d1)) = (w[0], w[1]);
                    if hz <= f1 {
                        let x = (hz.ln() - f0.ln()) / (f1.ln() - f0.ln());
                        return d0 + (d1 - d0) * x;
                    }
                }
                points.last().unwrap().1
            }
            Response::Resonance { resonance_hz, q, peak_db } => {
                // |H| of a band-pass resonator, 1 at the resonance
                let r = hz / resonance_hz;
                let h = 1.0 / (1.0 + q * q * (r - 1.0 / r).powi(2)).sqrt();
                peak_db + 20.0 * h.max(1e-12).log10()
            }
        }
    }
}

/// Beam pattern.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(untagged)]
pub enum Beam {
    #[default]
    #[serde(skip)]
    Omni,
    Named(String),
    /// A circular piston of this radius in an infinite baffle.
    Piston {
        piston_radius_m: f64,
    },
    /// Relative level in dB by angle off axis in degrees (symmetric about the axis).
    Table {
        pattern_db: Vec<(f64, f64)>,
    },
}

/// First-order Bessel function J1 (series; plenty for the arguments a piston pattern needs).
fn bessel_j1(x: f64) -> f64 {
    if x.abs() > 25.0 {
        // Asymptotic form
        let ax = x.abs();
        let v = (2.0 / (std::f64::consts::PI * ax)).sqrt() * (ax - 3.0 * std::f64::consts::FRAC_PI_4).cos();
        return if x < 0.0 { -v } else { v };
    }
    let mut term = x / 2.0;
    let mut sum = term;
    for k in 1..60 {
        term *= -(x * x / 4.0) / (k as f64 * (k + 1) as f64);
        sum += term;
        if term.abs() < 1e-16 {
            break;
        }
    }
    sum
}

impl Beam {
    /// Relative amplitude (0..1) at `angle_rad` off axis for a wave of `wavelength_m`.
    pub fn gain(&self, angle_rad: f64, wavelength_m: f64) -> f64 {
        match self {
            Beam::Omni => 1.0,
            Beam::Named(n) if n == "omni" => 1.0,
            Beam::Named(n) => {
                log::warn!("unknown beam pattern {n:?}, using omni");
                1.0
            }
            Beam::Piston { piston_radius_m } => {
                let ka = 2.0 * std::f64::consts::PI / wavelength_m * piston_radius_m;
                let x = ka * angle_rad.sin();
                if x.abs() < 1e-9 { 1.0 } else { (2.0 * bessel_j1(x) / x).abs() }
            }
            Beam::Table { pattern_db } => {
                let deg = angle_rad.to_degrees().abs();
                let db = Response::Table(pattern_db.iter().map(|&(a, d)| (a.max(1e-3), d)).collect()).at(deg.max(1e-3));
                10f64.powf(db / 20.0)
            }
        }
    }
}

/// A transducer: a projector, a hydrophone or both.
#[derive(Clone, Debug, Deserialize)]
pub struct Transducer {
    /// Transmitting voltage response, dB re 1 µPa/V at 1 m.
    pub tvr: Option<Response>,
    /// Receive sensitivity, dB re 1 V/µPa.
    #[serde(alias = "rvs_db")]
    pub rvs: Option<Response>,
    #[serde(default)]
    pub beam: Beam,
    /// Direction of the acoustic axis (any length); straight down by default.
    #[serde(default = "down")]
    pub axis: Vec3,
    /// Largest drive voltage (peak) before the projector is overdriven.
    pub max_drive_v: Option<f64>,
}

fn down() -> Vec3 {
    [0.0, 0.0, 1.0]
}

impl Transducer {
    /// An omnidirectional hydrophone with a flat sensitivity.
    pub fn hydrophone(rvs_db: f64) -> Self {
        Transducer { tvr: None, rvs: Some(Response::Const(rvs_db)), beam: Beam::Omni, axis: down(), max_drive_v: None }
    }

    /// Volts per µPa at `hz`.
    pub fn sensitivity(&self, hz: f64) -> f64 {
        self.rvs.as_ref().map_or(0.0, |r| 10f64.powf(r.at(hz) / 20.0))
    }

    /// Amplitude gain of the beam for a wave arriving from (or leaving towards) `direction`.
    pub fn beam_gain(&self, direction: Vec3, wavelength_m: f64) -> f64 {
        let angle = crate::geometry::angle_between(self.axis, direction);
        self.beam.gain(angle, wavelength_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn responses() {
        let r = Response::Resonance { resonance_hz: 51_000.0, q: 8.0, peak_db: 140.0 };
        assert!((r.at(51_000.0) - 140.0).abs() < 1e-9);
        assert!(r.at(30_000.0) < 125.0);
        let t = Response::Table(vec![(10_000.0, -200.0), (100_000.0, -190.0)]);
        assert!((t.at(31_622.8) - (-195.0)).abs() < 0.01, "{}", t.at(31_622.8));
        assert_eq!(t.at(1.0), -200.0);
        assert_eq!(t.at(1e6), -190.0);
    }

    #[test]
    fn piston_beam() {
        // ka = 3.83 puts the first null at 90 degrees
        let b = Beam::Piston { piston_radius_m: 0.03 };
        let wavelength = 2.0 * std::f64::consts::PI * 0.03 / 3.8317;
        assert!((b.gain(0.0, wavelength) - 1.0).abs() < 1e-9);
        assert!(b.gain(std::f64::consts::FRAC_PI_2, wavelength) < 1e-3);
        assert!((bessel_j1(1.0) - 0.440_050_585_7).abs() < 1e-9);
        assert_eq!(Beam::Named("omni".into()).gain(1.0, 0.05), 1.0);
    }

    #[test]
    fn parses_plan_example() {
        let t: Transducer =
            toml::from_str("tvr = { resonance_hz = 51_000, q = 8, peak_db = 140 }\nrvs_db = -190\n").unwrap();
        assert!((t.sensitivity(30_000.0) - 10f64.powf(-9.5)).abs() < 1e-15);
        assert!(matches!(t.beam, Beam::Omni));
        let t: Transducer = toml::from_str("rvs = -180\nbeam = \"omni\"\n").unwrap();
        assert!(matches!(t.beam, Beam::Named(_)));
    }
}
