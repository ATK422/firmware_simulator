//! The water itself: sound speed and absorption.

use serde::Deserialize;

/// Properties of the water. Depth-dependent quantities take the depth of the path's midpoint.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Water {
    #[serde(default = "default_temperature")]
    pub temperature_c: f64,
    /// Practical salinity, ‰ (0 for fresh water, about 35 for the sea).
    #[serde(default)]
    pub salinity_ppt: f64,
    #[serde(default = "default_ph")]
    pub ph: f64,
    /// Overrides the computed sound speed, m/s.
    pub sound_speed_mps: Option<f64>,
}

fn default_temperature() -> f64 {
    20.0
}

fn default_ph() -> f64 {
    8.0
}

impl Default for Water {
    fn default() -> Self {
        Water { temperature_c: 20.0, salinity_ppt: 0.0, ph: 8.0, sound_speed_mps: None }
    }
}

impl Water {
    /// Sound speed in m/s at `depth_m`: Mackenzie (1981) for sea water (S >= 25 ‰), Marczak (1997) for fresh water
    /// below 5 ‰ (with a linear blend in between).
    pub fn sound_speed(&self, depth_m: f64) -> f64 {
        if let Some(c) = self.sound_speed_mps {
            return c;
        }
        let t = self.temperature_c;
        let s = self.salinity_ppt;
        let d = depth_m.max(0.0);
        let mackenzie = 1448.96 + 4.591 * t - 5.304e-2 * t * t
            + 2.374e-4 * t.powi(3)
            + 1.340 * (s - 35.0)
            + 1.630e-2 * d
            + 1.675e-7 * d * d
            - 1.025e-2 * t * (s - 35.0)
            - 7.139e-13 * t * d.powi(3);
        // Pure water, plus the pressure term from Mackenzie
        let marczak = 1.402385e3 + 5.038813 * t - 5.799136e-2 * t * t + 3.287156e-4 * t.powi(3)
            - 1.398845e-6 * t.powi(4)
            + 2.787860e-9 * t.powi(5)
            + 1.630e-2 * d;
        if s >= 25.0 {
            mackenzie
        } else if s <= 5.0 {
            marczak
        } else {
            let f = (s - 5.0) / 20.0;
            marczak * (1.0 - f) + mackenzie * f
        }
    }

    /// Absorption in dB/km at `hz` and `depth_m` (Francois and Garrison, 1982: boric acid, magnesium sulphate and
    /// pure water viscosity terms).
    pub fn absorption_db_per_km(&self, hz: f64, depth_m: f64) -> f64 {
        let f = hz / 1000.0;
        let t = self.temperature_c;
        let s = self.salinity_ppt.max(0.0);
        let d = depth_m.max(0.0);
        let c = self.sound_speed(d);
        let theta = t + 273.0;
        // Boric acid
        let a1 = 8.86 / c * 10f64.powf(0.78 * self.ph - 5.0);
        let f1 = 2.8 * (s / 35.0).sqrt() * 10f64.powf(4.0 - 1245.0 / theta);
        let boric = if f1 > 0.0 { a1 * f1 * f * f / (f1 * f1 + f * f) } else { 0.0 };
        // Magnesium sulphate
        let a2 = 21.44 * s / c * (1.0 + 0.025 * t);
        let p2 = 1.0 - 1.37e-4 * d + 6.2e-9 * d * d;
        let f2 = 8.17 * 10f64.powf(8.0 - 1990.0 / theta) / (1.0 + 0.0018 * (s - 35.0));
        let mgso4 = a2 * p2 * f2 * f * f / (f2 * f2 + f * f);
        // Pure water
        let p3 = 1.0 - 3.83e-5 * d + 4.9e-10 * d * d;
        let a3 = if t <= 20.0 {
            4.937e-4 - 2.59e-5 * t + 9.11e-7 * t * t - 1.50e-8 * t.powi(3)
        } else {
            3.964e-4 - 1.146e-5 * t + 1.45e-7 * t * t - 6.5e-10 * t.powi(3)
        };
        boric + mgso4 + a3 * p3 * f * f
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sound_speed_reference_values() {
        // Fresh water at 20 °C: 1482.3 m/s; sea water (35 ‰, 10 °C, surface): about 1490 m/s
        let fresh = Water { temperature_c: 20.0, ..Default::default() };
        assert!((fresh.sound_speed(0.0) - 1482.3).abs() < 0.5, "{}", fresh.sound_speed(0.0));
        let sea = Water { temperature_c: 10.0, salinity_ppt: 35.0, ..Default::default() };
        assert!((sea.sound_speed(0.0) - 1489.8).abs() < 1.0, "{}", sea.sound_speed(0.0));
        assert!(sea.sound_speed(1000.0) > sea.sound_speed(0.0));
        assert_eq!(Water { sound_speed_mps: Some(1500.0), ..Default::default() }.sound_speed(10.0), 1500.0);
    }

    #[test]
    fn absorption_reference_values() {
        // Sea water at 10 °C: roughly 1 dB/km at 10 kHz, 8-10 dB/km at 30 kHz, 30-40 dB/km at 100 kHz
        let sea = Water { temperature_c: 10.0, salinity_ppt: 35.0, ..Default::default() };
        let a10 = sea.absorption_db_per_km(10_000.0, 0.0);
        let a30 = sea.absorption_db_per_km(30_000.0, 0.0);
        let a100 = sea.absorption_db_per_km(100_000.0, 0.0);
        assert!((0.6..1.5).contains(&a10), "{a10}");
        assert!((6.0..12.0).contains(&a30), "{a30}");
        assert!((25.0..45.0).contains(&a100), "{a100}");
        // Fresh water has only the viscous term: far less
        let fresh = Water { temperature_c: 10.0, ..Default::default() };
        assert!(fresh.absorption_db_per_km(30_000.0, 0.0) < 0.5);
    }
}
