//! Ambient noise and the deterministic random numbers behind it.

use serde::Deserialize;

/// Spectral level of ambient noise.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Noise {
    /// No ambient noise.
    #[default]
    None,
    /// The same level at every frequency.
    White { level_db_re_upa_per_hz: f64 },
    /// Wenz curves in the Coates (1990) approximation: turbulence, shipping (0 to 1), wind (m/s) and thermal noise.
    Wenz {
        #[serde(default = "half")]
        shipping: f64,
        #[serde(default)]
        wind_mps: f64,
    },
}

fn half() -> f64 {
    0.5
}

impl Noise {
    /// Noise spectral level at `hz`, dB re 1 µPa²/Hz (`None` gives negative infinity).
    pub fn level_db(&self, hz: f64) -> f64 {
        match self {
            Noise::None => f64::NEG_INFINITY,
            Noise::White { level_db_re_upa_per_hz } => *level_db_re_upa_per_hz,
            Noise::Wenz { shipping, wind_mps } => {
                let f = (hz / 1000.0).max(1e-3);
                let lg = |x: f64| x.log10();
                let turbulence = 17.0 - 30.0 * lg(f);
                let ship = 40.0 + 20.0 * (shipping - 0.5) + 26.0 * lg(f) - 60.0 * lg(f + 0.03);
                let wind = 50.0 + 7.5 * wind_mps.max(0.0).sqrt() + 20.0 * lg(f) - 40.0 * lg(f + 0.4);
                let thermal = -15.0 + 20.0 * lg(f);
                10.0 * [turbulence, ship, wind, thermal].iter().map(|l| 10f64.powf(l / 10.0)).sum::<f64>().log10()
            }
        }
    }
}

/// Snapping shrimp: Poisson-distributed broadband clicks.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Shrimp {
    /// Mean clicks per second.
    pub rate_hz: f64,
    /// Peak level of a click at the receiver, dB re 1 µPa.
    pub peak_db_re_upa: f64,
    /// Decay time constant of a click.
    #[serde(default = "click_decay")]
    pub decay_us: f64,
}

fn click_decay() -> f64 {
    20.0
}

/// SplitMix64: a stateless hash of `(seed, n)` to a 64-bit value.
pub fn hash(seed: u64, n: u64) -> u64 {
    let mut z = seed ^ n.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Uniform in (0, 1].
pub fn uniform(seed: u64, n: u64) -> f64 {
    ((hash(seed, n) >> 11) as f64 + 1.0) / (1u64 << 53) as f64
}

/// Standard normal (Box-Muller), a deterministic function of `(seed, n)`.
pub fn gaussian(seed: u64, n: u64) -> f64 {
    let u1 = uniform(seed, 2 * n);
    let u2 = uniform(seed, 2 * n + 1);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wenz_levels_are_plausible() {
        // Moderate shipping and a light wind: about 60-70 dB at 1 kHz, falling to 30-45 dB at 30 kHz
        let n = Noise::Wenz { shipping: 0.5, wind_mps: 5.0 };
        let l1 = n.level_db(1_000.0);
        let l30 = n.level_db(30_000.0);
        assert!((55.0..72.0).contains(&l1), "{l1}");
        assert!((28.0..48.0).contains(&l30), "{l30}");
        assert!(n.level_db(30_000.0) < Noise::Wenz { shipping: 0.5, wind_mps: 15.0 }.level_db(30_000.0));
        assert_eq!(Noise::White { level_db_re_upa_per_hz: 50.0 }.level_db(123.0), 50.0);
    }

    #[test]
    fn random_numbers_are_deterministic_and_normal() {
        assert_eq!(gaussian(7, 100), gaussian(7, 100));
        assert_ne!(gaussian(7, 100), gaussian(8, 100));
        let xs: Vec<f64> = (0..20_000).map(|n| gaussian(1, n)).collect();
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.03 && (var - 1.0).abs() < 0.05, "{mean} {var}");
    }
}
