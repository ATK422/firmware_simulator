//! Environments: a body of water, its boundaries and noise, and the sources in it.
//!
//! ```toml
//! # environments/pool.toml
//! kind = "tank"
//! size_m = [25.0, 12.0, 3.0]
//! wall_reflection = 0.6
//! bottom_reflection = 0.5
//! water = { temperature_c = 26.0, salinity_ppt = 0.0 }
//! noise = { kind = "white", level_db_re_upa_per_hz = 50.0 }
//! multipath = { max_order = 3 }
//!
//! [[source]]
//! kind = "pinger"
//! position_m = [20.0, 6.0, 2.0]
//! frequency_hz = 25_000
//! pulse_ms = 4
//! period_ms = 2000
//! source_level_db = 180
//! ```

use std::path::Path as FsPath;

use serde::Deserialize;

use crate::geometry::{self, Boundaries, Image, Reflections, Vec3};
use crate::noise::{Noise, Shrimp};
use crate::source::{Source, SourceSpec};
use crate::water::Water;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EnvSpec {
    /// `open`, `tank` or `lake`.
    pub kind: String,
    /// Tank size.
    pub size_m: Option<Vec3>,
    /// Lake depth.
    pub depth_m: Option<f64>,
    /// Lake bottom type, sets the bottom reflection: `mud`, `sand`, `gravel` or `rock`.
    pub bottom: Option<String>,
    pub wall_reflection: Option<f64>,
    pub bottom_reflection: Option<f64>,
    pub surface_reflection: Option<f64>,
    #[serde(default)]
    pub water: Water,
    #[serde(default)]
    pub noise: Noise,
    pub shrimp: Option<Shrimp>,
    #[serde(default)]
    pub multipath: Multipath,
    /// Seed for the noise; runs with the same seed are identical.
    #[serde(default = "one")]
    pub seed: u64,
    #[serde(default, rename = "source")]
    pub sources: Vec<SourceSpec>,
}

fn one() -> u64 {
    1
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Multipath {
    #[serde(default = "two")]
    pub max_order: u32,
}

fn two() -> u32 {
    2
}

impl Default for Multipath {
    fn default() -> Self {
        Multipath { max_order: 2 }
    }
}

/// One propagation path from a source to a receiver.
#[derive(Clone, Copy, Debug)]
pub struct Path {
    pub delay_s: f64,
    /// Pressure amplitude factor relative to the source level at 1 m (spreading, absorption, reflections).
    pub gain: f64,
    /// Unit direction the sound arrives from at the receiver (pointing from the receiver towards the image).
    pub arrival: Vec3,
    pub reflections: u32,
}

/// An environment ready to use.
#[derive(Clone, Debug)]
pub struct Environment {
    pub boundaries: Boundaries,
    pub reflections: Reflections,
    pub water: Water,
    pub noise: Noise,
    pub shrimp: Option<Shrimp>,
    pub max_order: u32,
    pub seed: u64,
    pub sources: Vec<Source>,
    /// Sound speed used for every path (at the mid depth of the water).
    pub sound_speed: f64,
}

fn bottom_coefficient(kind: &str) -> Result<f64, String> {
    // Rough normal-incidence pressure reflection coefficients of sediments
    Ok(match kind {
        "mud" => 0.2,
        "sand" => 0.45,
        "gravel" => 0.6,
        "rock" => 0.8,
        other => return Err(format!("unknown bottom type {other:?}")),
    })
}

impl Environment {
    pub fn from_spec(spec: &EnvSpec, dir: &FsPath) -> Result<Environment, String> {
        let boundaries = match spec.kind.as_str() {
            "open" => Boundaries::Open,
            "tank" | "pool" => Boundaries::Tank { size: spec.size_m.ok_or("a tank needs size_m")? },
            "lake" | "harbour" | "harbor" => Boundaries::Lake { depth: spec.depth_m.ok_or("a lake needs depth_m")? },
            other => return Err(format!("unknown environment kind {other:?}")),
        };
        let bottom = match (&spec.bottom, spec.bottom_reflection) {
            (_, Some(r)) => r,
            (Some(kind), None) => bottom_coefficient(kind)?,
            (None, None) => 0.5,
        };
        let reflections = Reflections {
            surface: spec.surface_reflection.unwrap_or(0.95),
            bottom,
            walls: spec.wall_reflection.unwrap_or(0.5),
        };
        let mid_depth = match boundaries {
            Boundaries::Open => 2.0,
            Boundaries::Tank { size } => size[2] / 2.0,
            Boundaries::Lake { depth } => depth / 2.0,
        };
        let sources = spec.sources.iter().map(|s| Source::from_spec(s, dir)).collect::<Result<Vec<_>, _>>()?;
        for s in &sources {
            if !boundaries.contains(s.position) {
                return Err(format!("source at {:?} is outside the water", s.position));
            }
        }
        Ok(Environment {
            sound_speed: spec.water.sound_speed(mid_depth),
            boundaries,
            reflections,
            water: spec.water.clone(),
            noise: spec.noise.clone(),
            shrimp: spec.shrimp.clone(),
            max_order: spec.multipath.max_order,
            seed: spec.seed,
            sources,
        })
    }

    pub fn parse(text: &str, dir: &FsPath) -> Result<Environment, String> {
        let spec: EnvSpec = toml::from_str(text).map_err(|e| e.to_string())?;
        Self::from_spec(&spec, dir)
    }

    pub fn load(path: &FsPath) -> Result<Environment, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text, path.parent().unwrap_or(FsPath::new("."))).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Open water with nothing in it.
    pub fn open_water(water: Water) -> Environment {
        Environment {
            boundaries: Boundaries::Open,
            reflections: Reflections { surface: 0.0, bottom: 0.0, walls: 0.0 },
            sound_speed: water.sound_speed(2.0),
            water,
            noise: Noise::None,
            shrimp: None,
            max_order: 0,
            seed: 1,
            sources: Vec::new(),
        }
    }

    /// Every path from `src` to `rx` with the environment's multipath order, for a narrowband signal at `hz`.
    pub fn paths(&self, src: Vec3, rx: Vec3, hz: f64) -> Vec<Path> {
        self.boundaries
            .images(src, self.reflections, self.max_order)
            .into_iter()
            .filter_map(|img: Image| {
                let d = geometry::sub(img.position, rx);
                let r = geometry::norm(d).max(0.01);
                let mid_depth = ((img.position[2] + rx[2]) / 2.0).abs();
                let absorption = 10f64.powf(-self.water.absorption_db_per_km(hz, mid_depth) * r / 1000.0 / 20.0);
                let gain = img.coefficient * absorption / r;
                (gain != 0.0).then(|| Path {
                    delay_s: r / self.sound_speed,
                    gain,
                    arrival: [d[0] / r, d[1] / r, d[2] / r],
                    reflections: img.order,
                })
            })
            .collect()
    }

    /// Transmission loss of the direct path (spherical spreading and absorption), dB.
    pub fn direct_loss_db(&self, src: Vec3, rx: Vec3, hz: f64) -> f64 {
        let r = geometry::norm(geometry::sub(src, rx)).max(0.01);
        20.0 * r.log10() + self.water.absorption_db_per_km(hz, (src[2] + rx[2]) / 2.0) * r / 1000.0
    }

    /// Transmission loss with every path added incoherently (by energy), dB.
    pub fn incoherent_loss_db(&self, src: Vec3, rx: Vec3, hz: f64) -> f64 {
        let e: f64 = self.paths(src, rx, hz).iter().map(|p| p.gain * p.gain).sum();
        -10.0 * e.log10()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POOL: &str = r#"
kind = "tank"
size_m = [25.0, 12.0, 3.0]
wall_reflection = 0.6
bottom_reflection = 0.5
water = { temperature_c = 26.0, salinity_ppt = 0.0 }
noise = { kind = "white", level_db_re_upa_per_hz = 50.0 }
multipath = { max_order = 3 }

[[source]]
kind = "pinger"
position_m = [20.0, 6.0, 2.0]
frequency_hz = 25_000
pulse_ms = 4
period_ms = 2000
source_level_db = 180
"#;

    #[test]
    fn parses_the_plan_example() {
        let env = Environment::parse(POOL, FsPath::new(".")).unwrap();
        assert_eq!(env.sources.len(), 1);
        assert_eq!(env.max_order, 3);
        assert!((env.sound_speed - 1499.0).abs() < 3.0, "{}", env.sound_speed);
        let paths = env.paths(env.sources[0].position, [2.0, 6.0, 1.0], 25_000.0);
        assert_eq!(paths.len(), 63);
        let direct = paths.iter().find(|p| p.reflections == 0).unwrap();
        let r = (18.0f64 * 18.0 + 1.0).sqrt();
        assert!((direct.delay_s - r / env.sound_speed).abs() < 1e-9);
        assert!((direct.gain - 1.0 / r).abs() < 1e-4);
        // Every reflected path is longer and weaker than the direct one
        assert!(paths.iter().filter(|p| p.reflections > 0).all(|p| p.delay_s > direct.delay_s));
        assert!(
            env.incoherent_loss_db([20.0, 6.0, 2.0], [2.0, 6.0, 1.0], 25_000.0)
                < env.direct_loss_db([20.0, 6.0, 2.0], [2.0, 6.0, 1.0], 25_000.0)
        );
    }

    #[test]
    fn spreading_and_errors() {
        let env = Environment::open_water(Water::default());
        let l10 = env.direct_loss_db([0.0; 3], [10.0, 0.0, 0.0], 30_000.0);
        let l20 = env.direct_loss_db([0.0; 3], [20.0, 0.0, 0.0], 30_000.0);
        assert!((l20 - l10 - 6.02).abs() < 0.01);
        assert!(Environment::parse("kind = \"tank\"\n", FsPath::new(".")).is_err());
        assert!(Environment::parse("kind = \"lake\"\ndepth_m = 5\nbottom = \"lava\"\n", FsPath::new(".")).is_err());
        let lake = Environment::parse("kind = \"lake\"\ndepth_m = 5\nbottom = \"rock\"\n", FsPath::new(".")).unwrap();
        assert_eq!(lake.reflections.bottom, 0.8);
        let outside = format!(
            "{POOL}\n[[source]]\nkind = \"tone\"\nposition_m = [1, 1, 9]\nfrequency_hz = 1\nsource_level_db = 1\n"
        );
        assert!(Environment::parse(&outside, FsPath::new(".")).is_err());
    }
}

#[cfg(test)]
mod environment_files {
    use super::*;

    #[test]
    fn shipped_environments_load() {
        let dir = FsPath::new(env!("CARGO_MANIFEST_DIR")).join("../../environments");
        let mut n = 0;
        for e in std::fs::read_dir(&dir).unwrap() {
            let p = e.unwrap().path();
            if p.extension().is_some_and(|x| x == "toml") {
                let env = Environment::load(&p).unwrap();
                assert!(!env.sources.is_empty(), "{}", p.display());
                n += 1;
            }
        }
        assert!(n >= 3);
    }
}
