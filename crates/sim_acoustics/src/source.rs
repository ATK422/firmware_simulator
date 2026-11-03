//! Sound sources placed in the water without a board behind them.

use std::f64::consts::PI;
use std::path::PathBuf;
use std::sync::Arc;

use serde::Deserialize;

use crate::geometry::Vec3;
use crate::wav::Wav;

/// A source as written in an environment file.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum SourceSpec {
    /// A gated tone: `pulse_ms` on every `period_ms`.
    Pinger {
        position_m: Vec3,
        frequency_hz: f64,
        pulse_ms: f64,
        period_ms: f64,
        /// RMS source level, dB re 1 µPa at 1 m.
        source_level_db: f64,
        #[serde(default)]
        start_ms: f64,
    },
    /// A continuous tone.
    Tone { position_m: Vec3, frequency_hz: f64, source_level_db: f64 },
    /// Linear chirps from `f0_hz` to `f1_hz` over `duration_ms`, every `period_ms`.
    Chirp { position_m: Vec3, f0_hz: f64, f1_hz: f64, duration_ms: f64, period_ms: f64, source_level_db: f64 },
    /// A recording, played at `source_level_db` for a full-scale RMS of 1.
    Wav {
        position_m: Vec3,
        file: PathBuf,
        source_level_db: f64,
        #[serde(default)]
        looped: bool,
        /// Frequency used for the narrowband parts of the channel model (absorption, sensitivity).
        nominal_hz: Option<f64>,
    },
}

#[derive(Clone, Debug)]
pub enum Waveform {
    Pinger { hz: f64, pulse: f64, period: f64, start: f64 },
    Tone { hz: f64 },
    Chirp { f0: f64, f1: f64, duration: f64, period: f64 },
    Wav { wav: Arc<Wav>, looped: bool, nominal_hz: f64 },
}

/// A source ready to use.
#[derive(Clone, Debug)]
pub struct Source {
    pub position: Vec3,
    pub waveform: Waveform,
    /// RMS source level in µPa at 1 m.
    pub rms_upa: f64,
}

impl Source {
    /// Builds a source; WAV paths are relative to `dir`.
    pub fn from_spec(spec: &SourceSpec, dir: &std::path::Path) -> Result<Source, String> {
        let lvl = |db: f64| 10f64.powf(db / 20.0);
        Ok(match spec {
            SourceSpec::Pinger { position_m, frequency_hz, pulse_ms, period_ms, source_level_db, start_ms } => Source {
                position: *position_m,
                waveform: Waveform::Pinger {
                    hz: *frequency_hz,
                    pulse: pulse_ms / 1e3,
                    period: period_ms / 1e3,
                    start: start_ms / 1e3,
                },
                rms_upa: lvl(*source_level_db),
            },
            SourceSpec::Tone { position_m, frequency_hz, source_level_db } => Source {
                position: *position_m,
                waveform: Waveform::Tone { hz: *frequency_hz },
                rms_upa: lvl(*source_level_db),
            },
            SourceSpec::Chirp { position_m, f0_hz, f1_hz, duration_ms, period_ms, source_level_db } => Source {
                position: *position_m,
                waveform: Waveform::Chirp {
                    f0: *f0_hz,
                    f1: *f1_hz,
                    duration: duration_ms / 1e3,
                    period: period_ms / 1e3,
                },
                rms_upa: lvl(*source_level_db),
            },
            SourceSpec::Wav { position_m, file, source_level_db, looped, nominal_hz } => {
                let wav = Wav::load(&dir.join(file))?;
                Source {
                    position: *position_m,
                    waveform: Waveform::Wav {
                        wav: Arc::new(wav),
                        looped: *looped,
                        nominal_hz: nominal_hz.unwrap_or(25_000.0),
                    },
                    rms_upa: lvl(*source_level_db),
                }
            }
        })
    }

    /// Pressure at 1 m in µPa at time `t` (seconds) of the source's own clock.
    pub fn pressure(&self, t: f64) -> f64 {
        let peak = self.rms_upa * std::f64::consts::SQRT_2;
        match &self.waveform {
            Waveform::Pinger { hz, pulse, period, start } => {
                let local = t - start;
                if local < 0.0 || local.rem_euclid(*period) >= *pulse { 0.0 } else { peak * (2.0 * PI * hz * t).sin() }
            }
            Waveform::Tone { hz } => peak * (2.0 * PI * hz * t).sin(),
            Waveform::Chirp { f0, f1, duration, period } => {
                let local = t.rem_euclid(*period);
                if t < 0.0 || local >= *duration {
                    0.0
                } else {
                    let k = (f1 - f0) / duration;
                    peak * (2.0 * PI * (f0 * local + 0.5 * k * local * local)).sin()
                }
            }
            Waveform::Wav { wav, looped, .. } => {
                let d = wav.duration();
                let local = if *looped && d > 0.0 { t.rem_euclid(d) } else { t };
                self.rms_upa * wav.at(local)
            }
        }
    }

    /// The frequency the narrowband parts of the model use for this source.
    pub fn nominal_hz(&self) -> f64 {
        match &self.waveform {
            Waveform::Pinger { hz, .. } | Waveform::Tone { hz } => *hz,
            Waveform::Chirp { f0, f1, .. } => (f0 + f1) / 2.0,
            Waveform::Wav { nominal_hz, .. } => *nominal_hz,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn src(spec: &str) -> Source {
        let s: SourceSpec = toml::from_str(spec).unwrap();
        Source::from_spec(&s, std::path::Path::new(".")).unwrap()
    }

    #[test]
    fn pinger_gating_and_level() {
        let p = src(
            "kind = \"pinger\"\nposition_m = [0, 0, 1]\nfrequency_hz = 25_000\npulse_ms = 4\nperiod_ms = 2000\nsource_level_db = 180\n",
        );
        assert_eq!(p.rms_upa, 1e9);
        // Quarter period of 25 kHz is 10 us: the peak
        assert!((p.pressure(10e-6) - 1e9 * 2f64.sqrt()).abs() < 1.0);
        assert_eq!(p.pressure(5e-3), 0.0, "between pulses");
        assert!(p.pressure(2.0 + 10e-6) > 0.0, "next pulse");
        assert_eq!(p.nominal_hz(), 25_000.0);
    }

    #[test]
    fn chirp_sweeps() {
        let c = src(
            "kind = \"chirp\"\nposition_m = [0, 0, 1]\nf0_hz = 20_000\nf1_hz = 40_000\nduration_ms = 10\nperiod_ms = 100\nsource_level_db = 170\n",
        );
        assert_eq!(c.nominal_hz(), 30_000.0);
        assert_eq!(c.pressure(0.05), 0.0);
        assert!(c.pressure(0.001).abs() > 0.0);
    }
}
