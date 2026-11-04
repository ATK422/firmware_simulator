//! Acoustic calibration: the receive chain of a board's transducer (gain, sensitivity, filter corners, ADC offset
//! and clipping) and the environment (noise level, wall, bottom and surface reflection, a source's level), fitted
//! by simulating each recording's geometry with `sim_acoustics` and comparing the simulated capture with the
//! recorded one by features: DC level, RMS, peak, clipped fraction, noise floor, a third-octave spectrum and the
//! decay after a ping.
//!
//! A manifest (TOML, `docs/calibration.md`) names the board file whose `[transducer]` is the starting point, the
//! environment file, the parameters to fit, and the recordings: WAV files of the board's ADC capture (`kind =
//! "board"`: -1..1 is the ADC's input range) or of a calibrated reference hydrophone (`kind = "reference"`, with
//! its sensitivity: it measures the pressure, so it calibrates the environment and source apart from the board),
//! each with where the receiver was, the source (or the environment's), the filter select level and any gain
//! offset in effect.
//!
//! The simulation is the one a board in the environment hears ([`sim_acoustics::Hydrophone`] sampled at the
//! recording's rate, quantised by the ADC), so the fitted values go straight back into a board overlay and an
//! environment file.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use serde::Deserialize;
use sim_acoustics::chain::ChainSpec;
use sim_acoustics::env::EnvSpec;
use sim_acoustics::transducer::Transducer;
use sim_acoustics::wav::Wav;
use sim_acoustics::{Environment, Hydrophone};
use sim_core::time::Time;

use crate::fit::{self, Fit, Options, Param};
use crate::table::{Series, markdown, svg_plot};

/// The parameters an acoustic calibration can fit.
pub const PARAMS: &[(&str, &str)] = &[
    ("gain_db", "the receive chain's preamp gain (`[transducer.receive] gain_db`)"),
    ("sensitivity_offset_db", "added to the transducer's receive sensitivity (`rvs`/`rvs_db`, any form)"),
    ("highpass_hz", "high-pass corner of the filter the recordings use"),
    ("lowpass_hz", "low-pass corner of the filter the recordings use"),
    ("bias_v", "DC level at the ADC pin (`bias_v`): the ADC offset"),
    ("rail_low_v", "where the chain clips below (`rails_v[0]`)"),
    ("rail_high_v", "where the chain clips above (`rails_v[1]`)"),
    ("noise_db", "the environment's white noise level, dB re 1 µPa²/Hz"),
    ("wall_reflection", "the environment's wall reflection coefficient"),
    ("bottom_reflection", "the environment's bottom reflection coefficient"),
    ("surface_reflection", "the environment's surface reflection coefficient"),
    (
        "source_level_db",
        "the source's RMS level, dB re 1 µPa at 1 m (the manifest's `[source]`, or the environment's first)",
    ),
];

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// The board's own ADC capture: -1..1 is the ADC's input range (`adc_range_v`).
    #[default]
    Board,
    /// A calibrated reference hydrophone: volts = sample * `volts_full_scale`, pressure = volts / sensitivity.
    Reference,
}

/// One recording in a manifest.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Recording {
    pub file: PathBuf,
    /// Name in the report (the file's stem by default).
    pub name: Option<String>,
    #[serde(default)]
    pub kind: Kind,
    /// Where the hydrophone was, environment coordinates (m).
    pub receiver_m: [f64; 3],
    /// The source for this recording (an environment `[[source]]` table); else the manifest's, else the
    /// environment's own.
    pub source: Option<toml::Value>,
    /// The filter select pin's level during the capture (1 by default: both IVCBoard2 firmwares drive it high).
    pub select: Option<u8>,
    /// Simulated time (s) of the recording's first sample used: when it starts relative to the source's pulses.
    #[serde(default)]
    pub start_s: f64,
    /// Seconds of the WAV file to skip before the part used.
    #[serde(default)]
    pub skip_s: f64,
    /// How much of it to use (all of it by default).
    pub duration_s: Option<f64>,
    /// Gain in effect during this recording relative to the fitted `gain_db` (switchable gain settings).
    #[serde(default)]
    pub gain_offset_db: f64,
    /// Reference hydrophone: its sensitivity, dB re 1 V/µPa, and the volts a full-scale sample stands for.
    pub sensitivity_db: Option<f64>,
    #[serde(default = "one")]
    pub volts_full_scale: f64,
}

fn one() -> f64 {
    1.0
}

/// A calibration manifest.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    /// Board file (its `[transducer]` is the starting point), relative to the manifest.
    pub board: PathBuf,
    /// Environment file, relative to the manifest.
    pub environment: PathBuf,
    /// Parameters to fit (see [`PARAMS`]).
    #[serde(default = "default_fit")]
    pub fit: Vec<String>,
    /// `lm` (default) or `nelder-mead`.
    pub method: Option<String>,
    /// The source every recording hears unless it names its own (an environment `[[source]]` table).
    pub source: Option<toml::Value>,
    /// Starting values that replace the board's and environment's (e.g. a corner the filter does not have yet).
    #[serde(default)]
    pub start: BTreeMap<String, f64>,
    /// ADC resolution (bits) and input range (V) of board recordings.
    #[serde(default = "twelve")]
    pub adc_bits: u32,
    #[serde(default = "adc_range")]
    pub adc_range_v: [f64; 2],
    #[serde(rename = "recording")]
    pub recordings: Vec<Recording>,
}

fn default_fit() -> Vec<String> {
    vec!["gain_db".into(), "noise_db".into()]
}

fn twelve() -> u32 {
    12
}

fn adc_range() -> [f64; 2] {
    [0.0, 3.3]
}

impl Manifest {
    pub fn load(path: &Path) -> Result<Manifest, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
    }
}

/// What a simulation of the recordings needs, loaded once.
#[derive(Clone, Debug)]
pub struct Setup {
    pub manifest: Manifest,
    /// The board file's `[transducer]` table and the environment file, as TOML.
    pub transducer: toml::Table,
    pub environment: toml::Table,
    pub env_dir: PathBuf,
    /// The recordings' samples (the part used) and rates.
    pub wavs: Vec<Wav>,
    pub names: Vec<String>,
}

fn table(v: &toml::Value) -> Option<&toml::Table> {
    v.as_table()
}

impl Setup {
    /// Loads a manifest and what it names; paths are relative to the manifest's directory.
    pub fn load(path: &Path) -> Result<Setup, String> {
        let m = Manifest::load(path)?;
        let dir = path.parent().unwrap_or(Path::new("."));
        let read = |p: &Path| -> Result<toml::Table, String> {
            let text = std::fs::read_to_string(dir.join(p)).map_err(|e| format!("{}: {e}", dir.join(p).display()))?;
            toml::from_str(&text).map_err(|e| format!("{}: {e}", dir.join(p).display()))
        };
        let board = read(&m.board)?;
        let transducer = board
            .get("transducer")
            .and_then(table)
            .cloned()
            .ok_or_else(|| format!("{} has no [transducer]", m.board.display()))?;
        let environment = read(&m.environment)?;
        let env_dir = dir.join(&m.environment).parent().map(Path::to_path_buf).unwrap_or_default();
        let mut wavs = Vec::new();
        let mut names = Vec::new();
        for r in &m.recordings {
            let w = Wav::load(&dir.join(&r.file))?;
            wavs.push(trim(&w, r));
            names.push(r.name.clone().unwrap_or_else(|| {
                r.file.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "recording".into())
            }));
        }
        Setup::new(m, transducer, environment, env_dir, wavs, names)
    }

    /// From parts already in memory (tests, and recordings made by [`Setup::simulate`]).
    pub fn new(
        manifest: Manifest,
        transducer: toml::Table,
        environment: toml::Table,
        env_dir: PathBuf,
        wavs: Vec<Wav>,
        names: Vec<String>,
    ) -> Result<Setup, String> {
        if manifest.recordings.is_empty() {
            return Err("the manifest has no [[recording]]".into());
        }
        for p in &manifest.fit {
            if !PARAMS.iter().any(|(n, _)| n == p) {
                return Err(format!(
                    "no acoustic parameter {p:?} (there are {})",
                    PARAMS.iter().map(|p| p.0).collect::<Vec<_>>().join(", ")
                ));
            }
        }
        for r in &manifest.recordings {
            if r.kind == Kind::Reference && r.sensitivity_db.is_none() {
                return Err(format!("{}: a reference recording needs sensitivity_db", r.file.display()));
            }
        }
        let s = Setup { manifest, transducer, environment, env_dir, wavs, names };
        // Everything the parameters touch must exist and simulate
        let p = s.start_values()?;
        s.edited(&p)?;
        Ok(s)
    }

    /// Which filter the recordings use, and checks they all use the same one when a corner is fitted.
    fn filter_index(&self) -> Result<usize, String> {
        let rx = self.transducer.get("receive").and_then(table).ok_or("the transducer has no [transducer.receive]")?;
        let has_select = rx.contains_key("select_pin");
        let idx: Vec<usize> = self
            .manifest
            .recordings
            .iter()
            .map(|r| if has_select { r.select.unwrap_or(1) as usize } else { 0 })
            .collect();
        if idx.windows(2).any(|w| w[0] != w[1])
            && self.manifest.fit.iter().any(|p| p == "highpass_hz" || p == "lowpass_hz")
        {
            return Err(
                "the recordings use different filters (select): fit corners with one filter's recordings".into()
            );
        }
        Ok(idx[0])
    }

    /// The parameters' starting values: the board's and environment's, or the manifest's `[start]`.
    pub fn start_values(&self) -> Result<BTreeMap<String, f64>, String> {
        let mut out = BTreeMap::new();
        let rx = self.transducer.get("receive").and_then(table);
        let f = |t: Option<&toml::Table>, k: &str| t.and_then(|t| t.get(k)).and_then(as_f64);
        let rails = rx
            .and_then(|r| r.get("rails_v"))
            .and_then(|v| v.as_array())
            .map(|a| {
                [a[0].as_float().or(a[0].as_integer().map(|i| i as f64)).unwrap_or(0.0), as_f64(&a[1]).unwrap_or(3.3)]
            })
            .unwrap_or([0.0, 3.3]);
        let env = Some(&self.environment);
        let filter = self.filter_table_value(&self.transducer)?;
        for name in &self.manifest.fit {
            let v = match name.as_str() {
                "gain_db" => f(rx, "gain_db").unwrap_or(0.0),
                "sensitivity_offset_db" => 0.0,
                "highpass_hz" => filter.0.unwrap_or(f64::NAN),
                "lowpass_hz" => filter.1.unwrap_or(f64::NAN),
                "bias_v" => f(rx, "bias_v").unwrap_or((rails[0] + rails[1]) / 2.0),
                "rail_low_v" => rails[0],
                "rail_high_v" => rails[1],
                "noise_db" => self
                    .environment
                    .get("noise")
                    .and_then(table)
                    .filter(|n| n.get("kind").and_then(|k| k.as_str()) == Some("white"))
                    .and_then(|n| n.get("level_db_re_upa_per_hz"))
                    .and_then(as_f64)
                    .unwrap_or(50.0),
                "wall_reflection" => f(env, "wall_reflection").unwrap_or(0.5),
                "bottom_reflection" => f(env, "bottom_reflection").unwrap_or(0.5),
                "surface_reflection" => f(env, "surface_reflection").unwrap_or(0.95),
                "source_level_db" => self
                    .manifest
                    .source
                    .as_ref()
                    .and_then(table)
                    .or_else(|| {
                        self.environment
                            .get("source")
                            .and_then(|s| s.as_array())
                            .and_then(|a| a.first())
                            .and_then(table)
                    })
                    .and_then(|s| s.get("source_level_db"))
                    .and_then(as_f64)
                    .ok_or("source_level_db: no [source] in the manifest and none in the environment")?,
                _ => unreachable!("checked in Setup::new"),
            };
            let v = self.manifest.start.get(name).copied().unwrap_or(v);
            if !v.is_finite() {
                return Err(format!("{name}: the filter has no such corner; give a starting value in [start]"));
            }
            out.insert(name.clone(), v);
        }
        for k in self.manifest.start.keys() {
            if !self.manifest.fit.contains(k) {
                return Err(format!("[start] {k}: not a fitted parameter"));
            }
        }
        Ok(out)
    }

    /// The (highpass, lowpass) corners of the filter the recordings use.
    fn filter_table_value(&self, transducer: &toml::Table) -> Result<(Option<f64>, Option<f64>), String> {
        let rx = transducer.get("receive").and_then(table).ok_or("the transducer has no [transducer.receive]")?;
        let i = self.filter_index()?;
        if let Some(bp) = rx.get("bandpass_hz").and_then(|v| v.as_array())
            && i == 0
        {
            return Ok((bp.first().and_then(as_f64), bp.get(1).and_then(as_f64)));
        }
        let offset = rx.contains_key("bandpass_hz") as usize;
        let filters = rx.get("filters").and_then(|v| v.as_array());
        match filters.and_then(|f| f.get(i - offset.min(i))).and_then(table) {
            Some(t) => Ok((t.get("highpass_hz").and_then(as_f64), t.get("lowpass_hz").and_then(as_f64))),
            None if filters.is_none() && i == 0 => Ok((None, None)),
            None => Err(format!("the receive chain has no filter {i}")),
        }
    }

    /// The transducer and environment tables with parameter values applied.
    pub fn edited(&self, p: &BTreeMap<String, f64>) -> Result<(toml::Table, toml::Table), String> {
        let mut tr = self.transducer.clone();
        let mut env = self.environment.clone();
        let fi = self.filter_index()?;
        if let Some(&v) = p.get("sensitivity_offset_db") {
            let key = if tr.contains_key("rvs") { "rvs" } else { "rvs_db" };
            let r = tr.get_mut(key).ok_or("the transducer has no rvs/rvs_db")?;
            shift_response(r, v)?;
        }
        let rx = tr.get_mut("receive").and_then(|v| v.as_table_mut()).ok_or("no [transducer.receive]")?;
        let mut rails = None;
        for (name, &v) in p {
            let fv = toml::Value::Float(v);
            match name.as_str() {
                "gain_db" => {
                    rx.insert("gain_db".into(), fv);
                }
                "bias_v" => {
                    rx.insert("bias_v".into(), fv);
                }
                "rail_low_v" | "rail_high_v" => {
                    let cur = rails.unwrap_or_else(|| {
                        rx.get("rails_v")
                            .and_then(|v| v.as_array())
                            .map(|a| [as_f64(&a[0]).unwrap_or(0.0), as_f64(&a[1]).unwrap_or(3.3)])
                            .unwrap_or([0.0, 3.3])
                    });
                    let mut r = cur;
                    r[(name == "rail_high_v") as usize] = v;
                    rails = Some(r);
                }
                "highpass_hz" | "lowpass_hz" => {
                    let key = name.as_str();
                    if let (Some(bp), 0) = (rx.get_mut("bandpass_hz").and_then(|v| v.as_array_mut()), fi) {
                        bp[(key == "lowpass_hz") as usize] = fv;
                    } else {
                        let offset = rx.contains_key("bandpass_hz") as usize;
                        let filters = rx
                            .entry("filters")
                            .or_insert_with(|| toml::Value::Array(vec![toml::Value::Table(Default::default())]))
                            .as_array_mut()
                            .ok_or("filters is not an array")?;
                        let t = filters
                            .get_mut(fi - offset.min(fi))
                            .and_then(|v| v.as_table_mut())
                            .ok_or_else(|| format!("the receive chain has no filter {fi}"))?;
                        t.insert(key.into(), fv);
                    }
                }
                "sensitivity_offset_db" => {}
                "noise_db" => {
                    let mut n = toml::Table::new();
                    n.insert("kind".into(), "white".into());
                    n.insert("level_db_re_upa_per_hz".into(), fv);
                    env.insert("noise".into(), toml::Value::Table(n));
                }
                "wall_reflection" | "bottom_reflection" | "surface_reflection" => {
                    env.insert(name.clone(), fv);
                }
                "source_level_db" => {}
                other => return Err(format!("unknown parameter {other}")),
            }
        }
        if let Some(r) = rails {
            let rx = tr.get_mut("receive").and_then(|v| v.as_table_mut()).unwrap();
            rx.insert("rails_v".into(), toml::Value::Array(vec![r[0].into(), r[1].into()]));
        }
        if let Some(&level) = p.get("source_level_db")
            && self.manifest.source.is_none()
            && let Some(s) = env.get_mut("source").and_then(|s| s.as_array_mut()).and_then(|a| a.first_mut())
            && let Some(t) = s.as_table_mut()
        {
            t.insert("source_level_db".into(), level.into());
        }
        Ok((tr, env))
    }

    /// Simulates recording `i` with parameter values `p`: samples in -1..1 as the recording has them.
    pub fn simulate(&self, i: usize, p: &BTreeMap<String, f64>) -> Result<Vec<f64>, String> {
        let r = &self.manifest.recordings[i];
        let (tr, mut env) = self.edited(p)?;
        // The source this recording hears
        let src = r.source.clone().or_else(|| self.manifest.source.clone());
        if let Some(mut s) = src {
            if let (Some(&level), Some(t)) = (p.get("source_level_db"), s.as_table_mut())
                && r.source.is_none()
            {
                t.insert("source_level_db".into(), level.into());
            }
            env.insert("source".into(), toml::Value::Array(vec![s]));
        }
        let spec: EnvSpec = toml::Value::Table(env).try_into().map_err(|e| format!("environment: {e}"))?;
        let env = Arc::new(Environment::from_spec(&spec, &self.env_dir)?);
        if !env.boundaries.contains(r.receiver_m) {
            return Err(format!("{}: receiver {:?} is outside the water", self.names[i], r.receiver_m));
        }
        let wav = &self.wavs[i];
        let fs = wav.rate as f64;
        let n = wav.samples.len();
        let start = |k: usize| Time::from_secs_f64(r.start_s + k as f64 / fs);
        match r.kind {
            Kind::Board => {
                let entry: sim_host::board::TransducerEntry =
                    toml::Value::Table(tr).try_into().map_err(|e| format!("[transducer]: {e}"))?;
                let rx = entry.receive.ok_or("no [transducer.receive]")?;
                let mut chain = rx.chain;
                chain.gain_db += r.gain_offset_db;
                let pins = chain.select_pin.map_or(0, |pin| (r.select.unwrap_or(1) as u32 & 1) << pin);
                let mut h = Hydrophone::new(env, r.receiver_m, entry.transducer, chain);
                let [lo, hi] = self.manifest.adc_range_v;
                let levels =
                    if self.manifest.adc_bits > 0 { ((1u64 << self.manifest.adc_bits) - 1) as f64 } else { 0.0 };
                Ok((0..n)
                    .map(|k| {
                        let v = h.sample(start(k), pins).clamp(lo, hi);
                        let v = if levels > 0.0 {
                            lo + ((v - lo) / (hi - lo) * levels).round() / levels * (hi - lo)
                        } else {
                            v
                        };
                        (v - (lo + hi) / 2.0) / ((hi - lo) / 2.0)
                    })
                    .collect())
            }
            Kind::Reference => {
                let t = Transducer::hydrophone(r.sensitivity_db.unwrap());
                let chain = ChainSpec { rails_v: [-1e9, 1e9], bias_v: Some(0.0), ..Default::default() };
                let mut h = Hydrophone::new(env, r.receiver_m, t, chain);
                Ok((0..n).map(|k| h.sample(start(k), 0) / r.volts_full_scale).collect())
            }
        }
    }
}

fn as_f64(v: &toml::Value) -> Option<f64> {
    v.as_float().or_else(|| v.as_integer().map(|i| i as f64))
}

/// Adds `db` to a response in any of its forms (a number, a `[[hz, db], ...]` table, a resonance's `peak_db`).
fn shift_response(r: &mut toml::Value, db: f64) -> Result<(), String> {
    match r {
        toml::Value::Float(_) | toml::Value::Integer(_) => *r = (as_f64(r).unwrap() + db).into(),
        toml::Value::Array(points) => {
            for p in points {
                let a = p.as_array_mut().ok_or("a response table is [[hz, db], ...]")?;
                a[1] = (as_f64(&a[1]).ok_or("bad response table")? + db).into();
            }
        }
        toml::Value::Table(t) => {
            let peak = t.get("peak_db").and_then(as_f64).ok_or("a resonance response needs peak_db")?;
            t.insert("peak_db".into(), (peak + db).into());
        }
        _ => return Err("unsupported response".into()),
    }
    Ok(())
}

/// The part of a WAV a recording uses.
fn trim(w: &Wav, r: &Recording) -> Wav {
    let fs = w.rate as f64;
    let from = ((r.skip_s * fs).round() as usize).min(w.samples.len());
    let to = r.duration_s.map_or(w.samples.len(), |d| (from + (d * fs).round() as usize).min(w.samples.len()));
    Wav { rate: w.rate, samples: w.samples[from..to].to_vec() }
}

// ---- features

/// In-place radix-2 FFT of `(re, im)`; the length must be a power of two.
pub fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (s, c) = (ang * k as f64).sin_cos();
                let (a, b) = (start + k, start + k + len / 2);
                let (xr, xi) = (re[b] * c - im[b] * s, re[b] * s + im[b] * c);
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
}

/// Welch power spectrum (Hann segments, half overlapping), one value per bin up to Nyquist, and the bin width.
pub fn welch(x: &[f64], fs: f64) -> (Vec<f64>, f64) {
    let mut seg = 1024usize;
    while seg > 64 && seg * 2 > x.len() {
        seg /= 2;
    }
    let mean = x.iter().sum::<f64>() / x.len().max(1) as f64;
    let w: Vec<f64> =
        (0..seg).map(|i| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / seg as f64).cos()).collect();
    let wsum: f64 = w.iter().map(|v| v * v).sum();
    let mut psd = vec![0.0; seg / 2 + 1];
    let mut count = 0;
    let mut start = 0;
    while start + seg <= x.len() {
        let mut re: Vec<f64> = (0..seg).map(|i| (x[start + i] - mean) * w[i]).collect();
        let mut im = vec![0.0; seg];
        fft(&mut re, &mut im);
        for (k, p) in psd.iter_mut().enumerate() {
            // One-sided: the negative frequencies' power folded onto the positive ones
            let fold = if k == 0 || k == seg / 2 { 1.0 } else { 2.0 };
            *p += fold * (re[k] * re[k] + im[k] * im[k]) / (wsum * fs);
        }
        count += 1;
        start += seg / 2;
    }
    for p in &mut psd {
        *p /= count.max(1) as f64;
    }
    (psd, fs / seg as f64)
}

fn db(power: f64) -> f64 {
    10.0 * power.max(1e-20).log10()
}

/// Which features a recording has: the spectrum's bands and how many decay blocks follow its ping.
#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
    /// Third-octave band centres (Hz) with at least one spectrum bin.
    pub bands: Vec<f64>,
    /// Number of 1 ms blocks after the ping compared (0: no ping found).
    pub decay: usize,
}

/// Block RMS levels (dB of the mean square) of `block` samples each, around the mean.
fn block_levels(x: &[f64], block: usize) -> Vec<f64> {
    let mean = x.iter().sum::<f64>() / x.len().max(1) as f64;
    x.chunks_exact(block.max(1))
        .map(|c| db(c.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / c.len() as f64))
        .collect()
}

fn percentile(v: &[f64], q: f64) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    s.get(((s.len() as f64 - 1.0) * q).round() as usize).copied().unwrap_or(f64::NAN)
}

/// The end of the ping: the last 1 ms block, from the loudest on, still within 10 dB of it; `None` without a
/// ping (the loudest block less than 15 dB above the floor).
fn ping_end(levels: &[f64]) -> Option<usize> {
    let (m, &max) = levels.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1))?;
    if max - percentile(levels, 0.1) < 15.0 {
        return None;
    }
    let mut e = m;
    while e + 1 < levels.len() && levels[e + 1] >= max - 10.0 {
        e += 1;
    }
    Some(e)
}

const DECAY_BLOCKS: usize = 20;

impl Layout {
    pub fn of(x: &[f64], fs: f64) -> Layout {
        let (psd, df) = welch(x, fs);
        let mut bands = Vec::new();
        let mut k = 0;
        loop {
            let c = 1000.0 * 10f64.powf(k as f64 / 10.0);
            if c * 2f64.powf(1.0 / 6.0) > 0.45 * fs {
                break;
            }
            let (lo, hi) = (c / 2f64.powf(1.0 / 6.0), c * 2f64.powf(1.0 / 6.0));
            if (0..psd.len()).any(|i| (i as f64 * df) >= lo && (i as f64 * df) < hi) {
                bands.push(c);
            }
            k += 1;
        }
        let levels = block_levels(x, (fs / 1000.0).round() as usize);
        let decay = ping_end(&levels).map_or(0, |e| DECAY_BLOCKS.min(levels.len().saturating_sub(e + 1)));
        Layout { bands, decay }
    }
}

/// A feature of a capture: its name, value, and the scale its residual is divided by.
pub type Feature = (String, f64, f64);

/// The features of a capture (`x` in -1..1 at `fs`), with their names and the scale each residual is divided by.
pub fn features(x: &[f64], fs: f64, layout: &Layout, adc_bits: u32) -> Vec<Feature> {
    let n = x.len().max(1) as f64;
    let mean = x.iter().sum::<f64>() / n;
    let ms = x.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    let peak = x.iter().fold(0.0f64, |m, v| m.max((v - mean).abs()));
    let eps = if adc_bits > 0 { 1.5 / (1u64 << adc_bits) as f64 } else { 1e-9 };
    let clipped = x.iter().filter(|v| v.abs() >= 1.0 - eps).count() as f64 / n * 100.0;
    let levels = block_levels(x, ((fs / 2000.0).round() as usize).max(1));
    let mut out = vec![
        ("dc".to_string(), mean, 0.005),
        ("rms_db".into(), db(ms), 0.5),
        ("peak_db".into(), db(peak * peak), 0.5),
        ("clipped_pct".into(), clipped, 1.0),
        ("floor_db".into(), percentile(&levels, 0.1), 1.0),
    ];
    let (psd, df) = welch(x, fs);
    for &c in &layout.bands {
        let (lo, hi) = (c / 2f64.powf(1.0 / 6.0), c * 2f64.powf(1.0 / 6.0));
        let p: f64 = psd
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as f64 * df) >= lo && (*i as f64 * df) < hi)
            .map(|(_, p)| p * df)
            .sum();
        out.push((format!("band_{:.0}hz_db", c), db(p), 1.0));
    }
    if layout.decay > 0 {
        let l1 = block_levels(x, (fs / 1000.0).round() as usize);
        let e = ping_end(&l1).unwrap_or(0);
        for k in 1..=layout.decay {
            let v = l1.get(e + k).or(l1.last()).copied().unwrap_or(-200.0);
            out.push((format!("decay_{k}ms_db"), v, 1.0));
        }
    }
    out
}

// ---- the fit

/// Per recording: the measured features and the simulated ones before and after the fit.
#[derive(Clone, Debug)]
pub struct RecordingResult {
    pub name: String,
    pub features: Vec<Feature>,
    pub before: Vec<f64>,
    pub after: Vec<f64>,
    pub simulated: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct Calibration {
    pub fit: Fit,
    pub values: BTreeMap<String, f64>,
    pub start: BTreeMap<String, f64>,
    pub recordings: Vec<RecordingResult>,
    pub report: String,
    /// The board overlay (`[transducer]`) and environment file with the fitted values.
    pub overlay: String,
    pub environment: String,
    pub plots: Vec<(String, String)>,
    /// The simulated captures with the fitted values, as WAV (file name, WAV).
    pub wavs: Vec<(String, Wav)>,
}

fn param_for(name: &str, v: f64, fs: f64) -> Param {
    match name {
        "highpass_hz" | "lowpass_hz" => Param::positive(name, v, 0.01).bounded(10.0, 0.5 * fs),
        n if n.ends_with("_reflection") => Param::new(name, v, 0.005).bounded(0.0, 0.999),
        "bias_v" | "rail_low_v" | "rail_high_v" => Param::new(name, v, 0.002),
        _ => Param::new(name, v, 0.1),
    }
}

/// Fits the manifest's parameters to its recordings.
pub fn calibrate(s: &Setup, opts: &Options) -> Result<Calibration, String> {
    let start = s.start_values()?;
    let names: Vec<String> = s.manifest.fit.clone();
    let fs_max = s.wavs.iter().map(|w| w.rate as f64).fold(0.0, f64::max);
    let params: Vec<Param> = names.iter().map(|n| param_for(n, start[n], fs_max)).collect();
    let measured: Vec<(Layout, Vec<Feature>)> = s
        .wavs
        .iter()
        .zip(&s.manifest.recordings)
        .map(|(w, r)| {
            let x: Vec<f64> = w.samples.iter().map(|v| *v as f64).collect();
            let fs = w.rate as f64;
            let l = Layout::of(&x, fs);
            let bits = if r.kind == Kind::Board { s.manifest.adc_bits } else { 0 };
            let f = features(&x, fs, &l, bits);
            (l, f)
        })
        .collect();
    let values_of = |v: &[f64]| -> BTreeMap<String, f64> { names.iter().cloned().zip(v.iter().copied()).collect() };
    let sim_features = |p: &BTreeMap<String, f64>| -> Result<Vec<Vec<f64>>, String> {
        (0..s.wavs.len())
            .map(|i| {
                let x = s.simulate(i, p)?;
                let r = &s.manifest.recordings[i];
                let bits = if r.kind == Kind::Board { s.manifest.adc_bits } else { 0 };
                Ok(features(&x, s.wavs[i].rate as f64, &measured[i].0, bits).into_iter().map(|f| f.1).collect())
            })
            .collect()
    };
    let resid = |v: &[f64]| -> Vec<f64> {
        let total: usize = measured.iter().map(|m| m.1.len()).sum();
        match sim_features(&values_of(v)) {
            Ok(sims) => measured
                .iter()
                .zip(&sims)
                .flat_map(|((_, meas), sim)| meas.iter().zip(sim).map(|(m, s)| (s - m.1) / m.2))
                .collect(),
            Err(_) => vec![f64::INFINITY; total],
        }
    };
    let mut f = fit::fit(&params, resid, opts);
    // Features of a capture are not smooth in every parameter (a percentile, the peak, a standing-wave pattern of
    // reflections): when derivatives stall, polish with Nelder–Mead from where they stopped
    if opts.method == fit::Method::LevenbergMarquardt && f.rms() > 0.05 {
        let mut from = params.clone();
        for (p, q) in from.iter_mut().zip(&f.params) {
            p.value = q.value;
        }
        let nm = fit::fit(&from, resid, &Options { method: fit::Method::NelderMead, ..opts.clone() });
        if nm.cost < f.cost {
            let (initial, evals, iters) = (f.initial_cost, f.evaluations, f.iterations);
            f = nm;
            f.initial_cost = initial;
            f.evaluations += evals;
            f.iterations += iters;
        }
    }
    let values = values_of(&f.params.iter().map(|p| p.value).collect::<Vec<_>>());
    let before = sim_features(&start)?;
    let after = sim_features(&values)?;
    let mut recordings = Vec::new();
    let mut wavs = Vec::new();
    for i in 0..s.wavs.len() {
        let sim = s.simulate(i, &values)?;
        wavs.push((
            format!("sim_{}.wav", s.names[i]),
            Wav { rate: s.wavs[i].rate, samples: sim.iter().map(|v| *v as f32).collect() },
        ));
        recordings.push(RecordingResult {
            name: s.names[i].clone(),
            features: measured[i].1.clone(),
            before: before[i].clone(),
            after: after[i].clone(),
            simulated: sim,
        });
    }
    let (tr, env) = s.edited(&values)?;
    let mut ov = toml::Table::new();
    ov.insert("transducer".into(), toml::Value::Table(tr));
    let overlay = format!(
        "# Fitted by `firmware_simulator calibrate acoustics`; see report.md. Paste into the board's overlay\n# \
         (boards/overlays/...) or a robot file's [board.transducer].\n{}",
        toml::to_string(&ov).map_err(|e| e.to_string())?
    );
    let environment = format!(
        "# Fitted by `firmware_simulator calibrate acoustics` from {}; see report.md.\n{}",
        s.manifest.environment.display(),
        toml::to_string(&env).map_err(|e| e.to_string())?
    );
    let plots = plots(s, &recordings);
    let mut cal =
        Calibration { fit: f, values, start, recordings, report: String::new(), overlay, environment, plots, wavs };
    cal.report = report(s, &cal);
    Ok(cal)
}

fn plots(s: &Setup, recs: &[RecordingResult]) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for (i, r) in recs.iter().enumerate() {
        let bands: Vec<(usize, f64)> = r
            .features
            .iter()
            .enumerate()
            .filter_map(|(k, f)| {
                f.0.strip_prefix("band_")
                    .and_then(|b| b.strip_suffix("hz_db"))
                    .and_then(|b| b.parse().ok())
                    .map(|c| (k, c))
            })
            .collect();
        let pick = |v: &dyn Fn(usize) -> f64| bands.iter().map(|&(k, c)| (c, v(k))).collect::<Vec<_>>();
        out.push((
            format!("spectrum_{}.svg", r.name),
            svg_plot(
                &format!("{}: third-octave spectrum", r.name),
                "Hz",
                "dB",
                &[
                    Series::dots("recorded", pick(&|k| r.features[k].1)),
                    Series::line("fitted", pick(&|k| r.after[k])),
                    Series::line("before", pick(&|k| r.before[k])),
                ],
                true,
            ),
        ));
        let fs = s.wavs[i].rate as f64;
        let b = (fs / 1000.0).round() as usize;
        let rec: Vec<f64> = s.wavs[i].samples.iter().map(|v| *v as f64).collect();
        let env = |x: &[f64]| -> Vec<(f64, f64)> {
            block_levels(x, b).into_iter().enumerate().map(|(k, l)| (k as f64 + 0.5, l)).collect()
        };
        out.push((
            format!("envelope_{}.svg", r.name),
            svg_plot(
                &format!("{}: level in 1 ms blocks", r.name),
                "ms",
                "dB re full scale",
                &[Series::dots("recorded", env(&rec)), Series::line("fitted", env(&r.simulated))],
                false,
            ),
        ));
    }
    out
}

fn report(s: &Setup, cal: &Calibration) -> String {
    let mut o = String::from("# Acoustic calibration\n\n");
    let f = &cal.fit;
    let _ = writeln!(
        o,
        "{} recording(s); {} evaluations, {} iterations; weighted residual RMS {:.2} at the start, {:.2} fitted (1 \
         is a typical feature off by its scale: 1 dB for levels, 1 % for clipping, 0.005 of full scale for DC).\n",
        cal.recordings.len(),
        f.evaluations,
        f.iterations,
        (2.0 * f.initial_cost / f.residuals.len().max(1) as f64).sqrt(),
        f.rms()
    );
    o += "## Parameters\n\n";
    let rows: Vec<Vec<String>> = f
        .params
        .iter()
        .zip(&f.std_err)
        .map(|(p, se)| {
            let what = PARAMS.iter().find(|x| x.0 == p.name).map_or("", |x| x.1);
            vec![
                format!("`{}`", p.name),
                format!("{:.3}", cal.start[&p.name]),
                format!("{:.3}", p.value),
                if se.is_finite() { format!("{se:.3}") } else { "-".into() },
                what.to_string(),
            ]
        })
        .collect();
    o += &markdown(&["parameter", "before", "fitted", "± (1 sigma)", "what"], &rows);
    o.push('\n');
    let deg = f.degenerate_pairs(0.95);
    for (a, b, c) in &deg {
        let _ = writeln!(o, "* `{a}` and `{b}` are {c:+.2} correlated: these recordings cannot tell them apart.");
    }
    if s.manifest.fit.iter().any(|p| p == "gain_db") && s.manifest.fit.iter().any(|p| p == "sensitivity_offset_db") {
        o += "* `gain_db` and `sensitivity_offset_db` only matter as their sum in a board recording; a reference \
              hydrophone recording at the same place separates them.\n";
    }
    if !deg.is_empty() {
        o.push('\n');
    }
    for r in &cal.recordings {
        let _ = writeln!(o, "## {}\n", r.name);
        let rows: Vec<Vec<String>> = r
            .features
            .iter()
            .zip(r.before.iter().zip(&r.after))
            .map(|((n, m, sc), (b, a))| {
                vec![
                    format!("`{n}`"),
                    format!("{m:.3}"),
                    format!("{b:.3}"),
                    format!("{a:.3}"),
                    format!("{:+.2}", (a - m) / sc),
                ]
            })
            .collect();
        o += &markdown(&["feature", "recorded", "simulated before", "simulated fitted", "residual (scales)"], &rows);
        let _ = writeln!(
            o,
            "\nPlots: `spectrum_{0}.svg`, `envelope_{0}.svg`; the fitted simulation: `sim_{0}.wav`.\n",
            r.name
        );
    }
    o += "## To use it\n\n`overlay.toml` is the board's `[transducer]` with the fitted values (paste it into the overlay \
          and rerun `python3 scripts/gen_boards.py`, or into a robot file's `[board.transducer]`); `environment.toml` \
          is the environment file with them. Features with residuals far above 1 after the fit are what the model \
          cannot reproduce: look at them before trusting the numbers.\n";
    o
}

/// A manifest with recordings for tests and examples: the parts of [`Setup::new`] for one board file and
/// environment file, recordings described by `recordings` (their `file` is only a name).
pub fn manifest_in_memory(
    board: &Path,
    environment: &Path,
    fit: &[&str],
    source: Option<toml::Value>,
    recordings: Vec<Recording>,
) -> Result<(Manifest, toml::Table, toml::Table, PathBuf), String> {
    let read = |p: &Path| -> Result<toml::Table, String> {
        let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
        toml::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))
    };
    let tr = read(board)?.get("transducer").and_then(table).cloned().ok_or("no [transducer]")?;
    let env = read(environment)?;
    let m = Manifest {
        board: board.to_path_buf(),
        environment: environment.to_path_buf(),
        fit: fit.iter().map(|s| s.to_string()).collect(),
        method: None,
        source,
        start: BTreeMap::new(),
        adc_bits: 12,
        adc_range_v: adc_range(),
        recordings,
    };
    Ok((m, tr, env, environment.parent().unwrap().to_path_buf()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    fn pinger() -> toml::Value {
        toml::from_str::<toml::Table>(
            "kind = \"pinger\"\nposition_m = [20.0, 6.0, 2.0]\nfrequency_hz = 25000\npulse_ms = 4\nperiod_ms = 100\n\
             source_level_db = 180\nstart_ms = 2",
        )
        .map(toml::Value::Table)
        .unwrap()
    }

    fn rec(name: &str, x: f64, kind: Kind) -> Recording {
        Recording {
            file: format!("{name}.wav").into(),
            name: Some(name.into()),
            kind,
            receiver_m: [x, 6.0, 1.5],
            source: None,
            select: Some(1),
            start_s: 0.0,
            skip_s: 0.0,
            duration_s: None,
            gain_offset_db: 0.0,
            sensitivity_db: (kind == Kind::Reference).then_some(-200.0),
            volts_full_scale: 1.0,
        }
    }

    /// The pool and the IVC board as shipped, with recordings "made" by simulating `truth`; `extra_noise` adds
    /// independent white noise to each recording (and a different environment seed), as a real one would differ.
    fn setup(fit: &[&str], recs: Vec<Recording>, truth: &[(&str, f64)], extra_noise: f64) -> Setup {
        let (m, tr, env, dir) = manifest_in_memory(
            &repo().join("boards/talos/ivc_board.toml"),
            &repo().join("environments/pool.toml"),
            fit,
            Some(pinger()),
            recs,
        )
        .unwrap();
        let blank = vec![Wav { rate: 500_000, samples: vec![0.0; 12_500] }; m.recordings.len()];
        let names = m.recordings.iter().map(|r| r.name.clone().unwrap()).collect();
        // The truth: everything named in it, simulated with every parameter it names
        let mut tm = m.clone();
        tm.fit = truth.iter().map(|t| t.0.to_string()).collect();
        let mut tenv = env.clone();
        if extra_noise > 0.0 {
            tenv.insert("seed".into(), 99.into());
        }
        let ts = Setup::new(tm, tr.clone(), tenv, dir.clone(), blank, names).unwrap();
        let tv: BTreeMap<String, f64> = truth.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        let wavs = (0..ts.wavs.len())
            .map(|i| {
                let x = ts.simulate(i, &tv).unwrap();
                let samples = x
                    .iter()
                    .enumerate()
                    .map(|(n, v)| {
                        (v + extra_noise * sim_acoustics::noise::gaussian(1234 + i as u64, n as u64)).clamp(-1.0, 1.0)
                            as f32
                    })
                    .collect();
                // Through a 16-bit WAV file and back, as a recording arrives
                Wav::parse(&Wav { rate: 500_000, samples }.to_bytes()).unwrap()
            })
            .collect();
        Setup::new(m, tr, env, dir, wavs, ts.names.clone()).unwrap()
    }

    #[test]
    fn fft_and_band_levels_find_a_tone() {
        let mut re: Vec<f64> = (0..64).map(|i| (2.0 * std::f64::consts::PI * 4.0 * i as f64 / 64.0).cos()).collect();
        let mut im = vec![0.0; 64];
        fft(&mut re, &mut im);
        assert!((re[4] - 32.0).abs() < 1e-9 && (re[60] - 32.0).abs() < 1e-9);
        assert!(re.iter().enumerate().filter(|(k, _)| *k != 4 && *k != 60).all(|(_, v)| v.abs() < 1e-9));
        // A 25 kHz sine at 0.5 amplitude: mean square 0.125 (-9 dB), nearly all of it in the 25 kHz band
        let fs = 500_000.0;
        let x: Vec<f64> =
            (0..20_000).map(|i| 0.5 * (2.0 * std::f64::consts::PI * 25_000.0 * i as f64 / fs).sin()).collect();
        let l = Layout::of(&x, fs);
        assert_eq!(l.decay, 0);
        let f = features(&x, fs, &l, 0);
        let get = |n: &str| f.iter().find(|x| x.0 == n).unwrap().1;
        assert!((get("rms_db") + 9.03).abs() < 0.01);
        assert!((get("peak_db") + 6.02).abs() < 0.01);
        assert!((get("band_25119hz_db") + 9.03).abs() < 0.3, "{f:?}");
        assert!(get("band_1000hz_db") < -60.0);
        assert_eq!(get("clipped_pct"), 0.0);
    }

    #[test]
    fn a_ping_has_a_decay_and_clipping_is_counted() {
        let fs = 500_000.0;
        let x: Vec<f64> = (0..15_000)
            .map(|i| {
                let t = i as f64 / fs;
                let env = if t < 0.002 {
                    1e-3
                } else if t < 0.006 {
                    2.0
                } else {
                    2.0 * (-(t - 0.006) / 0.002).exp() + 1e-3
                };
                (env * (2.0 * std::f64::consts::PI * 25_000.0 * t).sin()).clamp(-1.0, 1.0)
            })
            .collect();
        let l = Layout::of(&x, fs);
        assert_eq!(l.decay, 20);
        let f = features(&x, fs, &l, 12);
        let get = |n: &str| f.iter().find(|x| x.0 == n).unwrap().1;
        // 2 ms of 4 ms clipped hard; then about 4.3 dB less per ms
        assert!(get("clipped_pct") > 5.0);
        assert!(get("decay_2ms_db") - get("decay_3ms_db") > 3.0);
    }

    #[test]
    fn recovers_gain_corner_noise_and_wall_reflection() {
        let truth = [("gain_db", 12.0), ("lowpass_hz", 45_000.0), ("noise_db", 66.0), ("wall_reflection", 0.35)];
        let fit = ["gain_db", "lowpass_hz", "noise_db", "wall_reflection"];
        let recs = vec![rec("near", 16.0, Kind::Board), rec("mid", 10.0, Kind::Board), rec("far", 3.0, Kind::Board)];
        let s = setup(&fit, recs, &truth, 0.0);
        let cal = calibrate(&s, &Options::default()).unwrap();
        let v = &cal.values;
        assert!((v["gain_db"] - 12.0).abs() < 0.3, "{}", cal.report);
        assert!((v["lowpass_hz"] / 45_000.0 - 1.0).abs() < 0.03, "{}", cal.report);
        assert!((v["noise_db"] - 66.0).abs() < 0.5, "{}", cal.report);
        assert!((v["wall_reflection"] - 0.35).abs() < 0.03, "{}", cal.report);
        assert!(cal.fit.cost < 0.01 * cal.fit.initial_cost);
        // The outputs carry the values and load as what they are
        let ov: toml::Table = toml::from_str(&cal.overlay).unwrap();
        let entry: sim_host::board::TransducerEntry = ov["transducer"].clone().try_into().unwrap();
        assert!((entry.receive.unwrap().chain.gain_db - v["gain_db"]).abs() < 1e-9);
        let env = Environment::parse(&cal.environment, &repo().join("environments")).unwrap();
        assert!((env.reflections.walls - v["wall_reflection"]).abs() < 1e-9);
        assert_eq!(cal.wavs.len(), 3);
        assert_eq!(cal.plots.len(), 6);
        assert!(cal.report.contains("## near"));
    }

    #[test]
    fn recovers_a_clipping_chain_and_its_offset_from_noisy_recordings() {
        // The assumed 40 dB really is clipping, and the ADC sits 40 mV high
        let truth = [("gain_db", 34.0), ("bias_v", 1.69), ("noise_db", 55.0)];
        let fit = ["gain_db", "bias_v", "noise_db"];
        let recs = vec![rec("near", 17.0, Kind::Board), rec("far", 4.0, Kind::Board)];
        let s = setup(&fit, recs, &truth, 0.0005);
        let cal = calibrate(&s, &Options::default()).unwrap();
        let v = &cal.values;
        assert!((v["gain_db"] - 34.0).abs() < 1.0, "{}", cal.report);
        assert!((v["bias_v"] - 1.69).abs() < 0.005, "{}", cal.report);
        // The extra noise (about one step of the ADC) lifts the floor a little: the fit takes it as ambient noise
        assert!((v["noise_db"] - 55.0).abs() < 1.5, "{}", cal.report);
        let near = &cal.recordings[0];
        let clipped = near.features.iter().position(|f| f.0 == "clipped_pct").unwrap();
        assert!(near.features[clipped].1 > 1.0, "the truth clips: {:?}", near.features[clipped]);
    }

    #[test]
    fn reference_hydrophone_recordings_fit_the_source_and_the_bottom() {
        let truth = [("source_level_db", 172.0), ("bottom_reflection", 0.3)];
        let fit = ["source_level_db", "bottom_reflection"];
        let recs = vec![rec("ref_near", 16.0, Kind::Reference), rec("ref_far", 8.0, Kind::Reference)];
        let s = setup(&fit, recs, &truth, 0.0);
        let cal = calibrate(&s, &Options::default()).unwrap();
        assert!((cal.values["source_level_db"] - 172.0).abs() < 0.3, "{}", cal.report);
        assert!((cal.values["bottom_reflection"] - 0.3).abs() < 0.04, "{}", cal.report);
    }

    #[test]
    fn manifest_errors_and_response_shifts() {
        let (m, tr, env, dir) = manifest_in_memory(
            &repo().join("boards/talos/ivc_board.toml"),
            &repo().join("environments/pool.toml"),
            &["gain_db"],
            None,
            vec![rec("a", 10.0, Kind::Board)],
        )
        .unwrap();
        let w = vec![Wav { rate: 500_000, samples: vec![0.0; 1000] }];
        let mut bad = m.clone();
        bad.fit = vec!["volume".into()];
        assert!(Setup::new(bad, tr.clone(), env.clone(), dir.clone(), w.clone(), vec!["a".into()]).is_err());
        let mut bad = m.clone();
        bad.recordings[0].kind = Kind::Reference;
        bad.recordings[0].sensitivity_db = None;
        assert!(Setup::new(bad, tr.clone(), env.clone(), dir.clone(), w.clone(), vec!["a".into()]).is_err());
        // A high-pass corner on filter 1 (a low-pass only) needs a starting value
        let mut bad = m.clone();
        bad.fit = vec!["highpass_hz".into()];
        let e = Setup::new(bad.clone(), tr.clone(), env.clone(), dir.clone(), w.clone(), vec!["a".into()]).unwrap_err();
        assert!(e.contains("[start]"), "{e}");
        bad.start.insert("highpass_hz".into(), 5000.0);
        let s = Setup::new(bad, tr.clone(), env.clone(), dir.clone(), w.clone(), vec!["a".into()]).unwrap();
        let (t, _) = s.edited(&s.start_values().unwrap()).unwrap();
        assert_eq!(t["receive"]["filters"][1]["highpass_hz"].as_float(), Some(5000.0));
        for (mut r, want) in [
            (toml::Value::Float(-190.0), "-187.0"),
            (toml::from_str::<toml::Table>("x = [[1000.0, -190.0]]").unwrap()["x"].clone(), "-187.0"),
        ] {
            shift_response(&mut r, 3.0).unwrap();
            assert!(r.to_string().contains(want), "{r}");
        }
    }
}
