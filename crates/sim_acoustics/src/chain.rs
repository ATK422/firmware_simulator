//! Receive electronics between a hydrophone and an ADC input: preamp gain, band-pass filtering (optionally one of
//! several filters chosen by a GPIO), DC bias and clipping at the rails.

use serde::Deserialize;

/// A second-order IIR section (transposed direct form II).
#[derive(Clone, Copy, Debug, Default)]
pub struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
    z: [f64; 2],
}

impl Biquad {
    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[0] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }

    pub fn reset(&mut self) {
        self.z = [0.0; 2];
    }

    /// Butterworth-family low or high pass section with quality factor `q` (RBJ cookbook, bilinear transform).
    fn pass(hz: f64, q: f64, fs: f64, high: bool) -> Self {
        let w = 2.0 * std::f64::consts::PI * hz / fs;
        let (sin, cos) = w.sin_cos();
        let alpha = sin / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b = if high {
            [(1.0 + cos) / 2.0, -(1.0 + cos), (1.0 + cos) / 2.0]
        } else {
            [(1.0 - cos) / 2.0, 1.0 - cos, (1.0 - cos) / 2.0]
        };
        Biquad { b: b.map(|v| v / a0), a: [-2.0 * cos / a0, (1.0 - alpha) / a0], z: [0.0; 2] }
    }

    /// Magnitude response at `hz` for sample rate `fs`.
    pub fn magnitude(&self, hz: f64, fs: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * hz / fs;
        let z1 = (w.cos(), -w.sin());
        let z2 = ((2.0 * w).cos(), -(2.0 * w).sin());
        let num = (self.b[0] + self.b[1] * z1.0 + self.b[2] * z2.0, self.b[1] * z1.1 + self.b[2] * z2.1);
        let den = (1.0 + self.a[0] * z1.0 + self.a[1] * z2.0, self.a[0] * z1.1 + self.a[1] * z2.1);
        (num.0.hypot(num.1)) / (den.0.hypot(den.1))
    }
}

/// Q factors of the second-order sections of an even-order Butterworth filter.
fn butterworth_qs(order: u32) -> Vec<f64> {
    let n = order.max(2) & !1;
    (0..n / 2).map(|k| 1.0 / (2.0 * ((2 * k + 1) as f64 * std::f64::consts::PI / (2 * n) as f64).sin())).collect()
}

/// A filter as specified: corners in Hz and a Butterworth order (rounded up to even) per edge.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FilterSpec {
    /// High-pass corner.
    pub highpass_hz: Option<f64>,
    /// Low-pass corner.
    pub lowpass_hz: Option<f64>,
    #[serde(default = "two")]
    pub order: u32,
    /// Extra gain of this path, dB.
    #[serde(default)]
    pub gain_db: f64,
}

fn two() -> u32 {
    2
}

impl FilterSpec {
    pub fn bandpass(lo: f64, hi: f64, order: u32) -> Self {
        FilterSpec { highpass_hz: Some(lo), lowpass_hz: Some(hi), order, gain_db: 0.0 }
    }

    fn design(&self, fs: f64) -> Vec<Biquad> {
        let mut out = Vec::new();
        for q in butterworth_qs(self.order) {
            if let Some(f) = self.highpass_hz {
                out.push(Biquad::pass(f, q, fs, true));
            }
            if let Some(f) = self.lowpass_hz {
                out.push(Biquad::pass(f.min(fs * 0.45), q, fs, false));
            }
        }
        out
    }
}

/// The receive chain as specified.
#[derive(Clone, Debug, Deserialize)]
pub struct ChainSpec {
    /// Preamp gain, dB.
    #[serde(default)]
    pub gain_db: f64,
    /// `[highpass, lowpass]` corners of a single band-pass filter (shorthand for one entry in `filters`).
    pub bandpass_hz: Option<[f64; 2]>,
    #[serde(default = "two")]
    pub order: u32,
    /// Alternative filters; with `select_pin`, filter 0 is used while the pin is low and filter 1 while it is high.
    #[serde(default)]
    pub filters: Vec<FilterSpec>,
    pub select_pin: Option<u8>,
    /// DC level the signal rides on; half the rail by default.
    pub bias_v: Option<f64>,
    /// Supply rails the output clips at.
    #[serde(default = "rails")]
    pub rails_v: [f64; 2],
}

fn rails() -> [f64; 2] {
    [0.0, 3.3]
}

impl Default for ChainSpec {
    fn default() -> Self {
        ChainSpec {
            gain_db: 0.0,
            bandpass_hz: None,
            order: 2,
            filters: Vec::new(),
            select_pin: None,
            bias_v: None,
            rails_v: rails(),
        }
    }
}

/// A running receive chain at a fixed sample rate.
#[derive(Clone, Debug)]
pub struct Chain {
    pub spec: ChainSpec,
    gain: f64,
    paths: Vec<(Vec<Biquad>, f64)>,
    fs: f64,
}

impl Chain {
    pub fn new(spec: ChainSpec, fs: f64) -> Self {
        let mut filters = spec.filters.clone();
        if let Some([lo, hi]) = spec.bandpass_hz {
            filters.insert(0, FilterSpec::bandpass(lo, hi, spec.order));
        }
        if filters.is_empty() {
            filters.push(FilterSpec { highpass_hz: None, lowpass_hz: None, order: 2, gain_db: 0.0 });
        }
        let paths = filters.iter().map(|f| (f.design(fs), 10f64.powf(f.gain_db / 20.0))).collect();
        Chain { gain: 10f64.powf(spec.gain_db / 20.0), paths, spec, fs }
    }

    fn selected(&self, pins: u32) -> usize {
        match self.spec.select_pin {
            Some(p) => ((pins >> p) & 1) as usize % self.paths.len(),
            None => 0,
        }
    }

    /// Runs one sample of hydrophone voltage through every filter (so switching keeps each settled) and returns the
    /// selected output at the ADC pin.
    pub fn process(&mut self, v: f64, pins: u32) -> f64 {
        let sel = self.selected(pins);
        let mut out = 0.0;
        for (i, (sections, g)) in self.paths.iter_mut().enumerate() {
            let mut x = v * self.gain * *g;
            for s in sections.iter_mut() {
                x = s.process(x);
            }
            if i == sel {
                out = x;
            }
        }
        let [lo, hi] = self.spec.rails_v;
        (out + self.spec.bias_v.unwrap_or((lo + hi) / 2.0)).clamp(lo, hi)
    }

    /// Whether `v`, an output of [`Chain::process`], sits at a rail: the signal was clipped.
    pub fn at_rail(&self, v: f64) -> bool {
        let [lo, hi] = self.spec.rails_v;
        v <= lo || v >= hi
    }

    pub fn reset(&mut self) {
        for (sections, _) in &mut self.paths {
            sections.iter_mut().for_each(Biquad::reset);
        }
    }

    /// Small-signal gain (V at the pin per V from the hydrophone) at `hz` through the filter selected by `pins`.
    pub fn gain_at(&self, hz: f64, pins: u32) -> f64 {
        let (sections, g) = &self.paths[self.selected(pins)];
        self.gain * g * sections.iter().map(|s| s.magnitude(hz, self.fs)).product::<f64>()
    }

    pub fn sample_rate(&self) -> f64 {
        self.fs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn butterworth_bandpass_response() {
        let spec = ChainSpec { gain_db: 20.0, bandpass_hz: Some([20_000.0, 60_000.0]), order: 4, ..Default::default() };
        let c = Chain::new(spec, 1e6);
        let mid = c.gain_at(35_000.0, 0);
        assert!((mid - 10.0).abs() < 0.3, "{mid}");
        // -3 dB at the corners, steep outside
        assert!((c.gain_at(20_000.0, 0) / 10.0 - 0.707).abs() < 0.08, "{}", c.gain_at(20_000.0, 0));
        assert!(c.gain_at(5_000.0, 0) < 0.1);
        assert!(c.gain_at(200_000.0, 0) < 0.2);
        assert_eq!(butterworth_qs(2), vec![std::f64::consts::FRAC_1_SQRT_2]);
    }

    #[test]
    fn processes_a_tone_with_bias_clipping_and_select() {
        let lowpass = FilterSpec { highpass_hz: None, lowpass_hz: Some(60_000.0), order: 2, gain_db: 0.0 };
        let spec = ChainSpec {
            filters: vec![FilterSpec::bandpass(40_000.0, 60_000.0, 4), lowpass],
            select_pin: Some(3),
            ..Default::default()
        };
        let mut c = Chain::new(spec, 1e6);
        // A 25 kHz tone: rejected by the 40-60 kHz path (pin low), passed by the low-pass path (pin high)
        let run = |c: &mut Chain, pins: u32| {
            let mut peak: f64 = 0.0;
            for n in 0..4000 {
                let v = 0.5 * (2.0 * std::f64::consts::PI * 25_000.0 * n as f64 / 1e6).sin();
                let out = c.process(v, pins) - 1.65;
                if n > 2000 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };
        let low = run(&mut c, 0);
        let high = run(&mut c, 1 << 3);
        assert!(low < 0.15 && (high - 0.5).abs() < 0.05, "{low} {high}");
        // Clipping at the rails
        c.reset();
        assert_eq!(c.process(100.0, 1 << 3), 3.3);
        assert_eq!(c.process(-1000.0, 1 << 3), 0.0);
    }
}
