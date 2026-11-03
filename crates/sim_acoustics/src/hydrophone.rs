//! A hydrophone in an environment, feeding an ADC input through its receive chain.
//!
//! The acoustic signal is computed on a fixed internal sample grid (1 MHz by default): every source's pressure
//! reaches the hydrophone over each image path (delayed, attenuated, reflected), is converted by the transducer's
//! sensitivity and beam pattern, gets ambient noise added, and runs through the receive chain's filters. The grid
//! is advanced lazily when the ADC samples; after a gap longer than [`MAX_GAP_STEPS`] the filters are restarted with
//! [`PREROLL_STEPS`] of history, which is far longer than they take to settle. Noise is a function of the grid step
//! and the environment's seed, so runs are reproducible however the ADC samples.
//!
//! Absorption and the transducer's sensitivity are applied at each source's nominal frequency (a narrowband
//! approximation); ambient noise is white at the level the noise model gives at the chain's centre frequency.

use std::sync::Arc;

use sim_core::analog::AnalogSource;
use sim_core::time::Time;

use crate::chain::{Chain, ChainSpec};
use crate::env::Environment;
use crate::geometry::Vec3;
use crate::noise;
use crate::transducer::Transducer;
use crate::wav::Wav;

pub const DEFAULT_RATE_HZ: f64 = 1_000_000.0;
pub const MAX_GAP_STEPS: u64 = 2_000;
pub const PREROLL_STEPS: u64 = 1_000;

#[derive(Clone, Debug)]
struct Tap {
    source: usize,
    delay_s: f64,
    /// Volts at the hydrophone per µPa at 1 m from the source.
    volts_per_upa: f64,
}

#[derive(Clone, Debug)]
pub struct Hydrophone {
    pub env: Arc<Environment>,
    pub position: Vec3,
    pub transducer: Transducer,
    chain: Chain,
    rate: f64,
    taps: Vec<Tap>,
    noise_rms_v: f64,
    seed: u64,
    step: Option<u64>,
    last: f64,
    /// ADC samples (time, volts at the pin), when recording.
    pub recording: Option<Vec<(Time, f32)>>,
}

impl Hydrophone {
    pub fn new(env: Arc<Environment>, position: Vec3, transducer: Transducer, chain: ChainSpec) -> Self {
        Self::with_rate(env, position, transducer, chain, DEFAULT_RATE_HZ)
    }

    pub fn with_rate(
        env: Arc<Environment>,
        position: Vec3,
        transducer: Transducer,
        chain: ChainSpec,
        rate: f64,
    ) -> Self {
        let centre = match (chain.bandpass_hz, chain.filters.first()) {
            (Some([lo, hi]), _) => (lo * hi).sqrt(),
            (None, Some(f)) => match (f.highpass_hz, f.lowpass_hz) {
                (Some(lo), Some(hi)) => (lo * hi).sqrt(),
                (Some(f), None) | (None, Some(f)) => f,
                (None, None) => 30_000.0,
            },
            (None, None) => 30_000.0,
        };
        let chain = Chain::new(chain, rate);
        let mut taps = Vec::new();
        for (i, s) in env.sources.iter().enumerate() {
            let hz = s.nominal_hz();
            let wavelength = env.sound_speed / hz;
            let sens = transducer.sensitivity(hz);
            for p in env.paths(s.position, position, hz) {
                let beam = transducer.beam_gain(p.arrival, wavelength);
                taps.push(Tap { source: i, delay_s: p.delay_s, volts_per_upa: p.gain * beam * sens });
            }
        }
        let level = env.noise.level_db(centre);
        let noise_rms_v = if level.is_finite() {
            10f64.powf(level / 20.0) * (rate / 2.0).sqrt() * transducer.sensitivity(centre)
        } else {
            0.0
        };
        let seed = env.seed
            ^ noise::hash(position[0].to_bits() ^ position[1].to_bits().rotate_left(21), position[2].to_bits());
        Hydrophone {
            env,
            position,
            transducer,
            chain,
            rate,
            taps,
            noise_rms_v,
            seed,
            step: None,
            last: 0.0,
            recording: None,
        }
    }

    /// Starts keeping every ADC sample.
    pub fn record(mut self) -> Self {
        self.recording = Some(Vec::new());
        self
    }

    /// Hydrophone output (before the chain) at grid step `n`.
    fn raw(&self, n: u64) -> f64 {
        let t = n as f64 / self.rate;
        let mut v = 0.0;
        for tap in &self.taps {
            v += tap.volts_per_upa * self.env.sources[tap.source].pressure(t - tap.delay_s);
        }
        if self.noise_rms_v > 0.0 {
            v += self.noise_rms_v * noise::gaussian(self.seed, n);
        }
        if let Some(sh) = &self.env.shrimp {
            v += self.shrimp(sh, t);
        }
        v
    }

    fn shrimp(&self, sh: &noise::Shrimp, t: f64) -> f64 {
        const BIN: f64 = 1e-3;
        let sens = self.transducer.sensitivity(30_000.0);
        let peak = 10f64.powf(sh.peak_db_re_upa / 20.0) * sens;
        let decay = sh.decay_us * 1e-6;
        let m = (t / BIN).floor() as i64;
        let mut v = 0.0;
        for bin in [m - 1, m] {
            if bin < 0 {
                continue;
            }
            let b = bin as u64;
            if noise::uniform(self.seed ^ 0x5348_5249_4D50, b) > sh.rate_hz * BIN {
                continue;
            }
            let tc = bin as f64 * BIN + noise::uniform(self.seed ^ 0xC11C, b) * BIN;
            if t >= tc {
                let sign = if noise::hash(self.seed, b) & 1 == 0 { 1.0 } else { -1.0 };
                v += sign * peak * (-(t - tc) / decay).exp();
            }
        }
        v
    }

    /// Advances the chain to `t` and returns the voltage at the ADC pin.
    pub fn sample(&mut self, t: Time, pins: u32) -> f64 {
        let k = (t.as_secs_f64() * self.rate).floor() as u64;
        let from = match self.step {
            Some(s) if k <= s => return self.last,
            Some(s) if k - s <= MAX_GAP_STEPS => s + 1,
            _ => {
                self.chain.reset();
                k.saturating_sub(PREROLL_STEPS)
            }
        };
        for n in from..=k {
            self.last = self.chain.process(self.raw(n), pins);
        }
        self.step = Some(k);
        if let Some(r) = &mut self.recording {
            r.push((t, self.last as f32));
        }
        self.last
    }

    /// Expected peak voltage swing at the pin from source `i` over the direct path alone, for steady state (no
    /// multipath interference), through the filter `pins` select.
    pub fn direct_peak_v(&self, i: usize, pins: u32) -> f64 {
        let s = &self.env.sources[i];
        let hz = s.nominal_hz();
        let p = self.env.paths(s.position, self.position, hz);
        let direct = p.iter().find(|p| p.reflections == 0).map_or(0.0, |p| p.gain);
        let beam = self.transducer.beam_gain(
            p.iter().find(|p| p.reflections == 0).map_or([0.0, 0.0, 1.0], |p| p.arrival),
            self.env.sound_speed / hz,
        );
        s.rms_upa
            * std::f64::consts::SQRT_2
            * direct
            * beam
            * self.transducer.sensitivity(hz)
            * self.chain.gain_at(hz, pins)
    }

    /// The recording as a WAV file at `rate` (samples are placed at their times; gaps are silent).
    pub fn recording_wav(&self, rate: u32) -> Option<Wav> {
        let r = self.recording.as_ref()?;
        let start = r.first()?.0;
        let end = r.last()?.0;
        let n = ((end - start).as_secs_f64() * rate as f64) as usize + 1;
        let mut samples = vec![0.0f32; n];
        let bias = self.chain.spec.bias_v.unwrap_or((self.chain.spec.rails_v[0] + self.chain.spec.rails_v[1]) / 2.0);
        let span = (self.chain.spec.rails_v[1] - self.chain.spec.rails_v[0]) as f32 / 2.0;
        for (t, v) in r {
            let i = ((*t - start).as_secs_f64() * rate as f64) as usize;
            if let Some(s) = samples.get_mut(i) {
                *s = (*v - bias as f32) / span;
            }
        }
        Some(Wav { rate, samples })
    }
}

impl AnalogSource for Hydrophone {
    fn voltage(&mut self, t: Time, pins: u32) -> f64 {
        self.sample(t, pins)
    }

    fn clone_box(&self) -> Box<dyn AnalogSource> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::{Source, Waveform};
    use crate::water::Water;

    fn tone_env(hz: f64, level_db: f64, at: Vec3) -> Arc<Environment> {
        let mut env = Environment::open_water(Water::default());
        env.sources.push(Source {
            position: at,
            waveform: Waveform::Tone { hz },
            rms_upa: 10f64.powf(level_db / 20.0),
        });
        Arc::new(env)
    }

    fn peak(h: &mut Hydrophone, from: Time, n: usize) -> f64 {
        // Sample at 500 kS/s like the IVC boards
        (0..n).map(|i| (h.sample(from + Time::from_us(2 * i as u64), 0) - 1.65).abs()).fold(0.0, f64::max)
    }

    #[test]
    fn tone_level_matches_the_link_budget() {
        // 160 dB re 1 uPa at 1 m, 10 m away: 140 dB = 1e7 uPa rms; -190 dB re 1 V/uPa: 3.16 mV rms; 20 dB gain
        let env = tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]);
        let chain = ChainSpec { gain_db: 20.0, bandpass_hz: Some([20_000.0, 50_000.0]), ..Default::default() };
        let mut h = Hydrophone::new(env, [0.0, 0.0, 1.0], Transducer::hydrophone(-190.0), chain);
        let expected = h.direct_peak_v(0, 0);
        let rms_upa = 1e7;
        let budget = rms_upa * 2f64.sqrt() * 10f64.powf(-9.5) * h.chain.gain_at(30_000.0, 0);
        assert!((expected / budget - 1.0).abs() < 1e-3, "{expected} {budget}");
        let p = peak(&mut h, Time::from_ms(10), 2000);
        assert!((p / expected - 1.0).abs() < 0.03, "{p} vs {expected}");
    }

    #[test]
    fn amplitude_halves_with_doubled_distance_and_gaps_are_handled() {
        let chain = ChainSpec { gain_db: 20.0, bandpass_hz: Some([20_000.0, 50_000.0]), ..Default::default() };
        let mut near = Hydrophone::new(
            tone_env(30_000.0, 160.0, [5.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            chain.clone(),
        );
        let mut far = Hydrophone::new(
            tone_env(30_000.0, 160.0, [10.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0],
            Transducer::hydrophone(-190.0),
            chain,
        );
        let a = peak(&mut near, Time::from_ms(20), 1000);
        let b = peak(&mut far, Time::from_ms(20), 1000);
        assert!((a / b - 2.0).abs() < 0.05, "{a} {b}");
        // A long gap, then sampling again: the filter restarts with preroll and reads the same level
        let c = peak(&mut far, Time::from_secs(3), 1000);
        assert!((c / b - 1.0).abs() < 0.03, "{c} {b}");
    }

    #[test]
    fn noise_is_reproducible_and_scaled() {
        let mut env = Environment::open_water(Water::default());
        env.noise = crate::noise::Noise::White { level_db_re_upa_per_hz: 60.0 };
        let env = Arc::new(env);
        let chain = ChainSpec { gain_db: 40.0, ..Default::default() };
        let mut a =
            Hydrophone::new(env.clone(), [0.0, 0.0, 1.0], Transducer::hydrophone(-180.0), chain.clone()).record();
        let mut b = Hydrophone::new(env.clone(), [0.0, 0.0, 1.0], Transducer::hydrophone(-180.0), chain.clone());
        let xs: Vec<f64> = (0..5000).map(|i| a.sample(Time::from_us(i), 0) - 1.65).collect();
        let ys: Vec<f64> = (0..5000).map(|i| b.sample(Time::from_us(i), 0) - 1.65).collect();
        assert_eq!(xs, ys);
        // 60 dB/Hz over 500 kHz: 1e3 uPa * 707 = 7.07e5 uPa rms; -180 dB: 7.07e-4 V; 40 dB: 70.7 mV rms
        let rms = (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt();
        assert!((rms - 0.0707).abs() < 0.005, "{rms}");
        let wav = a.recording_wav(1_000_000).unwrap();
        assert_eq!(wav.samples.len(), 5000);
    }
}
