//! Minimal WAV (RIFF PCM) reading and writing: 8/16/24/32-bit integer and 32-bit float, first channel only.

use std::io;
use std::path::Path;

/// Samples (-1..1) and their rate.
#[derive(Clone, Debug, PartialEq)]
pub struct Wav {
    pub rate: u32,
    pub samples: Vec<f32>,
}

fn u16_at(b: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([b[i], b[i + 1]])
}

fn u32_at(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]])
}

impl Wav {
    pub fn parse(b: &[u8]) -> Result<Wav, String> {
        if b.len() < 12 || &b[0..4] != b"RIFF" || &b[8..12] != b"WAVE" {
            return Err("not a RIFF/WAVE file".into());
        }
        let (mut fmt, mut data) = (None, None);
        let mut i = 12;
        while i + 8 <= b.len() {
            let len = u32_at(b, i + 4) as usize;
            let body = &b[i + 8..(i + 8 + len).min(b.len())];
            match &b[i..i + 4] {
                b"fmt " => fmt = Some(body),
                b"data" => data = Some(body),
                _ => {}
            }
            i += 8 + len + (len & 1);
        }
        let fmt = fmt.ok_or("no fmt chunk")?;
        let data = data.ok_or("no data chunk")?;
        if fmt.len() < 16 {
            return Err("short fmt chunk".into());
        }
        let format = u16_at(fmt, 0);
        let channels = u16_at(fmt, 2).max(1) as usize;
        let rate = u32_at(fmt, 4);
        let bits = u16_at(fmt, 14) as usize;
        let bytes = bits / 8;
        let frame = bytes * channels;
        if frame == 0 {
            return Err("bad sample size".into());
        }
        let float = format == 3 || (format == 0xFFFE && bits == 32 && fmt.len() >= 26 && u16_at(fmt, 24) == 3);
        let samples = data
            .chunks_exact(frame)
            .map(|f| match (bytes, float) {
                (1, _) => (f[0] as f32 - 128.0) / 128.0,
                (2, _) => i16::from_le_bytes([f[0], f[1]]) as f32 / 32768.0,
                (3, _) => (i32::from_le_bytes([0, f[0], f[1], f[2]]) >> 8) as f32 / 8_388_608.0,
                (4, true) => f32::from_le_bytes([f[0], f[1], f[2], f[3]]),
                (4, false) => i32::from_le_bytes([f[0], f[1], f[2], f[3]]) as f32 / 2_147_483_648.0,
                _ => 0.0,
            })
            .collect();
        Ok(Wav { rate, samples })
    }

    pub fn load(path: &Path) -> Result<Wav, String> {
        let b = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&b).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// 16-bit mono PCM.
    pub fn to_bytes(&self) -> Vec<u8> {
        let n = self.samples.len() * 2;
        let mut out = Vec::with_capacity(44 + n);
        out.extend_from_slice(b"RIFF");
        out.extend_from_slice(&(36 + n as u32).to_le_bytes());
        out.extend_from_slice(b"WAVEfmt ");
        out.extend_from_slice(&16u32.to_le_bytes());
        out.extend_from_slice(&1u16.to_le_bytes());
        out.extend_from_slice(&1u16.to_le_bytes());
        out.extend_from_slice(&self.rate.to_le_bytes());
        out.extend_from_slice(&(self.rate * 2).to_le_bytes());
        out.extend_from_slice(&2u16.to_le_bytes());
        out.extend_from_slice(&16u16.to_le_bytes());
        out.extend_from_slice(b"data");
        out.extend_from_slice(&(n as u32).to_le_bytes());
        for s in &self.samples {
            out.extend_from_slice(&((s.clamp(-1.0, 1.0) * 32767.0).round() as i16).to_le_bytes());
        }
        out
    }

    pub fn save(&self, path: &Path) -> io::Result<()> {
        std::fs::write(path, self.to_bytes())
    }

    /// Linear interpolation at `t` seconds; 0 outside the recording.
    pub fn at(&self, t: f64) -> f64 {
        let x = t * self.rate as f64;
        if x < 0.0 {
            return 0.0;
        }
        let i = x.floor() as usize;
        let Some(&a) = self.samples.get(i) else { return 0.0 };
        let b = self.samples.get(i + 1).copied().unwrap_or(0.0);
        a as f64 + (b as f64 - a as f64) * (x - i as f64)
    }

    pub fn duration(&self) -> f64 {
        self.samples.len() as f64 / self.rate.max(1) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_interpolation() {
        let w = Wav { rate: 8000, samples: vec![0.0, 0.5, -0.5, 1.0] };
        let back = Wav::parse(&w.to_bytes()).unwrap();
        assert_eq!(back.rate, 8000);
        for (a, b) in w.samples.iter().zip(&back.samples) {
            assert!((a - b).abs() < 1e-4);
        }
        assert!((back.at(0.5 / 8000.0) - 0.25).abs() < 1e-3);
        assert_eq!(back.at(1.0), 0.0);
        assert!(Wav::parse(b"nope").is_err());
    }
}
