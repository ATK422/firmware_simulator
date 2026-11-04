//! Boards in a shared acoustic medium: the pin device that feeds a board's projector.

use sim_acoustics::Hydrophone;
use sim_acoustics::medium::SharedMedium;
use sim_core::analog::AnalogSource;
use sim_devices::DeviceCtx;
use sim_devices::fault::FaultyAnalog;
use sim_devices::pin::PinDevice;

/// Watches a projector's drive pin and records its level changes in the medium.
#[derive(Clone)]
pub struct ProjectorPin {
    medium: SharedMedium,
    id: usize,
    pin: u8,
}

impl ProjectorPin {
    pub fn new(medium: SharedMedium, id: usize, pin: u8) -> Self {
        ProjectorPin { medium, id, pin }
    }
}

impl PinDevice for ProjectorPin {
    fn name(&self) -> String {
        format!("projector (GPIO {})", self.pin)
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        self.medium.lock().unwrap().edge(self.id, ctx.now(), level);
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

/// The hydrophone an ADC input's source is, if it is one, looking through a fault injector wrapped around it (an
/// ADC fault on the hydrophone's input).
fn hydrophone_in(src: &mut dyn AnalogSource) -> Option<&mut Hydrophone> {
    let any = src.as_any_mut()?;
    if any.is::<Hydrophone>() {
        return any.downcast_mut::<Hydrophone>();
    }
    hydrophone_in(&mut *any.downcast_mut::<FaultyAnalog>()?.inner)
}

/// The hydrophones attached to a machine's ADC inputs.
pub fn hydrophones(m: &mut rp2040::Machine) -> impl Iterator<Item = &mut Hydrophone> {
    m.sys.analog_inputs.iter_mut().flatten().filter_map(|h| hydrophone_in(&mut **h))
}

/// A line about each of a machine's hydrophones whose receive chain clipped some of the samples the ADC took (the
/// signal was too loud for the chain's gain: amplitudes the firmware measures then stop following the sound), for
/// the end of a run.
pub fn clipping_notes(m: &mut rp2040::Machine) -> Vec<String> {
    hydrophones(m)
        .filter_map(|h| {
            let (n, clipped) = h.clipping();
            (clipped > 0).then(|| {
                format!(
                    "hydrophone: {:.1} % of the {n} samples the ADC took were clipped at the receive chain's rails (too \
                     loud for its assumed gain; see docs/acoustics.md)",
                    100.0 * clipped as f64 / n as f64
                )
            })
        })
        .collect()
}

/// Starts keeping every ADC sample of a machine's hydrophones; returns how many there are.
pub fn record_hydrophones(m: &mut rp2040::Machine) -> usize {
    let mut n = 0;
    for h in hydrophones(m) {
        h.recording.get_or_insert_with(Vec::new);
        n += 1;
    }
    n
}

/// Writes what a machine's hydrophones recorded ([`record_hydrophones`]) as WAV files: the first to `path`, any
/// further one to `path` with `_2`, `_3`, ... before its extension. -1..1 is the ADC input range (0 to 3.3 V), the
/// samples quantised to the RP2040 ADC's 12 bits, `rate` the WAV's sample rate. Returns a line about each file.
pub fn write_hydrophone_wavs(
    m: &mut rp2040::Machine,
    path: &std::path::Path,
    rate: u32,
) -> std::io::Result<Vec<String>> {
    let mut out = Vec::new();
    for (k, h) in hydrophones(m).enumerate() {
        let p = if k == 0 {
            path.to_path_buf()
        } else {
            let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
            let ext = path.extension().map(|e| format!(".{}", e.to_string_lossy())).unwrap_or_default();
            path.with_file_name(format!("{stem}_{}{ext}", k + 1))
        };
        let n = h.recording.as_ref().map_or(0, Vec::len);
        let first = h.recording.as_ref().and_then(|r| r.first()).map(|f| f.0);
        match h.recording_pin_wav(rate, [0.0, 3.3], 12) {
            Some(w) => {
                w.save(&p)?;
                out.push(format!(
                    "hydrophone: {n} ADC sample(s) in {} ({:.3} s at {rate} Hz from {}, -1..1 is 0..3.3 V at the pin)",
                    p.display(),
                    w.duration(),
                    first.unwrap_or_default()
                ));
            }
            None => out.push(format!("hydrophone: no ADC samples to write to {}", p.display())),
        }
    }
    Ok(out)
}

/// Points a machine's hydrophones and projector pins that use medium `old` at `new` (a copy of it).
pub fn relink(m: &mut rp2040::Machine, old: &SharedMedium, new: &SharedMedium) {
    for h in hydrophones(m) {
        if h.medium().is_some_and(|m| std::sync::Arc::ptr_eq(m, old)) {
            h.relink(new.clone());
        }
    }
    for d in m.sys.pin_devices.iter_mut() {
        if let Some(p) = d.as_any_mut().and_then(|a| a.downcast_mut::<ProjectorPin>())
            && std::sync::Arc::ptr_eq(&p.medium, old)
        {
            p.medium = new.clone();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_acoustics::chain::ChainSpec;
    use sim_acoustics::medium::Medium;
    use sim_acoustics::transducer::Transducer;
    use sim_acoustics::water::Water;

    #[test]
    fn hydrophone_recordings_are_written_as_wav() {
        use sim_acoustics::chain::ChainSpec;
        use sim_acoustics::transducer::Transducer;
        let env = std::sync::Arc::new(sim_acoustics::Environment::open_water(Default::default()));
        let chain = ChainSpec { bias_v: Some(2.475), ..Default::default() };
        let h = Hydrophone::new(env, [0.0; 3], Transducer::hydrophone(-190.0), chain);
        let mut m = rp2040::Machine::new(Default::default());
        m.sys.attach_analog(1, Box::new(h));
        assert_eq!(record_hydrophones(&mut m), 1);
        for k in 0..100u64 {
            m.sys.analog_inputs[1].as_mut().unwrap().voltage(sim_core::time::Time::from_us(2 * k), 0);
        }
        let dir = std::env::temp_dir().join(format!("hydro-wav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let lines = write_hydrophone_wavs(&mut m, &dir.join("h.wav"), 500_000).unwrap();
        assert!(lines[0].contains("100 ADC sample(s)"), "{lines:?}");
        let w = sim_acoustics::wav::Wav::load(&dir.join("h.wav")).unwrap();
        assert_eq!((w.rate, w.samples.len()), (500_000, 100));
        // A quiet pin at 2.475 V is half way up the upper half of 0..3.3 V (to the nearest 12-bit step)
        assert!(w.samples.iter().all(|s| (s - 0.5).abs() < 5e-4), "{:?}", &w.samples[..4]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn hydrophones_are_found_behind_a_fault_injector() {
        let medium = Medium::shared(std::sync::Arc::new(sim_acoustics::Environment::open_water(Water::default())));
        let env = medium.lock().unwrap().env.clone();
        let h = Hydrophone::new(env, [0.0; 3], Transducer::hydrophone(-190.0), ChainSpec::default())
            .in_medium(medium.clone(), 0);
        let mut m = rp2040::Machine::new(Default::default());
        m.sys.attach_analog(0, Box::new(FaultyAnalog::new(Box::new(h), 1)));
        assert_eq!(hydrophones(&mut m).count(), 1);
        // A snapshot's copy of the machine is moved to the copy of the medium
        let copy = std::sync::Arc::new(std::sync::Mutex::new(medium.lock().unwrap().clone()));
        relink(&mut m, &medium, &copy);
        assert!(std::sync::Arc::ptr_eq(hydrophones(&mut m).next().unwrap().medium().unwrap(), &copy));
    }

    #[test]
    fn clipped_samples_are_reported() {
        use sim_acoustics::source::{Source, Waveform};
        // A 30 kHz tone at 150 dB re 1 uPa, 2 m away: 1 V peak at the pin with 40 dB of gain, inside the rails;
        // 20 dB louder it clips
        let mut notes = Vec::new();
        for level in [150.0, 170.0] {
            let mut env = sim_acoustics::Environment::open_water(Water::default());
            env.sources.push(Source {
                position: [2.0, 0.0, 0.0],
                waveform: Waveform::Tone { hz: 30_000.0 },
                rms_upa: 10f64.powf(level / 20.0),
            });
            let chain = ChainSpec { gain_db: 40.0, ..Default::default() };
            let h = Hydrophone::new(std::sync::Arc::new(env), [0.0; 3], Transducer::hydrophone(-190.0), chain);
            let mut m = rp2040::Machine::new(Default::default());
            m.sys.attach_analog(0, Box::new(h));
            for k in 0..1000u64 {
                m.sys.analog_inputs[0].as_mut().unwrap().voltage(sim_core::time::Time::from_us(k * 2), 0);
            }
            notes.push(clipping_notes(&mut m));
        }
        assert!(notes[0].is_empty(), "{:?}", notes[0]);
        assert_eq!(notes[1].len(), 1);
        assert!(notes[1][0].contains("% of the 1000 samples the ADC took were clipped"), "{:?}", notes[1]);
    }
}
