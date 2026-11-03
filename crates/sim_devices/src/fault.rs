//! Fault models for the parts around the SoC: an I2C device that stops answering, freezes or glitches, and an
//! ADC input that is shorted, open or noisy. CAN bus faults (bit errors, a stuck bus, an unplugged node, a flood)
//! live with the bus in [`crate::can`], and the controller's reaction to them in [`crate::mcp251xfd`].
//!
//! The wrappers sit between the SoC and the real model: [`FaultyI2c`] around any [`I2cDevice`], [`FaultyAnalog`]
//! around any [`AnalogSource`]. With no fault set they pass everything through unchanged, so a device can be
//! wrapped once and broken and repaired as often as a test likes. Randomness comes from a seeded [`Rng`], so runs
//! (and snapshots, which copy its state) stay reproducible.

use std::collections::HashMap;

use sim_core::analog::AnalogSource;
use sim_core::time::Time;

use crate::DeviceCtx;
use crate::i2c::I2cDevice;

/// A small deterministic random number generator (SplitMix64): fault timing and glitches must repeat exactly from
/// a seed, and snapshots copy it with the rest of the state.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }

    /// A seed derived from a name, so differently named buses or devices get different streams.
    pub fn from_name(name: &str) -> Self {
        Rng(name.bytes().fold(0xCBF2_9CE4_8422_2325u64, |h, b| (h ^ b as u64).wrapping_mul(0x0100_0000_01B3)))
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// True with probability `p`.
    pub fn chance(&mut self, p: f64) -> bool {
        p > 0.0 && self.uniform() < p
    }

    /// Standard normal (Box-Muller).
    pub fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(f64::MIN_POSITIVE);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

// ---------------------------------------------------------------------------------------------------- I2C

/// What is wrong with an I2C device.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum I2cFault {
    /// The device does not answer: its address is not acknowledged (unpowered, unplugged, dead). The controller
    /// sees an address NAK (`IC_TX_ABRT_SOURCE.ABRT_7B_ADDR_NOACK` on the RP2040).
    Absent,
    /// Answers only some transfers: each START is not acknowledged with this probability (a marginal connector,
    /// a device browning out).
    Intermittent(f64),
    /// Answers, but its readings are frozen: each read returns the bytes the same command sequence returned the
    /// last time before the fault (a hung ADC, a register that stopped updating). Checksums stay valid, so
    /// firmware can only notice that the value stopped changing.
    Stuck,
    /// Each byte read has one random bit flipped with this probability (noise on SDA). Sensors with a CRC
    /// (SHT41) fail their check; those without (MS5837 ADC reads) return wrong values.
    Glitch(f64),
    /// The device holds SCL low for this long before every byte (a device hung mid-transfer, or one that is
    /// extremely slow); the controller waits it out unless it has a timeout of its own.
    Stretch(Time),
}

/// An [`I2cDevice`] with a fault injector in front. `as_any` still reaches the wrapped model, so code that looks
/// devices up by model type keeps working; [`I2cDevice::faulty`] reaches the wrapper.
#[derive(Clone)]
pub struct FaultyI2c {
    pub inner: Box<dyn I2cDevice>,
    pub fault: Option<I2cFault>,
    rng: Rng,
    /// Bytes written since the last read: the "command" the next read answers.
    command: Vec<u8>,
    /// What the last read after each command returned (for [`I2cFault::Stuck`]).
    history: HashMap<Vec<u8>, Vec<u8>>,
    reading: Option<(Vec<u8>, Vec<u8>)>,
    /// Whether the current transfer was refused at its START.
    refused: bool,
    /// Transfers refused, reads replayed and bytes corrupted, for tests.
    pub naks: u64,
    pub replayed: u64,
    pub glitched: u64,
}

impl FaultyI2c {
    pub fn new(inner: Box<dyn I2cDevice>) -> Self {
        let rng = Rng::from_name(&inner.name());
        FaultyI2c {
            inner,
            fault: None,
            rng,
            command: Vec::new(),
            history: HashMap::new(),
            reading: None,
            refused: false,
            naks: 0,
            replayed: 0,
            glitched: 0,
        }
    }

    fn finish_read(&mut self) {
        // While stuck, only a command not seen before is recorded: its first answer is the one that freezes
        if let Some((cmd, bytes)) = self.reading.take()
            && (self.fault != Some(I2cFault::Stuck) || !self.history.contains_key(&cmd))
        {
            self.history.insert(cmd, bytes);
        }
    }
}

impl I2cDevice for FaultyI2c {
    fn name(&self) -> String {
        self.inner.name()
    }

    fn responds_to(&self, address: u8) -> bool {
        self.inner.responds_to(address)
    }

    fn start(&mut self, ctx: &mut dyn DeviceCtx, address: u8, read: bool) -> bool {
        self.finish_read();
        let refuse = match self.fault {
            Some(I2cFault::Absent) => true,
            Some(I2cFault::Intermittent(p)) => self.rng.chance(p),
            _ => false,
        };
        self.refused = refuse;
        if refuse {
            self.naks += 1;
            return false;
        }
        let ack = self.inner.start(ctx, address, read);
        if ack && read {
            self.reading = Some((std::mem::take(&mut self.command), Vec::new()));
        }
        ack
    }

    fn write(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        if self.refused {
            return false;
        }
        self.command.push(byte);
        // Commands are short; a long write (a display frame) is not a command a read answers
        if self.command.len() > 16 {
            self.command.drain(..self.command.len() - 16);
        }
        self.inner.write(ctx, byte)
    }

    fn read(&mut self, ctx: &mut dyn DeviceCtx, ack: bool) -> u8 {
        if self.refused {
            return 0xFF; // nobody drives SDA: the pull-up reads as ones
        }
        // The real device always sees the read, so its state (pointers, conversions) moves on as on hardware
        let mut b = self.inner.read(ctx, ack);
        let Some((cmd, bytes)) = &mut self.reading else { return b };
        let i = bytes.len();
        bytes.push(b);
        match self.fault {
            Some(I2cFault::Stuck) => {
                if let Some(&old) = self.history.get(cmd.as_slice()).and_then(|h| h.get(i)) {
                    b = old;
                    if i == 0 {
                        self.replayed += 1;
                    }
                }
            }
            Some(I2cFault::Glitch(p)) if self.rng.chance(p) => {
                b ^= 1 << (self.rng.next_u64() % 8);
                self.glitched += 1;
            }
            _ => {}
        }
        b
    }

    fn stop(&mut self, ctx: &mut dyn DeviceCtx) {
        self.finish_read();
        if std::mem::take(&mut self.refused) {
            return;
        }
        self.inner.stop(ctx);
    }

    fn stretch(&mut self, ctx: &mut dyn DeviceCtx) -> Time {
        let own = self.inner.stretch(ctx);
        match self.fault {
            Some(I2cFault::Stretch(t)) if !self.refused => own.max(t),
            _ => own,
        }
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        self.inner.update(ctx);
    }

    fn clone_box(&self) -> Box<dyn I2cDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self.inner.as_any()
    }

    fn faulty(&mut self) -> Option<&mut FaultyI2c> {
        Some(self)
    }

    fn relink_inputs(&mut self, map: &sim_core::analog::Relink) {
        self.inner.relink_inputs(map);
    }
}

// ---------------------------------------------------------------------------------------------------- analog

/// What is wrong with an ADC input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalogFault {
    /// Shorted to a fixed voltage: 0 V for a short to ground, the 3.3 V rail for a short to it. A short through
    /// a divider's bottom resistor (the measured wire broken off) also reads 0 V.
    Shorted(f64),
    /// The pin is open (a broken trace or cracked joint between the divider and the pin): nothing drives it, so
    /// the ADC reads what the sample capacitor and leakage leave there, a value that wanders from the last real
    /// one in steps of about `0.01 V` per conversion and stays within the rails.
    Open,
    /// Gaussian noise of this standard deviation (volts) on top of the real value (a bad ground, a noisy rail).
    Noise(f64),
    /// Occasional large spikes: with probability `.0` a conversion reads the real value plus or minus `.1` volts
    /// (ESD or switching transients coupling in).
    Spikes(f64, f64),
}

/// An [`AnalogSource`] with a fault injector in front.
#[derive(Clone)]
pub struct FaultyAnalog {
    pub inner: Box<dyn AnalogSource>,
    pub fault: Option<AnalogFault>,
    /// The ADC's reference, for keeping a floating input within the rails.
    pub vref: f64,
    rng: Rng,
    /// Last value read (NaN before the first), where an open input starts wandering from.
    last: f64,
}

impl FaultyAnalog {
    pub fn new(inner: Box<dyn AnalogSource>, seed: u64) -> Self {
        FaultyAnalog { inner, fault: None, vref: 3.3, rng: Rng::new(seed), last: f64::NAN }
    }
}

impl AnalogSource for FaultyAnalog {
    fn voltage(&mut self, t: Time, pins: u32) -> f64 {
        // The real source is always evaluated, so stateful sources (a mux, a hydrophone) keep their timing
        let real = self.inner.voltage(t, pins);
        let v = match self.fault {
            None => real,
            Some(AnalogFault::Shorted(v)) => v,
            Some(AnalogFault::Open) => {
                let from = if self.last.is_nan() { real } else { self.last };
                (from + 0.01 * self.rng.normal()).clamp(0.0, self.vref)
            }
            Some(AnalogFault::Noise(sigma)) => real + sigma * self.rng.normal(),
            Some(AnalogFault::Spikes(p, size)) => {
                if self.rng.chance(p) {
                    real + if self.rng.chance(0.5) { size } else { -size }
                } else {
                    real
                }
            }
        };
        self.last = v;
        v
    }

    fn clone_box(&self) -> Box<dyn AnalogSource> {
        Box::new(self.clone())
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn relink_inputs(&mut self, map: &sim_core::analog::Relink) {
        self.inner.relink_inputs(map);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    /// A device that answers each read with a counter, so frozen and fresh readings can be told apart.
    #[derive(Clone)]
    struct Counter {
        n: u8,
        stops: u32,
    }

    impl I2cDevice for Counter {
        fn name(&self) -> String {
            "counter".into()
        }
        fn responds_to(&self, address: u8) -> bool {
            address == 0x40
        }
        fn start(&mut self, _: &mut dyn DeviceCtx, _: u8, _: bool) -> bool {
            true
        }
        fn write(&mut self, _: &mut dyn DeviceCtx, _: u8) -> bool {
            true
        }
        fn read(&mut self, _: &mut dyn DeviceCtx, _: bool) -> u8 {
            self.n = self.n.wrapping_add(1);
            self.n
        }
        fn stop(&mut self, _: &mut dyn DeviceCtx) {
            self.stops += 1;
        }
        fn clone_box(&self) -> Box<dyn I2cDevice> {
            Box::new(self.clone())
        }
        fn as_any(&self) -> &dyn std::any::Any {
            self
        }
    }

    /// Writes `cmd`, then reads `n` bytes, as two transfers.
    fn transact(d: &mut FaultyI2c, c: &mut TestCtx, cmd: u8, n: usize) -> Option<Vec<u8>> {
        if !d.start(c, 0x40, false) {
            d.stop(c);
            return None;
        }
        d.write(c, cmd);
        d.stop(c);
        if !d.start(c, 0x40, true) {
            d.stop(c);
            return None;
        }
        let out = (0..n).map(|i| d.read(c, i + 1 < n)).collect();
        d.stop(c);
        Some(out)
    }

    fn faulty() -> (FaultyI2c, TestCtx) {
        (FaultyI2c::new(Box::new(Counter { n: 0, stops: 0 })), TestCtx::default())
    }

    #[test]
    fn passes_through_without_a_fault() {
        let (mut d, mut c) = faulty();
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![1, 2]));
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![3, 4]));
        assert!(d.as_any().downcast_ref::<Counter>().is_some(), "as_any reaches the model");
        assert!(d.faulty().is_some());
    }

    #[test]
    fn absent_device_naks_its_address_and_comes_back() {
        let (mut d, mut c) = faulty();
        d.fault = Some(I2cFault::Absent);
        assert_eq!(transact(&mut d, &mut c, 1, 2), None);
        assert_eq!(d.naks, 1);
        assert_eq!(d.as_any().downcast_ref::<Counter>().unwrap().stops, 0, "the model never saw the transfer");
        d.fault = None;
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![1, 2]));
    }

    #[test]
    fn stuck_device_replays_its_last_answer_per_command() {
        let (mut d, mut c) = faulty();
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![1, 2]));
        assert_eq!(transact(&mut d, &mut c, 2, 2), Some(vec![3, 4]));
        d.fault = Some(I2cFault::Stuck);
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![1, 2]));
        assert_eq!(transact(&mut d, &mut c, 2, 2), Some(vec![3, 4]));
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![1, 2]));
        // A command never seen before the fault is answered live, and then stays frozen too
        assert_eq!(transact(&mut d, &mut c, 3, 1), Some(vec![11]));
        assert_eq!(d.replayed, 3);
        d.fault = None;
        assert_eq!(transact(&mut d, &mut c, 1, 2), Some(vec![12, 13]), "live again; the model kept counting");
    }

    #[test]
    fn glitches_flip_single_bits() {
        let (mut d, mut c) = faulty();
        d.fault = Some(I2cFault::Glitch(1.0));
        let got = transact(&mut d, &mut c, 1, 4).unwrap();
        for (i, b) in got.iter().enumerate() {
            assert_eq!((b ^ (i as u8 + 1)).count_ones(), 1, "{got:?}");
        }
        assert_eq!(d.glitched, 4);
    }

    #[test]
    fn intermittent_device_refuses_some_transfers() {
        let (mut d, mut c) = faulty();
        d.fault = Some(I2cFault::Intermittent(0.5));
        let ok = (0..200).filter(|_| transact(&mut d, &mut c, 1, 1).is_some()).count();
        assert!((20..=80).contains(&ok), "about a quarter of the two-transfer exchanges get through: {ok}");
    }

    #[test]
    fn stretching_device_holds_the_clock() {
        let (mut d, mut c) = faulty();
        assert_eq!(d.stretch(&mut c), Time::ZERO);
        d.fault = Some(I2cFault::Stretch(Time::from_ms(5)));
        assert_eq!(d.stretch(&mut c), Time::from_ms(5));
    }

    #[test]
    fn analog_faults() {
        let mut a = FaultyAnalog::new(Box::new(sim_core::analog::Signal::Const(1.2)), 7);
        assert_eq!(a.voltage(Time::ZERO, 0), 1.2);
        a.fault = Some(AnalogFault::Shorted(0.0));
        assert_eq!(a.voltage(Time::ZERO, 0), 0.0);
        let mut a = FaultyAnalog::new(Box::new(sim_core::analog::Signal::Const(1.2)), 7);
        a.fault = Some(AnalogFault::Open);
        let open: Vec<f64> = (0..1000).map(|_| a.voltage(Time::ZERO, 0)).collect();
        assert!(open.iter().all(|v| (0.0..=3.3).contains(v)));
        assert!((open[0] - 1.2).abs() < 0.05, "starts where the input was");
        assert!(open.windows(2).all(|w| (w[1] - w[0]).abs() < 0.06), "wanders in small steps");
        assert!(open.iter().any(|v| (v - 1.2).abs() > 0.05), "but does wander");
        a.fault = Some(AnalogFault::Noise(0.1));
        let n: Vec<f64> = (0..4000).map(|_| a.voltage(Time::ZERO, 0)).collect();
        let mean = n.iter().sum::<f64>() / n.len() as f64;
        let sd = (n.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n.len() as f64).sqrt();
        assert!((mean - 1.2).abs() < 0.01 && (sd - 0.1).abs() < 0.01, "{mean} {sd}");
        a.fault = Some(AnalogFault::Spikes(0.1, 1.0));
        let s: Vec<f64> = (0..1000).map(|_| a.voltage(Time::ZERO, 0)).collect();
        let spikes = s.iter().filter(|v| (**v - 1.2).abs() > 0.5).count();
        assert!((50..=150).contains(&spikes), "{spikes}");
        assert!(s.iter().all(|v| *v == 1.2 || ((*v - 1.2).abs() - 1.0).abs() < 1e-9));
    }

    #[test]
    fn rng_is_deterministic() {
        let mut a = Rng::new(3);
        let mut b = Rng::new(3);
        assert_eq!(
            (0..10).map(|_| a.next_u64()).collect::<Vec<_>>(),
            (0..10).map(|_| b.next_u64()).collect::<Vec<_>>()
        );
        assert_ne!(Rng::from_name("can0").next_u64(), Rng::from_name("can1").next_u64());
    }
}
