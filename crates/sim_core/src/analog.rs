//! Scripted physical quantities and analog voltages.
//!
//! A [`Signal`] is a value over virtual time: a constant, a script of points (stepped or linearly interpolated), a
//! sine, a function, or a value a test changes while the machine runs ([`Signal::shared`]). Sensor models take their
//! physical inputs (temperature, pressure, battery voltage) as signals, and ADC inputs are [`AnalogSource`]s, most
//! often a signal in volts.

use std::fmt;
use std::sync::{Arc, Mutex};

use crate::time::Time;

/// A value over virtual time.
#[derive(Clone)]
pub enum Signal {
    Const(f64),
    /// Points sorted by time. Before the first point the first value holds, after the last the last.
    Script {
        points: Vec<(Time, f64)>,
        interpolate: bool,
    },
    /// `offset + amplitude * sin(2 pi f t + phase)`.
    Sine {
        offset: f64,
        amplitude: f64,
        hz: f64,
        phase: f64,
    },
    /// A value a test or host tool sets while the machine runs.
    Shared(Arc<Mutex<f64>>),
    Fn(Arc<dyn Fn(Time) -> f64 + Send + Sync>),
}

impl Signal {
    /// A stepped script: each value holds from its time until the next.
    pub fn steps(points: &[(Time, f64)]) -> Signal {
        let mut points = points.to_vec();
        points.sort_by_key(|p| p.0);
        Signal::Script { points, interpolate: false }
    }

    /// A linearly interpolated script.
    pub fn ramp(points: &[(Time, f64)]) -> Signal {
        let mut points = points.to_vec();
        points.sort_by_key(|p| p.0);
        Signal::Script { points, interpolate: true }
    }

    /// A signal and the handle to change it with.
    pub fn shared(initial: f64) -> (Signal, Arc<Mutex<f64>>) {
        let h = Arc::new(Mutex::new(initial));
        (Signal::Shared(h.clone()), h)
    }

    pub fn func(f: impl Fn(Time) -> f64 + Send + Sync + 'static) -> Signal {
        Signal::Fn(Arc::new(f))
    }

    pub fn at(&self, t: Time) -> f64 {
        match self {
            Signal::Const(v) => *v,
            Signal::Script { points, interpolate } => {
                let Some(first) = points.first() else { return 0.0 };
                let i = points.partition_point(|p| p.0 <= t);
                if i == 0 {
                    return first.1;
                }
                let (t0, v0) = points[i - 1];
                match points.get(i) {
                    Some(&(t1, v1)) if *interpolate => {
                        let f = (t - t0).as_ps() as f64 / (t1 - t0).as_ps() as f64;
                        v0 + (v1 - v0) * f
                    }
                    _ => v0,
                }
            }
            Signal::Sine { offset, amplitude, hz, phase } => {
                offset + amplitude * (2.0 * std::f64::consts::PI * hz * t.as_secs_f64() + phase).sin()
            }
            Signal::Shared(v) => *v.lock().unwrap(),
            Signal::Fn(f) => f(t),
        }
    }
}

impl From<f64> for Signal {
    fn from(v: f64) -> Self {
        Signal::Const(v)
    }
}

impl fmt::Debug for Signal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Signal::Const(v) => write!(f, "Const({v})"),
            Signal::Script { points, interpolate } => {
                write!(f, "Script({} points, interpolate {interpolate})", points.len())
            }
            Signal::Sine { offset, amplitude, hz, .. } => write!(f, "Sine({offset} + {amplitude} @ {hz} Hz)"),
            Signal::Shared(v) => write!(f, "Shared({})", v.lock().unwrap()),
            Signal::Fn(_) => write!(f, "Fn"),
        }
    }
}

/// Something that puts a voltage on an analog input.
pub trait AnalogSource: Send {
    /// Voltage at time `t`. `pins` holds the SoC's resolved GPIO levels (for analog muxes selected by GPIOs).
    fn voltage(&mut self, t: Time, pins: u32) -> f64;
    fn clone_box(&self) -> Box<dyn AnalogSource>;
    /// For finding a source of a particular type again (e.g. to re-link a snapshot's copy).
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
}

crate::clone_box!(AnalogSource);

impl AnalogSource for Signal {
    fn voltage(&mut self, t: Time, _pins: u32) -> f64 {
        self.at(t)
    }
    fn clone_box(&self) -> Box<dyn AnalogSource> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scripts_step_and_ramp() {
        let pts = [(Time::from_ms(10), 1.0), (Time::from_ms(20), 3.0)];
        let s = Signal::steps(&pts);
        assert_eq!(s.at(Time::ZERO), 1.0);
        assert_eq!(s.at(Time::from_ms(15)), 1.0);
        assert_eq!(s.at(Time::from_ms(20)), 3.0);
        assert_eq!(s.at(Time::from_ms(99)), 3.0);
        let r = Signal::ramp(&pts);
        assert_eq!(r.at(Time::from_ms(15)), 2.0);
        assert_eq!(r.at(Time::from_ms(5)), 1.0);
        assert_eq!(Signal::steps(&[]).at(Time::ZERO), 0.0);
    }

    #[test]
    fn sine_shared_and_fn() {
        let s = Signal::Sine { offset: 1.0, amplitude: 0.5, hz: 1000.0, phase: 0.0 };
        assert!((s.at(Time::from_us(250)) - 1.5).abs() < 1e-9);
        let (s, h) = Signal::shared(2.0);
        assert_eq!(s.at(Time::ZERO), 2.0);
        *h.lock().unwrap() = 4.0;
        assert_eq!(s.at(Time::ZERO), 4.0);
        let f = Signal::func(|t| t.as_secs_f64() * 2.0);
        assert_eq!(f.at(Time::from_ms(500)), 1.0);
        let mut src: Box<dyn AnalogSource> = Box::new(Signal::Const(1.2));
        assert_eq!(src.voltage(Time::ZERO, 0), 1.2);
    }
}
