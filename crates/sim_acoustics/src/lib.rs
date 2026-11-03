//! Underwater acoustic environments for testing the IVC acoustic modems: transducers and receive chains described
//! by their specs, a body of water (spreading, absorption, multipath, noise), and other sound sources, coupled to
//! board ADC inputs. Nothing here knows about a board or a modem: a hydrophone is an
//! [`AnalogSource`](sim_core::analog::AnalogSource) for any ADC input.
//!
//! * [`water`]: sound speed and absorption
//! * [`noise`]: ambient noise models and the seeded random numbers behind them
//! * [`transducer`]: projectors and hydrophones by datasheet numbers (TVR, RVS, beam pattern)
//! * [`chain`]: receive electronics (gain, filters with a GPIO select, bias, rails)
//! * [`source`]: pingers, tones, chirps and WAV playback
//! * [`geometry`]: positions, boundaries and the image method
//! * [`env`]: environment files and propagation paths
//! * [`hydrophone`]: a hydrophone in an environment, driving an ADC input
//! * [`wav`]: WAV reading and writing
//!
//! See "Acoustic environments (IVC)" in `PLAN.md`.

pub mod chain;
pub mod env;
pub mod geometry;
pub mod hydrophone;
pub mod noise;
pub mod source;
pub mod transducer;
pub mod water;
pub mod wav;

pub use env::Environment;
pub use hydrophone::Hydrophone;
