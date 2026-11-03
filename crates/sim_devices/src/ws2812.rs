//! WS2812/WS2812B LED strip, decoded from its data pin.
//!
//! Each bit is a high pulse: short (T0H, 0.4 µs nominal) for 0, long (T1H, 0.8 µs) for 1; a pulse of
//! [`Ws2812::threshold`] or more counts as 1. Every 24 bits is one LED, sent as green, red, blue, MSB first. A low
//! gap of at least [`Ws2812::reset`] (50 µs, the WS2812's reset time; the WS2812B needs 280 µs) latches the frame:
//! the LEDs show it, it is added to `frames` and reported to the trace. Pulses outside the datasheet's windows are
//! counted in `timing_errors`.

use sim_core::time::Time;

use crate::DeviceCtx;
use crate::pin::PinDevice;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Clone, Debug)]
pub struct Ws2812 {
    pub name: String,
    pub pin: u8,
    pub leds: usize,
    pub threshold: Time,
    pub reset: Time,
    /// What the strip shows.
    pub shown: Vec<Rgb>,
    /// Latched frames (time, colours), the most recent last; at most `keep_frames` are kept.
    pub frames: Vec<(Time, Vec<Rgb>)>,
    pub keep_frames: usize,
    pub timing_errors: u32,
    bits: Vec<bool>,
    rise: Option<Time>,
    fall: Option<Time>,
    level: bool,
}

impl Ws2812 {
    pub fn new(name: &str, pin: u8, leds: usize) -> Self {
        Ws2812 {
            name: name.into(),
            pin,
            leds,
            threshold: Time::from_ns(550),
            reset: Time::from_us(50),
            shown: vec![Rgb::default(); leds],
            frames: Vec::new(),
            keep_frames: 1000,
            timing_errors: 0,
            bits: Vec::new(),
            rise: None,
            fall: None,
            level: false,
        }
    }

    fn latch(&mut self, ctx: &mut dyn DeviceCtx) {
        if self.bits.is_empty() {
            return;
        }
        let bits = std::mem::take(&mut self.bits);
        let byte = |c: &[bool]| c.iter().fold(0u8, |a, &b| a << 1 | b as u8);
        let colours: Vec<Rgb> =
            bits.chunks_exact(24).map(|c| Rgb { g: byte(&c[0..8]), r: byte(&c[8..16]), b: byte(&c[16..24]) }).collect();
        if !bits.len().is_multiple_of(24) {
            self.timing_errors += 1;
        }
        // Each LED takes the first 24 bits and passes the rest on
        for (i, c) in colours.iter().take(self.leds).enumerate() {
            self.shown[i] = *c;
        }
        let text: Vec<String> = colours.iter().map(|c| format!("{:02x}{:02x}{:02x}", c.r, c.g, c.b)).collect();
        ctx.report(format!("{} (GPIO {}): {} LEDs: {}", self.name, self.pin, colours.len(), text.join(" ")));
        self.frames.push((ctx.now(), colours));
        if self.frames.len() > self.keep_frames {
            self.frames.remove(0);
        }
    }

    /// The most recent frame.
    pub fn last_frame(&self) -> Option<&[Rgb]> {
        self.frames.last().map(|f| f.1.as_slice())
    }
}

impl PinDevice for Ws2812 {
    fn name(&self) -> String {
        self.name.clone()
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        let now = ctx.now();
        if level == self.level {
            return;
        }
        self.level = level;
        if level {
            if let Some(f) = self.fall
                && now - f >= self.reset
            {
                self.latch(ctx);
            }
            self.rise = Some(now);
        } else {
            if let Some(r) = self.rise.take() {
                let w = now - r;
                let one = w >= self.threshold;
                // Datasheet windows, generously: 0.2-0.55 us for 0, 0.55-1.0 us for 1
                if w < Time::from_ns(200) || w > Time::from_ns(1000) {
                    self.timing_errors += 1;
                }
                self.bits.push(one);
            }
            self.fall = Some(now);
            ctx.wake_at(now + self.reset);
        }
    }

    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        if !self.level
            && let Some(f) = self.fall
            && ctx.now() - f >= self.reset
        {
            self.latch(ctx);
        }
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    fn send(d: &mut Ws2812, ctx: &mut TestCtx, colours: &[(u8, u8, u8)]) {
        for &(r, g, b) in colours {
            let word = (g as u32) << 16 | (r as u32) << 8 | b as u32;
            for i in (0..24).rev() {
                let one = word >> i & 1 != 0;
                d.pin_changed(ctx, 0, true);
                ctx.now += Time::from_ns(if one { 800 } else { 400 });
                d.pin_changed(ctx, 0, false);
                ctx.now += Time::from_ns(if one { 450 } else { 850 });
            }
        }
    }

    #[test]
    fn decodes_and_latches_grb() {
        let mut ctx = TestCtx::default();
        let mut d = Ws2812::new("strip", 0, 2);
        send(&mut d, &mut ctx, &[(255, 10, 0), (1, 2, 3), (9, 9, 9)]);
        assert!(d.frames.is_empty());
        ctx.now += Time::from_us(60);
        d.update(&mut ctx);
        assert_eq!(
            d.last_frame().unwrap(),
            &[Rgb { r: 255, g: 10, b: 0 }, Rgb { r: 1, g: 2, b: 3 }, Rgb { r: 9, g: 9, b: 9 }]
        );
        assert_eq!(d.shown, vec![Rgb { r: 255, g: 10, b: 0 }, Rgb { r: 1, g: 2, b: 3 }], "two LEDs on the strip");
        assert!(ctx.reports[0].contains("ff0a00 010203"), "{:?}", ctx.reports);
        assert_eq!(d.timing_errors, 0);
        // A gap shorter than the reset time does not latch
        send(&mut d, &mut ctx, &[(0, 0, 1)]);
        ctx.now += Time::from_us(20);
        d.update(&mut ctx);
        assert_eq!(d.frames.len(), 1);
    }
}
