//! User bank GPIO: IO_BANK0 function select, overrides and interrupts, and PADS_BANK0 (datasheet 2.19).
//!
//! Each pin's level is resolved from the selected function's output and output enable (after the CTRL
//! overrides and the pad's output disable), the pad pulls and whatever is connected outside the chip. Input
//! levels feed SIO `GPIO_IN`, the peripherals and the interrupt logic. Edges are latched in `INTR` whenever a
//! resolved input level changes.

use sim_core::bus::WriteMode;
use sim_core::wire::{External, Level, resolve};

use crate::memmap::NUM_GPIOS;

/// GPIO function numbers (FUNCSEL).
pub mod func {
    pub const XIP: u32 = 0;
    pub const SPI: u32 = 1;
    pub const UART: u32 = 2;
    pub const I2C: u32 = 3;
    pub const PWM: u32 = 4;
    pub const SIO: u32 = 5;
    pub const PIO0: u32 = 6;
    pub const PIO1: u32 = 7;
    pub const CLOCK: u32 = 8;
    pub const USB: u32 = 9;
    pub const NULL: u32 = 0x1F;
}

const PAD_PDE: u32 = 1 << 2;
const PAD_PUE: u32 = 1 << 3;
const PAD_IE: u32 = 1 << 6;
const PAD_OD: u32 = 1 << 7;
const PAD_RESET: u32 = 0x56;

/// What a peripheral function drives on a pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Drive {
    pub out: bool,
    pub oe: bool,
}

#[derive(Clone, Debug)]
pub struct Gpio {
    /// IO_BANK0 GPIOx_CTRL and PADS_BANK0 GPIOx. Change them through [`Gpio::write_io`] and [`Gpio::write_pads`],
    /// which keep the input levels ([`Gpio::inputs`]) up to date.
    pub ctrl: [u32; NUM_GPIOS],
    pub pads: [u32; NUM_GPIOS],
    pub pads_voltage: u32,
    pub pads_swclk: u32,
    pub pads_swd: u32,
    /// Latched edge interrupts, in INTR layout (4 bits per pin, 8 pins per word; only the edge bits).
    edges: [u32; 4],
    /// PROC0, PROC1 and DORMANT_WAKE interrupt enables and forces.
    pub inte: [[u32; 4]; 3],
    pub intf: [[u32; 4]; 3],
    /// What is connected to each pin from outside.
    pub external: [External; NUM_GPIOS],
    /// Resolved pin levels (bit per pin).
    pub level: u32,
    /// Output value and enable after overrides (what reaches the pads).
    pub out_to_pad: u32,
    pub oe_to_pad: u32,
    /// Output and enable from the selected peripheral, before overrides.
    out_from_peri: u32,
    oe_from_peri: u32,
    /// [`Gpio::inputs`] and the levels the interrupt logic sees, kept up to date whenever the levels, pads or CTRL
    /// change (they are read on every SIO and PIO access and every pin update, and computing them visits every pin).
    input_levels: u32,
    irq_levels: u32,
}

impl Default for Gpio {
    fn default() -> Self {
        Gpio {
            ctrl: [func::NULL; NUM_GPIOS],
            pads: [PAD_RESET; NUM_GPIOS],
            pads_voltage: 0,
            pads_swclk: 0xDA,
            pads_swd: 0x5A,
            edges: [0; 4],
            inte: [[0; 4]; 3],
            intf: [[0; 4]; 3],
            external: [External::Floating; NUM_GPIOS],
            level: 0,
            out_to_pad: 0,
            oe_to_pad: 0,
            out_from_peri: 0,
            oe_from_peri: 0,
            // Level 0 everywhere and INOVER/IRQOVER normal
            input_levels: 0,
            irq_levels: 0,
        }
    }
}

/// OUTOVER/OEOVER/INOVER/IRQOVER semantics: 0 normal, 1 invert, 2 force low, 3 force high.
fn over(value: bool, sel: u32) -> bool {
    match sel & 3 {
        0 => value,
        1 => !value,
        2 => false,
        _ => true,
    }
}

impl Gpio {
    /// Reset state of IO_BANK0, keeping what is connected outside and (if `keep_pads`) the pad settings.
    pub fn reset_io(&mut self, keep_pads: bool) {
        let mut fresh = Gpio { external: self.external, ..Gpio::default() };
        if keep_pads {
            fresh.pads = self.pads;
            fresh.pads_voltage = self.pads_voltage;
            fresh.pads_swclk = self.pads_swclk;
            fresh.pads_swd = self.pads_swd;
        }
        *self = fresh;
        self.update_inputs();
    }

    pub fn funcsel(&self, pin: usize) -> u32 {
        self.ctrl[pin] & 0x1F
    }

    /// Input level as seen by peripherals: pad level gated by the input enable, then INOVER.
    pub fn input(&self, pin: usize) -> bool {
        let pad = self.pads[pin] & PAD_IE != 0 && self.level & (1 << pin) != 0;
        over(pad, self.ctrl[pin] >> 16)
    }

    /// SIO `GPIO_IN`.
    #[inline]
    pub fn inputs(&self) -> u32 {
        self.input_levels
    }

    fn update_inputs(&mut self) {
        self.input_levels = (0..NUM_GPIOS).filter(|&p| self.input(p)).fold(0, |acc, p| acc | 1 << p);
        self.irq_levels = (0..NUM_GPIOS).filter(|&p| self.irq_level(p)).fold(0, |acc, p| acc | 1 << p);
    }

    /// PADS_BANK0 back to its reset values (the block was reset).
    pub fn reset_pads(&mut self) {
        self.pads = Gpio::default().pads;
        self.update_inputs();
    }

    /// Level the interrupt logic sees on `pin` (pad input then IRQOVER).
    fn irq_level(&self, pin: usize) -> bool {
        let pad = self.pads[pin] & PAD_IE != 0 && self.level & (1 << pin) != 0;
        over(pad, self.ctrl[pin] >> 28)
    }

    /// Re-resolves every pin. `drive(func, pin)` gives the selected peripheral's output. Returns the pins whose
    /// level changed.
    pub fn evaluate(&mut self, mut drive: impl FnMut(u32, usize) -> Drive) -> u32 {
        let old_level = self.level;
        let old_irq = self.irq_levels;
        let (mut inputs, mut irq) = (0, 0);
        let mut level = 0;
        let (mut ofp, mut oefp, mut otp, mut oetp) = (0, 0, 0, 0);
        for pin in 0..NUM_GPIOS {
            let ctrl = self.ctrl[pin];
            let d = match ctrl & 0x1F {
                func::NULL => Drive::default(),
                f => drive(f, pin),
            };
            let out = over(d.out, ctrl >> 8);
            let oe = over(d.oe, ctrl >> 12) && self.pads[pin] & PAD_OD == 0;
            let pad = self.pads[pin];
            let l =
                resolve(oe.then(|| Level::from_bool(out)), pad & PAD_PUE != 0, pad & PAD_PDE != 0, self.external[pin]);
            let bit = 1 << pin;
            if l.is_high() {
                level |= bit;
            }
            // `input` and `irq_level` for the new level
            let pad_in = pad & PAD_IE != 0 && l.is_high();
            if over(pad_in, ctrl >> 16) {
                inputs |= bit;
            }
            if over(pad_in, ctrl >> 28) {
                irq |= bit;
            }
            if d.out {
                ofp |= bit;
            }
            if d.oe {
                oefp |= bit;
            }
            if out {
                otp |= bit;
            }
            if oe {
                oetp |= bit;
            }
        }
        self.level = level;
        self.input_levels = inputs;
        self.irq_levels = irq;
        self.out_from_peri = ofp;
        self.oe_from_peri = oefp;
        self.out_to_pad = otp;
        self.oe_to_pad = oetp;
        // Latch edges
        let mut changed = old_irq ^ irq;
        while changed != 0 {
            let pin = changed.trailing_zeros() as usize;
            changed &= changed - 1;
            let shift = 4 * (pin % 8);
            self.edges[pin / 8] |= if irq & (1 << pin) != 0 { 0b1000 } else { 0b0100 } << shift;
        }
        old_level ^ level
    }

    /// Raw interrupt status word `i` (INTR0-3).
    pub fn intr(&self, i: usize) -> u32 {
        let mut v = self.edges[i];
        for k in 0..8 {
            let pin = i * 8 + k;
            if pin >= NUM_GPIOS {
                break;
            }
            v |= if self.irq_levels & (1 << pin) != 0 { 0b0010 } else { 0b0001 } << (4 * k);
        }
        v
    }

    /// Interrupt status for `proc` (0, 1, or 2 for dormant wake) word `i`.
    pub fn ints(&self, proc: usize, i: usize) -> u32 {
        (self.intr(i) | self.intf[proc][i]) & self.inte[proc][i]
    }

    /// Whether IO_IRQ_BANK0 is asserted for `core`.
    pub fn irq(&self, core: usize) -> bool {
        // Only words with something enabled or forced can assert the line
        (0..4).any(|i| (self.inte[core][i] != 0 || self.intf[core][i] != 0) && self.ints(core, i) != 0)
    }

    fn status(&self, pin: usize) -> u32 {
        let bit = |m: u32, s: u32| if m & (1 << pin) != 0 { 1 << s } else { 0 };
        let irq_from_pad = self.irq_level(pin);
        let mut irq_to_proc = false;
        for p in 0..2 {
            if self.ints(p, pin / 8) & (0xF << (4 * (pin % 8))) != 0 {
                irq_to_proc = true;
            }
        }
        let in_pad = self.pads[pin] & PAD_IE != 0 && self.level & (1 << pin) != 0;
        bit(self.out_from_peri, 8)
            | bit(self.out_to_pad, 9)
            | bit(self.oe_from_peri, 12)
            | bit(self.oe_to_pad, 13)
            | (in_pad as u32) << 17
            | (self.input(pin) as u32) << 19
            | (irq_from_pad as u32) << 24
            | (irq_to_proc as u32) << 26
    }

    /// IO_BANK0 register read.
    pub fn read_io(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x000..=0x0EC => {
                let pin = (offset / 8) as usize;
                if offset.is_multiple_of(8) { self.status(pin) } else { self.ctrl[pin] }
            }
            0x0F0..=0x0FC => self.intr(((offset - 0x0F0) / 4) as usize),
            0x100..=0x18C => {
                let rel = offset - 0x100;
                let proc = (rel / 0x30) as usize;
                let kind = (rel % 0x30) / 0x10;
                let i = ((rel % 0x10) / 4) as usize;
                match kind {
                    0 => self.inte[proc][i],
                    1 => self.intf[proc][i],
                    _ => self.ints(proc, i),
                }
            }
            _ => return None,
        })
    }

    /// IO_BANK0 register write. Returns whether pin levels must be re-evaluated.
    pub fn write_io(&mut self, offset: u32, value: u32, mode: WriteMode) -> Option<bool> {
        match offset {
            0x000..=0x0EC => {
                let pin = (offset / 8) as usize;
                if offset % 8 == 4 {
                    mode.update(&mut self.ctrl[pin], value, 0x3033_331F);
                    self.update_inputs();
                    return Some(true);
                }
            }
            0x0F0..=0x0FC => {
                // Only the edge bits are write-1-to-clear
                let i = ((offset - 0x0F0) / 4) as usize;
                self.edges[i] &= !(mode.w1c_bits(value) & 0xCCCC_CCCC);
            }
            0x100..=0x18C => {
                let rel = offset - 0x100;
                let proc = (rel / 0x30) as usize;
                let kind = (rel % 0x30) / 0x10;
                let i = ((rel % 0x10) / 4) as usize;
                let mask = if i == 3 { 0x00FF_FFFF } else { 0xFFFF_FFFF };
                match kind {
                    0 => mode.update(&mut self.inte[proc][i], value, mask),
                    1 => mode.update(&mut self.intf[proc][i], value, mask),
                    _ => {}
                }
            }
            _ => return None,
        }
        Some(false)
    }

    /// PADS_BANK0 register read.
    pub fn read_pads(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x00 => self.pads_voltage,
            0x04..=0x78 => self.pads[((offset - 4) / 4) as usize],
            0x7C => self.pads_swclk,
            0x80 => self.pads_swd,
            _ => return None,
        })
    }

    /// PADS_BANK0 register write. Returns whether pin levels must be re-evaluated.
    pub fn write_pads(&mut self, offset: u32, value: u32, mode: WriteMode) -> Option<bool> {
        match offset {
            0x00 => mode.update(&mut self.pads_voltage, value, 1),
            0x04..=0x78 => {
                mode.update(&mut self.pads[((offset - 4) / 4) as usize], value, 0xFF);
                self.update_inputs();
            }
            0x7C => mode.update(&mut self.pads_swclk, value, 0xFF),
            0x80 => mode.update(&mut self.pads_swd, value, 0xFF),
            _ => return None,
        }
        Some(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sio(out: u32, oe: u32) -> impl FnMut(u32, usize) -> Drive {
        move |f, pin| {
            if f == func::SIO {
                Drive { out: out & (1 << pin) != 0, oe: oe & (1 << pin) != 0 }
            } else {
                Drive::default()
            }
        }
    }

    #[test]
    fn input_and_interrupt_levels_follow_every_change() {
        let mut g = Gpio::default();
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let fresh = |g: &Gpio| (0..NUM_GPIOS).filter(|&p| g.input(p)).fold(0u32, |a, p| a | 1 << p);
        for _ in 0..5000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let pin = (x >> 8) as usize % NUM_GPIOS;
            match x % 6 {
                0 => {
                    g.write_io(pin as u32 * 8 + 4, (x >> 16) as u32 & 0x3033_331F, WriteMode::Normal);
                }
                1 => {
                    g.write_pads(4 + 4 * pin as u32, (x >> 20) as u32 & 0xFF, WriteMode::Xor);
                }
                2 => {
                    g.external[pin] = match (x >> 30) % 3 {
                        0 => External::Floating,
                        1 => External::Driven(Level::from_bool(x & (1 << 40) != 0)),
                        _ => External::Pull(Level::from_bool(x & (1 << 41) != 0)),
                    };
                    g.evaluate(sio((x >> 32) as u32, (x >> 2) as u32));
                }
                3 => g.reset_io(x & 1 << 50 != 0),
                4 => g.reset_pads(),
                _ => {
                    g.evaluate(sio((x >> 32) as u32, (x >> 2) as u32));
                }
            }
            assert_eq!(g.inputs(), fresh(&g));
            let irq = (0..NUM_GPIOS).filter(|&p| g.irq_level(p)).fold(0u32, |a, p| a | 1 << p);
            assert_eq!(g.irq_levels, irq);
        }
    }

    #[test]
    fn reset_state() {
        let mut g = Gpio::default();
        g.evaluate(sio(0, 0));
        assert_eq!(g.level, 0, "pull-downs enabled at reset");
        assert_eq!(g.read_pads(0x04), Some(0x56));
        assert_eq!(g.read_io(0x04), Some(0x1F), "FUNCSEL NULL");
    }

    #[test]
    fn sio_output_and_pull_up_input() {
        let mut g = Gpio::default();
        g.write_io(25 * 8 + 4, func::SIO, WriteMode::Normal);
        g.write_pads(4 + 4 * 3, PAD_PUE | PAD_IE, WriteMode::Normal); // GPIO3 pull-up only
        let changed = g.evaluate(sio(1 << 25, 1 << 25));
        assert_eq!(changed, 1 << 25 | 1 << 3);
        assert_eq!(g.level & (1 << 25), 1 << 25);
        assert!(g.input(3));
        // GPIO_STATUS: OUTTOPAD, OETOPAD, INFROMPAD
        let st = g.read_io(25 * 8).unwrap();
        assert_eq!(st & (1 << 9 | 1 << 13 | 1 << 17), 1 << 9 | 1 << 13 | 1 << 17);
        // External driver overrides the pull
        g.external[3] = External::Driven(Level::Low);
        g.evaluate(sio(1 << 25, 1 << 25));
        assert!(!g.input(3));
        // Input disabled reads low
        g.external[3] = External::Driven(Level::High);
        g.write_pads(4 + 4 * 3, PAD_IE, WriteMode::Clear);
        g.evaluate(sio(0, 0));
        assert!(!g.input(3));
    }

    #[test]
    fn overrides_and_output_disable() {
        let mut g = Gpio::default();
        g.write_io(5 * 8 + 4, func::SIO | 1 << 8, WriteMode::Normal); // OUTOVER invert
        g.evaluate(sio(0, 1 << 5));
        assert!(g.level & (1 << 5) != 0);
        g.write_io(5 * 8 + 4, 3 << 12, WriteMode::Set); // OEOVER force enable
        g.write_pads(4 + 4 * 5, PAD_OD, WriteMode::Set); // but the pad disables output
        g.evaluate(sio(0, 0));
        assert_eq!(g.level & (1 << 5), 0, "OD wins; pull-down");
        g.write_io(5 * 8 + 4, 3 << 16, WriteMode::Set); // INOVER force high
        assert!(g.input(5));
    }

    #[test]
    fn edge_and_level_interrupts() {
        let mut g = Gpio::default();
        g.write_io(22 * 8 + 4, func::SIO, WriteMode::Normal);
        g.write_pads(4 + 4 * 22, PAD_PUE | PAD_IE, WriteMode::Normal);
        g.evaluate(sio(0, 0));
        // GPIO22 is group 2, slot 6: LEVEL_LOW bit 24, LEVEL_HIGH 25, EDGE_LOW 26, EDGE_HIGH 27
        assert_eq!(g.intr(2) >> 24 & 0xF, 0b1010, "high level, and the rising edge from the pull-up");
        g.write_io(0x0F0 + 8, 0xF << 24, WriteMode::Normal);
        assert_eq!(g.intr(2) >> 24 & 0xF, 0b0010);
        g.write_io(0x100 + 8, 1 << 24, WriteMode::Normal); // PROC0_INTE2 LEVEL_LOW
        assert!(!g.irq(0));
        g.external[22] = External::Driven(Level::Low);
        g.evaluate(sio(0, 0));
        assert_eq!(g.intr(2) >> 24 & 0xF, 0b0101);
        assert!(g.irq(0));
        assert!(!g.irq(1));
        assert_eq!(g.read_io(0x120 + 8), Some(1 << 24), "PROC0_INTS2");
        // PROC1 force
        g.write_io(0x130 + 0x10, 1, WriteMode::Normal); // PROC1_INTF0
        g.write_io(0x130, 1, WriteMode::Normal); // PROC1_INTE0
        assert!(g.irq(1));
    }
}
