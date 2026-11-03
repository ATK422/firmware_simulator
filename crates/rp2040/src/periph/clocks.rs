//! The clock tree: XOSC, ROSC, PLL_SYS, PLL_USB and the CLOCKS block (datasheet 2.15-2.18).
//!
//! Frequencies follow from the register configuration: oscillators report STABLE and PLLs LOCK after a startup
//! time, the glitchless muxes of `clk_ref`/`clk_sys` report the selected source in `*_SELECTED`, and the resulting
//! `clk_ref`, `clk_sys`, `clk_peri`, `clk_usb`, `clk_adc`, `clk_rtc` frequencies drive the rest of the machine
//! (CPU cycle time, the timer tick, peripheral baud rates). A wrongly configured clock tree therefore shows up as
//! wrong timing, as on hardware.
//!
//! The ROSC runs at a fixed nominal frequency ([`ROSC_HZ`]); its drive strength and divider registers are stored
//! but do not change it.

use sim_core::bus::WriteMode;
use sim_core::time::Time;

/// Nominal ROSC frequency (datasheet: 1.8-12 MHz, typically 6.5 MHz at reset).
pub const ROSC_HZ: u64 = 6_500_000;
/// Time from enabling the PLL VCO until LOCK.
pub const PLL_LOCK_TIME: Time = Time::from_us(50);

// ------------------------------------------------------------------------------------------------ XOSC

const XOSC_ENABLE: u32 = 0xFAB;
const XOSC_DISABLE: u32 = 0xD1E;

#[derive(Clone, Debug)]
pub struct Xosc {
    pub crystal_hz: u64,
    pub ctrl: u32,
    pub startup: u32,
    pub dormant: u32,
    pub count: u32,
    /// When the oscillator becomes stable, if enabled.
    stable_at: Option<Time>,
    badwrite: bool,
    /// Enabled: off after reset; a write of DISABLE turns it off, ENABLE or any invalid code turns it on.
    on: bool,
}

impl Xosc {
    pub fn new(crystal_hz: u64) -> Self {
        Xosc {
            crystal_hz,
            ctrl: 0x0000_0AA0,
            startup: 0x0000_00C4,
            dormant: 0,
            count: 0,
            stable_at: None,
            badwrite: false,
            on: false,
        }
    }

    pub fn enabled(&self) -> bool {
        self.on
    }

    pub fn is_stable(&self, now: Time) -> bool {
        self.stable_at.is_some_and(|t| now >= t)
    }

    /// Output frequency (0 until stable).
    pub fn hz(&self, now: Time) -> u64 {
        if self.is_stable(now) { self.crystal_hz } else { 0 }
    }

    pub fn read(&self, now: Time, offset: u32) -> Option<u32> {
        Some(match offset {
            0x00 => self.ctrl,
            0x04 => {
                let mut v = 0; // FREQ_RANGE 1_15MHZ reads as 0
                if self.enabled() {
                    v |= 1 << 12;
                }
                if self.badwrite {
                    v |= 1 << 24;
                }
                if self.is_stable(now) {
                    v |= 1 << 31;
                }
                v
            }
            0x08 => self.dormant,
            0x0C => self.startup,
            0x1C => self.count,
            _ => return None,
        })
    }

    pub fn write(&mut self, now: Time, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            0x00 => {
                let was = self.enabled();
                mode.update(&mut self.ctrl, value, 0x00FF_FFFF);
                let en = (self.ctrl >> 12) & 0xFFF;
                if en != XOSC_ENABLE && en != XOSC_DISABLE {
                    // "An invalid setting will enable the oscillator"
                    self.badwrite = true;
                }
                self.on = en != XOSC_DISABLE;
                match (was, self.enabled()) {
                    (false, true) => {
                        // DELAY counts 256 crystal cycles; X4 multiplies by four
                        let mut cycles = (self.startup & 0x3FFF) as u64 * 256;
                        if self.startup & (1 << 20) != 0 {
                            cycles *= 4;
                        }
                        let t = sim_core::time::ClockPeriod::from_hz(self.crystal_hz).cycles(cycles);
                        self.stable_at = Some(now + t);
                    }
                    (true, false) => self.stable_at = None,
                    _ => {}
                }
            }
            0x04 => {
                // STATUS.BADWRITE is write-1-to-clear
                if mode.w1c_bits(value) & (1 << 24) != 0 {
                    self.badwrite = false;
                }
            }
            0x08 => self.dormant = mode.apply(self.dormant, value),
            0x0C => mode.update(&mut self.startup, value, 0x0010_3FFF),
            0x1C => mode.update(&mut self.count, value, 0xFF),
            _ => return false,
        }
        true
    }
}

// ------------------------------------------------------------------------------------------------ ROSC

#[derive(Clone, Debug)]
pub struct Rosc {
    pub ctrl: u32,
    pub freqa: u32,
    pub freqb: u32,
    pub dormant: u32,
    pub div: u32,
    pub phase: u32,
    pub count: u32,
    rng: u64,
}

impl Rosc {
    pub fn new(seed: u64) -> Self {
        Rosc { ctrl: 0x0000_0AA0, freqa: 0, freqb: 0, dormant: 0, div: 0, phase: 0x0000_0008, count: 0, rng: seed | 1 }
    }

    pub fn enabled(&self) -> bool {
        (self.ctrl >> 12) & 0xFFF != 0xD1E
    }

    pub fn hz(&self) -> u64 {
        if self.enabled() { ROSC_HZ } else { 0 }
    }

    fn random_bit(&mut self) -> u32 {
        // xorshift64*: deterministic for a given machine seed
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 63) as u32
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x00 => self.ctrl,
            0x04 => self.freqa,
            0x08 => self.freqb,
            0x0C => self.dormant,
            0x10 => self.div,
            0x14 => self.phase,
            0x18 => {
                let mut v = 1 << 16; // DIV_RUNNING
                if self.enabled() {
                    v |= 1 << 12 | 1 << 31;
                }
                v
            }
            0x1C => self.random_bit(),
            0x20 => self.count,
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            0x00 => mode.update(&mut self.ctrl, value, 0x00FF_FFFF),
            0x04 => mode.update(&mut self.freqa, value, 0xFFFF_7777),
            0x08 => mode.update(&mut self.freqb, value, 0xFFFF_7777),
            0x0C => self.dormant = mode.apply(self.dormant, value),
            0x10 => mode.update(&mut self.div, value, 0xFFF),
            0x14 => mode.update(&mut self.phase, value, 0xFFF),
            0x18 | 0x1C => {}
            0x20 => mode.update(&mut self.count, value, 0xFF),
            _ => return false,
        }
        true
    }
}

// ------------------------------------------------------------------------------------------------ PLL

#[derive(Clone, Debug)]
pub struct Pll {
    pub cs: u32,
    pub pwr: u32,
    pub fbdiv_int: u32,
    pub prim: u32,
    /// When the VCO was last powered up with the current configuration.
    lock_at: Option<Time>,
}

impl Default for Pll {
    fn default() -> Self {
        Pll { cs: 0x0000_0001, pwr: 0x0000_002D, fbdiv_int: 0, prim: 0x0007_7000, lock_at: None }
    }
}

const PWR_PD: u32 = 1 << 0;
const PWR_POSTDIVPD: u32 = 1 << 3;
const PWR_VCOPD: u32 = 1 << 5;

impl Pll {
    fn vco_running(&self) -> bool {
        self.pwr & (PWR_PD | PWR_VCOPD) == 0
    }

    pub fn locked(&self, now: Time) -> bool {
        self.lock_at.is_some_and(|t| now >= t)
    }

    /// VCO frequency for a reference input of `ref_hz`.
    pub fn vco_hz(&self, ref_hz: u64) -> u64 {
        let refdiv = (self.cs & 0x3F).max(1) as u64;
        ref_hz / refdiv * (self.fbdiv_int & 0xFFF) as u64
    }

    /// Output frequency for a reference input of `ref_hz` (0 unless locked and the post dividers run).
    pub fn hz(&self, now: Time, ref_hz: u64) -> u64 {
        if self.cs & (1 << 8) != 0 {
            return ref_hz; // BYPASS
        }
        if !self.locked(now) || self.pwr & PWR_POSTDIVPD != 0 {
            return 0;
        }
        let pd1 = ((self.prim >> 16) & 7).max(1) as u64;
        let pd2 = ((self.prim >> 12) & 7).max(1) as u64;
        self.vco_hz(ref_hz) / (pd1 * pd2)
    }

    pub fn read(&self, now: Time, offset: u32) -> Option<u32> {
        Some(match offset {
            0x0 => (self.cs & 0x13F) | if self.locked(now) { 1 << 31 } else { 0 },
            0x4 => self.pwr,
            0x8 => self.fbdiv_int,
            0xC => self.prim,
            _ => return None,
        })
    }

    pub fn write(&mut self, now: Time, offset: u32, value: u32, mode: WriteMode) -> bool {
        let before = (self.cs & 0x3F, self.pwr & (PWR_PD | PWR_VCOPD), self.fbdiv_int);
        match offset {
            0x0 => mode.update(&mut self.cs, value, 0x13F),
            0x4 => mode.update(&mut self.pwr, value, 0x2D),
            0x8 => mode.update(&mut self.fbdiv_int, value, 0xFFF),
            0xC => mode.update(&mut self.prim, value, 0x0007_7000),
            _ => return false,
        }
        // A change to the VCO's configuration or power restarts the lock timer
        if before != (self.cs & 0x3F, self.pwr & (PWR_PD | PWR_VCOPD), self.fbdiv_int) {
            self.lock_at = if self.vco_running() && self.fbdiv_int >= 16 { Some(now + PLL_LOCK_TIME) } else { None };
        }
        true
    }
}

// ------------------------------------------------------------------------------------------------ CLOCKS

/// Indices of the clock generators in the CLOCKS block (each has CTRL, DIV, SELECTED at `index * 0xC`).
pub mod clk {
    pub const GPOUT0: usize = 0;
    pub const GPOUT1: usize = 1;
    pub const GPOUT2: usize = 2;
    pub const GPOUT3: usize = 3;
    pub const REF: usize = 4;
    pub const SYS: usize = 5;
    pub const PERI: usize = 6;
    pub const USB: usize = 7;
    pub const ADC: usize = 8;
    pub const RTC: usize = 9;
}

const CTRL_ENABLE: u32 = 1 << 11;

/// Frequencies of the clock tree outputs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Freqs {
    pub xosc: u64,
    pub rosc: u64,
    pub pll_sys: u64,
    pub pll_usb: u64,
    pub clk_ref: u64,
    pub clk_sys: u64,
    pub clk_peri: u64,
    pub clk_usb: u64,
    pub clk_adc: u64,
    pub clk_rtc: u64,
    pub gpout: [u64; 4],
}

#[derive(Clone, Debug)]
pub struct Clocks {
    pub ctrl: [u32; 10],
    pub div: [u32; 10],
    /// The other registers (resus, FC0, wake/sleep enables, interrupts), by offset from 0x78.
    pub misc: [u32; 18],
    pub xosc: Xosc,
    pub rosc: Rosc,
    pub pll_sys: Pll,
    pub pll_usb: Pll,
}

/// Writable CTRL bits per clock generator.
const CTRL_MASK: [u32; 10] = [
    0x0013_1DE0,
    0x0013_1DE0,
    0x0013_1DE0,
    0x0013_1DE0,
    0x0000_0063,
    0x0000_00E1,
    0x0000_0CE0,
    0x0013_0CE0,
    0x0013_0CE0,
    0x0013_0CE0,
];
const DIV_MASK: [u32; 10] =
    [0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0x300, 0xFFFF_FFFF, 0, 0x300, 0x300, 0xFFFF_FFFF];

impl Clocks {
    pub fn new(crystal_hz: u64, seed: u64) -> Self {
        let mut div = [0x100u32; 10];
        div[clk::PERI] = 0;
        Clocks {
            ctrl: [0; 10],
            div,
            misc: [0; 18],
            xosc: Xosc::new(crystal_hz),
            rosc: Rosc::new(seed),
            pll_sys: Pll::default(),
            pll_usb: Pll::default(),
        }
    }

    /// Applies a 24.8 (or 2.0 for the integer-only ones) divider to `hz`.
    fn divide(&self, i: usize, hz: u64) -> u64 {
        let d = self.div[i];
        let fixed =
            if DIV_MASK[i] == 0x300 { (((d >> 8) & 3) as u64) << 8 } else { (d as u64 >> 8) << 8 | (d & 0xFF) as u64 };
        if fixed == 0 {
            // INT = 0 means the maximum divisor (2^24 for the fractional dividers, 4 for the 2-bit ones)
            let max = if DIV_MASK[i] == 0x300 { 4u64 << 8 } else { 1u64 << 32 };
            return hz * 256 / max;
        }
        hz * 256 / fixed
    }

    fn enabled(&self, i: usize) -> bool {
        self.ctrl[i] & CTRL_ENABLE != 0
    }

    /// Computes every clock frequency at time `now`.
    pub fn freqs(&self, now: Time) -> Freqs {
        let xosc = self.xosc.hz(now);
        let rosc = self.rosc.hz();
        let pll_sys = self.pll_sys.hz(now, xosc);
        let pll_usb = self.pll_usb.hz(now, xosc);

        let ref_src = match self.ctrl[clk::REF] & 3 {
            0 => rosc,
            1 => match (self.ctrl[clk::REF] >> 5) & 3 {
                0 => pll_usb,
                _ => 0, // GPIN0/1 not connected
            },
            _ => xosc,
        };
        let clk_ref = self.divide(clk::REF, ref_src);

        let sys_aux = match (self.ctrl[clk::SYS] >> 5) & 7 {
            0 => pll_sys,
            1 => pll_usb,
            2 => rosc,
            3 => xosc,
            _ => 0,
        };
        let sys_src = if self.ctrl[clk::SYS] & 1 == 0 { clk_ref } else { sys_aux };
        let clk_sys = self.divide(clk::SYS, sys_src);

        let clk_peri = if self.enabled(clk::PERI) {
            match (self.ctrl[clk::PERI] >> 5) & 7 {
                0 => clk_sys,
                1 => pll_sys,
                2 => pll_usb,
                3 => rosc,
                4 => xosc,
                _ => 0,
            }
        } else {
            0
        };

        let aux4 = |i: usize| -> u64 {
            if !self.enabled(i) {
                return 0;
            }
            let src = match (self.ctrl[i] >> 5) & 7 {
                0 => pll_usb,
                1 => pll_sys,
                2 => rosc,
                3 => xosc,
                _ => 0,
            };
            self.divide(i, src)
        };
        let clk_usb = aux4(clk::USB);
        let clk_adc = aux4(clk::ADC);
        let clk_rtc = aux4(clk::RTC);

        let mut gpout = [0u64; 4];
        for (i, g) in gpout.iter_mut().enumerate() {
            if self.enabled(i) {
                let src = match (self.ctrl[i] >> 5) & 0xF {
                    0 => pll_sys,
                    3 => pll_usb,
                    4 => rosc,
                    5 => xosc,
                    6 => clk_sys,
                    7 => clk_usb,
                    8 => clk_adc,
                    9 => clk_rtc,
                    10 => clk_ref,
                    _ => 0,
                };
                *g = self.divide(i, src);
            }
        }

        Freqs { xosc, rosc, pll_sys, pll_usb, clk_ref, clk_sys, clk_peri, clk_usb, clk_adc, clk_rtc, gpout }
    }

    pub fn read(&self, offset: u32) -> Option<u32> {
        if offset < 0x78 {
            let i = (offset / 0xC) as usize;
            return Some(match offset % 0xC {
                0 => self.ctrl[i],
                4 => self.div[i],
                _ => match i {
                    clk::REF => 1 << (self.ctrl[i] & 3),
                    clk::SYS => 1 << (self.ctrl[i] & 1),
                    _ => 1,
                },
            });
        }
        let idx = ((offset - 0x78) / 4) as usize;
        match offset {
            // FC0_STATUS: report DONE and VALID so frequency counting terminates
            0x98 => Some(0x0000_0110),
            // FC0_RESULT is filled in by `count_frequency`
            0x78..=0xC4 => Some(self.misc[idx]),
            _ => None,
        }
    }

    /// Writes a CLOCKS register. Returns `None` for unknown offsets, otherwise whether frequencies may have changed.
    pub fn write(&mut self, now: Time, offset: u32, value: u32, mode: WriteMode) -> Option<bool> {
        if offset < 0x78 {
            let i = (offset / 0xC) as usize;
            match offset % 0xC {
                0 => mode.update(&mut self.ctrl[i], value, CTRL_MASK[i]),
                4 => mode.update(&mut self.div[i], value, DIV_MASK[i]),
                _ => {}
            }
            return Some(true);
        }
        let idx = ((offset - 0x78) / 4) as usize;
        match offset {
            0x9C | 0x98 => {}
            0x94 => {
                // FC0_SRC: writing starts a frequency count; the result is available at once
                self.misc[idx] = value & 0xFF;
                let khz = self.count_source(now, value & 0xFF) / 1000;
                self.misc[(0x9C - 0x78) / 4] = (khz as u32) << 5;
            }
            0xB8 => {
                // INTR is read-only (resus)
            }
            0x78..=0xC4 => self.misc[idx] = mode.apply(self.misc[idx], value),
            _ => return None,
        }
        Some(false)
    }

    /// Frequency of an FC0_SRC selection.
    fn count_source(&self, now: Time, src: u32) -> u64 {
        let f = self.freqs(now);
        match src {
            0x01 => f.pll_sys,
            0x02 => f.pll_usb,
            0x03 => f.rosc,
            0x04 => f.rosc,
            0x05 => f.xosc,
            0x08 => f.clk_ref,
            0x09 => f.clk_sys,
            0x0A => f.clk_peri,
            0x0B => f.clk_usb,
            0x0C => f.clk_adc,
            0x0D => f.clk_rtc,
            _ => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MHZ: u64 = 1_000_000;

    #[test]
    fn reset_state_runs_from_rosc() {
        let c = Clocks::new(12 * MHZ, 1);
        let f = c.freqs(Time::ZERO);
        assert_eq!(f.clk_ref, ROSC_HZ);
        assert_eq!(f.clk_sys, ROSC_HZ);
        assert_eq!(f.clk_peri, 0);
        assert_eq!(c.read(4 * 0xC + 8), Some(1), "CLK_REF_SELECTED = ROSC");
        assert_eq!(c.read(5 * 0xC + 8), Some(1), "CLK_SYS_SELECTED = CLK_REF");
        assert_eq!(c.read(6 * 0xC + 8), Some(1), "hardwired");
    }

    #[test]
    fn xosc_startup_delay() {
        let mut x = Xosc::new(12 * MHZ);
        x.write(Time::ZERO, 0x0C, 47, WriteMode::Normal); // ~1 ms
        x.write(Time::ZERO, 0x00, 0xFAB << 12 | 0xAA0, WriteMode::Normal);
        assert!(x.read(Time::ZERO, 0x04).unwrap() & (1 << 12) != 0, "ENABLED");
        assert!(x.read(Time::from_us(500), 0x04).unwrap() & (1 << 31) == 0);
        assert!(x.read(Time::from_us(1100), 0x04).unwrap() & (1 << 31) != 0);
        x.write(Time::ZERO, 0x00, 0x123 << 12, WriteMode::Normal);
        assert!(x.read(Time::ZERO, 0x04).unwrap() & (1 << 24) != 0, "BADWRITE");
    }

    #[test]
    fn pll_lock_and_frequency() {
        let mut p = Pll::default();
        // 12 MHz / 1 * 125 = 1500 MHz VCO, / (6 * 2) = 125 MHz
        p.write(Time::ZERO, 0x0, 1, WriteMode::Normal);
        p.write(Time::ZERO, 0x8, 125, WriteMode::Normal);
        p.write(Time::ZERO, 0x4, PWR_POSTDIVPD, WriteMode::Normal);
        assert!(!p.locked(Time::from_us(10)));
        assert!(p.read(Time::from_us(60), 0).unwrap() & (1 << 31) != 0);
        p.write(Time::ZERO, 0xC, 6 << 16 | 2 << 12, WriteMode::Normal);
        assert_eq!(p.hz(Time::from_us(60), 12 * MHZ), 0, "post dividers powered down");
        p.write(Time::from_us(60), 0x4, PWR_POSTDIVPD, WriteMode::Clear);
        // Powering up the post dividers (through the CLR alias) keeps the lock
        assert_eq!(p.hz(Time::from_us(60), 12 * MHZ), 125 * MHZ);
        p.write(Time::from_us(100), 0x8, 100, WriteMode::Normal);
        assert!(!p.locked(Time::from_us(120)), "new FBDIV relocks");
    }

    #[test]
    fn full_embassy_style_configuration() {
        let mut c = Clocks::new(12 * MHZ, 1);
        let t0 = Time::ZERO;
        c.xosc.write(t0, 0x0C, 47, WriteMode::Normal);
        c.xosc.write(t0, 0x00, 0xFAB << 12 | 0xAA0, WriteMode::Normal);
        let t = Time::from_ms(2);
        for (pll, fb, pd1, pd2) in [(&mut c.pll_sys, 125, 6, 2), (&mut c.pll_usb, 100, 5, 5)] {
            pll.write(t, 0x0, 1, WriteMode::Normal);
            pll.write(t, 0x8, fb, WriteMode::Normal);
            pll.write(t, 0x4, PWR_POSTDIVPD, WriteMode::Normal);
            pll.write(t, 0xC, pd1 << 16 | pd2 << 12, WriteMode::Normal);
            pll.write(t, 0x4, PWR_POSTDIVPD, WriteMode::Clear);
        }
        // clk_ref = XOSC, clk_sys = PLL_SYS aux, clk_peri = clk_sys, clk_usb = PLL_USB
        c.write(t, 4 * 0xC, 2, WriteMode::Normal);
        c.write(t, 5 * 0xC, 1, WriteMode::Normal);
        c.write(t, 6 * 0xC, CTRL_ENABLE, WriteMode::Normal);
        c.write(t, 7 * 0xC, CTRL_ENABLE, WriteMode::Normal);
        c.write(t, 7 * 0xC + 4, 1 << 8, WriteMode::Normal);
        c.write(t, 9 * 0xC, CTRL_ENABLE, WriteMode::Normal);
        c.write(t, 9 * 0xC + 4, 1024 << 8, WriteMode::Normal);
        let f = c.freqs(Time::from_ms(3));
        assert_eq!(f.clk_ref, 12 * MHZ);
        assert_eq!(f.clk_sys, 125 * MHZ);
        assert_eq!(f.clk_peri, 125 * MHZ);
        assert_eq!(f.clk_usb, 48 * MHZ);
        assert_eq!(f.clk_rtc, 46_875);
        assert_eq!(c.read(4 * 0xC + 8), Some(1 << 2));
        assert_eq!(c.read(5 * 0xC + 8), Some(1 << 1));
        // Frequency counter
        c.write(Time::from_ms(3), 0x94, 0x09, WriteMode::Normal);
        assert_eq!(c.read(0x9C).unwrap() >> 5, 125_000);
    }

    #[test]
    fn fractional_sys_divider_and_gpout() {
        let mut c = Clocks::new(12 * MHZ, 1);
        c.write(Time::ZERO, 5 * 0xC + 4, 2 << 8 | 0x80, WriteMode::Normal); // /2.5
        assert_eq!(c.freqs(Time::ZERO).clk_sys, ROSC_HZ * 2 / 5);
        c.write(Time::ZERO, 0, CTRL_ENABLE | 10 << 5, WriteMode::Normal); // GPOUT0 = clk_ref
        c.write(Time::ZERO, 4, 2 << 8, WriteMode::Normal);
        assert_eq!(c.freqs(Time::ZERO).gpout[0], ROSC_HZ / 2);
    }

    #[test]
    fn xosc_invalid_enable_code_keeps_it_running() {
        // pico-sdk's xosc_init writes CTRL with ENABLE = 0 before setting ENABLE: the crystal must keep running
        let mut x = Xosc::new(12_000_000);
        assert!(!x.enabled(), "off after reset");
        x.write(Time::ZERO, 0x00, XOSC_ENABLE << 12 | 0xAA0, WriteMode::Normal);
        let t = Time::from_ms(10);
        assert_eq!(x.hz(t), 12_000_000);
        x.write(t, 0x00, 0xAA0, WriteMode::Normal);
        assert_eq!(x.hz(t), 12_000_000, "an invalid code enables");
        assert_ne!(x.read(t, 0x04).unwrap() & (1 << 24), 0, "BADWRITE");
        x.write(t, 0x00, XOSC_DISABLE << 12 | 0xAA0, WriteMode::Normal);
        assert_eq!(x.hz(t), 0);
    }
}
