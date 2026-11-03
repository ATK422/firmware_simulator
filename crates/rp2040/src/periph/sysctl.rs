//! Small system control blocks: RESETS (2.14), PSM (2.13), VREG_AND_CHIP_RESET (2.10) and SYSINFO (2.20).

use sim_core::bus::WriteMode;

/// RESETS bit numbers.
pub mod bit {
    pub const ADC: u32 = 0;
    pub const BUSCTRL: u32 = 1;
    pub const DMA: u32 = 2;
    pub const I2C0: u32 = 3;
    pub const I2C1: u32 = 4;
    pub const IO_BANK0: u32 = 5;
    pub const IO_QSPI: u32 = 6;
    pub const JTAG: u32 = 7;
    pub const PADS_BANK0: u32 = 8;
    pub const PADS_QSPI: u32 = 9;
    pub const PIO0: u32 = 10;
    pub const PIO1: u32 = 11;
    pub const PLL_SYS: u32 = 12;
    pub const PLL_USB: u32 = 13;
    pub const PWM: u32 = 14;
    pub const RTC: u32 = 15;
    pub const SPI0: u32 = 16;
    pub const SPI1: u32 = 17;
    pub const SYSCFG: u32 = 18;
    pub const SYSINFO: u32 = 19;
    pub const TBMAN: u32 = 20;
    pub const TIMER: u32 = 21;
    pub const UART0: u32 = 22;
    pub const UART1: u32 = 23;
    pub const USBCTRL: u32 = 24;
}

pub const ALL_RESETS: u32 = 0x01FF_FFFF;

#[derive(Clone, Debug)]
pub struct Resets {
    pub reset: u32,
    pub wdsel: u32,
}

impl Resets {
    /// State after the bootrom has run: everything held in reset except the QSPI pads/IO the flash needs.
    pub fn after_bootrom() -> Self {
        Resets { reset: ALL_RESETS & !(1 << bit::IO_QSPI | 1 << bit::PADS_QSPI), wdsel: 0 }
    }

    pub fn power_on() -> Self {
        Resets { reset: ALL_RESETS, wdsel: 0 }
    }

    pub fn read(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x0 => self.reset,
            0x4 => self.wdsel,
            0x8 => !self.reset & ALL_RESETS,
            _ => return None,
        })
    }

    /// Writes a register. Returns `None` for unknown offsets, otherwise the peripherals that newly entered reset.
    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> Option<u32> {
        match offset {
            0x0 => {
                let old = self.reset;
                mode.update(&mut self.reset, value, ALL_RESETS);
                Some(self.reset & !old)
            }
            0x4 => {
                mode.update(&mut self.wdsel, value, ALL_RESETS);
                Some(0)
            }
            0x8 => Some(0),
            _ => None,
        }
    }

    pub fn in_reset(&self, b: u32) -> bool {
        self.reset & (1 << b) != 0
    }
}

/// PSM bit numbers.
pub mod psm {
    pub const ROSC: u32 = 0;
    pub const XOSC: u32 = 1;
    pub const CLOCKS: u32 = 2;
    pub const RESETS: u32 = 3;
    pub const BUSFABRIC: u32 = 4;
    pub const ROM: u32 = 5;
    pub const SRAM0: u32 = 6;
    pub const XIP: u32 = 12;
    pub const VREG_AND_CHIP_RESET: u32 = 13;
    pub const SIO: u32 = 14;
    pub const PROC0: u32 = 15;
    pub const PROC1: u32 = 16;
    pub const ALL: u32 = 0x1_FFFF;
}

#[derive(Clone, Debug, Default)]
pub struct Psm {
    pub frce_on: u32,
    pub frce_off: u32,
    pub wdsel: u32,
}

/// Effects of a PSM write the machine must carry out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PsmEffect {
    /// Domains that just got forced off (and are therefore reset).
    pub forced_off: u32,
    /// Domains whose force-off was just released (they restart).
    pub released: u32,
}

impl Psm {
    pub fn read(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x0 => self.frce_on,
            0x4 => self.frce_off,
            0x8 => self.wdsel,
            // DONE: everything is powered except what is forced off
            0xC => psm::ALL & !self.frce_off,
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> Option<PsmEffect> {
        match offset {
            0x0 => mode.update(&mut self.frce_on, value, psm::ALL),
            0x4 => {
                let old = self.frce_off;
                mode.update(&mut self.frce_off, value, psm::ALL);
                return Some(PsmEffect { forced_off: self.frce_off & !old, released: old & !self.frce_off });
            }
            0x8 => mode.update(&mut self.wdsel, value, psm::ALL),
            0xC => {}
            _ => return None,
        }
        Some(PsmEffect::default())
    }
}

/// Why the chip last came out of reset, as reported by `VREG_AND_CHIP_RESET.CHIP_RESET`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChipResetCause {
    PowerOn,
    /// RUN pin.
    RunPin,
    /// Debugger-requested PSM restart.
    PsmRestart,
}

#[derive(Clone, Debug)]
pub struct Vreg {
    pub vreg: u32,
    pub bod: u32,
    pub chip_reset: u32,
}

impl Default for Vreg {
    fn default() -> Self {
        Vreg { vreg: 0xB1, bod: 0x91, chip_reset: 1 << 8 }
    }
}

impl Vreg {
    /// Records the cause of a chip-level reset (watchdog resets leave CHIP_RESET alone).
    pub fn record(&mut self, cause: ChipResetCause) {
        self.chip_reset = match cause {
            ChipResetCause::PowerOn => 1 << 8,
            ChipResetCause::RunPin => 1 << 16,
            ChipResetCause::PsmRestart => 1 << 20,
        };
    }

    pub fn read(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x0 => self.vreg | 1 << 12, // ROK
            0x4 => self.bod,
            0x8 => self.chip_reset,
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            0x0 => mode.update(&mut self.vreg, value, 0xF3),
            0x4 => mode.update(&mut self.bod, value, 0xF1),
            // PSM_RESTART_FLAG (bit 24) is write-1-to-clear; the rest is read-only
            0x8 => self.chip_reset &= !(mode.w1c_bits(value) & (1 << 24)),
            _ => return false,
        }
        true
    }
}

/// SYSINFO: chip identification.
pub fn sysinfo_read(offset: u32, revision: u32) -> Option<u32> {
    Some(match offset {
        // REVISION[31:28], PART 0x0002, MANUFACTURER 0x927 (Raspberry Pi)
        0x00 => revision << 28 | 0x0002 << 12 | 0x927,
        // PLATFORM: ASIC
        0x04 => 0x2,
        // GITREF_RP2040: the bootrom puts its top 24 bits at the start of the BOOTSEL serial number, which on
        // production chips reads E0C912...
        0x40 => 0xE0C9_12E8,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resets_and_done() {
        let mut r = Resets::power_on();
        assert_eq!(r.read(0x8), Some(0));
        let entered = r.write(0x0, 1 << bit::TIMER | 1 << bit::UART0, WriteMode::Clear).unwrap();
        assert_eq!(entered, 0);
        assert_eq!(r.read(0x8), Some(1 << bit::TIMER | 1 << bit::UART0));
        assert!(!r.in_reset(bit::TIMER));
        let entered = r.write(0x0, 1 << bit::TIMER, WriteMode::Set).unwrap();
        assert_eq!(entered, 1 << bit::TIMER);
        assert_eq!(Resets::after_bootrom().reset & (1 << bit::IO_QSPI), 0);
    }

    #[test]
    fn psm_force_off_and_done() {
        let mut p = Psm::default();
        let e = p.write(0x4, 1 << psm::SIO | 1 << psm::PROC1, WriteMode::Normal).unwrap();
        assert_eq!(e.forced_off, 1 << psm::SIO | 1 << psm::PROC1);
        assert_eq!(p.read(0xC).unwrap() & (1 << psm::SIO), 0);
        let e = p.write(0x4, 0, WriteMode::Normal).unwrap();
        assert_eq!(e.released, 1 << psm::SIO | 1 << psm::PROC1);
        assert_eq!(p.read(0xC), Some(psm::ALL));
        // write_and_wait style read-back
        p.write(0x0, 1 << psm::PROC0, WriteMode::Normal);
        assert_eq!(p.read(0x0), Some(1 << psm::PROC0));
    }

    #[test]
    fn vreg_chip_reset() {
        let mut v = Vreg::default();
        assert_eq!(v.read(0x8), Some(1 << 8), "HAD_POR");
        v.record(ChipResetCause::RunPin);
        assert_eq!(v.read(0x8), Some(1 << 16));
        v.write(0x0, 0xFF, WriteMode::Normal);
        assert_eq!(v.vreg, 0xF3);
    }

    #[test]
    fn sysinfo() {
        assert_eq!(sysinfo_read(0, 2), Some(0x2000_2927));
        assert_eq!(sysinfo_read(4, 2), Some(2));
    }
}
