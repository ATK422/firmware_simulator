//! Single-cycle IO block (SIO, datasheet 2.3.1): per-core CPUID and the GPIO registers.

use sim_core::bus::WriteMode;

pub const CPUID: u32 = 0x000;
pub const GPIO_IN: u32 = 0x004;
pub const GPIO_HI_IN: u32 = 0x008;
pub const GPIO_OUT: u32 = 0x010;
pub const GPIO_OUT_SET: u32 = 0x014;
pub const GPIO_OUT_CLR: u32 = 0x018;
pub const GPIO_OUT_XOR: u32 = 0x01C;
pub const GPIO_OE: u32 = 0x020;
pub const GPIO_OE_SET: u32 = 0x024;
pub const GPIO_OE_CLR: u32 = 0x028;
pub const GPIO_OE_XOR: u32 = 0x02C;
pub const GPIO_HI_OUT: u32 = 0x030;
pub const GPIO_HI_OUT_SET: u32 = 0x034;
pub const GPIO_HI_OUT_CLR: u32 = 0x038;
pub const GPIO_HI_OUT_XOR: u32 = 0x03C;
pub const GPIO_HI_OE: u32 = 0x040;
pub const GPIO_HI_OE_SET: u32 = 0x044;
pub const GPIO_HI_OE_CLR: u32 = 0x048;
pub const GPIO_HI_OE_XOR: u32 = 0x04C;

const BANK0_MASK: u32 = (1 << 30) - 1;
const QSPI_MASK: u32 = (1 << 6) - 1;

#[derive(Clone, Debug, Default)]
pub struct Sio {
    pub gpio_out: u32,
    pub gpio_oe: u32,
    pub gpio_hi_out: u32,
    pub gpio_hi_oe: u32,
}

/// Applies a write through one of the four `REG`, `REG_SET`, `REG_CLR`, `REG_XOR` addresses (in that order).
fn update(reg: &mut u32, which: u32, value: u32, mask: u32) {
    let mode = match which {
        0 => WriteMode::Normal,
        1 => WriteMode::Set,
        2 => WriteMode::Clear,
        _ => WriteMode::Xor,
    };
    *reg = mode.apply(*reg, value) & mask;
}

impl Sio {
    /// Reads a GPIO register. `gpio_in` / `gpio_hi_in` are the current input levels of the two banks.
    pub fn read(&self, core: u8, offset: u32, gpio_in: u32, gpio_hi_in: u32) -> Option<u32> {
        Some(match offset {
            CPUID => core as u32,
            GPIO_IN => gpio_in & BANK0_MASK,
            GPIO_HI_IN => gpio_hi_in & QSPI_MASK,
            GPIO_OUT..=GPIO_OUT_XOR => self.gpio_out,
            GPIO_OE..=GPIO_OE_XOR => self.gpio_oe,
            GPIO_HI_OUT..=GPIO_HI_OUT_XOR => self.gpio_hi_out,
            GPIO_HI_OE..=GPIO_HI_OE_XOR => self.gpio_hi_oe,
            _ => return None,
        })
    }

    /// Writes a GPIO register. Returns false for offsets not handled here.
    pub fn write(&mut self, offset: u32, value: u32) -> bool {
        let which = (offset >> 2) & 3;
        match offset {
            GPIO_OUT..=GPIO_OUT_XOR => update(&mut self.gpio_out, which, value, BANK0_MASK),
            GPIO_OE..=GPIO_OE_XOR => update(&mut self.gpio_oe, which, value, BANK0_MASK),
            GPIO_HI_OUT..=GPIO_HI_OUT_XOR => update(&mut self.gpio_hi_out, which, value, QSPI_MASK),
            GPIO_HI_OE..=GPIO_HI_OE_XOR => update(&mut self.gpio_hi_oe, which, value, QSPI_MASK),
            // CPUID and the input registers are read-only
            CPUID | GPIO_IN | GPIO_HI_IN => {}
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpuid_per_core() {
        let s = Sio::default();
        assert_eq!(s.read(0, CPUID, 0, 0), Some(0));
        assert_eq!(s.read(1, CPUID, 0, 0), Some(1));
    }

    #[test]
    fn gpio_out_set_clr_xor() {
        let mut s = Sio::default();
        s.write(GPIO_OUT, 0xFFFF_FFFF);
        assert_eq!(s.gpio_out, BANK0_MASK, "only 30 GPIOs");
        s.write(GPIO_OUT_CLR, 0xF);
        assert_eq!(s.gpio_out, BANK0_MASK & !0xF);
        s.write(GPIO_OUT_SET, 0x1);
        assert_eq!(s.gpio_out & 0xF, 0x1);
        s.write(GPIO_OUT_XOR, 0x3);
        assert_eq!(s.gpio_out & 0xF, 0x2);
        assert_eq!(s.read(0, GPIO_OUT_XOR, 0, 0), Some(s.gpio_out), "aliases read the register");
        s.write(GPIO_OE_SET, 1 << 25);
        s.write(GPIO_OE_XOR, 1 << 24 | 1 << 25);
        assert_eq!(s.gpio_oe, 1 << 24);
        s.write(GPIO_HI_OUT, 0xFF);
        assert_eq!(s.gpio_hi_out, 0x3F);
    }

    #[test]
    fn inputs() {
        let mut s = Sio::default();
        assert_eq!(s.read(0, GPIO_IN, 0xFFFF_FFFF, 0), Some(BANK0_MASK));
        assert_eq!(s.read(0, GPIO_HI_IN, 0, 0xFF), Some(QSPI_MASK));
        assert!(s.write(GPIO_IN, 5), "read-only writes are accepted and ignored");
        assert!(!s.write(0x200, 0));
    }
}
