//! RP2040 address map (datasheet section 2.2).

pub const ROM_BASE: u32 = 0x0000_0000;
pub const ROM_SIZE: usize = 16 * 1024;

pub const XIP_BASE: u32 = 0x1000_0000;
pub const XIP_NOALLOC_BASE: u32 = 0x1100_0000;
pub const XIP_NOCACHE_BASE: u32 = 0x1200_0000;
pub const XIP_NOCACHE_NOALLOC_BASE: u32 = 0x1300_0000;
pub const XIP_CTRL_BASE: u32 = 0x1400_0000;
pub const XIP_SRAM_BASE: u32 = 0x1500_0000;
pub const XIP_SRAM_SIZE: usize = 16 * 1024;
pub const XIP_SSI_BASE: u32 = 0x1800_0000;

pub const SRAM_BASE: u32 = 0x2000_0000;
/// Striped SRAM0-3 (256 KiB) followed by SRAM4 and SRAM5 (4 KiB each).
pub const SRAM_SIZE: usize = 264 * 1024;
pub const SRAM_END: u32 = SRAM_BASE + SRAM_SIZE as u32;
/// Non-striped alias of SRAM0-3: bank n at `0x2100_0000 + n * 0x1_0000`.
pub const SRAM0_NONSTRIPED_BASE: u32 = 0x2100_0000;

pub const APB_BASE: u32 = 0x4000_0000;
pub const SYSINFO_BASE: u32 = 0x4000_0000;
pub const SYSCFG_BASE: u32 = 0x4000_4000;
pub const CLOCKS_BASE: u32 = 0x4000_8000;
pub const RESETS_BASE: u32 = 0x4000_C000;
pub const PSM_BASE: u32 = 0x4001_0000;
pub const IO_BANK0_BASE: u32 = 0x4001_4000;
pub const IO_QSPI_BASE: u32 = 0x4001_8000;
pub const PADS_BANK0_BASE: u32 = 0x4001_C000;
pub const PADS_QSPI_BASE: u32 = 0x4002_0000;
pub const XOSC_BASE: u32 = 0x4002_4000;
pub const PLL_SYS_BASE: u32 = 0x4002_8000;
pub const PLL_USB_BASE: u32 = 0x4002_C000;
pub const BUSCTRL_BASE: u32 = 0x4003_0000;
pub const UART0_BASE: u32 = 0x4003_4000;
pub const UART1_BASE: u32 = 0x4003_8000;
pub const SPI0_BASE: u32 = 0x4003_C000;
pub const SPI1_BASE: u32 = 0x4004_0000;
pub const I2C0_BASE: u32 = 0x4004_4000;
pub const I2C1_BASE: u32 = 0x4004_8000;
pub const ADC_BASE: u32 = 0x4004_C000;
pub const PWM_BASE: u32 = 0x4005_0000;
pub const TIMER_BASE: u32 = 0x4005_4000;
pub const WATCHDOG_BASE: u32 = 0x4005_8000;
pub const RTC_BASE: u32 = 0x4005_C000;
pub const ROSC_BASE: u32 = 0x4006_0000;
pub const VREG_AND_CHIP_RESET_BASE: u32 = 0x4006_4000;
pub const TBMAN_BASE: u32 = 0x4006_C000;

pub const DMA_BASE: u32 = 0x5000_0000;
pub const USBCTRL_DPRAM_BASE: u32 = 0x5010_0000;
pub const USBCTRL_REGS_BASE: u32 = 0x5011_0000;
pub const PIO0_BASE: u32 = 0x5020_0000;
pub const PIO1_BASE: u32 = 0x5030_0000;
pub const XIP_AUX_BASE: u32 = 0x5040_0000;

pub const SIO_BASE: u32 = 0xD000_0000;
pub const PPB_BASE: u32 = 0xE000_0000;

/// Number of user GPIOs in bank 0.
pub const NUM_GPIOS: usize = 30;

/// Offset of an SRAM address from `SRAM_BASE` in the striped layout, translating the non-striped bank aliases.
pub fn sram_offset(addr: u32) -> Option<usize> {
    if (SRAM_BASE..SRAM_END).contains(&addr) {
        return Some((addr - SRAM_BASE) as usize);
    }
    if (SRAM0_NONSTRIPED_BASE..SRAM0_NONSTRIPED_BASE + 0x4_0000).contains(&addr) {
        // Bank b, byte offset o within the bank -> striped: word index = (o / 4) * 4 + b
        let rel = addr - SRAM0_NONSTRIPED_BASE;
        let bank = rel >> 16;
        let o = rel & 0xFFFF;
        let word = (o >> 2) * 4 + bank;
        return Some((word * 4 + (o & 3)) as usize);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn striping() {
        assert_eq!(sram_offset(0x2000_0000), Some(0));
        assert_eq!(sram_offset(0x2004_1FFF), Some(SRAM_SIZE - 1));
        assert_eq!(sram_offset(0x2004_2000), None);
        // Bank 0 word 0 is striped word 0; bank 1 word 0 is striped word 1; bank 0 word 1 is striped word 4
        assert_eq!(sram_offset(0x2100_0000), Some(0));
        assert_eq!(sram_offset(0x2101_0000), Some(4));
        assert_eq!(sram_offset(0x2100_0004), Some(16));
        assert_eq!(sram_offset(0x2103_0007), Some(16 + 12 + 3));
    }
}
