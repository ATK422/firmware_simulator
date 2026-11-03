//! XIP SSI (Synopsys DW_apb_ssi, datasheet 4.10) and the QSPI flash chip behind it.
//!
//! Software (the bootrom's flash routines, boot2, pico-sdk's `flash_do_cmd`) writes frames to DR0; in standard SPI
//! mode each frame is exchanged byte by byte with the [`QspiFlash`] while its chip select is asserted, and the reply
//! frame goes to the RX FIFO. The chip select is either forced through IO_QSPI's SS output override (what the
//! bootrom and pico-sdk do) or driven by the SSI itself, in which case a transaction ends when software waits for the
//! SSI to go idle (reads SR) or disables it. In dual/quad mode a transaction is the instruction and address frames
//! described by SPI_CTRLR0, followed by NDF + 1 data frames read from flash.
//!
//! Memory-mapped XIP reads are served from the flash array directly, but only while the SSI and the chip are in a
//! state where hardware would read flash: SSI enabled, chip select not forced, chip not busy or powered down, and
//! the SSI's read format matching the chip (quad only with QE set; command-less reads only in continuous mode).
//! Otherwise [`Ssi::xip_ready`] says why and the access faults.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;
use sim_core::time::Time;

use super::qspi_flash::QspiFlash;

const SR_TFNF: u32 = 1 << 1;
const SR_TFE: u32 = 1 << 2;
const SR_RFNE: u32 = 1 << 3;
const SR_RFF: u32 = 1 << 4;
const FIFO_DEPTH: usize = 16;

const CTRLR0: usize = 0;
const CTRLR1: usize = 1;
const SSIENR: usize = 2;
const SPI_CTRLR0: usize = 0xF4 / 4;

/// IO_QSPI SS output override (GPIO_QSPI_SS_CTRL.OUTOVER).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CsOverride {
    Normal,
    Invert,
    Low,
    High,
}

#[derive(Clone, Debug)]
pub struct Ssi {
    regs: [u32; 64],
    rx: VecDeque<u32>,
    pub chip: QspiFlash,
    cs_override: CsOverride,
    /// A transaction with the SSI driving chip select is in progress.
    ssi_cs: bool,
    /// Enhanced-mode frames collected so far (instruction, address).
    enhanced: Vec<u32>,
    /// EEPROM read mode (TMOD 3): data frames still to clock in after the command.
    eeprom_left: u32,
}

impl Default for Ssi {
    fn default() -> Self {
        let mut regs = [0u32; 64];
        // Reset values from the SVD: CTRLR0 0x0007_0000 (DFS_32 = 7), BAUDR 0, SSI_VERSION_ID 0x3430_302a
        regs[CTRLR0] = 0x0007_0000;
        regs[0x58 / 4] = 0x5130_5949;
        regs[0x5C / 4] = 0x3430_302A;
        regs[SPI_CTRLR0] = 0x0300_0000;
        Ssi {
            regs,
            rx: VecDeque::new(),
            chip: QspiFlash::default(),
            cs_override: CsOverride::Normal,
            ssi_cs: false,
            enhanced: Vec::new(),
            eeprom_left: 0,
        }
    }
}

impl Ssi {
    /// Leaves the SSI and the chip as boot2 does (quad I/O continuous-read XIP), for direct boot.
    pub fn preset_boot2_xip(&mut self) {
        self.regs[SSIENR] = 0;
        self.regs[0x14 / 4] = 4; // BAUDR
        // Quad frame format, 32-bit frames, EEPROM read mode
        self.regs[CTRLR0] = 2 << 21 | 31 << 16 | 3 << 8;
        self.regs[CTRLR1] = 0;
        // XIP_CMD A0 (mode bits), no instruction, 32-bit address (with mode), 4 wait cycles, address in quad
        self.regs[SPI_CTRLR0] = 0xA0 << 24 | 4 << 11 | 8 << 2 | 2;
        self.regs[SSIENR] = 1;
        self.chip.preset_quad_xip();
    }

    /// The state the bootrom's `connect_internal_flash`/`flash_exit_xip` leave the SSI in: enabled, standard SPI,
    /// 8-bit frames. Used by the HLE flash traps.
    pub fn enter_command_mode(&mut self) {
        self.regs[SSIENR] = 0;
        self.regs[CTRLR0] = 7 << 16; // DFS_32 = 7: 8-bit frames
        self.regs[0x10 / 4] = 1; // SER
        self.regs[SSIENR] = 1;
        self.chip.continuous = false;
        self.rx.clear();
    }

    fn enabled(&self) -> bool {
        self.regs[SSIENR] & 1 != 0
    }

    fn frame_bits(&self) -> u32 {
        ((self.regs[CTRLR0] >> 16) & 0x1F) + 1
    }

    fn frame_format(&self) -> u32 {
        (self.regs[CTRLR0] >> 21) & 3
    }

    fn cs_asserted(&self) -> bool {
        match self.cs_override {
            CsOverride::Low => true,
            CsOverride::High => false,
            CsOverride::Normal => self.ssi_cs,
            CsOverride::Invert => !self.ssi_cs,
        }
    }

    fn update_cs(&mut self, before: bool, now: Time, data: &mut [u8]) {
        let after = self.cs_asserted();
        if after != before {
            self.chip.select(after, now, data);
        }
    }

    /// IO_QSPI's SS override changed.
    pub fn set_cs_override(&mut self, o: CsOverride, now: Time, data: &mut [u8]) {
        let before = self.cs_asserted();
        self.cs_override = o;
        self.update_cs(before, now, data);
    }

    /// Ends a transaction with the SSI driving chip select.
    fn end_ssi_transaction(&mut self, now: Time, data: &mut [u8]) {
        if self.ssi_cs {
            let before = self.cs_asserted();
            self.ssi_cs = false;
            self.update_cs(before, now, data);
        }
    }

    /// Whether a memory-mapped XIP read would return flash data now, or why not.
    pub fn xip_ready(&self, now: Time) -> Result<(), &'static str> {
        if self.chip.busy(now) {
            return Err("the flash is busy programming or erasing");
        }
        self.xip_config()
    }

    /// The part of [`Ssi::xip_ready`] that only changes with register writes and chip commands.
    pub fn xip_config(&self) -> Result<(), &'static str> {
        if !self.enabled() {
            return Err("the SSI is disabled");
        }
        if self.cs_override != CsOverride::Normal {
            return Err("the flash chip select is forced through IO_QSPI");
        }
        if self.chip.power_down {
            return Err("the flash is powered down");
        }
        let spi = self.regs[SPI_CTRLR0];
        let inst_l = (spi >> 8) & 3;
        match self.frame_format() {
            0 if self.chip.continuous => Err("the flash is in continuous read mode but the SSI reads in standard SPI"),
            2 if !self.chip.quad_enabled() => Err("quad XIP with the flash's QE bit clear"),
            1 | 2 if inst_l == 0 && !self.chip.continuous => {
                Err("XIP without a command, but the flash is not in continuous read mode")
            }
            _ => Ok(()),
        }
    }

    pub fn read(&mut self, offset: u32, now: Time, data: &mut [u8]) -> Option<u32> {
        Some(match offset {
            0x20 => 0, // TXFLR: frames leave at once
            0x24 => self.rx.len() as u32,
            0x28 => {
                // EEPROM read: the data frames clock in once the command has gone out
                self.eeprom_fill(now, data);
                // Software waiting for the SSI to finish: an SSI-driven transaction is over
                if self.eeprom_left == 0 {
                    self.end_ssi_transaction(now, data);
                }
                self.sr()
            }
            0x60..=0x9C => {
                self.eeprom_fill(now, data);
                let v = self.rx.pop_front().unwrap_or(0);
                self.eeprom_fill(now, data);
                v
            }
            0x00..=0xFC => self.regs[(offset / 4) as usize],
            _ => return None,
        })
    }

    fn sr(&self) -> u32 {
        let mut v = SR_TFNF | SR_TFE;
        if !self.rx.is_empty() {
            v |= SR_RFNE;
        }
        if self.rx.len() >= FIFO_DEPTH {
            v |= SR_RFF;
        }
        v
    }

    /// Side-effect free read.
    pub fn peek(&self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x20 => 0,
            0x24 => self.rx.len() as u32,
            0x28 => self.sr(),
            0x60..=0x9C => self.rx.front().copied().unwrap_or(0),
            0x00..=0xFC => self.regs[(offset / 4) as usize],
            _ => return None,
        })
    }

    /// Clocks in EEPROM-read data frames while the RX FIFO has room (the SSI stalls the clock when it is full).
    fn eeprom_fill(&mut self, now: Time, data: &mut [u8]) {
        let bytes = self.frame_bits().div_ceil(8);
        while self.eeprom_left > 0 && self.rx.len() < FIFO_DEPTH {
            let mut reply = 0u32;
            for _ in 0..bytes {
                let r = if self.cs_asserted() { self.chip.exchange(0, now, data) } else { 0xFF };
                reply = reply << 8 | r as u32;
            }
            self.rx.push_back(reply);
            self.eeprom_left -= 1;
        }
    }

    fn push_rx(&mut self, v: u32) {
        if self.rx.len() < FIFO_DEPTH {
            self.rx.push_back(v);
        }
    }

    fn standard_frame(&mut self, value: u32, now: Time, data: &mut [u8]) {
        if self.cs_override == CsOverride::Normal && !self.ssi_cs {
            let before = self.cs_asserted();
            self.ssi_cs = true;
            self.update_cs(before, now, data);
        }
        let bits = self.frame_bits();
        let bytes = bits.div_ceil(8);
        let mut reply = 0u32;
        for i in (0..bytes).rev() {
            let b = (value >> (8 * i)) as u8;
            let r = if self.cs_asserted() { self.chip.exchange(b, now, data) } else { 0xFF };
            reply = reply << 8 | r as u32;
        }
        let tmod = (self.regs[CTRLR0] >> 8) & 3;
        match tmod {
            // Transmit only
            1 => {}
            // EEPROM read: what comes back during the command is dropped; NDF + 1 frames follow it
            3 => self.eeprom_left = (self.regs[CTRLR1] & 0xFFFF) + 1,
            _ => {
                let mask = if bits >= 32 { u32::MAX } else { (1 << bits) - 1 };
                self.push_rx(reply & mask);
            }
        }
    }

    fn enhanced_frame(&mut self, value: u32, now: Time, data: &mut [u8]) {
        let spi = self.regs[SPI_CTRLR0];
        let inst_l = (spi >> 8) & 3;
        let addr_bits = ((spi >> 2) & 0xF) * 4;
        self.enhanced.push(value);
        let needed = 1 + (inst_l != 0) as usize;
        if self.enhanced.len() < needed {
            return;
        }
        let frames = std::mem::take(&mut self.enhanced);
        let cmd = (inst_l != 0).then(|| frames[0] as u8);
        let a = *frames.last().unwrap();
        let (addr, mode) = if addr_bits >= 32 { (a >> 8, a as u8) } else { (a, 0) };
        // One chip select cycle for the whole transaction
        self.chip.select(true, now, data);
        let start = self.chip.enhanced_read(cmd, addr, mode, now);
        let n = (self.regs[CTRLR1] & 0xFFFF) + 1;
        let bytes = self.frame_bits().div_ceil(8) as usize;
        for k in 0..n as usize {
            let v = match start {
                Some(s) => (0..bytes).fold(0u32, |w, j| w | (data[(s + k * bytes + j) % data.len()] as u32) << (8 * j)),
                None => 0,
            };
            self.push_rx(v);
        }
        // Continuous mode survives the deselect; anything else ends here
        let continuous = self.chip.continuous;
        self.chip.select(false, now, data);
        self.chip.continuous = continuous;
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode, now: Time, data: &mut [u8]) -> bool {
        match offset {
            0x60..=0x9C => {
                if self.enabled() {
                    if self.frame_format() == 0 {
                        self.standard_frame(value, now, data);
                    } else {
                        self.enhanced_frame(value, now, data);
                    }
                }
            }
            0x08 => {
                self.regs[SSIENR] = mode.apply(self.regs[SSIENR], value) & 1;
                if !self.enabled() {
                    self.rx.clear();
                    self.enhanced.clear();
                    self.eeprom_left = 0;
                    self.end_ssi_transaction(now, data);
                }
            }
            0x20 | 0x24 | 0x28 | 0x58 | 0x5C => {}
            0x00..=0xFC => {
                let i = (offset / 4) as usize;
                self.regs[i] = mode.apply(self.regs[i], value);
            }
            _ => return false,
        }
        true
    }
}

/// IO_QSPI: the six QSPI pins' control registers. The SS pin reads high (the flash pulls its chip select up), so the
/// bootrom sees BOOTSEL released, unless `bootsel_pressed`.
#[derive(Clone, Debug, Default)]
pub struct IoQspi {
    ctrl: [u32; 6],
    other: [u32; 16],
    pub bootsel_pressed: bool,
}

impl IoQspi {
    /// SS output override, from GPIO_QSPI_SS_CTRL.OUTOVER.
    pub fn cs_override(&self) -> CsOverride {
        match (self.ctrl[1] >> 8) & 3 {
            0 => CsOverride::Normal,
            1 => CsOverride::Invert,
            2 => CsOverride::Low,
            _ => CsOverride::High,
        }
    }

    /// Levels of the QSPI pins as SIO GPIO_HI_IN shows them (SCLK, SS, SD0-3): SS and the data lines idle high
    /// (pulled up), SS low while BOOTSEL is held.
    pub fn levels(&self) -> u32 {
        let ss = !self.bootsel_pressed as u32;
        0b11_1100 | ss << 1
    }

    pub fn read(&self, offset: u32) -> Option<u32> {
        let pin = (offset / 8) as usize;
        Some(match offset {
            0x00..=0x2C if offset % 8 == 4 => self.ctrl[pin],
            0x00..=0x2C => {
                // STATUS: INFROMPAD (bit 17) high for SS unless BOOTSEL is held
                let high = pin != 1 || !self.bootsel_pressed;
                (high as u32) << 17
            }
            0x30..=0x6C => self.other[((offset - 0x30) / 4) as usize],
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            0x00..=0x2C if offset % 8 == 4 => {
                let pin = (offset / 8) as usize;
                self.ctrl[pin] = mode.apply(self.ctrl[pin], value) & 0x3003_331F;
            }
            0x00..=0x2C => {}
            0x30..=0x6C => {
                let i = ((offset - 0x30) / 4) as usize;
                self.other[i] = mode.apply(self.other[i], value);
            }
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(s: &mut Ssi, off: u32, v: u32, data: &mut [u8]) {
        assert!(s.write(off, v, WriteMode::Normal, Time::ZERO, data));
    }

    fn command_mode(s: &mut Ssi, data: &mut [u8]) {
        w(s, 0x08, 0, data);
        w(s, 0x00, 7 << 16, data);
        w(s, 0x08, 1, data);
    }

    #[test]
    fn standard_frames_with_forced_chip_select() {
        let mut data = vec![0xFFu8; 2 << 20];
        let mut s = Ssi::default();
        command_mode(&mut s, &mut data);
        s.set_cs_override(CsOverride::Low, Time::ZERO, &mut data);
        for b in [0x9F, 0, 0, 0] {
            w(&mut s, 0x60, b, &mut data);
        }
        let got: Vec<u32> = (0..4).map(|_| s.read(0x60, Time::ZERO, &mut data).unwrap()).collect();
        assert_eq!(got, [0xFF, 0xEF, 0x40, 0x15]);
        // SR polls do not end a transaction while chip select is forced
        s.read(0x28, Time::ZERO, &mut data);
        s.set_cs_override(CsOverride::High, Time::ZERO, &mut data);
        s.set_cs_override(CsOverride::Normal, Time::ZERO, &mut data);
        assert!(s.xip_ready(Time::ZERO).is_ok(), "standard SPI XIP with the chip idle");
    }

    #[test]
    fn eeprom_read_mode_clocks_in_the_data_after_the_command() {
        // embassy-rp's unique ID read: TMOD 3, NDF = 4 dummy + 8 data - 1, then the command byte 0x4B
        let mut data = vec![0xFFu8; 2 << 20];
        let mut s = Ssi::default();
        w(&mut s, 0x08, 0, &mut data);
        w(&mut s, 0x00, 7 << 16 | 3 << 8, &mut data);
        w(&mut s, 0x04, 4 + 8 - 1, &mut data);
        w(&mut s, 0x08, 1, &mut data);
        w(&mut s, 0x60, 0x4B, &mut data);
        assert_eq!(s.peek(0x24), Some(0), "nothing received during the command");
        let mut got = Vec::new();
        while got.len() < 12 {
            if s.read(0x28, Time::ZERO, &mut data).unwrap() & SR_RFNE != 0 {
                got.push(s.read(0x60, Time::ZERO, &mut data).unwrap() as u8);
            }
        }
        assert_eq!(got[4..], crate::periph::qspi_flash::UNIQUE_ID, "dummy bytes, then the unique ID");
        assert_eq!(s.peek(0x24), Some(0));
        s.read(0x28, Time::ZERO, &mut data);
        assert!(!s.ssi_cs, "the transaction ended");
    }

    #[test]
    fn ssi_driven_transactions_end_on_sr_read() {
        let mut data = vec![0xFFu8; 2 << 20];
        let mut s = Ssi::default();
        command_mode(&mut s, &mut data);
        // boot2's QE write: WREN, then WRSR 00 02, each followed by a wait for the SSI
        w(&mut s, 0x60, 0x06, &mut data);
        s.read(0x28, Time::ZERO, &mut data);
        for b in [0x01, 0x00, 0x02] {
            w(&mut s, 0x60, b, &mut data);
        }
        s.read(0x28, Time::ZERO, &mut data);
        assert!(s.chip.quad_enabled());
    }

    #[test]
    fn quad_continuous_setup_and_xip_checks() {
        let mut data: Vec<u8> = (0..(2u32 << 20)).map(|i| i as u8).collect();
        let mut s = Ssi::default();
        // Quad XIP before QE is set: not readable
        w(&mut s, 0x08, 0, &mut data);
        w(&mut s, 0x00, 2 << 21 | 31 << 16 | 3 << 8, &mut data);
        w(&mut s, 0xF4, 0xEB << 24 | 4 << 11 | 2 << 8 | 8 << 2 | 1, &mut data);
        w(&mut s, 0x08, 1, &mut data);
        assert!(s.xip_ready(Time::ZERO).is_err());
        s.chip.sr2 |= 2;
        assert!(s.xip_ready(Time::ZERO).is_ok());
        // The dummy read that enters continuous mode: EBh, address 0 with mode A0
        w(&mut s, 0x60, 0xEB, &mut data);
        w(&mut s, 0x60, 0xA0, &mut data);
        assert_eq!(s.read(0x60, Time::ZERO, &mut data), Some(0x0302_0100));
        assert!(s.chip.continuous);
        // Then command-less XIP
        w(&mut s, 0x08, 0, &mut data);
        w(&mut s, 0xF4, 0xA0 << 24 | 4 << 11 | 8 << 2 | 2, &mut data);
        w(&mut s, 0x08, 1, &mut data);
        assert!(s.xip_ready(Time::ZERO).is_ok());
        // Standard SPI XIP while the chip is still in continuous mode is not
        w(&mut s, 0x08, 0, &mut data);
        w(&mut s, 0x00, 31 << 16 | 3 << 8, &mut data);
        w(&mut s, 0x08, 1, &mut data);
        assert!(s.xip_ready(Time::ZERO).is_err());
        let mut d = Ssi::default();
        d.preset_boot2_xip();
        assert!(d.xip_ready(Time::ZERO).is_ok());
    }

    #[test]
    fn io_qspi_status_and_override() {
        let mut q = IoQspi::default();
        assert_eq!(q.read(0x08).unwrap() >> 17 & 1, 1, "SS high: BOOTSEL released");
        q.bootsel_pressed = true;
        assert_eq!(q.read(0x08).unwrap() >> 17 & 1, 0);
        q.write(0x0C, 2 << 8, WriteMode::Normal);
        assert_eq!(q.cs_override(), CsOverride::Low);
        q.write(0x0C, 3 << 8, WriteMode::Set);
        assert_eq!(q.cs_override(), CsOverride::High);
    }
}
