//! XIP SSI (Synopsys DW_apb_ssi, datasheet 4.10) in its register-level role: software (boot2, pico-sdk's
//! `flash_do_cmd`) writes frames to DR0 and reads one frame back per frame sent.
//!
//! Until the QSPI flash chip is modelled (M6) the frames go to a responder that returns zeros, so status polls
//! read "not busy, quad enable clear" and boot2 completes. XIP reads of flash do not go through this model: they
//! always return the flash contents.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;

const SR_TFNF: u32 = 1 << 1;
const SR_TFE: u32 = 1 << 2;
const SR_RFNE: u32 = 1 << 3;
const SR_RFF: u32 = 1 << 4;
const FIFO_DEPTH: usize = 16;

#[derive(Clone, Debug)]
pub struct Ssi {
    regs: [u32; 64],
    rx: VecDeque<u32>,
}

impl Default for Ssi {
    fn default() -> Self {
        let mut regs = [0u32; 64];
        // Reset values from the SVD: CTRLR0 0x0007_0000 (DFS_32 = 7), BAUDR 0, SSI_VERSION_ID 0x3430_302a
        regs[0] = 0x0007_0000; // CTRLR0
        regs[0x58 / 4] = 0x5130_5949;
        regs[0x5C / 4] = 0x3430_302A;
        regs[0xF4 / 4] = 0x0300_0000;
        Ssi { regs, rx: VecDeque::new() }
    }
}

impl Ssi {
    /// The state the bootrom's `connect_internal_flash`/`flash_exit_xip` leave the SSI in: enabled, standard SPI,
    /// 8-bit frames, chip select under software control. Used by the HLE flash traps.
    pub fn enter_command_mode(&mut self) {
        self.regs[0x08 / 4] = 0;
        self.regs[0] = 7 << 16; // DFS_32 = 7: 8-bit frames
        self.regs[0x10 / 4] = 1; // SER
        self.regs[0x08 / 4] = 1;
        self.rx.clear();
    }

    fn enabled(&self) -> bool {
        self.regs[0x08 / 4] & 1 != 0
    }

    /// Mask of one data frame (CTRLR0.DFS_32 + 1 bits).
    fn frame_mask(&self) -> u32 {
        let bits = ((self.regs[0] >> 16) & 0x1F) + 1;
        if bits >= 32 { u32::MAX } else { (1 << bits) - 1 }
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            0x20 => 0, // TXFLR: frames leave at once
            0x24 => self.rx.len() as u32,
            0x28 => {
                let mut v = SR_TFNF | SR_TFE;
                if !self.rx.is_empty() {
                    v |= SR_RFNE;
                }
                if self.rx.len() >= FIFO_DEPTH {
                    v |= SR_RFF;
                }
                v
            }
            0x60..=0x9C => self.rx.pop_front().unwrap_or(0),
            0x00..=0xFC => self.regs[(offset / 4) as usize],
            _ => return None,
        })
    }

    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode) -> bool {
        match offset {
            0x60..=0x9C => {
                if self.enabled() && self.rx.len() < FIFO_DEPTH {
                    // The zero responder: one frame back per frame sent
                    let _ = value & self.frame_mask();
                    self.rx.push_back(0);
                }
            }
            0x08 => {
                self.regs[2] = mode.apply(self.regs[2], value) & 1;
                if !self.enabled() {
                    self.rx.clear();
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_echo_zeros_while_enabled() {
        let mut s = Ssi::default();
        s.write(0x60, 0x05, WriteMode::Normal);
        assert_eq!(s.read(0x24), Some(0), "disabled: nothing received");
        s.write(0x08, 1, WriteMode::Normal);
        s.write(0x60, 0x05, WriteMode::Normal);
        s.write(0x60, 0x00, WriteMode::Normal);
        assert_eq!(s.read(0x28).unwrap() & (SR_RFNE | SR_TFE), SR_RFNE | SR_TFE);
        assert_eq!(s.read(0x24), Some(2));
        assert_eq!(s.read(0x60), Some(0));
        assert_eq!(s.read(0x60), Some(0));
        assert_eq!(s.read(0x28).unwrap() & SR_RFNE, 0);
        s.write(0x14, 4, WriteMode::Normal);
        assert_eq!(s.read(0x14), Some(4));
    }
}
