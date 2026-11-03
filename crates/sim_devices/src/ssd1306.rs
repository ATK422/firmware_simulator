//! Solomon Systech SSD1306 OLED controller (datasheet rev 1.1) on I2C, with a renderer for its panel.
//!
//! Each I2C write is a sequence of control bytes (Co, D/C#) and command or display data bytes. Command parsing is
//! stateful across writes, so a command and its parameters may arrive in separate transfers. The 128x64 GDDRAM is
//! written in page, horizontal or vertical addressing mode; as the datasheet specifies, the page-mode commands
//! (B0h-B7h page start, 00h-1Fh column start) only move the write pointer in page addressing mode, and the
//! window commands (21h, 22h) only in horizontal and vertical mode.
//!
//! [`Ssd1306::frame`] gives what the panel shows (multiplex ratio, start line, display offset, segment remap, COM
//! scan direction, inverse and entire-on applied), [`Ssd1306::render_text`] draws it with Unicode half blocks and
//! [`Ssd1306::to_png`] encodes it. A panel mounted with A1h + C8h (the usual module orientation) shows RAM page 0
//! at the top and column 0 at the left.

use crate::DeviceCtx;
use crate::i2c::I2cDevice;

pub const DEFAULT_ADDRESS: u8 = 0x3C;
pub const WIDTH: usize = 128;
pub const RAM_PAGES: usize = 8;

#[derive(Clone, Debug)]
pub struct Ssd1306 {
    pub address: u8,
    pub ram: [[u8; WIDTH]; RAM_PAGES],
    pub display_on: bool,
    pub contrast: u8,
    pub inverse: bool,
    pub entire_on: bool,
    /// 0 horizontal, 1 vertical, 2 page.
    pub addressing_mode: u8,
    pub col: usize,
    pub page: usize,
    pub col_range: (usize, usize),
    pub page_range: (usize, usize),
    pub start_line: usize,
    pub segment_remap: bool,
    pub com_remap: bool,
    pub mux_ratio: usize,
    pub display_offset: usize,
    pub com_pins: u8,
    pub charge_pump: bool,
    /// Let the page-mode positioning commands (B0h-B7h, 00h-1Fh) move the pointer in every addressing mode, for
    /// controllers (or clones) that behave that way. Off by default: the datasheet limits them to page mode.
    pub page_commands_in_any_mode: bool,
    /// Command bytes waiting for their parameters.
    pending: Vec<u8>,
    /// Within the current write: whether a control byte is expected, and whether the stream is data.
    expect_control: bool,
    continuation: bool,
    data_mode: bool,
    /// Display data bytes received, and page-mode commands ignored outside page mode.
    pub data_bytes: u64,
    pub ignored_page_commands: u64,
}

impl Default for Ssd1306 {
    fn default() -> Self {
        Ssd1306 {
            address: DEFAULT_ADDRESS,
            ram: [[0; WIDTH]; RAM_PAGES],
            display_on: false,
            contrast: 0x7F,
            inverse: false,
            entire_on: false,
            addressing_mode: 2,
            col: 0,
            page: 0,
            col_range: (0, 127),
            page_range: (0, 7),
            start_line: 0,
            segment_remap: false,
            com_remap: false,
            mux_ratio: 64,
            display_offset: 0,
            com_pins: 0x12,
            charge_pump: false,
            page_commands_in_any_mode: false,
            pending: Vec::new(),
            expect_control: true,
            continuation: false,
            data_mode: false,
            data_bytes: 0,
            ignored_page_commands: 0,
        }
    }
}

/// Number of parameter bytes after a command byte.
fn params(cmd: u8) -> usize {
    match cmd {
        0x81 | 0x20 | 0xA8 | 0xD3 | 0xD5 | 0xD9 | 0xDA | 0xDB | 0x8D => 1,
        0x21 | 0x22 | 0xA3 => 2,
        0x29 | 0x2A => 5,
        0x26 | 0x27 => 6,
        _ => 0,
    }
}

impl Ssd1306 {
    pub fn new() -> Self {
        Self::default()
    }

    fn command(&mut self, c: &[u8]) {
        let page_mode = self.addressing_mode == 2 || self.page_commands_in_any_mode;
        match c[0] {
            0x00..=0x0F if page_mode => self.col = (self.col & 0xF0) | (c[0] & 0xF) as usize,
            0x10..=0x1F if page_mode => self.col = (self.col & 0x0F) | ((c[0] & 0x7) as usize) << 4,
            0xB0..=0xB7 if page_mode => self.page = (c[0] & 7) as usize,
            0x00..=0x1F | 0xB0..=0xB7 => self.ignored_page_commands += 1,
            0x20 => self.addressing_mode = (c[1] & 3).min(2),
            0x21 if self.addressing_mode != 2 => {
                self.col_range = ((c[1] & 0x7F) as usize, (c[2] & 0x7F) as usize);
                self.col = self.col_range.0;
            }
            0x22 if self.addressing_mode != 2 => {
                self.page_range = ((c[1] & 7) as usize, (c[2] & 7) as usize);
                self.page = self.page_range.0;
            }
            0x21 | 0x22 => {}
            0x40..=0x7F => self.start_line = (c[0] & 0x3F) as usize,
            0x81 => self.contrast = c[1],
            0x8D => self.charge_pump = c[1] & 0x04 != 0,
            0xA0 | 0xA1 => self.segment_remap = c[0] & 1 != 0,
            0xA4 | 0xA5 => self.entire_on = c[0] & 1 != 0,
            0xA6 | 0xA7 => self.inverse = c[0] & 1 != 0,
            0xA8 => self.mux_ratio = ((c[1] & 0x3F) as usize + 1).max(16),
            0xAE | 0xAF => self.display_on = c[0] & 1 != 0,
            0xC0..=0xCF => self.com_remap = c[0] & 0x08 != 0,
            0xD3 => self.display_offset = (c[1] & 0x3F) as usize,
            0xDA => self.com_pins = c[1],
            // Timing, scrolling, NOP: accepted, no visible effect modelled
            _ => {}
        }
    }

    fn command_byte(&mut self, b: u8) {
        self.pending.push(b);
        if self.pending.len() > params(self.pending[0]) {
            let c = std::mem::take(&mut self.pending);
            self.command(&c);
        }
    }

    fn data(&mut self, b: u8) {
        self.data_bytes += 1;
        self.ram[self.page][self.col] = b;
        match self.addressing_mode {
            2 => self.col = (self.col + 1) % WIDTH,
            0 => {
                if self.col >= self.col_range.1 {
                    self.col = self.col_range.0;
                    self.page = if self.page >= self.page_range.1 { self.page_range.0 } else { self.page + 1 };
                } else {
                    self.col += 1;
                }
            }
            _ => {
                if self.page >= self.page_range.1 {
                    self.page = self.page_range.0;
                    self.col = if self.col >= self.col_range.1 { self.col_range.0 } else { self.col + 1 };
                } else {
                    self.page += 1;
                }
            }
        }
    }

    fn ram_pixel(&self, x: usize, row: usize) -> bool {
        self.ram[row / 8][x] & (1 << (row % 8)) != 0
    }

    /// What the panel shows: `mux_ratio` rows of 128 pixels (empty when the display is off).
    pub fn frame(&self) -> Vec<Vec<bool>> {
        let rows = self.mux_ratio;
        (0..rows)
            .map(|y| {
                // Row y of the (upright, A1 + C8) panel is driven by COM y; COM n shows RAM row
                // (start line + n + offset). Without the remaps the image is mirrored.
                let com = if self.com_remap { y } else { rows - 1 - y };
                let row = (self.start_line + com + self.display_offset) % 64;
                (0..WIDTH)
                    .map(|x| {
                        if !self.display_on {
                            return false;
                        }
                        if self.entire_on {
                            return !self.inverse;
                        }
                        let col = if self.segment_remap { x } else { WIDTH - 1 - x };
                        self.ram_pixel(col, row) != self.inverse
                    })
                    .collect()
            })
            .collect()
    }

    /// The frame as text, two pixel rows per line.
    pub fn render_text(&self) -> String {
        let f = self.frame();
        let mut s = String::new();
        for pair in f.chunks(2) {
            for x in 0..WIDTH {
                let top = pair[0][x];
                let bottom = pair.get(1).is_some_and(|r| r[x]);
                s.push(match (top, bottom) {
                    (true, true) => '█',
                    (true, false) => '▀',
                    (false, true) => '▄',
                    (false, false) => ' ',
                });
            }
            s.push('\n');
        }
        s
    }

    /// The frame as a PNG, each pixel `scale` x `scale`.
    pub fn to_png(&self, scale: usize) -> Vec<u8> {
        let f = self.frame();
        let scale = scale.max(1);
        let (w, h) = (WIDTH * scale, f.len() * scale);
        let mut px = vec![0u8; w * h];
        for (y, row) in f.iter().enumerate() {
            for (x, &on) in row.iter().enumerate() {
                if on {
                    for dy in 0..scale {
                        let start = (y * scale + dy) * w + x * scale;
                        px[start..start + scale].fill(0xFF);
                    }
                }
            }
        }
        crate::png::greyscale(w as u32, h as u32, &px)
    }
}

impl I2cDevice for Ssd1306 {
    fn name(&self) -> String {
        "SSD1306".into()
    }

    fn responds_to(&self, address: u8) -> bool {
        address == self.address
    }

    fn start(&mut self, _ctx: &mut dyn DeviceCtx, _address: u8, _read: bool) -> bool {
        // Reads return the status byte
        self.expect_control = true;
        true
    }

    fn write(&mut self, _ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        if self.expect_control {
            self.continuation = byte & 0x80 != 0;
            self.data_mode = byte & 0x40 != 0;
            self.expect_control = false;
            return true;
        }
        if self.data_mode {
            self.data(byte);
        } else {
            self.command_byte(byte);
        }
        // Co = 1: one byte, then another control byte; Co = 0: the rest is a stream
        if self.continuation {
            self.expect_control = true;
        }
        true
    }

    fn read(&mut self, _ctx: &mut dyn DeviceCtx, _ack: bool) -> u8 {
        // Status: D6 = display off
        if self.display_on { 0x00 } else { 0x40 }
    }

    fn clone_box(&self) -> Box<dyn I2cDevice> {
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

    fn write(d: &mut Ssd1306, bytes: &[u8]) {
        let mut ctx = TestCtx::default();
        assert!(d.start(&mut ctx, 0x3C, false));
        for &b in bytes {
            assert!(d.write(&mut ctx, b));
        }
        d.stop(&mut ctx);
    }

    /// The smart battery's 128x32 init sequence, one command per write.
    fn init_128x32(d: &mut Ssd1306) {
        for c in [
            0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x1F, 0xA4, 0xD3, 0x00, 0xD5,
            0xF0, 0xD9, 0x22, 0xDA, 0x02, 0xDB, 0x20, 0x8D, 0x14, 0xAF,
        ] {
            write(d, &[0x00, c]);
        }
    }

    #[test]
    fn init_and_horizontal_writes() {
        let mut d = Ssd1306::new();
        init_128x32(&mut d);
        assert!(d.display_on && d.charge_pump && d.segment_remap && d.com_remap);
        assert_eq!((d.addressing_mode, d.mux_ratio, d.contrast), (0, 32, 0xFF));
        assert_eq!(d.ignored_page_commands, 3, "B0h, 00h and 10h do nothing in horizontal mode");
        // A full-width line on RAM row 0, then pixel (5, 9)
        let mut data = vec![0x40];
        data.extend([0x01u8; 128]);
        data.extend([0u8; 5]);
        data.push(0x02);
        write(&mut d, &data);
        let f = d.frame();
        assert_eq!(f.len(), 32);
        assert!(f[0].iter().all(|&p| p));
        assert!(f[9][5] && !f[9][4] && !f[1][0]);
        assert!(d.render_text().starts_with("▀▀▀"));
        // A lenient controller lets B0h move the pointer in horizontal mode
        let mut d = Ssd1306 { page_commands_in_any_mode: true, ..Ssd1306::new() };
        init_128x32(&mut d);
        write(&mut d, &[0x00, 0xB2]);
        write(&mut d, &[0x40, 0xAA]);
        assert_eq!((d.ignored_page_commands, d.ram[2][0]), (0, 0xAA));
    }

    #[test]
    fn page_mode_and_control_bytes() {
        let mut d = Ssd1306::new();
        // Co = 1 pairs: command, command, then a data stream
        write(&mut d, &[0x80, 0xAF, 0x80, 0xA1, 0x80, 0xC8, 0x80, 0xB2, 0x80, 0x05, 0x80, 0x11, 0x40, 0xFF, 0xFF]);
        assert_eq!(d.ram[2][0x15], 0xFF);
        assert_eq!(d.ram[2][0x16], 0xFF);
        assert_eq!(d.ignored_page_commands, 0);
        // Window commands only apply in horizontal/vertical mode
        write(&mut d, &[0x00, 0x21, 0, 3]);
        assert_eq!(d.col_range, (0, 127));
        write(&mut d, &[0x00, 0x20, 0x01, 0x21, 10, 11, 0x22, 1, 2]);
        write(&mut d, &[0x40, 1, 2, 3]);
        assert_eq!((d.ram[1][10], d.ram[2][10], d.ram[1][11]), (1, 2, 3), "vertical mode");
    }

    #[test]
    fn png_and_display_off() {
        let mut d = Ssd1306::new();
        init_128x32(&mut d);
        let png = d.to_png(2);
        assert_eq!(u32::from_be_bytes(png[16..20].try_into().unwrap()), 256);
        assert_eq!(u32::from_be_bytes(png[20..24].try_into().unwrap()), 64);
        write(&mut d, &[0x00, 0xA5]);
        assert!(d.frame()[3][7], "entire display on");
        write(&mut d, &[0x00, 0xAE]);
        assert!(!d.frame()[3][7]);
    }
}
