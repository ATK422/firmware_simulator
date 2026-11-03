//! TI BQ40Z80 battery pack manager / gas gauge, SMBus target at 0x0B (technical reference manual SLUUBT5).
//!
//! Smart Battery Data commands: words (`Read Word` with PEC) and blocks (`Block Read` with a length byte and PEC),
//! the PEC covering the address bytes, command and data as SMBus specifies. `ManufacturerBlockAccess()` (0x44)
//! takes a block write of a 16-bit MAC command (and data, for data flash writes) and answers the following block
//! read with the command echoed then its data: DeviceType, FirmwareVersion, ManufacturingStatus, DAStatus1/2,
//! OperationStatus, SafetyStatus, PFStatus, data flash reads (0x4000 and up), and the FET control / emergency
//! shutdown commands. Block writes with a bad PEC are NAKed.
//!
//! The pack's state comes from [`Signal`]s: voltage, current (negative discharging), state of charge, temperature.

use sim_core::analog::Signal;
use sim_core::time::Time;

use crate::DeviceCtx;
use crate::i2c::{I2cDevice, smbus_pec};

pub const ADDRESS: u8 = 0x0B;
pub const DEVICE_TYPE: u16 = 0x4800;

#[derive(Clone, Debug)]
pub struct Bq40z80 {
    pub serial: u16,
    /// `ManufacturerDate()` raw: day | month << 5 | (year - 1980) << 9.
    pub date: u16,
    pub voltage_mv: Signal,
    pub current_ma: Signal,
    pub soc_percent: Signal,
    pub temperature_c: Signal,
    pub cells: usize,
    /// `GPIORead()` RH1: which side of the vehicle the pack is on (true = port).
    pub side_port: bool,
    pub design_capacity_mah: u16,
    pub manufacturing_status: u16,
    pub operation_status: u32,
    /// Data flash, from 0x4000.
    pub data_flash: Vec<u8>,
    pub emergency_shutdown: bool,
    mac: u16,
    /// Error code of the last command (BatteryStatus bits 3:0).
    error_code: u16,
    written: Vec<u8>,
    response: Vec<u8>,
    pos: usize,
    pub reads: u32,
    pub pec_errors: u32,
    pub clock_stretch: Time,
}

impl Bq40z80 {
    pub fn new(serial: u16) -> Self {
        let mut df = vec![0xFF; 0x2000];
        // Scale factor: 1
        df[0x4AE8 - 0x4000] = 1;
        df[0x4AE9 - 0x4000..0x4AEC - 0x4000].fill(0);
        Bq40z80 {
            serial,
            date: (43 << 9) | (7 << 5) | 26,
            voltage_mv: Signal::Const(16_400.0),
            current_ma: Signal::Const(-2_500.0),
            soc_percent: Signal::Const(80.0),
            temperature_c: Signal::Const(25.0),
            cells: 4,
            side_port: true,
            design_capacity_mah: 10_000,
            manufacturing_status: 0x0018,
            // PRES, DSG, CHG
            operation_status: 0b111,
            data_flash: df,
            emergency_shutdown: false,
            mac: 0,
            error_code: 0,
            written: Vec::new(),
            response: Vec::new(),
            pos: 0,
            reads: 0,
            pec_errors: 0,
            clock_stretch: Time::from_us(20),
        }
    }

    fn word(&self, cmd: u8, t: Time) -> Option<u16> {
        let v = self.voltage_mv.at(t);
        let i = self.current_ma.at(t);
        let soc = self.soc_percent.at(t).clamp(0.0, 100.0);
        let cap = self.design_capacity_mah as f64;
        let remaining = cap * soc / 100.0;
        let discharging = i < 0.0;
        Some(match cmd {
            0x08 => ((self.temperature_c.at(t) + 273.15) * 10.0).round() as u16,
            0x09 => v.round() as u16,
            0x0A | 0x0B => i.round() as i16 as u16,
            0x0C => 1,
            0x0D | 0x0E => soc.round() as u16,
            0x0F => remaining as u16,
            0x10 | 0x18 => self.design_capacity_mah,
            0x11 | 0x12 => {
                if discharging {
                    (remaining / -i * 60.0).min(65534.0) as u16
                } else {
                    65535
                }
            }
            0x13 => {
                if !discharging && i > 0.0 {
                    ((cap - remaining) / i * 60.0).min(65534.0) as u16
                } else {
                    65535
                }
            }
            0x14 => 3000,
            0x15 => 16_800,
            0x16 => {
                // FD/FC/DSG/INIT plus the last command's error code
                let mut s = self.error_code & 0xF;
                if discharging {
                    s |= 1 << 6;
                }
                if soc >= 100.0 {
                    s |= 1 << 5;
                }
                s
            }
            0x17 => 12,
            0x19 => 14_800,
            0x1B => self.date,
            0x1C => self.serial,
            0x3C..=0x3F => {
                // CellVoltage7..4: zero for cells the pack doesn't have
                let cell = 7 - (cmd - 0x3C) as usize;
                if cell <= self.cells { (v / self.cells as f64).round() as u16 } else { 0 }
            }
            0x48 => (self.side_port as u16) << 3,
            0x4F => 100,
            _ => return None,
        })
    }

    fn mac_data(&self, mac: u16, t: Time) -> Vec<u8> {
        let le32 = |v: u32| v.to_le_bytes().to_vec();
        match mac {
            0x0001 => DEVICE_TYPE.to_le_bytes().to_vec(),
            0x0002 => vec![0x48, 0x00, 0x01, 0x09, 0x00, 0x24, 0x00, 0x03, 0x85, 0x02, 0x00],
            0x0003 => vec![0x00, 0x00],
            0x0051 | 0x0053 => le32(0),
            0x0054 => le32(self.op_status()),
            0x0057 => self.manufacturing_status.to_le_bytes().to_vec(),
            0x0071 => self.da_status1(t),
            0x0072 => vec![0; 16],
            0x4000..=0x5FFF => {
                let o = (mac - 0x4000) as usize;
                let end = (o + 32).min(self.data_flash.len());
                self.data_flash[o..end].to_vec()
            }
            _ => Vec::new(),
        }
    }

    fn op_status(&self) -> u32 {
        let mut s = self.operation_status;
        if self.emergency_shutdown {
            s |= 1 << 29;
            s &= !0b110;
        }
        s
    }

    fn da_status1(&self, t: Time) -> Vec<u8> {
        let v = self.voltage_mv.at(t);
        let i = self.current_ma.at(t);
        let cell = (v / self.cells as f64).round() as u16;
        let mut words = [0u16; 16];
        for (n, w) in words.iter_mut().take(4).enumerate() {
            *w = if n < self.cells { cell } else { 0 };
        }
        words[4] = v.round() as u16;
        words[5] = v.round() as u16;
        for w in words.iter_mut().skip(6).take(4) {
            *w = i.round() as i16 as u16;
        }
        let power = (v * i / 1e5).round() as i16 as u16;
        words[14] = power;
        words[15] = power;
        words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    fn block(&self, cmd: u8, t: Time) -> Option<Vec<u8>> {
        let le32 = |v: u32| v.to_le_bytes().to_vec();
        Some(match cmd {
            0x20 => b"Texas Inst.".to_vec(),
            0x21 => b"bq40z80".to_vec(),
            0x22 => b"LION".to_vec(),
            0x44 => [self.mac.to_le_bytes().to_vec(), self.mac_data(self.mac, t)].concat(),
            0x50..=0x53 => le32(0),
            0x54 => le32(self.op_status()),
            0x55 | 0x56 => vec![0, 0, 0],
            0x57 => self.manufacturing_status.to_le_bytes().to_vec(),
            0x71 => self.da_status1(t),
            0x72 => vec![0; 16],
            _ => return None,
        })
    }

    /// Runs a completed write: a MAC command through 0x44, or a word write.
    fn execute_write(&mut self) {
        let w = std::mem::take(&mut self.written);
        if w.first() != Some(&0x44) || w.len() < 4 {
            return;
        }
        let len = w[1] as usize;
        if w.len() < 2 + len || len < 2 {
            return;
        }
        let mac = u16::from_le_bytes([w[2], w[3]]);
        let data = &w[4..2 + len];
        self.mac = mac;
        self.error_code = 0;
        match mac {
            0x270C | 0x043D => self.emergency_shutdown = true,
            0x23A7 => self.emergency_shutdown = false,
            0x4000..=0x5FFF if !data.is_empty() => {
                let o = (mac - 0x4000) as usize;
                let end = (o + data.len()).min(self.data_flash.len());
                self.data_flash[o..end].copy_from_slice(&data[..end - o]);
            }
            _ => {}
        }
    }
}

impl I2cDevice for Bq40z80 {
    fn name(&self) -> String {
        "BQ40Z80".into()
    }

    fn responds_to(&self, address: u8) -> bool {
        address == ADDRESS
    }

    fn start(&mut self, ctx: &mut dyn DeviceCtx, address: u8, read: bool) -> bool {
        if !read {
            if !self.written.is_empty() {
                self.execute_write();
            }
            self.written.clear();
            return true;
        }
        // A read after writing the command byte: build the response with its PEC
        let Some(&cmd) = self.written.first() else { return true };
        let t = ctx.now();
        let data = match self.word(cmd, t) {
            Some(w) => w.to_le_bytes().to_vec(),
            None => match self.block(cmd, t) {
                Some(b) => [vec![b.len() as u8], b].concat(),
                None => {
                    self.error_code = 0x3; // UnsupportedCommand
                    vec![0xFF, 0xFF]
                }
            },
        };
        let mut pec_in = vec![address << 1, cmd, address << 1 | 1];
        pec_in.extend_from_slice(&data);
        self.response = data;
        self.response.push(smbus_pec(&pec_in));
        self.pos = 0;
        self.reads += 1;
        self.written.clear();
        true
    }

    fn write(&mut self, _ctx: &mut dyn DeviceCtx, byte: u8) -> bool {
        self.written.push(byte);
        // A block write's last byte is the PEC: NAK it if wrong
        if self.written.len() >= 3 && self.written[0] == 0x44 {
            let len = self.written[1] as usize;
            if self.written.len() == 3 + len {
                let mut pec_in = vec![ADDRESS << 1];
                pec_in.extend_from_slice(&self.written[..2 + len]);
                if smbus_pec(&pec_in) != byte {
                    self.pec_errors += 1;
                    self.written.clear();
                    return false;
                }
            }
        }
        true
    }

    fn read(&mut self, _ctx: &mut dyn DeviceCtx, _ack: bool) -> u8 {
        let b = self.response.get(self.pos).copied().unwrap_or(0xFF);
        self.pos += 1;
        b
    }

    fn stop(&mut self, _ctx: &mut dyn DeviceCtx) {
        if !self.written.is_empty() {
            self.execute_write();
        }
    }

    fn stretch(&mut self, _ctx: &mut dyn DeviceCtx) -> Time {
        self.clock_stretch
    }

    fn clone_box(&self) -> Box<dyn I2cDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn relink_inputs(&mut self, map: &sim_core::analog::Relink) {
        self.voltage_mv.relink(map);
        self.current_ma.relink(map);
        self.soc_percent.relink(map);
        self.temperature_c.relink(map);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    fn read(g: &mut Bq40z80, ctx: &mut TestCtx, cmd: u8, n: usize) -> Vec<u8> {
        g.start(ctx, ADDRESS, false);
        g.write(ctx, cmd);
        g.start(ctx, ADDRESS, true);
        (0..n).map(|_| g.read(ctx, true)).collect()
    }

    #[test]
    fn word_and_block_reads_with_pec() {
        let mut ctx = TestCtx::default();
        let mut g = Bq40z80::new(21);
        let r = read(&mut g, &mut ctx, 0x1C, 3);
        assert_eq!(u16::from_le_bytes([r[0], r[1]]), 21);
        assert_eq!(r[2], smbus_pec(&[0x16, 0x1C, 0x17, r[0], r[1]]));
        let r = read(&mut g, &mut ctx, 0x0A, 2);
        assert_eq!(i16::from_le_bytes([r[0], r[1]]), -2500);
        let r = read(&mut g, &mut ctx, 0x54, 6);
        assert_eq!(r[0], 4, "OperationStatus is a 4-byte block");
        assert_eq!(r[5], smbus_pec(&[0x16, 0x54, 0x17, 4, r[1], r[2], r[3], r[4]]));
        let r = read(&mut g, &mut ctx, 0x16, 2);
        assert_eq!(r[0] & (1 << 6), 1 << 6, "DSG while discharging");
    }

    fn mac_write(g: &mut Bq40z80, ctx: &mut TestCtx, mac: u16, good_pec: bool) -> bool {
        let body = [0x44, 2, mac as u8, (mac >> 8) as u8];
        let mut pec = smbus_pec(&[[0x16].as_slice(), &body].concat());
        if !good_pec {
            pec ^= 1;
        }
        g.start(ctx, ADDRESS, false);
        let ok = body.iter().chain(&[pec]).all(|&b| g.write(ctx, b));
        g.stop(ctx);
        ok
    }

    #[test]
    fn manufacturer_block_access() {
        let mut ctx = TestCtx::default();
        let mut g = Bq40z80::new(21);
        assert!(mac_write(&mut g, &mut ctx, 0x0001, true));
        let r = read(&mut g, &mut ctx, 0x44, 5);
        assert_eq!(r[..5], [4, 0x01, 0x00, 0x00, 0x48], "length, echoed command, 0x4800");
        // Data flash: the scale factor
        assert!(mac_write(&mut g, &mut ctx, 0x4AE8, true));
        let r = read(&mut g, &mut ctx, 0x44, 7);
        assert_eq!((r[0], r[3]), (34, 1));
        // DAStatus1: battery voltage in word 4
        assert!(mac_write(&mut g, &mut ctx, 0x0071, true));
        let r = read(&mut g, &mut ctx, 0x44, 35);
        assert_eq!(u16::from_le_bytes([r[3 + 8], r[3 + 9]]), 16_400);
        // A bad PEC is NAKed and the command not executed
        assert!(!mac_write(&mut g, &mut ctx, 0x270C, false));
        assert!(!g.emergency_shutdown);
        assert!(mac_write(&mut g, &mut ctx, 0x270C, true));
        assert!(g.emergency_shutdown);
        assert_eq!(g.pec_errors, 1);
    }
}
