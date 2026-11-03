//! The QSPI flash chip on the board: a Winbond W25Q16JV (2 MiB; the W25Q080 class of every UWRT board and the
//! Pico), datasheet rev F. The chip operates on the system's flash array.
//!
//! Standard SPI transactions (one command per chip select) cover the status registers (with write enable, the
//! volatile write enable and the Quad Enable bit), reads (03h, 0Bh), page program (02h, 32h, wrapping within the
//! page, only clearing bits), sector/block/chip erase (20h, 52h, D8h, C7h/60h), JEDEC, manufacturer/device and
//! unique IDs, SFDP, power-down and reset. Program and erase set BUSY for the datasheet's typical times; during them
//! only status reads are accepted. Quad I/O fast read (EBh) with mode bits Ax enters continuous read mode, where
//! the chip expects an address without a command; a standard transaction (the bootrom's `flash_exit_xip` sends FFh)
//! leaves it.

use sim_core::time::Time;

pub const JEDEC_ID: [u8; 3] = [0xEF, 0x40, 0x15];
pub const UNIQUE_ID: [u8; 8] = [0xE6, 0x60, 0x58, 0x38, 0x83, 0x2B, 0x51, 0x29];

const SR1_BUSY: u8 = 1 << 0;
const SR1_WEL: u8 = 1 << 1;
const SR2_QE: u8 = 1 << 1;

#[derive(Clone, Debug)]
pub struct QspiFlash {
    pub sr1: u8,
    pub sr2: u8,
    pub sr3: u8,
    busy_until: Time,
    /// Volatile status register write enable (50h) for the next status write.
    volatile_wel: bool,
    pub continuous: bool,
    pub power_down: bool,
    /// Bytes of the current transaction (command first).
    tx: Vec<u8>,
    selected: bool,
    /// Operations performed (for tests and statistics).
    pub erases: u32,
    pub programs: u32,
    pub reads: u32,
}

impl Default for QspiFlash {
    fn default() -> Self {
        QspiFlash {
            sr1: 0,
            // The JV's factory default has QE set (the "IQ" parts); boot2 checks and sets it anyway
            sr2: 0,
            sr3: 0,
            busy_until: Time::ZERO,
            volatile_wel: false,
            continuous: false,
            power_down: false,
            tx: Vec::new(),
            selected: false,
            erases: 0,
            programs: 0,
            reads: 0,
        }
    }
}

impl QspiFlash {
    /// When the program or erase in progress ends.
    pub fn busy_until(&self) -> Time {
        self.busy_until
    }

    pub fn busy(&self, now: Time) -> bool {
        now < self.busy_until
    }

    pub fn quad_enabled(&self) -> bool {
        self.sr2 & SR2_QE != 0
    }

    /// Updates BUSY/WEL once an operation is over.
    fn settle(&mut self, now: Time) {
        if self.sr1 & SR1_BUSY != 0 && !self.busy(now) {
            self.sr1 &= !(SR1_BUSY | SR1_WEL);
        }
    }

    /// Power applied again: volatile state (write enable, continuous read, power-down, any operation) is lost; the
    /// non-volatile status bits stay.
    pub fn power_up(&mut self) {
        self.sr1 &= !(SR1_BUSY | SR1_WEL);
        self.busy_until = Time::ZERO;
        self.volatile_wel = false;
        self.continuous = false;
        self.power_down = false;
        self.selected = false;
        self.tx.clear();
    }

    /// The state boot2 leaves the chip in (QE set, continuous quad read), for direct boot.
    pub fn preset_quad_xip(&mut self) {
        self.sr2 |= SR2_QE;
        self.continuous = true;
    }

    /// Chip select changed. A deselect ends the transaction and carries out writes, programs and erases.
    pub fn select(&mut self, selected: bool, now: Time, data: &mut [u8]) {
        if selected == self.selected {
            return;
        }
        self.selected = selected;
        if selected {
            self.tx.clear();
            return;
        }
        self.settle(now);
        let tx = std::mem::take(&mut self.tx);
        if tx.is_empty() {
            return;
        }
        if self.continuous {
            // Address and mode bits without a command; a standard SPI transaction here is the reset sequence
            self.continuous = false;
            return;
        }
        self.execute(&tx, now, data);
    }

    fn start_op(&mut self, now: Time, duration: Time) {
        self.sr1 |= SR1_BUSY;
        self.busy_until = now + duration;
    }

    fn addr(tx: &[u8]) -> Option<usize> {
        (tx.len() >= 4).then(|| (tx[1] as usize) << 16 | (tx[2] as usize) << 8 | tx[3] as usize)
    }

    /// Commands that take effect at the end of the transaction.
    fn execute(&mut self, tx: &[u8], now: Time, data: &mut [u8]) {
        let cmd = tx[0];
        if self.power_down && cmd != 0xAB {
            return;
        }
        if self.busy(now) && !matches!(cmd, 0x05 | 0x35 | 0x15) {
            log::debug!("QSPI flash: command {cmd:#04x} while busy ignored");
            return;
        }
        let wel = self.sr1 & SR1_WEL != 0;
        let len = data.len();
        match cmd {
            0x06 => self.sr1 |= SR1_WEL,
            0x04 => self.sr1 &= !SR1_WEL,
            0x50 => self.volatile_wel = true,
            0x01 | 0x31 | 0x11 if wel || self.volatile_wel => {
                match (cmd, tx.len()) {
                    (0x01, 2) => self.sr1 = (self.sr1 & 0x03) | (tx[1] & 0xFC),
                    (0x01, n) if n >= 3 => {
                        self.sr1 = (self.sr1 & 0x03) | (tx[1] & 0xFC);
                        self.sr2 = tx[2];
                    }
                    (0x31, n) if n >= 2 => self.sr2 = tx[1],
                    (0x11, n) if n >= 2 => self.sr3 = tx[1],
                    _ => {}
                }
                if self.volatile_wel {
                    self.volatile_wel = false;
                } else {
                    // Non-volatile write: 10 ms typical
                    self.start_op(now, Time::from_ms(10));
                }
            }
            0x02 | 0x32 if wel => {
                let Some(a) = Self::addr(tx) else { return };
                let page = a & !0xFF;
                for (i, &b) in tx[4..].iter().enumerate() {
                    let o = (page | ((a + i) & 0xFF)) % len;
                    data[o] &= b;
                }
                self.programs += 1;
                self.start_op(now, Time::from_us(400));
            }
            0x20 | 0x52 | 0xD8 if wel => {
                let Some(a) = Self::addr(tx) else { return };
                let (size, t) = match cmd {
                    0x20 => (4096, Time::from_ms(45)),
                    0x52 => (32768, Time::from_ms(120)),
                    _ => (65536, Time::from_ms(150)),
                };
                let start = (a & !(size - 1)) % len;
                let end = (start + size).min(len);
                data[start..end].fill(0xFF);
                self.erases += 1;
                self.start_op(now, t);
            }
            0xC7 | 0x60 if wel => {
                data.fill(0xFF);
                self.erases += 1;
                self.start_op(now, Time::from_secs(5));
            }
            0xB9 => self.power_down = true,
            0xAB => self.power_down = false,
            0x66 | 0x99 => {
                self.volatile_wel = false;
                self.continuous = false;
            }
            0x02 | 0x32 | 0x20 | 0x52 | 0xD8 | 0xC7 | 0x60 | 0x01 | 0x31 | 0x11 => {
                log::debug!("QSPI flash: command {cmd:#04x} without write enable ignored");
            }
            _ => {}
        }
    }

    /// One byte each way in standard SPI mode while selected.
    pub fn exchange(&mut self, mosi: u8, now: Time, data: &[u8]) -> u8 {
        if !self.selected {
            return 0xFF;
        }
        self.settle(now);
        let i = self.tx.len();
        self.tx.push(mosi);
        if self.continuous {
            return 0xFF;
        }
        let cmd = self.tx[0];
        if self.power_down && cmd != 0xAB {
            return 0xFF;
        }
        let at = |o: usize| data[o % data.len()];
        match cmd {
            0x05 if i >= 1 => self.sr1,
            0x35 if i >= 1 => self.sr2,
            0x15 if i >= 1 => self.sr3,
            0x9F if (1..=3).contains(&i) => JEDEC_ID[i - 1],
            // Manufacturer/device ID after 3 address bytes
            0x90 if i >= 4 => [0xEF, 0x14][(i - 4) % 2],
            0xAB if i >= 4 => 0x14,
            0x4B if (5..13).contains(&i) => UNIQUE_ID[i - 5],
            0x5A if i >= 5 => {
                sfdp(((self.tx[1] as usize) << 16 | (self.tx[2] as usize) << 8 | self.tx[3] as usize) + i - 5)
            }
            0x03 if i >= 4 && !self.busy(now) => {
                if i == 4 {
                    self.reads += 1;
                }
                at(Self::addr(&self.tx).unwrap() + i - 4)
            }
            0x0B if i >= 5 && !self.busy(now) => {
                if i == 5 {
                    self.reads += 1;
                }
                at(Self::addr(&self.tx).unwrap() + i - 5)
            }
            _ => 0xFF,
        }
    }

    /// An enhanced-mode (dual/quad) read the SSI frames itself: `cmd` (absent in continuous mode), a 24-bit address
    /// and the mode byte. Returns the bytes from `addr` on, or `None` if the chip would not answer.
    pub fn enhanced_read(&mut self, cmd: Option<u8>, addr: u32, mode: u8, now: Time) -> Option<usize> {
        self.settle(now);
        if self.busy(now) || self.power_down {
            return None;
        }
        match cmd {
            None if !self.continuous => return None,
            None => {}
            Some(0xEB | 0x6B | 0xE7) if !self.quad_enabled() => return None,
            Some(0xEB | 0x6B | 0xBB | 0x3B | 0xE7 | 0x0B | 0x03) => {}
            Some(_) => return None,
        }
        if matches!(cmd, None | Some(0xEB) | Some(0xBB)) {
            // Continuous read mode: mode bits Ax (M5-4 = 10)
            self.continuous = mode & 0x30 == 0x20;
        }
        self.reads += 1;
        Some(addr as usize & 0xFF_FFFF)
    }
}

/// A minimal SFDP table: signature, revision, one parameter header (JEDEC basic, 9 DWORDs at 0x80) and the 2 MiB
/// density.
fn sfdp(a: usize) -> u8 {
    const HDR: [u8; 16] =
        [0x53, 0x46, 0x44, 0x50, 0x05, 0x01, 0x00, 0xFF, 0x00, 0x05, 0x01, 0x09, 0x80, 0x00, 0x00, 0xFF];
    match a {
        0..=15 => HDR[a],
        0x84..=0x87 => (0x00FF_FFFFu32).to_le_bytes()[a - 0x84],
        0x80 => 0xE5,
        0x81 => 0x20,
        0x82 => 0xF1,
        _ => 0xFF,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn txn(f: &mut QspiFlash, bytes: &[u8], now: Time, data: &mut [u8]) -> Vec<u8> {
        f.select(true, now, data);
        let out = bytes.iter().map(|&b| f.exchange(b, now, data)).collect();
        f.select(false, now, data);
        out
    }

    #[test]
    fn ids_status_and_write_enable() {
        let mut data = vec![0xFFu8; 2 << 20];
        let mut f = QspiFlash::default();
        let t = Time::ZERO;
        assert_eq!(txn(&mut f, &[0x9F, 0, 0, 0], t, &mut data)[1..], JEDEC_ID);
        assert_eq!(txn(&mut f, &[0x4B, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], t, &mut data)[5..], UNIQUE_ID);
        assert_eq!(txn(&mut f, &[0x5A, 0, 0, 0, 0, 0, 0, 0, 0], t, &mut data)[5..], *b"SFDP");
        // Program without WREN is ignored
        txn(&mut f, &[0x02, 0, 0, 0x10, 0x12], t, &mut data);
        assert_eq!(data[0x10], 0xFF);
        txn(&mut f, &[0x06], t, &mut data);
        assert_eq!(txn(&mut f, &[0x05, 0], t, &mut data)[1], SR1_WEL);
        // QE through the 2-byte WRSR
        txn(&mut f, &[0x01, 0x00, 0x02], t, &mut data);
        assert_eq!(txn(&mut f, &[0x05, 0], t, &mut data)[1] & SR1_BUSY, SR1_BUSY, "10 ms non-volatile write");
        let t = Time::from_ms(11);
        assert_eq!(txn(&mut f, &[0x05, 0], t, &mut data)[1], 0, "WEL cleared after the write");
        assert_eq!(txn(&mut f, &[0x35, 0], t, &mut data)[1], SR2_QE);
    }

    #[test]
    fn program_erase_busy_and_read() {
        let mut data = vec![0xFFu8; 2 << 20];
        let mut f = QspiFlash::default();
        let t = Time::from_ms(1);
        txn(&mut f, &[0x06], t, &mut data);
        // Page program wraps within the page and only clears bits
        txn(&mut f, &[0x02, 0x00, 0x01, 0xFE, 0x0F, 0xF0, 0x3C], t, &mut data);
        assert_eq!((data[0x1FE], data[0x1FF], data[0x100]), (0x0F, 0xF0, 0x3C));
        // Busy for 0.4 ms: reads and other commands are refused
        assert_eq!(txn(&mut f, &[0x03, 0, 1, 0xFE, 0], t, &mut data)[4], 0xFF);
        txn(&mut f, &[0x06], t, &mut data);
        assert_eq!(f.sr1 & SR1_WEL, SR1_WEL, "WEL still set from the program in progress");
        let t = t + Time::from_ms(1);
        assert_eq!(txn(&mut f, &[0x03, 0, 1, 0xFE, 0, 0], t, &mut data)[4..], [0x0F, 0xF0]);
        assert_eq!(txn(&mut f, &[0x0B, 0, 1, 0xFE, 0, 0], t, &mut data)[5], 0x0F);
        // Sector erase: 45 ms
        txn(&mut f, &[0x06], t, &mut data);
        txn(&mut f, &[0x20, 0, 0x01, 0x23], t, &mut data);
        assert!(f.busy(t + Time::from_ms(44)) && !f.busy(t + Time::from_ms(46)));
        assert!(data[0..4096].iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn continuous_quad_read_and_exit() {
        let mut data = vec![0u8; 2 << 20];
        let mut f = QspiFlash::default();
        let t = Time::ZERO;
        assert_eq!(f.enhanced_read(Some(0xEB), 0, 0xA0, t), None, "quad needs QE");
        f.sr2 |= SR2_QE;
        assert_eq!(f.enhanced_read(Some(0xEB), 0x100, 0xA0, t), Some(0x100));
        assert!(f.continuous);
        assert_eq!(f.enhanced_read(None, 0x200, 0xA0, t), Some(0x200));
        // A standard transaction (the FFh reset sequence) leaves continuous mode without executing anything
        txn(&mut f, &[0xFF, 0xFF], t, &mut data);
        assert!(!f.continuous);
        assert_eq!(f.enhanced_read(None, 0, 0xA0, t), None);
        // Power down
        txn(&mut f, &[0xB9], t, &mut data);
        assert_eq!(txn(&mut f, &[0x9F, 0], t, &mut data)[1], 0xFF);
        txn(&mut f, &[0xAB, 0, 0, 0, 0], t, &mut data);
        assert!(!f.power_down);
    }
}
