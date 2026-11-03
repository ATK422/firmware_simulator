//! Host-side USB tools on the emulator's host controller model ([`sim_devices::usb::host::HostController`]):
//! what `picotool` does over PICOBOOT ([`Picoboot`]), and copying a UF2 file to the BOOTSEL drive ([`MscDrive`]).
//!
//! Each call queues transfers on the host and runs the machine until they finish, so the device side is whatever
//! the firmware does with them: in BOOTSEL mode that is the real bootrom's USB stack, its PICOBOOT command handler
//! and its virtual FAT disk, programming the QSPI flash model through its own flash routines.

use rp2040::Machine;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::usb::host::HostController;
use sim_devices::usb::{DeviceInfo, EndpointType, SetupPacket, UsbError, req_type};

use crate::Session;

/// Something that owns a machine with a USB host attached and can run it.
pub trait UsbRunner {
    fn machine(&mut self) -> &mut Machine;
    /// Runs for `d` of virtual time.
    fn advance(&mut self, d: Time);
}

impl UsbRunner for Machine {
    fn machine(&mut self) -> &mut Machine {
        self
    }
    fn advance(&mut self, d: Time) {
        self.run_for(d);
    }
}

impl UsbRunner for Session {
    fn machine(&mut self) -> &mut Machine {
        &mut self.machine
    }
    fn advance(&mut self, d: Time) {
        self.run_for(d);
    }
}

/// Errors from the USB tools.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// No USB host is attached to the machine.
    NoHost,
    /// The device did not show up (enumerated with the expected interface) in time.
    NoDevice(String),
    Usb(UsbError),
    /// The device rejected a PICOBOOT command; the status it reported.
    Picoboot {
        cmd: u8,
        status: u32,
    },
    /// Data read back differs from what was written.
    Verify {
        addr: u32,
    },
    Scsi(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::NoHost => write!(f, "no USB host attached"),
            Error::NoDevice(m) => write!(f, "{m}"),
            Error::Usb(e) => write!(f, "USB: {e}"),
            Error::Picoboot { cmd, status } => {
                write!(f, "PICOBOOT command {cmd:#04x} failed: {} ({status})", picoboot_status(*status))
            }
            Error::Verify { addr } => write!(f, "verify failed at {addr:#010x}"),
            Error::Scsi(m) => write!(f, "SCSI: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<UsbError> for Error {
    fn from(e: UsbError) -> Error {
        Error::Usb(e)
    }
}

/// How often the tools look at the host while waiting.
const POLL: Time = Time::from_us(200);

fn host(r: &mut impl UsbRunner) -> Result<&mut HostController, Error> {
    r.machine().sys.usb_host_mut().ok_or(Error::NoHost)
}

/// Queues a transfer with `f` and runs until it finishes (or `timeout` of virtual time passes).
pub fn transfer(
    r: &mut impl UsbRunner,
    timeout: Time,
    f: impl FnOnce(&mut HostController) -> u32,
) -> Result<Vec<u8>, Error> {
    let now = r.machine().now;
    let id = r.machine().sys.with_usb_host(now, f).ok_or(Error::NoHost)?;
    let deadline = now + timeout;
    host(r)?.set_deadline(id, deadline);
    loop {
        if let Some(res) = host(r)?.take_result(id) {
            return res.map_err(Error::Usb);
        }
        if r.machine().now > deadline + Time::from_ms(10) {
            // The host only expires transfers while its port is enabled; a detached device ends here
            host(r)?.cancel(id);
            host(r)?.take_result(id);
            return Err(Error::Usb(UsbError::TimedOut));
        }
        r.advance(POLL);
    }
}

/// Runs until the host has configured a device that `pred` accepts.
pub fn wait_for_device(
    r: &mut impl UsbRunner,
    timeout: Time,
    pred: impl Fn(&DeviceInfo) -> bool,
) -> Result<DeviceInfo, Error> {
    let deadline = r.machine().now + timeout;
    loop {
        if let Some(d) = host(r)?.configured()
            && pred(d)
        {
            return Ok(d.clone());
        }
        if r.machine().now >= deadline {
            let state = host(r).map(|h| format!("{:?}, {:?}", h.state, h.enumeration)).unwrap_or_default();
            return Err(Error::NoDevice(format!("no matching USB device within {timeout} (host: {state})")));
        }
        r.advance(Time::from_ms(1));
    }
}

// ---------------------------------------------------------------------------------------------- PICOBOOT

/// RP2040 BOOTSEL mode: Raspberry Pi's vendor ID and the RP2 Boot product.
pub const RP2_BOOT_VID: u16 = 0x2E8A;
pub const RP2_BOOT_PID: u16 = 0x0003;

/// PICOBOOT protocol constants (pico-sdk `boot/picoboot.h`).
pub mod pc {
    pub const MAGIC: u32 = 0x431F_D10B;
    pub const IF_RESET: u8 = 0x41;
    pub const IF_CMD_STATUS: u8 = 0x42;
    pub const EXCLUSIVE_ACCESS: u8 = 0x01;
    pub const REBOOT: u8 = 0x02;
    pub const FLASH_ERASE: u8 = 0x03;
    pub const READ: u8 = 0x84;
    pub const WRITE: u8 = 0x05;
    pub const EXIT_XIP: u8 = 0x06;
    pub const ENTER_CMD_XIP: u8 = 0x07;
    pub const EXEC: u8 = 0x08;
}

/// `enum picoboot_status` names.
pub fn picoboot_status(s: u32) -> &'static str {
    match s {
        0 => "OK",
        1 => "UNKNOWN_CMD",
        2 => "INVALID_CMD_LENGTH",
        3 => "INVALID_TRANSFER_LENGTH",
        4 => "INVALID_ADDRESS",
        5 => "BAD_ALIGNMENT",
        6 => "INTERLEAVED_WRITE",
        7 => "REBOOTING",
        8 => "UNKNOWN_ERROR",
        _ => "?",
    }
}

const FLASH_BASE: u32 = 0x1000_0000;
const SECTOR: u32 = 4096;
const PAGE: u32 = 256;
const SRAM_END: u32 = 0x2004_2000;

/// The reply to PICOBOOT_IF_CMD_STATUS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CmdStatus {
    pub token: u32,
    pub status: u32,
    pub cmd: u8,
    pub in_progress: bool,
}

/// What [`Picoboot::load`] did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LoadReport {
    /// Flash ranges written, `(address, length)`.
    pub ranges: Vec<(u32, u32)>,
    pub bytes: u32,
    /// Virtual time the whole load took.
    pub took: Time,
}

/// A PICOBOOT session with an RP2040 in BOOTSEL mode, as `picotool` opens one.
#[derive(Clone, Debug)]
pub struct Picoboot {
    pub interface: u8,
    pub ep_out: u8,
    pub ep_in: u8,
    token: u32,
    /// Virtual time a command may take before it is given up (flash erases are slow).
    pub timeout: Time,
}

impl Picoboot {
    /// Waits for the device to enumerate with a PICOBOOT interface (vendor class, two bulk endpoints) and resets
    /// the interface (PICOBOOT_IF_RESET), as `picotool` does when it connects.
    pub fn open(r: &mut impl UsbRunner, timeout: Time) -> Result<Picoboot, Error> {
        let has_picoboot = |d: &DeviceInfo| {
            d.vendor_id() == RP2_BOOT_VID
                && d.interface(0xFF).is_some_and(|i| {
                    i.endpoint(EndpointType::Bulk, true).is_some() && i.endpoint(EndpointType::Bulk, false).is_some()
                })
        };
        let dev = wait_for_device(r, timeout, has_picoboot)?;
        let i = dev.interface(0xFF).unwrap();
        let p = Picoboot {
            interface: i.number,
            ep_out: i.endpoint(EndpointType::Bulk, false).unwrap().address,
            ep_in: i.endpoint(EndpointType::Bulk, true).unwrap().address,
            token: 1,
            timeout: Time::from_secs(3),
        };
        p.reset(r)?;
        Ok(p)
    }

    /// PICOBOOT_IF_RESET: unstalls the endpoints, aborts a command in progress, clears exclusive access.
    pub fn reset(&self, r: &mut impl UsbRunner) -> Result<(), Error> {
        let ty = req_type::VENDOR | req_type::INTERFACE;
        let s = SetupPacket::new(ty, pc::IF_RESET, 0, self.interface as u16, 0);
        transfer(r, Time::from_secs(1), |h| h.submit_control(s, Vec::new()))?;
        Ok(())
    }

    /// PICOBOOT_IF_CMD_STATUS: the status of the last command.
    pub fn status(&self, r: &mut impl UsbRunner) -> Result<CmdStatus, Error> {
        let ty = req_type::DIR_IN | req_type::VENDOR | req_type::INTERFACE;
        let s = SetupPacket::new(ty, pc::IF_CMD_STATUS, 0, self.interface as u16, 16);
        let b = transfer(r, Time::from_secs(1), |h| h.submit_control(s, Vec::new()))?;
        if b.len() < 10 {
            return Err(Error::Usb(UsbError::NoResponse));
        }
        let w = |i: usize| u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        Ok(CmdStatus { token: w(0), status: w(4), cmd: b[8], in_progress: b[9] != 0 })
    }

    /// Sends one command: the 32 byte command block, the data phase (`out` for OUT commands, `in_len` bytes back
    /// for IN commands), and the zero length acknowledgement in the other direction.
    pub fn command(
        &mut self,
        r: &mut impl UsbRunner,
        cmd: u8,
        args: &[u8],
        out: &[u8],
        in_len: u32,
    ) -> Result<Vec<u8>, Error> {
        let token = self.token;
        self.token = self.token.wrapping_add(1);
        let transfer_len = if cmd & 0x80 != 0 { in_len } else { out.len() as u32 };
        let mut block = Vec::with_capacity(32);
        block.extend(pc::MAGIC.to_le_bytes());
        block.extend(token.to_le_bytes());
        block.extend([cmd, args.len() as u8, 0, 0]);
        block.extend(transfer_len.to_le_bytes());
        block.extend(args);
        block.resize(32, 0);
        let (ep_out, ep_in, timeout) = (self.ep_out, self.ep_in, self.timeout);
        let result = (|| {
            transfer(r, timeout, |h| h.submit_bulk_out(ep_out, block))?;
            let mut data = Vec::new();
            if cmd & 0x80 != 0 {
                if in_len > 0 {
                    data = transfer(r, timeout, |h| h.submit_bulk_in(ep_in, in_len as usize))?;
                }
                transfer(r, timeout, |h| h.submit_bulk_out(ep_out, Vec::new()))?;
            } else {
                if !out.is_empty() {
                    let o = out.to_vec();
                    transfer(r, timeout, |h| h.submit_bulk_out(ep_out, o))?;
                }
                transfer(r, timeout, |h| h.submit_bulk_in(ep_in, 0))?;
            }
            Ok(data)
        })();
        match result {
            Ok(d) => Ok(d),
            Err(Error::Usb(UsbError::Stall)) => {
                // The device halts the endpoints on a bad command: ask why, then reset the interface (which
                // unstalls them) and start the toggles afresh as after CLEAR_FEATURE(HALT)
                let st = self.status(r)?;
                self.reset(r)?;
                let (o, i) = (self.ep_out, self.ep_in);
                if let Some(h) = r.machine().sys.usb_host_mut() {
                    h.reset_toggle(o);
                    h.reset_toggle(i);
                }
                Err(Error::Picoboot { cmd, status: st.status })
            }
            Err(e) => Err(e),
        }
    }

    /// PC_EXCLUSIVE_ACCESS: 0 = shared with the mass storage drive, 1 = exclusive, 2 = exclusive and eject.
    pub fn exclusive_access(&mut self, r: &mut impl UsbRunner, mode: u8) -> Result<(), Error> {
        self.command(r, pc::EXCLUSIVE_ACCESS, &[mode], &[], 0).map(drop)
    }

    /// PC_EXIT_XIP: puts the flash in serial command mode.
    pub fn exit_xip(&mut self, r: &mut impl UsbRunner) -> Result<(), Error> {
        self.command(r, pc::EXIT_XIP, &[], &[], 0).map(drop)
    }

    /// PC_ENTER_CMD_XIP: back to (slow, serial) XIP.
    pub fn enter_cmd_xip(&mut self, r: &mut impl UsbRunner) -> Result<(), Error> {
        self.command(r, pc::ENTER_CMD_XIP, &[], &[], 0).map(drop)
    }

    fn range(addr: u32, size: u32) -> Vec<u8> {
        let mut a = addr.to_le_bytes().to_vec();
        a.extend(size.to_le_bytes());
        a
    }

    /// PC_FLASH_ERASE of whole 4 KiB sectors.
    pub fn flash_erase(&mut self, r: &mut impl UsbRunner, addr: u32, size: u32) -> Result<(), Error> {
        self.command(r, pc::FLASH_ERASE, &Self::range(addr, size), &[], 0).map(drop)
    }

    /// PC_WRITE: to flash (256 byte aligned, already erased) or RAM.
    pub fn write(&mut self, r: &mut impl UsbRunner, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.command(r, pc::WRITE, &Self::range(addr, data.len() as u32), data, 0).map(drop)
    }

    /// PC_READ from flash, RAM or ROM.
    pub fn read(&mut self, r: &mut impl UsbRunner, addr: u32, len: u32) -> Result<Vec<u8>, Error> {
        self.command(r, pc::READ, &Self::range(addr, len), &[], len)
    }

    /// PC_REBOOT: `pc` 0 reboots normally (from flash), otherwise into code at `pc` with stack `sp`, after
    /// `delay_ms`.
    pub fn reboot(&mut self, r: &mut impl UsbRunner, pc: u32, sp: u32, delay_ms: u32) -> Result<(), Error> {
        let mut a = pc.to_le_bytes().to_vec();
        a.extend(sp.to_le_bytes());
        a.extend(delay_ms.to_le_bytes());
        self.command(r, pc::REBOOT, &a, &[], 0).map(drop)
    }

    /// `picotool load -v -x`: erases the sectors the image's flash segments touch, writes them in 256 byte pages
    /// (padding with 0xFF), reads them back to verify, and reboots into the new image after `reboot_ms` (none if
    /// `None`). Segments outside the flash are ignored.
    pub fn load(
        &mut self,
        r: &mut impl UsbRunner,
        image: &Image,
        flash_size: u32,
        reboot_ms: Option<u32>,
    ) -> Result<LoadReport, Error> {
        let t0 = r.machine().now;
        let ranges = flash_pages(image, flash_size);
        self.exclusive_access(r, 1)?;
        self.exit_xip(r)?;
        let mut report = LoadReport::default();
        for (addr, data) in &ranges {
            let first = addr & !(SECTOR - 1);
            let end = (addr + data.len() as u32).next_multiple_of(SECTOR);
            self.flash_erase(r, first, end - first)?;
            for (i, chunk) in data.chunks(SECTOR as usize).enumerate() {
                self.write(r, addr + (i as u32) * SECTOR, chunk)?;
            }
            let back = self.read(r, *addr, data.len() as u32)?;
            if let Some(i) = back.iter().zip(data).position(|(a, b)| a != b) {
                return Err(Error::Verify { addr: addr + i as u32 });
            }
            report.ranges.push((*addr, data.len() as u32));
            report.bytes += data.len() as u32;
        }
        if let Some(ms) = reboot_ms {
            self.reboot(r, 0, SRAM_END, ms)?;
        }
        report.took = r.machine().now - t0;
        Ok(report)
    }
}

/// The image's flash contents as page-aligned runs (gaps of a page or more split runs; partial pages are padded
/// with 0xFF, which programming leaves erased).
pub fn flash_pages(image: &Image, flash_size: u32) -> Vec<(u32, Vec<u8>)> {
    let mut pages: std::collections::BTreeMap<u32, Vec<u8>> = std::collections::BTreeMap::new();
    for seg in &image.segments {
        for (i, &b) in seg.data.iter().enumerate() {
            let a = seg.addr.wrapping_add(i as u32);
            if !(FLASH_BASE..FLASH_BASE + flash_size).contains(&a) {
                continue;
            }
            let p = pages.entry(a & !(PAGE - 1)).or_insert_with(|| vec![0xFF; PAGE as usize]);
            p[(a % PAGE) as usize] = b;
        }
    }
    let mut out: Vec<(u32, Vec<u8>)> = Vec::new();
    for (a, p) in pages {
        match out.last_mut() {
            Some((start, data)) if *start + data.len() as u32 == a => data.extend(p),
            _ => out.push((a, p)),
        }
    }
    out
}

// ---------------------------------------------------------------------------------------------- mass storage

/// A USB mass storage (bulk-only transport, SCSI) drive, like the BOOTSEL "RPI-RP2" drive.
#[derive(Clone, Debug)]
pub struct MscDrive {
    pub interface: u8,
    pub ep_out: u8,
    pub ep_in: u8,
    tag: u32,
    pub block_size: u32,
    pub blocks: u32,
    pub timeout: Time,
}

/// Bulk-only transport signatures (USB MSC BOT 1.0, 5.1 and 5.2).
const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;

impl MscDrive {
    /// Waits for a device with a mass storage interface (class 8, SCSI transparent, bulk-only) and reads its
    /// capacity (after TEST UNIT READY and INQUIRY, as an OS does).
    pub fn open(r: &mut impl UsbRunner, timeout: Time) -> Result<MscDrive, Error> {
        let dev = wait_for_device(r, timeout, |d| d.interface(0x08).is_some_and(|i| i.protocol == 0x50))?;
        let i = dev.interface(0x08).unwrap();
        let mut d = MscDrive {
            interface: i.number,
            ep_out: i.endpoint(EndpointType::Bulk, false).map(|e| e.address).unwrap_or(0),
            ep_in: i.endpoint(EndpointType::Bulk, true).map(|e| e.address).unwrap_or(0x80),
            tag: 1,
            block_size: 512,
            blocks: 0,
            timeout: Time::from_secs(5),
        };
        let inquiry = d.scsi_in(r, &[0x12, 0, 0, 0, 36, 0], 36)?;
        if inquiry.len() < 36 {
            return Err(Error::Scsi(format!("short INQUIRY ({} bytes)", inquiry.len())));
        }
        d.scsi_in(r, &[0, 0, 0, 0, 0, 0], 0)?;
        let cap = d.scsi_in(r, &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8)?;
        if cap.len() < 8 {
            return Err(Error::Scsi("short READ CAPACITY".into()));
        }
        d.blocks = u32::from_be_bytes(cap[0..4].try_into().unwrap()) + 1;
        d.block_size = u32::from_be_bytes(cap[4..8].try_into().unwrap());
        Ok(d)
    }

    fn cbw(&mut self, cb: &[u8], len: u32, dir_in: bool) -> Vec<u8> {
        let tag = self.tag;
        self.tag += 1;
        let mut w = CBW_SIGNATURE.to_le_bytes().to_vec();
        w.extend(tag.to_le_bytes());
        w.extend(len.to_le_bytes());
        w.extend([if dir_in { 0x80 } else { 0 }, 0, cb.len() as u8]);
        let mut c = cb.to_vec();
        c.resize(16, 0);
        w.extend(c);
        w
    }

    fn csw(&self, r: &mut impl UsbRunner) -> Result<(), Error> {
        let ep = self.ep_in;
        let csw = transfer(r, self.timeout, |h| h.submit_bulk_in(ep, 13))?;
        if csw.len() != 13 || u32::from_le_bytes(csw[0..4].try_into().unwrap()) != CSW_SIGNATURE {
            return Err(Error::Scsi(format!("bad CSW {csw:02x?}")));
        }
        match csw[12] {
            0 => Ok(()),
            s => Err(Error::Scsi(format!("command failed, CSW status {s}"))),
        }
    }

    /// A SCSI command with an IN (or no) data phase.
    pub fn scsi_in(&mut self, r: &mut impl UsbRunner, cb: &[u8], len: u32) -> Result<Vec<u8>, Error> {
        let w = self.cbw(cb, len, true);
        let (o, i) = (self.ep_out, self.ep_in);
        transfer(r, self.timeout, |h| h.submit_bulk_out(o, w))?;
        let data = if len > 0 { transfer(r, self.timeout, |h| h.submit_bulk_in(i, len as usize))? } else { Vec::new() };
        self.csw(r)?;
        Ok(data)
    }

    /// A SCSI command with an OUT data phase.
    pub fn scsi_out(&mut self, r: &mut impl UsbRunner, cb: &[u8], data: &[u8]) -> Result<(), Error> {
        let w = self.cbw(cb, data.len() as u32, false);
        let o = self.ep_out;
        transfer(r, self.timeout, |h| h.submit_bulk_out(o, w))?;
        let d = data.to_vec();
        transfer(r, self.timeout, |h| h.submit_bulk_out(o, d))?;
        self.csw(r)
    }

    /// READ(10).
    pub fn read_blocks(&mut self, r: &mut impl UsbRunner, lba: u32, count: u16) -> Result<Vec<u8>, Error> {
        let mut cb = vec![0x28, 0];
        cb.extend(lba.to_be_bytes());
        cb.extend([0, (count >> 8) as u8, count as u8, 0]);
        self.scsi_in(r, &cb, count as u32 * self.block_size)
    }

    /// WRITE(10).
    pub fn write_blocks(&mut self, r: &mut impl UsbRunner, lba: u32, data: &[u8]) -> Result<(), Error> {
        let count = (data.len() as u32).div_ceil(self.block_size) as u16;
        let mut d = data.to_vec();
        d.resize(count as usize * self.block_size as usize, 0);
        let mut cb = vec![0x2A, 0];
        cb.extend(lba.to_be_bytes());
        cb.extend([0, (count >> 8) as u8, count as u8, 0]);
        self.scsi_out(r, &cb, &d)
    }

    /// Copies a UF2 file to the drive the way a file manager would end up writing its data: every 512 byte UF2
    /// block goes to consecutive sectors from `lba` (the bootrom recognises UF2 blocks in whatever sectors they
    /// land in, and reboots once it has seen every block of the file). The FAT and directory updates an OS would
    /// also write are left out.
    pub fn copy_uf2(&mut self, r: &mut impl UsbRunner, uf2: &[u8], lba: u32) -> Result<(), Error> {
        for (i, chunk) in uf2.chunks(8 * 512).enumerate() {
            self.write_blocks(r, lba + 8 * i as u32, chunk)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::image::Segment;

    #[test]
    fn flash_pages_pad_and_merge() {
        let image = Image {
            segments: vec![
                Segment { addr: 0x1000_0000, data: vec![1; 300] },
                Segment { addr: 0x1000_0200, data: vec![2; 4] },
                Segment { addr: 0x1000_1000, data: vec![3; 2] },
                Segment { addr: 0x2000_0000, data: vec![4; 8] },
            ],
            ..Image::default()
        };
        let p = flash_pages(&image, 2 << 20);
        assert_eq!(p.len(), 2);
        assert_eq!((p[0].0, p[0].1.len()), (0x1000_0000, 768));
        assert_eq!(p[0].1[299], 1);
        assert_eq!(p[0].1[300], 0xFF);
        assert_eq!(p[0].1[512], 2);
        assert_eq!((p[1].0, p[1].1.len()), (0x1000_1000, 256));
    }
}
