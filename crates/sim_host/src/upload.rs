//! An in-process `upload_tool`: flashes an image into a board running the C Titan bootloader, over the simulated CAN
//! bus, with the real `canmore` register mapped client.
//!
//! It follows titan_firmware_rs's `upload_tool` CAN path step by step: wait for the board's heartbeat in boot delay
//! mode, send the bootloader request on the control interface channel, wait for bootloader mode, check the
//! bootloader (magic, version, flash geometry), then write every page but the first (erasing each 4 KiB sector on
//! first use), verify them by CRC-32 computed on the device, write and verify the first page last ("sealing"), and
//! reboot. Waiting for an answer runs the simulation, so timeouts are in virtual time.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use canmore::id::{self, Direction, Fields, FrameType};
use canmore::reg_mapped::interface::bootloader::{self as itf, flash_control, mcu_control};
use canmore::reg_mapped::{self, Client, TransferMode};
use canmore::{CHAN_CONTROL_INTERFACE, CHAN_HEARTBEAT, ControlInterfaceMode, Heartbeat};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

use crate::session::Session;

const PAGE: usize = 256;
const SECTOR: u32 = 4096;
const FLASH_BASE: u32 = 0x1000_0000;

/// The CRC-32 of the CANmore bootloader (CRC-32/MPEG-2: polynomial 0x04C11DB7, initial 0xFFFFFFFF, no final XOR).
pub fn crc32_mpeg2(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= (b as u32) << 24;
        for _ in 0..8 {
            crc = if crc & (1 << 31) != 0 { (crc << 1) ^ 0x04C1_1DB7 } else { crc << 1 };
        }
    }
    crc
}

#[derive(Default)]
struct LinkState {
    tx: VecDeque<CanFrame>,
    /// Control interface responses from the client.
    rx: VecDeque<Vec<u8>>,
    heartbeats: Vec<(Time, Heartbeat)>,
}

/// The tool's side of the bus: one client's heartbeats and control interface channel.
#[derive(Clone)]
pub struct CanmoreLink {
    state: Arc<Mutex<LinkState>>,
    pub client_id: u8,
}

impl CanmoreLink {
    pub fn new(client_id: u8) -> Self {
        CanmoreLink { state: Arc::default(), client_id }
    }

    /// Control interface mode of the client's latest heartbeat after `since`.
    pub fn mode_since(&self, since: Time) -> Option<u8> {
        self.state.lock().unwrap().heartbeats.iter().rev().find(|h| h.0 > since).map(|h| h.1.mode)
    }
}

impl CanEndpoint for CanmoreLink {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.state.lock().unwrap().tx.drain(..).map(|f| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        let f = Fields::decode(frame.id, frame.extended);
        if f.client_id != self.client_id || f.direction != Direction::ClientToAgent || f.frame_type != FrameType::Util {
            return;
        }
        let mut s = self.state.lock().unwrap();
        match f.noc {
            CHAN_HEARTBEAT if frame.data.len() == 1 => s.heartbeats.push((at, Heartbeat::from_byte(frame.data[0]))),
            CHAN_CONTROL_INTERFACE if !frame.extended => s.rx.push_back(frame.data.clone()),
            _ => {}
        }
    }
}

/// The register mapped transport: classic frames on the control interface channel; receiving runs the session.
struct Transport<'a> {
    session: &'a mut Session,
    link: CanmoreLink,
    timeout: Time,
}

#[derive(Debug)]
pub struct TimedOut;

impl std::fmt::Display for TimedOut {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("register mapped response timed out")
    }
}

impl reg_mapped::Transport for Transport<'_> {
    type Error = TimedOut;

    fn mtu(&self) -> usize {
        canmore::MAX_FRAME_SIZE
    }

    fn clear_rx(&mut self) -> Result<(), TimedOut> {
        self.link.state.lock().unwrap().rx.clear();
        Ok(())
    }

    fn send(&mut self, data: &[u8]) -> Result<(), TimedOut> {
        let id = id::util_id(self.link.client_id, Direction::AgentToClient, CHAN_CONTROL_INTERFACE);
        self.link.state.lock().unwrap().tx.push_back(CanFrame::classic(id, false, data));
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, TimedOut> {
        let deadline = self.session.machine.now + self.timeout;
        loop {
            if let Some(p) = self.link.state.lock().unwrap().rx.pop_front() {
                let n = p.len().min(buf.len());
                buf[..n].copy_from_slice(&p[..n]);
                return Ok(n);
            }
            if self.session.machine.now >= deadline {
                return Err(TimedOut);
            }
            let next = self.session.machine.now + Time::from_us(50);
            self.session.run_until(next);
        }
    }
}

/// What an upload did.
#[derive(Clone, Debug, Default)]
pub struct UploadReport {
    pub bootloader_version: String,
    pub flash_size: u32,
    pub app_base: u32,
    pub flash_id: u64,
    pub pages_written: usize,
    pub sectors_erased: usize,
    pub started: Time,
    pub finished: Time,
}

/// The image's 256-byte pages (address, data), blank-filled.
pub fn pages(image: &Image) -> Vec<(u32, [u8; PAGE])> {
    let mut map = std::collections::BTreeMap::<u32, [u8; PAGE]>::new();
    for seg in &image.segments {
        for (i, &b) in seg.data.iter().enumerate() {
            let a = seg.addr + i as u32;
            if !(FLASH_BASE..FLASH_BASE + 0x0100_0000).contains(&a) {
                continue;
            }
            map.entry(a & !(PAGE as u32 - 1)).or_insert([0xFF; PAGE])[(a as usize) % PAGE] = b;
        }
    }
    map.into_iter().collect()
}

/// Runs the session until the client heartbeats in `mode`, or `timeout` passes.
fn wait_for_mode(
    session: &mut Session,
    link: &CanmoreLink,
    mode: ControlInterfaceMode,
    timeout: Time,
) -> Result<(), String> {
    let start = session.machine.now;
    let deadline = start + timeout;
    while session.machine.now < deadline {
        if link.mode_since(start) == Some(mode as u8) {
            return Ok(());
        }
        let next = session.machine.now + Time::from_ms(1);
        session.run_until(next);
    }
    Err(format!("client {} did not report {mode:?} within {timeout}", link.client_id))
}

/// Catches the board in its boot delay, puts it in the bootloader and flashes `image` as an over-the-air
/// application. `link` must be one of the session's CAN endpoints.
pub fn upload_ota(
    session: &mut Session,
    link: &CanmoreLink,
    image: &Image,
    wait: Time,
) -> Result<UploadReport, String> {
    let mut report = UploadReport { started: session.machine.now, ..Default::default() };
    wait_for_mode(session, link, ControlInterfaceMode::BootDelay, wait)?;
    // The raw bootloader request on the control interface
    let id = id::util_id(link.client_id, Direction::AgentToClient, CHAN_CONTROL_INTERFACE);
    link.state.lock().unwrap().tx.push_back(CanFrame::classic(id, false, &reg_mapped::BOOTLOADER_REQUEST));
    wait_for_mode(session, link, ControlInterfaceMode::Bootloader, Time::from_secs(2))?;

    let transport = Transport { session, link: link.clone(), timeout: Time::from_secs(1) };
    let mut c =
        Client::new(transport, ControlInterfaceMode::Bootloader as u8, TransferMode::Bulk { max_in_flight: 32 });
    fn e(what: &'static str) -> impl Fn(reg_mapped::ClientError<TimedOut>) -> String {
        move |err| format!("{what}: {err}")
    }
    let magic = c.read_register(itf::MCU_CONTROL_PAGE, mcu_control::MAGIC).map_err(e("magic"))?;
    if magic != mcu_control::MAGIC_VALUE {
        return Err(format!("unexpected bootloader magic {magic:#010x}"));
    }
    let major = c.read_register(itf::MCU_CONTROL_PAGE, mcu_control::MAJOR_VERSION).map_err(e("version"))?;
    let minor = c.read_register(itf::MCU_CONTROL_PAGE, mcu_control::MINOR_VERSION).map_err(e("version"))?;
    report.bootloader_version = format!("{major}.{minor}");
    let lower = c.read_register(itf::MCU_CONTROL_PAGE, mcu_control::LOWER_FLASH_ID).map_err(e("flash id"))?;
    let upper = c.read_register(itf::MCU_CONTROL_PAGE, mcu_control::UPPER_FLASH_ID).map_err(e("flash id"))?;
    report.flash_id = (upper as u64) << 32 | lower as u64;
    report.flash_size = c.read_register(itf::FLASH_CONTROL_PAGE, flash_control::FLASH_SIZE).map_err(e("flash size"))?;
    report.app_base = c.read_register(itf::FLASH_CONTROL_PAGE, flash_control::APP_BASE).map_err(e("app base"))?;
    // Lock the bootloader region in case an earlier session unlocked it
    c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::BL_WRITE_KEY, 0).map_err(e("lock"))?;

    let pages = pages(image);
    let Some(&(first_addr, _)) = pages.first() else { return Err("empty image".into()) };
    if first_addr < report.app_base {
        return Err(format!(
            "image at {first_addr:#010x} overlaps the bootloader (app base {:#010x})",
            report.app_base
        ));
    }
    if pages.last().unwrap().0 + PAGE as u32 > FLASH_BASE + report.flash_size {
        return Err("image does not fit in flash".into());
    }
    let mut erased = std::collections::BTreeSet::new();
    let mut write = |c: &mut Client<Transport>, addr: u32, data: &[u8; PAGE]| -> Result<(), String> {
        let sector = addr & !(SECTOR - 1);
        if erased.insert(sector) {
            c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, sector).map_err(e("erase"))?;
            c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_ERASE)
                .map_err(e("erase"))?;
        }
        c.write_bytes(itf::FLASH_BUFFER_PAGE, 0, data).map_err(e("fill buffer"))?;
        c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr).map_err(e("address"))?;
        c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_WRITE)
            .map_err(e("write"))
    };
    let verify = |c: &mut Client<Transport>, addr: u32, data: &[u8; PAGE]| -> Result<(), String> {
        c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr).map_err(e("address"))?;
        c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_READ)
            .map_err(e("read"))?;
        c.write_register(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_CRC)
            .map_err(e("crc"))?;
        let crc = c.read_register(itf::FLASH_CONTROL_PAGE, flash_control::CRC).map_err(e("crc"))?;
        if crc != crc32_mpeg2(data) {
            return Err(format!("verify failed at {addr:#010x}"));
        }
        Ok(())
    };
    // Everything but the first page, verify, then seal with the first page
    for (addr, data) in &pages[1..] {
        write(&mut c, *addr, data)?;
    }
    for (addr, data) in &pages[1..] {
        verify(&mut c, *addr, data)?;
    }
    write(&mut c, pages[0].0, &pages[0].1)?;
    verify(&mut c, pages[0].0, &pages[0].1)?;
    report.pages_written = pages.len();
    report.sectors_erased = erased.len();
    // The reboot request is not answered (the MCU resets): ignore the timeout
    let _ = c.write_register(itf::MCU_CONTROL_PAGE, mcu_control::REBOOT_MCU, 1);
    drop(c);
    report.finished = session.machine.now;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value_and_pages() {
        assert_eq!(crc32_mpeg2(b"123456789"), 0x0376_E6E7);
        let image = Image {
            segments: vec![sim_core::image::Segment { addr: 0x1000_4000, data: vec![1; 300] }],
            entry: None,
            symbols: vec![],
            elf: None,
        };
        let p = pages(&image);
        assert_eq!(p.len(), 2);
        assert_eq!((p[1].0, p[1].1[43], p[1].1[44]), (0x1000_4100, 1, 0xFF));
    }
}
