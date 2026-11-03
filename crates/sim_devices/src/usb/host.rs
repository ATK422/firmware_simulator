//! A USB host controller with one full speed root port, as a PC's would look to the device: it notices the
//! device's pull-up, resets the bus, enumerates the device and then runs the transfers clients queue on it.
//!
//! # Timing
//!
//! Times follow USB 2.0 chapter 7 and 9 and what Linux does: attach debounce 100 ms (TATTDB), bus reset 10 ms
//! (TDRST), reset recovery 10 ms (TRSTRCY), 2 ms after SET_ADDRESS (TDSETADDR). Once the port is enabled a SOF
//! starts every 1 ms frame. Each transaction takes the bus for the time its packets need at 12 Mbit/s (tokens,
//! data, handshakes and turnarounds, without bit stuffing), and a transaction that would run into the next SOF
//! waits for the next frame, so a bulk pipe peaks at nineteen 64 byte packets per frame (about 1.2 MB/s) as on a
//! real full speed bus. The host keeps one transfer active per endpoint and serves the active pipes round robin.
//!
//! A NAKed control or bulk pipe is retried as soon as the device changes something it can answer from (the SoC
//! calls [`HostController::kick`] when firmware arms a buffer or writes the controller's registers); a real host
//! retries continuously, which ends the same way. Interrupt pipes are polled once per bInterval frames.
//!
//! # Clients
//!
//! [`HostController::submit_control`], [`HostController::submit_bulk_out`], [`HostController::submit_bulk_in`] and
//! [`HostController::submit_interrupt_in`] queue transfers; [`HostController::take_result`] collects them. A CDC
//! ACM driver ([`HostController::cdc_open`]) opens a serial port on the device the way a terminal program does
//! (SET_LINE_CODING, SET_CONTROL_LINE_STATE with DTR and RTS) and keeps reading it.

use std::collections::{BTreeMap, VecDeque};

use sim_core::time::Time;

use super::*;

/// USB 2.0 7.1.7.3: debounce interval after a connect.
pub const ATTACH_DEBOUNCE: Time = Time::from_ms(100);
/// Bus reset length (USB 2.0 7.1.7.5, TDRST minimum for a hub port).
pub const RESET_TIME: Time = Time::from_ms(10);
/// Idle time after which a device suspends (USB 2.0 7.1.7.6).
pub const SUSPEND_IDLE: Time = Time::from_ms(3);
/// How long the host drives resume signalling (TDRSMDN).
pub const RESUME_TIME: Time = Time::from_ms(20);
/// Reset recovery before the first request (TRSTRCY).
pub const RESET_RECOVERY: Time = Time::from_ms(10);
/// Recovery after SET_ADDRESS (TDSETADDR).
pub const SET_ADDRESS_RECOVERY: Time = Time::from_ms(2);
/// Linux's default control transfer timeout (USB_CTRL_GET_TIMEOUT).
pub const CONTROL_TIMEOUT: Time = Time::from_secs(5);

/// Packet lengths in bit times: sync + PID + fields + CRC + EOP.
const TOKEN_BITS: u64 = 35;
const HANDSHAKE_BITS: u64 = 19;
const DATA_OVERHEAD_BITS: u64 = 35;
/// Inter-packet gap / bus turnaround, and the host's response timeout.
const GAP_BITS: u64 = 8;
const TIMEOUT_BITS: u64 = 18;
/// Bit times kept free at the end of a frame (EOF1/EOF2).
const EOF_BITS: u64 = 32;

const CDC_COMM_CLASS: u8 = 0x02;
const CDC_DATA_CLASS: u8 = 0x0A;
const CDC_SET_LINE_CODING: u8 = 0x20;
const CDC_SET_CONTROL_LINE_STATE: u8 = 0x22;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortState {
    /// Nothing attached (no pull-up).
    Detached,
    /// Pull-up seen; waiting out the debounce interval.
    Debounce(Time),
    /// Driving a bus reset until the given time.
    Reset(Time),
    /// Port enabled after the reset; waiting for the device to recover.
    Recovery(Time),
    /// Port enabled, SOFs running.
    Enabled,
    /// Port suspended (no SOFs) since the given time; `notified` once the device saw 3 ms of idle bus.
    Suspended { since: Time, notified: bool },
    /// Driving resume signalling until the given time, then SOFs start again.
    Resuming(Time),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage {
    Setup,
    DataIn,
    DataOut,
    StatusIn,
    StatusOut,
    /// A bulk or interrupt transfer, in the endpoint's direction.
    Data,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Control(SetupPacket),
    Bulk,
    Interrupt,
}

#[derive(Clone, Debug)]
struct Transfer {
    id: u32,
    addr: u8,
    /// Endpoint address (bit 7 set for IN); 0 for control transfers.
    ep: u8,
    kind: Kind,
    stage: Stage,
    mps: usize,
    out: Vec<u8>,
    sent: usize,
    /// End an OUT transfer whose length is a multiple of the packet size with a zero length packet.
    zlp: bool,
    /// At least one packet went out (a zero length transfer is one empty packet).
    any_sent: bool,
    want: usize,
    got: Vec<u8>,
    deadline: Option<Time>,
    /// Consecutive transactions without a response.
    fails: u8,
    /// NAKed since the last kick.
    blocked: bool,
    /// Interrupt pipes: the first frame the next poll may happen in, and the polling interval.
    next_frame: u64,
    interval: u64,
}

impl Transfer {
    fn pipe(&self) -> (u8, u8) {
        (self.addr, if matches!(self.kind, Kind::Control(_)) { 0 } else { self.ep })
    }

    fn is_in(&self) -> bool {
        match self.stage {
            Stage::DataIn | Stage::StatusIn => true,
            Stage::Data => self.ep & 0x80 != 0,
            _ => false,
        }
    }
}

/// Where enumeration has got to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Enumeration {
    Idle,
    /// Waiting for the transfer with this id, in the given step.
    Running(EnumStep, u32),
    /// Waiting after SET_ADDRESS.
    AddressRecovery(Time),
    Configured,
    Failed(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnumStep {
    DeviceDescriptor64,
    SetAddress,
    DeviceDescriptor,
    ConfigHeader,
    Config,
    Languages,
    String(u8),
    SetConfiguration,
}

/// The host side of a CDC ACM serial port.
#[derive(Clone, Debug, Default)]
pub struct CdcPort {
    /// Which CDC function of the device (0 = the first).
    pub index: usize,
    /// A client opened or used this port (the others are left alone).
    used: bool,
    /// Line coding baud rate the host sets.
    pub baud: u32,
    /// DTR and RTS asserted (the port is open).
    pub open: bool,
    /// Bytes received from the device not yet taken.
    pub rx: Vec<u8>,
    /// Bytes waiting to be sent.
    pub tx: VecDeque<u8>,
    /// SET_LINE_CODING / SET_CONTROL_LINE_STATE have been sent since the device was configured.
    pub configured: bool,
    comm_if: u8,
    data_in: u8,
    data_out: u8,
    reading: Option<u32>,
    writing: Option<u32>,
    requests: Vec<u32>,
    /// Line state / coding changes still to send.
    dirty: bool,
}

#[derive(Clone, Debug)]
pub struct HostController {
    pub state: PortState,
    pub enumeration: Enumeration,
    /// What enumeration learnt, once the device is configured (and partly while enumerating).
    pub device: Option<DeviceInfo>,
    /// Enumerate automatically after a connect (otherwise the port stays enabled with the device at address 0).
    pub auto_enumerate: bool,
    /// The CDC ACM ports opened (or written to) so far, by index ([`HostController::cdc_open`]).
    pub cdc: Vec<CdcPort>,
    /// Number of bus resets and completed enumerations so far.
    pub resets: u32,
    pub enumerations: u32,
    /// Transactions carried out, and how many of them the device NAKed.
    pub transactions: u64,
    pub naks: u64,
    frame0: Time,
    last_sof: Option<u64>,
    bus_free: Time,
    queue: Vec<Transfer>,
    results: BTreeMap<u32, Result<Vec<u8>, UsbError>>,
    toggles: BTreeMap<(u8, u8), Pid>,
    next_id: u32,
    next_address: u8,
    pending_address: u8,
    resume_signalled: bool,
    rr: usize,
    reports: Vec<(Time, String)>,
}

impl Default for HostController {
    fn default() -> Self {
        HostController::new()
    }
}

impl HostController {
    pub fn new() -> HostController {
        HostController {
            state: PortState::Detached,
            enumeration: Enumeration::Idle,
            device: None,
            auto_enumerate: true,
            cdc: Vec::new(),
            resets: 0,
            enumerations: 0,
            transactions: 0,
            naks: 0,
            frame0: Time::ZERO,
            last_sof: None,
            bus_free: Time::ZERO,
            queue: Vec::new(),
            results: BTreeMap::new(),
            toggles: BTreeMap::new(),
            next_id: 1,
            next_address: 1,
            pending_address: 0,
            resume_signalled: false,
            rr: 0,
            reports: Vec::new(),
        }
    }

    /// The configured device, once enumeration finished.
    pub fn configured(&self) -> Option<&DeviceInfo> {
        if self.enumeration == Enumeration::Configured { self.device.as_ref() } else { None }
    }

    /// Messages about what happened on the port (connects, resets, enumeration, errors), for the trace.
    pub fn take_reports(&mut self) -> Vec<(Time, String)> {
        std::mem::take(&mut self.reports)
    }

    fn report(&mut self, now: Time, msg: String) {
        log::debug!("[{now}] usb host: {msg}");
        if self.reports.len() < 10_000 {
            self.reports.push((now, msg));
        }
    }

    /// The frame `now` falls in (frames count from the end of the last bus reset).
    pub fn frame_at(&self, now: Time) -> u64 {
        (now.saturating_sub(self.frame0)).as_ps() / FRAME.as_ps()
    }

    fn frame_start(&self, frame: u64) -> Time {
        self.frame0 + Time::from_ps(frame * FRAME.as_ps())
    }

    // ------------------------------------------------------------------------------------------ client API

    fn add(&mut self, t: Transfer) -> u32 {
        let id = t.id;
        self.queue.push(t);
        id
    }

    fn new_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1).max(1);
        id
    }

    fn address(&self) -> u8 {
        self.device.as_ref().map_or(0, |d| d.address)
    }

    /// Queues a control transfer to the device's endpoint 0. `data` is the OUT data stage (its length must match
    /// `setup.length`); IN requests return up to `setup.length` bytes.
    pub fn submit_control(&mut self, setup: SetupPacket, data: Vec<u8>) -> u32 {
        let addr = self.address();
        let mps = self.device.as_ref().map_or(64, |d| d.max_packet0() as usize).max(8);
        self.control_to(addr, mps, setup, data)
    }

    fn control_to(&mut self, addr: u8, mps: usize, setup: SetupPacket, data: Vec<u8>) -> u32 {
        let id = self.new_id();
        self.add(Transfer {
            id,
            addr,
            ep: 0,
            kind: Kind::Control(setup),
            stage: Stage::Setup,
            mps,
            out: data,
            sent: 0,
            zlp: false,
            any_sent: false,
            want: if setup.is_in() { setup.length as usize } else { 0 },
            got: Vec::new(),
            deadline: None,
            fails: 0,
            blocked: false,
            next_frame: 0,
            interval: 1,
        })
    }

    fn data_transfer(&mut self, ep: u8, kind: Kind, out: Vec<u8>, want: usize, zlp: bool) -> u32 {
        let id = self.new_id();
        let info = self.device.as_ref().and_then(|d| d.endpoint(ep)).cloned();
        let mps = info.as_ref().map_or(64, |e| e.max_packet as usize).max(1);
        let interval = info.as_ref().map_or(1, |e| e.interval.max(1) as u64);
        let addr = self.address();
        self.add(Transfer {
            id,
            addr,
            ep,
            kind,
            stage: Stage::Data,
            mps,
            out,
            sent: 0,
            zlp,
            any_sent: false,
            want,
            got: Vec::new(),
            deadline: None,
            fails: 0,
            blocked: false,
            next_frame: 0,
            interval,
        })
    }

    /// Queues a bulk OUT transfer to endpoint `ep` (an OUT endpoint address). An empty `data` sends one zero
    /// length packet.
    pub fn submit_bulk_out(&mut self, ep: u8, data: Vec<u8>) -> u32 {
        self.data_transfer(ep & 0x0F, Kind::Bulk, data, 0, false)
    }

    /// Like [`HostController::submit_bulk_out`], ending with a zero length packet if `data` fills its last packet.
    pub fn submit_bulk_out_zlp(&mut self, ep: u8, data: Vec<u8>) -> u32 {
        self.data_transfer(ep & 0x0F, Kind::Bulk, data, 0, true)
    }

    /// Queues a bulk IN transfer of up to `len` bytes from endpoint `ep`: it ends at a short packet or when `len`
    /// bytes arrived. `len` 0 reads one zero length packet.
    pub fn submit_bulk_in(&mut self, ep: u8, len: usize) -> u32 {
        self.data_transfer(ep | 0x80, Kind::Bulk, Vec::new(), len, false)
    }

    /// Queues an interrupt IN transfer (polled every bInterval frames).
    pub fn submit_interrupt_in(&mut self, ep: u8, len: usize) -> u32 {
        self.data_transfer(ep | 0x80, Kind::Interrupt, Vec::new(), len, false)
    }

    /// Gives a queued transfer a deadline: if it has not finished by then it ends with [`UsbError::TimedOut`].
    pub fn set_deadline(&mut self, id: u32, at: Time) {
        if let Some(t) = self.queue.iter_mut().find(|t| t.id == id) {
            t.deadline = Some(at);
        }
    }

    /// The result of a finished transfer (removing it), or `None` while it runs.
    pub fn take_result(&mut self, id: u32) -> Option<Result<Vec<u8>, UsbError>> {
        self.results.remove(&id)
    }

    /// Whether a transfer is still queued or running.
    pub fn pending(&self, id: u32) -> bool {
        self.queue.iter().any(|t| t.id == id)
    }

    /// Cancels a queued transfer (it ends with [`UsbError::TimedOut`]), as a driver that gives up on it does.
    pub fn cancel(&mut self, id: u32) {
        if let Some(i) = self.queue.iter().position(|t| t.id == id) {
            self.queue.remove(i);
            self.results.insert(id, Err(UsbError::TimedOut));
        }
    }

    /// Resets the host's data toggle for an endpoint to DATA0, as after CLEAR_FEATURE(ENDPOINT_HALT).
    pub fn reset_toggle(&mut self, ep: u8) {
        let addr = self.address();
        self.toggles.remove(&(addr, ep));
    }

    /// Queues CLEAR_FEATURE(ENDPOINT_HALT) for `ep` (which resets the host's toggle for it once it succeeds).
    pub fn clear_halt(&mut self, ep: u8) -> u32 {
        self.submit_control(SetupPacket::new(req_type::ENDPOINT, request::CLEAR_FEATURE, 0, ep as u16, 0), Vec::new())
    }

    /// Suspends the port (selective suspend): SOFs and transfers stop, and after 3 ms of idle bus the device
    /// suspends.
    pub fn suspend(&mut self, now: Time) {
        if self.state == PortState::Enabled {
            self.report(now, "port suspended".into());
            self.state = PortState::Suspended { since: now, notified: false };
        }
    }

    /// Resumes a suspended port: 20 ms of resume signalling, then SOFs and transfers carry on.
    pub fn resume(&mut self, now: Time) {
        if let PortState::Suspended { .. } = self.state {
            self.report(now, "resume".into());
            self.resume_signalled = false;
            self.state = PortState::Resuming(now + RESUME_TIME);
        }
    }

    /// Resets the port and enumerates the device again (`usbreset`).
    pub fn reenumerate(&mut self, now: Time) {
        if self.state != PortState::Detached {
            self.fail_all(UsbError::Disconnected);
            self.state = PortState::Debounce(now);
        }
    }

    // ------------------------------------------------------------------------------------------ CDC ACM

    fn cdc_port(&mut self, port: usize) -> &mut CdcPort {
        while self.cdc.len() <= port {
            let index = self.cdc.len();
            self.cdc.push(CdcPort { index, ..CdcPort::default() });
        }
        let c = &mut self.cdc[port];
        c.used = true;
        c
    }

    /// Opens the device's CDC ACM port `port` (0 for the first CDC function; now, or once the device is
    /// configured): line coding `baud` 8N1, DTR and RTS on.
    pub fn cdc_open(&mut self, port: usize, baud: u32) {
        let cdc = self.cdc_port(port);
        cdc.baud = baud;
        cdc.open = true;
        cdc.dirty = true;
    }

    /// Changes the line coding (1200 baud makes pico-sdk stdio reboot into BOOTSEL).
    pub fn cdc_set_baud(&mut self, port: usize, baud: u32) {
        let c = self.cdc_port(port);
        c.baud = baud;
        c.dirty = true;
    }

    /// Drops DTR and RTS (the terminal closes the port).
    pub fn cdc_close(&mut self, port: usize) {
        let c = self.cdc_port(port);
        c.open = false;
        c.dirty = true;
    }

    /// Queues bytes for the device.
    pub fn cdc_write(&mut self, port: usize, bytes: &[u8]) {
        self.cdc_port(port).tx.extend(bytes);
    }

    /// Takes what the device sent.
    pub fn cdc_take_rx(&mut self, port: usize) -> Vec<u8> {
        self.cdc.get_mut(port).map(|c| std::mem::take(&mut c.rx)).unwrap_or_default()
    }

    fn drive_cdc(&mut self, now: Time) {
        let mut ports = std::mem::take(&mut self.cdc);
        for c in &mut ports {
            self.drive_cdc_inner(now, c);
        }
        self.cdc = ports;
    }

    fn drive_cdc_inner(&mut self, now: Time, cdc: &mut CdcPort) {
        if !cdc.used {
            return;
        }
        // Collect finished requests
        cdc.requests.retain(|id| match self.results.remove(id) {
            None => true,
            Some(Ok(_)) => false,
            Some(Err(e)) => {
                log::warn!("[{now}] CDC control request failed: {e}");
                false
            }
        });
        if let Some(id) = cdc.reading
            && let Some(r) = self.results.remove(&id)
        {
            cdc.reading = None;
            match r {
                Ok(data) => cdc.rx.extend(data),
                Err(e) => log::warn!("[{now}] CDC read failed: {e}"),
            }
        }
        if let Some(id) = cdc.writing
            && let Some(r) = self.results.remove(&id)
        {
            cdc.writing = None;
            if let Err(e) = r {
                log::warn!("[{now}] CDC write failed: {e}");
            }
        }
        let Some(dev) = self.configured() else {
            cdc.configured = false;
            cdc.reading = None;
            cdc.writing = None;
            cdc.requests.clear();
            return;
        };
        if !cdc.configured {
            // The n-th communication interface and the n-th data interface make port n
            let nth = |class: u8| dev.interfaces.iter().filter(|i| i.class == class && i.alternate == 0).nth(cdc.index);
            let (Some(comm), Some(data)) = (nth(CDC_COMM_CLASS), nth(CDC_DATA_CLASS)) else {
                return;
            };
            let (Some(i), Some(o)) =
                (data.endpoint(EndpointType::Bulk, true), data.endpoint(EndpointType::Bulk, false))
            else {
                return;
            };
            cdc.comm_if = comm.number;
            cdc.data_in = i.address;
            cdc.data_out = o.address;
            cdc.configured = true;
            cdc.dirty = true;
        }
        if cdc.dirty && cdc.requests.is_empty() {
            cdc.dirty = false;
            let ty = req_type::CLASS | req_type::INTERFACE;
            let mut coding = cdc.baud.to_le_bytes().to_vec();
            coding.extend([0, 0, 8]);
            let iface = cdc.comm_if as u16;
            let a = self.submit_control(SetupPacket::new(ty, CDC_SET_LINE_CODING, 0, iface, 7), coding);
            let lines = if cdc.open { 3 } else { 0 };
            let b = self.submit_control(SetupPacket::new(ty, CDC_SET_CONTROL_LINE_STATE, lines, iface, 0), Vec::new());
            cdc.requests.extend([a, b]);
        }
        // Like a tty, the port is only read while it is open
        if cdc.reading.is_none() && cdc.open {
            cdc.reading = Some(self.submit_bulk_in(cdc.data_in, 64));
        }
        if cdc.writing.is_none() && !cdc.tx.is_empty() {
            let n = cdc.tx.len().min(256);
            let chunk: Vec<u8> = cdc.tx.drain(..n).collect();
            cdc.writing = Some(self.submit_bulk_out(cdc.data_out, chunk));
        }
    }

    // ------------------------------------------------------------------------------------------ enumeration

    fn drive_enumeration(&mut self, now: Time) {
        loop {
            let before = self.enumeration.clone();
            self.enumeration_step(now);
            if self.enumeration == before {
                return;
            }
        }
    }

    fn enumeration_step(&mut self, now: Time) {
        let (step, id) = match self.enumeration.clone() {
            Enumeration::Idle => {
                if self.auto_enumerate && self.state == PortState::Enabled {
                    self.device = Some(DeviceInfo::default());
                    let s = SetupPacket::get_descriptor(desc_type::DEVICE, 0, 0, 64);
                    let id = self.control_to(0, 64, s, Vec::new());
                    self.enumeration = Enumeration::Running(EnumStep::DeviceDescriptor64, id);
                }
                return;
            }
            Enumeration::AddressRecovery(until) => {
                if now >= until {
                    self.get(EnumStep::DeviceDescriptor, SetupPacket::get_descriptor(desc_type::DEVICE, 0, 0, 18));
                }
                return;
            }
            Enumeration::Configured | Enumeration::Failed(_) => return,
            Enumeration::Running(step, id) => (step, id),
        };
        let Some(result) = self.results.remove(&id) else { return };
        let data = match result {
            Ok(d) => d,
            Err(e) => {
                // Strings are optional; anything else ends enumeration
                if let EnumStep::String(_) | EnumStep::Languages = step {
                    self.next_string(step);
                    return;
                }
                let msg = format!("enumeration failed at {step:?}: {e}");
                self.report(now, msg.clone());
                log::warn!("[{now}] USB {msg}");
                self.enumeration = Enumeration::Failed(msg);
                return;
            }
        };
        let dev = self.device.get_or_insert_with(DeviceInfo::default);
        match step {
            EnumStep::DeviceDescriptor64 => {
                if data.len() < 8 || data[1] != desc_type::DEVICE {
                    self.enumeration = Enumeration::Failed(format!("bad device descriptor {data:02x?}"));
                    return;
                }
                dev.device_descriptor = data;
                let addr = self.next_address;
                self.next_address = if addr >= 127 { 1 } else { addr + 1 };
                let mps = dev.max_packet0() as usize;
                let s = SetupPacket::new(0, request::SET_ADDRESS, addr as u16, 0, 0);
                let id = self.control_to(0, mps, s, Vec::new());
                self.enumeration = Enumeration::Running(EnumStep::SetAddress, id);
                self.pending_address = addr;
            }
            EnumStep::SetAddress => {
                dev.address = self.pending_address;
                self.report(now, format!("device at address {}", self.pending_address));
                self.enumeration = Enumeration::AddressRecovery(now + SET_ADDRESS_RECOVERY);
            }
            EnumStep::DeviceDescriptor => {
                dev.device_descriptor = data;
                self.get(EnumStep::ConfigHeader, SetupPacket::get_descriptor(desc_type::CONFIGURATION, 0, 0, 9));
            }
            EnumStep::ConfigHeader => {
                let total = if data.len() >= 4 { u16::from_le_bytes([data[2], data[3]]) } else { 9 };
                self.get(EnumStep::Config, SetupPacket::get_descriptor(desc_type::CONFIGURATION, 0, 0, total));
            }
            EnumStep::Config => {
                dev.interfaces = parse_configuration(&data);
                dev.config_descriptor = data;
                self.get(EnumStep::Languages, SetupPacket::get_descriptor(desc_type::STRING, 0, 0, 255));
            }
            EnumStep::Languages => self.next_string(step),
            EnumStep::String(i) => {
                let s = parse_string(&data);
                let dd = &dev.device_descriptor;
                if dd.get(14) == Some(&i) {
                    dev.manufacturer = s.clone();
                }
                if dd.get(15) == Some(&i) {
                    dev.product = s.clone();
                }
                if dd.get(16) == Some(&i) {
                    dev.serial = s;
                }
                self.next_string(step);
            }
            EnumStep::SetConfiguration => {
                // (`finish` reset the bulk and interrupt toggles)
                self.enumerations += 1;
                self.enumeration = Enumeration::Configured;
                let d = self.device.clone().unwrap_or_default();
                self.report(
                    now,
                    format!(
                        "configured {:04x}:{:04x} {} {} (serial {}), {} interface(s)",
                        d.vendor_id(),
                        d.product_id(),
                        d.manufacturer.as_deref().unwrap_or("?"),
                        d.product.as_deref().unwrap_or("?"),
                        d.serial.as_deref().unwrap_or("?"),
                        d.interfaces.len()
                    ),
                );
            }
        }
    }

    fn get(&mut self, step: EnumStep, setup: SetupPacket) {
        let id = self.submit_control(setup, Vec::new());
        self.enumeration = Enumeration::Running(step, id);
    }

    /// Asks for the next of the manufacturer, product and serial strings, then sets configuration 1.
    fn next_string(&mut self, after: EnumStep) {
        let dd = self.device.as_ref().map(|d| d.device_descriptor.clone()).unwrap_or_default();
        let indices: Vec<u8> = [14, 15, 16].iter().filter_map(|&o| dd.get(o).copied()).filter(|&i| i != 0).collect();
        let pos = match after {
            EnumStep::String(i) => indices.iter().position(|&x| x == i).map_or(indices.len(), |p| p + 1),
            _ => 0,
        };
        match indices.get(pos) {
            Some(&i) => self.get(EnumStep::String(i), SetupPacket::get_descriptor(desc_type::STRING, i, 0x0409, 255)),
            None => {
                let config = self.device.as_ref().and_then(|d| d.config_descriptor.get(5).copied()).unwrap_or(1);
                let s = SetupPacket::new(0, request::SET_CONFIGURATION, config as u16, 0, 0);
                self.get(EnumStep::SetConfiguration, s);
            }
        }
    }

    // ------------------------------------------------------------------------------------------ port and bus

    fn fail_all(&mut self, e: UsbError) {
        for t in self.queue.drain(..) {
            self.results.insert(t.id, Err(e.clone()));
        }
        self.toggles.clear();
        self.enumeration = Enumeration::Idle;
        self.device = None;
    }

    fn step_port(&mut self, now: Time, port: &mut dyn UsbDevicePort) {
        let pullup = port.pullup();
        loop {
            match self.state {
                PortState::Detached if pullup => {
                    self.report(now, "device attached".into());
                    self.state = PortState::Debounce(now + ATTACH_DEBOUNCE);
                }
                PortState::Detached => return,
                _ if !pullup => {
                    if matches!(self.state, PortState::Reset(_)) {
                        port.bus_reset(now, false);
                    }
                    self.report(now, "device detached".into());
                    self.state = PortState::Detached;
                    self.fail_all(UsbError::Disconnected);
                    return;
                }
                PortState::Debounce(t) if now >= t => {
                    self.fail_all(UsbError::Disconnected);
                    self.resets += 1;
                    self.report(now, "bus reset".into());
                    port.bus_reset(now, true);
                    self.state = PortState::Reset(now + RESET_TIME);
                }
                PortState::Reset(t) if now >= t => {
                    port.bus_reset(now, false);
                    self.frame0 = now;
                    self.last_sof = None;
                    self.bus_free = now;
                    self.state = PortState::Recovery(now + RESET_RECOVERY);
                }
                PortState::Recovery(t) if now >= t => self.state = PortState::Enabled,
                PortState::Suspended { since, notified: false } if now >= since + SUSPEND_IDLE => {
                    port.suspend(now, true);
                    self.state = PortState::Suspended { since, notified: true };
                }
                PortState::Suspended { notified: true, .. } if port.remote_wakeup() => {
                    self.report(now, "remote wakeup".into());
                    self.resume_signalled = false;
                    self.state = PortState::Resuming(now + RESUME_TIME);
                }
                PortState::Resuming(_) if !self.resume_signalled => {
                    // The device sees the K state
                    port.suspend(now, false);
                    self.resume_signalled = true;
                }
                PortState::Resuming(t) if now >= t => {
                    // The frame counter kept running; the next SOF is the current frame's
                    self.last_sof = Some(self.frame_at(now).saturating_sub(1));
                    self.bus_free = now;
                    self.state = PortState::Enabled;
                }
                _ => return,
            }
        }
    }

    fn sof_running(&self) -> bool {
        matches!(self.state, PortState::Recovery(_) | PortState::Enabled)
    }

    fn deliver_sof(&mut self, now: Time, port: &mut dyn UsbDevicePort) {
        if !self.sof_running() {
            return;
        }
        let frame = self.frame_at(now);
        if self.last_sof.is_none_or(|f| frame > f) {
            self.last_sof = Some(frame);
            let start = self.frame_start(frame);
            port.sof(start, (frame & 0x7FF) as u16);
            self.bus_free = self.bus_free.max(start + bits(TOKEN_BITS));
        }
    }

    /// Whether transfer `i` could do a transaction at `now` (ignoring the bus being busy).
    fn runnable(&self, i: usize, now: Time) -> bool {
        let t = &self.queue[i];
        if self.queue[..i].iter().any(|o| o.pipe() == t.pipe()) {
            return false;
        }
        match t.kind {
            Kind::Interrupt => self.frame_at(now) >= t.next_frame,
            _ => !t.blocked,
        }
    }

    /// Runs the host up to `now`: port state, SOFs, enumeration and at most one transaction. Returns when it
    /// next needs to run.
    pub fn poll(&mut self, now: Time, port: &mut dyn UsbDevicePort) -> Option<Time> {
        self.step_port(now, port);
        self.deliver_sof(now, port);
        if self.state == PortState::Enabled {
            self.drive_enumeration(now);
            self.drive_cdc(now);
            self.expire(now);
            if now >= self.bus_free {
                self.transact(now, port);
                self.drive_enumeration(now);
                self.drive_cdc(now);
            }
        }
        self.next_wake(now)
    }

    /// The device changed something (armed a buffer, wrote a register): NAKed pipes are retried and attach or
    /// detach is noticed. Returns when the host next needs to run.
    pub fn kick(&mut self, now: Time, port: &mut dyn UsbDevicePort) -> Option<Time> {
        for t in &mut self.queue {
            t.blocked = false;
        }
        let was = self.state;
        self.step_port(now, port);
        if self.state != was {
            self.drive_enumeration(now);
        }
        self.next_wake(now)
    }

    /// When the host next needs to run, given what is queued.
    pub fn next_wake(&self, now: Time) -> Option<Time> {
        let mut next: Option<Time> = None;
        let mut at = |t: Time| next = Some(next.map_or(t, |n: Time| n.min(t)));
        match self.state {
            PortState::Detached => {}
            PortState::Debounce(t) | PortState::Reset(t) | PortState::Resuming(t) => at(t),
            PortState::Suspended { since, notified: false } => at(since + SUSPEND_IDLE),
            PortState::Suspended { notified: true, .. } => {}
            PortState::Recovery(t) => {
                at(t);
                at(self.frame_start(self.frame_at(now) + 1));
            }
            PortState::Enabled => {
                at(self.frame_start(self.frame_at(now) + 1));
                if let Enumeration::AddressRecovery(t) = self.enumeration {
                    at(t);
                }
                for (i, t) in self.queue.iter().enumerate() {
                    if let Some(d) = t.deadline {
                        at(d);
                    }
                    if self.runnable(i, now) {
                        at(self.bus_free.max(now));
                    } else if t.kind == Kind::Interrupt && !self.queue[..i].iter().any(|o| o.pipe() == t.pipe()) {
                        at(self.frame_start(t.next_frame).max(self.bus_free));
                    }
                }
            }
        }
        next.map(|t| t.max(now))
    }

    fn expire(&mut self, now: Time) {
        for t in &mut self.queue {
            if matches!(t.kind, Kind::Control(_)) && t.deadline.is_none() {
                t.deadline = Some(now + CONTROL_TIMEOUT);
            }
        }
        let mut i = 0;
        while i < self.queue.len() {
            if self.queue[i].deadline.is_some_and(|d| now >= d) {
                let t = self.queue.remove(i);
                self.report(now, format!("transfer {} on ep {:#04x} timed out", t.id, t.ep));
                self.results.insert(t.id, Err(UsbError::TimedOut));
            } else {
                i += 1;
            }
        }
    }

    fn finish(&mut self, i: usize, r: Result<Vec<u8>, UsbError>) {
        let t = self.queue.remove(i);
        // Standard requests that reset data toggles on the device reset the host's too (USB 2.0 9.4.5, 9.4.10:
        // SET_CONFIGURATION and SET_INTERFACE reset every endpoint, CLEAR_FEATURE(ENDPOINT_HALT) one)
        if let (Kind::Control(s), Ok(_)) = (t.kind, &r)
            && s.request_type & 0x60 == req_type::STANDARD
        {
            match s.request {
                request::SET_CONFIGURATION | request::SET_INTERFACE => {
                    self.toggles.retain(|(a, ep), _| *a != t.addr || *ep == 0);
                }
                request::CLEAR_FEATURE if s.request_type & 0x1F == req_type::ENDPOINT && s.value == 0 => {
                    self.toggles.remove(&(t.addr, s.index as u8));
                }
                _ => {}
            }
        }
        self.results.insert(t.id, r);
    }

    fn toggle(&self, addr: u8, ep: u8) -> Pid {
        self.toggles.get(&(addr, ep)).copied().unwrap_or(Pid::Data0)
    }

    /// Picks the next runnable transfer round robin and carries out one transaction for it.
    fn transact(&mut self, now: Time, port: &mut dyn UsbDevicePort) {
        let n = self.queue.len();
        let Some(i) = (0..n).map(|k| (self.rr + k) % n).find(|&i| self.runnable(i, now)) else { return };
        // Don't run into the next frame's SOF
        let t = &self.queue[i];
        let payload = if t.is_in() {
            t.mps.min(t.want.saturating_sub(t.got.len()))
        } else if t.stage == Stage::Setup {
            8
        } else {
            t.mps.min(t.out.len().saturating_sub(t.sent))
        };
        let worst = bits(TOKEN_BITS + 2 * GAP_BITS + DATA_OVERHEAD_BITS + 8 * payload as u64 + HANDSHAKE_BITS);
        let frame_end = self.frame_start(self.frame_at(now) + 1) - bits(EOF_BITS);
        if now + worst > frame_end {
            self.bus_free = self.frame_start(self.frame_at(now) + 1);
            return;
        }
        self.rr = (i + 1) % n.max(1);
        self.transactions += 1;
        let t = self.queue[i].clone();
        let ep_num = t.ep & 0x0F;
        let mut t = t;
        let (dur, done) = match t.stage {
            Stage::Setup => {
                let Kind::Control(setup) = t.kind else { unreachable!() };
                let hs = port.setup(now, t.addr, &setup.bytes());
                let d = bits(TOKEN_BITS + GAP_BITS / 4 + DATA_OVERHEAD_BITS + 64 + GAP_BITS + HANDSHAKE_BITS);
                match hs {
                    Handshake::Ack => {
                        t.fails = 0;
                        self.toggles.insert((t.addr, 0), Pid::Data1);
                        t.stage = if setup.length == 0 {
                            Stage::StatusIn
                        } else if setup.is_in() {
                            Stage::DataIn
                        } else {
                            Stage::DataOut
                        };
                        (d, None)
                    }
                    Handshake::Timeout => self.no_response(&mut t, d),
                    // A device must accept every SETUP (USB 2.0 8.5.3); treat anything else as a failure
                    Handshake::Nak | Handshake::Stall => (d, Some(Err(UsbError::Stall))),
                }
            }
            Stage::DataIn | Stage::StatusIn | Stage::Data if t.is_in() => {
                let key = (t.addr, if t.ep & 0x80 != 0 && t.stage == Stage::Data { t.ep } else { 0 });
                match port.in_token(now, t.addr, ep_num) {
                    InReply::Data(pid, data) => {
                        t.fails = 0;
                        let d = bits(
                            TOKEN_BITS
                                + GAP_BITS
                                + DATA_OVERHEAD_BITS
                                + 8 * data.len() as u64
                                + GAP_BITS
                                + HANDSHAKE_BITS,
                        );
                        let expected = self.toggle(key.0, key.1);
                        if pid != expected {
                            // A retransmission of a packet whose ACK the device missed: acknowledged and dropped
                            self.report(now, format!("ep {:#04x}: {pid:?} when expecting {expected:?}, dropped", t.ep));
                            (d, None)
                        } else {
                            self.toggles.insert(key, pid.toggled());
                            match t.stage {
                                Stage::StatusIn => (d, Some(Ok(std::mem::take(&mut t.got)))),
                                _ => {
                                    let room = t.want.saturating_sub(t.got.len());
                                    let short = data.len() < t.mps;
                                    t.got.extend_from_slice(&data[..data.len().min(room)]);
                                    if data.len() > room {
                                        self.report(now, format!("ep {:#04x}: babble ({} bytes)", t.ep, data.len()));
                                    }
                                    let complete = short || t.got.len() >= t.want;
                                    if !complete {
                                        (d, None)
                                    } else if t.stage == Stage::DataIn {
                                        t.stage = Stage::StatusOut;
                                        self.toggles.insert(key, Pid::Data1);
                                        (d, None)
                                    } else {
                                        (d, Some(Ok(std::mem::take(&mut t.got))))
                                    }
                                }
                            }
                        }
                    }
                    InReply::Nak => {
                        self.naks += 1;
                        t.blocked = true;
                        if t.kind == Kind::Interrupt {
                            t.next_frame = self.frame_at(now) + t.interval;
                        }
                        (bits(TOKEN_BITS + GAP_BITS + HANDSHAKE_BITS), None)
                    }
                    InReply::Stall => (bits(TOKEN_BITS + GAP_BITS + HANDSHAKE_BITS), Some(Err(UsbError::Stall))),
                    InReply::Timeout => {
                        let d = bits(TOKEN_BITS + TIMEOUT_BITS);
                        self.no_response(&mut t, d)
                    }
                }
            }
            _ => {
                // OUT: control data or status stage, or a bulk/interrupt OUT
                let key = (t.addr, if t.stage == Stage::Data { t.ep } else { 0 });
                let end = if t.stage == Stage::StatusOut { 0 } else { (t.sent + t.mps).min(t.out.len()) };
                let chunk: Vec<u8> = if t.stage == Stage::StatusOut { Vec::new() } else { t.out[t.sent..end].to_vec() };
                let pid = self.toggle(key.0, key.1);
                let hs = port.out(now, t.addr, ep_num, pid, &chunk);
                let d = bits(
                    TOKEN_BITS + GAP_BITS / 4 + DATA_OVERHEAD_BITS + 8 * chunk.len() as u64 + GAP_BITS + HANDSHAKE_BITS,
                );
                match hs {
                    Handshake::Ack => {
                        t.fails = 0;
                        self.toggles.insert(key, pid.toggled());
                        t.sent += chunk.len();
                        t.any_sent = true;
                        let last_full = !chunk.is_empty() && chunk.len() == t.mps;
                        let more = t.sent < t.out.len() || (t.zlp && last_full);
                        match t.stage {
                            Stage::StatusOut => (d, Some(Ok(std::mem::take(&mut t.got)))),
                            Stage::DataOut if !more => {
                                t.stage = Stage::StatusIn;
                                self.toggles.insert((t.addr, 0), Pid::Data1);
                                (d, None)
                            }
                            Stage::Data if !more => (d, Some(Ok(Vec::new()))),
                            _ => (d, None),
                        }
                    }
                    Handshake::Nak => {
                        self.naks += 1;
                        t.blocked = true;
                        (d, None)
                    }
                    Handshake::Stall => (d, Some(Err(UsbError::Stall))),
                    Handshake::Timeout => {
                        let d = d - bits(HANDSHAKE_BITS) + bits(TIMEOUT_BITS);
                        self.no_response(&mut t, d)
                    }
                }
            }
        };
        self.bus_free = now + dur;
        match done {
            Some(r) => {
                if let Err(e) = &r {
                    self.report(now, format!("transfer {} on ep {:#04x} failed: {e}", t.id, t.ep));
                }
                self.queue[i] = t;
                self.finish(i, r);
            }
            None => self.queue[i] = t,
        }
    }

    /// A transaction got no response: retried twice more, then the transfer fails (USB 2.0 8.5: three errors).
    fn no_response(&mut self, t: &mut Transfer, d: Time) -> (Time, Option<Result<Vec<u8>, UsbError>>) {
        t.fails += 1;
        if t.fails >= 3 { (d, Some(Err(UsbError::NoResponse))) } else { (d, None) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An idealised device: answers standard requests at once (no NAKs on endpoint 0), echoes bulk OUT data on
    /// its bulk IN endpoint, and NAKs bulk IN while it has nothing to send.
    #[derive(Default)]
    struct Fake {
        pullup: bool,
        addr: u8,
        pending_addr: Option<u8>,
        config: u8,
        ep0_in: VecDeque<Vec<u8>>,
        ep0_pid: Option<Pid>,
        out_pid: Option<Pid>,
        in_pid: Option<Pid>,
        echo: VecDeque<u8>,
        resets: u32,
        suspended: Vec<(Time, bool)>,
        wake: bool,
        sofs: Vec<(Time, u16)>,
        setups: Vec<SetupPacket>,
        drop_acks: u32,
    }

    const DEVICE: [u8; 18] = [18, 1, 0x10, 1, 0, 0, 0, 64, 0x8A, 0x2E, 0x03, 0, 0, 1, 1, 2, 0, 1];
    const CONFIG: [u8; 32] = [
        9, 2, 32, 0, 1, 1, 0, 0x80, 50, // configuration 1
        9, 4, 0, 0, 2, 0xFF, 0, 0, 0, // vendor interface
        7, 5, 0x01, 2, 64, 0, 0, // EP1 OUT bulk
        7, 5, 0x81, 2, 64, 0, 0, // EP1 IN bulk
    ];

    fn string(s: &str) -> Vec<u8> {
        let mut v = vec![2 + 2 * s.len() as u8, 3];
        for c in s.encode_utf16() {
            v.extend(c.to_le_bytes());
        }
        v
    }

    impl UsbDevicePort for Fake {
        fn pullup(&self) -> bool {
            self.pullup
        }
        fn bus_reset(&mut self, _now: Time, active: bool) {
            if active {
                self.resets += 1;
                self.addr = 0;
                self.config = 0;
            }
        }
        fn sof(&mut self, now: Time, frame: u16) {
            self.sofs.push((now, frame));
        }
        fn suspend(&mut self, now: Time, suspended: bool) {
            self.suspended.push((now, suspended));
            self.wake = false;
        }
        fn remote_wakeup(&self) -> bool {
            self.wake
        }
        fn setup(&mut self, _now: Time, addr: u8, data: &[u8; 8]) -> Handshake {
            if addr != self.addr {
                return Handshake::Timeout;
            }
            let s = SetupPacket::parse(data);
            self.setups.push(s);
            self.ep0_pid = Some(Pid::Data1);
            self.ep0_in.clear();
            let reply: Option<Vec<u8>> = match (s.request, s.value >> 8) {
                (request::GET_DESCRIPTOR, 1) => Some(DEVICE.to_vec()),
                (request::GET_DESCRIPTOR, 2) => Some(CONFIG.to_vec()),
                (request::GET_DESCRIPTOR, 3) => Some(match s.value & 0xFF {
                    0 => vec![4, 3, 9, 4],
                    1 => string("Fake"),
                    _ => string("Echo"),
                }),
                (request::SET_ADDRESS, _) => {
                    self.pending_addr = Some(s.value as u8);
                    None
                }
                (request::CLEAR_FEATURE, _) => {
                    match s.index {
                        0x01 => self.out_pid = Some(Pid::Data0),
                        0x81 => self.in_pid = Some(Pid::Data0),
                        _ => {}
                    }
                    None
                }
                (request::SET_CONFIGURATION, _) => {
                    self.config = s.value as u8;
                    self.out_pid = Some(Pid::Data0);
                    self.in_pid = Some(Pid::Data0);
                    None
                }
                _ => None,
            };
            match reply {
                Some(mut r) => {
                    r.truncate(s.length as usize);
                    let n = r.len();
                    for c in r.chunks(64) {
                        self.ep0_in.push_back(c.to_vec());
                    }
                    if n % 64 == 0 && n < s.length as usize {
                        self.ep0_in.push_back(Vec::new());
                    }
                }
                // Status stage of a no-data request
                None => self.ep0_in.push_back(Vec::new()),
            }
            Handshake::Ack
        }
        fn out(&mut self, _now: Time, addr: u8, ep: u8, pid: Pid, data: &[u8]) -> Handshake {
            if addr != self.addr {
                return Handshake::Timeout;
            }
            match ep {
                0 => Handshake::Ack,
                1 if self.config != 0 => {
                    if Some(pid) == self.out_pid {
                        self.out_pid = Some(pid.toggled());
                        self.echo.extend(data);
                    }
                    Handshake::Ack
                }
                _ => Handshake::Stall,
            }
        }
        fn in_token(&mut self, _now: Time, addr: u8, ep: u8) -> InReply {
            if addr != self.addr {
                return InReply::Timeout;
            }
            match ep {
                0 => match self.ep0_in.pop_front() {
                    Some(d) => {
                        let pid = self.ep0_pid.unwrap();
                        self.ep0_pid = Some(pid.toggled());
                        if d.is_empty()
                            && let Some(a) = self.pending_addr.take()
                        {
                            self.addr = a;
                        }
                        InReply::Data(pid, d)
                    }
                    None => InReply::Nak,
                },
                1 if !self.echo.is_empty() => {
                    let n = self.echo.len().min(64);
                    let d: Vec<u8> = self.echo.iter().take(n).copied().collect();
                    let pid = self.in_pid.unwrap();
                    if self.drop_acks > 0 {
                        // The host's ACK got lost: the device keeps the packet and its toggle, and sends it again
                        self.drop_acks -= 1;
                    } else {
                        self.echo.drain(..n);
                        self.in_pid = Some(pid.toggled());
                    }
                    InReply::Data(pid, d)
                }
                1 => InReply::Nak,
                _ => InReply::Stall,
            }
        }
    }

    /// Polls the host (and kicks it as the SoC would after the device changed) until `until`.
    fn run(h: &mut HostController, d: &mut Fake, from: Time, until: Time) -> Time {
        let mut now = from;
        let mut n = 0;
        while let Some(t) = h.poll(now, d) {
            if t > until {
                break;
            }
            assert!(t >= now, "time went backwards");
            now = t;
            n += 1;
            assert!(n < 1_000_000, "runaway polling");
        }
        until
    }

    fn enumerated() -> (HostController, Fake, Time) {
        let mut h = HostController::new();
        let mut d = Fake { pullup: true, ..Fake::default() };
        assert_eq!(h.kick(Time::ZERO, &mut d), Some(ATTACH_DEBOUNCE));
        let now = run(&mut h, &mut d, Time::ZERO, Time::from_ms(200));
        (h, d, now)
    }

    #[test]
    fn enumerates_with_spec_timing() {
        let (h, d, _) = enumerated();
        assert_eq!(h.enumeration, Enumeration::Configured, "{:?}", h.reports);
        assert_eq!(d.resets, 1);
        assert_eq!((d.addr, d.config), (1, 1));
        let dev = h.configured().unwrap();
        assert_eq!((dev.vendor_id(), dev.product_id()), (0x2E8A, 0x0003));
        assert_eq!(dev.manufacturer.as_deref(), Some("Fake"));
        assert_eq!(dev.product.as_deref(), Some("Echo"));
        assert_eq!(dev.interfaces[0].endpoints.len(), 2);
        let reqs: Vec<(u8, u16)> = d.setups.iter().map(|s| (s.request, s.value)).collect();
        assert_eq!(reqs[0], (request::GET_DESCRIPTOR, 0x100));
        assert_eq!(reqs[1], (request::SET_ADDRESS, 1));
        assert_eq!(*reqs.last().unwrap(), (request::SET_CONFIGURATION, 1));
        // SOFs start when the reset ends (debounce + reset), one per millisecond with consecutive numbers
        let reset_end = ATTACH_DEBOUNCE + RESET_TIME;
        assert_eq!(d.sofs[0], (reset_end, 0));
        assert_eq!(d.sofs[5], (reset_end + Time::from_ms(5), 5));
        // The first request waits out reset recovery
        assert!(h.transactions > 10);
    }

    #[test]
    fn bulk_round_trip_and_throughput() {
        let (mut h, mut d, now) = enumerated();
        let data: Vec<u8> = (0..4096u32).map(|i| (i * 7) as u8).collect();
        let w = h.submit_bulk_out(0x01, data.clone());
        let r = h.submit_bulk_in(0x81, 4096);
        let t0 = now;
        let end = run(&mut h, &mut d, now, now + Time::from_ms(20));
        assert_eq!(h.take_result(w), Some(Ok(Vec::new())));
        assert_eq!(h.take_result(r), Some(Ok(data)));
        let _ = (t0, end);
    }

    #[test]
    fn bulk_rate_is_bounded_by_the_bus() {
        let (mut h, mut d, now) = enumerated();
        // 64 KiB OUT: 1024 packets of 64 bytes at no more than ~19 per frame needs at least 53 frames
        let w = h.submit_bulk_out(0x01, vec![0x55; 65536]);
        run(&mut h, &mut d, now, now + Time::from_ms(52));
        assert_eq!(h.take_result(w), None, "too fast for full speed");
        run(&mut h, &mut d, now + Time::from_ms(52), now + Time::from_ms(70));
        assert_eq!(h.take_result(w), Some(Ok(Vec::new())));
    }

    #[test]
    fn retransmitted_in_packets_are_dropped() {
        let (mut h, mut d, now) = enumerated();
        d.drop_acks = 1;
        let w = h.submit_bulk_out(0x01, b"hello".to_vec());
        let r = h.submit_bulk_in(0x81, 64);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(5));
        assert_eq!(h.take_result(w), Some(Ok(Vec::new())));
        // The first copy completes the read; the device's retransmission (same toggle) is dropped by the next one
        assert_eq!(h.take_result(r), Some(Ok(b"hello".to_vec())));
        let r2 = h.submit_bulk_in(0x81, 64);
        run(&mut h, &mut d, now, now + Time::from_ms(5));
        assert_eq!(h.take_result(r2), None, "nothing new to read");
        assert!(h.reports.iter().any(|(_, m)| m.contains("dropped")), "{:?}", h.reports);
    }

    #[test]
    fn nak_waits_for_a_kick_and_deadline_ends_it() {
        let (mut h, mut d, now) = enumerated();
        let r = h.submit_bulk_in(0x81, 64);
        h.set_deadline(r, now + Time::from_ms(50));
        let naks = h.naks;
        let now = run(&mut h, &mut d, now, now + Time::from_ms(10));
        // One NAK, then nothing until something changes (only SOFs)
        assert_eq!(h.naks, naks + 1);
        d.echo.extend(b"x");
        let next = h.kick(now, &mut d).unwrap();
        assert!(next <= now + Time::from_us(5), "retried at once (after the SOF), not {next}");
        run(&mut h, &mut d, now, now + Time::from_ms(1));
        assert_eq!(h.take_result(r), Some(Ok(b"x".to_vec())));
        let r = h.submit_bulk_in(0x81, 64);
        h.set_deadline(r, now + Time::from_ms(20));
        run(&mut h, &mut d, now, now + Time::from_ms(30));
        assert_eq!(h.take_result(r), Some(Err(UsbError::TimedOut)));
    }

    #[test]
    fn standard_requests_reset_toggles() {
        let (mut h, mut d, now) = enumerated();
        let w = h.submit_bulk_out(0x01, b"abc".to_vec());
        let now = run(&mut h, &mut d, now, now + Time::from_ms(2));
        assert_eq!(h.take_result(w), Some(Ok(Vec::new())));
        assert_eq!(h.toggle(1, 0x01), Pid::Data1);
        // CLEAR_FEATURE(ENDPOINT_HALT) resets that endpoint's toggle once it succeeds
        let c = h.clear_halt(0x01);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(2));
        assert_eq!(h.take_result(c), Some(Ok(Vec::new())));
        assert_eq!(h.toggle(1, 0x01), Pid::Data0);
        // So does SET_CONFIGURATION for every endpoint (the device resets its own: the next OUT is DATA0 again)
        let w = h.submit_bulk_out(0x01, b"d".to_vec());
        let now = run(&mut h, &mut d, now, now + Time::from_ms(2));
        h.take_result(w);
        let c = h.submit_control(SetupPacket::new(0, request::SET_CONFIGURATION, 1, 0, 0), Vec::new());
        let now = run(&mut h, &mut d, now, now + Time::from_ms(2));
        assert_eq!(h.take_result(c), Some(Ok(Vec::new())));
        assert_eq!(h.toggle(1, 0x01), Pid::Data0);
        let w = h.submit_bulk_out(0x01, b"e".to_vec());
        run(&mut h, &mut d, now, now + Time::from_ms(2));
        h.take_result(w);
        assert_eq!(d.echo.iter().copied().collect::<Vec<u8>>(), b"abcde", "nothing dropped as a repeat");
    }

    #[test]
    fn suspend_resume_and_remote_wakeup() {
        let (mut h, mut d, now) = enumerated();
        h.suspend(now);
        let sofs = d.sofs.len();
        let r = h.submit_bulk_in(0x81, 64);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(10));
        assert_eq!(d.sofs.len(), sofs, "no SOFs while suspended");
        assert_eq!(d.suspended, vec![(now - Time::from_ms(10) + SUSPEND_IDLE, true)]);
        assert!(h.pending(r), "transfers wait");
        h.resume(now);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(25));
        assert_eq!(d.suspended[1], (now - Time::from_ms(25), false));
        assert!(d.sofs.len() > sofs, "SOFs again after the 20 ms of resume");
        // Remote wakeup: the device signals while suspended; the host notices at the kick and resumes
        h.suspend(now);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(5));
        d.wake = true;
        h.kick(now, &mut d);
        assert!(matches!(h.state, PortState::Resuming(_)), "{:?}", h.state);
        run(&mut h, &mut d, now, now + Time::from_ms(25));
        assert_eq!(h.state, PortState::Enabled);
    }

    #[test]
    fn stall_and_detach() {
        let (mut h, mut d, now) = enumerated();
        let r = h.submit_bulk_in(0x82, 64);
        let now = run(&mut h, &mut d, now, now + Time::from_ms(2));
        assert_eq!(h.take_result(r), Some(Err(UsbError::Stall)));
        let r = h.submit_bulk_in(0x81, 64);
        d.pullup = false;
        h.kick(now, &mut d);
        assert_eq!(h.state, PortState::Detached);
        assert_eq!(h.take_result(r), Some(Err(UsbError::Disconnected)));
        assert!(h.configured().is_none());
        // Re-attaching enumerates again, at a new address
        d.pullup = true;
        h.kick(now, &mut d);
        run(&mut h, &mut d, now, now + Time::from_ms(200));
        assert_eq!(h.enumerations, 2);
        assert_eq!(d.addr, 2);
    }
}
