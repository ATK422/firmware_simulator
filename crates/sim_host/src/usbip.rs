//! A USB/IP server for the emulated board's USB device, so the real host's tools can use it: `usbip attach`
//! makes it a local USB device (the kernel's vhci-hcd), and `picotool`, `screen /dev/ttyACM0` or a mounted
//! RPI-RP2 drive then talk to the emulated firmware.
//!
//! The protocol (Linux `Documentation/usb/usbip_protocol.rst`): over TCP, OP_REQ_DEVLIST / OP_REQ_IMPORT pick the
//! device, then each USB request block arrives as USBIP_CMD_SUBMIT and is answered with USBIP_RET_SUBMIT;
//! USBIP_CMD_UNLINK cancels one. Every field is big-endian. Requests are turned into transfers on the emulator's
//! host controller model, which has already enumerated the device (vhci-hcd handles SET_ADDRESS and port resets
//! itself and never forwards them). The device's standard requests (descriptors, SET_CONFIGURATION, ...) go to the
//! firmware like any other. When the device drops off the bus (a reboot), the connection is closed, as a real
//! server does when its device goes away; attach again once it is back.
//!
//! Real host tools have timeouts: run with `--realtime` so virtual time keeps pace with them.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};

use rp2040::Machine;
use sim_devices::usb::{DeviceInfo, EndpointType, SetupPacket, UsbError, request};

pub const USBIP_VERSION: u16 = 0x0111;
pub const OP_REQ_DEVLIST: u16 = 0x8005;
pub const OP_REP_DEVLIST: u16 = 0x0005;
pub const OP_REQ_IMPORT: u16 = 0x8003;
pub const OP_REP_IMPORT: u16 = 0x0003;
pub const CMD_SUBMIT: u32 = 1;
pub const CMD_UNLINK: u32 = 2;
pub const RET_SUBMIT: u32 = 3;
pub const RET_UNLINK: u32 = 4;
/// URB transfer flag: end an OUT transfer that fills its last packet with a zero length packet.
pub const URB_ZERO_PACKET: u32 = 0x40;
/// The bus ID the device is exported as.
pub const BUSID: &str = "1-1";

const EPIPE: i32 = -32;
const EPROTO: i32 = -71;
const ECONNRESET: i32 = -104;
const ESHUTDOWN: i32 = -108;
const EINVAL: i32 = -22;
/// The longest transfer accepted: Linux's usbfs limit (`usbfs_memory_mb`, 16 MiB by default), far beyond anything
/// the emulated device needs. A longer one is refused with -EINVAL instead of waiting for (and buffering) up to
/// 4 GiB of OUT data.
pub const MAX_TRANSFER: usize = 16 << 20;
/// Isochronous packet descriptors accepted in a request (the controller has no isochronous endpoints anyway).
const MAX_ISO_PACKETS: usize = 1024;

fn errno(e: &UsbError) -> i32 {
    match e {
        UsbError::Stall => EPIPE,
        UsbError::NoResponse => EPROTO,
        UsbError::TimedOut => ECONNRESET,
        UsbError::Disconnected => ESHUTDOWN,
    }
}

struct Pending {
    seqnum: u32,
    id: u32,
    dir_in: bool,
    /// Bytes an OUT transfer carries (reported back as its actual length).
    out_len: u32,
}

struct Conn {
    stream: TcpStream,
    rx: Vec<u8>,
    tx: Vec<u8>,
    imported: bool,
    pending: Vec<Pending>,
    /// Bytes still to skip: the data of a refused request.
    skip: usize,
}

/// Serves the board's USB device over USB/IP.
pub struct UsbIpServer {
    listener: TcpListener,
    conn: Option<Conn>,
}

/// The `usb_device` record of OP_REP_DEVLIST / OP_REP_IMPORT (312 bytes).
pub fn device_record(d: &DeviceInfo) -> Vec<u8> {
    let mut v = Vec::with_capacity(312);
    let mut path = b"/sys/devices/platform/firmware_simulator/usb1/1-1".to_vec();
    path.resize(256, 0);
    v.extend(path);
    let mut busid = BUSID.as_bytes().to_vec();
    busid.resize(32, 0);
    v.extend(busid);
    v.extend(1u32.to_be_bytes());
    v.extend((d.address as u32).to_be_bytes());
    // USB_SPEED_FULL
    v.extend(2u32.to_be_bytes());
    v.extend(d.vendor_id().to_be_bytes());
    v.extend(d.product_id().to_be_bytes());
    let dd = |i: usize| d.device_descriptor.get(i).copied().unwrap_or(0);
    v.extend(u16::from_le_bytes([dd(12), dd(13)]).to_be_bytes());
    v.extend([dd(4), dd(5), dd(6)]);
    v.push(d.config_descriptor.get(5).copied().unwrap_or(1));
    v.push(dd(17));
    v.push(d.config_descriptor.get(4).copied().unwrap_or(0));
    v
}

fn op_header(code: u16, status: u32) -> Vec<u8> {
    let mut v = USBIP_VERSION.to_be_bytes().to_vec();
    v.extend(code.to_be_bytes());
    v.extend(status.to_be_bytes());
    v
}

fn be32(b: &[u8], i: usize) -> u32 {
    u32::from_be_bytes(b[i..i + 4].try_into().unwrap())
}

impl UsbIpServer {
    /// Listens on `addr` (usbip's port is 3240).
    pub fn bind(addr: impl ToSocketAddrs) -> std::io::Result<UsbIpServer> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        Ok(UsbIpServer { listener, conn: None })
    }

    pub fn local_addr(&self) -> std::io::Result<std::net::SocketAddr> {
        self.listener.local_addr()
    }

    /// Whether a client has imported the device.
    pub fn attached(&self) -> bool {
        self.conn.as_ref().is_some_and(|c| c.imported)
    }

    /// Accepts a client, handles what it sent, passes finished transfers back. Call between machine slices.
    pub fn poll(&mut self, machine: &mut Machine) {
        if self.conn.is_none()
            && let Ok((stream, peer)) = self.listener.accept()
            && stream.set_nonblocking(true).is_ok()
        {
            let _ = stream.set_nodelay(true);
            log::info!("USB/IP client {peer} connected");
            self.conn =
                Some(Conn { stream, rx: Vec::new(), tx: Vec::new(), imported: false, pending: Vec::new(), skip: 0 });
        }
        let Some(conn) = &mut self.conn else { return };
        let mut buf = [0u8; 16384];
        let mut closed = false;
        loop {
            match conn.stream.read(&mut buf) {
                Ok(0) => {
                    closed = true;
                    break;
                }
                Ok(n) => conn.rx.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(_) => {
                    closed = true;
                    break;
                }
            }
        }
        let mut drop_conn = closed;
        while let Some(used) = Self::handle(conn, machine) {
            conn.rx.drain(..used);
            if used == 0 {
                drop_conn = true;
                break;
            }
        }
        // Finished transfers
        if let Some(host) = machine.sys.usb_host_mut() {
            let mut i = 0;
            while i < conn.pending.len() {
                match host.take_result(conn.pending[i].id) {
                    Some(r) => {
                        let p = conn.pending.remove(i);
                        if r == Err(UsbError::Disconnected) {
                            drop_conn = true;
                        }
                        conn.tx.extend(ret_submit(&p, r));
                    }
                    None => i += 1,
                }
            }
            if conn.imported && host.configured().is_none() {
                // The device went away (rebooted, or was reset): end the session as usbipd does
                drop_conn = true;
            }
        }
        while !conn.tx.is_empty() {
            match conn.stream.write(&conn.tx) {
                Ok(0) => {
                    drop_conn = true;
                    break;
                }
                Ok(n) => {
                    conn.tx.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(_) => {
                    drop_conn = true;
                    break;
                }
            }
        }
        if drop_conn {
            let _ = conn.stream.flush();
            if let Some(host) = machine.sys.usb_host_mut() {
                for p in &conn.pending {
                    host.cancel(p.id);
                    host.take_result(p.id);
                }
            }
            log::info!("USB/IP client disconnected");
            self.conn = None;
        }
    }

    /// Handles one complete message at the start of `conn.rx`. Returns the bytes it used, `Some(0)` to close the
    /// connection, `None` when the message is not complete yet.
    fn handle(conn: &mut Conn, machine: &mut Machine) -> Option<usize> {
        if conn.skip > 0 {
            let n = conn.skip.min(conn.rx.len());
            conn.skip -= n;
            return (n > 0).then_some(n);
        }
        let b = &conn.rx;
        if !conn.imported {
            if b.len() < 8 {
                return None;
            }
            let code = u16::from_be_bytes([b[2], b[3]]);
            let dev = machine.sys.usb_host_mut().and_then(|h| h.configured().cloned());
            return match code {
                OP_REQ_DEVLIST => {
                    let mut r = op_header(OP_REP_DEVLIST, 0);
                    match &dev {
                        Some(d) => {
                            r.extend(1u32.to_be_bytes());
                            r.extend(device_record(d));
                            for i in d.interfaces.iter().filter(|i| i.alternate == 0) {
                                r.extend([i.class, i.subclass, i.protocol, 0]);
                            }
                        }
                        None => r.extend(0u32.to_be_bytes()),
                    }
                    conn.tx.extend(r);
                    Some(8)
                }
                OP_REQ_IMPORT => {
                    if b.len() < 40 {
                        return None;
                    }
                    let busid = String::from_utf8_lossy(&b[8..40]).trim_end_matches('\0').to_string();
                    match &dev {
                        Some(d) if busid == BUSID => {
                            conn.tx.extend(op_header(OP_REP_IMPORT, 0));
                            conn.tx.extend(device_record(d));
                            conn.imported = true;
                            log::info!("USB/IP: {BUSID} ({:04x}:{:04x}) imported", d.vendor_id(), d.product_id());
                        }
                        _ => conn.tx.extend(op_header(OP_REP_IMPORT, 1)),
                    }
                    Some(40)
                }
                _ => {
                    log::warn!("USB/IP: unknown operation {code:#06x}");
                    Some(0)
                }
            };
        }
        if b.len() < 48 {
            return None;
        }
        let command = be32(b, 0);
        let seqnum = be32(b, 4);
        let dir_in = be32(b, 12) == 1;
        let ep = be32(b, 16);
        match command {
            CMD_SUBMIT => {
                let flags = be32(b, 20);
                let len = be32(b, 24) as usize;
                let packets = be32(b, 32) as i32;
                let out_len = if dir_in { 0 } else { len };
                let iso_len = if packets > 0 && packets != -1 { 16 * packets as usize } else { 0 };
                if ep > 15 || len > MAX_TRANSFER || iso_len > 16 * MAX_ISO_PACKETS {
                    // Refused without waiting for its data, which is skipped as it arrives
                    log::warn!("USB/IP: refusing a request for endpoint {ep} of {len} bytes");
                    let p = Pending { seqnum, id: 0, dir_in, out_len: 0 };
                    conn.tx.extend(ret_submit_status(&p, EINVAL, Vec::new()));
                    conn.skip = out_len + iso_len;
                    return Some(48);
                }
                if b.len() < 48 + out_len + iso_len {
                    return None;
                }
                let setup: [u8; 8] = b[40..48].try_into().unwrap();
                let data = b[48..48 + out_len].to_vec();
                let used = 48 + out_len + iso_len;
                let p = Pending { seqnum, id: 0, dir_in, out_len: out_len as u32 };
                if iso_len > 0 {
                    // No isochronous endpoints on the emulated controller
                    conn.tx.extend(ret_submit_status(&p, EINVAL, Vec::new()));
                    return Some(used);
                }
                let now = machine.now;
                let id = machine.sys.with_usb_host(now, |h| {
                    if ep == 0 {
                        let s = SetupPacket::parse(&setup);
                        if s.request == request::SET_ADDRESS && s.request_type == 0 {
                            // vhci-hcd never sends it; answer it without disturbing the device
                            return None;
                        }
                        let s = if s.is_in() { SetupPacket { length: s.length.min(len as u16), ..s } } else { s };
                        return Some(h.submit_control(s, data));
                    }
                    let addr = ep as u8 | if dir_in { 0x80 } else { 0 };
                    let interrupt = h
                        .configured()
                        .and_then(|d| d.endpoint(addr))
                        .is_some_and(|e| e.kind == EndpointType::Interrupt);
                    Some(match (dir_in, interrupt) {
                        (true, true) => h.submit_interrupt_in(addr, len),
                        (true, false) => h.submit_bulk_in(addr, len),
                        (false, _) if flags & URB_ZERO_PACKET != 0 => h.submit_bulk_out_zlp(addr, data),
                        (false, _) => h.submit_bulk_out(addr, data),
                    })
                })?;
                match id {
                    Some(id) => conn.pending.push(Pending { id, ..p }),
                    None => conn.tx.extend(ret_submit(&p, Ok(Vec::new()))),
                }
                Some(used)
            }
            CMD_UNLINK => {
                let victim = be32(b, 20);
                let status = match conn.pending.iter().position(|p| p.seqnum == victim) {
                    Some(i) => {
                        let p = conn.pending.remove(i);
                        match machine.sys.usb_host_mut().map(|h| (h.take_result(p.id), h)) {
                            // It finished before the unlink came (its result not collected yet): as usbip-host,
                            // send its RET_SUBMIT, and unlink nothing
                            Some((Some(r), _)) => {
                                conn.tx.extend(ret_submit(&p, r));
                                0
                            }
                            Some((None, h)) => {
                                h.cancel(p.id);
                                h.take_result(p.id);
                                ECONNRESET
                            }
                            None => ECONNRESET,
                        }
                    }
                    // Already completed (its RET_SUBMIT is on its way)
                    None => 0,
                };
                let mut r = Vec::with_capacity(48);
                for w in [RET_UNLINK, seqnum, 0, 0, 0] {
                    r.extend(w.to_be_bytes());
                }
                r.extend(status.to_be_bytes());
                r.resize(48, 0);
                conn.tx.extend(r);
                Some(48)
            }
            _ => {
                log::warn!("USB/IP: unknown command {command:#x}");
                Some(0)
            }
        }
    }
}

/// USBIP_RET_SUBMIT for a finished transfer (with the IN data).
fn ret_submit(p: &Pending, r: Result<Vec<u8>, UsbError>) -> Vec<u8> {
    match r {
        Ok(d) => ret_submit_status(p, 0, d),
        Err(e) => ret_submit_status(p, errno(&e), Vec::new()),
    }
}

fn ret_submit_status(p: &Pending, status: i32, data: Vec<u8>) -> Vec<u8> {
    let actual = match (p.dir_in, status) {
        (true, _) => data.len() as u32,
        (false, 0) => p.out_len,
        (false, _) => 0,
    };
    let mut v = Vec::with_capacity(48 + data.len());
    for w in [RET_SUBMIT, p.seqnum, 0, 0, 0] {
        v.extend(w.to_be_bytes());
    }
    v.extend(status.to_be_bytes());
    v.extend(actual.to_be_bytes());
    // start_frame, number_of_packets (-1: not isochronous), error_count, setup padding
    v.extend(0u32.to_be_bytes());
    v.extend((-1i32).to_be_bytes());
    v.extend(0u32.to_be_bytes());
    v.extend([0u8; 8]);
    if p.dir_in {
        v.extend(data);
    }
    v
}
