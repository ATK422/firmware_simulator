//! Bridges an in-process CAN bus to a Linux SocketCAN interface (`vcan0`, `can0`), so `titan_bridge`,
//! `canmore_cli`, `upload_tool` and `candump` on the host see the emulated boards.
//!
//! Frames from the interface are injected into the bus when the bus next steps; frames on the bus are written to
//! the interface as they complete. The socket is opened with CAN FD frames enabled. Use real-time mode
//! (`--realtime`) with host tools, which have their own timeouts.

use std::io;
use std::mem::size_of;

use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CANFD_BRS: u8 = 0x01;
const CANFD_ESI: u8 = 0x02;
const CANFD_FDF: u8 = 0x04;

/// `struct canfd_frame` (also used for classic frames, which are 16 bytes on the wire).
#[repr(C)]
#[derive(Clone, Copy)]
struct RawFrame {
    can_id: u32,
    len: u8,
    flags: u8,
    res0: u8,
    res1: u8,
    data: [u8; 64],
}

const CAN_MTU: usize = 16;
const CANFD_MTU: usize = 72;

/// Converts a frame to the kernel layout; returns the bytes to write (16 for classic, 72 for FD).
fn to_raw(frame: &CanFrame) -> (RawFrame, usize) {
    let mut id = if frame.extended { frame.id & 0x1FFF_FFFF | CAN_EFF_FLAG } else { frame.id & 0x7FF };
    if frame.rtr {
        id |= CAN_RTR_FLAG;
    }
    let mut raw = RawFrame { can_id: id, len: frame.data.len() as u8, flags: 0, res0: 0, res1: 0, data: [0; 64] };
    raw.data[..frame.data.len()].copy_from_slice(&frame.data);
    if frame.fd {
        raw.flags = CANFD_FDF | if frame.brs { CANFD_BRS } else { 0 } | if frame.esi { CANFD_ESI } else { 0 };
        (raw, CANFD_MTU)
    } else {
        (raw, CAN_MTU)
    }
}

/// Converts a frame read from the kernel; `n` is the number of bytes read. Error frames are dropped.
fn from_raw(raw: &RawFrame, n: usize) -> Option<CanFrame> {
    if raw.can_id & CAN_ERR_FLAG != 0 {
        return None;
    }
    let extended = raw.can_id & CAN_EFF_FLAG != 0;
    let id = if extended { raw.can_id & 0x1FFF_FFFF } else { raw.can_id & 0x7FF };
    let fd = n == CANFD_MTU;
    let len = (raw.len as usize).min(if fd { 64 } else { 8 });
    Some(CanFrame {
        id,
        extended,
        fd,
        brs: fd && raw.flags & CANFD_BRS != 0,
        rtr: !fd && raw.can_id & CAN_RTR_FLAG != 0,
        esi: fd && raw.flags & CANFD_ESI != 0,
        data: raw.data[..len].to_vec(),
    })
}

pub struct SocketCan {
    fd: i32,
    pub interface: String,
    /// Frames written and read, for statistics.
    pub tx_count: u64,
    pub rx_count: u64,
}

impl SocketCan {
    /// SocketCAN is Linux only: elsewhere (macOS) opening an interface fails with [`io::ErrorKind::Unsupported`].
    /// Use the in-process bus, mock bridges or a cannelloni tunnel (`can-tunnel`) to a Linux machine instead.
    #[cfg(not(target_os = "linux"))]
    pub fn open(interface: &str) -> io::Result<SocketCan> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!("{interface}: SocketCAN needs Linux (on macOS, tunnel CAN to a Linux machine with can-tunnel)"),
        ))
    }

    /// Opens a raw CAN socket on `interface` with CAN FD frames enabled.
    #[cfg(target_os = "linux")]
    pub fn open(interface: &str) -> io::Result<SocketCan> {
        let name = std::ffi::CString::new(interface).map_err(|_| io::Error::other("bad interface name"))?;
        // SAFETY: plain socket API calls with checked return values and correctly sized structures
        unsafe {
            let index = libc::if_nametoindex(name.as_ptr());
            if index == 0 {
                return Err(io::Error::other(format!(
                    "no CAN interface {interface} (try: ip link add {interface} type vcan && ip link set {interface} up)"
                )));
            }
            let fd = libc::socket(libc::PF_CAN, libc::SOCK_RAW | libc::SOCK_NONBLOCK, libc::CAN_RAW);
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            let on: libc::c_int = 1;
            if libc::setsockopt(
                fd,
                libc::SOL_CAN_RAW,
                libc::CAN_RAW_FD_FRAMES,
                (&on as *const libc::c_int).cast(),
                size_of::<libc::c_int>() as u32,
            ) != 0
            {
                let e = io::Error::last_os_error();
                libc::close(fd);
                return Err(e);
            }
            let mut addr: libc::sockaddr_can = std::mem::zeroed();
            addr.can_family = libc::AF_CAN as u16;
            addr.can_ifindex = index as i32;
            if libc::bind(fd, (&addr as *const libc::sockaddr_can).cast(), size_of::<libc::sockaddr_can>() as u32) != 0
            {
                let e = io::Error::last_os_error();
                libc::close(fd);
                return Err(e);
            }
            Ok(SocketCan { fd, interface: interface.to_string(), tx_count: 0, rx_count: 0 })
        }
    }

    pub fn write(&mut self, frame: &CanFrame) -> io::Result<()> {
        let (raw, n) = to_raw(frame);
        // SAFETY: writing `n` bytes of a repr(C) struct that is at least `n` bytes long
        let r = unsafe { libc::write(self.fd, (&raw as *const RawFrame).cast(), n) };
        if r < 0 {
            return Err(io::Error::last_os_error());
        }
        self.tx_count += 1;
        Ok(())
    }

    /// Reads every frame waiting on the socket.
    pub fn read_all(&mut self) -> Vec<CanFrame> {
        let mut out = Vec::new();
        loop {
            let mut raw = RawFrame { can_id: 0, len: 0, flags: 0, res0: 0, res1: 0, data: [0; 64] };
            // SAFETY: reading at most size_of::<RawFrame>() bytes into the struct
            let n = unsafe { libc::read(self.fd, (&mut raw as *mut RawFrame).cast(), size_of::<RawFrame>()) };
            if n <= 0 {
                break;
            }
            self.rx_count += 1;
            if let Some(f) = from_raw(&raw, n as usize) {
                out.push(f);
            }
        }
        out
    }
}

impl Drop for SocketCan {
    fn drop(&mut self) {
        // SAFETY: closing our own socket
        unsafe {
            libc::close(self.fd);
        }
    }
}

impl CanEndpoint for SocketCan {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.read_all().into_iter().map(|f| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        if let Err(e) = self.write(frame) {
            log::warn!("{}: write failed: {e}", self.interface);
        }
    }

    /// What host tools write waits for the bus in a bounded queue, as a real interface's `txqueuelen` bounds it
    /// (vcan itself never refuses a write), so a stuck or flooded bus does not collect it without end.
    fn tx_queue_limit(&self) -> Option<usize> {
        Some(sim_devices::can::BRIDGE_TX_QUEUE)
    }

    /// What host tools wrote is handed in at the step that reads it.
    fn live(&self) -> bool {
        true
    }
}

/// Stands in for a SocketCAN interface where there is none (tests, benchmarks, recorded runs): hands in what
/// `inner` sends as a live endpoint does ([`CanEndpoint::live`]), at the bus step that reads it and stamped with
/// that step's time, and says nothing about when it will send. With a scripted `inner` (a mock bridge, a replay of
/// when frames arrived) the arrivals are the same in every run, so a robot run slice by slice and one run ahead
/// can be compared.
pub struct Live<E>(pub E);

impl<E: CanEndpoint> CanEndpoint for Live<E> {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.0.take_tx(until).into_iter().map(|(_, f)| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        self.0.receive(frame, at);
    }

    fn tx_queue_limit(&self) -> Option<usize> {
        Some(sim_devices::can::BRIDGE_TX_QUEUE)
    }

    fn live(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_frame_conversion() {
        let fd = CanFrame::fd(0x1420802, true, &[1, 2, 3]);
        let (raw, n) = to_raw(&fd);
        assert_eq!(n, CANFD_MTU);
        assert_eq!(raw.can_id, 0x1420802 | CAN_EFF_FLAG);
        assert_eq!(raw.flags, CANFD_FDF | CANFD_BRS);
        assert_eq!(from_raw(&raw, n).unwrap(), fd);
        let classic = CanFrame::classic(0x6F, false, &[0xC0]);
        let (raw, n) = to_raw(&classic);
        assert_eq!((n, raw.len), (CAN_MTU, 1));
        assert_eq!(from_raw(&raw, n).unwrap(), classic);
        let mut err = raw;
        err.can_id |= CAN_ERR_FLAG;
        assert!(from_raw(&err, CAN_MTU).is_none());
        assert_eq!(size_of::<RawFrame>(), CANFD_MTU);
    }

    #[test]
    fn missing_interface_is_an_error() {
        assert!(SocketCan::open("nosuchcan0").is_err());
    }

    /// A stand-in for an interface hands in what its inner endpoint sends at the step that reads it, and is live.
    #[test]
    fn live_stand_in_stamps_frames_with_the_step() {
        let f = CanFrame::classic(0x6F, false, &[0xC0]);
        let inner = sim_devices::can::Monitor { outbox: vec![(Time::from_us(30), f.clone())], ..Default::default() };
        assert!(!inner.live() && inner.idle_until() == Some(Time::from_us(30)));
        let mut live = Live(inner);
        assert!(live.live() && live.idle_until().is_none());
        assert!(live.take_tx(Time::from_us(20)).is_empty());
        assert_eq!(live.take_tx(Time::from_us(100)), [(Time::from_us(100), f.clone())]);
        live.receive(&f, Time::from_us(200));
        assert_eq!(live.0.received, [(Time::from_us(200), f)]);
    }
}
