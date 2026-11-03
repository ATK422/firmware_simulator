//! USB full speed, seen from the cable: the packet-level interface between a device controller and a host, the
//! standard requests and descriptors, and a host controller model ([`host::HostController`]) that enumerates a
//! device and runs control, bulk and interrupt transfers on the virtual clock.
//!
//! The wire itself is not simulated bit by bit. A *transaction* (token, data, handshake; USB 2.0 section 8.5) is
//! exchanged atomically through [`UsbDevicePort`] at the time the host starts it, and the host then keeps the bus
//! busy for as long as the packets take at 12 Mbit/s. That is enough for firmware to see what it sees on hardware:
//! SETUP packets arriving, buffers being filled and emptied with the right data toggles, NAKs while it has not
//! armed a buffer, STALLs, bus resets and start-of-frame packets every millisecond.

pub mod host;

use sim_core::time::Time;

/// Full speed bit time: 12 Mbit/s.
pub const BIT_TIME_PS: u64 = 1_000_000 / 12;
/// A full speed frame.
pub const FRAME: Time = Time::from_ms(1);

/// Time `bits` bit times take on a full speed bus.
pub fn bits(bits: u64) -> Time {
    Time::from_ps(bits * 1_000_000 / 12)
}

/// Data packet identifier (the data toggle).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pid {
    Data0,
    Data1,
}

impl Pid {
    pub fn toggled(self) -> Pid {
        match self {
            Pid::Data0 => Pid::Data1,
            Pid::Data1 => Pid::Data0,
        }
    }
}

/// The device's answer to a SETUP or OUT transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Handshake {
    Ack,
    Nak,
    Stall,
    /// No handshake at all: wrong address, endpoint not enabled, controller off, or a packet the device could not
    /// take. The host times out after 16-18 bit times (USB 2.0 7.1.19.1).
    Timeout,
}

/// The device's answer to an IN token.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InReply {
    /// A data packet; the host acknowledges it (it ACKs even a packet with the wrong toggle, which it then drops as
    /// a retransmission, USB 2.0 8.6.4).
    Data(Pid, Vec<u8>),
    Nak,
    Stall,
    Timeout,
}

/// What the host sees of a device on its port: the pull-up that announces it, and the transactions it answers.
pub trait UsbDevicePort {
    /// Whether the device presents its D+ pull-up (a full speed device is attached).
    fn pullup(&self) -> bool;
    /// The host starts (`true`) or ends (`false`) a bus reset (SE0 on the lines, USB 2.0 7.1.7.5).
    fn bus_reset(&mut self, now: Time, active: bool);
    /// A start-of-frame packet with the 11-bit frame number.
    fn sof(&mut self, now: Time, frame: u16);
    /// A SETUP transaction to endpoint 0 of `addr`: the 8 byte request in a DATA0 packet.
    fn setup(&mut self, now: Time, addr: u8, data: &[u8; 8]) -> Handshake;
    /// An OUT transaction: `data` in a packet with toggle `pid` to endpoint `ep` (number, 0-15) of `addr`.
    fn out(&mut self, now: Time, addr: u8, ep: u8, pid: Pid, data: &[u8]) -> Handshake;
    /// An IN token to endpoint `ep` (number) of `addr`.
    fn in_token(&mut self, now: Time, addr: u8, ep: u8) -> InReply;
    /// The bus has been idle for 3 ms and the device must suspend (`true`, USB 2.0 7.1.7.6), or the host
    /// signals resume (`false`, K state, 7.1.7.7).
    fn suspend(&mut self, now: Time, suspended: bool) {
        let _ = (now, suspended);
    }
    /// Whether the suspended device signals a remote wakeup.
    fn remote_wakeup(&self) -> bool {
        false
    }
}

/// Request type bits (bmRequestType, USB 2.0 9.3.1).
pub mod req_type {
    pub const DIR_IN: u8 = 0x80;
    pub const STANDARD: u8 = 0x00;
    pub const CLASS: u8 = 0x20;
    pub const VENDOR: u8 = 0x40;
    pub const DEVICE: u8 = 0x00;
    pub const INTERFACE: u8 = 0x01;
    pub const ENDPOINT: u8 = 0x02;
}

/// Standard request codes (USB 2.0 table 9-4).
pub mod request {
    pub const GET_STATUS: u8 = 0;
    pub const CLEAR_FEATURE: u8 = 1;
    pub const SET_FEATURE: u8 = 3;
    pub const SET_ADDRESS: u8 = 5;
    pub const GET_DESCRIPTOR: u8 = 6;
    pub const GET_CONFIGURATION: u8 = 8;
    pub const SET_CONFIGURATION: u8 = 9;
    pub const SET_INTERFACE: u8 = 11;
}

/// Descriptor types (USB 2.0 table 9-5).
pub mod desc_type {
    pub const DEVICE: u8 = 1;
    pub const CONFIGURATION: u8 = 2;
    pub const STRING: u8 = 3;
    pub const INTERFACE: u8 = 4;
    pub const ENDPOINT: u8 = 5;
    pub const INTERFACE_ASSOCIATION: u8 = 11;
}

/// An 8 byte control request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SetupPacket {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl SetupPacket {
    pub fn new(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> SetupPacket {
        SetupPacket { request_type, request, value, index, length }
    }

    pub fn get_descriptor(kind: u8, index: u8, lang: u16, length: u16) -> SetupPacket {
        SetupPacket::new(req_type::DIR_IN, request::GET_DESCRIPTOR, (kind as u16) << 8 | index as u16, lang, length)
    }

    pub fn is_in(&self) -> bool {
        self.request_type & req_type::DIR_IN != 0
    }

    pub fn bytes(&self) -> [u8; 8] {
        let [v0, v1] = self.value.to_le_bytes();
        let [i0, i1] = self.index.to_le_bytes();
        let [l0, l1] = self.length.to_le_bytes();
        [self.request_type, self.request, v0, v1, i0, i1, l0, l1]
    }

    pub fn parse(b: &[u8; 8]) -> SetupPacket {
        SetupPacket {
            request_type: b[0],
            request: b[1],
            value: u16::from_le_bytes([b[2], b[3]]),
            index: u16::from_le_bytes([b[4], b[5]]),
            length: u16::from_le_bytes([b[6], b[7]]),
        }
    }
}

/// Transfer types (endpoint bmAttributes bits 1:0).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EndpointType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

impl EndpointType {
    pub fn from_attributes(a: u8) -> EndpointType {
        match a & 3 {
            0 => EndpointType::Control,
            1 => EndpointType::Isochronous,
            2 => EndpointType::Bulk,
            _ => EndpointType::Interrupt,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EndpointInfo {
    /// bEndpointAddress: number, bit 7 set for IN.
    pub address: u8,
    pub kind: EndpointType,
    pub max_packet: u16,
    pub interval: u8,
}

impl EndpointInfo {
    pub fn is_in(&self) -> bool {
        self.address & 0x80 != 0
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InterfaceInfo {
    pub number: u8,
    pub alternate: u8,
    pub class: u8,
    pub subclass: u8,
    pub protocol: u8,
    pub string_index: u8,
    pub endpoints: Vec<EndpointInfo>,
}

impl InterfaceInfo {
    /// The interface's first endpoint of a type and direction.
    pub fn endpoint(&self, kind: EndpointType, dir_in: bool) -> Option<&EndpointInfo> {
        self.endpoints.iter().find(|e| e.kind == kind && e.is_in() == dir_in)
    }
}

/// What enumeration learnt about a device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    pub address: u8,
    /// The 18 byte device descriptor.
    pub device_descriptor: Vec<u8>,
    /// The whole configuration descriptor (with its interfaces, endpoints and class descriptors).
    pub config_descriptor: Vec<u8>,
    pub interfaces: Vec<InterfaceInfo>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub serial: Option<String>,
}

impl DeviceInfo {
    pub fn vendor_id(&self) -> u16 {
        self.device_descriptor.get(8..10).map_or(0, |b| u16::from_le_bytes([b[0], b[1]]))
    }

    pub fn product_id(&self) -> u16 {
        self.device_descriptor.get(10..12).map_or(0, |b| u16::from_le_bytes([b[0], b[1]]))
    }

    pub fn max_packet0(&self) -> u8 {
        self.device_descriptor.get(7).copied().unwrap_or(8)
    }

    /// The first interface (alternate setting 0) of a class.
    pub fn interface(&self, class: u8) -> Option<&InterfaceInfo> {
        self.interfaces.iter().find(|i| i.class == class && i.alternate == 0)
    }

    /// The endpoint with this address, in any interface.
    pub fn endpoint(&self, address: u8) -> Option<&EndpointInfo> {
        self.interfaces.iter().flat_map(|i| i.endpoints.iter()).find(|e| e.address == address)
    }
}

/// Splits a configuration descriptor into its interfaces and their endpoints (other descriptors are skipped).
pub fn parse_configuration(desc: &[u8]) -> Vec<InterfaceInfo> {
    let mut out: Vec<InterfaceInfo> = Vec::new();
    let mut p = 0;
    while p + 2 <= desc.len() {
        let len = desc[p] as usize;
        if len < 2 || p + len > desc.len() {
            break;
        }
        let d = &desc[p..p + len];
        match d[1] {
            desc_type::INTERFACE if len >= 9 => out.push(InterfaceInfo {
                number: d[2],
                alternate: d[3],
                class: d[5],
                subclass: d[6],
                protocol: d[7],
                string_index: d[8],
                endpoints: Vec::new(),
            }),
            desc_type::ENDPOINT if len >= 7 => {
                if let Some(i) = out.last_mut() {
                    i.endpoints.push(EndpointInfo {
                        address: d[2],
                        kind: EndpointType::from_attributes(d[3]),
                        max_packet: u16::from_le_bytes([d[4], d[5]]) & 0x7FF,
                        interval: d[6],
                    });
                }
            }
            _ => {}
        }
        p += len;
    }
    out
}

/// Decodes a string descriptor (UTF-16LE after the two header bytes).
pub fn parse_string(desc: &[u8]) -> Option<String> {
    if desc.len() < 2 || desc[1] != desc_type::STRING {
        return None;
    }
    let n = (desc[0] as usize).min(desc.len());
    let units: Vec<u16> = desc[2..n].chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    Some(String::from_utf16_lossy(&units))
}

/// Errors a transfer can end with.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UsbError {
    /// The endpoint answered STALL (a halted endpoint or a request the device does not support).
    Stall,
    /// The device did not answer three times in a row.
    NoResponse,
    /// The transfer's deadline passed (the device kept NAKing).
    TimedOut,
    /// The device went away (pull-up removed) or the bus was reset.
    Disconnected,
}

impl std::fmt::Display for UsbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UsbError::Stall => write!(f, "endpoint stalled"),
            UsbError::NoResponse => write!(f, "no response from the device"),
            UsbError::TimedOut => write!(f, "transfer timed out"),
            UsbError::Disconnected => write!(f, "device disconnected"),
        }
    }
}

impl std::error::Error for UsbError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn setup_packet_round_trip() {
        let s = SetupPacket::get_descriptor(desc_type::CONFIGURATION, 0, 0, 0x109);
        assert_eq!(s.bytes(), [0x80, 6, 0, 2, 0, 0, 0x09, 0x01]);
        assert_eq!(SetupPacket::parse(&s.bytes()), s);
        assert!(s.is_in());
    }

    #[test]
    fn configuration_parsing() {
        // A CDC-like configuration: IAD, comm interface with a class descriptor and an interrupt endpoint, data
        // interface with two bulk endpoints
        let desc = [
            9, 2, 75, 0, 2, 1, 0, 0x80, 50, // configuration
            8, 11, 0, 2, 2, 2, 0, 0, // IAD
            9, 4, 0, 0, 1, 2, 2, 0, 4, // interface 0, CDC comm
            5, 0x24, 0, 0x20, 1, // CDC header
            7, 5, 0x81, 3, 8, 0, 16, // EP1 IN interrupt
            9, 4, 1, 0, 2, 0x0A, 0, 0, 0, // interface 1, CDC data
            7, 5, 0x02, 2, 64, 0, 0, // EP2 OUT bulk
            7, 5, 0x82, 2, 64, 0, 0, // EP2 IN bulk
        ];
        let ifs = parse_configuration(&desc);
        assert_eq!(ifs.len(), 2);
        assert_eq!((ifs[0].class, ifs[0].string_index, ifs[0].endpoints.len()), (2, 4, 1));
        assert_eq!(ifs[0].endpoints[0].kind, EndpointType::Interrupt);
        assert_eq!(ifs[0].endpoints[0].interval, 16);
        let data = &ifs[1];
        assert_eq!(data.endpoint(EndpointType::Bulk, true).unwrap().address, 0x82);
        assert_eq!(data.endpoint(EndpointType::Bulk, false).unwrap().address, 0x02);
        assert_eq!(data.endpoint(EndpointType::Bulk, true).unwrap().max_packet, 64);
    }

    #[test]
    fn strings() {
        assert_eq!(parse_string(&[8, 3, b'R', 0, b'P', 0, b'2', 0]).as_deref(), Some("RP2"));
        assert_eq!(parse_string(&[4, 2, 0, 0]), None);
    }

    #[test]
    fn bit_times() {
        assert_eq!(bits(12_000), FRAME);
        assert_eq!(bits(12), Time::from_us(1));
    }
}
