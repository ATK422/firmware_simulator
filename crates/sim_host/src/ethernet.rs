//! The Ethernet side of a board with an Ethernet controller (the Puddles backplane's W5500): host participants
//! that exchange raw frames with the board's cable.
//!
//! [`PuddlesBridge`] plays `titan_bridge --board puddles_backplane` on the robot computer (`192.168.1.22`): it
//! answers ARP, speaks the Puddles UDP transport (`puddles_core::wire`: `"TL"` plus one postcard encoded
//! `titan_link` message per datagram, port 2203 on both ends) with the 100 ms ping and kill switch forwarding of
//! the CAN mock bridge, and records what the board sends, CANmore heartbeat broadcasts (port 2201) included.
//! [`UdpRelay`] instead connects the board to a real UDP socket on the host, so the real `titan_bridge` can talk to
//! it (see `docs/running.md`).

use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};

use canmore::Heartbeat;
use sim_core::time::Time;
use titan_link::{DeviceToHost, HostToDevice};

/// A host participant on a board's Ethernet cable.
pub trait EthEndpoint: Send {
    /// A frame the board sent, at `at`.
    fn receive(&mut self, frame: &[u8], at: Time);
    /// Frames to deliver to the board, up to `until`.
    fn take_tx(&mut self, until: Time) -> Vec<Vec<u8>>;
}

pub const ROBOT_COMPUTER_IP: [u8; 4] = [192, 168, 1, 22];
pub const BOARD_IP: [u8; 4] = [192, 168, 1, 42];
pub const LINK_PORT: u16 = 2203;
pub const HEARTBEAT_PORT: u16 = 2201;
const HOST_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x22];
const ETH_ARP: u16 = 0x0806;
const ETH_IPV4: u16 = 0x0800;

/// A UDP datagram parsed out of an Ethernet frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Udp {
    pub src_mac: [u8; 6],
    pub src: ([u8; 4], u16),
    pub dst: ([u8; 4], u16),
    pub payload: Vec<u8>,
}

/// What a frame carries, as far as a small host cares.
#[derive(Clone, Debug, PartialEq)]
pub enum Parsed {
    /// ARP: (operation, sender MAC, sender IP, target IP).
    Arp(u16, [u8; 6], [u8; 4], [u8; 4]),
    Udp(Udp),
    Other,
}

fn checksum(data: &[u8], mut sum: u32) -> u16 {
    for c in data.chunks(2) {
        sum += u16::from_be_bytes([c[0], *c.get(1).unwrap_or(&0)]) as u32;
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

pub fn parse(frame: &[u8]) -> Parsed {
    if frame.len() < 14 {
        return Parsed::Other;
    }
    let src_mac: [u8; 6] = frame[6..12].try_into().unwrap();
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    let p = &frame[14..];
    match ethertype {
        ETH_ARP if p.len() >= 28 => Parsed::Arp(
            u16::from_be_bytes([p[6], p[7]]),
            p[8..14].try_into().unwrap(),
            p[14..18].try_into().unwrap(),
            p[24..28].try_into().unwrap(),
        ),
        ETH_IPV4 if p.len() >= 20 && p[9] == 17 => {
            let ihl = (p[0] & 0x0F) as usize * 4;
            let total = (u16::from_be_bytes([p[2], p[3]]) as usize).min(p.len());
            if total < ihl + 8 {
                return Parsed::Other;
            }
            let u = &p[ihl..total];
            let len = (u16::from_be_bytes([u[4], u[5]]) as usize).min(u.len());
            if len < 8 {
                return Parsed::Other;
            }
            Parsed::Udp(Udp {
                src_mac,
                src: (p[12..16].try_into().unwrap(), u16::from_be_bytes([u[0], u[1]])),
                dst: (p[16..20].try_into().unwrap(), u16::from_be_bytes([u[2], u[3]])),
                payload: u[8..len].to_vec(),
            })
        }
        _ => Parsed::Other,
    }
}

/// An ARP frame from `HOST_MAC`/`ip`.
pub fn arp(op: u16, ip: [u8; 4], dst_mac: [u8; 6], target_mac: [u8; 6], target_ip: [u8; 4]) -> Vec<u8> {
    let mut f = Vec::with_capacity(42);
    f.extend(dst_mac);
    f.extend(HOST_MAC);
    f.extend(ETH_ARP.to_be_bytes());
    f.extend([0, 1, 8, 0, 6, 4]);
    f.extend(op.to_be_bytes());
    f.extend(HOST_MAC);
    f.extend(ip);
    f.extend(target_mac);
    f.extend(target_ip);
    f
}

/// A UDP datagram in an IPv4 frame from `HOST_MAC`, with both checksums.
pub fn udp(dst_mac: [u8; 6], src: ([u8; 4], u16), dst: ([u8; 4], u16), payload: &[u8]) -> Vec<u8> {
    let ulen = 8 + payload.len();
    let mut u = Vec::with_capacity(ulen);
    u.extend(src.1.to_be_bytes());
    u.extend(dst.1.to_be_bytes());
    u.extend((ulen as u16).to_be_bytes());
    u.extend([0, 0]);
    u.extend(payload);
    let mut pseudo = Vec::new();
    pseudo.extend(src.0);
    pseudo.extend(dst.0);
    pseudo.extend([0, 17]);
    pseudo.extend((ulen as u16).to_be_bytes());
    let pre: u32 = pseudo.chunks(2).map(|c| u16::from_be_bytes([c[0], c[1]]) as u32).sum();
    let c = match checksum(&u, pre) {
        0 => 0xFFFF,
        c => c,
    };
    u[6..8].copy_from_slice(&c.to_be_bytes());
    let mut ip = vec![0x45, 0, 0, 0, 0, 0, 0x40, 0, 64, 17, 0, 0];
    ip[2..4].copy_from_slice(&((20 + ulen) as u16).to_be_bytes());
    ip.extend(src.0);
    ip.extend(dst.0);
    let c = checksum(&ip, 0);
    ip[10..12].copy_from_slice(&c.to_be_bytes());
    let mut f = Vec::with_capacity(14 + 20 + ulen);
    f.extend(dst_mac);
    f.extend(HOST_MAC);
    f.extend(ETH_IPV4.to_be_bytes());
    f.extend(ip);
    f.extend(u);
    f
}

/// The robot computer's network presence: ARP for its address, and the board's MAC once known.
#[derive(Clone, Debug, Default)]
struct Host {
    board_mac: Option<[u8; 6]>,
    arp_sent: Option<Time>,
    out: Vec<Vec<u8>>,
}

impl Host {
    /// Handles ARP for the robot computer; learns the board's MAC.
    fn arp_in(&mut self, op: u16, mac: [u8; 6], ip: [u8; 4], target: [u8; 4]) {
        if ip == BOARD_IP {
            self.board_mac = Some(mac);
        }
        if op == 1 && target == ROBOT_COMPUTER_IP {
            self.out.push(arp(2, ROBOT_COMPUTER_IP, mac, mac, ip));
        }
    }

    /// Queues a datagram for the board, or an ARP request for it while its MAC is unknown (once a second).
    fn send(&mut self, payload: &[u8], now: Time) -> bool {
        match self.board_mac {
            Some(mac) => {
                self.out.push(udp(mac, (ROBOT_COMPUTER_IP, LINK_PORT), (BOARD_IP, LINK_PORT), payload));
                true
            }
            None => {
                if self.arp_sent.is_none_or(|t| now >= t + Time::from_secs(1)) {
                    self.out.push(arp(1, ROBOT_COMPUTER_IP, [0xFF; 6], [0; 6], BOARD_IP));
                    self.arp_sent = Some(now);
                }
                false
            }
        }
    }
}

/// What the Puddles mock bridge saw and is about to send.
#[derive(Clone, Debug, Default)]
pub struct PuddlesState {
    pub running: bool,
    /// The `state/kill` value forwarded with every ping (`Some(false)` enables the vehicle).
    pub kill: Option<bool>,
    pub received: Vec<(Time, DeviceToHost)>,
    /// CANmore heartbeat broadcasts.
    pub heartbeats: Vec<(Time, Heartbeat)>,
    pub errors: Vec<String>,
    pub pending: Vec<HostToDevice>,
    /// Encoded titan_link messages (any bytes) to send once as `"TL"` datagrams ([`PuddlesBridge::send_encoded`]).
    pub pending_encoded: Vec<Vec<u8>>,
    pub pings: u64,
    pub last_ping: Option<Time>,
    next_ping: Time,
    host: Host,
}

/// A scripted `titan_bridge` for the Puddles backplane, over the board's Ethernet.
#[derive(Clone)]
pub struct PuddlesBridge(pub Arc<Mutex<PuddlesState>>);

impl PuddlesBridge {
    pub fn new(start: Time) -> Self {
        PuddlesBridge(Arc::new(Mutex::new(PuddlesState { running: true, next_ping: start, ..Default::default() })))
    }

    pub fn state(&self) -> std::sync::MutexGuard<'_, PuddlesState> {
        self.0.lock().unwrap()
    }

    pub fn set_kill(&self, kill: Option<bool>) {
        self.state().kill = kill;
    }

    /// An independent bridge with a copy of this one's state (for a snapshot of the session it is attached to).
    pub fn fork(&self) -> Self {
        PuddlesBridge(Arc::new(Mutex::new(self.state().clone())))
    }

    pub fn send(&self, msg: HostToDevice) {
        self.state().pending.push(msg);
    }

    /// Sends `bytes` as an encoded titan_link message (a `"TL"` datagram around whatever they are).
    pub fn send_encoded(&self, bytes: &[u8]) {
        self.state().pending_encoded.push(bytes.to_vec());
    }

    pub fn stop(&self) {
        self.state().running = false;
    }

    /// Starts again (pinging from `at`), like restarting the `titan_bridge` process: it has to learn the board's
    /// MAC address again.
    pub fn start(&self, at: Time) {
        let mut s = self.state();
        s.running = true;
        s.next_ping = at;
        s.host = Host::default();
    }

    pub fn statuses(&self) -> Vec<(Time, titan_link::FirmwareStatus)> {
        self.state()
            .received
            .iter()
            .filter_map(|(t, m)| match m {
                DeviceToHost::FirmwareStatus(s) => Some((*t, s.clone())),
                _ => None,
            })
            .collect()
    }
}

fn datagram(msg: &HostToDevice) -> Option<Vec<u8>> {
    let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
    let enc = titan_link::encode(msg, &mut buf).ok()?;
    let mut d = b"TL".to_vec();
    d.extend_from_slice(enc);
    Some(d)
}

impl EthEndpoint for PuddlesBridge {
    fn receive(&mut self, frame: &[u8], at: Time) {
        let mut s = self.state();
        match parse(frame) {
            Parsed::Arp(op, mac, ip, target) => s.host.arp_in(op, mac, ip, target),
            Parsed::Udp(u) if u.src.0 == BOARD_IP => {
                s.host.board_mac = Some(u.src_mac);
                match u.dst.1 {
                    LINK_PORT => match u.payload.strip_prefix(b"TL").map(titan_link::decode::<DeviceToHost>) {
                        Some(Ok(m)) => s.received.push((at, m)),
                        _ => {
                            let e = format!("bad datagram {:02x?}", u.payload);
                            s.errors.push(e)
                        }
                    },
                    HEARTBEAT_PORT => match u.payload.as_slice() {
                        [b] => s.heartbeats.push((at, Heartbeat::from_byte(*b))),
                        other => {
                            let e = format!("bad heartbeat {other:02x?}");
                            s.errors.push(e)
                        }
                    },
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn take_tx(&mut self, until: Time) -> Vec<Vec<u8>> {
        let mut s = self.state();
        if s.running {
            let pending = std::mem::take(&mut s.pending);
            let mut kept = Vec::new();
            for msg in pending {
                if let Some(d) = datagram(&msg)
                    && !s.host.send(&d, until)
                {
                    kept.push(msg);
                }
            }
            s.pending = kept;
            let mut kept = Vec::new();
            for bytes in std::mem::take(&mut s.pending_encoded) {
                let mut d = b"TL".to_vec();
                d.extend_from_slice(&bytes);
                if !s.host.send(&d, until) {
                    kept.push(bytes);
                }
            }
            s.pending_encoded = kept;
            while s.next_ping <= until {
                let at = s.next_ping;
                let ping = HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION };
                if let Some(d) = datagram(&ping) {
                    s.host.send(&d, at);
                }
                if let Some(kill) = s.kill
                    && let Some(d) = datagram(&HostToDevice::KillSwitch { asserting_kill: kill })
                {
                    s.host.send(&d, at);
                }
                s.pings += 1;
                s.last_ping = Some(at);
                s.next_ping = at + Time::from_ms(titan_link::HOST_PING_INTERVAL_MS);
            }
        }
        std::mem::take(&mut s.host.out)
    }
}

/// Connects the board to a UDP socket on the host: the board's datagrams to the robot computer's port 2203 are sent
/// to `peer` (where `titan_bridge` listens), and datagrams arriving on the local socket reach the board as if from
/// the robot computer. Only datagrams from `peer`'s IP (any port; any sender if it is unspecified, `0.0.0.0`) are
/// accepted, as cannelloni does. Heartbeat broadcasts are forwarded to `peer`'s address on port 2201.
pub struct UdpRelay {
    socket: UdpSocket,
    peer: SocketAddr,
    host: Host,
}

impl UdpRelay {
    pub fn bind(local: SocketAddr, peer: SocketAddr) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(local)?;
        socket.set_nonblocking(true)?;
        Ok(UdpRelay { socket, peer, host: Host::default() })
    }

    /// The local address (for a port chosen by the system).
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.socket.local_addr()
    }
}

impl EthEndpoint for UdpRelay {
    fn receive(&mut self, frame: &[u8], _at: Time) {
        match parse(frame) {
            Parsed::Arp(op, mac, ip, target) => self.host.arp_in(op, mac, ip, target),
            Parsed::Udp(u) if u.src.0 == BOARD_IP => {
                self.host.board_mac = Some(u.src_mac);
                let to = match u.dst.1 {
                    HEARTBEAT_PORT => SocketAddr::new(self.peer.ip(), HEARTBEAT_PORT),
                    _ => self.peer,
                };
                if let Err(e) = self.socket.send_to(&u.payload, to) {
                    log::warn!("UDP relay: sending to {to}: {e}");
                }
            }
            _ => {}
        }
    }

    fn take_tx(&mut self, until: Time) -> Vec<Vec<u8>> {
        let mut buf = [0u8; 2048];
        while let Ok((n, from)) = self.socket.recv_from(&mut buf) {
            // Only the peer talks to the board (from any port), as the board only accepts the robot computer's IP;
            // an unspecified peer IP accepts every sender
            if from.ip() != self.peer.ip() && !self.peer.ip().is_unspecified() {
                log::debug!("UDP relay: ignoring a datagram from {from}");
                continue;
            }
            self.host.send(&buf[..n], until);
        }
        std::mem::take(&mut self.host.out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn udp_frames_round_trip_with_valid_checksums() {
        let f = udp([1, 2, 3, 4, 5, 6], (BOARD_IP, 1234), (ROBOT_COMPUTER_IP, LINK_PORT), b"TLhello");
        // IPv4 header checksum verifies to zero
        assert_eq!(checksum(&f[14..34], 0), 0);
        match parse(&f) {
            Parsed::Udp(u) => {
                assert_eq!(u.src, (BOARD_IP, 1234));
                assert_eq!(u.dst, (ROBOT_COMPUTER_IP, LINK_PORT));
                assert_eq!(u.payload, b"TLhello");
                assert_eq!(u.src_mac, HOST_MAC);
            }
            other => panic!("{other:?}"),
        }
        // The UDP checksum over the pseudo header verifies too
        let u = &f[34..];
        let mut pseudo = Vec::new();
        pseudo.extend(BOARD_IP);
        pseudo.extend(ROBOT_COMPUTER_IP);
        pseudo.extend([0, 17]);
        pseudo.extend((u.len() as u16).to_be_bytes());
        let pre: u32 = pseudo.chunks(2).map(|c| u16::from_be_bytes([c[0], c[1]]) as u32).sum();
        assert_eq!(checksum(u, pre), 0);
    }

    /// A UDP header whose length field is below its own 8 bytes (a malformed datagram from the firmware) is not
    /// a datagram; it used to panic slicing `u[8..len]`.
    #[test]
    fn udp_length_below_the_header_is_not_a_datagram() {
        for bad in [0u16, 7] {
            let mut f = udp([1, 2, 3, 4, 5, 6], (BOARD_IP, 1234), (ROBOT_COMPUTER_IP, LINK_PORT), b"TLhello");
            f[38..40].copy_from_slice(&bad.to_be_bytes());
            assert_eq!(parse(&f), Parsed::Other, "UDP length {bad}");
        }
        let mut f = udp([1, 2, 3, 4, 5, 6], (BOARD_IP, 1234), (ROBOT_COMPUTER_IP, LINK_PORT), b"TLhello");
        f[38..40].copy_from_slice(&8u16.to_be_bytes());
        assert!(matches!(parse(&f), Parsed::Udp(u) if u.payload.is_empty()), "an empty datagram");
    }

    #[test]
    fn bridge_answers_arp_and_waits_for_the_board_mac() {
        let mut b = PuddlesBridge::new(Time::ZERO);
        // No MAC yet: the first ping becomes an ARP request for the board
        let out = b.take_tx(Time::from_ms(50));
        assert_eq!(out.len(), 1);
        assert_eq!(parse(&out[0]), Parsed::Arp(1, HOST_MAC, ROBOT_COMPUTER_IP, BOARD_IP));
        // The board asks who has 192.168.1.22: the bridge answers, and learns the board's MAC
        let board_mac = [0x2A, 0xCD, 9, 9, 9, 9];
        let mut req = arp(1, BOARD_IP, [0xFF; 6], [0; 6], ROBOT_COMPUTER_IP);
        req[6..12].copy_from_slice(&board_mac);
        req[22..28].copy_from_slice(&board_mac);
        b.receive(&req, Time::from_ms(60));
        let out = b.take_tx(Time::from_ms(150));
        assert_eq!(parse(&out[0]), Parsed::Arp(2, HOST_MAC, ROBOT_COMPUTER_IP, BOARD_IP));
        // Then pings go out as datagrams to the board
        match parse(&out[1]) {
            Parsed::Udp(u) => {
                assert_eq!(u.dst, (BOARD_IP, LINK_PORT));
                assert_eq!(&u.payload[..2], b"TL");
                let msg: HostToDevice = titan_link::decode(&u.payload[2..]).unwrap();
                assert!(matches!(msg, HostToDevice::Ping { .. }));
            }
            other => panic!("{other:?}"),
        }
    }

    /// Only the peer's IP may talk to the board through the relay (as the board itself only accepts the robot
    /// computer's IP), from any port; a datagram from anywhere else on the network is dropped.
    #[test]
    fn relay_only_forwards_datagrams_from_the_peer() {
        let bridge = UdpSocket::bind("127.0.0.1:0").unwrap();
        let other_port = UdpSocket::bind("127.0.0.1:0").unwrap();
        // Another host's IP: a second loopback address, which Linux has (all of 127/8) and macOS doesn't by default
        let stranger = match UdpSocket::bind("127.0.0.2:0") {
            Ok(s) => s,
            Err(e) => {
                eprintln!("skipping: no second loopback address here to send from ({e})");
                return;
            }
        };
        let mut relay = UdpRelay::bind("127.0.0.1:0".parse().unwrap(), bridge.local_addr().unwrap()).unwrap();
        let to = relay.local_addr().unwrap();
        // The board has sent something: its MAC is known
        relay.receive(&udp(HOST_MAC, (BOARD_IP, LINK_PORT), (ROBOT_COMPUTER_IP, LINK_PORT), b"TLup"), Time::ZERO);
        stranger.send_to(b"TLstranger", to).unwrap();
        bridge.send_to(b"TLbridge", to).unwrap();
        other_port.send_to(b"TLother port", to).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));
        let got: Vec<Vec<u8>> = relay
            .take_tx(Time::from_ms(1))
            .iter()
            .map(|f| match parse(f) {
                Parsed::Udp(u) => u.payload,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(got, [b"TLbridge".to_vec(), b"TLother port".to_vec()]);
    }
}
