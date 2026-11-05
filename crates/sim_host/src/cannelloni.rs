//! CAN over UDP, in the framing of [cannelloni](https://github.com/mguentner/cannelloni)'s UDP mode, so a bus
//! can reach host tools on another machine, VM or container that has its own `vcan` (Docker Desktop, a VM without
//! the emulator's network namespace, a robot computer on the LAN). There, `cannelloni -I vcan0 -R <emulator host>
//! -r <emulator port> -l <its port>` (or `firmware_simulator can-tunnel`) connects the far `vcan0` to the bus.
//!
//! Host tools can also speak it themselves, with no `vcan` anywhere: the emulator's **hub** mode
//! ([`Cannelloni::hub`]) serves any number of them on one port (titan_bridge's `--interface cannelloni:HOST:PORT`),
//! which is how the robot computer's side runs where there is no SocketCAN (WSL2, macOS, a laptop without sudo).
//!
//! A datagram is a five byte header, then the frames:
//!
//! ```text
//! version (2) | op code (0 = data) | sequence number | frame count (u16, big endian)
//! per frame:  can_id (u32, big endian, Linux flags: 0x80000000 extended, 0x40000000 remote)
//!             len (u8, | 0x80 for a CAN FD frame) | [CAN FD flags (u8: 1 = BRS, 2 = ESI)] | data (none if remote)
//! ```
//!
//! This follows cannelloni's protocol version 2 as its source describes it; it has been tested against this
//! module's own peer over loopback, not against the `cannelloni` program itself.

use std::io;
use std::net::{SocketAddr, UdpSocket};

use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

const VERSION: u8 = 2;
const OP_DATA: u8 = 0;
const HEADER_LEN: usize = 5;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CANFD_FRAME: u8 = 0x80;
const CANFD_BRS: u8 = 0x01;
const CANFD_ESI: u8 = 0x02;
/// Datagrams are kept under a typical Ethernet MTU.
pub const MAX_PACKET: usize = 1400;

fn frame_len(f: &CanFrame) -> usize {
    4 + 1 + f.fd as usize + if f.rtr { 0 } else { f.data.len() }
}

/// Packs frames into as few datagrams as fit [`MAX_PACKET`]; `seq` numbers them and is advanced.
pub fn encode(frames: &[CanFrame], seq: &mut u8) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut rest = frames;
    while !rest.is_empty() {
        let mut size = HEADER_LEN;
        let n = rest.iter().take_while(|f| {
            size += frame_len(f);
            size <= MAX_PACKET
        });
        let n = n.count().max(1);
        let mut p = vec![VERSION, OP_DATA, *seq];
        p.extend_from_slice(&(n as u16).to_be_bytes());
        for f in &rest[..n] {
            let mut id = if f.extended { f.id & 0x1FFF_FFFF | CAN_EFF_FLAG } else { f.id & 0x7FF };
            if f.rtr {
                id |= CAN_RTR_FLAG;
            }
            p.extend_from_slice(&id.to_be_bytes());
            if f.fd {
                p.push(f.data.len() as u8 | CANFD_FRAME);
                p.push(if f.brs { CANFD_BRS } else { 0 } | if f.esi { CANFD_ESI } else { 0 });
            } else {
                p.push(f.data.len() as u8);
            }
            if !f.rtr {
                p.extend_from_slice(&f.data);
            }
        }
        out.push(p);
        *seq = seq.wrapping_add(1);
        rest = &rest[n..];
    }
    out
}

/// Unpacks a datagram: its sequence number and frames (error frames, and CAN FD frames flagged as remote, are
/// dropped).
pub fn decode(p: &[u8]) -> Result<(u8, Vec<CanFrame>), String> {
    if p.len() < HEADER_LEN {
        return Err(format!("{} byte datagram", p.len()));
    }
    if p[0] != VERSION {
        return Err(format!("protocol version {} (expected {VERSION})", p[0]));
    }
    if p[1] != OP_DATA {
        return Err(format!("op code {} (only data packets are handled)", p[1]));
    }
    let count = u16::from_be_bytes([p[3], p[4]]) as usize;
    let mut at = HEADER_LEN;
    let mut take = |n: usize| -> Result<&[u8], String> {
        let s = p.get(at..at + n).ok_or_else(|| format!("datagram ends inside a frame ({} bytes)", p.len()))?;
        at += n;
        Ok(s)
    };
    let mut frames = Vec::with_capacity(count);
    for _ in 0..count {
        let id = u32::from_be_bytes(take(4)?.try_into().unwrap());
        let len_byte = take(1)?[0];
        let fd = len_byte & CANFD_FRAME != 0;
        let len = (len_byte & !CANFD_FRAME) as usize;
        let flags = if fd { take(1)?[0] } else { 0 };
        if len > if fd { 64 } else { 8 } {
            return Err(format!("{len} byte frame"));
        }
        // As cannelloni's parser: no data follows any id with the RTR flag, CAN FD or not
        let rtr = id & CAN_RTR_FLAG != 0;
        let data = if rtr { vec![0; len] } else { take(len)?.to_vec() };
        // CAN FD has no remote frames
        if id & CAN_ERR_FLAG != 0 || (fd && rtr) {
            continue;
        }
        let extended = id & CAN_EFF_FLAG != 0;
        frames.push(CanFrame {
            id: if extended { id & 0x1FFF_FFFF } else { id & 0x7FF },
            extended,
            fd,
            brs: fd && flags & CANFD_BRS != 0,
            rtr,
            esi: fd && flags & CANFD_ESI != 0,
            data,
        });
    }
    Ok((p[2], frames))
}

/// How long a hub forgets a peer it has not heard from (see [`Cannelloni::hub`]). Peers send at least a keepalive
/// every [`KEEPALIVE`].
pub const HUB_PEER_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(10);
/// How often a quiet peer of a hub sends an empty datagram, so the hub keeps it.
pub const KEEPALIVE: std::time::Duration = std::time::Duration::from_secs(1);
/// The most peers a hub keeps (the one heard from longest ago goes first).
pub const HUB_MAX_PEERS: usize = 64;

/// A peer of a hub: where it is, when it was last heard from, and the sequence numbers each way.
struct HubPeer {
    addr: SocketAddr,
    last: std::time::Instant,
    tx_seq: u8,
    next_rx_seq: Option<u8>,
}

/// A bus endpoint that tunnels the bus to a cannelloni peer over UDP: frames on the bus are sent (batched once per
/// bus step), frames received are put on the bus. Use it in real-time mode, like a SocketCAN bridge.
///
/// As a **hub** ([`Cannelloni::hub`]: `--cannelloni BUS=LOCAL`, no peer) it serves any number of peers that send
/// to its address, each a host tool (titan_bridge's `cannelloni:` interface, `can-tunnel`) as if on a shared vcan:
/// every peer gets the bus's frames and every other peer's, and a peer it has not heard from for
/// [`HUB_PEER_TIMEOUT`] is forgotten. Peers are learnt from the datagrams they send (an empty one, frame count 0,
/// is a keepalive), so the hub needs no peer addresses and works through NAT (a container sending to its host).
pub struct Cannelloni {
    socket: UdpSocket,
    /// The peer; `None` for a hub.
    pub peer: Option<SocketAddr>,
    seq: u8,
    /// Frames to send: from the bus (`None`), or from a hub's peer to the others.
    pending: Vec<(Option<SocketAddr>, CanFrame)>,
    /// Frames received to answer [`CanEndpoint::waiting`], handed in by the next `take_tx`.
    held: Vec<CanFrame>,
    hub: Vec<HubPeer>,
    /// How long a hub keeps a quiet peer ([`HUB_PEER_TIMEOUT`]).
    pub hub_timeout: std::time::Duration,
    /// Frames sent to and received from the peer(s), and datagrams that could not be decoded.
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub rx_errors: u64,
    /// Datagrams that arrived out of sequence (lost or reordered on the way).
    pub rx_gaps: u64,
    next_rx_seq: Option<u8>,
    /// When something was last sent to the peer (`None`: never).
    last_tx: Option<std::time::Instant>,
}

impl Cannelloni {
    /// Binds `local` and sends to `peer`; only datagrams from `peer`'s address are accepted.
    pub fn bind(local: SocketAddr, peer: SocketAddr) -> io::Result<Cannelloni> {
        Self::open(local, Some(peer))
    }

    /// Binds `local` as a hub for any number of peers (see [`Cannelloni`]).
    pub fn hub(local: SocketAddr) -> io::Result<Cannelloni> {
        Self::open(local, None)
    }

    /// [`Cannelloni::bind`] with a peer, [`Cannelloni::hub`] without.
    pub fn open(local: SocketAddr, peer: Option<SocketAddr>) -> io::Result<Cannelloni> {
        let socket = UdpSocket::bind(local)?;
        socket.set_nonblocking(true)?;
        Ok(Cannelloni {
            socket,
            peer,
            seq: 0,
            pending: Vec::new(),
            held: Vec::new(),
            hub: Vec::new(),
            hub_timeout: HUB_PEER_TIMEOUT,
            tx_frames: 0,
            rx_frames: 0,
            rx_errors: 0,
            rx_gaps: 0,
            next_rx_seq: None,
            last_tx: None,
        })
    }

    /// Parses `LOCAL,PEER` (e.g. `0.0.0.0:20000,192.168.1.50:20001`), or `LOCAL` alone for a hub.
    pub fn parse_spec(spec: &str) -> Result<(SocketAddr, Option<SocketAddr>), String> {
        let addr = |s: &str| s.trim().parse::<SocketAddr>().map_err(|e| format!("{s:?}: {e}"));
        match spec.split_once(',') {
            Some((l, p)) => Ok((addr(l)?, Some(addr(p)?))),
            None => Ok((addr(spec).map_err(|e| format!("expected LOCAL,PEER or LOCAL addresses: {e}"))?, None)),
        }
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// The peers a hub serves now.
    pub fn hub_peers(&self) -> Vec<SocketAddr> {
        self.hub.iter().map(|p| p.addr).collect()
    }

    /// Sends the frames waiting to go to the peer (or to a hub's peers).
    pub fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let pending = std::mem::take(&mut self.pending);
        if let Some(peer) = self.peer {
            let frames: Vec<CanFrame> = pending.into_iter().map(|(_, f)| f).collect();
            for p in encode(&frames, &mut self.seq) {
                if let Err(e) = self.socket.send_to(&p, peer) {
                    log::warn!("cannelloni: sending to {peer}: {e}");
                    return;
                }
            }
            self.tx_frames += frames.len() as u64;
            self.last_tx = Some(std::time::Instant::now());
            return;
        }
        for peer in &mut self.hub {
            let frames: Vec<CanFrame> =
                pending.iter().filter(|(from, _)| *from != Some(peer.addr)).map(|(_, f)| f.clone()).collect();
            if frames.is_empty() {
                continue;
            }
            for p in encode(&frames, &mut peer.tx_seq) {
                // A peer that went away answers with an ICMP error, reported on a later call; it is forgotten
                // once it has been quiet for HUB_PEER_TIMEOUT
                if let Err(e) = self.socket.send_to(&p, peer.addr) {
                    log::debug!("cannelloni hub: sending to {}: {e}", peer.addr);
                    break;
                }
            }
            self.tx_frames += frames.len() as u64;
        }
    }

    /// Sends an empty datagram (frame count 0) if nothing was sent yet or for [`KEEPALIVE`], so a hub learns and
    /// keeps this peer.
    pub fn keepalive(&mut self) {
        let Some(peer) = self.peer else { return };
        if self.last_tx.is_some_and(|t| t.elapsed() < KEEPALIVE) {
            return;
        }
        let p = [VERSION, OP_DATA, self.seq, 0, 0];
        self.seq = self.seq.wrapping_add(1);
        self.last_tx = Some(std::time::Instant::now());
        if let Err(e) = self.socket.send_to(&p, peer) {
            log::debug!("cannelloni: keepalive to {peer}: {e}");
        }
    }

    /// Every frame waiting in the socket (for a hub, the ones its peers sent, which also go to the other peers).
    pub fn receive_all(&mut self) -> Vec<CanFrame> {
        let mut out = Vec::new();
        let mut buf = [0u8; 65536];
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((n, from)) => {
                    if let Some(peer) = self.peer
                        && from.ip() != peer.ip()
                        && !peer.ip().is_unspecified()
                    {
                        continue;
                    }
                    let (seq, frames) = match decode(&buf[..n]) {
                        Ok(d) => d,
                        Err(e) => {
                            if self.rx_errors == 0 {
                                log::warn!("cannelloni: datagram from {from}: {e}");
                            }
                            self.rx_errors += 1;
                            continue;
                        }
                    };
                    let next_rx_seq =
                        if self.peer.is_some() { &mut self.next_rx_seq } else { &mut self.hub_peer(from).next_rx_seq };
                    let gap = next_rx_seq.is_some_and(|s| s != seq);
                    *next_rx_seq = Some(seq.wrapping_add(1));
                    if gap {
                        self.rx_gaps += 1;
                    }
                    self.rx_frames += frames.len() as u64;
                    if self.peer.is_none() && self.hub.len() > 1 {
                        self.pending.extend(frames.iter().map(|f| (Some(from), f.clone())));
                    }
                    out.extend(frames);
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                // An ICMP port unreachable from a peer that isn't up (or is gone) shows up here on Linux; it is
                // consumed, so read on
                Err(e) if matches!(e.kind(), io::ErrorKind::ConnectionRefused | io::ErrorKind::ConnectionReset) => {
                    log::debug!("cannelloni: {e}");
                }
                Err(e) => {
                    log::debug!("cannelloni: {e}");
                    break;
                }
            }
        }
        if self.peer.is_none() {
            let timeout = self.hub_timeout;
            self.hub.retain(|p| {
                let keep = p.last.elapsed() < timeout;
                if !keep {
                    log::info!("cannelloni hub: {} left (nothing from it for {timeout:?})", p.addr);
                }
                keep
            });
        }
        out
    }

    /// A hub's record of the peer at `addr`, added if new.
    fn hub_peer(&mut self, addr: SocketAddr) -> &mut HubPeer {
        let now = std::time::Instant::now();
        let at = match self.hub.iter().position(|p| p.addr == addr) {
            Some(i) => i,
            None => {
                if self.hub.len() >= HUB_MAX_PEERS {
                    let oldest = (0..self.hub.len()).min_by_key(|&i| self.hub[i].last).unwrap();
                    let gone = self.hub.remove(oldest);
                    log::warn!("cannelloni hub: {HUB_MAX_PEERS} peers; forgetting {}", gone.addr);
                }
                log::info!("cannelloni hub: {addr} joined");
                self.hub.push(HubPeer { addr, last: now, tx_seq: 0, next_rx_seq: None });
                self.hub.len() - 1
            }
        };
        self.hub[at].last = now;
        &mut self.hub[at]
    }
}

impl Drop for Cannelloni {
    fn drop(&mut self) {
        self.flush();
    }
}

impl CanEndpoint for Cannelloni {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        self.flush();
        let mut frames = std::mem::take(&mut self.held);
        frames.extend(self.receive_all());
        frames.into_iter().map(|f| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        // A hub with no peers has no one to send to
        if self.peer.is_some() || !self.hub.is_empty() {
            self.pending.push((None, frame.clone()));
        }
    }

    /// The far side's frames wait for the bus in a bounded queue (the far `vcan` would refuse writes with `ENOBUFS`
    /// on real hardware), so a stuck or flooded bus does not collect them without end.
    fn tx_queue_limit(&self) -> Option<usize> {
        Some(sim_devices::can::BRIDGE_TX_QUEUE)
    }

    /// What the far side sent is handed in at the step that reads it.
    fn live(&self) -> bool {
        true
    }

    fn waiting(&mut self, _until: Time) -> bool {
        let more = self.receive_all();
        self.held.extend(more);
        !self.held.is_empty()
    }
}

/// Connects a SocketCAN interface to a cannelloni peer until the process is stopped: the far end of a tunnel to
/// an emulated bus, for a machine whose tools (titan_bridge, candump) use a local `vcan`.
pub fn tunnel(iface: &str, local: SocketAddr, peer: SocketAddr) -> io::Result<()> {
    let mut can = crate::socketcan::SocketCan::open(iface)?;
    let mut udp = Cannelloni::bind(local, peer)?;
    log::info!("tunnel: {iface} <-> udp {local} -> {peer}");
    let mut last_report = std::time::Instant::now();
    loop {
        let mut idle = true;
        let from_can = can.read_all();
        if !from_can.is_empty() {
            idle = false;
            udp.pending.extend(from_can.into_iter().map(|f| (None, f)));
            udp.flush();
        }
        // So an emulator's hub keeps this tunnel while its vcan is quiet
        udp.keepalive();
        for f in udp.receive_all() {
            idle = false;
            if let Err(e) = can.write(&f) {
                log::warn!("tunnel: writing to {iface}: {e}");
            }
        }
        if last_report.elapsed() > std::time::Duration::from_secs(10) {
            last_report = std::time::Instant::now();
            log::info!(
                "tunnel: {} frame(s) to the emulator, {} from it, {} bad datagram(s), {} sequence gap(s)",
                udp.tx_frames,
                udp.rx_frames,
                udp.rx_errors,
                udp.rx_gaps
            );
        }
        if idle {
            // Frames arrive every few hundred µs at most on the robot buses; this keeps latency well under a ms
            std::thread::sleep(std::time::Duration::from_micros(200));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_devices::can::{BRIDGE_TX_QUEUE, CanBus, Monitor};

    #[test]
    fn packet_layout() {
        let frames = [
            CanFrame::classic(0x6F, false, &[0xC0]),
            CanFrame::fd(0x0140_0801, true, &[1, 2, 3]),
            CanFrame { rtr: true, ..CanFrame::classic(0x123, false, &[0, 0]) },
        ];
        let mut seq = 7;
        let p = encode(&frames, &mut seq);
        assert_eq!(seq, 8);
        assert_eq!(p.len(), 1);
        #[rustfmt::skip]
        let expect = [
            2, 0, 7, 0, 3,
            0, 0, 0, 0x6F, 1, 0xC0,
            0x81, 0x40, 0x08, 0x01, 0x83, 0x01, 1, 2, 3,
            0x40, 0, 0x01, 0x23, 2,
        ];
        assert_eq!(p[0], expect);
        let (s, back) = decode(&p[0]).unwrap();
        assert_eq!((s, back.as_slice()), (7, &frames[..]));
    }

    #[test]
    fn large_batches_split_and_bad_datagrams_fail() {
        let frames: Vec<CanFrame> = (0..100).map(|i| CanFrame::fd(i, false, &[i as u8; 64])).collect();
        let mut seq = 255;
        let packets = encode(&frames, &mut seq);
        assert!(packets.len() > 1 && packets.iter().all(|p| p.len() <= MAX_PACKET));
        let back: Vec<CanFrame> = packets.iter().flat_map(|p| decode(p).unwrap().1).collect();
        assert_eq!(back, frames);
        assert_eq!(decode(&packets[1]).unwrap().0, 0, "the sequence number wraps");
        assert!(decode(&[2, 0, 0]).is_err());
        assert!(decode(&[1, 0, 0, 0, 0]).is_err(), "version 1");
        assert!(decode(&[2, 0, 0, 0, 1, 0, 0, 0, 1, 4, 1]).is_err(), "truncated");
        assert!(decode(&[2, 0, 0, 0, 1, 0, 0, 0, 1, 9]).is_err(), "nine byte classic frame");
        let err_frame = [2, 0, 0, 0, 1, 0x20, 0, 0, 4, 0];
        assert_eq!(decode(&err_frame).unwrap().1, vec![], "error frames are dropped");
        assert!(Cannelloni::parse_spec("127.0.0.1:1,127.0.0.1:2").is_ok());
        assert_eq!(Cannelloni::parse_spec("127.0.0.1:1"), Ok(("127.0.0.1:1".parse().unwrap(), None)), "a hub");
        assert!(Cannelloni::parse_spec("127.0.0.1").is_err());
        assert!(Cannelloni::parse_spec("127.0.0.1:1,nowhere").is_err());
    }

    /// cannelloni's parser reads no data for any id with `CAN_RTR_FLAG`, CAN FD or not (and its encoder writes
    /// none), so the frames after such a one are where cannelloni put them. CAN FD has no remote frames: that one
    /// is dropped.
    #[test]
    fn an_fd_frame_with_the_rtr_flag_carries_no_data() {
        #[rustfmt::skip]
        let p = [
            2, 0, 0, 0, 2,
            0x40, 0, 0x01, 0x23, 0x88, 0x01,
            0, 0, 0, 0x6F, 1, 0xC0,
        ];
        assert_eq!(decode(&p).unwrap().1, vec![CanFrame::classic(0x6F, false, &[0xC0])]);
    }

    /// What the peer sent can be noticed before it is handed in: it is held, and handed in by the next `take_tx`,
    /// stamped with that step, before anything that came later.
    #[test]
    fn a_waiting_frame_is_held_for_the_next_step() {
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        let mut t = Cannelloni::bind("127.0.0.1:0".parse().unwrap(), peer.local_addr().unwrap()).unwrap();
        let to = t.local_addr().unwrap();
        assert!(t.live() && !t.waiting(Time::ZERO));
        let mut seq = 0;
        let send = |f: CanFrame, seq: &mut u8| {
            for p in encode(&[f], seq) {
                peer.send_to(&p, to).unwrap();
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        };
        let (a, b) = (CanFrame::classic(1, false, &[1]), CanFrame::classic(2, false, &[2]));
        send(a.clone(), &mut seq);
        assert!(t.waiting(Time::from_us(100)) && t.waiting(Time::from_us(100)), "still waiting, held once");
        send(b.clone(), &mut seq);
        let at = Time::from_us(100);
        assert_eq!(t.take_tx(at), [(at, a), (at, b)]);
        assert!(!t.waiting(at) && t.take_tx(at).is_empty());
    }

    /// A peer that keeps sending while the bus is stuck: the tunnel's frames wait in a bounded queue, and the bus
    /// drops (and counts) the newest beyond it, as it does for a mock bridge, instead of queueing without bound.
    #[test]
    fn a_stuck_bus_bounds_the_tunnel_queue() {
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        let mut t = Cannelloni::bind("127.0.0.1:0".parse().unwrap(), peer.local_addr().unwrap()).unwrap();
        let to = t.local_addr().unwrap();
        let rates = crate::board::bus_rates("internal").unwrap();
        let mut bus = CanBus::new("a", rates);
        let mut mon = Monitor::default();
        bus.set_stuck_dominant(true, Time::ZERO);
        let frames: Vec<CanFrame> =
            (0..3000).map(|i| CanFrame::classic(0x100, false, &(i as u16).to_be_bytes())).collect();
        let mut seq = 0;
        for (k, chunk) in frames.chunks(500).enumerate() {
            for p in encode(chunk, &mut seq) {
                peer.send_to(&p, to).unwrap();
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
            bus.step(&mut [], &mut [&mut mon, &mut t], Time::from_ms(k as u64 + 1));
        }
        assert_eq!(t.rx_frames, 3000);
        assert_eq!(bus.endpoint_dropped, 3000 - BRIDGE_TX_QUEUE as u64);
        bus.set_stuck_dominant(false, Time::from_ms(10));
        bus.step(&mut [], &mut [&mut mon, &mut t], Time::from_secs(1));
        assert_eq!(mon.received.len(), BRIDGE_TX_QUEUE, "the oldest go out once the bus is free");
        assert_eq!(mon.received[BRIDGE_TX_QUEUE - 1].1, frames[BRIDGE_TX_QUEUE - 1]);
    }

    /// A hub serves every peer that sends to it, as a shared vcan would: the bus's frames go to all of them, a
    /// peer's frames go onto the bus and to the other peers (never back to it), an empty datagram is a keepalive,
    /// and a peer that goes quiet is forgotten.
    #[test]
    fn a_hub_serves_its_peers() {
        let mut hub = Cannelloni::hub("127.0.0.1:0".parse().unwrap()).unwrap();
        hub.hub_timeout = std::time::Duration::from_millis(300);
        let at = hub.local_addr().unwrap();
        let rates = crate::board::bus_rates("internal").unwrap();
        let mut bus = CanBus::new("internal", rates);
        let mut board = Monitor::default();
        let mut a = Cannelloni::bind("127.0.0.1:0".parse().unwrap(), at).unwrap();
        let mut b = Cannelloni::bind("127.0.0.1:0".parse().unwrap(), at).unwrap();
        let settle = || std::thread::sleep(std::time::Duration::from_millis(30));
        // Before anyone has sent, the hub has no one to send to
        board.outbox.push((Time::from_us(10), CanFrame::classic(0x10, false, &[0])));
        bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_us(100));
        assert!(hub.hub_peers().is_empty() && hub.tx_frames == 0);
        // A says hello with a keepalive (it has sent nothing yet), B with a frame
        a.keepalive();
        b.pending.push((None, CanFrame::fd(0x55, true, &[1, 2, 3])));
        b.flush();
        settle();
        bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_us(200));
        bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_us(300));
        assert_eq!(hub.hub_peers(), [a.local_addr().unwrap(), b.local_addr().unwrap()]);
        assert_eq!(board.received.iter().map(|(_, f)| f.id).collect::<Vec<_>>(), [0x55], "B's frame on the bus");
        // The board answers: both peers get it; A also got B's frame, B did not get its own back
        board.outbox.push((Time::from_us(310), CanFrame::classic(0x6F, false, &[0xC0])));
        bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_us(400));
        bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_us(500));
        settle();
        let ids = |t: &mut Cannelloni| t.receive_all().iter().map(|f| f.id).collect::<Vec<_>>();
        assert_eq!(ids(&mut a), [0x55, 0x6F]);
        assert_eq!(ids(&mut b), [0x6F]);
        assert_eq!((a.rx_gaps, b.rx_gaps, hub.rx_gaps), (0, 0, 0), "each peer's own sequence numbers");
        // B goes quiet and is forgotten; A keeps itself in with keepalives
        for k in 0..4 {
            std::thread::sleep(std::time::Duration::from_millis(120));
            a.last_tx = None;
            a.keepalive();
            settle();
            bus.step(&mut [], &mut [&mut board, &mut hub], Time::from_ms(1 + k));
        }
        assert_eq!(hub.hub_peers(), [a.local_addr().unwrap()]);
        assert_eq!(hub.rx_errors, 0);
    }

    /// Two buses joined by a tunnel over loopback: what is sent on one arrives on the other, both ways.
    #[test]
    fn tunnel_between_two_buses() {
        let a_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let b_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let (a_addr, b_addr) = (a_sock.local_addr().unwrap(), b_sock.local_addr().unwrap());
        drop((a_sock, b_sock));
        let mut ta = Cannelloni::bind(a_addr, b_addr).unwrap();
        let mut tb = Cannelloni::bind(b_addr, a_addr).unwrap();
        let rates = crate::board::bus_rates("internal").unwrap();
        let (mut bus_a, mut bus_b) = (CanBus::new("a", rates), CanBus::new("b", rates));
        let (mut ma, mut mb) = (Monitor::default(), Monitor::default());
        ma.outbox.push((Time::from_us(10), CanFrame::fd(0x55, true, &[1, 2, 3])));
        mb.outbox.push((Time::from_us(20), CanFrame::classic(0x6F, false, &[0xC0])));
        for k in 1..=5 {
            let t = Time::from_us(100 * k);
            bus_a.step(&mut [], &mut [&mut ma, &mut ta], t);
            bus_b.step(&mut [], &mut [&mut mb, &mut tb], t);
        }
        assert_eq!(mb.received.iter().map(|(_, f)| f.id).collect::<Vec<_>>(), [0x55]);
        assert_eq!(ma.received.iter().map(|(_, f)| f.id).collect::<Vec<_>>(), [0x6F]);
        assert_eq!(mb.received[0].1, CanFrame::fd(0x55, true, &[1, 2, 3]));
        assert_eq!((ta.tx_frames, ta.rx_frames, tb.rx_gaps), (1, 1, 0));
    }
}
