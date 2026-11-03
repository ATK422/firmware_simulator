//! CAN over UDP, in the framing of [cannelloni](https://github.com/mguentner/cannelloni)'s UDP mode, so a bus
//! can reach host tools on another machine, VM or container that has its own `vcan` (Docker Desktop, a VM without
//! the emulator's network namespace, a robot computer on the LAN). There, `cannelloni -I vcan0 -R <emulator host>
//! -r <emulator port> -l <its port>` (or `firmware_simulator can-tunnel`) connects the far `vcan0` to the bus.
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

/// Unpacks a datagram: its sequence number and frames (error frames are dropped).
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
        let rtr = !fd && id & CAN_RTR_FLAG != 0;
        let data = if rtr { vec![0; len] } else { take(len)?.to_vec() };
        if id & CAN_ERR_FLAG != 0 {
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

/// A bus endpoint that tunnels the bus to a cannelloni peer over UDP: frames on the bus are sent (batched once per
/// bus step), frames received are put on the bus. Use it in real-time mode, like a SocketCAN bridge.
pub struct Cannelloni {
    socket: UdpSocket,
    pub peer: SocketAddr,
    seq: u8,
    pending: Vec<CanFrame>,
    /// Frames sent to and received from the peer, and datagrams that could not be decoded.
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub rx_errors: u64,
    /// Datagrams that arrived out of sequence (lost or reordered on the way).
    pub rx_gaps: u64,
    next_rx_seq: Option<u8>,
}

impl Cannelloni {
    /// Binds `local` and sends to `peer`; only datagrams from `peer`'s address are accepted.
    pub fn bind(local: SocketAddr, peer: SocketAddr) -> io::Result<Cannelloni> {
        let socket = UdpSocket::bind(local)?;
        socket.set_nonblocking(true)?;
        Ok(Cannelloni {
            socket,
            peer,
            seq: 0,
            pending: Vec::new(),
            tx_frames: 0,
            rx_frames: 0,
            rx_errors: 0,
            rx_gaps: 0,
            next_rx_seq: None,
        })
    }

    /// Parses `LOCAL,PEER` (e.g. `0.0.0.0:20000,192.168.1.50:20001`).
    pub fn parse_spec(spec: &str) -> Result<(SocketAddr, SocketAddr), String> {
        let (l, p) = spec.split_once(',').ok_or("expected LOCAL,PEER addresses")?;
        let addr = |s: &str| s.trim().parse::<SocketAddr>().map_err(|e| format!("{s:?}: {e}"));
        Ok((addr(l)?, addr(p)?))
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Sends the frames waiting to go to the peer.
    pub fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let frames = std::mem::take(&mut self.pending);
        for p in encode(&frames, &mut self.seq) {
            if let Err(e) = self.socket.send_to(&p, self.peer) {
                log::warn!("cannelloni: sending to {}: {e}", self.peer);
                return;
            }
        }
        self.tx_frames += frames.len() as u64;
    }

    /// Every frame waiting in the socket.
    pub fn receive_all(&mut self) -> Vec<CanFrame> {
        let mut out = Vec::new();
        let mut buf = [0u8; 65536];
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((n, from)) => {
                    if from.ip() != self.peer.ip() && !self.peer.ip().is_unspecified() {
                        continue;
                    }
                    match decode(&buf[..n]) {
                        Ok((seq, frames)) => {
                            if self.next_rx_seq.is_some_and(|s| s != seq) {
                                self.rx_gaps += 1;
                            }
                            self.next_rx_seq = Some(seq.wrapping_add(1));
                            self.rx_frames += frames.len() as u64;
                            out.extend(frames);
                        }
                        Err(e) => {
                            if self.rx_errors == 0 {
                                log::warn!("cannelloni: datagram from {from}: {e}");
                            }
                            self.rx_errors += 1;
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => {
                    // An ICMP port unreachable from a peer that isn't up yet shows up here on Linux
                    log::debug!("cannelloni: {e}");
                    break;
                }
            }
        }
        out
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
        self.receive_all().into_iter().map(|f| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        self.pending.push(frame.clone());
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
            udp.pending.extend(from_can);
            udp.flush();
        }
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
    use sim_devices::can::{CanBus, Monitor};

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
        assert!(Cannelloni::parse_spec("127.0.0.1:1").is_err());
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
