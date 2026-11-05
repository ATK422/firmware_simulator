//! The C boards' micro-ROS sessions served to a micro-ROS agent over UDP, so no `vcan` and no CAN-capable agent is
//! needed: `micro_ros_agent udp4 --port 8888` (any build of it, the stock one included) serves boards that speak
//! micro-ROS over CAN.
//!
//! titan_firmware's micro-ROS CAN transport (`lib/micro_ros_pico/transport_can`) is packet oriented
//! (`rmw_uros_set_custom_transport(false, ...)`): every XRCE-DDS message is one CANmore message of subtype 0
//! ([`canmore::MSG_SUBTYPE_XRCE_DDS`]), client to agent or agent to client, and the CAN agent (osu-uwrt's
//! Micro-XRCE-DDS-Agent `can-support` branch, `CanAgentLinux.cpp`) does nothing else: it reassembles a message,
//! hands it to the XRCE core as one packet from "client N", and sends each reply as one message to that client.
//! The agent's UDP transport is the same thing with a datagram per packet and the sender's address as the
//! client's endpoint. So this endpoint plays the CAN agent on the bus and, for each client ID it hears micro-ROS
//! from, a UDP client of the real agent, from a socket of its own (one source port per board, as boards on a
//! network would have). Message frames of other subtypes (titan_link, the Rust firmware) are left alone, as are
//! utility frames: heartbeats need no answer, and thruster commands come from whoever sends them.
//!
//! It is a live endpoint, like SocketCAN and cannelloni: what the agent sends is handed in at the bus step that
//! reads it. Use it with `--realtime`, since the agent's session timeouts run on the wall clock.

use std::collections::BTreeMap;
use std::io;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};

use canmore::id::{Direction, Fields, FrameType};
use canmore::{MSG_SUBTYPE_XRCE_DDS, MsgDecoder, MsgEncoder};
use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

/// The micro-ROS agent's usual UDP port (`micro_ros_agent udp4 --port 8888`).
pub const DEFAULT_AGENT_PORT: u16 = 8888;

/// One board's side: its socket to the agent and the reassembly of what it sends.
struct Client {
    socket: UdpSocket,
    decoder: MsgDecoder,
    /// Whether the board's last message came as CAN FD frames; replies go the same way (titan_firmware's C
    /// boards send classic frames).
    fd: bool,
    to_agent: u64,
    from_agent: u64,
}

/// What a board's message frames are reassembled with before it has a socket (or when it speaks titan_link).
struct Pending {
    decoder: MsgDecoder,
    fd: bool,
}

/// A bus endpoint that relays the C boards' micro-ROS sessions to a micro-ROS agent over UDP.
pub struct MicroRosUdp {
    agent: SocketAddr,
    clients: BTreeMap<u8, Client>,
    /// Boards heard on the bus that have not sent micro-ROS yet.
    others: BTreeMap<u8, Pending>,
    /// Frames from the agent noticed by [`CanEndpoint::waiting`], handed in by the next `take_tx`.
    held: Vec<CanFrame>,
    /// CANmore messages that could not be reassembled, and agent datagrams too long for one (1024 bytes).
    pub decode_errors: u64,
    pub oversized: u64,
}

impl MicroRosUdp {
    /// Relays to the agent at `agent`, which may be a host name (`localhost:8888`, `host.docker.internal:8888`).
    pub fn new(agent: &str) -> Result<MicroRosUdp, String> {
        let agent = Self::parse_agent(agent)?;
        Ok(MicroRosUdp {
            agent,
            clients: BTreeMap::new(),
            others: BTreeMap::new(),
            held: Vec::new(),
            decode_errors: 0,
            oversized: 0,
        })
    }

    /// `HOST:PORT` or `HOST` (port 8888), resolved now.
    pub fn parse_agent(spec: &str) -> Result<SocketAddr, String> {
        let spec = spec.trim();
        let resolve = |s: &str| s.to_socket_addrs().ok().and_then(|mut a| a.next());
        resolve(spec)
            .or_else(|| resolve(&format!("{spec}:{DEFAULT_AGENT_PORT}")))
            .ok_or_else(|| format!("micro-ROS agent address {spec:?}: expected HOST:PORT (or HOST, for port 8888)"))
    }

    pub fn agent(&self) -> SocketAddr {
        self.agent
    }

    /// The client IDs with a session relayed, with the messages relayed each way.
    pub fn clients(&self) -> Vec<(u8, u64, u64)> {
        self.clients.iter().map(|(&id, c)| (id, c.to_agent, c.from_agent)).collect()
    }

    /// The local address a board's datagrams come from (what the agent sees as that client).
    pub fn client_addr(&self, client_id: u8) -> Option<SocketAddr> {
        self.clients.get(&client_id).and_then(|c| c.socket.local_addr().ok())
    }

    fn open_socket(&self) -> io::Result<UdpSocket> {
        let any: SocketAddr = if self.agent.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" }.parse().unwrap();
        let socket = UdpSocket::bind(any)?;
        // Connected: only the agent's datagrams are taken, and sending needs no address
        socket.connect(self.agent)?;
        socket.set_nonblocking(true)?;
        Ok(socket)
    }

    /// A message frame from a board.
    fn from_board(&mut self, f: &CanFrame) {
        let fields = Fields::decode(f.id, f.extended);
        let id = fields.client_id;
        // A message's first frame says how it is sent; a decoder in the other mode is replaced
        let (decoder, fd) = match self.clients.get_mut(&id) {
            Some(c) => (&mut c.decoder, &mut c.fd),
            None => {
                let p = self.others.entry(id).or_insert_with(|| Pending { decoder: MsgDecoder::new(f.fd), fd: f.fd });
                (&mut p.decoder, &mut p.fd)
            }
        };
        if fields.noc == 0 && *fd != f.fd {
            *decoder = MsgDecoder::new(f.fd);
            *fd = f.fd;
        }
        let msg = match decoder.decode_frame(f.id, f.extended, &f.data) {
            Ok(Some(m)) if m.subtype == MSG_SUBTYPE_XRCE_DDS => m.data.to_vec(),
            Ok(_) => return,
            Err(e) => {
                if self.decode_errors == 0 {
                    log::warn!("micro-ROS over UDP: client {id}: CANmore decode error {e:?}");
                }
                self.decode_errors += 1;
                return;
            }
        };
        if !self.clients.contains_key(&id) {
            let socket = match self.open_socket() {
                Ok(s) => s,
                Err(e) => {
                    log::warn!("micro-ROS over UDP: client {id}: opening a socket to {}: {e}", self.agent);
                    return;
                }
            };
            let fd = self.others.remove(&id).map(|p| p.fd).unwrap_or(f.fd);
            log::info!(
                "micro-ROS over UDP: client {id} -> agent {} (from {})",
                self.agent,
                socket.local_addr().map(|a| a.to_string()).unwrap_or_default()
            );
            self.clients.insert(id, Client { socket, decoder: MsgDecoder::new(fd), fd, to_agent: 0, from_agent: 0 });
        }
        let c = self.clients.get_mut(&id).unwrap();
        // A connected socket reports an earlier datagram's ICMP error (the agent was not up) on the next call: that
        // one is consumed, so send again once
        let sent = match c.socket.send(&msg) {
            Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => c.socket.send(&msg),
            r => r,
        };
        match sent {
            Ok(_) => c.to_agent += 1,
            // The agent is not up yet: the board retries, as it does on CAN
            Err(e) => log::debug!("micro-ROS over UDP: client {id}: sending to {}: {e}", self.agent),
        }
    }

    /// Every datagram the agent sent, as CANmore frames to its client.
    fn receive_all(&mut self) -> Vec<CanFrame> {
        let mut out = Vec::new();
        let mut buf = vec![0u8; 65536];
        for (&id, c) in &mut self.clients {
            loop {
                match c.socket.recv(&mut buf) {
                    Ok(n) => {
                        if n == 0 {
                            continue;
                        }
                        if n > canmore::MAX_MSG_LENGTH {
                            // The client's MTU (from CREATE_CLIENT) keeps the agent below this
                            if self.oversized == 0 {
                                log::warn!("micro-ROS over UDP: client {id}: {n} byte packet from the agent dropped");
                            }
                            self.oversized += 1;
                            continue;
                        }
                        c.from_agent += 1;
                        let enc = MsgEncoder::new(id, Direction::AgentToClient, c.fd, MSG_SUBTYPE_XRCE_DDS, &buf[..n]);
                        out.extend(enc.map(|fr| {
                            if c.fd {
                                CanFrame::fd(fr.id, fr.extended, fr.data())
                            } else {
                                CanFrame::classic(fr.id, fr.extended, fr.data())
                            }
                        }));
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    // An ICMP error from an agent that is not (yet) listening; it is consumed, so read on
                    Err(e) if matches!(e.kind(), io::ErrorKind::ConnectionRefused | io::ErrorKind::ConnectionReset) => {
                        continue;
                    }
                    Err(e) => {
                        log::debug!("micro-ROS over UDP: client {id}: {e}");
                        break;
                    }
                }
            }
        }
        out
    }
}

impl CanEndpoint for MicroRosUdp {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut frames = std::mem::take(&mut self.held);
        frames.extend(self.receive_all());
        frames.into_iter().map(|f| (until, f)).collect()
    }

    fn receive(&mut self, frame: &CanFrame, _at: Time) {
        if frame.rtr {
            return;
        }
        let fields = Fields::decode(frame.id, frame.extended);
        if fields.frame_type == FrameType::Msg && fields.direction == Direction::ClientToAgent {
            self.from_board(frame);
        }
    }

    /// As a SocketCAN agent's writes: a bounded queue while the bus cannot take them.
    fn tx_queue_limit(&self) -> Option<usize> {
        Some(sim_devices::can::BRIDGE_TX_QUEUE)
    }

    fn live(&self) -> bool {
        true
    }

    fn waiting(&mut self, _until: Time) -> bool {
        let more = self.receive_all();
        self.held.extend(more);
        !self.held.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn frames_of(client: u8, dir: Direction, fd: bool, subtype: u8, msg: &[u8]) -> Vec<CanFrame> {
        MsgEncoder::new(client, dir, fd, subtype, msg)
            .map(|f| if fd { CanFrame::fd(f.id, f.extended, f.data()) } else { CanFrame::classic(f.id, f.extended, f.data()) })
            .collect()
    }

    fn reassemble(frames: &[(Time, CanFrame)]) -> Vec<(u8, Vec<u8>)> {
        let mut d = MsgDecoder::new(false);
        let mut out = Vec::new();
        for (_, f) in frames {
            let fields = Fields::decode(f.id, f.extended);
            assert_eq!(fields.direction, Direction::AgentToClient);
            assert!(!f.fd, "classic frames to a classic board");
            if let Some(m) = d.decode_frame(f.id, f.extended, &f.data).unwrap() {
                assert_eq!(m.subtype, MSG_SUBTYPE_XRCE_DDS);
                out.push((fields.client_id, m.data.to_vec()));
            }
        }
        out
    }

    fn recv_from(agent: &UdpSocket) -> (Vec<u8>, SocketAddr) {
        let mut buf = [0u8; 2048];
        let (n, from) = agent.recv_from(&mut buf).unwrap();
        (buf[..n].to_vec(), from)
    }

    #[test]
    fn agent_addresses() {
        assert_eq!(MicroRosUdp::parse_agent("127.0.0.1:9999").unwrap(), "127.0.0.1:9999".parse().unwrap());
        assert_eq!(MicroRosUdp::parse_agent("127.0.0.1").unwrap(), "127.0.0.1:8888".parse().unwrap());
        assert_eq!(MicroRosUdp::parse_agent("localhost:8888").unwrap().port(), 8888);
        assert_eq!(MicroRosUdp::parse_agent("[::1]:7000").unwrap(), "[::1]:7000".parse().unwrap());
        assert!(MicroRosUdp::parse_agent("127.0.0.1:99999").is_err());
    }

    /// A board's XRCE-DDS packets go to the agent as one datagram each, from a socket of the board's own; the
    /// agent's replies come back to that board as CANmore messages; titan_link messages and utility frames are
    /// left alone.
    #[test]
    fn sessions_are_relayed_per_client() {
        let agent = UdpSocket::bind("127.0.0.1:0").unwrap();
        agent.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let mut gw = MicroRosUdp::new(&agent.local_addr().unwrap().to_string()).unwrap();
        assert!(gw.live() && !gw.waiting(Time::ZERO));
        // CREATE_CLIENT-sized packets (more than one classic frame) from boards 1 and 4
        let create: Vec<u8> = (0..40).collect();
        for f in frames_of(1, Direction::ClientToAgent, false, MSG_SUBTYPE_XRCE_DDS, &create) {
            gw.receive(&f, Time::ZERO);
        }
        let (p, from1) = recv_from(&agent);
        assert_eq!(p, create);
        for f in frames_of(4, Direction::ClientToAgent, false, MSG_SUBTYPE_XRCE_DDS, b"\x81\x00\x00\x00") {
            gw.receive(&f, Time::ZERO);
        }
        let (p, from4) = recv_from(&agent);
        assert_eq!(p, b"\x81\x00\x00\x00");
        assert_ne!(from1, from4, "each board is its own client to the agent");
        assert_eq!(gw.client_addr(1), Some(from1));
        // Not micro-ROS: a Rust board's titan_link message, a heartbeat, the agent's own direction
        for f in frames_of(2, Direction::ClientToAgent, true, canmore::MSG_SUBTYPE_TITAN_LINK, &[1; 30]) {
            gw.receive(&f, Time::ZERO);
        }
        gw.receive(&CanFrame::classic(canmore::id::util_id(2, Direction::ClientToAgent, 15), false, &[1]), Time::ZERO);
        for f in frames_of(3, Direction::AgentToClient, false, MSG_SUBTYPE_XRCE_DDS, &[2; 9]) {
            gw.receive(&f, Time::ZERO);
        }
        assert_eq!(gw.clients().iter().map(|c| c.0).collect::<Vec<_>>(), [1, 4]);
        // The agent answers each board at the address it came from
        let reply: Vec<u8> = (100..200).collect();
        agent.send_to(&reply, from4).unwrap();
        agent.send_to(b"ack", from1).unwrap();
        std::thread::sleep(Duration::from_millis(50));
        assert!(gw.waiting(Time::from_us(10)));
        let at = Time::from_us(10);
        let tx = gw.take_tx(at);
        assert!(tx.iter().all(|(t, _)| *t == at));
        let mut got = reassemble(&tx);
        got.sort();
        assert_eq!(got, [(1, b"ack".to_vec()), (4, reply)]);
        assert_eq!(gw.clients(), [(1, 1, 1), (4, 1, 1)]);
        assert!(gw.take_tx(at).is_empty());
        assert_eq!((gw.decode_errors, gw.oversized), (0, 0));
    }

    /// A broken message is counted and dropped, the next one goes through; an agent that is not up yet loses the
    /// packet (the board retries) without stopping the relay; a packet too long for CANmore is dropped.
    #[test]
    fn errors_are_counted() {
        let agent = UdpSocket::bind("127.0.0.1:0").unwrap();
        let addr = agent.local_addr().unwrap();
        drop(agent);
        let mut gw = MicroRosUdp::new(&addr.to_string()).unwrap();
        let msg: Vec<u8> = (0..20).collect();
        let mut frames = frames_of(5, Direction::ClientToAgent, false, MSG_SUBTYPE_XRCE_DDS, &msg);
        frames[1].data[0] ^= 1;
        for f in &frames {
            gw.receive(f, Time::ZERO);
        }
        assert_eq!(gw.decode_errors, 1);
        assert!(gw.clients().is_empty());
        for f in frames_of(5, Direction::ClientToAgent, false, MSG_SUBTYPE_XRCE_DDS, &msg) {
            gw.receive(&f, Time::ZERO);
        }
        std::thread::sleep(Duration::from_millis(20));
        assert!(gw.take_tx(Time::ZERO).is_empty(), "nobody listening: nothing back, nothing stuck");
        let agent = UdpSocket::bind(addr).unwrap();
        agent.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        for f in frames_of(5, Direction::ClientToAgent, false, MSG_SUBTYPE_XRCE_DDS, &msg) {
            gw.receive(&f, Time::ZERO);
        }
        let (p, from) = recv_from(&agent);
        assert_eq!(p, msg);
        agent.send_to(&[0u8; 1500], from).unwrap();
        agent.send_to(&[7u8; 3], from).unwrap();
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(reassemble(&gw.take_tx(Time::ZERO)), [(5, vec![7u8; 3])]);
        assert_eq!(gw.oversized, 1);
        assert_eq!(gw.clients(), [(5, 2, 1)]);
    }
}
