//! A scripted stand-in for `titan_bridge`, for checking a board's CAN link without ROS.
//!
//! It speaks the bridge's side of the protocol with the real `canmore` and `titan_link` crates: CANmore message
//! frames of subtype `MSG_SUBTYPE_TITAN_LINK` to and from one client, a `HostToDevice::Ping` every
//! `titan_link::HOST_PING_INTERVAL_MS`, and the `state/kill` forwarding as `HostToDevice::KillSwitch`. It records
//! what the board sends (titan_link messages, CANmore heartbeats, utility frames). Stopping it is how a test
//! simulates the bridge going away.

use std::sync::{Arc, Mutex};

use canmore::id::{Direction, Fields, FrameType};
use canmore::{Heartbeat, MsgDecoder, MsgEncoder};
use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};
use titan_link::{DeviceToHost, HostToDevice};

pub struct BridgeState {
    pub client_id: u8,
    pub fd: bool,
    /// Whether the bridge is running (pinging and forwarding).
    pub running: bool,
    /// The `state/kill` value to forward with every ping (`Some(false)` enables the vehicle), or nothing.
    pub kill: Option<bool>,
    /// titan_link messages from the board.
    pub received: Vec<(Time, DeviceToHost)>,
    /// CANmore heartbeats from the board.
    pub heartbeats: Vec<(Time, Heartbeat)>,
    /// Utility frames from the board other than the heartbeat.
    pub utility: Vec<(Time, u8, Vec<u8>)>,
    /// Problems decoding what the board sent.
    pub errors: Vec<String>,
    /// Messages to send once, at the next opportunity.
    pub pending: Vec<HostToDevice>,
    /// Pings sent, and when the last one was.
    pub pings: u64,
    pub last_ping: Option<Time>,
    next_ping: Time,
    decoder: MsgDecoder,
}

/// A handle to a mock bridge; clone it to keep looking at what the bridge saw after handing it to a session.
#[derive(Clone)]
pub struct MockBridge(pub Arc<Mutex<BridgeState>>);

impl MockBridge {
    /// A bridge for `client_id` on a CAN FD (`fd`) or classic bus, starting to ping at `start`.
    pub fn new(client_id: u8, fd: bool, start: Time) -> Self {
        MockBridge(Arc::new(Mutex::new(BridgeState {
            client_id,
            fd,
            running: true,
            kill: None,
            received: Vec::new(),
            heartbeats: Vec::new(),
            utility: Vec::new(),
            errors: Vec::new(),
            pending: Vec::new(),
            pings: 0,
            last_ping: None,
            next_ping: start,
            decoder: MsgDecoder::new(fd),
        })))
    }

    pub fn state(&self) -> std::sync::MutexGuard<'_, BridgeState> {
        self.0.lock().unwrap()
    }

    /// Stops pinging and forwarding, like killing the `titan_bridge` process.
    pub fn stop(&self) {
        self.state().running = false;
    }

    /// Starts again (pinging from `at`).
    pub fn start(&self, at: Time) {
        let mut s = self.state();
        s.running = true;
        s.next_ping = at;
    }

    pub fn set_kill(&self, kill: Option<bool>) {
        self.state().kill = kill;
    }

    /// An independent bridge with a copy of this one's state (for a snapshot of the session it is attached to). A
    /// message the board was in the middle of sending is not carried over.
    pub fn fork(&self) -> Self {
        let s = self.state();
        MockBridge(Arc::new(Mutex::new(BridgeState {
            client_id: s.client_id,
            fd: s.fd,
            running: s.running,
            kill: s.kill,
            received: s.received.clone(),
            heartbeats: s.heartbeats.clone(),
            utility: s.utility.clone(),
            errors: s.errors.clone(),
            pending: s.pending.clone(),
            pings: s.pings,
            last_ping: s.last_ping,
            next_ping: s.next_ping,
            decoder: MsgDecoder::new(s.fd),
        })))
    }

    pub fn send(&self, msg: HostToDevice) {
        self.state().pending.push(msg);
    }

    /// Firmware status messages received.
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

    /// titan_link heartbeats received.
    pub fn link_heartbeats(&self) -> Vec<Time> {
        self.state().received.iter().filter_map(|(t, m)| matches!(m, DeviceToHost::Heartbeat).then_some(*t)).collect()
    }
}

fn frames_for(state: &BridgeState, msg: &HostToDevice, at: Time) -> Vec<(Time, CanFrame)> {
    let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
    let Ok(encoded) = titan_link::encode(msg, &mut buf) else { return Vec::new() };
    MsgEncoder::new(state.client_id, Direction::AgentToClient, state.fd, canmore::MSG_SUBTYPE_TITAN_LINK, encoded)
        .map(|f| {
            let frame = if state.fd {
                // Pad to a valid FD length like SocketCAN does
                let mut data = f.data().to_vec();
                data.resize(sim_devices::can::fd_dlc_to_len(sim_devices::can::fd_len_to_dlc(data.len())), 0);
                CanFrame::fd(f.id, f.extended, &data)
            } else {
                CanFrame::classic(f.id, f.extended, f.data())
            };
            (at, frame)
        })
        .collect()
}

impl CanEndpoint for MockBridge {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut s = self.state();
        let mut out = Vec::new();
        if !s.running {
            return out;
        }
        for msg in std::mem::take(&mut s.pending) {
            let at = s.next_ping.min(until);
            out.extend(frames_for(&s, &msg, at));
        }
        while s.next_ping <= until {
            let at = s.next_ping;
            let ping = HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION };
            out.extend(frames_for(&s, &ping, at));
            if let Some(kill) = s.kill {
                out.extend(frames_for(&s, &HostToDevice::KillSwitch { asserting_kill: kill }, at));
            }
            s.pings += 1;
            s.last_ping = Some(at);
            s.next_ping = at + Time::from_ms(titan_link::HOST_PING_INTERVAL_MS);
        }
        out
    }

    /// `titan_bridge` writes to its SocketCAN interface and drops a frame the interface refuses (`ENOBUFS`), so
    /// pings do not pile up while the bus is unusable.
    fn tx_queue_limit(&self) -> Option<usize> {
        Some(sim_devices::can::ENDPOINT_TX_QUEUE)
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        let mut s = self.state();
        let fields = Fields::decode(frame.id, frame.extended);
        if fields.client_id != s.client_id || fields.direction != Direction::ClientToAgent {
            return;
        }
        if fields.frame_type == FrameType::Util {
            match (fields.noc, frame.data.as_slice()) {
                (canmore::CHAN_HEARTBEAT, [b]) => s.heartbeats.push((at, Heartbeat::from_byte(*b))),
                (ch, data) => s.utility.push((at, ch, data.to_vec())),
            }
            return;
        }
        let decoded = s
            .decoder
            .decode_frame(frame.id, frame.extended, &frame.data)
            .map(|m| m.map(|m| (m.subtype, titan_link::decode::<DeviceToHost>(m.data).map_err(|_| m.data.to_vec()))));
        match decoded {
            Ok(Some((canmore::MSG_SUBTYPE_TITAN_LINK, Ok(msg)))) => s.received.push((at, msg)),
            Ok(Some((sub, Ok(_)))) | Ok(Some((sub, Err(_)))) if sub != canmore::MSG_SUBTYPE_TITAN_LINK => {
                s.errors.push(format!("CANmore message of subtype {sub} (micro-ROS firmware?)"))
            }
            Ok(Some((_, Err(data)))) => s.errors.push(format!("undecodable titan_link message {data:02x?}")),
            Ok(_) => {}
            Err(e) => s.errors.push(format!("CANmore decode error {e:?}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pings_at_the_bridge_interval_and_stops() {
        let mut b = MockBridge::new(1, true, Time::ZERO);
        b.set_kill(Some(false));
        let frames = b.take_tx(Time::from_ms(250));
        // Pings at 0, 100, 200 ms, each followed by a kill switch message; each fits one FD frame
        assert_eq!(frames.len(), 6);
        assert_eq!(frames[2].0, Time::from_ms(100));
        let f = Fields::decode(frames[0].1.id, frames[0].1.extended);
        assert_eq!((f.client_id, f.direction, f.frame_type), (1, Direction::AgentToClient, FrameType::Msg));
        b.stop();
        assert!(b.take_tx(Time::from_ms(1000)).is_empty());
    }

    #[test]
    fn decodes_what_a_board_sends() {
        let mut b = MockBridge::new(1, true, Time::ZERO);
        // A board's titan_link heartbeat and its CANmore heartbeat
        let mut buf = [0u8; 64];
        let enc = titan_link::encode(&DeviceToHost::Heartbeat, &mut buf).unwrap();
        for f in MsgEncoder::new(1, Direction::ClientToAgent, true, canmore::MSG_SUBTYPE_TITAN_LINK, enc) {
            b.receive(&CanFrame::fd(f.id, f.extended, f.data()), Time::from_ms(5));
        }
        let hb = Heartbeat { cnt: 1, error: true, mode: 0, term_valid: true, term_enabled: true }.frame(1);
        b.receive(&CanFrame::classic(hb.id, hb.extended, hb.data()), Time::from_ms(6));
        // Frames of another client are ignored
        let other = Heartbeat::default().frame(2);
        b.receive(&CanFrame::classic(other.id, other.extended, other.data()), Time::from_ms(7));
        assert_eq!(b.link_heartbeats(), vec![Time::from_ms(5)]);
        let s = b.state();
        assert_eq!(s.heartbeats.len(), 1);
        assert!(s.heartbeats[0].1.error);
        assert!(s.errors.is_empty());
    }
}
