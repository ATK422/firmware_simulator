//! How nano-ROS's byte stream goes over CAN: as the payloads of CANmore messages, with the real `canmore` crate.
//!
//! nano_ros has no CAN transport of its own yet (libnanoros's `transport_can.c` is still micro-ROS's and
//! nano_ros_agent's `canmore.cpp` is empty), so this follows what a board has to do with titan_firmware's CAN
//! driver, as micro-ROS over CAN does: each `write` of the stream is one `canbus_msg_write`, a CANmore message of
//! subtype 0 (`MSG_SUBTYPE_XRCE_DDS`, the only one the C driver sends) in classic 8-byte frames (the driver's
//! encoder and decoder are set up without CAN FD), client to agent; the agent answers with agent-to-client
//! messages the board reads with `canbus_msg_read`. The stream framing (magic and CRC) makes the message
//! boundaries irrelevant to the parsers on both sides.

use canmore::id::{Direction, Fields, FrameType};
use canmore::{MsgDecoder, MsgEncoder};
use sim_devices::can::CanFrame;

/// The CANmore message subtype nano-ROS travels in.
pub const SUBTYPE: u8 = canmore::MSG_SUBTYPE_XRCE_DDS;
/// Whether its CANmore frames are CAN FD frames (titan_firmware's C driver: `canmore_msg_encode_init(..., false)`).
pub const USE_CANFD: bool = false;
/// The most stream bytes the agent puts in one CANmore message. libnanoros's `rx_poll` reads 64 bytes at a time,
/// and a transport that reads straight through `canbus_msg_read` would cut a longer message there.
pub const HOST_CHUNK: usize = 64;

/// What came in on a frame for `client`.
#[derive(Debug, PartialEq, Eq)]
pub enum Incoming {
    /// Not this client's, or not to the agent.
    Other,
    /// A CANmore heartbeat or other utility frame of the client.
    Utility(u8),
    /// A frame of a message not complete yet.
    Partial,
    /// A complete message's stream bytes.
    Bytes(Vec<u8>),
    /// A complete message of another subtype (titan_link firmware?).
    WrongSubtype(u8),
    Error(String),
}

/// The CANmore end of one client's stream.
pub struct CanmoreStream {
    pub client_id: u8,
    decoder: MsgDecoder,
}

impl CanmoreStream {
    pub fn new(client_id: u8) -> CanmoreStream {
        CanmoreStream { client_id, decoder: MsgDecoder::new(USE_CANFD) }
    }

    pub fn receive(&mut self, frame: &CanFrame) -> Incoming {
        let f = Fields::decode(frame.id, frame.extended);
        if f.client_id != self.client_id || f.direction != Direction::ClientToAgent {
            return Incoming::Other;
        }
        if f.frame_type == FrameType::Util {
            return Incoming::Utility(f.noc);
        }
        match self.decoder.decode_frame(frame.id, frame.extended, &frame.data) {
            Ok(Some(m)) if m.subtype == SUBTYPE => Incoming::Bytes(m.data.to_vec()),
            Ok(Some(m)) => Incoming::WrongSubtype(m.subtype),
            Ok(None) => Incoming::Partial,
            Err(e) => Incoming::Error(format!("CANmore decode error {e:?}")),
        }
    }

    /// `bytes` of the stream to the board, as CANmore messages of at most [`HOST_CHUNK`] bytes, each its frames.
    pub fn encode(&self, bytes: &[u8]) -> Vec<Vec<CanFrame>> {
        bytes
            .chunks(HOST_CHUNK)
            .map(|chunk| {
                MsgEncoder::new(self.client_id, Direction::AgentToClient, USE_CANFD, SUBTYPE, chunk)
                    .map(|f| CanFrame::classic(f.id, f.extended, f.data()))
                    .collect()
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_through_canmore_in_chunks() {
        let s = CanmoreStream::new(6);
        let bytes: Vec<u8> = (0..150u8).collect();
        let msgs = s.encode(&bytes);
        assert_eq!(msgs.len(), 3, "64 + 64 + 22 bytes");
        assert!(msgs.iter().flatten().all(|f| f.data.len() <= 8), "classic frames");
        // The board's side of it: what it decodes is agent to client
        let f = Fields::decode(msgs[0][0].id, msgs[0][0].extended);
        assert_eq!((f.client_id, f.direction), (6, Direction::AgentToClient));
        let mut d = MsgDecoder::new(false);
        let mut got = Vec::new();
        for fr in msgs.iter().flatten() {
            if let Some(m) = d.decode_frame(fr.id, fr.extended, &fr.data).unwrap() {
                assert_eq!(m.subtype, 0);
                got.extend_from_slice(m.data);
            }
        }
        assert_eq!(got, bytes);
    }

    #[test]
    fn takes_only_its_clients_messages() {
        let mut agent = CanmoreStream::new(6);
        let mine = |client, subtype, data: &[u8]| -> Vec<CanFrame> {
            MsgEncoder::new(client, Direction::ClientToAgent, false, subtype, data)
                .map(|f| CanFrame::classic(f.id, f.extended, f.data()))
                .collect()
        };
        let frames = mine(6, 0, b"0123456789abcdef");
        let results: Vec<Incoming> = frames.iter().map(|f| agent.receive(f)).collect();
        assert_eq!(results.last().unwrap(), &Incoming::Bytes(b"0123456789abcdef".to_vec()));
        assert!(results[..results.len() - 1].iter().all(|r| *r == Incoming::Partial));
        assert_eq!(agent.receive(&mine(7, 0, b"x")[0]), Incoming::Other);
        assert_eq!(agent.receive(&mine(6, 1, b"x")[0]), Incoming::WrongSubtype(1));
        let hb = canmore::Heartbeat::default().frame(6);
        assert_eq!(agent.receive(&CanFrame::classic(hb.id, hb.extended, hb.data())), Incoming::Utility(15));
    }
}
