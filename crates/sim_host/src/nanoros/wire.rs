//! nano-ROS frames: what `common/include/protocol.h` defines and `common/src/nros_stream_parser.c` encodes and
//! parses, on any byte stream (on CAN, the stream of CANmore message payloads: see [`super::transport`]).
//!
//! A frame is a 6-byte header (magic `AA 55`, packet type, topic ID, payload length as a little-endian `u16`), the
//! payload, and a CRC-16/CCITT-FALSE (polynomial 0x1021, initial 0xFFFF) of header and payload, little-endian.

/// The two bytes every frame starts with (`NROS_MAGIC0`, `NROS_MAGIC1`).
pub const MAGIC: [u8; 2] = [0xAA, 0x55];
/// `sizeof(nros_frame_header_t)`.
pub const HEADER_LEN: usize = 6;
/// `NROS_CRC_LEN`.
pub const CRC_LEN: usize = 2;
/// `NROS_BUFFER_SIZE`: the longest payload either side parses (and the firmware's whole encode buffer).
pub const MAX_PAYLOAD: usize = 1024;
/// Topic IDs from here up are the board's publishers, below it its subscribers: nano_ros_codegen numbers
/// subscribers from 0 and publishers from 127, and nano_ros_agent's `PUBLISHER_TOPIC_ID_START` is
/// `UINT8_MAX / 2` (protocol.md says 128; the code says 127).
pub const PUBLISHER_ID_START: u8 = 127;

/// `nros_packet_type_t`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketType {
    /// Board to host: one topic (ID, QoS, type name, topic name); the host answers with an [`PacketType::Ack`].
    Register,
    /// Either way: a message on a topic.
    Data,
    /// Host to board: the registration of the frame's topic ID was taken.
    Ack,
    /// Board to host: every topic is registered.
    RegisterDone,
    /// Host to board: register your topics.
    Discover,
}

impl PacketType {
    pub fn byte(self) -> u8 {
        match self {
            PacketType::Register => 0x01,
            PacketType::Data => 0x02,
            PacketType::Ack => 0x03,
            PacketType::RegisterDone => 0x04,
            PacketType::Discover => 0x05,
        }
    }

    pub fn from_byte(b: u8) -> Option<PacketType> {
        Some(match b {
            0x01 => PacketType::Register,
            0x02 => PacketType::Data,
            0x03 => PacketType::Ack,
            0x04 => PacketType::RegisterDone,
            0x05 => PacketType::Discover,
            _ => return None,
        })
    }
}

/// One frame: its packet type byte (kept raw so an unknown type can be reported), topic ID and payload.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub kind: u8,
    pub topic_id: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    pub fn new(kind: PacketType, topic_id: u8, payload: Vec<u8>) -> Frame {
        Frame { kind: kind.byte(), topic_id, payload }
    }

    pub fn packet_type(&self) -> Option<PacketType> {
        PacketType::from_byte(self.kind)
    }

    /// The frame on the wire (`nros_stream_encode_frame`).
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_LEN + self.payload.len() + CRC_LEN);
        out.extend_from_slice(&MAGIC);
        out.push(self.kind);
        out.push(self.topic_id);
        out.extend_from_slice(&(self.payload.len() as u16).to_le_bytes());
        out.extend_from_slice(&self.payload);
        let crc = crc16(&out);
        out.extend_from_slice(&crc.to_le_bytes());
        out
    }
}

/// CRC-16/CCITT-FALSE, as `crc16_buf`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// A topic registration (`nros_registration_payload_t` and the two strings after it: type name, then topic name).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Registration {
    pub topic_id: u8,
    pub qos: u8,
    pub type_name: String,
    pub topic_name: String,
}

impl Registration {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![self.topic_id, self.qos, self.type_name.len() as u8, self.topic_name.len() as u8];
        out.extend_from_slice(self.type_name.as_bytes());
        out.extend_from_slice(self.topic_name.as_bytes());
        out
    }

    pub fn decode(p: &[u8]) -> Result<Registration, String> {
        let [topic_id, qos, type_len, name_len, rest @ ..] = p else {
            return Err(format!("registration of {} byte(s), shorter than its 4-byte head", p.len()));
        };
        let (t, n) = (*type_len as usize, *name_len as usize);
        if rest.len() != t + n {
            return Err(format!("registration names {t} + {n} byte(s) of strings but carries {}", rest.len()));
        }
        Ok(Registration {
            topic_id: *topic_id,
            qos: *qos,
            type_name: String::from_utf8_lossy(&rest[..t]).into_owned(),
            topic_name: String::from_utf8_lossy(&rest[t..]).into_owned(),
        })
    }
}

/// Finds frames in a byte stream, as `nros_stream_parser_feed` does: hunts for the magic, drops a header whose
/// length is too long and a frame whose CRC does not match, and goes on hunting after either.
#[derive(Clone, Debug, Default)]
pub struct Parser {
    buf: Vec<u8>,
    /// Headers with an impossible length (`resync_count`).
    pub resyncs: u64,
    /// Frames whose CRC did not match (`crc_fail_count`).
    pub crc_failures: u64,
    /// Bytes skipped while hunting for the magic.
    pub skipped: u64,
}

impl Parser {
    pub fn new() -> Parser {
        Parser::default()
    }

    /// Takes in `bytes` and returns the frames they complete.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(bytes);
        let mut out = Vec::new();
        let mut at = 0;
        loop {
            let rest = &self.buf[at..];
            let Some(start) = rest.windows(2).position(|w| w == MAGIC) else {
                // Keep a last byte that may be the first half of the magic
                let keep = usize::from(rest.last() == Some(&MAGIC[0]));
                self.skipped += (rest.len() - keep) as u64;
                at = self.buf.len() - keep;
                break;
            };
            self.skipped += start as u64;
            at += start;
            let rest = &self.buf[at..];
            if rest.len() < HEADER_LEN {
                break;
            }
            let len = u16::from_le_bytes([rest[4], rest[5]]) as usize;
            if len > MAX_PAYLOAD {
                self.resyncs += 1;
                at += 1;
                continue;
            }
            let total = HEADER_LEN + len + CRC_LEN;
            if rest.len() < total {
                break;
            }
            let crc = u16::from_le_bytes([rest[total - 2], rest[total - 1]]);
            if crc != crc16(&rest[..HEADER_LEN + len]) {
                self.crc_failures += 1;
                at += 1;
                continue;
            }
            out.push(Frame { kind: rest[2], topic_id: rest[3], payload: rest[HEADER_LEN..HEADER_LEN + len].to_vec() });
            at += total;
        }
        self.buf.drain(..at);
        out
    }

    /// Bytes held for a frame not complete yet.
    pub fn pending(&self) -> usize {
        self.buf.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_is_ccitt_false() {
        // The standard check value of CRC-16/CCITT-FALSE
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(&[]), 0xFFFF);
    }

    /// A DISCOVER frame as nano_ros_agent sends it: header only, then the CRC of the header.
    #[test]
    fn encodes_a_header_only_frame() {
        let f = Frame::new(PacketType::Discover, 0, Vec::new()).encode();
        let crc = crc16(&[0xAA, 0x55, 0x05, 0x00, 0x00, 0x00]).to_le_bytes();
        assert_eq!(f, [0xAA, 0x55, 0x05, 0x00, 0x00, 0x00, crc[0], crc[1]]);
    }

    #[test]
    fn parses_frames_split_anywhere_and_after_junk() {
        let a = Frame::new(PacketType::Data, 128, vec![0, 1, 0, 0, 7]);
        let b = Frame::new(PacketType::RegisterDone, 0, Vec::new());
        let mut stream = vec![0x13, 0xAA, 0x00];
        stream.extend(a.encode());
        stream.extend(b.encode());
        for split in 0..stream.len() {
            let mut p = Parser::new();
            let mut got = p.feed(&stream[..split]);
            got.extend(p.feed(&stream[split..]));
            assert_eq!(got, [a.clone(), b.clone()], "split at {split}");
            assert_eq!(p.pending(), 0);
            assert_eq!(p.skipped, 3);
        }
    }

    #[test]
    fn drops_bad_crcs_and_impossible_lengths_and_resyncs() {
        let good = Frame::new(PacketType::Data, 130, vec![1, 2, 3]);
        let mut bad = good.encode();
        bad[7] ^= 0xFF;
        let mut stream = bad;
        // A header claiming more than NROS_BUFFER_SIZE
        stream.extend([0xAA, 0x55, 0x02, 0x01, 0x01, 0x04]);
        stream.extend(good.encode());
        let mut p = Parser::new();
        assert_eq!(p.feed(&stream), [good]);
        assert_eq!((p.crc_failures, p.resyncs), (1, 1));
    }

    #[test]
    fn registrations_round_trip_type_name_first() {
        let r = Registration {
            topic_id: 127,
            qos: 0,
            type_name: "riptide_msgs2/msg/FirmwareStatus".into(),
            topic_name: "firmware_status".into(),
        };
        let p = r.encode();
        assert_eq!(&p[..4], [127, 0, 32, 15]);
        assert_eq!(&p[4..8], b"ript");
        assert_eq!(Registration::decode(&p).unwrap(), r);
        assert!(Registration::decode(&p[..10]).unwrap_err().contains("carries 6"));
        assert!(Registration::decode(&[1, 2]).is_err());
    }
}
