//! Fuzz inputs: a deterministic, seeded, structure-aware generator and mutator for the traffic a board sees.
//!
//! A [`FuzzInput`] is a list of [`Op`]s, each a thing to do to the board's CAN bus at a virtual time offset from
//! the start of the fuzz window. Generation mixes structure-aware cases (valid CANmore framing to the board's
//! client with mutated `titan_link` payloads, register-mapped control-interface writes with wild addresses) with
//! raw mutation (arbitrary identifiers, lengths, FD flags and data). Everything is a pure function of a seed and
//! the generator's parameters, so a case reproduces exactly, and mutation keeps that: a corpus entry is its op
//! list, and the reproducer writes the op list back out as a scenario.

use canmore::id::{self, Direction, FrameType};
use canmore::{MSG_SUBTYPE_TITAN_LINK, MsgEncoder};
use sim_devices::can::{CanFrame, fd_dlc_to_len, fd_len_to_dlc};
use sim_host::fault::Fault;
use titan_link::{HostToDevice, PROTOCOL_VERSION};

/// The `--fault` spelling of a fault the generator produces.
pub fn fault_spec(f: &Fault) -> String {
    match f {
        Fault::CanErrorRate(p) => format!("can-error-rate:{p}"),
        Fault::CanCorrupt(n) => format!("can-corrupt:{n}"),
        Fault::CanStuckDominant(true) => "can-stuck".into(),
        Fault::CanStuckDominant(false) => "can-release".into(),
        Fault::CanUnplugged(true) => "can-unplug".into(),
        Fault::CanUnplugged(false) => "can-plug".into(),
        Fault::CanFlood { id, len, load, .. } => format!("can-flood:0x{id:x}:{load}:{len}"),
        Fault::CanFloodStop => "can-flood-stop".into(),
        Fault::KillSwitch(Some(true)) => "kill:on".into(),
        Fault::KillSwitch(Some(false)) => "kill:off".into(),
        Fault::KillSwitch(None) => "kill:none".into(),
        other => format!("{other:?}"),
    }
}

/// serde for the `Fault` inside an `Op`, as its `--fault` spelling (Fault itself is not `Serialize`).
mod fault_serde {
    use super::Fault;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(f: &Fault, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&super::fault_spec(f))
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Fault, D::Error> {
        let spec = String::deserialize(d)?;
        spec.parse().map_err(serde::de::Error::custom)
    }
}

/// A small, fast, deterministic PRNG (SplitMix64). Seeded runs reproduce.
#[derive(Clone, Debug)]
pub struct Rng(pub u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_add(0x9E37_79B9_7F4A_7C15))
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn below(&mut self, n: u32) -> u32 {
        if n == 0 { 0 } else { (self.next_u64() % n as u64) as u32 }
    }

    pub fn range(&mut self, lo: u32, hi: u32) -> u32 {
        lo + self.below(hi - lo + 1)
    }

    pub fn chance(&mut self, num: u32, den: u32) -> bool {
        self.below(den) < num
    }

    pub fn pick<'a, T>(&mut self, xs: &'a [T]) -> &'a T {
        &xs[self.below(xs.len() as u32) as usize]
    }

    /// `n` bytes of noise.
    pub fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_u64() as u8).collect()
    }
}

/// One thing done to the board's bus at `at_us` microseconds into the fuzz window.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Op {
    /// A raw CAN (or CAN FD) frame put on the bus, as a babbling node or adversary would (`cansend`).
    Raw { at_us: u32, id: u32, extended: bool, fd: bool, brs: bool, data: Vec<u8> },
    /// Bytes carried to the board's client as one CANmore `titan_link` message (the bridge's encoder frames them,
    /// so the firmware's `titan_link` decoder sees them whatever they are).
    Encoded { at_us: u32, bytes: Vec<u8> },
    /// A well-formed `titan_link` message from the bridge (the structured path).
    Host { at_us: u32, msg: HostToDevice },
    /// A bus condition.
    Fault {
        at_us: u32,
        #[serde(with = "fault_serde")]
        fault: Fault,
    },
}

impl Op {
    pub fn at_us(&self) -> u32 {
        match self {
            Op::Raw { at_us, .. } | Op::Encoded { at_us, .. } | Op::Host { at_us, .. } | Op::Fault { at_us, .. } => {
                *at_us
            }
        }
    }

    fn set_at(&mut self, t: u32) {
        match self {
            Op::Raw { at_us, .. } | Op::Encoded { at_us, .. } | Op::Host { at_us, .. } | Op::Fault { at_us, .. } => {
                *at_us = t
            }
        }
    }
}

/// A fuzz case: a seeded, ordered list of operations on one board's bus.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FuzzInput {
    pub ops: Vec<Op>,
}

/// What the generator needs to know about the board under test.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// The board's CANmore client ID.
    pub client: u8,
    /// Whether the bus is CAN FD.
    pub fd: bool,
    /// The fuzz window, in microseconds (op times fall inside it).
    pub window_us: u32,
    /// Largest number of ops in a generated case.
    pub max_ops: u32,
}

impl FuzzInput {
    /// Sorts the ops by time and clamps their times into the window.
    pub fn normalize(&mut self, window_us: u32) {
        for op in &mut self.ops {
            op.set_at(op.at_us().min(window_us.saturating_sub(1)));
        }
        self.ops.sort_by_key(Op::at_us);
    }

    /// A fresh case generated from `seed`.
    pub fn generate(seed: u64, p: Params) -> FuzzInput {
        let mut rng = Rng::new(seed);
        let n = rng.range(1, p.max_ops.max(1));
        let mut input = FuzzInput { ops: (0..n).map(|_| gen_op(&mut rng, p)).collect() };
        input.normalize(p.window_us);
        input
    }
}

/// A well-formed CANmore identifier for a message frame to the board's client (sequence `noc`, the first frame
/// extended and carrying `len`/`subtype`, later frames standard, a final extended frame carrying a CRC in `extra`).
fn canmore_msg_id(client: u8, noc: u8, first_extra: u32, last_crc: Option<u32>) -> (u32, bool) {
    if noc == 0 {
        (id::ext_id(client, FrameType::Msg, Direction::AgentToClient, 0, first_extra), true)
    } else if let Some(crc) = last_crc {
        (id::ext_id(client, FrameType::Msg, Direction::AgentToClient, noc, crc), true)
    } else {
        (id::std_id(client, FrameType::Msg, Direction::AgentToClient, noc), false)
    }
}

/// A valid-ish `titan_link` message to flip bytes of, so `Encoded` ops often start from something the decoder
/// accepts and explore nearby. Pure noise is covered separately.
fn sample_encoded(rng: &mut Rng) -> Vec<u8> {
    let msgs = [
        HostToDevice::Ping { protocol_version: PROTOCOL_VERSION },
        HostToDevice::Ping { protocol_version: rng.next_u64() as u8 },
        HostToDevice::KillSwitch { asserting_kill: rng.chance(1, 2) },
        HostToDevice::Power(titan_link::power::HostToDevice::ElectricalCommand(rng.next_u64() as u8)),
        HostToDevice::Esc(titan_link::esc::HostToDevice::ThrusterRpm([rng.next_u64() as i16; 8])),
        HostToDevice::CameraCage(titan_link::camera_cage::HostToDevice::ElectricalCommand(rng.next_u64() as u8)),
    ];
    let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
    let mut bytes = titan_link::encode(rng.pick(&msgs), &mut buf).map(|b| b.to_vec()).unwrap_or_default();
    // Flip a few bytes and maybe grow or shrink it
    let flips = rng.below(4);
    for _ in 0..flips {
        if !bytes.is_empty() {
            let i = rng.below(bytes.len() as u32) as usize;
            bytes[i] ^= 1 << rng.below(8);
        }
    }
    match rng.below(4) {
        0 => {
            let n = rng.below(8) as usize;
            bytes.extend(rng.bytes(n));
        }
        1 if !bytes.is_empty() => bytes.truncate(rng.below(bytes.len() as u32) as usize),
        _ => {}
    }
    bytes
}

/// One random operation, weighted towards structure-aware cases.
fn gen_op(rng: &mut Rng, p: Params) -> Op {
    let at_us = rng.below(p.window_us);
    match rng.below(100) {
        // A well-formed `titan_link` message (structured path)
        0..=14 => Op::Host { at_us, msg: HostToDevice::KillSwitch { asserting_kill: rng.chance(1, 2) } },
        15..=22 => Op::Host { at_us, msg: HostToDevice::Ping { protocol_version: rng.next_u64() as u8 } },
        // Encoded titan_link bytes: a mutated valid message, or pure noise
        23..=45 => Op::Encoded { at_us, bytes: sample_encoded(rng) },
        46..=55 => {
            let n = rng.range(1, 40) as usize;
            Op::Encoded { at_us, bytes: rng.bytes(n) }
        }
        // A raw CANmore message frame to our client with a mutated field (framing edge cases)
        56..=73 => gen_canmore_raw(rng, p, at_us),
        // Register-mapped control interface write (channel 14) with wild page/offset
        74..=82 => gen_reg_mapped(rng, p, at_us),
        // Pure raw frame: arbitrary id, FD flags, length, data; sometimes to another client
        83..=93 => gen_pure_raw(rng, p, at_us),
        // A bus condition
        _ => Op::Fault { at_us, fault: gen_fault(rng, p) },
    }
}

fn gen_canmore_raw(rng: &mut Rng, p: Params, at_us: u32) -> Op {
    let fd = p.fd && rng.chance(3, 4);
    // Choose a sequence number, length and payload, then mutate one of them to make it malformed
    let noc = rng.below(17) as u8;
    let len = rng.range(1, if fd { 1024 } else { 200 });
    let subtype = if rng.chance(3, 4) { MSG_SUBTYPE_TITAN_LINK } else { rng.below(64) as u8 };
    let single = rng.chance(1, 2);
    let first_extra = id::msg_first_extra(single, len as usize, subtype);
    let crc = (noc != 0 && rng.chance(1, 2)).then(|| rng.below(0x3FFFF));
    let (id, extended) = canmore_msg_id(p.client, noc, first_extra, crc);
    // Payload: often truncated or oversized versus what the length field claims
    let dlen = if fd { rng.below(65) } else { rng.below(9) };
    let data = rng.bytes(dlen as usize);
    Op::Raw { at_us, id, extended, fd, brs: fd && rng.chance(1, 2), data }
}

fn gen_reg_mapped(rng: &mut Rng, p: Params, at_us: u32) -> Op {
    // Control interface is a classic utility frame on channel 14. A register-mapped request is
    // flags|count|page|offset[|data...]; wild page/offset probe the server's address handling.
    let id = id::util_id(p.client, Direction::AgentToClient, canmore::CHAN_CONTROL_INTERFACE);
    let flags = rng.next_u64() as u8;
    let count = rng.next_u64() as u8;
    let page = rng.next_u64() as u8;
    let offset = rng.next_u64() as u8;
    let mut data = vec![flags, count, page, offset];
    if flags & 1 != 0 {
        data.extend(rng.bytes(4)); // a write word
    }
    if rng.chance(1, 3) {
        let n = rng.below(4) as usize; // a ragged tail
        data.extend(rng.bytes(n));
    }
    data.truncate(8);
    Op::Raw { at_us, id, extended: false, fd: false, brs: false, data }
}

fn gen_pure_raw(rng: &mut Rng, p: Params, at_us: u32) -> Op {
    let extended = rng.chance(1, 2);
    let id = if extended { rng.next_u64() as u32 & 0x1FFF_FFFF } else { rng.below(0x800) };
    // Sometimes address our own client, sometimes another
    let id = if rng.chance(1, 3) {
        let noc = rng.below(16) as u8;
        if extended {
            id::ext_id(p.client, FrameType::Msg, Direction::AgentToClient, noc, rng.below(0x3FFFF))
        } else {
            id::std_id(p.client, FrameType::Msg, Direction::AgentToClient, noc)
        }
    } else {
        id
    };
    let fd = p.fd && rng.chance(1, 2);
    let maxlen = if fd { 64 } else { 8 };
    let len = rng.below(maxlen + 1);
    let mut data = rng.bytes(len as usize);
    if fd {
        data.resize(fd_dlc_to_len(fd_len_to_dlc(data.len())), 0);
    }
    Op::Raw { at_us, id, extended, fd, brs: fd && rng.chance(1, 2), data }
}

fn gen_fault(rng: &mut Rng, p: Params) -> Fault {
    match rng.below(7) {
        0 => Fault::CanErrorRate(0.1 + 0.9 * (rng.below(100) as f64 / 100.0)),
        1 => Fault::CanCorrupt(rng.range(1, 20)),
        2 => Fault::CanStuckDominant(true),
        3 => Fault::CanUnplugged(true),
        4 => {
            let id = if rng.chance(1, 2) { rng.below(0x800) } else { rng.next_u64() as u32 & 0x1FFF_FFFF };
            Fault::CanFlood {
                id,
                extended: id > 0x7FF,
                len: rng.range(0, if p.fd { 64 } else { 8 }) as usize,
                load: 0.2 + 0.8 * (rng.below(100) as f64 / 100.0),
            }
        }
        5 => Fault::KillSwitch(Some(rng.chance(1, 2))),
        _ => Fault::KillSwitch(None),
    }
}

/// Builds the CAN frames an `Encoded` op puts on the bus: the bytes framed as a CANmore `titan_link` message to
/// the client. Returns an empty list if the bytes do not fit a message.
pub fn encoded_frames(client: u8, fd: bool, bytes: &[u8]) -> Vec<CanFrame> {
    MsgEncoder::new(client, Direction::AgentToClient, fd, MSG_SUBTYPE_TITAN_LINK, bytes)
        .map(|f| {
            if fd {
                let mut d = f.data().to_vec();
                d.resize(fd_dlc_to_len(fd_len_to_dlc(d.len())), 0);
                CanFrame::fd(f.id, f.extended, &d)
            } else {
                CanFrame::classic(f.id, f.extended, f.data())
            }
        })
        .collect()
}

// ---- Mutation (corpus-guided)

/// Mutates `input` in place from `rng`: tweak, add, drop, splice. Kept normalized.
pub fn mutate(input: &mut FuzzInput, rng: &mut Rng, p: Params, corpus: &[FuzzInput]) {
    let rounds = rng.range(1, 4);
    for _ in 0..rounds {
        if input.ops.is_empty() {
            input.ops.push(gen_op(rng, p));
            continue;
        }
        match rng.below(100) {
            0..=29 => {
                // Mutate one op's bytes/fields
                let i = rng.below(input.ops.len() as u32) as usize;
                mutate_op(&mut input.ops[i], rng, p);
            }
            30..=49 => input.ops.push(gen_op(rng, p)),
            50..=64 if input.ops.len() > 1 => {
                let i = rng.below(input.ops.len() as u32) as usize;
                input.ops.remove(i);
            }
            65..=79 => {
                // Splice a run of ops from a corpus entry
                if let Some(donor) = corpus.iter().find(|c| !c.ops.is_empty()) {
                    let k = rng.range(0, donor.ops.len() as u32 - 1) as usize;
                    input.ops.push(donor.ops[k].clone());
                } else {
                    input.ops.push(gen_op(rng, p));
                }
            }
            80..=89 => {
                // Duplicate an op (bursts / repeats)
                let i = rng.below(input.ops.len() as u32) as usize;
                let mut d = input.ops[i].clone();
                d.set_at(rng.below(p.window_us));
                input.ops.push(d);
            }
            _ => {
                // Retime an op
                let i = rng.below(input.ops.len() as u32) as usize;
                input.ops[i].set_at(rng.below(p.window_us));
            }
        }
    }
    // Keep cases from exploding
    if input.ops.len() > (p.max_ops * 2) as usize {
        input.ops.truncate((p.max_ops * 2) as usize);
    }
    input.normalize(p.window_us);
}

fn mutate_op(op: &mut Op, rng: &mut Rng, p: Params) {
    match op {
        Op::Raw { id, data, extended, fd, brs, .. } => match rng.below(5) {
            0 => *id ^= 1 << rng.below(29),
            1 => *extended = !*extended,
            2 if p.fd => *fd = !*fd,
            3 => *brs = !*brs,
            _ => flip_bytes(data, rng),
        },
        Op::Encoded { bytes, .. } => {
            if rng.chance(1, 3) {
                *bytes = sample_encoded(rng);
            } else {
                flip_bytes(bytes, rng);
            }
        }
        Op::Host { msg, .. } => *msg = HostToDevice::KillSwitch { asserting_kill: rng.chance(1, 2) },
        Op::Fault { fault, .. } => *fault = gen_fault(rng, p),
    }
}

fn flip_bytes(data: &mut Vec<u8>, rng: &mut Rng) {
    match rng.below(4) {
        0 if !data.is_empty() => {
            let i = rng.below(data.len() as u32) as usize;
            data[i] ^= 1 << rng.below(8);
        }
        1 => data.push(rng.next_u64() as u8),
        2 if !data.is_empty() => {
            data.truncate(data.len() - 1);
        }
        _ if !data.is_empty() => {
            let i = rng.below(data.len() as u32) as usize;
            data[i] = rng.next_u64() as u8;
        }
        _ => data.push(rng.next_u64() as u8),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: Params = Params { client: 1, fd: true, window_us: 200_000, max_ops: 12 };

    #[test]
    fn generation_is_deterministic_and_in_window() {
        let a = FuzzInput::generate(42, P);
        let b = FuzzInput::generate(42, P);
        assert_eq!(a, b, "same seed, same case");
        assert_ne!(FuzzInput::generate(43, P), a);
        for op in &a.ops {
            assert!(op.at_us() < P.window_us);
        }
        assert!(a.ops.windows(2).all(|w| w[0].at_us() <= w[1].at_us()), "sorted by time");
        assert!(!a.ops.is_empty() && a.ops.len() <= P.max_ops as usize);
    }

    #[test]
    fn mutation_stays_valid_and_deterministic() {
        let base = FuzzInput::generate(7, P);
        let run = || {
            let mut input = base.clone();
            let mut rng = Rng::new(99);
            for _ in 0..50 {
                mutate(&mut input, &mut rng, P, std::slice::from_ref(&base));
                input.normalize(P.window_us);
                assert!(input.ops.windows(2).all(|w| w[0].at_us() <= w[1].at_us()));
                assert!(input.ops.iter().all(|o| o.at_us() < P.window_us));
            }
            input
        };
        assert_eq!(run(), run(), "mutation is a pure function of the seed");
    }

    #[test]
    fn encoded_frames_round_trip_through_the_decoder() {
        let bytes: Vec<u8> = (0..50).collect();
        let frames = encoded_frames(8, false, &bytes);
        assert!(frames.len() > 1, "fragmented over classic frames");
        let mut d = canmore::MsgDecoder::new(false);
        let mut got = None;
        for f in &frames {
            if let Some(m) = d.decode_frame(f.id, f.extended, &f.data).unwrap() {
                got = Some((m.subtype, m.data.to_vec()));
            }
        }
        assert_eq!(got, Some((MSG_SUBTYPE_TITAN_LINK, bytes)));
    }
}
