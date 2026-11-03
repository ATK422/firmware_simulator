//! CAN frames and an in-process CAN bus.
//!
//! The bus serialises frames with arbitration (lowest identifier wins among requests pending when the bus becomes
//! free), charges each frame its on-wire duration at the bus's nominal and data bit rates, acknowledges a frame only
//! if another active node or a host endpoint is listening, and delivers it to everyone else when its last bit is
//! sent. It is stepped by whoever owns the machines (a single board's session, or a robot of many boards) after
//! they have all run to the same time, which keeps multi-board runs deterministic.

use std::collections::VecDeque;

use sim_core::time::Time;

/// One CAN or CAN FD frame.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct CanFrame {
    /// 11-bit or 29-bit identifier.
    pub id: u32,
    pub extended: bool,
    /// CAN FD frame (FDF).
    pub fd: bool,
    /// Bit rate switch (data phase at the data bit rate).
    pub brs: bool,
    /// Remote frame (classic only).
    pub rtr: bool,
    pub esi: bool,
    pub data: Vec<u8>,
}

/// CAN FD payload length for a DLC.
pub fn fd_dlc_to_len(dlc: u8) -> usize {
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64][(dlc & 0xF) as usize]
}

/// Smallest DLC that holds `len` bytes in CAN FD.
pub fn fd_len_to_dlc(len: usize) -> u8 {
    (0..16u8).find(|&d| fd_dlc_to_len(d) >= len).unwrap_or(15)
}

impl CanFrame {
    pub fn classic(id: u32, extended: bool, data: &[u8]) -> CanFrame {
        CanFrame { id, extended, fd: false, brs: false, rtr: false, esi: false, data: data.to_vec() }
    }

    pub fn fd(id: u32, extended: bool, data: &[u8]) -> CanFrame {
        CanFrame { id, extended, fd: true, brs: true, rtr: false, esi: false, data: data.to_vec() }
    }

    pub fn dlc(&self) -> u8 {
        if self.fd { fd_len_to_dlc(self.data.len()) } else { self.data.len().min(8) as u8 }
    }

    /// Key used in arbitration: the base identifier, then IDE (standard wins), then the extension.
    pub fn arbitration_key(&self) -> (u32, bool, u32) {
        if self.extended { (self.id >> 18, true, self.id & 0x3FFFF) } else { (self.id & 0x7FF, false, 0) }
    }

    /// Bits on the wire at the nominal rate and at the data rate, including stuff bits (estimated: one per four
    /// stuffable bits for classic frames, the fixed stuff bits of the CAN FD CRC plus one in eight data bits), ACK,
    /// EOF and intermission.
    pub fn bits(&self) -> (u64, u64) {
        let payload = if self.rtr { 0 } else { 8 * self.data.len() as u64 };
        let arbitration = if self.extended { 32 } else { 12 }; // ID (+ SRR, IDE, ext ID), RTR/RRS
        if !self.fd {
            // SOF + arbitration + IDE + r0 + DLC + data + CRC15
            let stuffable = 1 + arbitration + 2 + 4 + payload + 15;
            let total = stuffable + stuffable / 4 + 1 + 2 + 7 + 3;
            return (total, 0);
        }
        // Nominal phase: SOF, arbitration, IDE/FDF/res, BRS, then ACK, EOF, intermission
        let nominal_head = 1 + arbitration + 3 + 1;
        let nominal_tail = 1 + 2 + 7 + 3;
        let crc = if self.data.len() > 16 { 21 } else { 17 };
        // Dynamic stuff bits in the data phase: about one in eight for typical payloads
        let data_phase = 1 + 4 + 4 + payload + crc + crc / 4 + (1 + 4 + payload) / 8;
        if self.brs { (nominal_head + nominal_tail, data_phase) } else { (nominal_head + nominal_tail + data_phase, 0) }
    }

    /// Time the frame occupies the bus.
    pub fn duration(&self, nominal_bps: u64, data_bps: u64) -> Time {
        let (n, d) = self.bits();
        let ps = |bits: u64, bps: u64| (bits * 1_000_000_000_000).checked_div(bps).unwrap_or(0);
        Time(ps(n, nominal_bps) + ps(d, data_bps.max(nominal_bps)))
    }
}

/// Bit rates a node is configured for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BitRates {
    pub nominal: u64,
    /// Data phase rate, `None` for a classic CAN node.
    pub data: Option<u64>,
}

/// A frame a node wants to send.
#[derive(Clone, Debug)]
pub struct TxRequest {
    /// Node-chosen identifier returned in [`CanNode::tx_done`].
    pub token: u64,
    pub frame: CanFrame,
    /// When the node became ready to send it.
    pub ready_at: Time,
}

/// A CAN controller on the bus.
pub trait CanNode {
    /// Bit rates, or `None` if the node is not participating (configuration or sleep mode).
    fn bit_rates(&self) -> Option<BitRates>;
    /// Whether the node acknowledges frames (normal operation, not listen-only).
    fn acknowledges(&self) -> bool;
    /// The frame this node would put on the bus next.
    fn tx_request(&mut self) -> Option<TxRequest>;
    /// The frame `token` finished at `at`; `acked` is false if no other node acknowledged it.
    fn tx_done(&mut self, token: u64, at: Time, acked: bool);
    /// A frame from another node finished at `at`.
    fn receive(&mut self, frame: &CanFrame, at: Time);
}

/// A host-side participant: a SocketCAN bridge, a scripted mock, a recorder. Endpoints always acknowledge.
pub trait CanEndpoint {
    /// Frames the endpoint wants to send up to `until`, with the time each became ready.
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)>;
    fn receive(&mut self, frame: &CanFrame, at: Time);
}

/// Who sent a frame on the bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sender {
    Node(usize),
    Endpoint(usize),
}

/// One frame as it went over the bus.
#[derive(Clone, Debug)]
pub struct BusRecord {
    pub start: Time,
    pub end: Time,
    pub sender: Sender,
    pub frame: CanFrame,
    pub acked: bool,
}

#[derive(Clone, Debug)]
pub struct CanBus {
    pub name: String,
    pub rates: BitRates,
    /// The bus is free again from this time.
    pub free_at: Time,
    /// Frames endpoints handed in, waiting for the bus.
    endpoint_queue: Vec<VecDeque<(Time, CanFrame)>>,
    /// Everything that went over the bus, if recording.
    pub record: Option<Vec<BusRecord>>,
    warned_rate: Vec<bool>,
}

impl CanBus {
    pub fn new(name: &str, rates: BitRates) -> CanBus {
        CanBus {
            name: name.to_string(),
            rates,
            free_at: Time::ZERO,
            endpoint_queue: Vec::new(),
            record: None,
            warned_rate: Vec::new(),
        }
    }

    fn compatible(&mut self, idx: usize, r: Option<BitRates>) -> bool {
        let Some(r) = r else { return false };
        let ok = r.nominal == self.rates.nominal && (r.data.is_none() || r.data == self.rates.data);
        if !ok {
            if self.warned_rate.len() <= idx {
                self.warned_rate.resize(idx + 1, false);
            }
            if !self.warned_rate[idx] {
                self.warned_rate[idx] = true;
                log::warn!("CAN bus {}: node {idx} configured for {r:?}, bus runs {:?}", self.name, self.rates);
            }
        }
        ok
    }

    /// Runs the bus up to `until`: arbitrates, transmits and delivers every frame that finishes by then.
    pub fn step(&mut self, nodes: &mut [&mut dyn CanNode], endpoints: &mut [&mut dyn CanEndpoint], until: Time) {
        if self.endpoint_queue.len() < endpoints.len() {
            self.endpoint_queue.resize(endpoints.len(), VecDeque::new());
        }
        for (i, e) in endpoints.iter_mut().enumerate() {
            self.endpoint_queue[i].extend(e.take_tx(until));
        }
        loop {
            // Collect requests
            let mut cands: Vec<(Time, Sender, u64, CanFrame)> = Vec::new();
            for (i, node) in nodes.iter_mut().enumerate() {
                if !self.compatible(i, node.bit_rates()) {
                    continue;
                }
                if let Some(r) = node.tx_request() {
                    cands.push((r.ready_at, Sender::Node(i), r.token, r.frame));
                }
            }
            for (i, q) in self.endpoint_queue.iter().enumerate() {
                if let Some((t, f)) = q.front() {
                    cands.push((*t, Sender::Endpoint(i), 0, f.clone()));
                }
            }
            let Some(first) = cands.iter().map(|c| c.0).min() else { break };
            let start = first.max(self.free_at);
            if start > until {
                break;
            }
            // Arbitration among everything ready by the start of the frame
            let winner = cands
                .into_iter()
                .filter(|c| c.0 <= start)
                .min_by_key(|c| (c.3.arbitration_key(), matches!(c.1, Sender::Endpoint(_))))
                .unwrap();
            let (_, sender, token, frame) = winner;
            let data_rate = self.rates.data.unwrap_or(self.rates.nominal);
            let end = start + frame.duration(self.rates.nominal, data_rate);
            if end > until {
                break;
            }
            // Acknowledged if any other active node or any endpoint listens
            let mut acked = (0..endpoints.len()).any(|i| Sender::Endpoint(i) != sender);
            for (i, node) in nodes.iter().enumerate() {
                if Sender::Node(i) != sender && node.acknowledges() && self.compatible(i, node.bit_rates()) {
                    acked = true;
                }
            }
            match sender {
                Sender::Node(i) => nodes[i].tx_done(token, end, acked),
                // An endpoint's frame is sent once whether or not anyone acknowledges it (SocketCAN reports errors
                // to its own writer, which is outside the simulation)
                Sender::Endpoint(i) => {
                    self.endpoint_queue[i].pop_front();
                }
            }
            if acked {
                for (i, node) in nodes.iter_mut().enumerate() {
                    if Sender::Node(i) != sender && self.compatible(i, node.bit_rates()) {
                        node.receive(&frame, end);
                    }
                }
                for (i, e) in endpoints.iter_mut().enumerate() {
                    if Sender::Endpoint(i) != sender {
                        e.receive(&frame, end);
                    }
                }
            }
            if let Some(rec) = &mut self.record {
                rec.push(BusRecord { start, end, sender, frame, acked });
            }
            self.free_at = end;
        }
    }
}

/// An endpoint that records everything and sends what it is given: the in-process `candump`/`cansend`.
#[derive(Clone, Debug, Default)]
pub struct Monitor {
    pub received: Vec<(Time, CanFrame)>,
    pub outbox: Vec<(Time, CanFrame)>,
}

impl CanEndpoint for Monitor {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let (now, later): (Vec<_>, Vec<_>) =
            std::mem::take(&mut self.outbox).into_iter().partition(|(t, _)| *t <= until);
        self.outbox = later;
        now
    }
    fn receive(&mut self, frame: &CanFrame, at: Time) {
        self.received.push((at, frame.clone()));
    }
}

/// `candump`-style text for a frame: `123#DEADBEEF` or `12345678##1DEADBEEF` for FD.
pub fn candump(frame: &CanFrame) -> String {
    let id = if frame.extended { format!("{:08X}", frame.id) } else { format!("{:03X}", frame.id) };
    let data: String = frame.data.iter().map(|b| format!("{b:02X}")).collect();
    if frame.fd {
        format!("{id}##{}{data}", if frame.brs { 1 } else { 0 })
    } else if frame.rtr {
        format!("{id}#R")
    } else {
        format!("{id}#{data}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A node that sends a fixed list of frames.
    struct Node {
        rates: Option<BitRates>,
        queue: VecDeque<(Time, CanFrame)>,
        done: Vec<(u64, Time, bool)>,
        rx: Vec<(Time, CanFrame)>,
        acks: bool,
    }

    impl CanNode for Node {
        fn bit_rates(&self) -> Option<BitRates> {
            self.rates
        }
        fn acknowledges(&self) -> bool {
            self.acks
        }
        fn tx_request(&mut self) -> Option<TxRequest> {
            self.queue.front().map(|(t, f)| TxRequest { token: f.id as u64, frame: f.clone(), ready_at: *t })
        }
        fn tx_done(&mut self, token: u64, at: Time, acked: bool) {
            self.done.push((token, at, acked));
            if acked {
                self.queue.pop_front();
            }
        }
        fn receive(&mut self, frame: &CanFrame, at: Time) {
            self.rx.push((at, frame.clone()));
        }
    }

    const FD: BitRates = BitRates { nominal: 1_000_000, data: Some(5_000_000) };

    fn node(frames: &[(u64, CanFrame)]) -> Node {
        Node {
            rates: Some(FD),
            queue: frames.iter().map(|(us, f)| (Time::from_us(*us), f.clone())).collect(),
            done: vec![],
            rx: vec![],
            acks: true,
        }
    }

    #[test]
    fn frame_durations() {
        let f = CanFrame::classic(0x123, false, &[0; 8]);
        // 1 + 12 + 2 + 4 + 64 + 15 = 98 stuffable, +24 stuff, +13 = 135 bits at 1 Mbit/s
        assert_eq!(f.duration(1_000_000, 1_000_000), Time::from_us(135));
        let fd = CanFrame::fd(0x123, false, &[0; 64]);
        let d = fd.duration(1_000_000, 5_000_000);
        assert!(d < Time::from_us(160) && d > Time::from_us(120), "{d:?}");
        assert_eq!(fd_len_to_dlc(9), 9);
        assert_eq!(fd_len_to_dlc(64), 15);
        assert_eq!(fd_dlc_to_len(13), 32);
    }

    #[test]
    fn arbitration_lowest_id_wins_and_delivery() {
        let mut a = node(&[(0, CanFrame::fd(0x200, false, &[1]))]);
        let mut b = node(&[(0, CanFrame::fd(0x100, false, &[2]))]);
        let mut mon = Monitor::default();
        let mut bus = CanBus::new("can0", FD);
        bus.record = Some(vec![]);
        bus.step(&mut [&mut a, &mut b], &mut [&mut mon], Time::from_ms(1));
        let rec = bus.record.unwrap();
        assert_eq!(rec[0].frame.id, 0x100, "lower ID first");
        assert_eq!(rec[1].frame.id, 0x200);
        assert_eq!(rec[1].start, rec[0].end, "back to back");
        assert!(rec.iter().all(|r| r.acked));
        assert_eq!(a.rx.len(), 1);
        assert_eq!(b.rx.len(), 1);
        assert_eq!(mon.received.len(), 2);
        assert_eq!(candump(&mon.received[0].1), "100##102");
    }

    #[test]
    fn unacknowledged_without_listeners() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[]))]);
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a], &mut [], Time::from_us(500));
        assert!(!a.done.is_empty());
        assert!(a.done.iter().all(|d| !d.2), "no ACK on an empty bus");
    }

    #[test]
    fn frames_finishing_after_the_step_wait() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[0; 8]))]);
        let mut mon = Monitor::default();
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_us(100));
        assert!(mon.received.is_empty());
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_us(200));
        assert_eq!(mon.received.len(), 1);
        assert_eq!(mon.received[0].0, Time::from_us(135));
    }

    #[test]
    fn mismatched_bit_rate_is_excluded() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[]))]);
        a.rates = Some(BitRates { nominal: 500_000, data: None });
        let mut mon = Monitor::default();
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_ms(1));
        assert!(mon.received.is_empty());
        assert!(a.done.is_empty());
    }

    #[test]
    fn endpoint_frames_reach_nodes() {
        let mut a = node(&[]);
        let mut mon = Monitor::default();
        mon.outbox.push((Time::from_us(10), CanFrame::fd(0x55, true, &[9, 9])));
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_ms(1));
        assert_eq!(a.rx.len(), 1);
        assert_eq!(a.rx[0].1.id, 0x55);
        assert!(mon.received.is_empty(), "an endpoint does not hear itself");
    }
}
