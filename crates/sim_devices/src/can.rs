//! CAN frames and an in-process CAN bus.
//!
//! The bus serialises frames with arbitration (lowest identifier wins among requests pending when the bus becomes
//! free), charges each frame its on-wire duration at the bus's nominal and data bit rates, acknowledges a frame only
//! if another active node or a host endpoint is listening, and delivers it to everyone else when its last bit is
//! sent. It is stepped by whoever owns the machines (a single board's session, or a robot of many boards) after
//! they have all run to the same time, which keeps multi-board runs deterministic.
//!
//! Faults ([`BusFaults`]) are injected on the bus itself:
//! * **bit errors**: a frame hit by a disturbance is cut off where the error struck and followed by an error frame
//!   (error flag, delimiter, intermission); it is not delivered, the transmitter's controller counts a transmit
//!   error and retries, every receiver counts a receive error (ISO 11898-1 fault confinement, see
//!   [`CanNode::tx_error`]);
//! * **stuck dominant**: no frame can start, a frame on the wire is cut off, and every node counts the dominant
//!   bits until the bus is released;
//! * **unplugged node**: the node sends alone on its stub, so nothing it sends is acknowledged and it hears
//!   nothing, while the rest of the bus carries on without it;
//! * **floods**: an extra node sending one frame at a set bus load, competing in arbitration like any other.
//!
//! An unacknowledged frame is 6 bits longer (the transmitter's error flag after the ACK slot), and an error
//! passive transmitter waits 8 more bit times before its next frame ("suspend transmission"), so error active
//! nodes get the bus first.

use std::collections::VecDeque;

use sim_core::time::Time;

use crate::fault::Rng;

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
///
/// Besides frames, the bus reports what fault confinement (ISO 11898-1 clause 12, "Fault confinement") counts:
/// error frames that destroyed a frame the node sent or was receiving, dominant bits on a bus that is stuck
/// dominant, and the 11-recessive-bit sequences a bus-off node counts to recover. The defaults ignore them, for
/// nodes that keep no error counters.
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
    /// The frame `token` was destroyed by an error frame that ended at `at`. It is still pending: the controller
    /// retransmits it (within its retransmission limit). The transmitter's TEC rises by 8.
    fn tx_error(&mut self, token: u64, at: Time, error: BusError) {
        let _ = (token, at, error);
    }
    /// A frame from another node was destroyed by an error frame that ended at `at`; the receiver's REC rises by 1.
    fn rx_error(&mut self, at: Time, error: BusError) {
        let _ = (at, error);
    }
    /// The bus stayed dominant for `bits` more bit times up to `at` (a stuck bus): after its error flag a node
    /// adds 8 to an error counter for every 8 consecutive dominant bits.
    fn dominant_bits(&mut self, at: Time, bits: u64) {
        let _ = (at, bits);
    }
    /// `n` more sequences of 11 consecutive recessive bits went by up to `at`: one at the end of every frame or
    /// error frame (EOF or error delimiter, plus intermission), and one per 11 bit times of idle bus.
    fn recessive_sequences(&mut self, at: Time, n: u64) {
        let _ = (at, n);
    }
    /// Whether the node is error passive: after sending it then waits 8 more bit times ("suspend transmission")
    /// before it may start another frame, letting error active nodes go first.
    fn error_passive(&self) -> bool {
        false
    }
}

/// A host-side participant: a SocketCAN bridge, a scripted mock, a recorder. Endpoints always acknowledge.
pub trait CanEndpoint {
    /// Frames the endpoint wants to send up to `until`, with the time each became ready.
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)>;
    fn receive(&mut self, frame: &CanFrame, at: Time);
    /// How many frames the endpoint can have waiting for the bus; it loses the ones beyond. A program writing to
    /// a Linux CAN interface sees its transmit queue (`txqueuelen`, [`ENDPOINT_TX_QUEUE`] by default) refuse
    /// writes with `ENOBUFS` while the bus cannot take frames (stuck, flooded, or no one acknowledging), and
    /// drops them unless it retries. `None` (the default): it waits for room, nothing is lost.
    fn tx_queue_limit(&self) -> Option<usize> {
        None
    }
}

/// Who sent a frame on the bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sender {
    Node(usize),
    Endpoint(usize),
    /// A flood injected on the bus ([`Flood`], by index in [`BusFaults::floods`]).
    Flood(usize),
}

/// Why a frame on the bus was destroyed by an error frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BusError {
    /// A disturbance flipped a bit: the transmitter saw a bit error (or the receivers a stuff, CRC or form error)
    /// and signalled it with an error flag. `data_phase` if it hit the data phase of a CAN FD frame with bit rate
    /// switching (the error counters in C1BDIAG0 tell the phases apart).
    Bit { data_phase: bool },
    /// The bus became stuck dominant while the frame was being sent.
    StuckDominant,
}

/// One frame as it went over the bus.
#[derive(Clone, Debug)]
pub struct BusRecord {
    pub start: Time,
    /// End of the frame, or of the error frame that destroyed it.
    pub end: Time,
    pub sender: Sender,
    pub frame: CanFrame,
    pub acked: bool,
    /// The frame was destroyed by an error frame (and not delivered).
    pub error: Option<BusError>,
}

/// A node that floods the bus with one frame (a babbling node, a bus load test), injected as a fault.
#[derive(Clone, Debug)]
pub struct Flood {
    pub frame: CanFrame,
    /// Time from the start of one frame to the earliest start of the next; zero sends back to back.
    pub period: Time,
    /// When the next frame is ready.
    pub next: Time,
    /// Frames that made it onto the bus.
    pub sent: u64,
}

impl Flood {
    /// Floods with `frame` from `start`, aiming at `load` (0 to 1) of the bus's capacity at `rates`. At a load of 1
    /// a flood whose identifier wins arbitration keeps every other node off the bus (a "babbling idiot").
    pub fn with_load(frame: CanFrame, load: f64, rates: BitRates, start: Time) -> Flood {
        let d = frame.duration(rates.nominal, rates.data.unwrap_or(rates.nominal));
        let period = if load >= 1.0 { Time::ZERO } else { Time((d.as_ps() as f64 / load.max(1e-6)) as u64) };
        Flood { frame, period, next: start, sent: 0 }
    }
}

/// Faults injected on a bus.
#[derive(Clone, Debug)]
pub struct BusFaults {
    /// Probability that a frame is hit by a bit error (an EMI burst, a marginal termination).
    pub error_rate: f64,
    /// Frames still to be corrupted whatever the rate.
    pub corrupt_next: u32,
    /// The bus is held dominant from this time (a transceiver whose TXD is stuck low, CANH shorted to the
    /// supply). Nothing can be sent; every node's error counters climb.
    pub stuck_dominant: Option<Time>,
    /// Nodes (by index) unplugged from the bus, each alone on its own stub, with when that stub is free.
    pub unplugged: Vec<(usize, Time)>,
    pub floods: Vec<Flood>,
    /// Frames destroyed by injected errors, for tests.
    pub corrupted: u64,
    rng: Rng,
}

impl BusFaults {
    fn new(name: &str) -> Self {
        BusFaults {
            error_rate: 0.0,
            corrupt_next: 0,
            stuck_dominant: None,
            unplugged: Vec::new(),
            floods: Vec::new(),
            corrupted: 0,
            rng: Rng::from_name(name),
        }
    }

    fn is_unplugged(&self, i: usize) -> bool {
        self.unplugged.iter().any(|(n, _)| *n == i)
    }

    /// Whether the next frame is hit, and where: the fraction of the frame sent before the error.
    fn roll(&mut self) -> Option<f64> {
        let hit = if self.corrupt_next > 0 {
            self.corrupt_next -= 1;
            true
        } else {
            self.rng.chance(self.error_rate)
        };
        hit.then(|| 0.05 + 0.9 * self.rng.uniform())
    }
}

/// Bits an ACK error adds to a frame: the transmitter's error flag (6) starts after the ACK slot, then its
/// delimiter (8) and the intermission (3), instead of the ACK delimiter (1), EOF (7) and intermission (3).
const ACK_ERROR_EXTRA_BITS: u64 = 6;
/// An error passive transmitter's "suspend transmission" time.
const SUSPEND_TRANSMISSION_BITS: u64 = 8;

/// Default transmit queue length of a Linux CAN interface (`txqueuelen`).
pub const ENDPOINT_TX_QUEUE: usize = 10;

/// An error flag (6 bits), its delimiter (8) and the intermission (3), sent at the nominal rate after an error.
const ERROR_FRAME_TAIL_BITS: u64 = 17;

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
    pub faults: BusFaults,
    /// Frames endpoints handed in that did not fit in their queue ([`CanEndpoint::tx_queue_limit`]).
    pub endpoint_dropped: u64,
    /// Idle bus up to this time has been reported to the nodes as recessive sequences, with what was left over.
    idle_reported: Time,
    idle_carry: Time,
    /// Dominant bits of a stuck bus have been reported up to here.
    dominant_reported: Time,
    /// An error passive node that just sent may not start again before this time.
    suspended: Option<(usize, Time)>,
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
            faults: BusFaults::new(name),
            endpoint_dropped: 0,
            idle_reported: Time::ZERO,
            idle_carry: Time::ZERO,
            dominant_reported: Time::ZERO,
            suspended: None,
        }
    }

    /// One nominal bit time.
    fn bit_time(&self) -> Time {
        Time(1_000_000_000_000 / self.rates.nominal.max(1))
    }

    fn bits(&self, n: u64) -> Time {
        Time(self.bit_time().as_ps() * n)
    }

    /// Holds the bus dominant from `at` (`stuck`), or releases it at `at`.
    pub fn set_stuck_dominant(&mut self, stuck: bool, at: Time) {
        match (stuck, self.faults.stuck_dominant) {
            (true, None) => {
                self.faults.stuck_dominant = Some(at);
                self.dominant_reported = at;
            }
            (false, Some(_)) => {
                self.faults.stuck_dominant = None;
                // Nothing was sent while it was stuck: the bus is idle from the release
                self.free_at = self.free_at.max(at);
                self.idle_reported = self.idle_reported.max(at);
            }
            _ => {}
        }
    }

    /// Unplugs node `node` (by its index in the node list [`CanBus::step`] gets) from the bus at `at`, or plugs it
    /// back in. Unplugged, it sends alone on its stub: nobody acknowledges, and it hears nothing.
    pub fn set_unplugged(&mut self, node: usize, unplugged: bool, at: Time) {
        self.faults.unplugged.retain(|(n, _)| *n != node);
        if unplugged {
            self.faults.unplugged.push((node, at));
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

    /// The nodes that are on the bus and running at its rates.
    fn listening(&mut self, nodes: &[&mut dyn CanNode]) -> Vec<bool> {
        (0..nodes.len()).map(|i| !self.faults.is_unplugged(i) && self.compatible(i, nodes[i].bit_rates())).collect()
    }

    /// Reports idle bus from where the last report ended up to `to` as recessive sequences.
    fn report_idle(&mut self, nodes: &mut [&mut dyn CanNode], on: &[bool], to: Time) {
        let from = self.idle_reported.max(self.free_at);
        if to <= from {
            return;
        }
        self.idle_reported = to;
        let seq = Time(self.bit_time().as_ps() * 11);
        let total = self.idle_carry + (to - from);
        let n = total.as_ps() / seq.as_ps().max(1);
        self.idle_carry = Time(total.as_ps() % seq.as_ps().max(1));
        if n > 0 {
            for (node, _) in nodes.iter_mut().zip(on).filter(|(_, on)| **on) {
                node.recessive_sequences(to, n);
            }
        }
    }

    /// An unplugged node sends alone: every frame goes unacknowledged on its stub.
    fn step_unplugged(&mut self, nodes: &mut [&mut dyn CanNode], until: Time) {
        let data_rate = self.rates.data.unwrap_or(self.rates.nominal);
        for k in 0..self.faults.unplugged.len() {
            let (i, mut free) = self.faults.unplugged[k];
            if i >= nodes.len() || !self.compatible(i, nodes[i].bit_rates()) {
                continue;
            }
            // Bounded: each pass either sends or stops; a node can only retry so often per step
            for _ in 0..10_000 {
                let Some(r) = nodes[i].tx_request() else { break };
                let start = r.ready_at.max(free);
                let end = start + r.frame.duration(self.rates.nominal, data_rate) + self.bits(ACK_ERROR_EXTRA_BITS);
                if end > until {
                    break;
                }
                nodes[i].tx_done(r.token, end, false);
                free = end;
                if nodes[i].error_passive() {
                    free += self.bits(SUSPEND_TRANSMISSION_BITS);
                }
            }
            self.faults.unplugged[k].1 = free;
        }
    }

    /// Runs the bus up to `until`: arbitrates, transmits and delivers every frame that finishes by then.
    pub fn step(&mut self, nodes: &mut [&mut dyn CanNode], endpoints: &mut [&mut dyn CanEndpoint], until: Time) {
        if self.endpoint_queue.len() < endpoints.len() {
            self.endpoint_queue.resize(endpoints.len(), VecDeque::new());
        }
        for (i, e) in endpoints.iter_mut().enumerate() {
            let limit = e.tx_queue_limit();
            for f in e.take_tx(until) {
                if limit.is_some_and(|l| self.endpoint_queue[i].len() >= l) {
                    self.endpoint_dropped += 1;
                } else {
                    self.endpoint_queue[i].push_back(f);
                }
            }
        }
        if !self.faults.unplugged.is_empty() {
            self.step_unplugged(nodes, until);
        }
        let on = self.listening(nodes);
        let stuck = self.faults.stuck_dominant;
        let data_rate = self.rates.data.unwrap_or(self.rates.nominal);
        // Idle time is reported up to here: not past the start of a frame that is already on the wire
        let mut idle_to = until;
        loop {
            // Collect requests
            let mut cands: Vec<(Time, Sender, u64, CanFrame)> = Vec::new();
            for (i, node) in nodes.iter_mut().enumerate() {
                if !on[i] {
                    continue;
                }
                if let Some(r) = node.tx_request() {
                    let ready = match self.suspended {
                        Some((n, until)) if n == i => r.ready_at.max(until),
                        _ => r.ready_at,
                    };
                    cands.push((ready, Sender::Node(i), r.token, r.frame));
                }
            }
            for (i, q) in self.endpoint_queue.iter().enumerate() {
                if let Some((t, f)) = q.front() {
                    cands.push((*t, Sender::Endpoint(i), 0, f.clone()));
                }
            }
            for (i, f) in self.faults.floods.iter().enumerate() {
                cands.push((f.next, Sender::Flood(i), 0, f.frame.clone()));
            }
            let Some(first) = cands.iter().map(|c| c.0).min() else { break };
            let start = first.max(self.free_at);
            // Nothing starts on a bus held dominant: every node sees a bus that never goes idle
            if start > until || stuck.is_some_and(|s| start >= s) {
                break;
            }
            // Arbitration among everything ready by the start of the frame
            let winner = cands
                .into_iter()
                .filter(|c| c.0 <= start)
                .min_by_key(|c| (c.3.arbitration_key(), !matches!(c.1, Sender::Node(_))))
                .unwrap();
            let (_, sender, token, frame) = winner;
            let full_end = start + frame.duration(self.rates.nominal, data_rate);
            let cut_by_stuck = stuck.filter(|&s| full_end > s);
            if cut_by_stuck.is_none_or(|s| s > until) && full_end > until {
                idle_to = start;
                break;
            }
            self.report_idle(nodes, &on, start);
            let error = match cut_by_stuck {
                Some(_) => Some((BusError::StuckDominant, 0.0)),
                None => self.faults.roll().map(|f| {
                    let (n, d) = frame.bits();
                    // Where the error hit: the nominal phase is before and after the data phase
                    let data_phase = frame.brs && d > 0 && {
                        let head = (1 + if frame.extended { 32 } else { 12 } + 4) as f64 / (n + d) as f64;
                        f > head && f < head + d as f64 / (n + d) as f64
                    };
                    (BusError::Bit { data_phase }, f)
                }),
            };
            if let Some((err, frac)) = error {
                let end = match cut_by_stuck {
                    Some(s) => s,
                    None => {
                        let sent = Time(((full_end - start).as_ps() as f64 * frac) as u64);
                        start + sent + Time(self.bit_time().as_ps() * ERROR_FRAME_TAIL_BITS)
                    }
                };
                self.faults.corrupted += 1;
                self.suspended = None;
                match sender {
                    Sender::Node(i) => {
                        nodes[i].tx_error(token, end, err);
                        if nodes[i].error_passive() {
                            self.suspended = Some((i, end + self.bits(SUSPEND_TRANSMISSION_BITS)));
                        }
                    }
                    // An endpoint's or a flood's frame is simply retried (their controllers are outside the model)
                    Sender::Endpoint(_) | Sender::Flood(_) => {}
                }
                for (i, node) in nodes.iter_mut().enumerate() {
                    if on[i] && Sender::Node(i) != sender {
                        node.rx_error(end, err);
                    }
                }
                if let Some(rec) = &mut self.record {
                    rec.push(BusRecord { start, end, sender, frame, acked: false, error: Some(err) });
                }
                self.free_at = end;
                if cut_by_stuck.is_none() {
                    // The error delimiter and intermission are 11 recessive bits
                    for (node, _) in nodes.iter_mut().zip(&on).filter(|(_, on)| **on) {
                        node.recessive_sequences(end, 1);
                    }
                    self.idle_reported = self.idle_reported.max(end);
                }
                continue;
            }
            // Acknowledged if any other active node or any endpoint listens
            let mut acked = (0..endpoints.len()).any(|i| Sender::Endpoint(i) != sender);
            for (i, node) in nodes.iter().enumerate() {
                if Sender::Node(i) != sender && on[i] && node.acknowledges() {
                    acked = true;
                }
            }
            let end = if acked { full_end } else { full_end + self.bits(ACK_ERROR_EXTRA_BITS) };
            self.suspended = None;
            match sender {
                Sender::Node(i) => {
                    nodes[i].tx_done(token, end, acked);
                    if nodes[i].error_passive() {
                        self.suspended = Some((i, end + self.bits(SUSPEND_TRANSMISSION_BITS)));
                    }
                }
                // An endpoint's frame is sent once whether or not anyone acknowledges it (SocketCAN reports errors
                // to its own writer, which is outside the simulation)
                Sender::Endpoint(i) => {
                    self.endpoint_queue[i].pop_front();
                }
                Sender::Flood(i) => {
                    let f = &mut self.faults.floods[i];
                    f.sent += 1;
                    f.next = (start + f.period).max(end);
                }
            }
            if acked {
                for (i, node) in nodes.iter_mut().enumerate() {
                    if Sender::Node(i) != sender && on[i] {
                        node.receive(&frame, end);
                    }
                }
                for (i, e) in endpoints.iter_mut().enumerate() {
                    if Sender::Endpoint(i) != sender {
                        e.receive(&frame, end);
                    }
                }
            }
            for (node, _) in nodes.iter_mut().zip(&on).filter(|(_, on)| **on) {
                node.recessive_sequences(end, 1);
            }
            if let Some(rec) = &mut self.record {
                rec.push(BusRecord { start, end, sender, frame, acked, error: None });
            }
            self.free_at = end;
            self.idle_reported = self.idle_reported.max(end);
        }
        match stuck {
            Some(s) => {
                // Held dominant: the nodes count the dominant bits, and the bus stays busy
                let from = self.dominant_reported.max(s).max(self.free_at);
                if until > from {
                    let bits = (until - from).as_ps() / self.bit_time().as_ps().max(1);
                    let used = Time(bits * self.bit_time().as_ps());
                    if bits > 0 {
                        for (node, _) in nodes.iter_mut().zip(&on).filter(|(_, on)| **on) {
                            node.dominant_bits(until, bits);
                        }
                        self.dominant_reported = from + used;
                    }
                }
                self.free_at = self.free_at.max(until);
                self.idle_reported = self.idle_reported.max(until);
            }
            None => self.report_idle(nodes, &on, idle_to),
        }
    }
}

/// An endpoint that records everything and sends what it is given: the in-process `candump`/`cansend`.
#[derive(Clone, Debug, Default)]
pub struct Monitor {
    pub received: Vec<(Time, CanFrame)>,
    pub outbox: Vec<(Time, CanFrame)>,
    /// Its transmit queue length, `None` for unbounded ([`CanEndpoint::tx_queue_limit`]).
    pub limit: Option<usize>,
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
    fn tx_queue_limit(&self) -> Option<usize> {
        self.limit
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
        tx_errors: Vec<(Time, BusError)>,
        rx_errors: u32,
        dominant: u64,
        recessive: u64,
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
        fn tx_error(&mut self, _token: u64, at: Time, error: BusError) {
            self.tx_errors.push((at, error));
        }
        fn rx_error(&mut self, _at: Time, _error: BusError) {
            self.rx_errors += 1;
        }
        fn dominant_bits(&mut self, _at: Time, bits: u64) {
            self.dominant += bits;
        }
        fn recessive_sequences(&mut self, _at: Time, n: u64) {
            self.recessive += n;
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
            tx_errors: vec![],
            rx_errors: 0,
            dominant: 0,
            recessive: 0,
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

    #[test]
    fn a_corrupted_frame_becomes_an_error_frame_and_is_retried() {
        let f = CanFrame::fd(0x100, false, &[1; 16]);
        let mut a = node(&[(0, f.clone())]);
        let mut b = node(&[]);
        let mut mon = Monitor::default();
        let mut bus = CanBus::new("can0", FD);
        bus.record = Some(vec![]);
        bus.faults.corrupt_next = 1;
        bus.step(&mut [&mut a, &mut b], &mut [&mut mon], Time::from_ms(1));
        let rec = bus.record.as_ref().unwrap();
        assert_eq!(rec.len(), 2, "the error frame, then the retransmission");
        assert!(rec[0].error.is_some() && !rec[0].acked);
        assert!(rec[1].error.is_none() && rec[1].acked);
        assert_eq!(rec[1].start, rec[0].end, "retransmitted as soon as the bus is free");
        // Cut off where the error hit, plus 17 bits of error flag, delimiter and intermission
        let err_len = rec[0].end - rec[0].start;
        assert!(err_len < f.duration(1_000_000, 5_000_000) + Time::from_us(17) && err_len > Time::from_us(17));
        assert_eq!(a.tx_errors.len(), 1, "the transmitter counts a transmit error");
        assert_eq!(b.rx_errors, 1, "the receiver a receive error");
        assert_eq!(b.rx.len(), 1, "and gets the frame once, from the retransmission");
        assert_eq!(mon.received.len(), 1);
        assert_eq!(bus.faults.corrupted, 1);
    }

    #[test]
    fn error_rate_hits_about_that_share_of_frames() {
        let frames: Vec<(u64, CanFrame)> =
            (0..400).map(|i| (i * 200, CanFrame::classic(0x10, false, &[0; 8]))).collect();
        let mut a = node(&frames);
        let mut b = node(&[]);
        let mut bus = CanBus::new("can0", FD);
        bus.faults.error_rate = 0.25;
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(200));
        assert_eq!(b.rx.len(), 400, "every frame gets through in the end");
        let errors = a.tx_errors.len();
        // 0.25 / 0.75 errors per delivered frame
        assert!((100..=170).contains(&errors), "{errors}");
    }

    #[test]
    fn stuck_dominant_bus_stops_everything_until_released() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[1])), (500, CanFrame::classic(0x11, false, &[2]))]);
        let mut b = node(&[]);
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_us(300));
        assert_eq!(b.rx.len(), 1);
        bus.set_stuck_dominant(true, Time::from_us(300));
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(2));
        assert_eq!(b.rx.len(), 1, "nothing starts on a dominant bus");
        assert!(a.tx_errors.is_empty(), "a node waiting to send never starts, so never counts a transmit error");
        // 1.7 ms at 1 Mbit/s
        assert_eq!(b.dominant, 1700);
        assert_eq!(a.dominant, 1700);
        bus.set_stuck_dominant(false, Time::from_ms(2));
        let before = b.recessive;
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(3));
        assert_eq!(b.rx.len(), 2, "sent once the bus is released");
        assert_eq!(b.rx[1].0, Time::from_ms(2) + CanFrame::classic(0x11, false, &[2]).duration(1_000_000, 1_000_000));
        assert!(b.recessive > before + 80, "idle bus counts as recessive sequences");
    }

    #[test]
    fn stuck_dominant_cuts_off_the_frame_on_the_wire() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[0; 8]))]);
        let mut b = node(&[]);
        let mut bus = CanBus::new("can0", FD);
        bus.record = Some(vec![]);
        bus.set_stuck_dominant(true, Time::from_us(50));
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_us(200));
        assert_eq!(a.tx_errors, vec![(Time::from_us(50), BusError::StuckDominant)]);
        assert_eq!(b.rx_errors, 1);
        assert_eq!(a.dominant, 150);
        let rec = bus.record.unwrap();
        assert_eq!(rec[0].error, Some(BusError::StuckDominant));
    }

    #[test]
    fn unplugged_node_sends_alone_and_hears_nothing() {
        let mut a = node(&[(0, CanFrame::classic(0x10, false, &[1]))]);
        let mut b = node(&[(0, CanFrame::classic(0x20, false, &[2]))]);
        let mut c = node(&[]);
        let mut mon = Monitor::default();
        mon.outbox.push((Time::from_us(10), CanFrame::classic(0x30, false, &[3])));
        let mut bus = CanBus::new("can0", FD);
        bus.record = Some(vec![]);
        bus.set_unplugged(0, true, Time::ZERO);
        bus.step(&mut [&mut a, &mut b, &mut c], &mut [&mut mon], Time::from_ms(1));
        assert!(a.done.len() > 5 && a.done.iter().all(|d| !d.2), "retried without ever being acknowledged");
        assert!(a.rx.is_empty(), "hears nothing");
        assert_eq!(c.rx.len(), 2, "the rest of the bus carries on");
        assert!(bus.record.as_ref().unwrap().iter().all(|r| r.sender != Sender::Node(0)));
        bus.set_unplugged(0, false, Time::from_ms(1));
        bus.step(&mut [&mut a, &mut b, &mut c], &mut [&mut mon], Time::from_ms(2));
        assert_eq!(c.rx.len(), 3, "plugged back in");
    }

    #[test]
    fn a_flood_takes_the_bus() {
        let flood = Flood::with_load(CanFrame::classic(0x001, false, &[0xAA; 8]), 1.0, FD, Time::ZERO);
        let mut a = node(&[(10, CanFrame::classic(0x100, false, &[1]))]);
        let mut b = node(&[]);
        let mut bus = CanBus::new("can0", FD);
        bus.faults.floods.push(flood);
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(5));
        assert!(a.done.is_empty(), "a higher identifier never wins arbitration against a full-load flood");
        assert_eq!(b.rx.len() as u64, bus.faults.floods[0].sent);
        assert!(bus.faults.floods[0].sent >= 36);
        // At half load there is room in between
        bus.faults.floods[0] = Flood::with_load(CanFrame::classic(0x001, false, &[0xAA; 8]), 0.5, FD, Time::from_ms(5));
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(10));
        assert_eq!(a.done.len(), 1);
        let sent = bus.faults.floods[0].sent;
        assert!((17..=20).contains(&sent), "{sent} frames of 135 us in 5 ms at 50 %");
    }

    #[test]
    fn idle_bus_reports_recessive_sequences() {
        let mut a = node(&[]);
        let mut bus = CanBus::new("can0", FD);
        for t in 1..=10 {
            bus.step(&mut [&mut a], &mut [], Time::from_us(100 * t));
        }
        // 1 ms of idle bus at 1 Mbit/s: 1000 / 11 sequences, without losing the remainders between steps
        assert_eq!(a.recessive, 90);
    }

    #[test]
    fn an_endpoint_queue_is_bounded_like_a_socketcan_interface() {
        let mut a = node(&[]);
        let mut mon = Monitor { limit: Some(ENDPOINT_TX_QUEUE), ..Monitor::default() };
        for i in 0..30 {
            mon.outbox.push((Time::from_us(i), CanFrame::classic(0x10, false, &[i as u8])));
        }
        let mut bus = CanBus::new("can0", FD);
        bus.set_stuck_dominant(true, Time::ZERO);
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_ms(1));
        assert_eq!(bus.endpoint_dropped, 20, "ENOBUFS beyond txqueuelen while the bus is stuck");
        bus.set_stuck_dominant(false, Time::from_ms(1));
        bus.step(&mut [&mut a], &mut [&mut mon], Time::from_ms(10));
        assert_eq!(a.rx.len(), 10, "the first ten go out once it is free");
        assert_eq!(a.rx[9].1.data, vec![9]);
    }

    #[test]
    fn error_passive_transmitter_suspends_and_ack_errors_add_an_error_flag() {
        struct Passive(Node);
        impl CanNode for Passive {
            fn bit_rates(&self) -> Option<BitRates> {
                self.0.bit_rates()
            }
            fn acknowledges(&self) -> bool {
                true
            }
            fn tx_request(&mut self) -> Option<TxRequest> {
                self.0.tx_request()
            }
            fn tx_done(&mut self, token: u64, at: Time, acked: bool) {
                self.0.tx_done(token, at, acked)
            }
            fn receive(&mut self, frame: &CanFrame, at: Time) {
                self.0.receive(frame, at)
            }
            fn error_passive(&self) -> bool {
                true
            }
        }
        let f = CanFrame::classic(0x10, false, &[0; 8]);
        let g = CanFrame::classic(0x20, false, &[0; 8]);
        // The passive node has the lower ID and two frames ready; the active one gets in between
        let mut a = Passive(node(&[(0, f.clone()), (0, f.clone())]));
        let mut b = node(&[(0, g.clone())]);
        let mut bus = CanBus::new("can0", FD);
        bus.record = Some(vec![]);
        bus.step(&mut [&mut a, &mut b], &mut [], Time::from_ms(1));
        let ids: Vec<u32> = bus.record.as_ref().unwrap().iter().map(|r| r.frame.id).collect();
        assert_eq!(ids, vec![0x10, 0x20, 0x10], "suspend transmission lets the higher ID go first");
        // Alone, without acknowledgement: each attempt is 6 bits longer, and 8 bits apart
        let mut a = Passive(node(&[(0, f.clone())]));
        let mut bus = CanBus::new("can0", FD);
        bus.step(&mut [&mut a], &mut [], Time::from_us(500));
        let done = &a.0.done;
        let d = f.duration(1_000_000, 1_000_000);
        assert_eq!(done[0].1, d + Time::from_us(6));
        assert_eq!(done[1].1 - done[0].1, d + Time::from_us(6 + 8));
    }
}
