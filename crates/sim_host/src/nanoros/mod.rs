//! A scripted stand-in for osu-uwrt/nano_ros's host agent (`nano_ros_agent`), for C firmware built with
//! libnanoros: it speaks the agent's side of nano-ROS over CAN without ROS 2, and decodes and encodes the topics'
//! messages from their `.msg` definitions.
//!
//! **Everything nano-ROS-specific is in this module**, one file per thing that may change upstream (see
//! `docs/nanoros.md` for which to edit when): [`wire`] (frames, CRC, registration), [`transport`] (how the byte
//! stream goes over CAN), [`cdr`] (message layout), [`msgs`] (`.msg` files), [`rostoml`] (`ros.toml`), and the
//! handshake and topic bookkeeping here. The rest of the emulator sees it only as a
//! [`crate::host_protocol::HostAgent`].
//!
//! The handshake as nano_ros_agent and libnanoros do it (as of nano_ros 9c289c6): the agent sends DISCOVER; the
//! board sends one REGISTER per topic, waiting for the agent's ACK (header only, the topic's ID) before the next;
//! then REGISTER_DONE, after which it publishes. The agent then forwards DATA both ways: the board's payloads
//! carry a CDR encapsulation header, those to the board do not (nano_ros_agent strips it from what ROS gives it).
//! nano_ros_agent sends DISCOVER once, when it starts; this mock repeats it every [`DISCOVER_INTERVAL`] until
//! the board registers (as if the agent were started after the board boots), and again after [`HostAgent::start`].
//! Messages sent before the board has registered wait for it.

pub mod cdr;
pub mod msgs;
pub mod rostoml;
pub mod transport;
pub mod wire;

use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};

use serde_json::{Value, json};
use sim_core::time::Time;
use sim_devices::can::{CanEndpoint, CanFrame};

pub use msgs::Registry;
pub use rostoml::{Direction, RosToml};

use crate::host_protocol::{HostAgent, HostMessage, HostMessageHook};
use transport::{CanmoreStream, Incoming};
use wire::{Frame, PUBLISHER_ID_START, PacketType, Registration};

/// How often the mock sends DISCOVER until the board has registered: not a multiple of the usual publish periods,
/// since libnanoros ignores a DISCOVER that comes while a publish is going out.
pub const DISCOVER_INTERVAL: Time = Time::from_ms(230);
/// Errors kept (the count goes on).
const MAX_ERRORS: usize = 200;

/// A topic as the agent knows it: from the board's registration and/or its `ros.toml`.
#[derive(Clone, Debug)]
pub struct TopicState {
    pub id: u8,
    pub name: String,
    /// `pkg/msg/Name`.
    pub msg_type: String,
    pub qos: u8,
    pub direction: Direction,
    /// The board registered it in its latest handshake.
    pub registered: bool,
    /// It is in the `ros.toml`.
    pub declared: bool,
    pub received: u64,
    pub sent: u64,
}

pub struct AgentState {
    pub client_id: u8,
    pub running: bool,
    pub config: Option<RosToml>,
    pub registry: Registry,
    /// By topic ID.
    pub topics: BTreeMap<u8, TopicState>,
    /// When the board's latest handshake finished (REGISTER_DONE), if it has.
    pub connected_at: Option<Time>,
    /// Handshakes finished.
    pub handshakes: u32,
    pub discovers: u64,
    /// Messages from the board, decoded.
    pub received: Vec<HostMessage>,
    /// Messages the agent sent the board, and when they went out.
    pub sent: Vec<HostMessage>,
    pub errors: Vec<(Time, String)>,
    pub error_count: u64,
    /// CANmore heartbeats of the client.
    pub heartbeats: u64,
    pub last_heartbeat: Option<Time>,
    pub on_message: Option<HostMessageHook>,
    /// To send once the board has registered: when (None: at once), topic, message.
    pending: Vec<(Option<Time>, String, Value)>,
    /// To send at a virtual time, in time order.
    scheduled: Vec<(Time, String, Value)>,
    /// CANmore messages ready to go, each its frames, in time order.
    outbox: VecDeque<(Time, Vec<CanFrame>)>,
    next_discover: Time,
    parser: wire::Parser,
    stream: CanmoreStream,
}

impl AgentState {
    fn error(&mut self, t: Time, e: String) {
        self.error_count += 1;
        if self.errors.len() < MAX_ERRORS {
            log::debug!("nano-ROS agent for client {}: {e}", self.client_id);
            self.errors.push((t, e));
        }
    }

    fn queue(&mut self, at: Time, frame: Frame) {
        let i = self.outbox.partition_point(|(t, _)| *t <= at);
        for (k, m) in self.stream.encode(&frame.encode()).into_iter().enumerate() {
            self.outbox.insert(i + k, (at, m));
        }
    }

    /// The topic called `name` and whether the board subscribes to it, from the registration or the `ros.toml`.
    fn topic_by_name(&self, name: &str) -> Option<(u8, String, Direction)> {
        if let Some(t) = self.topics.values().find(|t| t.name == name) {
            return Some((t.id, t.msg_type.clone(), t.direction));
        }
        let d = self.config.as_ref()?.topic(name)?;
        Some((d.id, d.msg_type.clone(), d.direction))
    }

    fn known_topics(&self) -> Vec<String> {
        let mut v: Vec<String> = self.topics.values().map(|t| t.name.clone()).collect();
        for d in self.config.iter().flat_map(|c| &c.topics) {
            if !v.contains(&d.name) {
                v.push(d.name.clone());
            }
        }
        v
    }

    /// Checks `msg` for `topic` as far as is known now; the topic's ID and type if known.
    fn check_send(&self, topic: &str, msg: &Value) -> Result<(), String> {
        match self.topic_by_name(topic) {
            Some((_, _, Direction::Publisher)) => {
                Err(format!("{topic} is one of the board's publishers: the agent only sends to its subscribers"))
            }
            Some((_, ty, Direction::Subscriber)) => cdr::encode(&self.registry, &ty, msg).map(|_| ()),
            None if self.config.is_some() || self.connected_at.is_some() => {
                Err(format!("no topic {topic:?} (the board's topics: {})", self.known_topics().join(", ")))
            }
            // Not known until the board registers
            None => Ok(()),
        }
    }

    /// Encodes and queues `msg` for `topic` at `at` (the board has registered).
    fn send_now(&mut self, at: Time, topic: &str, msg: Value) {
        let Some((id, ty, dir)) =
            self.topics.values().find(|t| t.name == topic).map(|t| (t.id, t.msg_type.clone(), t.direction))
        else {
            let known = self.known_topics().join(", ");
            return self.error(at, format!("send on {topic:?}: the board did not register it (its topics: {known})"));
        };
        if dir != Direction::Subscriber {
            return self.error(at, format!("send on {topic}: one of the board's publishers"));
        }
        match cdr::encode(&self.registry, &ty, &msg) {
            Ok(body) => {
                self.queue(at, Frame::new(PacketType::Data, id, body));
                if let Some(t) = self.topics.get_mut(&id) {
                    t.sent += 1;
                }
                self.sent.push(HostMessage { t: at, topic: topic.to_string(), msg });
            }
            Err(e) => self.error(at, format!("send on {topic}: {e}")),
        }
    }

    fn handle(&mut self, f: Frame, at: Time) {
        match f.packet_type() {
            Some(PacketType::Register) => self.register(f, at),
            Some(PacketType::RegisterDone) => {
                self.connected_at = Some(at);
                self.handshakes += 1;
                let missing: Vec<String> = self
                    .config
                    .iter()
                    .flat_map(|c| &c.topics)
                    .filter(|d| !self.topics.get(&d.id).is_some_and(|t| t.registered && t.name == d.name))
                    .map(|d| d.name.clone())
                    .collect();
                if !missing.is_empty() {
                    self.error(at, format!("the board finished registering without ros.toml's {}", missing.join(", ")));
                }
                log::info!(
                    "[{at}] nano-ROS: client {} registered {} topic(s)",
                    self.client_id,
                    self.topics.values().filter(|t| t.registered).count()
                );
            }
            Some(PacketType::Data) => self.data(f, at),
            Some(other) => self.error(at, format!("{other:?} frame from the board (topic {})", f.topic_id)),
            None => self.error(at, format!("frame of unknown packet type {:#04x}", f.kind)),
        }
    }

    fn register(&mut self, f: Frame, at: Time) {
        if self.connected_at.take().is_some() {
            // A new handshake: what was registered before is not any more
            for t in self.topics.values_mut() {
                t.registered = false;
            }
        }
        let r = match Registration::decode(&f.payload) {
            Ok(r) => r,
            Err(e) => return self.error(at, e),
        };
        if r.topic_id != f.topic_id {
            let e = format!("REGISTER frame for topic {} carries the registration of topic {}", f.topic_id, r.topic_id);
            self.error(at, e);
        }
        let direction = if r.topic_id >= PUBLISHER_ID_START { Direction::Publisher } else { Direction::Subscriber };
        let msg_type = msgs::full_name(&r.type_name).unwrap_or_else(|_| r.type_name.clone());
        if let Err(e) = self.registry.check(&msg_type) {
            self.error(at, format!("topic {}: {e}", r.topic_name));
        }
        let declared = match self.config.as_ref().map(|c| c.topic(&r.topic_name)) {
            None => false,
            Some(None) => {
                self.error(
                    at,
                    format!("the board registered {} (topic {}), which is not in ros.toml", r.topic_name, r.topic_id),
                );
                false
            }
            Some(Some(d)) => {
                if (d.id, &d.msg_type, d.direction) != (r.topic_id, &msg_type, direction) {
                    let e = format!(
                        "the board registered {} as topic {} {msg_type} ({direction:?}); ros.toml has topic {} {} ({:?})",
                        r.topic_name, r.topic_id, d.id, d.msg_type, d.direction
                    );
                    self.error(at, e);
                }
                true
            }
        };
        let old = self.topics.remove(&r.topic_id);
        let (received, sent) = old.map_or((0, 0), |o| (o.received, o.sent));
        self.topics.insert(
            r.topic_id,
            TopicState {
                id: r.topic_id,
                name: r.topic_name,
                msg_type,
                qos: r.qos,
                direction,
                registered: true,
                declared,
                received,
                sent,
            },
        );
        // nano_ros_agent's ack_client: a header-only ACK with the topic's ID
        self.queue(at, Frame::new(PacketType::Ack, r.topic_id, Vec::new()));
    }

    fn data(&mut self, f: Frame, at: Time) {
        let Some(t) = self.topics.get(&f.topic_id) else {
            return self.error(at, format!("DATA for topic {}, which the board never registered", f.topic_id));
        };
        if t.direction != Direction::Publisher {
            let e = format!("DATA from the board on {} (topic {}), one of its subscribers", t.name, f.topic_id);
            return self.error(at, e);
        }
        let (name, ty) = (t.name.clone(), t.msg_type.clone());
        match cdr::decode_payload(&self.registry, &ty, &f.payload) {
            Ok(msg) => {
                self.topics.get_mut(&f.topic_id).unwrap().received += 1;
                let m = HostMessage { t: at, topic: name, msg };
                if let Some(h) = &mut self.on_message {
                    h(&m);
                }
                self.received.push(m);
            }
            Err(e) => self.error(at, format!("{name} ({ty}): {e}")),
        }
    }
}

/// A handle to a nano-ROS mock agent; clones see the same agent.
#[derive(Clone)]
pub struct NanoRosAgent(pub Arc<Mutex<AgentState>>);

impl NanoRosAgent {
    /// An agent for CANmore client `client_id`, sending its first DISCOVER at `start`. With the board's `ros.toml`
    /// it checks the registration against it and knows the topics before the board registers them.
    pub fn new(client_id: u8, config: Option<RosToml>, registry: Registry, start: Time) -> Result<Self, String> {
        for d in config.iter().flat_map(|c| &c.topics) {
            registry.check(&d.msg_type).map_err(|e| format!("ros.toml topic {}: {e}", d.name))?;
        }
        Ok(NanoRosAgent(Arc::new(Mutex::new(AgentState {
            client_id,
            running: true,
            config,
            registry,
            topics: BTreeMap::new(),
            connected_at: None,
            handshakes: 0,
            discovers: 0,
            received: Vec::new(),
            sent: Vec::new(),
            errors: Vec::new(),
            error_count: 0,
            heartbeats: 0,
            last_heartbeat: None,
            on_message: None,
            pending: Vec::new(),
            scheduled: Vec::new(),
            outbox: VecDeque::new(),
            next_discover: start,
            parser: wire::Parser::new(),
            stream: CanmoreStream::new(client_id),
        }))))
    }

    pub fn state(&self) -> std::sync::MutexGuard<'_, AgentState> {
        self.0.lock().unwrap()
    }

    /// Messages received on `topic`.
    pub fn on_topic(&self, topic: &str) -> Vec<(Time, Value)> {
        self.state().received.iter().filter(|m| m.topic == topic).map(|m| (m.t, m.msg.clone())).collect()
    }

    fn copy(&self) -> NanoRosAgent {
        let s = self.state();
        NanoRosAgent(Arc::new(Mutex::new(AgentState {
            client_id: s.client_id,
            running: s.running,
            config: s.config.clone(),
            registry: s.registry.clone(),
            topics: s.topics.clone(),
            connected_at: s.connected_at,
            handshakes: s.handshakes,
            discovers: s.discovers,
            received: s.received.clone(),
            sent: s.sent.clone(),
            errors: s.errors.clone(),
            error_count: s.error_count,
            heartbeats: s.heartbeats,
            last_heartbeat: s.last_heartbeat,
            on_message: None,
            pending: s.pending.clone(),
            scheduled: s.scheduled.clone(),
            outbox: s.outbox.clone(),
            next_discover: s.next_discover,
            parser: s.parser.clone(),
            // A CANmore message the board was in the middle of sending is not carried over
            stream: CanmoreStream::new(s.client_id),
        })))
    }
}

impl CanEndpoint for NanoRosAgent {
    fn take_tx(&mut self, until: Time) -> Vec<(Time, CanFrame)> {
        let mut s = self.state();
        if !s.running {
            return Vec::new();
        }
        while s.connected_at.is_none() && s.next_discover <= until {
            let at = s.next_discover;
            s.queue(at, Frame::new(PacketType::Discover, 0, Vec::new()));
            s.discovers += 1;
            s.next_discover = at + DISCOVER_INTERVAL;
        }
        let due = s.scheduled.partition_point(|(t, _, _)| *t <= until);
        let due: Vec<_> = s.scheduled.drain(..due).collect();
        s.pending.extend(due.into_iter().map(|(t, topic, msg)| (Some(t), topic, msg)));
        if let Some(c) = s.connected_at {
            for (t, topic, msg) in std::mem::take(&mut s.pending) {
                let at = t.map_or(until, |t| t.max(c));
                s.send_now(at, &topic, msg);
            }
        }
        let mut out = Vec::new();
        while s.outbox.front().is_some_and(|(t, _)| *t <= until) {
            let (t, frames) = s.outbox.pop_front().unwrap();
            out.extend(frames.into_iter().map(|f| (t, f)));
        }
        out
    }

    fn receive(&mut self, frame: &CanFrame, at: Time) {
        let mut s = self.state();
        if !s.running {
            return;
        }
        match s.stream.receive(frame) {
            Incoming::Other | Incoming::Partial => {}
            Incoming::Utility(ch) => {
                if ch == canmore::CHAN_HEARTBEAT {
                    s.heartbeats += 1;
                    s.last_heartbeat = Some(at);
                }
            }
            Incoming::Bytes(b) => {
                for f in s.parser.feed(&b) {
                    s.handle(f, at);
                }
            }
            Incoming::WrongSubtype(sub) => {
                s.error(at, format!("CANmore message of subtype {sub}, not nano-ROS's 0 (titan_link firmware?)"))
            }
            Incoming::Error(e) => s.error(at, e),
        }
    }

    /// The next DISCOVER, reply or scheduled message; now if a message is waiting to go out.
    fn idle_until(&self) -> Option<Time> {
        let s = self.state();
        if !s.running {
            return Some(Time::MAX);
        }
        let mut t = Time::MAX;
        if s.connected_at.is_none() {
            t = t.min(s.next_discover);
        } else if !s.pending.is_empty() {
            return Some(Time::ZERO);
        }
        if let Some((o, _)) = s.outbox.front() {
            t = t.min(*o);
        }
        if let Some((o, _, _)) = s.scheduled.first() {
            t = t.min(*o);
        }
        Some(t)
    }
}

impl HostAgent for NanoRosAgent {
    fn protocol(&self) -> &'static str {
        "nanoros"
    }

    fn client_id(&self) -> u8 {
        self.state().client_id
    }

    fn message_count(&self) -> usize {
        self.state().received.len()
    }

    fn messages(&self, from: usize) -> Vec<HostMessage> {
        self.state().received.iter().skip(from).cloned().collect()
    }

    fn send(&self, topic: &str, msg: &Value, at: Option<Time>) -> Result<(), String> {
        let mut s = self.state();
        s.check_send(topic, msg)?;
        match at {
            None => s.pending.push((None, topic.to_string(), msg.clone())),
            Some(t) => {
                let i = s.scheduled.partition_point(|(x, _, _)| *x <= t);
                s.scheduled.insert(i, (t, topic.to_string(), msg.clone()));
            }
        }
        Ok(())
    }

    fn topics(&self) -> Vec<(String, bool)> {
        let s = self.state();
        let mut v: Vec<(String, bool)> =
            s.topics.values().map(|t| (t.name.clone(), t.direction == Direction::Publisher)).collect();
        for d in s.config.iter().flat_map(|c| &c.topics) {
            if !v.iter().any(|(n, _)| *n == d.name) {
                v.push((d.name.clone(), d.direction == Direction::Publisher));
            }
        }
        v
    }

    fn connected(&self, _now: Time) -> bool {
        self.state().connected_at.is_some()
    }

    fn status(&self, now: Time) -> Value {
        let s = self.state();
        let age = |t: Time| now.saturating_sub(t).as_secs_f64();
        let mut topics: Vec<Value> = Vec::new();
        let mut names: Vec<&str> = Vec::new();
        for t in s.topics.values() {
            names.push(&t.name);
            let last = s.received.iter().rev().chain(s.sent.iter().rev()).find(|m| m.topic == t.name);
            topics.push(json!({
                "id": t.id,
                "name": t.name,
                "type": t.msg_type,
                "direction": if t.direction == Direction::Publisher { "publisher" } else { "subscriber" },
                "qos": t.qos,
                "registered": t.registered,
                "in_ros_toml": t.declared,
                "received": t.received,
                "sent": t.sent,
                "last": last.map(|m| &m.msg),
                "last_age_s": last.map(|m| age(m.t)),
                // The board aligns 8-byte fields from its encapsulation header, ROS 2 from after it (docs/nanoros.md)
                "ros2_layout_differs": t.direction == Direction::Publisher && s.registry.has_8_byte_field(&t.msg_type),
            }));
        }
        for d in s.config.iter().flat_map(|c| &c.topics).filter(|d| !names.contains(&d.name.as_str())) {
            topics.push(json!({
                "id": d.id, "name": d.name, "type": d.msg_type,
                "direction": if d.direction == Direction::Publisher { "publisher" } else { "subscriber" },
                "registered": false, "in_ros_toml": true, "received": 0, "sent": 0,
            }));
        }
        let errors: Vec<String> = s.errors.iter().rev().take(5).map(|(t, e)| format!("[{t}] {e}")).collect();
        json!({
            "protocol": "nanoros",
            "client": s.client_id,
            "board": s.config.as_ref().map(|c| &c.board),
            "running": s.running,
            "connected": s.connected_at.is_some(),
            "connected_at_s": s.connected_at.map(|t| t.as_secs_f64()),
            "handshakes": s.handshakes,
            "discovers": s.discovers,
            "heartbeats": s.heartbeats,
            "last_heartbeat_age_s": s.last_heartbeat.map(age),
            "messages": s.received.len(),
            "sent": s.sent.len(),
            "topics": topics,
            "decode_errors": s.error_count,
            "recent_errors": errors,
            "stream": {"crc_failures": s.parser.crc_failures, "resyncs": s.parser.resyncs, "skipped_bytes": s.parser.skipped},
        })
    }

    fn errors(&self) -> Vec<String> {
        self.state().errors.iter().map(|(t, e)| format!("[{t}] {e}")).collect()
    }

    fn stop(&self) {
        self.state().running = false;
    }

    /// Starts again as nano_ros_agent starting: it knows no registration and discovers from `at`.
    fn start(&self, at: Time) {
        let mut s = self.state();
        s.running = true;
        s.connected_at = None;
        s.next_discover = at;
        for t in s.topics.values_mut() {
            t.registered = false;
        }
    }

    fn on_message(&self, mut hook: HostMessageHook) {
        let mut s = self.state();
        let mut old = s.on_message.take();
        s.on_message = Some(Box::new(move |m| {
            if let Some(o) = &mut old {
                o(m);
            }
            hook(m);
        }));
    }

    fn endpoint(&self) -> Box<dyn CanEndpoint + Send> {
        Box::new(self.clone())
    }

    fn handle(&self) -> Box<dyn HostAgent> {
        Box::new(self.clone())
    }

    fn fork(&self) -> Box<dyn HostAgent> {
        Box::new(self.copy())
    }
}

#[cfg(test)]
mod tests;
