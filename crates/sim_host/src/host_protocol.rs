//! The scripted host side of a board's CAN link, whatever protocol the firmware speaks: titan_link through
//! [`crate::mock_bridge::MockBridge`] (titan_firmware_rs), nano-ROS through [`crate::nanoros::NanoRosAgent`]
//! (titan_firmware with osu-uwrt/nano_ros).
//!
//! [`HostAgent`] is what the command line, scenarios, the dashboard and the MCP server use without knowing the
//! protocol: messages from the board as `(time, topic, JSON)`, sending one by topic, a status for the dashboard,
//! and the [`CanEndpoint`] to put on the board's bus. A titan_link "topic" is a message's variant
//! (`FirmwareStatus`, `Esc`) and its JSON the variant's contents; a nano-ROS topic is a `ros.toml` topic and its
//! JSON the ROS message's fields. A new protocol is a module implementing the trait and an arm in [`create`].

use std::path::{Path, PathBuf};

use serde_json::{Value, json};
use sim_core::time::Time;
use sim_devices::can::CanEndpoint;

use crate::mock_bridge::MockBridge;

/// One message an agent got from its board.
#[derive(Clone, Debug, PartialEq)]
pub struct HostMessage {
    pub t: Time,
    pub topic: String,
    pub msg: Value,
}

/// Called with each message an agent gets from its board, as it arrives.
pub type HostMessageHook = Box<dyn FnMut(&HostMessage) + Send>;

/// A scripted host side of one CANmore client's link. Implementations are handles (clones see the same agent).
pub trait HostAgent: Send {
    /// `titan_link` or `nanoros`.
    fn protocol(&self) -> &'static str;
    fn client_id(&self) -> u8;
    /// How many messages it has got so far.
    fn message_count(&self) -> usize;
    /// The messages from the `from`th on.
    fn messages(&self, from: usize) -> Vec<HostMessage>;
    /// Sends `msg` on `topic` at the next opportunity (`at: None`) or at virtual time `at`. Checks what it can
    /// now (the topic, the message's fields) and says what is wrong.
    fn send(&self, topic: &str, msg: &Value, at: Option<Time>) -> Result<(), String>;
    /// The topics it knows the board has, `(name, sent by the board)`.
    fn topics(&self) -> Vec<(String, bool)>;
    /// Whether the board's link is up by the protocol's own measure (titan_link heartbeats in the last 300 ms;
    /// nano-ROS registration done).
    fn connected(&self, now: Time) -> bool;
    /// Its state as JSON, for the dashboard and the MCP server.
    fn status(&self, now: Time) -> Value;
    /// Problems decoding what the board sent, or with what it was asked to send.
    fn errors(&self) -> Vec<String>;
    /// Stops it (it sends and takes in nothing), like killing the host process.
    fn stop(&self);
    /// Starts it again at `at`, as the host process starting.
    fn start(&self, at: Time);
    /// Calls `hook` with every message from now on (after any hook already set).
    fn on_message(&self, hook: HostMessageHook);
    /// The endpoint to put on the board's bus.
    fn endpoint(&self) -> Box<dyn CanEndpoint + Send>;
    /// Another handle to the same agent.
    fn handle(&self) -> Box<dyn HostAgent>;
    /// An independent agent with a copy of this one's state (hooks are not copied).
    fn fork(&self) -> Box<dyn HostAgent>;
}

/// The protocols an agent can speak.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    TitanLink,
    NanoRos,
}

impl Protocol {
    pub fn parse(s: &str) -> Result<Protocol, String> {
        match s {
            "titan_link" | "titan-link" | "titan" => Ok(Protocol::TitanLink),
            "nanoros" | "nano_ros" | "nano-ros" => Ok(Protocol::NanoRos),
            other => Err(format!("unknown protocol {other:?} (titan_link or nanoros)")),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Protocol::TitanLink => "titan_link",
            Protocol::NanoRos => "nanoros",
        }
    }
}

/// `PROTOCOL[:BUS][:TARGET][=CONFIG]`, as `--mock-agent` and a scenario's `agents` give it: `nanoros:led`,
/// `nanoros:internal:led=ros.toml`, `nanoros:6=ros.toml` (a client ID), `titan_link:internal:poac`. The target is a
/// board name or a CANmore client ID (left out: the only board); the config is the protocol's (nano-ROS: the board's
/// `ros.toml`, by default the robot file's `ros_toml` for the board).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AgentSpec {
    pub protocol: Protocol,
    pub bus: Option<String>,
    pub target: Option<String>,
    pub config: Option<PathBuf>,
}

impl std::str::FromStr for AgentSpec {
    type Err = String;

    fn from_str(s: &str) -> Result<AgentSpec, String> {
        let (head, config) = match s.split_once('=') {
            Some((h, c)) if !c.is_empty() => (h, Some(PathBuf::from(c))),
            Some(_) => return Err(format!("{s:?}: nothing after `=`")),
            None => (s, None),
        };
        let parts: Vec<&str> = head.split(':').collect();
        let (protocol, bus, target) = match parts[..] {
            [p] => (p, None, None),
            [p, t] => (p, None, Some(t)),
            [p, b, t] => (p, Some(b), Some(t)),
            _ => return Err(format!("{s:?}: expected PROTOCOL[:BUS][:BOARD][=CONFIG], e.g. nanoros:led=ros.toml")),
        };
        if [bus, target].iter().flatten().any(|x| x.is_empty()) {
            return Err(format!("{s:?}: an empty bus or board"));
        }
        let protocol = Protocol::parse(protocol).map_err(|e| format!("{s:?}: {e}"))?;
        if protocol == Protocol::TitanLink && config.is_some() {
            return Err(format!("{s:?}: titan_link takes no `=CONFIG`"));
        }
        Ok(AgentSpec { protocol, bus: bus.map(str::to_string), target: target.map(str::to_string), config })
    }
}

impl std::fmt::Display for AgentSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.protocol.name())?;
        for x in [&self.bus, &self.target].into_iter().flatten() {
            write!(f, ":{x}")?;
        }
        if let Some(c) = &self.config {
            write!(f, "={}", c.display())?;
        }
        Ok(())
    }
}

/// An agent for CANmore client `client_id` on a classic or CAN FD (`fd`) bus, starting at `start`, with its
/// protocol's configuration (nano-ROS: the board's `ros.toml`, optional).
pub fn create(
    protocol: Protocol,
    client_id: u8,
    fd: bool,
    start: Time,
    config: Option<&Path>,
) -> Result<Box<dyn HostAgent>, String> {
    Ok(match protocol {
        Protocol::TitanLink => {
            let b = MockBridge::new(client_id, fd, start);
            b.set_kill(Some(false));
            Box::new(b)
        }
        Protocol::NanoRos => {
            let cfg = config.map(crate::nanoros::RosToml::load).transpose()?;
            let reg = crate::nanoros::Registry::from_env()?;
            Box::new(crate::nanoros::NanoRosAgent::new(client_id, cfg, reg, start)?)
        }
    })
}

/// Where an [`AgentSpec`] goes on a robot: the board it serves (if one is the client), its bus and CANmore client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Placement {
    pub board: Option<usize>,
    pub bus: usize,
    pub client: u8,
}

/// Works out where `spec` goes on `robot`: a board by name (its bus and client from the robot and board files), or
/// a client ID on a bus (the only bus if none is given), or with no target the robot's only board on a bus.
pub fn place(robot: &crate::robot::Robot, spec: &AgentSpec) -> Result<Placement, String> {
    let bus = spec
        .bus
        .as_ref()
        .map(|b| {
            robot
                .bus_names
                .iter()
                .position(|n| n == b)
                .ok_or_else(|| format!("{spec}: no bus {b:?} (the buses: {})", robot.bus_names.join(", ")))
        })
        .transpose()?;
    let on_bus: Vec<usize> = (0..robot.boards.len()).filter(|&i| robot.board_bus[i].is_some()).collect();
    let board = match spec.target.as_deref() {
        None if on_bus.len() == 1 => Some(on_bus[0]),
        None => {
            return Err(format!("{spec}: which board? (give PROTOCOL:BOARD; the boards: {})", robot.names.join(", ")));
        }
        Some(t) => match robot.names.iter().position(|n| n == t) {
            Some(i) => Some(i),
            None => {
                let client: u8 = t.parse().map_err(|_| {
                    format!("{spec}: no board {t:?} (the boards: {}), nor a client ID", robot.names.join(", "))
                })?;
                let j = match bus {
                    Some(j) => j,
                    None if robot.buses.len() == 1 => 0,
                    None => return Err(format!("{spec}: which bus? (give PROTOCOL:BUS:{client})")),
                };
                let i = (0..robot.boards.len())
                    .find(|&i| robot.board_bus[i] == Some(j) && robot.client_ids[i] == Some(client));
                return Ok(Placement { board: i, bus: j, client });
            }
        },
    };
    let i = board.unwrap();
    let name = &robot.names[i];
    let j = robot.board_bus[i].ok_or_else(|| format!("{spec}: {name} is not on a CAN bus"))?;
    if bus.is_some_and(|b| b != j) {
        return Err(format!("{spec}: {name} is on bus {}", robot.bus_names[j]));
    }
    let client = robot.client_ids[i].ok_or_else(|| {
        format!("{spec}: {name}'s board file gives no CANmore client ID ([can] client_id); give PROTOCOL:BUS:CLIENT")
    })?;
    Ok(Placement { board, bus: j, client })
}

/// Creates the agent `spec` names and puts it on its bus of `robot` (a relative `=CONFIG` is taken from `dir`; a
/// nano-ROS agent without one reads the board's `ros_toml` from the robot file, if it has one).
pub fn attach(
    robot: &mut crate::robot::Robot,
    spec: &AgentSpec,
    dir: &Path,
) -> Result<(Placement, Box<dyn HostAgent>), String> {
    let p = place(robot, spec)?;
    let config = match &spec.config {
        Some(c) => Some(dir.join(crate::expand_path(&c.to_string_lossy())?)),
        None => p.board.and_then(|i| robot.ros_tomls[i].clone()),
    };
    let fd = robot.buses[p.bus].bus.rates.data.is_some();
    let agent =
        create(spec.protocol, p.client, fd, Time::ZERO, config.as_deref()).map_err(|e| format!("{spec}: {e}"))?;
    robot.buses[p.bus].endpoints.push(agent.endpoint());
    Ok((p, agent))
}

/// A titan_link message as `(topic, JSON)`: its variant and the variant's contents (`null` for a unit variant).
fn split_variant(v: Value) -> (String, Value) {
    match v {
        Value::String(s) => (s, Value::Null),
        Value::Object(o) if o.len() == 1 => {
            let (k, v) = o.into_iter().next().unwrap();
            (k, v)
        }
        other => (String::new(), other),
    }
}

impl HostAgent for MockBridge {
    fn protocol(&self) -> &'static str {
        "titan_link"
    }

    fn client_id(&self) -> u8 {
        self.state().client_id
    }

    fn message_count(&self) -> usize {
        self.state().received.len()
    }

    fn messages(&self, from: usize) -> Vec<HostMessage> {
        let s = self.state();
        s.received
            .iter()
            .skip(from)
            .map(|(t, m)| {
                let (topic, msg) = split_variant(serde_json::to_value(m).unwrap_or_default());
                HostMessage { t: *t, topic, msg }
            })
            .collect()
    }

    fn send(&self, topic: &str, msg: &Value, at: Option<Time>) -> Result<(), String> {
        let v = if msg.is_null() { Value::from(topic) } else { json!({ topic: msg }) };
        let m: titan_link::HostToDevice =
            serde_json::from_value(v).map_err(|e| format!("titan_link HostToDevice {topic}: {e}"))?;
        match at {
            Some(t) => self.send_at(t, m),
            None => MockBridge::send(self, m),
        }
        Ok(())
    }

    fn topics(&self) -> Vec<(String, bool)> {
        let mut seen: Vec<(String, bool)> = Vec::new();
        for m in self.messages(0) {
            if !seen.iter().any(|(t, _)| *t == m.topic) {
                seen.push((m.topic, true));
            }
        }
        seen
    }

    fn connected(&self, now: Time) -> bool {
        self.link_heartbeats().last().is_some_and(|t| now.saturating_sub(*t) <= Time::from_ms(300))
    }

    fn status(&self, now: Time) -> Value {
        let connected = self.connected(now);
        let s = self.state();
        json!({
            "protocol": "titan_link",
            "client": s.client_id,
            "running": s.running,
            "messages": s.received.len(),
            "pings": s.pings,
            "decode_errors": s.errors.len(),
            "connected": connected,
        })
    }

    fn errors(&self) -> Vec<String> {
        self.state().errors.clone()
    }

    fn stop(&self) {
        MockBridge::stop(self)
    }

    fn start(&self, at: Time) {
        MockBridge::start(self, at)
    }

    fn on_message(&self, mut hook: HostMessageHook) {
        let mut s = self.state();
        let mut old = s.on_message.take();
        s.on_message = Some(Box::new(move |t, m| {
            if let Some(o) = &mut old {
                o(t, m);
            }
            let (topic, msg) = split_variant(serde_json::to_value(m).unwrap_or_default());
            hook(&HostMessage { t, topic, msg });
        }));
    }

    fn endpoint(&self) -> Box<dyn CanEndpoint + Send> {
        Box::new(self.clone())
    }

    fn handle(&self) -> Box<dyn HostAgent> {
        Box::new(self.clone())
    }

    fn fork(&self) -> Box<dyn HostAgent> {
        Box::new(MockBridge::fork(self))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agent_specs() {
        let p = |s: &str| s.parse::<AgentSpec>();
        let s = p("nanoros:internal:led=robots/x/ros.toml").unwrap();
        assert_eq!(s.protocol, Protocol::NanoRos);
        assert_eq!((s.bus.as_deref(), s.target.as_deref()), (Some("internal"), Some("led")));
        assert_eq!(s.config, Some(PathBuf::from("robots/x/ros.toml")));
        assert_eq!(s.to_string(), "nanoros:internal:led=robots/x/ros.toml");
        let s = p("nano_ros:6").unwrap();
        assert_eq!((s.bus, s.target.as_deref(), s.config), (None, Some("6"), None));
        assert_eq!(p("nanoros=ros.toml").unwrap().target, None);
        assert_eq!(p("titan_link:internal:poac").unwrap().protocol, Protocol::TitanLink);
        assert!(p("titan_link:poac=x.toml").unwrap_err().contains("no `=CONFIG`"));
        assert!(p("mqtt:x").unwrap_err().contains("unknown protocol"));
        assert!(p("nanoros:a:b:c").is_err());
        assert!(p("nanoros::b").is_err());
        assert!(p("nanoros:b=").is_err());
    }

    /// The titan_link mock bridge through the protocol-neutral interface: variants as topics.
    #[test]
    fn a_mock_bridge_as_a_host_agent() {
        use canmore::MsgEncoder;
        use canmore::id::Direction;
        use sim_devices::can::CanFrame;
        let b = create(Protocol::TitanLink, 1, true, Time::ZERO, None).unwrap();
        assert_eq!((b.protocol(), b.client_id()), ("titan_link", 1));
        let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = seen.clone();
        b.on_message(Box::new(move |m| log.lock().unwrap().push(m.topic.clone())));
        let mut ep = b.endpoint();
        let mut buf = [0u8; 64];
        let enc = titan_link::encode(&titan_link::DeviceToHost::Heartbeat, &mut buf).unwrap();
        for f in MsgEncoder::new(1, Direction::ClientToAgent, true, canmore::MSG_SUBTYPE_TITAN_LINK, enc) {
            ep.receive(&CanFrame::fd(f.id, f.extended, f.data()), Time::from_ms(5));
        }
        let m = b.messages(0);
        assert_eq!((m[0].topic.as_str(), &m[0].msg), ("Heartbeat", &Value::Null));
        assert_eq!(*seen.lock().unwrap(), ["Heartbeat"]);
        assert!(b.connected(Time::from_ms(100)));
        assert!(!b.connected(Time::from_ms(400)));
        b.send("KillSwitch", &json!({"asserting_kill": true}), Some(Time::from_ms(50))).unwrap();
        assert!(b.send("KillSwitch", &json!({"nope": 1}), None).unwrap_err().contains("HostToDevice"));
        assert_eq!(b.fork().message_count(), 1);
        assert_eq!(b.status(Time::from_ms(10))["protocol"], "titan_link");
    }
}
