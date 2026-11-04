//! The mock agent against a scripted board that does what libnanoros's `nano_ros.c` does (as of 9c289c6), frame
//! by frame over CANmore: registration on DISCOVER, one topic per ACK, REGISTER_DONE, then DATA both ways.

use super::*;
use canmore::id::Direction as CanDir;
use canmore::{MsgDecoder, MsgEncoder};

/// The board's side: its topics `(id, type, name)`, the stream parser, what it got.
struct Board {
    client: u8,
    topics: Vec<(u8, &'static str, &'static str)>,
    decoder: MsgDecoder,
    parser: wire::Parser,
    reg_idx: Option<usize>,
    registered: bool,
    got: Vec<Frame>,
}

impl Board {
    fn new(client: u8) -> Board {
        Board {
            client,
            topics: vec![
                (0, "std_msgs/msg/UInt8", "bit_sub"),
                (127, "riptide_msgs2/msg/FirmwareStatus", "firmware_status"),
                (128, "sensor_msgs/msg/Imu", "imu_data"),
            ],
            decoder: MsgDecoder::new(false),
            parser: wire::Parser::new(),
            reg_idx: None,
            registered: false,
            got: Vec::new(),
        }
    }

    /// One `canbus_msg_write` of `frame`, as frames.
    fn send(&self, frame: Frame) -> Vec<CanFrame> {
        MsgEncoder::new(self.client, CanDir::ClientToAgent, false, 0, &frame.encode())
            .map(|f| CanFrame::classic(f.id, f.extended, f.data()))
            .collect()
    }

    fn register_frame(&self, k: usize) -> Frame {
        let (id, ty, name) = self.topics[k];
        let r = Registration { topic_id: id, qos: 0, type_name: ty.into(), topic_name: name.into() };
        Frame::new(PacketType::Register, id, r.encode())
    }

    /// What the board answers to the agent's frames (nros_dispatch).
    fn take(&mut self, frames: &[(Time, CanFrame)]) -> Vec<CanFrame> {
        let mut out = Vec::new();
        for (_, f) in frames {
            let Some(m) = self.decoder.decode_frame(f.id, f.extended, &f.data).unwrap() else { continue };
            assert!(m.data.len() <= transport::HOST_CHUNK);
            for fr in self.parser.feed(m.data) {
                match fr.packet_type().unwrap() {
                    PacketType::Discover => {
                        self.reg_idx = Some(0);
                        out.extend(self.send(self.register_frame(0)));
                    }
                    PacketType::Ack => {
                        let k = self.reg_idx.unwrap() + 1;
                        self.reg_idx = Some(k);
                        if k < self.topics.len() {
                            out.extend(self.send(self.register_frame(k)));
                        } else {
                            out.extend(self.send(Frame::new(PacketType::RegisterDone, 0, Vec::new())));
                            self.registered = true;
                        }
                    }
                    _ => self.got.push(fr),
                }
            }
        }
        out
    }

    fn publish(&self, id: u8, ty: &str, msg: &Value) -> Vec<CanFrame> {
        let mut payload = cdr::ENCAPSULATION.to_vec();
        // As nros_publish: the body serialized after the header in the same micro-CDR buffer
        let body = encode_after_header(&Registry::builtin(), ty, msg);
        payload.extend(body);
        self.send(Frame::new(PacketType::Data, id, payload))
    }
}

/// A message laid out as the board does it: aligned from the start of the encapsulation header.
fn encode_after_header(reg: &Registry, ty: &str, msg: &Value) -> Vec<u8> {
    // Encoding with 4 bytes of something 4-aligned first gives the same layout, then drop those 4 bytes
    let mut r = reg.clone();
    let t = r.get(ty).unwrap().clone();
    let mut text = String::from("uint32 encapsulation_header\n");
    for f in &t.fields {
        let ty = match &f.ty {
            msgs::FieldType::Prim(p) => prim_name(*p).to_string(),
            msgs::FieldType::PrimArray(p, n) => format!("{}[{n}]", prim_name(*p)),
            msgs::FieldType::Nested(n) => n.replace("/msg/", "/"),
            msgs::FieldType::NestedArray(n, k) => format!("{}[{k}]", n.replace("/msg/", "/")),
        };
        text.push_str(&format!("{ty} {}\n", f.name));
    }
    r.add("probe/msg/WithHeader", &text).unwrap();
    let mut m = msg.as_object().cloned().unwrap_or_default();
    m.insert("encapsulation_header".into(), json!(0));
    cdr::encode(&r, "probe/msg/WithHeader", &Value::Object(m)).unwrap()[4..].to_vec()
}

fn prim_name(p: msgs::Prim) -> &'static str {
    use msgs::Prim::*;
    match p {
        Bool => "bool",
        Byte => "byte",
        Char => "char",
        Float32 => "float32",
        Float64 => "float64",
        Int8 => "int8",
        Int16 => "int16",
        Int32 => "int32",
        Int64 => "int64",
        Uint8 => "uint8",
        Uint16 => "uint16",
        Uint32 => "uint32",
        Uint64 => "uint64",
        String => "string",
    }
}

fn config() -> RosToml {
    RosToml::parse(
        "[board]\nname = \"test\"\ntransport = \"can\"\n\
         [[topics]]\nname = \"bit_sub\"\ntopic_type = \"subscriber\"\nmsg_type = \"std_msgs/msg/UInt8\"\n\
         [[topics]]\nname = \"firmware_status\"\ntopic_type = \"publisher\"\nmsg_type = \"riptide_msgs2/msg/FirmwareStatus\"\n\
         [[topics]]\nname = \"imu_data\"\ndirection = \"publisher\"\nros_type = \"sensor_msgs/Imu\"\n",
    )
    .unwrap()
}

/// Runs agent and board against each other up to `until`, a millisecond at a time.
fn exchange(agent: &mut NanoRosAgent, board: &mut Board, from: Time, until: Time) {
    let mut t = from;
    while t <= until {
        let tx = agent.take_tx(t);
        for f in board.take(&tx) {
            agent.receive(&f, t);
        }
        t += Time::from_ms(1);
    }
}

#[test]
fn handshake_then_data_both_ways() {
    let mut agent = NanoRosAgent::new(6, Some(config()), Registry::builtin(), Time::from_ms(10)).unwrap();
    let mut board = Board::new(6);
    assert_eq!(agent.idle_until(), Some(Time::from_ms(10)), "the first DISCOVER");
    assert!(agent.take_tx(Time::from_ms(9)).is_empty());
    // Sent before the board registers: waits for it
    agent.send("bit_sub", &json!({"data": 41}), None).unwrap();
    exchange(&mut agent, &mut board, Time::from_ms(10), Time::from_ms(20));
    assert!(board.registered);
    assert!(HostAgent::connected(&agent, Time::from_ms(20)));
    {
        let s = agent.state();
        assert_eq!(s.connected_at, Some(Time::from_ms(13)), "a topic per millisecond: each ACK goes out at the next");
        assert_eq!((s.handshakes, s.discovers), (1, 1));
        assert!(s.topics.values().all(|t| t.registered && t.declared));
        assert!(s.errors.is_empty(), "{:?}", s.errors);
    }
    // The board got the pending message, without an encapsulation header
    let data: Vec<&Frame> = board.got.iter().filter(|f| f.packet_type() == Some(PacketType::Data)).collect();
    assert_eq!(data.len(), 1);
    assert_eq!((data[0].topic_id, data[0].payload.as_slice()), (0, &[41u8][..]));
    // The board publishes; the agent decodes
    let st = json!({"board_name": "nanoros_led", "client_id": 6, "uptime_ms": 1234, "faults": 2});
    for f in board.publish(127, "riptide_msgs2/msg/FirmwareStatus", &st) {
        agent.receive(&f, Time::from_ms(30));
    }
    let imu = json!({"header": {"stamp": {"sec": 1, "nanosec": 5}, "frame_id": "imu_link"},
        "orientation": {"w": 1.0}, "linear_acceleration": {"z": 9.81}});
    for f in board.publish(128, "sensor_msgs/msg/Imu", &imu) {
        agent.receive(&f, Time::from_ms(31));
    }
    let got = agent.messages(0);
    assert_eq!(got.len(), 2, "{:?}", agent.errors());
    assert_eq!((got[0].t, got[0].topic.as_str()), (Time::from_ms(30), "firmware_status"));
    assert_eq!(got[0].msg["uptime_ms"], 1234);
    assert_eq!(got[0].msg["faults"], 2);
    assert_eq!(got[1].msg["orientation"]["w"], 1.0);
    assert_eq!(got[1].msg["linear_acceleration"]["z"], 9.81);
    assert_eq!(got[1].msg["header"]["frame_id"], "imu_link");
    let status = agent.status(Time::from_ms(40));
    assert_eq!(status["connected"], true);
    let topics = status["topics"].as_array().unwrap();
    let imu_topic = topics.iter().find(|t| t["name"] == "imu_data").unwrap();
    assert_eq!(imu_topic["ros2_layout_differs"], true, "an Imu has float64s");
    assert_eq!(topics.iter().find(|t| t["name"] == "firmware_status").unwrap()["ros2_layout_differs"], false);
    // Scheduled sends go out at their time
    agent.send("bit_sub", &json!({"data": 7}), Some(Time::from_ms(50))).unwrap();
    assert_eq!(agent.idle_until(), Some(Time::from_ms(50)));
    assert!(agent.take_tx(Time::from_ms(49)).is_empty());
    let tx = agent.take_tx(Time::from_ms(50));
    assert!(!tx.is_empty() && tx.iter().all(|(t, _)| *t == Time::from_ms(50)));
    assert_eq!(agent.idle_until(), Some(Time::MAX), "registered, nothing left to send");
}

#[test]
fn discovers_until_the_board_answers_and_again_after_a_restart() {
    let mut agent = NanoRosAgent::new(6, None, Registry::builtin(), Time::ZERO).unwrap();
    // Nobody answers for a while
    let tx = agent.take_tx(Time::from_ms(600));
    assert_eq!(tx.len(), 3, "DISCOVER at 0, 230 and 460 ms, one classic frame each");
    assert_eq!(agent.state().discovers, 3);
    let mut board = Board::new(6);
    exchange(&mut agent, &mut board, Time::from_ms(750), Time::from_ms(760));
    assert!(HostAgent::connected(&agent, Time::ZERO));
    assert_eq!(agent.state().discovers, 4);
    // Without a ros.toml the topics come from the registration
    assert_eq!(HostAgent::topics(&agent).len(), 3);
    assert!(agent.send("nope", &Value::Null, None).unwrap_err().contains("no topic \"nope\""));
    // Stopped, it neither sends nor takes in
    agent.stop();
    assert_eq!(agent.idle_until(), Some(Time::MAX));
    agent.start(Time::from_ms(900));
    assert!(!HostAgent::connected(&agent, Time::ZERO));
    let mut board = Board::new(6);
    exchange(&mut agent, &mut board, Time::from_ms(900), Time::from_ms(905));
    let s = agent.state();
    assert_eq!(s.handshakes, 2);
    assert!(s.topics.values().all(|t| t.registered));
}

#[test]
fn checks_what_it_sends_and_what_the_board_registers() {
    let agent = NanoRosAgent::new(6, Some(config()), Registry::builtin(), Time::ZERO).unwrap();
    let e = |topic: &str, msg: Value| agent.send(topic, &msg, None).unwrap_err();
    assert!(e("firmware_status", Value::Null).contains("publishers"));
    assert!(e("bit_sub", json!({"data": 300})).contains("out of range"));
    assert!(e("bits", Value::Null).contains("bit_sub"));
    // A board whose registration differs from its ros.toml
    let mut agent = NanoRosAgent::new(6, Some(config()), Registry::builtin(), Time::ZERO).unwrap();
    let mut board = Board::new(6);
    board.topics[1].1 = "std_msgs/msg/UInt32";
    board.topics.push((129, "std_msgs/msg/Bool", "extra"));
    exchange(&mut agent, &mut board, Time::ZERO, Time::from_ms(3));
    let errors = agent.errors().join("\n");
    assert!(errors.contains("firmware_status as topic 127 std_msgs/msg/UInt32"), "{errors}");
    assert!(errors.contains("extra (topic 129), which is not in ros.toml"), "{errors}");
    // DATA for a topic never registered, and from one of the board's subscribers
    for (id, why) in [(200, "never registered"), (0, "one of its subscribers")] {
        for f in board.send(Frame::new(PacketType::Data, id, vec![0, 1, 0, 0, 1])) {
            agent.receive(&f, Time::from_ms(5));
        }
        assert!(agent.errors().last().unwrap().contains(why), "{:?}", agent.errors());
    }
    // A payload the type does not fit
    for f in board.send(Frame::new(PacketType::Data, 128, vec![0, 1, 0, 0, 1])) {
        agent.receive(&f, Time::from_ms(6));
    }
    assert!(agent.errors().last().unwrap().contains("imu_data (sensor_msgs/msg/Imu)"));
    assert_eq!(agent.message_count(), 0);
    // A titan_link board on the same client
    let mut buf = [0u8; 64];
    let enc = titan_link::encode(&titan_link::DeviceToHost::Heartbeat, &mut buf).unwrap();
    for f in MsgEncoder::new(6, CanDir::ClientToAgent, false, canmore::MSG_SUBTYPE_TITAN_LINK, enc) {
        agent.receive(&CanFrame::classic(f.id, f.extended, f.data()), Time::from_ms(7));
    }
    assert!(agent.errors().last().unwrap().contains("subtype 1"));
}

#[test]
fn a_fork_carries_on_independently() {
    let mut agent = NanoRosAgent::new(6, Some(config()), Registry::builtin(), Time::ZERO).unwrap();
    let mut board = Board::new(6);
    exchange(&mut agent, &mut board, Time::ZERO, Time::from_ms(5));
    let fork = HostAgent::fork(&agent);
    assert!(fork.connected(Time::ZERO));
    fork.send("bit_sub", &json!({"data": 1}), None).unwrap();
    assert_eq!(agent.idle_until(), Some(Time::MAX), "the original has nothing to send");
    assert_eq!(fork.endpoint().idle_until(), Some(Time::ZERO));
    let seen = Arc::new(Mutex::new(0));
    let n = seen.clone();
    HostAgent::on_message(&agent, Box::new(move |_| *n.lock().unwrap() += 1));
    for f in board.publish(127, "riptide_msgs2/msg/FirmwareStatus", &json!({})) {
        agent.receive(&f, Time::from_ms(6));
    }
    assert_eq!((*seen.lock().unwrap(), agent.message_count(), fork.message_count()), (1, 1, 0));
}
