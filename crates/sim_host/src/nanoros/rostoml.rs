//! A board's `ros.toml`: its name, transport and topics, with the topic IDs nano_ros_codegen gives them.
//!
//! Both spellings are read: the codegen's (`topic_type`, `msg_type = "pkg/msg/Name"`, as `nano_ros_codegen/ros.toml`)
//! and the README's (`direction`, `ros_type = "pkg/Name"`, `qos`), which it says will change.

use std::path::Path;

use serde::Deserialize;

use super::msgs::full_name;
use super::wire::PUBLISHER_ID_START;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Direction {
    /// The board publishes it (host side: the agent receives it).
    Publisher,
    /// The board subscribes to it (the agent sends it).
    Subscriber,
}

#[derive(Clone, Debug, Deserialize)]
struct RawTopic {
    name: String,
    #[serde(alias = "direction")]
    topic_type: Direction,
    #[serde(alias = "ros_type")]
    msg_type: String,
    qos: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
struct RawBoard {
    name: String,
    transport: String,
}

#[derive(Clone, Debug, Deserialize)]
struct Raw {
    board: RawBoard,
    #[serde(default)]
    topics: Vec<RawTopic>,
}

/// One topic of the board.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TopicDecl {
    pub name: String,
    pub direction: Direction,
    /// `pkg/msg/Name`.
    pub msg_type: String,
    pub qos: Option<String>,
    /// The ID the codegen gives it (`generate_topic_ids`).
    pub id: u8,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RosToml {
    pub board: String,
    pub transport: String,
    pub topics: Vec<TopicDecl>,
}

impl RosToml {
    pub fn parse(text: &str) -> Result<RosToml, String> {
        let raw: Raw = toml::from_str(text).map_err(|e| e.to_string())?;
        let (mut sub, mut publ) = (0u8, PUBLISHER_ID_START);
        let mut topics = Vec::new();
        for t in raw.topics {
            if topics.iter().any(|o: &TopicDecl| o.name == t.name) {
                return Err(format!("two topics called {:?}", t.name));
            }
            let id = match t.topic_type {
                Direction::Subscriber if sub < PUBLISHER_ID_START => {
                    sub += 1;
                    sub - 1
                }
                Direction::Publisher if publ < u8::MAX => {
                    publ += 1;
                    publ - 1
                }
                _ => return Err(format!("topic {:?}: out of topic IDs", t.name)),
            };
            let msg_type = full_name(&t.msg_type).map_err(|e| format!("topic {:?}: {e}", t.name))?;
            topics.push(TopicDecl { name: t.name, direction: t.topic_type, msg_type, qos: t.qos, id });
        }
        Ok(RosToml { board: raw.board.name, transport: raw.board.transport, topics })
    }

    pub fn load(path: &Path) -> Result<RosToml, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        RosToml::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn topic(&self, name: &str) -> Option<&TopicDecl> {
        self.topics.iter().find(|t| t.name == name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_topics_as_the_codegen_and_reads_both_spellings() {
        let t = RosToml::parse(
            r#"
            [board]
            name = "foo_board"
            transport = "can"
            [[topics]]
            name = "imu_data"
            direction = "publisher"
            ros_type = "sensor_msgs/Imu"
            qos = "default"
            [[topics]]
            name = "bit_sub"
            topic_type = "subscriber"
            msg_type = "std_msgs/msg/UInt8"
            [[topics]]
            name = "firmware_status"
            topic_type = "publisher"
            msg_type = "riptide_msgs2/msg/FirmwareStatus"
            "#,
        )
        .unwrap();
        assert_eq!((t.board.as_str(), t.transport.as_str()), ("foo_board", "can"));
        let ids: Vec<(&str, u8, &str)> =
            t.topics.iter().map(|t| (t.name.as_str(), t.id, t.msg_type.as_str())).collect();
        assert_eq!(
            ids,
            [
                ("imu_data", 127, "sensor_msgs/msg/Imu"),
                ("bit_sub", 0, "std_msgs/msg/UInt8"),
                ("firmware_status", 128, "riptide_msgs2/msg/FirmwareStatus")
            ]
        );
        assert_eq!(t.topic("imu_data").unwrap().qos.as_deref(), Some("default"));
    }

    #[test]
    fn the_test_programs_ros_toml() {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../test_programs/nanoros_led/ros.toml");
        let t = RosToml::load(&p).unwrap();
        assert_eq!(t.topic("bit_sub").unwrap().id, 0);
        assert_eq!(t.topic("imu_cmd").unwrap().id, 1);
        assert_eq!(t.topic("bit_echo").unwrap().id, 129);
    }

    #[test]
    fn bad_files() {
        let head = "[board]\nname = \"b\"\ntransport = \"can\"\n";
        let topic = |name: &str, ty: &str| {
            format!("[[topics]]\nname = \"{name}\"\ntopic_type = \"publisher\"\nmsg_type = \"{ty}\"\n")
        };
        assert!(RosToml::parse(&format!("{head}{}", topic("a", "Imu"))).unwrap_err().contains("pkg/msg/Name"));
        let dup = format!("{head}{}{}", topic("a", "std_msgs/Bool"), topic("a", "std_msgs/Bool"));
        assert!(RosToml::parse(&dup).unwrap_err().contains("two topics"));
        assert!(RosToml::parse("[board]\nname = \"b\"\n").is_err());
        let many: String = (0..129).map(|k| topic(&format!("t{k}"), "std_msgs/Bool")).collect();
        assert!(RosToml::parse(&format!("{head}{many}")).unwrap_err().contains("out of topic IDs"));
    }
}
