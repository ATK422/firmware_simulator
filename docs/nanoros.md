# nano-ROS (early, experimental)

UWRT's C firmware is moving from micro-ROS (a library built in Docker) to its own **nano-ROS**,
[osu-uwrt/nano_ros](https://github.com/osu-uwrt/nano_ros): "a lightweight firmware library, codegen tool, and
host side agent". nano-ROS is young and still changing, so the emulator keeps everything it knows about it in one
module that is meant to be rewritten: this page says what nano-ROS is as of the commit read, what the emulator
does with it, what looked wrong in nano_ros, and **which files to change when nano-ROS changes**.

- [What nano-ROS is (as of 9c289c6)](#what-nano-ros-is-as-of-9c289c6)
- [What the emulator implements](#what-the-emulator-implements)
- [Using it](#using-it)
- [Findings in nano_ros](#findings-in-nano_ros)
- [When nano-ROS changes: which files](#when-nano-ros-changes-which-files)

## What nano-ROS is (as of 9c289c6)

Read at commit `9c289c60366e5746356e75f8d61a0e081f7a2c9c` (2026-10-12, "strip cdr encapsulation header on
subscription data"), the latest then. Three parts and some shared C:

| Part | What it is |
|---|---|
| `nano_ros_codegen` (Rust) | Reads a board's `ros.toml` and the `.msg` files of its topics' types (from a ROS install, `--ros-prefix`, and a directory of the team's packages, `--user-msgs`) and writes `generated/nano_ros_codegen.h` and two `.c` files next to the `ros.toml`: a C struct per message type (`sensor_msgs__msg__Imu_t`), micro-CDR serialize/deserialize functions per type, `nros_publish_TOPIC(&msg)` per publisher, `nros_set_TOPIC_subscriber_cb(cb)` per subscriber, the topic IDs (`IMU_DATA_ID`) and the topic table libnanoros registers |
| `libnanoros` (C) | `nros_init(&transport)`, `nros_spin()` (poll: read, parse, dispatch; write what is pending), `nros_publish`. Built into a titan_firmware CMake project with `add_subdirectory`; it builds Micro-CDR 2.0.2 from `external/` |
| `nano_ros_agent` (C++, ROS 2) | The host side: a transport (only serial is written; `canmore.cpp` and `tcp.cpp` are empty), the handshake, and a ROS 2 node with an `rclcpp` generic publisher or subscription per topic, passing serialized messages through without decoding them |
| `common/` (C) | The frame header (`protocol.h`) and a stream parser and encoder (`nros_stream_parser.c`), used by both ends |

**ros.toml.** `[board] name, transport` (`can` or `usb`) and `[[topics]]` with `name`, `topic_type`
(`publisher`/`subscriber`) and `msg_type = "pkg/msg/Name"`, as `nano_ros_codegen/ros.toml` and the codegen's
schema have it; the README spells them `direction`, `ros_type = "pkg/Name"` and adds `qos`, and says the naming
will change. Topic IDs are not in the file: the codegen numbers **subscribers from 0 and publishers from 127**, in
file order (`protocol.md` says publishers start at 128; the codegen and the agent's `PUBLISHER_TOPIC_ID_START =
UINT8_MAX / 2` say 127).

**Frames** (`protocol.h`, `nros_stream_parser.c`), on any byte stream: magic `AA 55`, packet type, topic ID,
payload length (`u16`, little-endian), the payload (at most 1024 bytes), and a CRC-16/CCITT-FALSE (poly 0x1021,
init 0xFFFF) of header and payload, little-endian. The parser hunts for the magic and drops frames with an
impossible length or a bad CRC. Packet types:

| Type | Direction | Payload |
|---|---|---|
| `REGISTER` (1) | board to host | topic ID, QoS byte, type name length, topic name length, then the type name (`sensor_msgs/msg/Imu`) and the topic name |
| `DATA` (2) | either | a message (below) |
| `ACK` (3) | host to board | none: the frame's topic ID is the registration acknowledged (`nros_ack_payload_t` is defined but not sent) |
| `REGISTER_DONE` (4) | board to host | none |
| `DISCOVER` (5) | host to board | none |

**Handshake.** The agent sends `DISCOVER` once, when it starts. The board answers with a `REGISTER` for its first
topic and waits for its `ACK`, then the next, and after the last a `REGISTER_DONE`; only then does
`nros_publish` work. On `REGISTER_DONE` the agent creates the ROS 2 publishers and subscriptions. There is no
keepalive, no timeout, no session or client ID (one board per agent).

**Messages** are micro-CDR (Micro-CDR 2.0.2): little-endian, every primitive aligned to its size counting from the
start of the micro-CDR buffer, `bool` one byte, a string a `u32` length counting its NUL then the bytes and the
NUL, fixed arrays element after element, nested messages in place. From the board, `nros_publish` writes the CDR
encapsulation header `00 01 00 00` (little-endian CDR) into the buffer first and the message after it, and the
agent hands the payload to ROS 2 as a serialized message as it is. To the board, the agent strips the 4-byte
header off what ROS 2 gives it, and the board deserializes the rest from its start. The codegen supports
primitives, strings (a `char[256]` in the struct), fixed arrays and nested messages; it **leaves dynamic and
bounded arrays out** of the struct and the serializer, and skips constants.

**QoS.** A QoS byte goes with each registration (`nano_ros.h` reserves bit 0 for reliable, bit 1 for
replay-on-subscribe, bits 2-4 for a retry depth), but the codegen does not set it (it is 0) and the agent uses
`SystemDefaultsQoS` for everything.

**Over CAN** nothing is written yet: libnanoros's `transport_can.c` is still the micro-ROS transport (it includes
`rmw_microros`) and the agent's CANmore transport is empty. A board brings its own `nros_transport_t` (open,
close, write, read, each returning how many bytes it moved).

## What the emulator implements

**A mock agent**, `sim_host::nanoros::NanoRosAgent`: the agent's side of the protocol without ROS 2, as a CAN
endpoint for one CANmore client. It discovers, acknowledges each registration, checks the registrations against
the board's `ros.toml` (topic names, IDs, types, directions: mismatches are errors in its status), decodes every
`DATA` from the board from the topic type's `.msg` definition into JSON, and encodes JSON into `DATA` for the
board's subscribers. Where it differs from nano_ros_agent, on purpose:

* it repeats `DISCOVER` every 230 ms until the board registers (nano_ros_agent sends one, so it has to be started
  after the board is up), and again after it is stopped and started (`stop`/`start`: nano_ros_agent restarting);
* a message sent before the board has registered waits for the registration (the agent would have no ROS
  subscription yet);
* it refuses to send to one of the board's publishers, to a topic the board does not have, and a message the
  board's struct cannot hold (a string of 256 bytes or more, an integer out of range).

**The transport on CAN** is the emulator's assumption, following micro-ROS over CAN in titan_firmware (which
libnanoros's CAN transport will most likely wrap): each write of the stream is one `canbus_msg_write`, a CANmore
message of subtype 0 in classic frames (the C driver sets its encoder and decoder up without CAN FD), client to
agent; the agent answers in CANmore messages of at most 64 bytes (libnanoros reads 64 at a time). The framing
makes message boundaries irrelevant to both parsers.

**Message types** come from `.msg` files, read the way the codegen reads them, not from hand-written structs. The
emulator carries std_msgs, builtin_interfaces, geometry_msgs, sensor_msgs (Imu, Temperature, FluidPressure,
RelativeHumidity) and riptide_msgs2's (FirmwareStatus, Depth, ElectricalReadings, ...) in
`crates/sim_host/nanoros_msgs/`, a package per directory, usable as the codegen's `--ros-prefix`;
`NANOROS_MSG_PATH` (directories, `:`-separated: a ROS install's `share/`, a workspace's `src/`) adds others.

**A test board**, `test_programs/nanoros_led`: libnanoros and the codegen's code for its `ros.toml`, both
unmodified, on titan_firmware's CAN driver, for the Talos LED board (client 6), with its own CAN transport (a
CANmore message per write; reads hand a message out piece by piece). It publishes `firmware_status`
(`riptide_msgs2/FirmwareStatus`) every second and `imu_data` (`sensor_msgs/Imu`) every 100 ms; `bit_sub`
(`std_msgs/UInt8`) is printed and echoed on `bit_echo` plus one; `imu_cmd` (`sensor_msgs/Imu`) is printed and its
orientation and frame go into the next `imu_data`. The ELF is committed; `make -C test_programs nanoros_led
NANO_ROS_DIR=~/nano_ros` rebuilds it (the same bytes) with the codegen built in that checkout.

**The protocol-neutral interface**, `sim_host::host_protocol::HostAgent`, is what the rest of the emulator sees:
messages from the board as `(time, topic, JSON)`, `send(topic, JSON, at)`, the topics, connected or not, a status
JSON, stop and start, the CAN endpoint. The titan_link mock bridge implements it too (its message variants are its
topics), so the command line (`--mock-agent PROTOCOL:...`), scenarios (`agents`, `send: {topic, msg}`,
`expect: {topic, msg}`), the dashboard (a board's `agent`, `POST /api/agent`) and the MCP server (`agent_send`, the
`agent` state section) work with either.

**Reuse or reimplement.** nano_ros_agent needs ROS 2 (`rclcpp`) and has no CAN transport, so it cannot run here or
be linked in; its protocol logic is a few dozen lines inside the ROS node. The codegen is a binary crate (no
library to depend on), and the shared C parser is 150 lines. So the mock is a reimplementation in Rust that
follows that code line by line (the parser's resync rules, the agent's header-only ACK, the codegen's `.msg`
rules), and is tested against the real thing: the unmodified libnanoros and generated code running in the
emulator.

**Not implemented:** the ROS 2 side (nothing reaches ROS; to verify locally once nano_ros_agent has a CAN
transport), QoS, the USB and Ethernet transports, several boards behind one agent.

## Using it

```bash
firmware_simulator robot robots/nanoros_led.toml --mock-agent nanoros:led --time 2s
firmware_simulator run test_programs/nanoros_led/nanoros_led.elf --board boards/talos/led_board.toml \
    --mock-agent nanoros=test_programs/nanoros_led/ros.toml --time 2s
firmware_simulator scenario scenarios/nanoros_led.yaml
```

The spec is `nanoros[:BUS][:BOARD][=ros.toml]` ([robots.md](robots.md#mock-agents-nano-ros-and-other-protocols));
a robot file's `[[board]]` may name the board's `ros_toml`. Each message the board publishes is printed as
`[t] led <- imu_data {...}`, and the run ends with the agent's summary (connected, messages, errors). Scenario steps
([scenarios.md](scenarios.md#actions)):

```yaml
agents: [nanoros:led]
steps:
  - expect: {board: led, topic: imu_data, msg: {header: {frame_id: imu_link}}, within: 1s}
  - send: {board: led, topic: bit_sub, msg: {data: 41}}
  - expect: {board: led, topic: bit_echo, msg: {data: 42}, within: 200ms}
```

The tests: `crates/sim_host/src/nanoros/` (frames, CRC, parser, CDR layout, `.msg` parsing, `ros.toml`, the agent
against a scripted board that does what `nano_ros.c` does), `crates/sim_host/tests/nanoros.rs` (the agent against
the real firmware, both ways, and an agent restart), `crates/sim_test/tests/nanoros.rs` and
`scenarios/nanoros_led.yaml` (scenarios), and `nanoros` cases in `tests/cli.rs` and `tests/mcp.rs`.

## Findings in nano_ros

What running libnanoros in the emulator, and reading nano_ros at 9c289c6, showed. Each may well be known and in
progress upstream.

1. **8-byte fields from the board are where ROS 2 does not look for them.** `nros_publish` puts the encapsulation
   header and the message in one micro-CDR buffer started with `ucdr_init_buffer`, so alignment counts from the
   header's start. ROS 2 (Fast CDR) aligns from the end of the header. A `float64`, `int64` or `uint64` lands 4 bytes
   off (at payload offset 8k, ROS 2 expects 8k + 4), so ROS 2 would misread every such message the board publishes
   (`sensor_msgs/Imu`, anything with a `Header` and doubles); 4-byte and smaller fields agree. Messages to the board
   are fine (the agent strips the header and the board aligns from the body). The mock decodes what the board
   sends and flags those topics with `ros2_layout_differs` in its status. A fix: serialize the body into a buffer
   that starts after the header (`ucdr_init_buffer` at `payload_buffer + 4`).
2. **A `DISCOVER` that comes while a publish is going out is dropped.** `register_topics_start` returns false
   unless the transmitter is idle, and nothing retries; the agent sends only one. The test board drops one in the
   restart test (`a_restarted_agent_sees_the_board_register_again_but_never_finish`).
3. **Registering again never finishes.** `registration_complete` stays true from the first handshake, so the
   board goes on publishing while it registers again; `nros_publish` clears `tx.is_registration`, and the ACK of
   the last topic then sends no `REGISTER_DONE`. A board that registered once never registers again completely
   (an agent restarted, or a second agent), so the agent never creates the ROS topics. Seen in the same test.
4. **`DATA` for a topic the board did not register, or for one of its publishers, calls a NULL function**
   (`topic_map[id]` is NULL, or its `cb` is): a HardFault from one bad frame. (Read in the code; the mock never
   sends one.)
5. **Dynamic arrays are dropped** by the codegen: a type with a `T[]` field (e.g. `sensor_msgs/BatteryState`)
   would be laid out differently from ROS 2's, with no error.
6. **Every publish prints the whole payload in hex** on stdout (and every `DATA` received), synchronously: about
   60 ms per `sensor_msgs/Imu` on a 115200-baud UART, during which the board's loop does nothing else.
7. **Padding bytes are not zeroed** (micro-CDR skips them), so whatever was in the buffer before goes out in them
   (the test board's `imu_data` carries bits of earlier topic names). Harmless to a decoder.
8. `nano_ros.c` calls `printf` without including `stdio.h` (a warning with GCC 13, an error from GCC 14), and
   `nros_dispatch` is declared `bool` and returns nothing.

## When nano-ROS changes: which files

Everything nano-ROS-specific is in `crates/sim_host/src/nanoros/`, one file per thing that may change, plus the
message definitions and the test board. Nothing outside depends on nano-ROS's details: the command line,
scenarios, the dashboard and MCP see only `HostAgent` (`crates/sim_host/src/host_protocol.rs`).

| When nano-ROS changes... | Change |
|---|---|
| The frame header, magic, CRC, length limit, packet types, the registration or ACK payload | `nanoros/wire.rs` |
| The topic ID ranges | `PUBLISHER_ID_START` in `nanoros/wire.rs` (and the numbering in `nanoros/rostoml.rs`) |
| The handshake: what the agent sends when, retries, re-registration, keepalives, client IDs, several boards | `nanoros/mod.rs` (`AgentState::handle`, `register`, `data`; `take_tx` for what goes out when) |
| The CAN transport: a CANmore subtype of its own, CAN FD, a framing of its own, chunk sizes | `nanoros/transport.rs` |
| The serialization: where alignment starts (finding 1), encapsulation, endianness, strings, or not CDR at all | `nanoros/cdr.rs` |
| What the codegen supports in `.msg` files (dynamic arrays, bounded strings, constants) | `nanoros/msgs.rs` (`parse_msg`), and `nanoros/cdr.rs` for the layout |
| New message types the team uses | a `.msg` file in `crates/sim_host/nanoros_msgs/PKG/msg/` and its name in `BUILTIN` (`nanoros/msgs.rs`), or `NANOROS_MSG_PATH` |
| `ros.toml`'s keys (the README's spelling becoming the real one), QoS names | `nanoros/rostoml.rs` |
| QoS taking effect | `TopicState::qos` in `nanoros/mod.rs` (it is recorded, not acted on) |
| libnanoros's API, its own CAN transport, its CMake layout | `test_programs/nanoros_led/main.c` (its transport goes when libnanoros has one), `CMakeLists.txt`; rebuild with `make -C test_programs nanoros_led NANO_ROS_DIR=...` and commit the ELF |
| Behaviour the tests pin (the findings, timings) | `crates/sim_host/tests/nanoros.rs`, `scenarios/nanoros_led.yaml` |
| A protocol to add next to it (another host agent) | a module implementing `HostAgent` and an arm in `host_protocol::create` and `Protocol` |

When nano_ros_agent gets a CAN transport and the robot computer runs it, the mock's transport and handshake
should be checked against it (to verify locally: vcan, ROS 2, `nano_ros_agent` against an emulated board on
`--can internal=vcan0`).
