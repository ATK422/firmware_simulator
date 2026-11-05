//! `titan_bridge` command lines for the boards of a robot file: one bridge process per Rust board, with the
//! `--board` profile, `--interface`, `--namespace` and `--client-id` it needs (titan_firmware_rs's
//! `host/titan_bridge`, whose profiles are mirrored in [`PROFILES`]), as a list, a shell script or a ROS 2 launch
//! file.
//!
//! Which profile a board gets follows from its firmware image (the titan_firmware_rs crate name: `esc_board`,
//! `ivc_board2`, ...). Two boards choose their client ID at run time: an ESC board reads its ID strap on GPIO 29
//! (high is board 1), taken from the robot file's or board file's `gpio` entries, and a smart battery uses the
//! serial number in its last flash sector, taken from the board file's `[[flash]]` entry. [`probe`] instead boots
//! the robot and reads each board's client ID from its CANmore heartbeats, which also covers images the static
//! rules don't know.

use std::fmt::Write as _;
use std::path::Path;

use canmore::id::{Direction, Fields, FrameType};
use sim_core::time::Time;
use sim_devices::can::{Monitor, Sender};

use crate::board::Board;
use crate::robot::{Robot, RobotSpec};

/// A `titan_bridge --board` profile (titan_bridge's `host/titan_bridge/src/boards/*.rs`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Profile {
    /// `--board` name.
    pub name: &'static str,
    /// The titan_firmware_rs firmware crate (image name) the profile serves.
    pub firmware: &'static str,
    /// Default ROS namespace.
    pub namespace: &'static str,
    /// Default CANmore client ID.
    pub client_id: u8,
    /// Reached over UDP (Puddles) rather than CAN.
    pub udp: bool,
}

const fn p(name: &'static str, firmware: &'static str, namespace: &'static str, client_id: u8) -> Profile {
    Profile { name, firmware, namespace, client_id, udp: false }
}

/// titan_bridge's board profiles (its `BOARDS`), with the firmware each serves.
pub const PROFILES: &[Profile] = &[
    p("ivc", "ivc_board", "talos", 8),
    p("ivc2", "ivc_board2", "talos", 8),
    p("ivc_dac", "ivc_dac", "talos", 8),
    p("actuator", "actuator_board", "talos", 5),
    p("camera_cage", "camera_cage_bb", "talos", 4),
    p("esc0", "esc_board", "talos", 2),
    p("esc1", "esc_board", "talos", 3),
    p("led", "led_board", "talos", 6),
    p("led2", "led_board2", "talos", 6),
    p("poac", "poac_board", "talos", 1),
    p("power", "power_board", "talos", 1),
    p("smart_battery", "smart_battery", "talos", 1),
    p("sbc", "sbc_board", "talos", 7),
    Profile { name: "puddles_backplane", firmware: "puddles_backplane", namespace: "puddles", client_id: 1, udp: true },
    p("hbridge", "hbridge_board", "liltank", 2),
    p("po_half", "po_half_board", "liltank", 1),
];

/// The ESC board's ID strap.
const ESC_ID_PIN: u8 = 29;
/// Where the smart battery's serial number (its client ID) is programmed: the last sector of its 2 MB flash.
const SMART_BATTERY_SERIAL_ADDR: u32 = 0x101F_F000;

/// The bridge for one board of a robot.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct BridgeCommand {
    /// The board's name in the robot file.
    pub board: String,
    /// The firmware image name.
    pub image: String,
    /// The robot bus and the interface the bridge opens.
    pub bus: Option<String>,
    pub iface: Option<String>,
    /// The titan_bridge profile, if the board has one.
    pub profile: Option<Profile>,
    pub namespace: String,
    pub client_id: Option<u8>,
    /// Why there is no command, or something to know about it.
    pub note: Option<String>,
}

impl BridgeCommand {
    /// The command's arguments (after the program name), if the board can be bridged.
    pub fn args(&self) -> Option<Vec<String>> {
        let (Some(p), Some(iface), Some(client)) = (self.profile, &self.iface, self.client_id) else { return None };
        if p.udp {
            return None;
        }
        Some(
            ["--board", p.name, "--interface", iface, "--namespace", &self.namespace, "--client-id"]
                .iter()
                .map(|s| s.to_string())
                .chain([client.to_string()])
                .collect(),
        )
    }
}

/// The image name of a firmware path: its file name without an `.elf`/`.uf2` extension.
pub fn image_name(path: &Path) -> String {
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    for ext in [".elf", ".uf2"] {
        if let Some(stem) = name.strip_suffix(ext) {
            return stem.to_string();
        }
    }
    name
}

/// Reads a robot file without booting its boards.
pub fn read_spec(path: &Path) -> Result<RobotSpec, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Works out each board's bridge from the robot file alone. `ifaces` maps bus names to interfaces; a bus not in it
/// gets `vcan<N>`, N its position in the robot file. `namespace` replaces the profiles' namespaces.
pub fn plan(
    spec: &RobotSpec,
    dir: &Path,
    ifaces: &[(String, String)],
    namespace: Option<&str>,
) -> Result<Vec<BridgeCommand>, String> {
    let iface_of = |bus: &str| -> Option<String> {
        if let Some((_, i)) = ifaces.iter().find(|(b, _)| b == bus) {
            return Some(i.clone());
        }
        spec.bus.iter().position(|b| b.name == bus).map(|n| format!("vcan{n}"))
    };
    for (b, _) in ifaces {
        if !spec.bus.iter().any(|s| &s.name == b) {
            return Err(format!("the robot has no bus {b:?}"));
        }
    }
    let mut out = Vec::new();
    for b in &spec.board {
        let board = Board::load(&dir.join(&b.board))?;
        let image = image_name(&b.elf);
        let bus =
            b.bus.clone().or_else(|| (spec.bus.len() == 1 && board.can.is_some()).then(|| spec.bus[0].name.clone()));
        let mut cmd = BridgeCommand {
            board: b.name.clone(),
            image: image.clone(),
            iface: bus.as_deref().and_then(iface_of),
            bus,
            profile: None,
            namespace: String::new(),
            client_id: None,
            note: None,
        };
        let candidates: Vec<&Profile> = PROFILES.iter().filter(|p| p.firmware == image).collect();
        let profile = match image.as_str() {
            "esc_board" => {
                // The robot file's straps replace the board file's for the same pin
                let strap = b.gpio.iter().chain(board.gpio.iter()).find(|g| g.pin == ESC_ID_PIN);
                let high = strap.and_then(|g| g.level.as_deref()) == Some("high");
                candidates.into_iter().find(|p| p.name == if high { "esc1" } else { "esc0" })
            }
            _ => candidates.into_iter().next(),
        };
        match profile {
            Some(p) => {
                cmd.profile = Some(*p);
                cmd.namespace = namespace.unwrap_or(p.namespace).to_string();
                cmd.client_id = Some(p.client_id);
                if p.name == "smart_battery" {
                    let serial = board
                        .flash
                        .iter()
                        .find(|f| f.address == SMART_BATTERY_SERIAL_ADDR)
                        .and_then(|f| f.bytes.first().copied());
                    cmd.client_id = serial;
                    if serial.is_none() {
                        cmd.note = Some("no serial number in the board file's flash; give --client-id <serial> by hand, or use --probe".into());
                    }
                }
                if p.udp {
                    cmd.note = Some(
                        "reached over UDP: run the board with `firmware_simulator run --udp LOCAL=PEER` and \
                         `titan_bridge --board puddles_backplane --interface LOCAL,PEER`"
                            .into(),
                    );
                } else if cmd.iface.is_none() {
                    cmd.note = Some("not on a CAN bus".into());
                }
            }
            None => {
                cmd.namespace = namespace.unwrap_or("talos").to_string();
                cmd.note = Some(format!(
                    "no titan_bridge profile for image {image:?} (a C board needs the micro-ROS agent instead; an \
                     example image has no ROS side)"
                ));
            }
        }
        out.push(cmd);
    }
    Ok(out)
}

/// Boots the robot for `time` with a listener on every bus and returns the CANmore client ID each board sent its
/// heartbeat as (`None` for a board that sent none).
pub fn probe(robot: &mut Robot, time: Time) -> Result<Vec<Option<u8>>, String> {
    for rb in &mut robot.buses {
        rb.bus.record = Some(Vec::new());
        // A listener acknowledges frames, as on a robot with the rest of its boards and computer
        rb.endpoints.push(Box::new(Monitor::default()));
    }
    if let Some((i, why)) = robot.run_until(time) {
        return Err(format!("{} stopped: {why:?}", robot.names[i]));
    }
    let mut clients = vec![None; robot.boards.len()];
    for (j, rb) in robot.buses.iter().enumerate() {
        // The bus's nodes are its boards' CAN controllers, in board order (as `Robot::run_until` lists them)
        let mut node_board = Vec::new();
        for (i, b) in robot.boards.iter_mut().enumerate() {
            if robot.board_bus[i] == Some(j) {
                node_board.extend(std::iter::repeat_n(i, b.machine.can_nodes().len()));
            }
        }
        for rec in rb.bus.record.as_deref().unwrap_or(&[]) {
            let Sender::Node(n) = rec.sender else { continue };
            let f = Fields::decode(rec.frame.id, rec.frame.extended);
            if !rec.frame.extended
                && f.frame_type == FrameType::Util
                && f.noc == canmore::CHAN_HEARTBEAT
                && f.direction == Direction::ClientToAgent
                && let Some(&i) = node_board.get(n)
            {
                clients[i].get_or_insert(f.client_id);
            }
        }
    }
    Ok(clients)
}

/// Applies probed client IDs to a plan: a probed ID replaces the static one (with a note when they differ), and
/// picks the ESC profile that matches it.
pub fn apply_probe(cmds: &mut [BridgeCommand], probed: &[Option<u8>]) {
    for (c, p) in cmds.iter_mut().zip(probed) {
        match (*p, c.client_id) {
            (None, _) => {
                if c.args().is_some() {
                    c.note = Some("no CANmore heartbeat seen while probing".into());
                }
            }
            (Some(id), old) => {
                if c.image == "esc_board"
                    && let Some(esc) = PROFILES.iter().find(|p| p.firmware == "esc_board" && p.client_id == id)
                {
                    c.profile = Some(*esc);
                }
                if old.is_some_and(|o| o != id) {
                    c.note = Some(format!("the robot file suggests client {}, the board sent {id}", old.unwrap()));
                }
                c.client_id = Some(id);
            }
        }
    }
}

fn shell_quote(s: &str) -> String {
    if !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || "-_./:=,@".contains(c)) {
        s.to_string()
    } else {
        format!("'{}'", s.replace('\'', r"'\''"))
    }
}

/// The emulator command that puts the robot on the interfaces the bridges use.
pub fn emulator_command(robot_file: &str, cmds: &[BridgeCommand]) -> String {
    let mut pairs: Vec<(String, String)> = Vec::new();
    for c in cmds {
        if let (Some(b), Some(i)) = (&c.bus, &c.iface)
            && !pairs.iter().any(|(pb, _)| pb == b)
        {
            pairs.push((b.clone(), i.clone()));
        }
    }
    let mut s = format!("firmware_simulator robot {}", shell_quote(robot_file));
    for (b, i) in pairs {
        match cannelloni_addr(&i) {
            Some(addr) => {
                // The hub binds the address the bridges send to, or every address for a host name (the bridges'
                // name for the emulator's machine, e.g. host.docker.internal)
                let local = match addr.parse::<std::net::SocketAddr>() {
                    Ok(a) => a.to_string(),
                    Err(_) => format!("0.0.0.0:{}", addr.rsplit_once(':').map(|(_, p)| p).unwrap_or("20000")),
                };
                let _ = write!(s, " --cannelloni {}", shell_quote(&format!("{b}={local}")));
            }
            None => {
                let _ = write!(s, " --can {}", shell_quote(&format!("{b}={i}")));
            }
        }
    }
    s + " --realtime"
}

/// The emulator's hub address in a titan_bridge interface `cannelloni:HOST:PORT` (CAN over UDP, no vcan), or
/// `None` for a SocketCAN interface.
pub fn cannelloni_addr(iface: &str) -> Option<&str> {
    iface.strip_prefix(CANNELLONI_PREFIX)
}

/// How a titan_bridge `--interface` names CAN over UDP to the emulator's `--cannelloni` hub, instead of a SocketCAN
/// interface (titan_firmware_rs's titan_bridge from the change that added it on).
pub const CANNELLONI_PREFIX: &str = "cannelloni:";

/// Interfaces the bridges use, in order.
fn interfaces(cmds: &[BridgeCommand]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for c in cmds.iter().filter(|c| c.args().is_some()) {
        let i = c.iface.clone().unwrap();
        if !out.contains(&i) {
            out.push(i);
        }
    }
    out
}

/// One line per bridge, with a comment naming the board (and a note for boards without one).
pub fn render_lines(cmds: &[BridgeCommand], program: &str) -> String {
    let mut o = String::new();
    for c in cmds {
        let on = match (&c.bus, &c.iface) {
            (Some(b), Some(i)) => format!(" on {b} ({i})"),
            _ => String::new(),
        };
        let _ = writeln!(
            o,
            "# {} ({}){on}{}",
            c.board,
            c.image,
            c.note.as_ref().map(|n| format!(": {n}")).unwrap_or_default()
        );
        if let Some(args) = c.args() {
            let args: Vec<String> = args.iter().map(|a| shell_quote(a)).collect();
            let _ = writeln!(o, "{} {}", shell_quote(program), args.join(" "));
        }
    }
    o
}

/// A bash script that checks the interfaces, starts every bridge with its output prefixed by the board name, and
/// stops them all when it is interrupted or one of them exits.
pub fn render_script(cmds: &[BridgeCommand], program: &str, robot_file: &str) -> String {
    let mut o = String::new();
    let _ = writeln!(o, "#!/usr/bin/env bash");
    let _ = writeln!(o, "# Generated by `firmware_simulator bridges {robot_file}`: one titan_bridge per board.");
    let _ = writeln!(o, "# Start the emulator first (in another terminal):");
    let _ = writeln!(o, "#   {}", emulator_command(robot_file, cmds));
    let _ = writeln!(o, "# and source the ROS 2 environment titan_bridge was built in (riptide-shell).");
    let _ = writeln!(o, "set -euo pipefail\n");
    let _ = writeln!(o, "TITAN_BRIDGE=\"${{TITAN_BRIDGE:-{}}}\"", program.replace(['"', '$', '`', '\\'], ""));
    // SocketCAN interfaces must exist; CAN over UDP needs nothing on this machine
    let ifaces: Vec<String> = interfaces(cmds).into_iter().filter(|i| cannelloni_addr(i).is_none()).collect();
    if !ifaces.is_empty() {
        let _ = writeln!(o, "for iface in {}; do", ifaces.iter().map(|i| shell_quote(i)).collect::<Vec<_>>().join(" "));
        let _ = writeln!(o, "    if ! ip link show \"$iface\" >/dev/null 2>&1; then");
        let _ = writeln!(o, "        echo \"no interface $iface (scripts/vcan_up.sh creates vcan interfaces)\" >&2");
        let _ = writeln!(o, "        exit 1");
        let _ = writeln!(o, "    fi");
        let _ = writeln!(o, "done\n");
    }
    let _ = writeln!(o, "pids=()");
    let _ = writeln!(o, "stop() {{");
    let _ = writeln!(o, "    trap - EXIT INT TERM");
    let _ = writeln!(o, "    if [ \"${{#pids[@]}}\" -gt 0 ]; then kill \"${{pids[@]}}\" 2>/dev/null || true; fi");
    let _ = writeln!(o, "    wait || true");
    let _ = writeln!(o, "}}");
    let _ = writeln!(o, "trap stop EXIT INT TERM\n");
    let _ = writeln!(o, "# start NAME ARGS...: runs one bridge in the background, its output prefixed with [NAME]");
    let _ = writeln!(o, "start() {{");
    let _ = writeln!(o, "    local name=$1");
    let _ = writeln!(o, "    shift");
    let _ = writeln!(o, "    \"$TITAN_BRIDGE\" \"$@\" > >(sed -u \"s/^/[$name] /\") 2>&1 &");
    let _ = writeln!(o, "    pids+=(\"$!\")");
    let _ = writeln!(o, "}}\n");
    for c in cmds {
        match c.args() {
            Some(args) => {
                let args: Vec<String> = args.iter().map(|a| shell_quote(a)).collect();
                let _ = writeln!(o, "start {} {}", shell_quote(&c.board), args.join(" "));
            }
            None => {
                let _ = writeln!(o, "# {}: {}", c.board, c.note.as_deref().unwrap_or("no bridge"));
            }
        }
    }
    let _ = writeln!(o, "\n# Until one bridge exits (or Ctrl-C), then stop the others");
    let _ = writeln!(o, "wait -n");
    o
}

fn py_str(s: &str) -> String {
    format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
}

/// A ROS 2 Python launch file starting every bridge (`ros2 launch ./bridges.launch.py`).
pub fn render_launch(cmds: &[BridgeCommand], program: &str, robot_file: &str) -> String {
    let mut o = String::new();
    let _ = writeln!(
        o,
        "# Generated by `firmware_simulator bridges {robot_file} --format launch`: one titan_bridge per board."
    );
    let _ = writeln!(o, "# Start the emulator first: {}", emulator_command(robot_file, cmds));
    let _ = writeln!(o, "# Run with: ros2 launch <this file>   (TITAN_BRIDGE overrides the program path)");
    let _ = writeln!(o, "import os\n");
    let _ = writeln!(o, "from launch import LaunchDescription");
    let _ = writeln!(o, "from launch.actions import ExecuteProcess\n");
    let _ = writeln!(o, "TITAN_BRIDGE = os.environ.get('TITAN_BRIDGE', {})\n", py_str(program));
    let _ = writeln!(o, "BRIDGES = [");
    for c in cmds {
        match c.args() {
            Some(args) => {
                let args: Vec<String> = args.iter().map(|a| py_str(a)).collect();
                let _ = writeln!(o, "    ({}, [{}]),", py_str(&c.board), args.join(", "));
            }
            None => {
                let _ = writeln!(o, "    # {}: {}", c.board, c.note.as_deref().unwrap_or("no bridge"));
            }
        }
    }
    let _ = writeln!(o, "]\n\n");
    let _ = writeln!(o, "def generate_launch_description():");
    let _ = writeln!(o, "    return LaunchDescription([");
    let _ =
        writeln!(o, "        ExecuteProcess(cmd=[TITAN_BRIDGE] + args, name='titan_bridge_' + name, output='screen')");
    let _ = writeln!(o, "        for name, args in BRIDGES");
    let _ = writeln!(o, "    ])");
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    fn talos() -> (RobotSpec, std::path::PathBuf) {
        let path = repo().join("robots/talos.toml");
        let spec: RobotSpec = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        (spec, path.parent().unwrap().to_path_buf())
    }

    #[test]
    fn talos_bridges() {
        let (spec, dir) = talos();
        let cmds = plan(&spec, &dir, &[], None).unwrap();
        let lines = render_lines(&cmds, "titan_bridge");
        let got: Vec<&str> = lines.lines().filter(|l| !l.starts_with('#')).collect();
        assert_eq!(
            got,
            [
                "titan_bridge --board poac --interface vcan0 --namespace talos --client-id 1",
                "titan_bridge --board esc0 --interface vcan0 --namespace talos --client-id 2",
                "titan_bridge --board esc1 --interface vcan0 --namespace talos --client-id 3",
                "titan_bridge --board camera_cage --interface vcan0 --namespace talos --client-id 4",
                "titan_bridge --board actuator --interface vcan0 --namespace talos --client-id 5",
                "titan_bridge --board led --interface vcan0 --namespace talos --client-id 6",
                "titan_bridge --board sbc --interface vcan0 --namespace talos --client-id 7",
                "titan_bridge --board ivc2 --interface vcan0 --namespace talos --client-id 8",
                "titan_bridge --board smart_battery --interface vcan1 --namespace talos --client-id 21",
            ]
        );
        assert!(lines.contains("# battery (smart_battery) on external (vcan1)"), "{lines}");
        assert_eq!(
            emulator_command("robots/talos.toml", &cmds),
            "firmware_simulator robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime"
        );
    }

    #[test]
    fn interfaces_namespace_and_unknown_images() {
        let (mut spec, dir) = talos();
        spec.board[0].elf = "../build/c/PowerBoard/PowerBoard.elf".into();
        let ifaces = [("internal".to_string(), "can0".to_string())];
        let cmds = plan(&spec, &dir, &ifaces, Some("sim")).unwrap();
        assert_eq!(cmds[0].args(), None);
        assert!(cmds[0].note.as_deref().unwrap().contains("no titan_bridge profile for image \"PowerBoard\""));
        assert_eq!(cmds[1].args().unwrap().join(" "), "--board esc0 --interface can0 --namespace sim --client-id 2");
        assert_eq!(cmds[8].iface.as_deref(), Some("vcan1"));
        assert!(plan(&spec, &dir, &[("nosuch".into(), "can9".into())], None).is_err());
        assert_eq!(image_name(Path::new("a/b/esc_board.elf")), "esc_board");
        assert_eq!(image_name(Path::new("x.uf2")), "x");
    }

    #[test]
    fn script_and_launch_file() {
        let (spec, dir) = talos();
        let cmds = plan(&spec, &dir, &[], None).unwrap();
        let sh = render_script(&cmds, "titan_bridge", "robots/talos.toml");
        assert!(sh.starts_with("#!/usr/bin/env bash\n"));
        assert!(sh.contains("for iface in vcan0 vcan1; do"), "{sh}");
        assert!(sh.contains("\nstart esc1 --board esc1 --interface vcan0 --namespace talos --client-id 3\n"), "{sh}");
        assert!(sh.contains("--can internal=vcan0 --can external=vcan1 --realtime"));
        let py = render_launch(&cmds, "titan_bridge", "robots/talos.toml");
        assert!(py.contains("    ('battery', ['--board', 'smart_battery', '--interface', 'vcan1', '--namespace', 'talos', '--client-id', '21']),"), "{py}");
        assert_eq!(shell_quote("it's"), r"'it'\''s'");
        assert_eq!(py_str("a'b"), r"'a\'b'");
    }

    /// Bridges to the emulator's cannelloni hub (no vcan): the interface is passed on, the emulator command
    /// serves the bus on that address, and the script checks no interfaces.
    #[test]
    fn cannelloni_interfaces() {
        let (spec, dir) = talos();
        let ifaces = [
            ("internal".to_string(), "cannelloni:127.0.0.1:20000".to_string()),
            ("external".to_string(), "cannelloni:host.docker.internal:20001".to_string()),
        ];
        let cmds = plan(&spec, &dir, &ifaces, None).unwrap();
        assert_eq!(
            cmds[2].args().unwrap().join(" "),
            "--board esc1 --interface cannelloni:127.0.0.1:20000 --namespace talos --client-id 3"
        );
        assert_eq!(
            emulator_command("robots/talos.toml", &cmds),
            "firmware_simulator robot robots/talos.toml --cannelloni internal=127.0.0.1:20000 \
             --cannelloni external=0.0.0.0:20001 --realtime"
        );
        let sh = render_script(&cmds, "titan_bridge", "robots/talos.toml");
        assert!(!sh.contains("ip link show"), "{sh}");
        assert!(sh.contains("start esc0 --board esc0 --interface cannelloni:127.0.0.1:20000 "), "{sh}");
        assert_eq!(cannelloni_addr("vcan0"), None);
    }

    #[test]
    fn probed_ids_replace_static_ones() {
        let (spec, dir) = talos();
        let mut cmds = plan(&spec, &dir, &[], None).unwrap();
        let mut probed = vec![Some(1), Some(3), Some(3), Some(4), Some(5), Some(6), Some(7), None, Some(22)];
        probed.truncate(cmds.len());
        apply_probe(&mut cmds, &probed);
        assert_eq!(cmds[1].profile.unwrap().name, "esc1", "the probed ID picks the ESC profile");
        assert!(cmds[1].note.as_deref().unwrap().contains("suggests client 2, the board sent 3"));
        assert_eq!(cmds[7].note.as_deref(), Some("no CANmore heartbeat seen while probing"));
        assert_eq!(cmds[8].client_id, Some(22));
    }

    /// The profiles mirror titan_bridge's: compare with its sources, which are in the titan_firmware_rs checkout.
    #[test]
    fn profiles_match_titan_bridge() {
        let dir = repo().join("external/titan_firmware_rs/host/titan_bridge/src/boards");
        let Ok(entries) = std::fs::read_dir(&dir) else {
            eprintln!("skipping: no {}", dir.display());
            return;
        };
        let ids = [
            ("CLIENT_ID_POWER_BOARD", 1),
            ("CLIENT_ID_ESC_BOARD0", 2),
            ("CLIENT_ID_ESC_BOARD1", 3),
            ("CLIENT_ID_CAMERA_CAGE_BB", 4),
            ("CLIENT_ID_ACTUATOR_BOARD", 5),
            ("CLIENT_ID_LED_BOARD", 6),
            ("CLIENT_ID_SBC_BOARD", 7),
            ("CLIENT_ID_IVC_BOARD", 8),
            ("CLIENT_ID_SMART_BATTERY_PORT", 1),
        ];
        let mut found = Vec::new();
        for e in entries {
            let path = e.unwrap().path();
            if path.file_name().unwrap() == "mod.rs" {
                continue;
            }
            let text = std::fs::read_to_string(path).unwrap();
            // `name: "esc0",` ... `client_id: client_ids::CLIENT_ID_ESC_BOARD0,` within each profile literal
            for block in text.split("BoardProfile {").skip(1) {
                let field = |key: &str| -> Option<String> {
                    let at = block.find(&format!("{key}: "))?;
                    let rest = &block[at + key.len() + 2..];
                    Some(rest[..rest.find([',', '}'])?].trim().trim_matches('"').to_string())
                };
                let Some(name) = field("name") else { continue };
                let ns = field("namespace");
                let client = field("client_id").map(|c| {
                    let c = c.trim_start_matches("client_ids::");
                    ids.iter().find(|(n, _)| *n == c).map(|(_, v)| *v).unwrap_or_else(|| c.parse().unwrap())
                });
                found.push((name, ns, client));
            }
        }
        assert_eq!(found.len(), PROFILES.len(), "{found:?}");
        for (name, ns, client) in found {
            let p = PROFILES.iter().find(|p| p.name == name).unwrap_or_else(|| panic!("titan_bridge has {name}"));
            if let Some(ns) = ns {
                assert_eq!(p.namespace, ns, "{name}");
            }
            assert_eq!(Some(p.client_id), client, "{name}");
        }
    }
}
