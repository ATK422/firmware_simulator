#!/usr/bin/env bash
# Robot in a box: the emulated Talos on vcan interfaces named like the Orin's CAN interfaces (can0: internal CAN FD
# bus, can1: external bus), so the robot computer's side runs unchanged: riptide's copro_agent.launch.py
# (`micro_ros_agent can -D can0` and `can -D can1`) for titan_firmware's C boards, or one titan_bridge per board for
# titan_firmware_rs's images, and the controller's raw thruster frames on can0. Without vcan (WSL2, macOS, Linux
# without the module or sudo) it uses UDP instead: a micro-ROS agent's udp4 transport for the C boards, CAN over UDP
# (titan_bridge --interface cannelloni:HOST:PORT) for the Rust ones.
#
#   scripts/robot_in_a_box.sh                  # C images (robots/talos_c.toml, $TITAN_FIRMWARE_DIR), micro-ROS agents
#   scripts/robot_in_a_box.sh --rust           # Rust images (robots/talos.toml), titan_bridge
#   scripts/robot_in_a_box.sh --stack bringup  # also riptide's bringup (hardware:=none: controller, EKF, autonomy)
#   scripts/robot_in_a_box.sh --dry-run        # print what it would run
#
# Options:
#   --robot FILE        robot file (default robots/talos_c.toml, or robots/talos.toml with --rust)
#   --rust              titan_firmware_rs images and titan_bridge instead of the C images and micro-ROS agents
#   --stack MODE        agents (default): the boards' host side only; bringup: also riptide_bringup2's bringup
#                       with hardware:=none; none: only the emulator (start the host side yourself)
#   --dashboard ADDR    the emulator's dashboard (default 127.0.0.1:8080; "" for none)
#   --threads N         the emulator's --threads (default 0: as many as help)
#   --transport T       auto (default): vcan if the vcan module is there, else udp; vcan: can0 and can1 (made
#                       with scripts/vcan_up.sh and sudo unless they exist); udp: no CAN interfaces
#   --no-vcan           the same as --transport udp
#   --agent-port N      udp, C images: the micro-ROS agent's UDP port (default 8888)
#   --dry-run           print the commands instead of running them
#
# For the host side: a sourced ROS 2 Humble with the riptide workspace (on vcan, micro_ros_agent built from
# osu-uwrt/Micro-XRCE-DDS-Agent's can-support branch, as on the Orin; on udp any micro-ROS agent, or MicroXRCEAgent),
# or titan_bridge on PATH for --rust. docs/robots.md#robot-in-a-box-the-orins-side-on-vcan has the details.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
rust=0
robot=""
stack=agents
dashboard=127.0.0.1:8080
threads=0
transport_wanted=auto
agent_port=8888
dry=0
while [ $# -gt 0 ]; do
    case "$1" in
        --robot) robot="$2"; shift ;;
        --rust) rust=1 ;;
        --stack) stack="$2"; shift ;;
        --dashboard) dashboard="$2"; shift ;;
        --threads) threads="$2"; shift ;;
        --transport) transport_wanted="$2"; shift ;;
        --no-vcan) transport_wanted=udp ;;
        --agent-port) agent_port="$2"; shift ;;
        --dry-run) dry=1 ;;
        -h | --help) sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done
case "$stack" in agents | bringup | none) ;; *) echo "--stack: agents, bringup or none" >&2; exit 2 ;; esac
if [ -z "$robot" ]; then
    if [ "$rust" -eq 1 ]; then robot="$repo/robots/talos.toml"; else robot="$repo/robots/talos_c.toml"; fi
fi

run() {
    echo "+ $*"
    if [ "$dry" -eq 0 ]; then "$@"; fi
}

sim="${FIRMWARE_SIMULATOR:-}"
if [ -z "$sim" ]; then
    if [ -x "$repo/target/release/firmware_simulator" ]; then
        sim="$repo/target/release/firmware_simulator"
    elif command -v firmware_simulator >/dev/null; then
        sim="$(command -v firmware_simulator)"
    elif [ "$dry" -eq 1 ]; then
        sim=firmware_simulator
    else
        echo "no firmware_simulator: cargo build --release (or set FIRMWARE_SIMULATOR)" >&2
        exit 1
    fi
fi
if [ "$rust" -eq 0 ] && [ -z "${TITAN_FIRMWARE_DIR:-}" ] && [ "$dry" -eq 0 ]; then
    echo "TITAN_FIRMWARE_DIR is not set: the C images come from your titan_firmware checkout" >&2
    echo "(build them with scripts/build_c_boards.sh), or use --rust for the titan_firmware_rs images" >&2
    exit 1
fi

# shellcheck disable=SC1091
. "$repo/scripts/can_transport.sh"
choose_transport "$transport_wanted" can0 can1
echo "CAN transport: $transport ($transport_why)" >&2

# A real CAN adapter called can0 would get the emulator's frames (or the controller's): refuse anything but vcan
adapters="$(not_a_real_adapter can0 can1)"
if [ -n "$adapters" ]; then
    echo "$adapters" >&2
    exit 1
fi
if [ "$transport" = vcan ] && ! vcan_exists can0 can1; then
    run "$repo/scripts/vcan_up.sh" can0 can1
fi

run_dir="${RUN_DIR:-/tmp/robot_in_a_box}"
if [ "$dry" -eq 0 ]; then
    mkdir -p "$run_dir"
fi
# The external bus (the smart battery) when the robot file has one
external=0
buses=(internal)
if grep -q '^name = "external"' "$robot" 2>/dev/null; then
    external=1
    buses+=(external)
fi
# How each bus reaches the robot computer's side: its vcan, or UDP (one micro-ROS agent for both buses' C boards,
# a cannelloni hub per bus for the titan_bridges)
emu=("$sim" robot "$robot")
cans=()
for k in "${!buses[@]}"; do
    b="${buses[$k]}"
    if [ "$transport" = vcan ]; then
        emu+=(--can "$b=can$k")
        cans+=(--can "$b=can$k")
    elif [ "$rust" -eq 1 ]; then
        emu+=(--cannelloni "$b=127.0.0.1:$((20000 + k))")
        cans+=(--can "$b=cannelloni:127.0.0.1:$((20000 + k))")
    else
        emu+=(--micro-ros-agent "$b=127.0.0.1:$agent_port")
    fi
done
emu+=(--realtime --time 1000000s --threads "$threads")
if [ -n "$dashboard" ]; then
    emu+=(--dashboard "$dashboard")
fi

pids=()
cleanup() {
    for p in "${pids[@]}"; do
        kill "$p" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "+ ${emu[*]} > $run_dir/emulator.log 2>&1 &"
if [ "$dry" -eq 0 ]; then
    "${emu[@]}" >"$run_dir/emulator.log" 2>&1 &
    pids+=($!)
    sleep 2
    if ! kill -0 "${pids[0]}" 2>/dev/null; then
        echo "the emulator stopped:" >&2
        tail -20 "$run_dir/emulator.log" >&2
        exit 1
    fi
    echo "emulator running (pid ${pids[0]}), log in $run_dir/emulator.log${dashboard:+, dashboard on http://$dashboard/}"
fi

host=()
if [ "$rust" -eq 1 ]; then
    run "$sim" bridges "$robot" "${cans[@]}" --format launch -o "$run_dir/bridges.launch.py"
    host=(ros2 launch "$run_dir/bridges.launch.py")
elif [ "$transport" = vcan ]; then
    host=(ros2 launch riptide_hardware2 copro_agent.launch.py robot:=talos)
else
    micro_ros_agent_udp "$agent_port"
    if [ "${#agent_cmd[@]}" -eq 0 ]; then agent_cmd=(MicroXRCEAgent udp4 --port "$agent_port"); fi
    host=("${agent_cmd[@]}")
fi
bringup=(ros2 launch riptide_bringup2 bringup.launch.py robot:=talos hardware:=none)

have_ros=0
if [ -n "${ROS_DISTRO:-}" ] && command -v ros2 >/dev/null; then
    if [ "$rust" -eq 1 ] && command -v titan_bridge >/dev/null; then
        have_ros=1
    elif [ "$rust" -eq 0 ] && [ "$transport" = vcan ] && ros2 pkg prefix riptide_hardware2 >/dev/null 2>&1 &&
        ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
        have_ros=1
    elif [ "$rust" -eq 0 ] && [ "$transport" = udp ] && command -v "${host[0]}" >/dev/null; then
        have_ros=1
    fi
fi

if [ "$stack" = none ] || [ "$have_ros" -eq 0 ]; then
    if [ "$stack" != none ]; then
        echo
        if [ "$rust" -eq 1 ]; then what="ROS 2 and titan_bridge"; else what="ROS 2 with the riptide workspace"; fi
        echo "No sourced $what found. In a shell that has it:"
    else
        echo "The robot computer's side, in a shell with ROS 2 and the riptide workspace:"
    fi
    echo "  ${host[*]}"
    echo "  ${bringup[*]}      # optional: controller, EKF, autonomy"
    echo "  ros2 topic list | grep /talos    # the boards' topics, e.g. /talos/state/firmware, /talos/state/kill"
    if [ "$dry" -eq 0 ]; then
        echo "Ctrl-C stops the emulator."
        wait "${pids[0]}"
    fi
    exit 0
fi

echo "+ ${host[*]} &"
if [ "$dry" -eq 0 ]; then
    "${host[@]}" &
    pids+=($!)
fi
if [ "$stack" = bringup ]; then
    run "${bringup[@]}"
elif [ "$dry" -eq 0 ]; then
    wait "${pids[0]}"
fi
