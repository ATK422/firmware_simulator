# shellcheck disable=SC2034 # transport, transport_why and agent_cmd are this file's outputs, for the scripts that source it
# Sourced by scripts/robot_in_a_box.sh and scripts/nereus_firmware_sim.sh: how the robot computer's side reaches
# the emulated CAN buses.
#
#   vcan  SocketCAN vcan interfaces named like the Orin's (can0, can1), so the host tools run exactly as on the
#         robot (micro_ros_agent can -D can0, titan_bridge on can0, the controller's raw thruster frames). Needs
#         Linux with the vcan module, and root or sudo to create the interfaces (scripts/vcan_up.sh).
#   udp   No CAN interface at all: the emulator relays the C boards' micro-ROS sessions to a stock micro-ROS agent
#         over UDP (--micro-ros-agent BUS=HOST:PORT) and serves each bus to titan_bridge as a cannelloni hub
#         (--cannelloni BUS=HOST:PORT, titan_bridge --interface cannelloni:HOST:PORT). Any Linux (no sudo), WSL2,
#         macOS.

# choose_transport WANTED IFACE...: sets transport (vcan or udp) and transport_why. WANTED is auto, vcan or udp;
# auto takes vcan when the interfaces exist as vcan already, or can be made, and udp otherwise.
choose_transport() {
    local wanted="$1"
    shift
    case "$wanted" in
        vcan | udp)
            transport="$wanted"
            transport_why="asked for"
            return
            ;;
        auto) ;;
        *)
            echo "--transport: auto, vcan or udp (got $wanted)" >&2
            exit 2
            ;;
    esac
    transport=udp
    local os
    os="$(uname -s)"
    if [ "$os" != Linux ]; then
        transport_why="$os has no SocketCAN"
        return
    fi
    if ! command -v ip >/dev/null; then
        transport_why="no ip command (iproute2) to make vcan interfaces with"
        return
    fi
    local i all=1
    for i in "$@"; do
        if ! ip -details link show "$i" 2>/dev/null | grep -q vcan; then
            all=0
        fi
    done
    if [ "$all" -eq 1 ]; then
        transport=vcan
        transport_why="$* are vcan interfaces already"
        return
    fi
    local modinfo
    modinfo="$(command -v modinfo || echo /sbin/modinfo)"
    if ! "$modinfo" vcan >/dev/null 2>&1; then
        transport_why="this kernel has no vcan module (WSL2's stock kernel has none)"
        return
    fi
    if [ "$(id -u)" -ne 0 ] && ! command -v sudo >/dev/null; then
        transport_why="making vcan interfaces needs root or sudo"
        return
    fi
    transport=vcan
    transport_why="the vcan module is available (scripts/vcan_up.sh makes the interfaces, with sudo)"
}

# vcan_exists IFACE...: true when every interface is a vcan interface already
vcan_exists() {
    local i
    for i in "$@"; do
        if ! ip -details link show "$i" 2>/dev/null | grep -q vcan; then
            return 1
        fi
    done
}

# not_a_real_adapter IFACE...: prints a problem for each interface that exists and is not vcan (a real CAN adapter
# would get the emulator's frames, and with udp the controller's raw thruster frames)
not_a_real_adapter() {
    local i
    command -v ip >/dev/null || return 0
    for i in "$@"; do
        if ip -details link show "$i" >/dev/null 2>&1 && ! ip -details link show "$i" | grep -q vcan; then
            echo "$i exists and is not a vcan interface (a real CAN adapter?): unplug it or rename it first"
        fi
    done
}

# micro_ros_agent_udp PORT: sets agent_cmd to a micro-ROS agent serving UDP on PORT (riptide's micro_ros_agent
# package, any build of it, or a standalone MicroXRCEAgent), or empties it when there is none
micro_ros_agent_udp() {
    agent_cmd=()
    if [ -n "${ROS_DISTRO:-}" ] && command -v ros2 >/dev/null && ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
        agent_cmd=(ros2 run micro_ros_agent micro_ros_agent udp4 --port "$1")
    elif command -v MicroXRCEAgent >/dev/null; then
        agent_cmd=(MicroXRCEAgent udp4 --port "$1")
    fi
}
