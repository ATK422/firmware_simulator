#!/usr/bin/env bash
# One command for Talos's firmware in nereus: vcan can0/can1, the emulated boards on them (with nereus as the
# physics), nereus with the riptide stack and its viewer, the robot computer's side of the boards, and the bridge
# between the emulator and nereus. Watch the vehicle in nereus's viewer while the real firmware drives it.
#
#   scripts/nereus_firmware_sim.sh                 # C images from $TITAN_FIRMWARE_DIR (robots/talos_c_nereus.toml)
#   scripts/nereus_firmware_sim.sh --rust          # titan_firmware_rs images and titan_bridge (robots/talos_nereus.toml)
#   scripts/nereus_firmware_sim.sh --dry-run       # print what it would run
#
# Options:
#   --nereus DIR         your nereus checkout, built with ./build.sh (default $NEREUS_DIR, ~/osu-uwrt/release/nereus,
#                        ~/osu-uwrt/nereus or ~/nereus)
#   --rust               titan_firmware_rs images and one titan_bridge per board
#   --robot FILE         another robot file (its vehicle must be external)
#   --pool robosub|rpac  nereus's course (default robosub)
#   --no-viewer, --no-cameras, --no-stack   leave out nereus's viewer, its cameras, or the riptide stack
#   --rmw RMW            RMW for every process (e.g. rmw_zenoh_cpp; default: the shell's)
#   --dashboard ADDR     the emulator's dashboard, which the bridge talks to (default 127.0.0.1:8080)
#   --threads N          the emulator's --threads (default 0: as many as help)
#   --rpm-from-forces    the bridge also turns the controller's thruster_forces into command/thruster_rpm (for a
#                        controller that does not write can0 itself, such as riptide_mpc)
#   --depth nereus       nereus publishes depth/pose as without the firmware (default: the firmware's reading)
#   --actuators nereus   nereus publishes state/actuator/* (default: the PoAC firmware does)
#   --no-clock-sync      keep nereus at real time even when the emulator is slower
#   --no-vcan            can0 and can1 exist already (do not run scripts/vcan_up.sh, which needs sudo)
#   --dry-run            print the commands instead of running them
#
# docs/nereus.md has the setup, what to look for and troubleshooting.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
nereus="${NEREUS_DIR:-}"
rust=0
robot=""
pool=robosub
viewer=true
cameras=true
stack=true
rmw=""
dashboard=127.0.0.1:8080
threads=0
rpm_from_forces=false
clock_sync=true
depth=firmware
actuators=firmware
vcan=1
dry=0
while [ $# -gt 0 ]; do
    case "$1" in
        --nereus) nereus="$2"; shift ;;
        --rust) rust=1 ;;
        --robot) robot="$2"; shift ;;
        --pool) pool="$2"; shift ;;
        --no-viewer) viewer=false ;;
        --no-cameras) cameras=false ;;
        --no-stack) stack=false ;;
        --rmw) rmw="$2"; shift ;;
        --dashboard) dashboard="$2"; shift ;;
        --threads) threads="$2"; shift ;;
        --rpm-from-forces) rpm_from_forces=true ;;
        --depth) depth="$2"; shift ;;
        --actuators) actuators="$2"; shift ;;
        --no-clock-sync) clock_sync=false ;;
        --no-vcan) vcan=0 ;;
        --dry-run) dry=1 ;;
        -h | --help) sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done

problems=()
need() {
    problems+=("$1")
}
run() {
    echo "+ $*"
    if [ "$dry" -eq 0 ]; then "$@"; fi
}

if [ -z "$nereus" ]; then
    for d in ~/osu-uwrt/release/nereus ~/osu-uwrt/nereus ~/nereus; do
        if [ -f "$d/integrations/uwrt/launch/sim.launch.py" ]; then
            nereus="$d"
            break
        fi
    done
fi
if [ -z "$nereus" ] || [ ! -f "$nereus/integrations/uwrt/launch/sim.launch.py" ]; then
    need "nereus: no checkout found; pass --nereus DIR (git clone https://github.com/osu-uwrt/nereus)"
    nereus="${nereus:-NEREUS_DIR}"
elif [ ! -x "$nereus/build/ros-viewer/integrations/ros2/bridge/nereus-sim" ]; then
    need "nereus: not built; in $nereus, with ROS 2 and the riptide workspace sourced: ./setup.sh && ./build.sh"
fi
if [ -z "$robot" ]; then
    if [ "$rust" -eq 1 ]; then robot="$repo/robots/talos_nereus.toml"; else robot="$repo/robots/talos_c_nereus.toml"; fi
fi
if [ "$rust" -eq 0 ] && [ -z "${TITAN_FIRMWARE_DIR:-}" ]; then
    need "TITAN_FIRMWARE_DIR is not set: the C images come from your titan_firmware checkout (scripts/build_c_boards.sh), or use --rust"
fi
sim="${FIRMWARE_SIMULATOR:-}"
if [ -z "$sim" ]; then
    if [ -x "$repo/target/release/firmware_simulator" ]; then
        sim="$repo/target/release/firmware_simulator"
    elif command -v firmware_simulator >/dev/null; then
        sim="$(command -v firmware_simulator)"
    else
        need "firmware_simulator: not built (cargo build --release)"
        sim=firmware_simulator
    fi
fi
if [ -z "${ROS_DISTRO:-}" ] || ! command -v ros2 >/dev/null; then
    need "ROS 2: not sourced (source /opt/ros/humble/setup.bash and your riptide workspace's install/setup.bash)"
else
    for pkg in riptide_bringup2 riptide_hardware2 riptide_msgs2; do
        if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
            need "ROS 2: no $pkg; source your riptide workspace (osu-uwrt/riptide_release, built with colcon)"
        fi
    done
    if [ "$rust" -eq 0 ] && ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
        need "ROS 2: no micro_ros_agent (build micro-ROS-Agent with osu-uwrt/Micro-XRCE-DDS-Agent's can-support branch, as the Orin has it)"
    fi
fi
if [ "$rust" -eq 1 ] && ! command -v titan_bridge >/dev/null; then
    need "titan_bridge: not on PATH (cargo install --path external/titan_firmware_rs/host/titan_bridge, in riptide-shell)"
fi
for i in can0 can1; do
    if ip -details link show "$i" >/dev/null 2>&1 && ! ip -details link show "$i" | grep -q vcan; then
        need "$i exists and is not a vcan interface (a real CAN adapter?): unplug it first"
    fi
done
if [ "${#problems[@]}" -gt 0 ]; then
    printf '%s\n' "${problems[@]}" >&2
    if [ "$dry" -eq 0 ]; then
        exit 1
    fi
    echo "(--dry-run: printing the commands anyway)" >&2
fi

run_dir="${RUN_DIR:-/tmp/nereus_firmware/$(date +%Y%m%d-%H%M%S)}"
if [ "$dry" -eq 0 ]; then
    mkdir -p "$run_dir"
fi
if [ "$vcan" -eq 1 ]; then
    run "$repo/scripts/vcan_up.sh" can0 can1
fi
scenario="$run_dir/packs/scenarios/talos_firmware_$pool"
run python3 "$repo/integrations/nereus/make_packs.py" "$nereus" "$run_dir/packs" --pool "$pool" --depth "$depth" \
    --actuators "$actuators"

emu=("$sim" robot "$robot" --can internal=can0 --realtime --time 1000000s --threads "$threads"
    --dashboard "$dashboard")
pid=""
cleanup() {
    if [ -n "$pid" ]; then
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM
echo "+ ${emu[*]} > $run_dir/emulator.log 2>&1 &"
if [ "$dry" -eq 0 ]; then
    "${emu[@]}" >"$run_dir/emulator.log" 2>&1 &
    pid=$!
    for _ in $(seq 50); do
        if python3 -c "import urllib.request,sys; urllib.request.urlopen('http://$dashboard/api/thrusters', timeout=1)" \
            2>/dev/null; then
            break
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "the emulator stopped:" >&2
            tail -20 "$run_dir/emulator.log" >&2
            exit 1
        fi
        sleep 0.2
    done
    echo "emulator running (pid $pid): dashboard on http://$dashboard/, log in $run_dir/emulator.log"
fi

launch=(ros2 launch "$repo/integrations/nereus/nereus_firmware.launch.py" "nereus_dir:=$nereus" "scenario:=$scenario"
    "dashboard:=$dashboard" "viewer:=$viewer" "cameras:=$cameras" "stack:=$stack" "rpm_from_forces:=$rpm_from_forces"
    "clock_sync:=$clock_sync" "output:=$run_dir/nereus")
if [ -n "$rmw" ]; then
    launch+=("rmw:=$rmw")
fi
if [ "$rust" -eq 1 ]; then
    run "$sim" bridges "$robot" --can internal=can0 --format launch -o "$run_dir/bridges.launch.py"
    launch+=("firmware:=rust" "bridges_launch:=$run_dir/bridges.launch.py")
fi
run "${launch[@]}"
