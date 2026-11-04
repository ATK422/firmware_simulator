#!/usr/bin/env bash
# Starts the emulator for a debug session in VS Code (the tasks in .vscode/tasks.json; see docs/running.md,
# "Debugging in VS Code"). Links the ELF that GDB loads symbols from to target/vscode/debug.elf (the `executable` of
# every configuration in .vscode/launch.json), then runs the emulator waiting for GDB on 127.0.0.1:3333. It runs
# only as long as GDB drives it (`--time 0s`), so it ends with the debug session.
#
#   scripts/vscode_gdb.sh run "IMAGE BOARD [CLIENT]"            a titan_firmware_rs image on boards/BOARD.toml, with
#                                                              a mock titan_bridge for CANmore client CLIENT
#   scripts/vscode_gdb.sh elf ELF [BOARD]                       any ELF, e.g. a C pico-sdk build under build/c
#   scripts/vscode_gdb.sh robot ROBOT "BOARD IMAGE [BUS:CLIENT]" board BOARD (running IMAGE) of a robot file, with a
#                                                              mock titan_bridge for it
#
# titan_firmware_rs images come from $TITAN_RS_RELEASE_DIR, else external/titan_firmware_rs/firmware/target/
# thumbv6m-none-eabi/release. Extra arguments go to the emulator.
set -eu
cd "$(dirname "$0")/.."
rs=${TITAN_RS_RELEASE_DIR:-external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release}
gdb=127.0.0.1:3333

# Points target/vscode/debug.elf at $1
link() {
    if [ ! -f "$1" ]; then
        echo "error: $1 not built (see docs/running.md, Images)" >&2
        exit 1
    fi
    mkdir -p target/vscode
    ln -sf "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")" target/vscode/debug.elf
}

mode=${1:?usage: scripts/vscode_gdb.sh run|elf|robot ...}
shift
case "$mode" in
run)
    read -r image board client <<<"$1"
    shift
    link "$rs/$image"
    exec cargo run --release -- run "$rs/$image" --board "boards/$board.toml" ${client:+--mock-bridge "$client"} \
        --time 0s --gdb "$gdb" "$@"
    ;;
elf)
    elf=$1 board=${2:-}
    shift $(($# < 2 ? $# : 2))
    link "$elf"
    exec cargo run --release -- run "$elf" ${board:+--board "boards/$board.toml"} --time 0s --gdb "$gdb" "$@"
    ;;
robot)
    robot=$1
    read -r board image bridge <<<"$2"
    shift 2
    link "$rs/$image"
    exec cargo run --release -- robot "$robot" ${bridge:+--mock-bridge "$bridge"} --time 0s --gdb "$board=$gdb" "$@"
    ;;
*)
    echo "usage: scripts/vscode_gdb.sh run|elf|robot ..." >&2
    exit 2
    ;;
esac
