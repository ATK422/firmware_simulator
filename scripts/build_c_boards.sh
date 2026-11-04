#!/usr/bin/env bash
# Builds titan_firmware's C Talos board apps that robots/talos_c.toml and robots/talos_c_nereus.toml run, each in
# its project's own build/ directory (as titan_firmware's docs do it), in your titan_firmware checkout:
#
#   export TITAN_FIRMWARE_DIR=~/osu-uwrt/titan_firmware        # a checkout of osu-uwrt/titan_firmware
#   git -C "$TITAN_FIRMWARE_DIR" submodule update --init --recursive
#   (cd "$TITAN_FIRMWARE_DIR/lib/micro_ros_pico" && ./build.sh) # once: micro-ROS for the RP2040, built in Docker
#   scripts/build_c_boards.sh                                   # all of them
#   scripts/build_c_boards.sh talos/ESCBoard talos/PoacBoard    # or some
#
# Each build writes NAME_with_bl.uf2 (the CAN bootloader and the app, what a board's flash holds). Needs
# gcc-arm-none-eabi, cmake and make (or ninja: CMAKE_GENERATOR=Ninja). This never runs in this repository's CI:
# the micro-ROS library needs Docker.
set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
fi
tf="${TITAN_FIRMWARE_DIR:?set TITAN_FIRMWARE_DIR to your titan_firmware checkout}"
projects=("$@")
if [ "${#projects[@]}" -eq 0 ]; then
    projects=(talos/PoacBoard talos/ESCBoard talos/CameraCageBB talos/Actuator talos/LEDBoard SBCBoard
        talos/SmartBattery)
fi
if [ ! -f "$tf/lib/micro_ros_pico/libmicroros/libmicroros.a" ]; then
    echo "no $tf/lib/micro_ros_pico/libmicroros/libmicroros.a: build micro-ROS first" >&2
    echo "  (cd $tf/lib/micro_ros_pico && ./build.sh)     # needs Docker" >&2
    exit 1
fi
for p in "${projects[@]}"; do
    echo "== $p"
    cmake -S "$tf/$p" -B "$tf/$p/build"
    cmake --build "$tf/$p/build" -j "$(nproc 2>/dev/null || echo 4)"
    ls "$tf/$p/build/"*_with_bl.uf2
done
