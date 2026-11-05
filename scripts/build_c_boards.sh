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
# the micro-ROS library needs Docker. The Nix shell (flake.nix) has the library prebuilt in $MICRO_ROS_LIB_DIR:
# link it in as lib/micro_ros_pico/libmicroros, where titan_firmware looks for it.
#
# Two apps do not build as they are at titan_firmware 058df2a; without changing the checkout, this script
#   * links SBCBoard against driver_canbus and micro_ros_can (scripts/cmake/sbc_board_link_canbus.cmake), which
#     its CMakeLists.txt leaves out;
#   * builds talos/Actuator with riptide_msgs2's ActuatorStatus.CLAW_CLOSED defined as CLAW_BOTTLE_CLOSED when the
#     micro-ROS library's riptide_msgs2 is newer than the app (it split CLAW_CLOSED into BOTTLE and SPOON); this
#     takes effect in a fresh build directory.
set -euo pipefail

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'
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
    if [ -n "${MICRO_ROS_LIB_DIR:-}" ]; then
        echo "  ln -s $MICRO_ROS_LIB_DIR $tf/lib/micro_ros_pico/libmicroros   # the Nix shell's" >&2
    else
        echo "  (cd $tf/lib/micro_ros_pico && ./build.sh)     # needs Docker" >&2
    fi
    exit 1
fi
here="$(cd "$(dirname "$0")" && pwd)"
actuator_status="$tf/lib/micro_ros_pico/libmicroros/include/riptide_msgs2/msg/detail/actuator_status__struct.h"
for p in "${projects[@]}"; do
    echo "== $p"
    extra=() cflags="${CFLAGS:-}"
    case "$p" in
    SBCBoard) extra=(-DCMAKE_PROJECT_sbc_board_INCLUDE="$here/cmake/sbc_board_link_canbus.cmake") ;;
    talos/Actuator)
        if [ -f "$actuator_status" ] && ! grep -q 'ActuatorStatus__CLAW_CLOSED\b' "$actuator_status"; then
            echo "   (the micro-ROS library's riptide_msgs2 has no ActuatorStatus.CLAW_CLOSED: CLAW_BOTTLE_CLOSED for it)"
            cflags="$cflags -Driptide_msgs2__msg__ActuatorStatus__CLAW_CLOSED=riptide_msgs2__msg__ActuatorStatus__CLAW_BOTTLE_CLOSED"
        fi
        ;;
    esac
    CFLAGS="$cflags" cmake -S "$tf/$p" -B "$tf/$p/build" "${extra[@]}"
    cmake --build "$tf/$p/build" -j "$(nproc 2>/dev/null || echo 4)"
    ls "$tf/$p/build/"*_with_bl.uf2
done
