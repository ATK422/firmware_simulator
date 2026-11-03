#!/bin/sh
# Builds the C titan_firmware programs the emulator tests use, into build/c (outside external/):
#   * examples/basic_template (titan_logger + titan_safety on UART0, no micro-ROS)
#   * the C Titan bootloader as configured for talos/PowerBoard (client 1) and talos/LEDBoard (client 6)
# The board applications themselves need the prebuilt micro-ROS library (lib/micro_ros_pico/build.sh, Docker).
# Needs gcc-arm-none-eabi, cmake, ninja and `git -C external/titan_firmware submodule update --init --recursive`.
set -e
cd "$(dirname "$0")/.."
TF=external/titan_firmware
mkdir -p build/c
cmake -G Ninja -S $TF/examples/basic_template -B build/c/basic_template
ninja -C build/c/basic_template
cmake -G Ninja -S $TF/talos/PowerBoard -B build/c/PowerBoard
ninja -C build/c/PowerBoard titan_bootloader
cmake -G Ninja -S $TF/talos/LEDBoard -B build/c/LEDBoard
ninja -C build/c/LEDBoard titan_bootloader
echo "built:"
ls build/c/basic_template/template_project.elf build/c/*/titan_lib/titan_bootloader/src/titan_bootloader-build/titan_bootloader.elf
# An over-the-air application behind the bootloader, in two versions, for the upload test
cmake -G Ninja -S test_programs/c_bl_app -B build/c/bl_app_v0 -DAPP_MINOR=0
ninja -C build/c/bl_app_v0
cmake -G Ninja -S test_programs/c_bl_app -B build/c/bl_app_v1 -DAPP_MINOR=1
ninja -C build/c/bl_app_v1
