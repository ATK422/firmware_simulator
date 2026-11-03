# titan_firmware's titan_dual_stdio_usb (two CDC ACM ports: stdio and the micro-ROS transport's) in a small program
# for the emulator's USB host model: stdio greets the host once the port is open, the secondary port echoes what
# it receives in upper case.
#   cmake -G Ninja -S test_programs/c_dual_usb -B build/c/c_dual_usb && ninja -C build/c/c_dual_usb
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico)

get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../external/titan_firmware" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

project(c_dual_usb C CXX ASM)
titan_firmware_init()

add_executable(c_dual_usb main.c)
pico_enable_stdio_uart(c_dual_usb 0)
pico_set_program_name(c_dual_usb "Dual CDC test")
pico_add_extra_outputs(c_dual_usb)
target_link_libraries(c_dual_usb PUBLIC pico_stdlib titan_dual_stdio_usb)
