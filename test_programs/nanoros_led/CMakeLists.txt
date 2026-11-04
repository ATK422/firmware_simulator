# The nano-ROS test board (see main.c): osu-uwrt/nano_ros's libnanoros and the code nano_ros_codegen generates from
# ros.toml, on titan_firmware's CAN driver, for the Talos LED board (CANmore client 6), without the bootloader.
# Build (needs a nano_ros checkout and its codegen built; neither is modified):
#   make -C test_programs nanoros_led NANO_ROS_DIR=~/nano_ros
# NANO_ROS_GEN_DIR is where the codegen wrote nano_ros_codegen.h and its two .c files.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD talos/led_board)
set(UWRT_ROBOT talos)

if(NOT DEFINED REPO_DIR)
    get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../external/titan_firmware" ABSOLUTE)
endif()
if(NOT DEFINED NANO_ROS_DIR OR NOT DEFINED NANO_ROS_GEN_DIR)
    message(FATAL_ERROR "set NANO_ROS_DIR (a nano_ros checkout) and NANO_ROS_GEN_DIR (the codegen's output)")
endif()
include(${REPO_DIR}/global_defs.cmake)

project(nanoros_led C CXX ASM)
titan_firmware_init()

add_subdirectory(${NANO_ROS_DIR}/libnanoros ${CMAKE_CURRENT_BINARY_DIR}/libnanoros)

add_executable(nanoros_led main.c ${NANO_ROS_GEN_DIR}/nano_ros_serde_codegen.c ${NANO_ROS_GEN_DIR}/nano_ros_impl_codegen.c)
target_include_directories(nanoros_led PUBLIC ${NANO_ROS_GEN_DIR})
pico_enable_stdio_uart(nanoros_led 1)
pico_set_program_name(nanoros_led "nano-ROS test board")
pico_add_extra_outputs(nanoros_led)
target_link_libraries(nanoros_led PUBLIC pico_stdlib driver_canbus libnanoros)
