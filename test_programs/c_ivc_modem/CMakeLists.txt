# The C IVCBoard2 modem without micro-ROS: titan_firmware branch blake/ivc's talos/IVCBoard2 sources (ivc.c, rx.c,
# tx.c, consensus.c, protocol.c, storage.c, fft/), unmodified, with ros.c and main.c replaced by main.c here. It runs
# the same loop as IVCBoard2's main (ivc_init, then ivc_tick forever) without the ROS executor, so the C and Rust
# modems can be tested against each other without the Docker-built micro-ROS library:
#   * every byte received on UART0 is enqueued for transmission (what the ivc/enqueue_data callback does)
#   * every received packet is printed as `ROS ivc/rx_new N` (what ros_publish_rx publishes, as an Int8)
# Build (extracts the sources from the titan_firmware submodule's blake/ivc branch into build/c/src):
#   make -C test_programs c_ivc_modem
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD talos/ivc_board)
set(UWRT_ROBOT talos)

if(NOT DEFINED REPO_DIR)
    get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../external/titan_firmware" ABSOLUTE)
endif()
if(NOT DEFINED IVC_DIR)
    get_filename_component(IVC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../build/c/src/talos/IVCBoard2" ABSOLUTE)
endif()
include(${REPO_DIR}/global_defs.cmake)

project(c_ivc_modem C CXX ASM)
titan_firmware_init()

add_executable(c_ivc_modem main.c
    ${IVC_DIR}/src/ivc.c ${IVC_DIR}/src/rx.c ${IVC_DIR}/src/tx.c ${IVC_DIR}/src/consensus.c ${IVC_DIR}/src/protocol.c
    ${IVC_DIR}/src/storage.c ${IVC_DIR}/src/fft/fft.c ${IVC_DIR}/src/fft/filter.c ${IVC_DIR}/src/fft/kiss_fft.c
    ${IVC_DIR}/src/fft/kiss_fftr.c)
# stub/ros.h stands in for the micro-ROS one
target_include_directories(c_ivc_modem BEFORE PUBLIC stub)
target_include_directories(c_ivc_modem PUBLIC ${IVC_DIR}/include)
pico_enable_stdio_uart(c_ivc_modem 1)
pico_set_program_name(c_ivc_modem "C IVC modem")
pico_add_extra_outputs(c_ivc_modem)
target_link_libraries(c_ivc_modem PUBLIC pico_stdlib titan_logger titan_queue hardware_pwm hardware_adc hardware_dma
    hardware_flash hardware_sync)
