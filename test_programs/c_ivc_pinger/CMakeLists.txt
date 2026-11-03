# The C IVCBoard2 in pinger mode without micro-ROS: the same blake/ivc sources as c_ivc_modem (whose stub ros.h it
# uses), with a main.c that switches the modem to 25 kHz pinger mode (what `ivc/pinger/set_freq_khz` 25 and
# `ivc/pinger/enable` do) and prints every amplitude the pinger path publishes as `ROS pinger_amp X` on UART0.
# Build (extracts the sources from the titan_firmware submodule's blake/ivc branch into build/c/src):
#   make -C test_programs c_ivc_pinger
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

project(c_ivc_pinger C CXX ASM)
titan_firmware_init()

add_executable(c_ivc_pinger main.c
    ${IVC_DIR}/src/ivc.c ${IVC_DIR}/src/rx.c ${IVC_DIR}/src/tx.c ${IVC_DIR}/src/consensus.c ${IVC_DIR}/src/protocol.c
    ${IVC_DIR}/src/storage.c ${IVC_DIR}/src/fft/fft.c ${IVC_DIR}/src/fft/filter.c ${IVC_DIR}/src/fft/kiss_fft.c
    ${IVC_DIR}/src/fft/kiss_fftr.c)
# stub/ros.h stands in for the micro-ROS one
target_include_directories(c_ivc_pinger BEFORE PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../c_ivc_modem/stub)
target_include_directories(c_ivc_pinger PUBLIC ${IVC_DIR}/include)
pico_enable_stdio_uart(c_ivc_pinger 1)
pico_set_program_name(c_ivc_pinger "C IVC pinger")
pico_add_extra_outputs(c_ivc_pinger)
target_link_libraries(c_ivc_pinger PUBLIC pico_stdlib titan_logger titan_queue hardware_pwm hardware_adc hardware_dma
    hardware_flash hardware_sync)
