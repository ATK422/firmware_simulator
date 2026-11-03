# titan_firmware's examples/basic_template built as an over-the-air application behind the C Titan bootloader
# (LEDBoard wiring: CAN client 6 on the internal bus). APP_MINOR sets the version, so an upload can be told apart.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD talos/led_board)
set(UWRT_ROBOT talos)
if (NOT DEFINED APP_MINOR)
    set(APP_MINOR 0)
endif()

get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../external/titan_firmware" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

project(bl_app C CXX ASM)
titan_firmware_init()

file(GLOB_RECURSE SOURCES "${REPO_DIR}/examples/basic_template/src/*.c")
add_executable(bl_app ${SOURCES})
target_include_directories(bl_app PUBLIC ${REPO_DIR}/examples/basic_template/include)
titan_generate_version_tag(bl_app 1 ${APP_MINOR})
pico_enable_stdio_uart(bl_app 1)
pico_set_program_name(bl_app "Bootloader test app")
titan_use_upload_tool(bl_app)
titan_enable_bootloader(bl_app can)
pico_add_extra_outputs(bl_app)
target_link_libraries(bl_app PUBLIC pico_stdlib titan_logger titan_safety)
