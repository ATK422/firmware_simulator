# pico-sdk conformance programs for the emulator, mirroring pico-examples' uart/hello_uart,
# multicore/hello_multicore and flash/program, and a flash unique ID reader (pico_unique_id). Built against
# titan_firmware's pico-sdk submodule:
#   cmake -G Ninja -S test_programs/pico -B build/pico && ninja -C build/pico
cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../../external/titan_firmware/lib/pico-sdk)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
project(pico_tests C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(hello_uart hello_uart.c)
target_link_libraries(hello_uart pico_stdlib)

add_executable(hello_multicore hello_multicore.c)
target_link_libraries(hello_multicore pico_stdlib pico_multicore hardware_divider hardware_interp hardware_sync)

add_executable(flash_program flash_program.c)
target_link_libraries(flash_program pico_stdlib hardware_flash)

# stdio over USB (TinyUSB CDC ACM), for the USB device controller and host model
add_executable(hello_usb hello_usb.c)
target_link_libraries(hello_usb pico_stdlib)
pico_enable_stdio_usb(hello_usb 1)
pico_enable_stdio_uart(hello_usb 0)

# The same linked to run from SRAM (no_flash), for loading over PICOBOOT without touching the flash
add_executable(hello_usb_ram hello_usb.c)
target_link_libraries(hello_usb_ram pico_stdlib)
pico_enable_stdio_usb(hello_usb_ram 1)
pico_enable_stdio_uart(hello_usb_ram 0)
pico_set_binary_type(hello_usb_ram no_flash)

add_executable(unique_id unique_id.c)
target_link_libraries(unique_id pico_stdlib pico_unique_id hardware_flash)
