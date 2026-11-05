# Included by scripts/build_c_boards.sh after SBCBoard's project() (CMAKE_PROJECT_sbc_board_INCLUDE), without
# changing the checkout: titan_firmware's SBCBoard (at 058df2a, work in progress) includes driver/canbus.h and
# micro_ros_pico/transport_can.h and calls canbus_init, but links only driver_mcp251Xfd_ll_driver, so it does not
# compile. Once its CMakeLists.txt has been read, link the two libraries those come from.
cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL target_link_libraries sbc_board PUBLIC driver_canbus micro_ros_can)
