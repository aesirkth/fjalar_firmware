cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(OPENOCD_NRF5_SUBFAMILY "nrf52")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
include(${ZEPHYR_BASE}/cmake/cfb.cmake NO_POLICY_SCOPE)

project(tracker)
target_sources(app PRIVATE
    src/main.c
    src/display.c
    src/communication.c
    src/sensors.c
)

add_subdirectory(../common ../build/common)
target_link_libraries(app PRIVATE protocol)
target_link_libraries(app PRIVATE minmea)

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)
# generate_cfb_font_for_target(app "fonts/Big Apple 3PM.ttf" "${gen_dir}/cfb_custom_font.h" 28 32)
# generate_cfb_font_for_target(app "fonts/m12.TTF" "${gen_dir}/cfb_custom_font.h" 34 32 -s 32)
# generate_cfb_font_for_target(app "fonts/Daydream.ttf" "${gen_dir}/cfb_custom_font.h" 23 24 -s 16)
# generate_cfb_font_for_target(app "fonts/Daydream.ttf" "${gen_dir}/cfb_custom_font.h" 28 32 -s 20)

generate_cfb_font_for_target(app "fonts/DotGothic16-Regular.ttf" "${gen_dir}/cfb_custom_font.h" 6 16)
