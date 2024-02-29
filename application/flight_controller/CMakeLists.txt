cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})


if(TEST_BUILD)
    set(EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/testing/preferences.conf)
else()
    set(EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/preferences.conf)
endif()

list(APPEND EXTRA_ZEPHYR_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dummysensor
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ms5611
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fjalar)

add_subdirectory(../common ../build/common)

add_subdirectory(lib)

# main
target_sources(app PRIVATE
    src/flight_state.c
    src/sensors.c
    src/communication.c
    src/commands.c
    src/actuation.c
    src/filter.c
)

if(TEST_BUILD)
    message(STATUS "Building test")
    target_sources(app PRIVATE
        testing/test_main.c
    )
else()
    message(STATUS "Building app")
    target_sources(app PRIVATE
        src/main.c
    )
endif()


target_link_libraries(app PRIVATE protocol)
target_link_libraries(app PRIVATE minmea)