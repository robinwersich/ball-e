cmake_minimum_required(VERSION 3.13)

# PICO_BOARD should be set to 'pico_w', or not set at all.
if(DEFINED PICO_BOARD AND NOT ${PICO_BOARD} STREQUAL "pico_w")
    message(FATAL_ERROR "PICO_BOARD must be set to 'pico_w' or not set at all")
else()
    set(PICO_BOARD "pico_w")
endif()

include(pico_sdk_import.cmake)
# use the modified version of btstack
include(lib/btcontrol/bluepad32_import.cmake)

project(robot-engineering-project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

if (DEFINED ENV{Eigen3_DIR} AND (NOT Eigen3_DIR))
    set(Eigen3_DIR $ENV{Eigen3_DIR})
    message("Using Eigen3_DIR from environment ('${Eigen3_DIR}')")
endif ()

find_package(Eigen3 REQUIRED CONFIG)

add_subdirectory(lib)
add_subdirectory(src)
