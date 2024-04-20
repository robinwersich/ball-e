cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(robot-engineering-project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

add_subdirectory(lib)
add_subdirectory(src)
