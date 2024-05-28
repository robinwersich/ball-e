add_library(pid STATIC
  ./src/pid.cpp
  ./src/lowpass.cpp
)

target_include_directories(pid PUBLIC
  ./include
)

target_link_libraries(pid
  PUBLIC
    pico_stdlib
  PRIVATE
    debug debugging
)
