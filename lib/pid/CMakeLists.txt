add_library(pid STATIC
  ./src/pid.cpp
)

target_include_directories(pid PUBLIC
  ./include
)

target_link_libraries(pid
  PUBLIC
    pico_stdlib
    filter
  PRIVATE
    debug debugging
)
