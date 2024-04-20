add_library(pid STATIC
  ./src/pid.cpp
)

target_include_directories(pid PUBLIC
  ./include
)

target_link_libraries(pid PRIVATE
  pico_stdlib_headers
)
