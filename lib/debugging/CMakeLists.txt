add_library(debugging STATIC
  ./src/parameters.cpp
)

target_include_directories(debugging PUBLIC
  ./include
)

target_link_libraries(debugging PUBLIC
  pico_stdlib
  pico_multicore
)
