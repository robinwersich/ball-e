add_library(pico_utility STATIC
  ./src/interrupt_handler.cpp
)

target_include_directories(pico_utility PUBLIC
  ./include
)

target_link_libraries(pico_utility PUBLIC
  pico_stdlib
)
