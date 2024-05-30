add_library(filter STATIC
  ./src/lowpass.cpp
)

target_include_directories(filter PUBLIC
  ./include
)
