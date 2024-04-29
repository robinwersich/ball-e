add_library(motor_hardware STATIC
  ./src/pwm_slice.cpp
  ./src/motor_decoder.cpp
  ./src/motor_state.cpp
  ./src/motor_drivers/dri0044.cpp
  ./src/motor_drivers/l298n.cpp
)

target_include_directories(motor_hardware PUBLIC
  ./include
)

target_link_libraries(motor_hardware PUBLIC
  pico_stdlib
  hardware_pwm
)
