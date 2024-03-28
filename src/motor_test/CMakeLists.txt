add_executable(motor_test
    motor_test.cpp
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
    pico_cyw43_arch_none
)

pico_add_extra_outputs(motor_test)
