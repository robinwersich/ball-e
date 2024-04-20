# --- blink test ---
add_executable(blink
    blink.c
)

target_include_directories(blink PRIVATE
    ${INCLUDE_DIR}
)

target_link_libraries(blink
    pico_stdlib
    pico_cyw43_arch_none
)

pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

pico_add_extra_outputs(blink)

# --- motor test ---
add_executable(motor_test
    motor_test.cpp
    ../motor_drivers/dri0044.cpp
    ../pwm_slice.cpp
)

target_include_directories(motor_test PRIVATE
    ${INCLUDE_DIR}
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
)

pico_add_extra_outputs(motor_test)

# --- decoder test ---
add_executable(decoder_test
    decoder_test.cpp
    ../motor_decoder.cpp
    ../interrupt_handler.cpp
)

target_include_directories(decoder_test PRIVATE
    ${INCLUDE_DIR}
)

target_link_libraries(decoder_test
    pico_stdlib
    pico_cyw43_arch_none
)

pico_enable_stdio_usb(decoder_test 1)
pico_enable_stdio_uart(decoder_test 0)

pico_add_extra_outputs(decoder_test)

# --- pid driver test ---
add_executable(pid_driver_test
    pid_driver_test.cpp
    ../motor_driver_pid.cpp
    ../motor_decoder.cpp
    ../velocity_measurement.cpp
    ../pid.cpp
    ../pwm_slice.cpp
    ../interrupt_handler.cpp
    ../motor_drivers/dri0044.cpp
)

target_include_directories(pid_driver_test PRIVATE
    ${INCLUDE_DIR}
)

target_link_libraries(pid_driver_test
    pico_stdlib
    hardware_pwm
)

pico_add_extra_outputs(pid_driver_test)
