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
