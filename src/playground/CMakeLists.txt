# --- blink test ---
add_executable(blink
    blink.c
)

target_include_directories(blink PRIVATE
    ..
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
)

target_include_directories(motor_test PRIVATE
    ..
)

target_link_libraries(motor_test
    pico_stdlib
    motor_hardware
)

pico_add_extra_outputs(motor_test)

# --- decoder test ---
add_executable(decoder_test
    decoder_test.cpp
)

target_include_directories(decoder_test PRIVATE
    ..
)

target_link_libraries(decoder_test
    pico_stdlib
    pico_cyw43_arch_none
    motor_hardware
)

pico_enable_stdio_usb(decoder_test 1)
pico_enable_stdio_uart(decoder_test 0)

pico_add_extra_outputs(decoder_test)

# --- pid driver test ---
add_executable(pid_driver_test
    pid_driver_test.cpp
    ../motor_driver_pid.cpp
)

target_include_directories(pid_driver_test PRIVATE
    ..
)

target_link_libraries(pid_driver_test
    pico_stdlib
    motor_hardware
    pid
    debugging
)

pico_enable_stdio_usb(pid_driver_test 1)
pico_enable_stdio_uart(pid_driver_test 0)

pico_add_extra_outputs(pid_driver_test)

# --- debugging test ---
add_executable(debugging_test
    debugging_test.cpp
)

target_include_directories(debugging_test PRIVATE
    ..
)

target_link_libraries(debugging_test
    pico_stdlib
    debugging
)

pico_enable_stdio_usb(debugging_test 1)
pico_enable_stdio_uart(debugging_test 0)

pico_add_extra_outputs(debugging_test)
