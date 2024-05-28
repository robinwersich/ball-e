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
    motor_hardware
    debugging
    pico_multicore
)

pico_enable_stdio_usb(decoder_test 1)
pico_enable_stdio_uart(decoder_test 0)

pico_add_extra_outputs(decoder_test)

# --- velocity test ---
add_executable(velocity_test
    velocity_test.cpp
)

target_include_directories(velocity_test PRIVATE
    ..
)

target_link_libraries(velocity_test
    pico_stdlib
    motor_hardware
    debugging
    pico_multicore
)

pico_enable_stdio_usb(velocity_test 1)
pico_enable_stdio_uart(velocity_test 0)

pico_add_extra_outputs(velocity_test)

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

# --- gamepad test ---
add_executable(gamepad_test
    gamepad_test.cpp
)

target_include_directories(gamepad_test PRIVATE
    ..
)

target_link_libraries(gamepad_test
    pico_stdlib
    btcontrol
)

pico_enable_stdio_usb(gamepad_test 1)
pico_enable_stdio_uart(gamepad_test 0)

pico_add_extra_outputs(gamepad_test)

# --- IMU test ---
add_executable(imu_test
    imu_test.cpp
)

target_include_directories(imu_test PRIVATE
    ..
)

target_link_libraries(imu_test
    pico_stdlib
    imu
    debugging
)

pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)

pico_add_extra_outputs(imu_test)


# --- orientation test ---
add_executable(orientation_test
    orientation_test.cpp
)

target_include_directories(orientation_test PRIVATE
    ..
)

target_link_libraries(orientation_test
    pico_stdlib
    imu
    debugging
)

pico_enable_stdio_usb(orientation_test 1)
pico_enable_stdio_uart(orientation_test 0)

pico_add_extra_outputs(orientation_test)


# --- IMU calibration ---
add_executable(imu_calibration
    imu_calibration.cpp
)

target_include_directories(imu_calibration PRIVATE
    ..
)

target_link_libraries(imu_calibration
    pico_stdlib
    imu
)

pico_enable_stdio_usb(imu_calibration 1)
pico_enable_stdio_uart(imu_calibration 0)

pico_add_extra_outputs(imu_calibration)


# --- kickstart driver test ---
add_executable(kickstart_driver_test
    kickstart_driver_test.cpp
    ../motor_driver_kickstart.cpp
)

target_include_directories(kickstart_driver_test PRIVATE
    ..
)

target_link_libraries(kickstart_driver_test
    pico_stdlib
    debugging
    motor_hardware
    pico_multicore
)

pico_enable_stdio_usb(kickstart_driver_test 1)
pico_enable_stdio_uart(kickstart_driver_test 0)

pico_add_extra_outputs(kickstart_driver_test)

# --- lp filter test ---
add_executable(lp_filter_test
    lp_filter_test.cpp
)

target_link_libraries(lp_filter_test
    pico_stdlib
    pico_multicore
    motor_hardware
    pid
    debugging
)

pico_enable_stdio_usb(lp_filter_test 1)
pico_enable_stdio_uart(lp_filter_test 0)

pico_add_extra_outputs(lp_filter_test)
