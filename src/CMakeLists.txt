add_subdirectory(playground)

add_executable(robot_firmware
    main.cpp
    robot.cpp
    omniwheel.cpp
    motor_driver_kickstart.cpp
)

target_link_libraries(robot_firmware
    pico_stdlib
    motor_hardware
    btcontrol
    imu
    pid
)

pico_enable_stdio_usb(robot_firmware 1)
pico_enable_stdio_uart(robot_firmware 0)

pico_add_extra_outputs(robot_firmware)
