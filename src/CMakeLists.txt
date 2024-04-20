add_subdirectory(playground)

add_executable(robot_firmware
    main.cpp
    robot.cpp
    omniwheel.cpp
)

target_link_libraries(robot_firmware
    pico_stdlib
    hardware_pwm
    motor_hardware
    pid
)

pico_add_extra_outputs(robot_firmware)
