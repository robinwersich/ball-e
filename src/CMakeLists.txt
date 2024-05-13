add_subdirectory(playground)

add_executable(robot_firmware
    main.cpp
    robot.cpp
    omniwheel.cpp
)

target_link_libraries(robot_firmware
    pico_stdlib
    motor_hardware
    btcontrol
    maths
)

pico_add_extra_outputs(robot_firmware)
