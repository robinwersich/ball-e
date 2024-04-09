add_subdirectory(playground)

add_executable(robot_firmware
    main.cpp
    robot.cpp
    omniwheel.cpp
    pwm_slice.cpp
    motor_drivers/dri0044.cpp
)

target_include_directories(robot_firmware PRIVATE
    ${INCLUDE_DIR}
)

target_link_libraries(robot_firmware
    pico_stdlib
    hardware_pwm
)

pico_add_extra_outputs(robot_firmware)
