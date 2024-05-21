add_library(imu STATIC
  ./src/lsm6.cpp
  ./src/orientation_estimator.cpp
)

target_include_directories(imu PUBLIC
  ./include
)

target_link_libraries(imu
  PUBLIC
    pico_stdlib
    Eigen3::Eigen
    hardware_i2c
  PRIVATE
    debug debugging
)
