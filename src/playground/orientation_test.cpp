#include <stdio.h>

#include "imu_calibration_values.h"
#include "lsm6.h"
#include "orientation_estimator.h"
#include "pico/stdlib.h"
#include "plot.h"

int main() {
  stdio_init_all();
  sleep_ms(2000);

  LSM6::AccelConfig accel_config{.odr = lsm6::odr::HZ_208, .fs = lsm6::fs::acc::G_2};
  LSM6::GyroConfig gyro_config{.odr = lsm6::odr::HZ_208, .fs = lsm6::fs::gyro::DPS_1000};
  const auto i2c_port = init_i2c_port(7, LSM6::MAX_BAUDRATE);
  const auto lsm6 = std::make_shared<LSM6>(
    i2c_port, accel_config, gyro_config, true, LSM6_CALIBRATION,
    Eigen::Matrix3f{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
  );
  if (not lsm6->is_connected()) {
    printf("IMU not connected\n");
    return 1;
  }
  const auto lis3 = std::make_shared<LIS3>(
    i2c_port, LIS3::MagnetometerConfig{.odr = lis3::odr::HZ_155, .fs = lis3::fs::GS_4}, true,
    LIS3_CALIBRATION, Eigen::Matrix3f{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
  );

  OrientationEstimator orientation_estimator{lsm6, lis3};

  while (true) {
    orientation_estimator.update(true);
    const auto orientation = orientation_estimator.orientation();
    const auto x = orientation.col(0);
    const auto y = orientation.col(1);
    const auto z = orientation.col(2);
    show_vector("imu x", x.x(), x.y(), x.z());
    show_vector("imu y", y.x(), y.y(), y.z());
    show_vector("imu z", z.x(), z.y(), z.z());
    plot("angle", orientation_estimator.horizonatal_angle() * 180 / M_PI);
    sleep_ms(100);
  }
}