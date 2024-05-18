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
  const auto imu = std::make_shared<LSM6>(
    7, accel_config, gyro_config, true, IMU_CALIBRATION,
    Eigen::Matrix3f{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
  );
  if (not imu->is_connected()) {
    printf("IMU not connected\n");
    return 1;
  }

  OrientationEstimator orientation_estimator{imu};

  while (true) {
    if (orientation_estimator.update()) {
      const auto up = orientation_estimator.up();
      show_vector("up", up.x(), up.y(), up.z());
    }
  }
}