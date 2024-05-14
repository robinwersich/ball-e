#include <stdio.h>

#include "lsm6.h"
#include "orientation_estimator.h"
#include "pico/stdlib.h"
#include "plot.h"

int main() {
  stdio_init_all();
  sleep_ms(2000);

  const auto imu = std::make_shared<LSM6>(7, LSM6::AccelConfig{}, LSM6::GyroConfig{});
  if (not imu->is_connected()) {
    printf("IMU not connected\n");
    return 1;
  }

  OrientationEstimator orientation_estimator{imu};

  while (true) {
    if (orientation_estimator.update()) {
      const auto orientation = orientation_estimator.orientation();
      show_vector("orientation", orientation.x(), orientation.y(), orientation.z());
    }
  }
}