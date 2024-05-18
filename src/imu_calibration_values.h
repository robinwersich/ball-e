#pragma once

#include "imu_calibration.h"

static const ImuCalibration IMU_CALIBRATION{
  .gyro_bias = Eigen::Vector3f{0.477315, 0.182501, 1.420789},
  .accel_bias = Eigen::Vector3f{0.015350, -0.011141, 0.024156},
  .accel_transform =
    Eigen::Matrix3f{
      {0.987756, -0.000095, -0.004895},
      {-0.000095, 0.998067, 0.000316},
      {-0.004895, 0.000316, 0.993727}
    }
};
