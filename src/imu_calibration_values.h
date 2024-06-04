#pragma once

#include "imu_calibration.h"

static const LSM6Calibration LSM6_CALIBRATION{
  .gyro_bias = Eigen::Vector3f{0.477315, 0.182501, 1.420789},
  .accel_bias = Eigen::Vector3f{0.015350, -0.011141, 0.024156},
  .accel_transform =
    Eigen::Matrix3f{
      {0.987756, -0.000095, -0.004895},
      {-0.000095, 0.998067, 0.000316},
      {-0.004895, 0.000316, 0.993727}
    }
};

static const LIS3Calibration LIS3_CALIBRATION{
  .mag_bias = Eigen::Vector3f{0.083621, -0.1706595, -0.298227},
};
