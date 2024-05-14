#pragma once

#include <memory>

#include "lsm6.h"

/** Estimates the xy-orientation of the IMU based on accelerometer and gyroscope data. */
class OrientationEstimator {
 public:
  /**
   * Creates a new orientation estimator.
   * @param imu The IMU to read sensor data from.
   * @param imu_orientation The matrix to convert from IMU to device coordinates
   */
  OrientationEstimator(
    std::shared_ptr<LSM6> imu, Eigen::Matrix3f imu_orientation = Eigen::Matrix3f::Identity()
  );
  /**
   * Updates the orientation estimate based on the latest sensor data.
   * @returns True if the orientation was updated, false if no new data was available.
   */
  bool update();

  /** Returns the current orientation as the global up vector in device-local coordinates. */
  const Eigen::Vector3f& up() const { return _up; }
  /** Returns the current orientation as the device normal vector in global coordinates. */
  Eigen::Vector3f orientation() const { return {-_up.x(), -_up.y(), _up.z()}; }

 private:
  std::shared_ptr<LSM6> _imu;
  Eigen::Matrix3f _imu_orientation;
  Eigen::Vector3f _up;
  uint32_t _last_update = 0;
};
