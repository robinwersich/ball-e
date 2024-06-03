#pragma once

#include <memory>

#include "lsm6.h"

/** Estimates the xy-orientation of the IMU based on accelerometer and gyroscope data. */
class OrientationEstimator {
 public:
  /**
   * Creates a new orientation estimator.
   * @param imu The IMU to read sensor data from.
   * @param imu_rotation The matrix to convert from IMU to device coordinates
   */
  OrientationEstimator(std::shared_ptr<LSM6> imu);
  /**
   * Updates the orientation estimate based on the latest sensor data.
   * @param force If true, availability of new data will not be checked.
   * @returns True if the orientation was updated, false if no new data was available.
   */
  bool update(bool force = false);

  /** Returns the current orientation as the global up vector in device-local coordinates. */
  const Eigen::Vector3f& up() const { return _up; }

  /** Returns the current angle of the device around the up vector in radians. */
  float z_angle() const { return _z_angle; }

  /** Returns the expected time to wait between new sensor data */
  const uint64_t update_period_us() const { return _imu->period_us(); }

#ifndef NDEBUG
  bool show_debug = false;
#endif

 private:
  std::shared_ptr<LSM6> _imu;
  Eigen::Vector3f _up;
  float _z_angle = 0;  // rotation around the z axis
  uint32_t _last_update = 0;

  /** Returns the up vector based on the accelerometer. */
  Eigen::Vector3f get_accel_up() const;
};
