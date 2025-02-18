#pragma once

#include <memory>

#include "lis3.h"
#include "lsm6.h"

/** Estimates the xy-orientation of the IMU based on accelerometer and gyroscope data. */
class OrientationEstimator {
 public:
  /**
   * Creates a new orientation estimator.
   * @param imu The IMU to read sensor data from.
   * @param imu_rotation The matrix to convert from IMU to device coordinates
   */
  OrientationEstimator(std::shared_ptr<LSM6> lsm6, std::shared_ptr<LIS3> lis3);
  /**
   * Updates the orientation estimate and north vector based on the latest sensor data.
   * @param force If true, availability of new data will not be checked.
   * @returns True if the orientation was updated, false if no new data was available.
   */
  bool update(bool force = false);
  bool update_up(bool force = false);
  bool update_north(bool force = false);

  /** Returns the current orientation as a rotation matrix. */
  Eigen::Matrix3f orientation() const;

  /** Returns the current orientation as the global up vector in device-local coordinates. */
  const Eigen::Vector3f& up() const { return _up; }

  /** Returns the current orientation as the global north vector in device-local coordinates. */
  Eigen::Vector3f north() const { return _north.normalized(); }

  /** Returns the current angle of the device around the up axis in radians. */
  float horizonatal_angle() const;

  /** Returns the expected time to wait between new sensor data */
  const uint64_t update_period_us() const { return _lsm6->period_us(); }

#ifndef NDEBUG
  bool show_debug = false;
#endif

 private:
  std::shared_ptr<LSM6> _lsm6;
  std::shared_ptr<LIS3> _lis3;
  Eigen::Vector3f _up;
  Eigen::Vector3f _north;
  uint32_t _last_update = 0;

  /** Returns the up vector based on the accelerometer. */
  Eigen::Vector3f get_accel_up() const;

  /** Returns the (unnormalized) direction of north. */
  Eigen::Vector3f get_north() const;
};
