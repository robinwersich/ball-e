#pragma once

#include <memory>

#include "lsm6.h"

/** Estimates the xy-orientation of the IMU based on accelerometer and gyroscope data. */
class OrientationEstimator {
 public:
  OrientationEstimator(std::shared_ptr<LSM6> imu);
  /**
   * Updates the orientation estimate based on the latest sensor data.
   * @returns True if the orientation was updated, false if no new data was available.
   */
  bool update();

  /** Returns the current orientation estimate. */
  const Vector2D<float>& orientation() const { return _orientation; }

 private:
  std::shared_ptr<LSM6> _imu;
  Vector2D<float> _orientation;
  uint32_t _last_update = 0;
};
