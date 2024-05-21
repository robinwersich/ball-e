#include "orientation_estimator.h"

#include <Eigen/Geometry>
#include <cmath>

#include "pico/time.h"

const float GYRO_WEIGHT = 0.99;
const float ACCEL_WEIGHT = 1 - GYRO_WEIGHT;

OrientationEstimator::OrientationEstimator(std::shared_ptr<LSM6> imu)
  : _imu{std::move(imu)}, _up{get_accel_up()} {}

Eigen::Vector3f OrientationEstimator::get_accel_up() const {
  return _imu->read_acceleration().normalized();
}

bool OrientationEstimator::update(bool force) {
  if (not force and not _imu->is_new_data_available()) return false;

  // Calculate the up vector based on the accelerometer.
  // This value doesn't drift, but is noisy when imu is moved.
  const auto accel_up = get_accel_up();

  // Calculate the up vector based on the gyroscope.
  // This value drifts over time, but is less affected by movement / noise.
  const auto now = time_us_32();
  const auto dt = (now - _last_update) / 1e6f;
  _last_update = now;
  const auto rotation = _imu->read_rotation();
  const auto velocity_dps = rotation.norm();
  const auto rotation_axis = rotation / velocity_dps;
  const auto rotation_angle = static_cast<float>(velocity_dps * dt * M_PI / 180);
  // rotation measures imu rotation, so up vector rotates in opposite direction
  const auto gyro_up = Eigen::AngleAxisf{-rotation_angle, rotation_axis} * _up;

  // Use complementary filter to combine the two orientations.
  // Using mainly gyroscope but adding a bit of the accelerometer avoids drift and noise.
  _up = (gyro_up * GYRO_WEIGHT + accel_up * ACCEL_WEIGHT).normalized();

  return true;
}
