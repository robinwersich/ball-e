#include "orientation_estimator.h"

#include <Eigen/Geometry>
#include <cmath>

#ifndef NDEBUG
#include "plot.h"
#endif
#include "pico/time.h"

const float GYRO_WEIGHT = 0.99;
const float ACCEL_WEIGHT = 1 - GYRO_WEIGHT;

OrientationEstimator::OrientationEstimator(std::shared_ptr<LSM6> lsm6, std::shared_ptr<LIS3> lis3)
  : _lsm6{std::move(lsm6)}, _lis3{std::move(lis3)}, _up{get_accel_up()}, _north{get_north()} {}

Eigen::Vector3f OrientationEstimator::get_accel_up() const {
  return _lsm6->read_acceleration().normalized();
}

Eigen::Vector3f OrientationEstimator::get_north() const { return _lis3->read_field(); }

bool OrientationEstimator::update(bool force) {
  const auto north_updated = update_north(force);
  const auto up_updated = update_up(force);
  return north_updated or up_updated;
}

bool OrientationEstimator::update_north(bool force) {
  if (not force and not _lis3->is_new_data_available()) return false;
  _north = get_north();
  return true;
}

bool OrientationEstimator::update_up(bool force) {
  if (not force and not _lsm6->is_new_data_available()) return false;

  // Calculate the up vector based on the accelerometer.
  // This value doesn't drift, but is noisy when imu is moved.
  const auto accel_up = get_accel_up();

  // Calculate the up vector based on the gyroscope.
  // This value drifts over time, but is less affected by movement / noise.
  const auto now = time_us_32();
  const auto dt = (now - _last_update) / 1e6f;
  _last_update = now;
  const auto rotation = _lsm6->read_rotation();
  const auto velocity_dps = rotation.norm();
  const auto rotation_axis = rotation / velocity_dps;
  const auto rotation_angle = static_cast<float>(velocity_dps * dt * M_PI / 180);
  // rotation measures imu rotation, so up vector rotates in opposite direction
  const auto gyro_up = Eigen::AngleAxisf{-rotation_angle, rotation_axis} * _up;

  // Use complementary filter to combine the two orientations.
  // Using mainly gyroscope but adding a bit of the accelerometer avoids drift and noise.
  _up = (gyro_up * GYRO_WEIGHT + accel_up * ACCEL_WEIGHT).normalized();

#ifndef NDEBUG
  if (show_debug) {
    show_vector("gyro_up", gyro_up.x(), gyro_up.y(), gyro_up.z());
    show_vector("accel_up", accel_up.x(), accel_up.y(), accel_up.z());
    show_vector("up", _up.x(), _up.y(), _up.z());
  }
#endif

  return true;
}

float OrientationEstimator::horizonatal_angle() const { 
  const auto right = _north.cross(_up);
  return -std::atan2(right.y(), right.x());
 }

Eigen::Matrix3f OrientationEstimator::orientation() const {
  const auto north = _north.normalized();
  const auto right = _north.cross(_up).normalized();
  const auto forward = _up.cross(right).normalized();

  // The orientation matrix is the inverse and thus transposed version of the rotation matrix.
  Eigen::Matrix3f orientation;
  orientation.row(0) = right;
  orientation.row(1) = forward;
  orientation.row(2) = up();
  return orientation;
}
