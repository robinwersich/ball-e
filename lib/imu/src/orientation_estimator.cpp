#include "orientation_estimator.h"

#include <cmath>
#include "pico/time.h"

const float GYRO_WEIGHT = 0.98;
const float ACCEL_WEIGHT = 1 - GYRO_WEIGHT;

OrientationEstimator::OrientationEstimator(std::shared_ptr<LSM6> imu)
  : _imu{imu}, _orientation{0, 0} {}

bool OrientationEstimator::update() {
  if (not _imu->is_new_data_available()) return false;

  // Calculate the orientation based on the accelerometer.
  // This value doesn't drift, but is noisy when imu is moved.
  const auto& [ax, ay, az] = _imu->read_acceleration().data;
  const float roll = std::atan2(ay, std::sqrt(ax * ax + az * az)) * 180 / M_PI;
  const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 180 / M_PI;
  const Vector2D accel_orientation{roll, pitch};

  // Calculate the orientation based on the gyroscope.
  // This value drifts over time, but is less affected by movement / noise.
  const auto& [gx, gy, gz] = _imu->read_rotation().data;
  const uint32_t now = time_us_32();
  const float dt = (now - _last_update) / 1e6f;
  _last_update = now;
  const Vector2D gyro_orientation{_orientation.x() + gx * dt, _orientation.y() + gy * dt};

  // Use complementary filter to combine the two orientations.
  // Using mainly gyroscope but adding a bit of the accelerometer avoids drift and noise.
  _orientation = gyro_orientation * GYRO_WEIGHT + accel_orientation * ACCEL_WEIGHT;

  return true;
}
