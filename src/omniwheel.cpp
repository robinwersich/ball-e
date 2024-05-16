#include "omniwheel.h"

#include <cmath>

Omniwheel::Omniwheel(float angle, std::unique_ptr<MotorDriver> motor_driver, bool swap_direction)
  : _wheel_rotation{Eigen::Rotation2Df(-angle * M_PI / 180)}
  , _motor_driver{std::move(motor_driver)}
  , _swap_direction{swap_direction} {}

void Omniwheel::drive(float x, float y) const {
  auto v = _wheel_rotation * Eigen::Vector2f{{x, y}};
  // ensure that total speed is not higher than 1
  const auto speed = v.norm();
  if (speed > 1) v /= speed;
  drive(v.y());
}

void Omniwheel::drive(float speed) const { _motor_driver->drive(_swap_direction ? -speed : speed); }

void Omniwheel::stop() const { _motor_driver->stop(); }
