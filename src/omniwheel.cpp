#include "omniwheel.h"

#include <cmath>

Omniwheel::Omniwheel(float angle, std::unique_ptr<MotorDriver> motor_driver, bool swap_direction)
  : _wheel_rotation{Eigen::Rotation2Df(angle * M_PI / 180)}
  , _motor_driver{std::move(motor_driver)}
  , _swap_direction{swap_direction} {}

void Omniwheel::drive(float x, float y) const {
  const auto speed = _wheel_rotation * Eigen::Vector2f{{x, y}};
  drive(speed.y());
}

void Omniwheel::drive(float speed) const { _motor_driver->drive(_swap_direction ? -speed : speed); }

void Omniwheel::stop() const { _motor_driver->stop(); }
