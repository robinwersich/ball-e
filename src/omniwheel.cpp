#include "omniwheel.h"

#include <cmath>

#include "matrix.h"

Omniwheel::Omniwheel(float angle, std::unique_ptr<MotorDriver> motor_driver, bool swap_direction)
  : _transform{Matrix<float, 2, 2>::rotate(-angle)}
  , _motor_driver{std::move(motor_driver)}
  , _swap_direction{swap_direction} {}

void Omniwheel::drive(float x, float y) const {
  auto [secondary_speed, primary_speed] = (_transform * Matrix<float, 2, 1>{{x, y}}).data;
  drive(primary_speed);
}

void Omniwheel::drive(float speed) const { _motor_driver->drive(_swap_direction ? -speed : speed); }

void Omniwheel::stop() const { _motor_driver->stop(); }
