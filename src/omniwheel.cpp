#include "omniwheel.h"

#include <cmath>

#include "matrix.h"

Omniwheel::Omniwheel(float angle, std::unique_ptr<MotorDriver> motor_driver, bool swap_direction)
  : transform{Matrix<float, 2, 2>::rotate(-angle)}
  , motor_driver{std::move(motor_driver)}
  , swap_direction{swap_direction} {}

void Omniwheel::drive(float x, float y) const {
  auto [primary_speed, secondary_speed] = (transform * Matrix<float, 2, 1>{{x, y}}).data;
  drive(primary_speed);
}

void Omniwheel::drive(float speed) const { motor_driver->drive(swap_direction ? -speed : speed); }

void Omniwheel::stop() const { motor_driver->stop(); }
