#include "omniwheel.h"

#include <cmath>

Omniwheel::Omniwheel(
  float angle, std::unique_ptr<MotorDriver> motor_driver, const MotorState& motor_state
)
  : _wheel_direction{-std::sin(angle / 180 * M_PI), std::cos(angle / 180 * M_PI)}
  , _motor_driver{std::move(motor_driver)}
  , _motor_state{motor_state} {}

void Omniwheel::drive(const Eigen::Vector2f& global_speed, float local_speed) const {
  drive(get_local_speed(global_speed) + local_speed);
}

void Omniwheel::drive(const Eigen::Vector2f& speed) const { drive(speed, 0); }

void Omniwheel::drive(float speed) const { _motor_driver->drive(speed); }

void Omniwheel::stop() const { _motor_driver->stop(); }

float Omniwheel::compute_speed() {
  return _motor_state.compute_speed_rps();
}

const Eigen::Vector2f& Omniwheel::wheel_direction() const {
  return _wheel_direction;
}

float Omniwheel::get_local_speed(const Eigen::Vector2f global_speed) const {
  return global_speed.dot(_wheel_direction);
}