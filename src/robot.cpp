#include "robot.h"

#include <cmath>

Robot::Robot(
  std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator,
  PidGains balance_gains, std::optional<LowPassFilter> balance_filter
)
  : _wheels{std::move(wheels)}
  , _orientation_estimator{std::move(orientation_estimator)}
  , _pid_x{-1, 1, balance_gains, balance_filter}
  , _pid_y{-1, 1, balance_gains, balance_filter} {
  critical_section_init(&_cs);
}

Robot::~Robot() {
  stop_updating();
  critical_section_deinit(&_cs);
}

void Robot::set_speed(float x, float y, float rot) {
  critical_section_enter_blocking(&_cs);
  _speed_x = x;
  _speed_y = y;
  _speed_rot = rot;
  critical_section_exit(&_cs);
}

void Robot::set_drive_speed(float x, float y) {
  critical_section_enter_blocking(&_cs);
  _speed_x = x;
  _speed_y = y;
  critical_section_exit(&_cs);
}

void Robot::set_rotate_speed(float speed) {
  critical_section_enter_blocking(&_cs);
  _speed_rot = speed;
  critical_section_exit(&_cs);
}

void Robot::stop() {
  _speed_x = 0;
  _speed_y = 0;
  _speed_rot = 0;
}

void Robot::start_updating() {
  add_repeating_timer_us(
    _orientation_estimator.update_period_us(),
    [](repeating_timer_t* rt) {
      static_cast<Robot*>(rt->user_data)->update();
      return true;
    },
    this, &_update_timer
  );
}

void Robot::stop_updating() { cancel_repeating_timer(&_update_timer); }

void Robot::update() {
  _orientation_estimator.update(true);
  _balancing_mode ? update_balancing() : update_ground();
}

void Robot::update_ground() { drive(_speed_x, _speed_y, _speed_rot); }

void Robot::update_balancing() {
  // the down vector represents how much gravity affects each of the axes
  const auto down = -_orientation_estimator.up();
  const auto speed_x = _pid_x.compute(down.x());
  const auto speed_y = _pid_y.compute(down.y());
  // because of physics (tm), the effect of the motor movement on the angle is proportional
  // to cos²(angle) = 1 - sin²(angle) = 1 - down²
  const auto efficiency_x = 1 - down.x() * down.x();
  const auto efficiency_y = 1 - down.y() * down.y();
  // TODO: feed speed through motor PID controller
  drive(speed_x / efficiency_x, speed_y / efficiency_y, _speed_rot);
}

void Robot::set_balancing(bool enabled) {
  if (_balancing_mode == false and enabled == true) {
    _pid_x.reset();
    _pid_y.reset();
  }
  _balancing_mode = enabled;
}

void Robot::drive(float x, float y, float rot) {
  for (const auto& wheel : _wheels) { wheel.drive(x, y, rot); }
}
