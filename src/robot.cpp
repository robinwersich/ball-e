#include "robot.h"

Robot::Robot(
  std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator,
  PidGains balance_gains
)
  : _wheels{std::move(wheels)}
  , _orientation_estimator{std::move(orientation_estimator)}
  , _pid_x{-1, 1, balance_gains}
  , _pid_y{-1, 1, balance_gains} {
}

void Robot::drive(float x, float y) {
  _speed_x = x;
  _speed_y = y;
}

void Robot::rotate(float speed) { _speed_rot = speed; }

void Robot::stop() {
  _speed_x = 0;
  _speed_y = 0;
  _speed_rot = 0;
}

void Robot::start_updating() {
  add_repeating_timer_us(
    _orientation_estimator.update_period_us(),
    [](repeating_timer_t* rt) {
      reinterpret_cast<Robot*>(rt->user_data)->update();
      return true;
    },
    this, &_update_timer
  );
}

void Robot::stop_updating() { cancel_repeating_timer(&_update_timer); }

void Robot::update() {
  _orientation_estimator.update(true);
  if (_balancing_mode) {
    update_balancing();
  } else {
    update_ground();
  }
}

void Robot::update_ground() {
  if (_speed_rot) {
    for (const auto& wheel : _wheels) { wheel.drive(_speed_rot); }
  } else {
    for (const auto& wheel : _wheels) { wheel.drive(_speed_x, _speed_y); }
  }
}

void Robot::update_balancing() {
  const auto up = _orientation_estimator.up();
  // to move the up vector, we need to move the robot in the opposite direction
  const auto speed_x = -_pid_x.compute(up.x());
  const auto speed_y = -_pid_y.compute(up.y());
  // TODO: feed speed through motor PID controller
  drive(speed_x, speed_y);
}

void Robot::set_balancing(bool enabled) { _balancing_mode = enabled; }
void Robot::toggle_balancing() { _balancing_mode = !_balancing_mode; }
bool Robot::is_balancing() const { return _balancing_mode; }
