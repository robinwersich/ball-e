#include "robot.h"

#include <cmath>

Robot::Robot(
  std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator,
  PidGains balance_gains
)
  : _wheels{std::move(wheels)}
  , _orientation_estimator{std::move(orientation_estimator)}
  , _pid_x{-1, 1, balance_gains}
  , _pid_y{-1, 1, balance_gains} {
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
      reinterpret_cast<Robot*>(rt->user_data)->update();
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
  const auto up = _orientation_estimator.up();
  const auto gravity_influence = get_gravity_influence(-up);
  const auto speed_x = _pid_x.compute(gravity_influence.x());
  const auto speed_y = _pid_y.compute(gravity_influence.y());
  // TODO: feed speed through motor PID controller
  drive(speed_x, speed_y, _speed_rot);
}

void Robot::set_balancing(bool enabled) {
  if (_balancing_mode == false and enabled == true) {
    _pid_x.reset();
    _pid_y.reset();
  }
  _balancing_mode = enabled;
}

void Robot::toggle_balancing() { _balancing_mode = !_balancing_mode; }
bool Robot::is_balancing() const { return _balancing_mode; }

void Robot::drive(float x, float y, float rot) {
  for (const auto& wheel : _wheels) { wheel.drive(x, y, rot); }
}

Eigen::Vector2f Robot::get_gravity_influence(const Eigen::Vector3f& down) {
  // gravity influence on each axis is proportional to 1-abs(z) and cos/sin(theta) of the
  // angle of the down vector in the xy plane

  // scale x and y so that x² + y² = 1 to get the maximum gravity influence
  const auto xy_scale = 1 / std::sqrt(1 - down.z() * down.z());
  const auto max_gravity = Eigen::Vector2f{down.x(), down.y()} * xy_scale;

  return max_gravity * (1 - std::abs(down.z()));
}
