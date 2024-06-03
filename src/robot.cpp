#include "robot.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ranges>

SpeedConfig::SpeedConfig(
  double ball_radius, double ground_wheel_radius, double ground_circle_radius,
  double ball_wheel_radius, double ball_circle_radius, uint32_t update_period_ms
)
  : ground_m_per_rev{static_cast<float>(2 * M_PI * ground_wheel_radius / 1000)}
  , ground_rad_per_rev{static_cast<float>(ground_wheel_radius / ground_circle_radius * 2 * M_PI)}
  , balance_m_per_rev{static_cast<float>(
      -ball_radius / std::sqrt(ball_radius * ball_radius - ball_circle_radius * ball_circle_radius)
      * 2 * M_PI * ball_wheel_radius / 1000
    )}
  , balance_rad_per_rev{static_cast<float>(ball_wheel_radius / ball_circle_radius * 2 * M_PI)}
  , update_period_ms{update_period_ms} {}

Robot::Robot(
  std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator, PidGains pid_gains,
  float max_tilt, const SpeedConfig& speed_config, const LowPassCoefficients& orientation_filter,
  const LowPassCoefficients& balance_speed_filter
)
  : _speed_to_balance_factor{static_cast<float>(std::sin(max_tilt / 180 * M_PI))}
  , _wheels{std::move(wheels)}
  , _orientation_estimator{std::move(orientation_estimator)}
  , _pid_x{-1, 1, pid_gains, orientation_filter}
  , _pid_y{-1, 1, pid_gains, orientation_filter}
  , _balance_speed_filter_x{balance_speed_filter}
  , _balance_speed_filter_y{balance_speed_filter}
  , _speed_config{speed_config} {
  critical_section_init(&_cs);
}

Robot::~Robot() {
  stop_updating();
  critical_section_deinit(&_cs);
}

void Robot::set_speed(float x, float y, float rot, bool is_global) {
  critical_section_enter_blocking(&_cs);
  _target_speed = {x, y};
  _target_rotation = rot;
  _is_speed_global = is_global;
  critical_section_exit(&_cs);
}

void Robot::set_drive_speed(float x, float y, bool is_global) {
  critical_section_enter_blocking(&_cs);
  _target_speed = {x, y};
  _is_speed_global = is_global;
  critical_section_exit(&_cs);
}

void Robot::set_rotate_speed(float speed) {
  critical_section_enter_blocking(&_cs);
  _target_rotation = speed;
  critical_section_exit(&_cs);
}

void Robot::stop() {
  critical_section_enter_blocking(&_cs);
  _target_speed = {0, 0};
  _target_rotation = 0;
  critical_section_exit(&_cs);
}

void Robot::set_angle(float radians) {
  critical_section_enter_blocking(&_cs);
  _current_angle = radians;
  critical_section_exit(&_cs);
}

void Robot::set_position(const Eigen::Vector2f& position) {
  critical_section_enter_blocking(&_cs);
  _current_position = position;
  critical_section_exit(&_cs);
}

void Robot::set_max_distance(float distance) {
  critical_section_enter_blocking(&_cs);
  _squared_max_distance = distance * distance;
  critical_section_exit(&_cs);
}

void Robot::set_max_ground_speed(float speed) {
  critical_section_enter_blocking(&_cs);
  _max_ground_speed = speed;
  critical_section_exit(&_cs);
}

void Robot::set_max_rotation_speed(float speed) {
  critical_section_enter_blocking(&_cs);
  _max_rotation_speed = speed;
  critical_section_exit(&_cs);
}

void Robot::set_max_tilt(float max_tilt) {
  critical_section_enter_blocking(&_cs);
  _speed_to_balance_factor = static_cast<float>(std::sin(max_tilt / 180 * M_PI));
  critical_section_exit(&_cs);
}

void Robot::toggle_balancing() {
  set_balancing(!_balancing_mode);
}

void Robot::set_balancing(bool enabled) {
  if (_balancing_mode == false and enabled == true) reset_balancing();

  critical_section_enter_blocking(&_cs);
  set_balancing(enabled);
  critical_section_exit(&_cs);
}

bool Robot::is_balancing() const {
  return _balancing_mode;
}

void Robot::start_updating() {
  _last_update_us = time_us_32();
  add_repeating_timer_ms(
    _speed_config.update_period_ms,
    [](repeating_timer_t* rt) {
      static_cast<Robot*>(rt->user_data)->update_speed();
      return true;
    },
    this, &_speed_update_timer
  );
  add_repeating_timer_us(
    _orientation_estimator.update_period_us(),
    [](repeating_timer_t* rt) {
      static_cast<Robot*>(rt->user_data)->update();
      return true;
    },
    this, &_balance_update_timer
  );
}

void Robot::stop_updating() { cancel_repeating_timer(&_balance_update_timer); }

void Robot::update() {
  _orientation_estimator.update(true);
  update_pos_and_angle();
  _balancing_mode ? update_balancing() : update_ground();
}

void Robot::update_ground() {
  drive(bounded_local_target_speed() * _max_ground_speed, _target_rotation * _max_rotation_speed);
}

void Robot::update_balancing() {
  const auto global_speed = bounded_global_target_speed();
  // filter in global space to avoid delay when rotating while driving
  const Eigen::Vector2f filtered_target_speed{
    _balance_speed_filter_x.filter(global_speed.x()),
    _balance_speed_filter_y.filter(global_speed.y())
  };
  const auto target = compute_target_vector(global_to_local_speed(filtered_target_speed));

  // the down vector represents how much gravity affects each of the axes
  const auto down = -_orientation_estimator.up();
  Eigen::Vector2f speed{_pid_x.compute(down.x(), target.x()), _pid_y.compute(down.y(), target.y())};

  // because of physics™, the effect of the motor movement on the angle is proportional
  // to cos²(angle) = 1 - sin²(angle) = 1 - down²
  auto x = down.array().square();
  const Eigen::Vector2f efficiency = 1 - down.head<2>().array().square();
  drive(speed.array() / efficiency.array(), _target_rotation * _max_rotation_speed);
}

void Robot::reset_balancing() {
  _pid_x.reset();
  _pid_y.reset();
  _balance_speed_filter_x.reset();
  _balance_speed_filter_y.reset();
}

void Robot::drive(Eigen::Vector2f speed, float rotation) {
  const auto abs_speed = speed.norm();
  if (abs_speed > 1) {
    rotation = 0;
    speed /= abs_speed;
  } else {
    const auto speed_reserve = 1 - abs_speed;
    rotation = speed_reserve > 0 ? std::clamp(rotation, -speed_reserve, speed_reserve) : 0;
  }

  _is_translating = not speed.isZero();
  _is_rotating = rotation != 0;
  for (const auto& wheel : _wheels) wheel.drive(speed, rotation);
}

void Robot::update_speed() {
  std::array<float, 3> wheel_speeds;
  for (size_t i = 0; i < _wheels.size(); ++i) { wheel_speeds[i] = _wheels[i].compute_speed(); }

  // the average wheel speed turns out to be half the total speed, regardless of additional rotation
  Eigen::Vector2f avg_speed = {0, 0};
  for (size_t i = 0; i < _wheels.size(); ++i) {
    avg_speed += wheel_speeds[i] * _wheels[i].wheel_direction();
  }
  avg_speed /= wheel_speeds.size();
  _measured_speed = avg_speed * 2;

  // rotation speed is the additional speed wheels have compared to just linear motion
  float avg_rotation = 0;
  for (size_t i = 0; i < _wheels.size(); ++i) {
    avg_rotation += wheel_speeds[i] - _wheels[i].get_local_speed(avg_speed);
  }
  avg_rotation /= wheel_speeds.size();
  _measured_rotation = avg_rotation;
}

void Robot::update_pos_and_angle() {
  const auto now = time_us_32();
  const auto dt = now - _last_update_us;
  _last_update_us = now;

  // only update position and angle if there is corresponding input to avoid noise
  if (_is_translating) {
    const auto rev_to_m =
      _balancing_mode ? _speed_config.balance_m_per_rev : _speed_config.ground_m_per_rev;
    _current_position += local_to_global_speed(_measured_speed) * rev_to_m * (dt / 1e6f);
  }

  if (_is_rotating) {
    const auto rev_to_rad =
      _balancing_mode ? _speed_config.balance_rad_per_rev : _speed_config.ground_rad_per_rev;
    _current_angle += _measured_rotation * rev_to_rad * (dt / 1e6f);
  }
}

Eigen::Vector2f Robot::compute_target_vector(Eigen::Vector2f target_speed) const {
  auto speed_norm = target_speed.norm();
  if (speed_norm > 1) target_speed /= speed_norm;
  return target_speed * _speed_to_balance_factor;
}

Eigen::Vector2f Robot::global_to_local_speed(Eigen::Vector2f global_speed) const {
  return Eigen::Rotation2Df{-_current_angle} * global_speed;
}

Eigen::Vector2f Robot::local_to_global_speed(Eigen::Vector2f local_speed) const {
  return Eigen::Rotation2Df{_current_angle} * local_speed;
}

Eigen::Vector2f Robot::bounded_local_target_speed() const {
  return global_to_local_speed(bounded_global_target_speed());
}

Eigen::Vector2f Robot::bounded_global_target_speed() const {
  const auto global_speed = _is_speed_global ? _target_speed : local_to_global_speed(_target_speed);
  if (std::isinf(_squared_max_distance)) return global_speed;
  if (_current_position.squaredNorm() < _squared_max_distance) return global_speed;

  const auto outward_direction = _current_position.normalized();
  const auto outward_speed = outward_direction.dot(global_speed);
  if (outward_speed <= 0) return global_speed;

  return global_speed - outward_speed * outward_direction;
}
