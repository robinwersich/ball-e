#include "robot.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ranges>

Robot::Robot(
  std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator, PidGains pid_gains,
  float max_rotation, float auto_calibration_speed, LowPassCoefficients orientation_filter,
  LowPassCoefficients balance_speed_filter
)
  : _max_angle_sin{static_cast<float>(std::sin(max_rotation / 180 * M_PI))}
  , _wheels{std::move(wheels)}
  , _orientation_estimator{std::move(orientation_estimator)}
  , _auto_calibration_speed{auto_calibration_speed}
  , _pid_x{-1, 1, pid_gains, orientation_filter}
  , _pid_y{-1, 1, pid_gains, orientation_filter}
  , _balance_speed_filter{balance_speed_filter} {
  critical_section_init(&_cs);
}

Robot::~Robot() {
  stop_updating();
  critical_section_deinit(&_cs);
}

void Robot::set_speed(float x, float y, float rot) {
  critical_section_enter_blocking(&_cs);
  _target_speed = {x, y};
  _target_rotation = rot;
  critical_section_exit(&_cs);
}

void Robot::set_drive_speed(float x, float y) {
  critical_section_enter_blocking(&_cs);
  _target_speed = {x, y};
  critical_section_exit(&_cs);
}

void Robot::set_rotate_speed(float speed) {
  critical_section_enter_blocking(&_cs);
  _target_rotation = speed;
  critical_section_exit(&_cs);
}

void Robot::stop() {
  _target_speed = {0, 0};
  _target_rotation = 0;
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
  update_speed();
  _balancing_mode ? update_balancing() : update_ground();
}

void Robot::update_ground() { drive(_target_speed, _target_rotation); }

void Robot::update_balancing() {
  // the down vector represents how much gravity affects each of the axes
  const auto down = -_orientation_estimator.up();

  // only auto calibrate if robot is currently not trying to move
  if (_target_speed == Eigen::Vector2f{0, 0}) {
    const auto offset_rotation = Eigen::Quaternionf::FromTwoVectors(down, _local_down);
    // nudge the local down vector in the direction opposite vector of the current down vector
    _local_down = _local_down * (1 - _auto_calibration_speed)
                + offset_rotation * _local_down * _auto_calibration_speed;
  }

  const auto target = compute_target_vector(_balance_speed_filter.filter(_target_speed));
  Eigen::Vector2f speed{_pid_x.compute(down.x(), target.x()), _pid_y.compute(down.y(), target.y())};

  // because of physics™, the effect of the motor movement on the angle is proportional
  // to cos²(angle) = 1 - sin²(angle) = 1 - down²
  auto x = down.array().square();
  const Eigen::Vector2f efficiency = 1 - down.head<2>().array().square();
  drive(speed.array() / efficiency.array(), _target_rotation);
}

void Robot::set_balancing(bool enabled) {
  if (_balancing_mode == false and enabled == true) reset_balancing();
  _balancing_mode = enabled;
}

void Robot::reset_balancing() {
  _pid_x.reset();
  _pid_y.reset();
  _balance_speed_filter.reset();
  _local_down = {0, 0, -1};
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

  for (const auto& wheel : _wheels) wheel.drive(speed, rotation);
}

void Robot::update_speed() {
  std::array<Eigen::Vector2f, 3> wheel_speeds;
  for (size_t i = 0; i < _wheels.size(); ++i) wheel_speeds[i] = _wheels[i].compute_speed();

  // the average wheel speed turns out to be half the total speed, regardless of additional rotation
  Eigen::Vector2f avg_speed = {0, 0};
  for (const auto& speed : wheel_speeds) avg_speed += speed;
  avg_speed /= wheel_speeds.size();
  _measured_speed = avg_speed * 2;

  // rotation speed is the additional speed wheels have compared to just linear motion
  float avg_rotation = 0;
  for (size_t i = 0; i < _wheels.size(); ++i) {
    avg_rotation += wheel_speeds[i].norm() - _wheels[i].get_local_speed(avg_speed);
  }
  avg_rotation /= wheel_speeds.size();
  _measured_rotation = avg_rotation;
}

Eigen::Vector3f Robot::compute_target_vector(Eigen::Vector2f target_speed) const {
  const Eigen::Vector3f global_down{0, 0, -1};
  // we want to apply speed based tilt in global space, but then need to convert to local space
  const auto global_to_local = Eigen::Quaternionf::FromTwoVectors(global_down, _local_down);

  const auto speed_norm = target_speed.norm();
  if (speed_norm > 1) target_speed /= speed_norm;
  const auto target_2d = target_speed * _max_angle_sin;
  const Eigen::Vector3f target_3d{
    target_2d.x(), target_2d.y(), -std::sqrt(1 - target_2d.squaredNorm())
  };
  return (global_to_local * target_3d);
}
