#include "robot_control.h"

#include <limits>

RobotController::RobotController(Robot* robot) : _robot{robot} {}

void RobotController::initialize_controller(RobotController* controller) {
  controller->initialize_limits();
  btcontrol::init();
  btcontrol::register_gampad_behavior([controller](const uni_gamepad_t& gamepad) {
    controller->on_gamepad_data(gamepad);
  });
}

void RobotController::initialize_limits() {
  _robot->set_max_ground_speed(_speed_limit_range.current());
  _robot->set_max_tilt(_tilt_limit_range.current());
  _robot->set_max_rotation_speed(_rotation_limit_range.current());
}

void RobotController::run_loop() { btcontrol::run_loop(); }

void RobotController::on_gamepad_data(const uni_gamepad_t& gamepad) {
  const auto pressed_buttons = ~_last_buttons & gamepad.buttons;
  const auto released_buttons = _last_buttons & ~gamepad.buttons;
  _last_buttons = gamepad.buttons;
  const auto pressed_dpad = ~_last_dpad & gamepad.dpad;
  _last_dpad = gamepad.dpad;
  const auto left_trigger = gamepad.brake / MAX_TRIGGER_VALUE;
  const auto right_trigger = gamepad.throttle / MAX_TRIGGER_VALUE;
  const Eigen::Vector2f left_stick = {
    gamepad.axis_x / MAX_STICK_VALUE, gamepad.axis_y / MAX_STICK_VALUE
  };
  const Eigen::Vector2f right_stick = {
    gamepad.axis_rx / MAX_STICK_VALUE, gamepad.axis_ry / MAX_STICK_VALUE
  };

  // switch between local and global mode for the right control stick
  if (pressed_buttons & BUTTON_A) _global_mode = !_global_mode;
  // toggle balancing
  if (pressed_buttons & BUTTON_B) _robot->toggle_balancing();
  // toggle constraint mode
  if (pressed_buttons & BUTTON_X) {
    _robot->set_max_distance(
      _constraint_mode ? MAX_DISTANCE : std::numeric_limits<float>::infinity()
    );
    _constraint_mode = !_constraint_mode;
  }
  // increase speed limit
  if (pressed_dpad & DPAD_DOWN) {
    if (_robot->is_balancing()) _robot->set_max_tilt(_tilt_limit_range.decrease());
    else _robot->set_max_ground_speed(_speed_limit_range.decrease());
  }
  // decrease speed limit
  if (pressed_dpad & DPAD_UP) {
    if (_robot->is_balancing()) _robot->set_max_tilt(_tilt_limit_range.increase());
    else _robot->set_max_ground_speed(_speed_limit_range.increase());
  }
  // increase rotation limit
  if (pressed_dpad & DPAD_LEFT) _robot->set_max_rotation_speed(_rotation_limit_range.decrease());
  // decrease rotation limit
  if (pressed_dpad & DPAD_RIGHT) _robot->set_max_rotation_speed(_rotation_limit_range.increase());
  // record angle calibration
  if (gamepad.buttons & BUTTON_SHOULDER_R and left_stick.norm() > 0.5) {
    _forward_orientation = left_stick;
  }
  // activate angle calibration
  if (released_buttons & BUTTON_SHOULDER_R) {
    if (!_forward_orientation.isZero()) {
      _robot->set_angle(angle_between({0, 1}, _forward_orientation));
    }
    _forward_orientation = {0, 0};
  }
  // reset position
  if (pressed_buttons & BUTTON_SHOULDER_L) _robot->set_position({0, 0});

  if (left_trigger or right_trigger) {
    // car mode
    _robot->set_speed(0, right_trigger - left_trigger, -right_stick.x(), false);
  } else if (gamepad.buttons & BUTTON_SHOULDER_R) {
    // calibration mode
    _robot->stop();
  } else {
    // omnidirectional mode
    _robot->set_speed(left_stick.x(), left_stick.y(), -right_stick.x(), _global_mode);
  }
}

float angle_between(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  return std::atan2(a.x() * b.y() - a.y() * b.x(), a.x() * b.x() + a.y() * b.y());
}
