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

uint16_t RobotController::get_pressed_buttons(uint16_t current_buttons) {
  const uint16_t pressed_buttons = ~_last_buttons & current_buttons;
  _last_buttons = current_buttons;
  return pressed_buttons;
}

uint16_t RobotController::get_released_buttons(uint16_t current_buttons) {
  const uint16_t released_buttons = _last_buttons & ~current_buttons;
  _last_buttons = current_buttons;
  return released_buttons;
}

uint8_t RobotController::get_pressed_dpad(uint8_t current_dpad) {
  const uint8_t pressed_dpad = ~_last_dpad & current_dpad;
  _last_dpad = current_dpad;
  return pressed_dpad;
}

uint8_t RobotController::get_released_dpad(uint8_t current_dpad) {
  const uint8_t released_dpad = _last_dpad & ~current_dpad;
  _last_dpad = current_dpad;
  return released_dpad;
}

void RobotController::on_gamepad_data(const uni_gamepad_t& gamepad) {
  const auto pressed_buttons = get_pressed_buttons(gamepad.buttons);
  const auto released_buttons = get_released_buttons(gamepad.buttons);
  const auto pressed_dpad = get_pressed_dpad(gamepad.dpad);
  const auto left_shoulder = gamepad.brake / MAX_TRIGGER_VALUE;
  const auto right_shoulder = gamepad.throttle / MAX_TRIGGER_VALUE;
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
  if (gamepad.buttons & BUTTON_SHOULDER_L and right_stick.norm() > 0.5) {
    _forward_orientation = right_stick;
  }
  // activate angle calibration
  if (released_buttons & BUTTON_SHOULDER_L) {
    if (!_forward_orientation.isZero()) {
      _robot->angle() = angle_between({0, 1}, _forward_orientation);
    }
    _forward_orientation = {0, 0};
  }
  // reset position
  if (pressed_buttons & BUTTON_SHOULDER_R) _robot->position() = {0, 0};

  if (left_shoulder or right_shoulder) {
    // car mode
    _robot->set_speed(0, right_shoulder - left_shoulder, -left_stick.x(), false);
  } else {
    // omnidirectional mode
    _robot->set_speed(right_stick.x(), right_stick.y(), -left_stick.x(), _global_mode);
  }
}

float angle_between(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  return std::atan2(a.x() * b.y() - a.y() * b.x(), a.x() * b.x() + a.y() * b.y());
}
