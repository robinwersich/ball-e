#pragma once

#include "btcontrol.h"
#include "robot.h"

/** Definition of discrete possible values for a parameter. */
struct ValueRange {
  int min;
  int max;
  int current_value;
  float factor;

  float current() const { return current_value * factor; }
  float increase() {
    current_value = std::min(current_value + 1, max);
    return current();
  }
  float decrease() {
    current_value = std::max(current_value - 1, min);
    return current();
  }
};

class RobotController {
  /** Max distance the robot is allowed to move from the center in constraint mode. */
  const float MAX_DISTANCE = 0.75;
  const float MAX_TRIGGER_VALUE = 1024;
  const float MAX_STICK_VALUE = 512;

 public:
  RobotController(Robot* robot);

  /** Initializes the bluetooth module and sets the given controller as the active one. */
  static void initialize_controller(RobotController* controller);

  /** Starts the control loop. Will not return. */
  void run_loop();

 private:
  Robot* _robot;
  uint16_t _last_buttons = 0x0000;
  uint8_t _last_dpad = 0x00;
  bool _global_mode = true;
  bool _constraint_mode = false;
  ValueRange _speed_limit_range = {1, 10, 8, 0.1};
  ValueRange _tilt_limit_range = {20, 30, 25, 0.1};
  ValueRange _rotation_limit_range = {1, 10, 5, 0.1};
  Eigen::Vector2f _forward_orientation = {0, 0};  // only used while calibrating orientation

  /** Sets the speed, tilt and rotation limits of the robot to the current values. */
  void initialize_limits();

  /** Handles incoming gamepad data. */
  void on_gamepad_data(const uni_gamepad_t& gamepad);
};

/** Returns the angle to get from vector a to vector b in radians. */
float angle_between(const Eigen::Vector2f& a, const Eigen::Vector2f& b);
