#pragma once

#include "omniwheel.h"
#include "orientation_estimator.h"
#include "pid.h"

class Robot {
 public:
  Robot(
    std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator,
    PidGains balance_gains
  );

  /**
   * Make the robot drive in the direction and with the speed given by the movement vector.
   * The length of the movement vector should be between 0 and 1.
   * @param x The rightward component of the movement vector.
   * @param y The forward component of the movement vector.
   */
  void drive(float x, float y);

  /**
   * Make the robot rotate around its center.
   * @param speed The speed at which to rotate, between -1 (clockwise) and 1 (anti-clockwise).
   */
  void rotate(float speed);

  /** Stops the robot. */
  void stop();

  /** Enables or disables the ball balancing mode. */
  void set_balancing(bool enabled);
  /** Toggles the ball balancing mode. */
  void toggle_balancing();
  /** Returns true if the robot is currently balancing. */
  bool is_balancing() const;

  /** Runs the robot control loop. This will not return. */
  void run_control_loop();

 private:
  std::array<Omniwheel, 3> _wheels;
  OrientationEstimator _orientation_estimator;
  PidController _pid_x, _pid_y;
  float _speed_x = 0, _speed_y = 0;
  float _speed_rot = 0;
  bool _balancing_mode = false;

  void update_ground();
  void update_balancing();
};