#pragma once

#include "omniwheel.h"
#include "orientation_estimator.h"
#include "pico/time.h"
#include "pico/critical_section.h"
#include "pid.h"

class Robot {
 public:
  /**
   * Creates a new robot instance.
   * @param wheels The three omniwheels of the robot.
   * @param orientation_estimator The orientation estimator to use for balancing.
   * @param balance_gains The PID gains to use for balancing.
   *  Units are S/g, S/(g·s), and S/(g/s) for kp, ki, and kd respectively
   *  where S is the speed fraction of the robot and g is the gravity influence on each axis
   * @param balance_filter An optional low-pass filter to apply to the orientation signal.
   * @note The robot will not start moving until `start_updating` is called.
   */
  Robot(
    std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator,
    PidGains balance_gains, std::optional<LowPassFilter> balance_filter = {}
  );
  ~Robot();

  /**
   * Make the robot drive in the direction and with the speed given by the movement vector.
   * The length of the movement vector should be between 0 and 1.
   * @param x The rightward component of the movement vector.
   * @param y The forward component of the movement vector.
   */
  void set_drive_speed(float x, float y);

  /**
   * Make the robot rotate around its center.
   * @param speed The speed at which to rotate, between -1 (clockwise) and 1 (anti-clockwise).
   */
  void set_rotate_speed(float speed);

  /**
   * Sets linear drive and rotational speed at once.
   * @param x The rightward component of the movement vector.
   * @param y The forward component of the movement vector.
   * @param speed The speed at which to rotate, between -1 (clockwise) and 1 (anti-clockwise).
   */
  void set_speed(float x, float y, float rot);

  /** Stops the robot. */
  void stop();

  /** Enables or disables the ball balancing mode. */
  void set_balancing(bool enabled);
  /** Toggles the ball balancing mode. */
  void toggle_balancing() { _balancing_mode = !_balancing_mode; }
  /** Returns true if the robot is currently balancing. */
  bool is_balancing() const { return _balancing_mode; }

  /** Sets up a timer interrupt for handling the robot movement. */
  void start_updating();
  /** Stops the timer interrupt for handling the robot movement. */
  void stop_updating();

  /** Returns the PID controller for the x-axis  */
  PidController& pid_x() { return _pid_x; }
  /** Returns the PID controller for the y-axis  */
  PidController& pid_y() { return _pid_y; }

 private:
  std::array<Omniwheel, 3> _wheels;
  OrientationEstimator _orientation_estimator;
  PidController _pid_x, _pid_y;
  float _speed_x = 0, _speed_y = 0;
  float _speed_rot = 0;
  bool _balancing_mode = false;
  repeating_timer_t _update_timer;
  critical_section_t _cs;

  /** Updates the robot's orientation and movement (based on the current movement goal) */
  void update();
  /** Updates the robots movement based on the desired speed */
  void update_ground();
  /** Updates the robots movement based goal of balancing and achieving the desired speed */
  void update_balancing();

  /** Directly makes the robot move in the given direction, speed and rotation. */
  void drive(float x, float y, float rot);
};