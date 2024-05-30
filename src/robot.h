#pragma once

#include <Eigen/Core>

#include "omniwheel.h"
#include "orientation_estimator.h"
#include "pico/critical_section.h"
#include "pico/time.h"
#include "pid.h"

class Robot {
 public:
  /**
   * Creates a new robot instance.
   * @param wheels The three omniwheels of the robot.
   * @param orientation_estimator The orientation estimator to use for balancing.
   * @param pid_gains The PID gains to use for balancing.
   *  Units are S/g, S/(g·s), and S/(g/s) for kp, ki, and kd respectively
   *  where S is the speed fraction of the robot and g is the gravity influence on each axis
   * @param encoder_gain The gain with which to add the current speed (rps) to the balance output.
   * @param balance_filter An optional low-pass filter to apply to the orientation signal.
   * @note The robot will not start moving until `start_updating` is called.
   */
  Robot(
    std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator, PidGains pid_gains,
    float encoder_gain, LowPassCoefficients balance_filter = {}
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
  /** Returns the encoder gain for the balancing mode */
  float& encoder_gain() { return _encoder_gain; }

 private:
  std::array<Omniwheel, 3> _wheels;
  OrientationEstimator _orientation_estimator;
  PidController _pid_x, _pid_y;
  float _encoder_gain;
  Eigen::Vector2f _target_speed = {0, 0};
  float _target_rotation = 0;
  Eigen::Vector2f _measured_speed = {0, 0};
  float _measured_rotation = 0;
  bool _balancing_mode = false;
  repeating_timer_t _update_timer;
  critical_section_t _cs;

  /** Updates the robot's orientation and movement (based on the current movement goal) */
  void update();
  /** Updates the robots movement based on the desired speed */
  void update_ground();
  /** Updates the robots movement based goal of balancing and achieving the desired speed */
  void update_balancing();

  /**
   * Directly makes the robot move in the given direction, speed and rotation.
   * This method ensures the final wheel speed is within the limits for all wheels.
   * If that is not the case, the rotation speed will be compromised first.
   * Then the speed will be scaled equally for all wheels so that direction is preserved.
   * @param speed The direction and speed of movement.
   * @param rotation The speed of rotation.
   */
  void drive(Eigen::Vector2f speed, float rotation);

  /** Updates measured speed and measured rotation speed based on encoder readings. */
  void update_speed();
};