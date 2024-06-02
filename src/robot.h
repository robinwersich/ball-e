#pragma once

#include <Eigen/Core>

#include "omniwheel.h"
#include "orientation_estimator.h"
#include "pico/critical_section.h"
#include "pico/time.h"
#include "pid.h"

struct SpeedConfig {
  /** How many meters one wheel rotation corresponds to on the ground. */
  float ground_m_per_rev;
  /** How many meters one wheel rotation corresponds to on the ball. */
  float balance_m_per_rev;
  /** How many radians of rotation one wheel rotation corresponds to on the ground. */
  float ground_rad_per_rev;
  /** How many radians of rotation one wheel rotation corresponds to on the ball. */
  float balance_rad_per_rev;
  /** How long the distance between speed measurement updates should be. */
  uint32_t update_period_ms;

  /**
   * Creates a speed configuration from various measurements (all in mm).
   * @param ball_radius The radius of the ball.
   * @param ground_wheel_radius The distance between the wheel axis and ground contact point.
   * @param gound_circle_radius The distance between the wheel contact point and robot center.
   * @param ball_wheel_radius The distance between the wheel axis and ball contact point.
   * @param ball_circle_radius The distance between the ball contact point and robot center.
   */
  SpeedConfig(
    double ball_radius, double ground_wheel_radius, double ground_circle_radius,
    double ball_wheel_radius, double ball_circle_radius, uint32_t update_period_ms
  );
};

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
   * @param max_rotation The maximum rotation the robot can still recover from in degrees.
   * @param speed_config The configuration for converting wheel rotations to meters and radians.
   * @param orientation_filter An optional low-pass filter to apply to the orientation derivative.
   * @param balance_speed_filter An optional low-pass filter to apply to the speed when balancing.
   * @note The robot will not start moving until `start_updating` is called.
   */
  Robot(
    std::array<Omniwheel, 3> wheels, OrientationEstimator orientation_estimator, PidGains pid_gains,
    float max_rotation, const SpeedConfig& speed_config,
    const LowPassCoefficients& orientation_filter = {},
    const LowPassCoefficients& balance_speed_filter = {}
  );
  ~Robot();

  /**
   * Make the robot drive in the direction and with the speed given by the movement vector.
   * The length of the movement vector should be between 0 and 1.
   * @param x The rightward component of the movement vector.
   * @param y The forward component of the movement vector.
   * @param is_global If the speed is given in global coordinates.
   */
  void set_drive_speed(float x, float y, bool is_global = false);

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
   * @param is_global If the speed is given in global coordinates.
   */
  void set_speed(float x, float y, float rot, bool is_global = false);

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

  /** Returns the current position estimate. */
  Eigen::Vector2f& position() { return _current_position; }
  /** Returns the current angle estimate. */
  float& angle() { return _current_angle; }

  /** Returns the PID controller for the x-axis  */
  PidController& pid_x() { return _pid_x; }
  /** Returns the PID controller for the y-axis  */
  PidController& pid_y() { return _pid_y; }
  /** Returns the encoder gain for the balancing mode */
  float& encoder_gain() { return _encoder_gain; }

 private:
  float _speed_to_balance_factor;  // factor for converting speed vector to balance target vector
  std::array<Omniwheel, 3> _wheels;
  OrientationEstimator _orientation_estimator;
  PidController _pid_x, _pid_y;
  float _encoder_gain;
  LowPassFilter _balance_speed_filter_x;
  LowPassFilter _balance_speed_filter_y;
  bool _is_speed_global = false;
  Eigen::Vector2f _target_speed = {0, 0};      // fractional speed, local or global
  float _target_rotation = 0;                  // fractional speed
  bool _is_translating = false;                // whether there currently is a translation signal
  bool _is_rotating = false;                   // whether there currently is a rotation signal
  Eigen::Vector2f _measured_speed = {0, 0};    // rps, local
  float _measured_rotation = 0;                // rps
  float _current_angle = 0;                    // radians
  Eigen::Vector2f _current_position = {0, 0};  // meters, global
  SpeedConfig _speed_config;
  bool _balancing_mode = false;
  repeating_timer_t _balance_update_timer;
  repeating_timer_t _speed_update_timer;
  critical_section_t _cs;
  uint32_t _last_update_us = 0;

  /** Resets values that are updated while balancing */
  void reset_balancing();

  /** Updates the robot's orientation and movement (based on the current movement goal) */
  void update();
  /** Updates the robots movement based on the desired speed */
  void update_ground();
  /** Updates the robots movement based goal of balancing and achieving the desired speed */
  void update_balancing();

  /** Computes the target vector to feed to the PID controller based on a target speed. */
  Eigen::Vector2f compute_target_vector(Eigen::Vector2f target_speed) const;

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
  /** Updates the current position and angle estimate. */
  void update_pos_and_angle();

  Eigen::Vector2f global_to_local_speed(Eigen::Vector2f global_speed) const;
  Eigen::Vector2f local_to_global_speed(Eigen::Vector2f local_speed) const;
  Eigen::Vector2f ensure_local(Eigen::Vector2f speed) const;
};