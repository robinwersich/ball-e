#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <utility>

#include "motor_driver.h"
#include "motor_state.h"

/**
 * An omniwheel can move in any direction. It is a wheel with rollers around its circumference.
 * Thus, it has a (powered) main axis and a (non-powered) perpendicular axis of movement.
 */
class Omniwheel {
 public:
  /**
   * Creates a new omniwheel controller.
   * @param angle The angle between the main axis of the omniwheel and what is considered forward,
   *  in anti-clockwise degrees.
   * @param motor_driver The motor driver for the main axis of the omniwheel.
   * @param motor_state The state of the motor driver.
   */
  Omniwheel(
    float angle, std::unique_ptr<MotorDriver> motor_driver, const MotorState& motor_state
  );

  /**
   * Make the omniwheel drive in the direction and with the speed given by the movement vector.
   * The length of the movement vector must be between 0 and 1.
   * @param speed A vector with length < 1 giving the direction and speed of movement.
   */
  void drive(const Eigen::Vector2f& speed) const;

  /**
   * Make the omniwheel drive along the main axis.
   * @param speed The speed at which to drive, between -1 and 1.
   */
  void drive(float speed) const;

  /**
   * Make the omniwheel drive in the direction and with the speed given by the movement vector,
   * and additionally by some speed along the main axis.
   * The sum of the length of the movement vector and the absolute value of the forward speed
   * must be between 0 and 1.
   * @param global_speed A vector with length < 1 giving the direction and speed of movement.
   * @param local_speed The additional speed along the main axis.
   */
  void drive(const Eigen::Vector2f& global_speed, float local_speed) const;

  /** Stops the omniwheel. */
  void stop() const;

  /** Computes the current speed vector in rev/s */
  float compute_speed();

  /** Returns the direction of the main axis. */
  const Eigen::Vector2f& wheel_direction() const;

  /** Computes the required speed along the main axis given a global speed vector. */
  float get_local_speed(const Eigen::Vector2f global_speed) const;

 private:
  Eigen::Vector2f _wheel_direction;  // unit vector pointing in the direction of the main axis
  std::unique_ptr<MotorDriver> _motor_driver;
  MotorState _motor_state;
};
