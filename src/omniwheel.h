#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <utility>

#include "motor_driver.h"

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
   * @param swap_direction Whether to swap the direction of the main axis.
   */
  Omniwheel(float angle, std::unique_ptr<MotorDriver> motor_driver, bool swap_direction = false);

  /**
   * Make the omniwheel drive in the direction and with the speed given by the movement vector.
   * The length of the movement vector should be between 0 and 1.
   * @param x The rightward component of the movement vector.
   * @param y The forward component of the movement vector.
   */
  void drive(float x, float y) const;

  /**
   * Make the omniwheel drive along the main axis.
   * @param speed The speed at which to drive, between -1 and 1.
   */
  void drive(float speed) const;

  /** Stops the omniwheel. */
  void stop() const;

 private:
  /** The conversion from the global coordinate system to the wheel coordinate system */
  Eigen::Rotation2Df _wheel_rotation;
  std::unique_ptr<MotorDriver> _motor_driver;
  bool _swap_direction;
};
