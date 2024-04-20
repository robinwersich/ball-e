#pragma once

#include "omniwheel.h"

class Robot {
 public:
  Robot(std::array<Omniwheel, 3> wheels);

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

 private:
  std::array<Omniwheel, 3> _wheels;
};