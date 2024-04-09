#include "robot.h"

Robot::Robot(std::array<Omniwheel, 3> wheels) : wheels{std::move(wheels)} {}

void Robot::drive(float x, float y) {
  for (const auto& wheel : wheels) {
    wheel.drive(x, y);
  }
}

void Robot::rotate(float speed) {
  for (const auto& wheel : wheels) {
    wheel.drive(speed);
  }
}
