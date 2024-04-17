#include "robot.h"

Robot::Robot(std::array<Omniwheel, 3> wheels) : _wheels{std::move(wheels)} {}

void Robot::drive(float x, float y) {
  for (const auto& wheel : _wheels) { wheel.drive(x, y); }
}

void Robot::rotate(float speed) {
  for (const auto& wheel : _wheels) { wheel.drive(speed); }
}

void Robot::stop() {
  for (const auto& wheel : _wheels) { wheel.stop(); }
}
