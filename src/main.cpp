#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"
#include "robot.h"

const uint PWM_FREQUENCY = 1000;
// -- motor 1 --
const uint DIR1 = 4;
const uint PWM1 = 5;
// -- motor 2 --
const uint DIR2 = 7;
const uint PWM2 = 6;
// -- motor 3 --
const uint DIR3 = 12;
const uint PWM3 = 13;

int main() {
  std::array wheels = {
    Omniwheel(0, std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY)),
    Omniwheel(120, std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY)),
    Omniwheel(240, std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY)),
  };

  Robot robot{std::move(wheels)};
  float speed = 0.5;

  while (true) {
    robot.drive(speed, 0);
    sleep_ms(2000);
    robot.drive(0, speed);
    sleep_ms(2000);
    robot.drive(-speed, 0);
    sleep_ms(2000);
    robot.drive(0, -speed);
    sleep_ms(2000);
    robot.rotate(1);
    sleep_ms(1000);
  }

  return 0;
}
