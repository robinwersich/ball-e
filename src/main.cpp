#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"
#include "robot.h"

const uint PWM_FREQUENCY = 25000;
// -- motor 1 --
const uint DIR1 = 0;
const uint PWM1 = 1;
const uint ENC1A = 2;
const uint ENC1B = 3;
// -- motor 2 --
const uint ENC2B = 4;
const uint ENC2A = 5;
const uint DIR2 = 6;
const uint PWM2 = 7;
// -- motor 3 --
const uint PWM3 = 8;
const uint DIR3 = 9;
const uint ENC3A = 10;
const uint ENC3B = 11;

int main() {
  std::array wheels = {
    Omniwheel(30, std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY), true),
    Omniwheel(150, std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY)),
    Omniwheel(270, std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY)),
  };

  Robot robot{std::move(wheels)};

  while (true) {
    sleep_ms(2000);
    robot.drive(0, 0.7);
    sleep_ms(1000);
    robot.stop();
    sleep_ms(500);
    robot.drive(0.7, 0);
    sleep_ms(1000);
    robot.stop();
    sleep_ms(500);
    robot.drive(-0.7, -0.7);
    sleep_ms(1000);
    robot.stop();
    sleep_ms(500);
    robot.rotate(0.5);
    sleep_ms(1000);
    robot.rotate(-0.5);
    sleep_ms(1000);
    robot.stop();
  }

  return 0;
}
