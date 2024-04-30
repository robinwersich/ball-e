#include "btcontrol.h"
#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"
#include "robot.h"

const uint PWM_FREQUENCY = 25000;
// -- motor 1 --
const uint DIR1 = 0;
const uint PWM1 = 1;
const uint ENC1_SLOT = 1;
// -- motor 2 --
const uint DIR2 = 6;
const uint PWM2 = 7;
const uint ENC2_SLOT = 2;
// -- motor 3 --
const uint PWM3 = 8;
const uint DIR3 = 9;
const uint ENC3_SLOT = 5;

std::unique_ptr<Robot> robot;

void on_gamepad_data(const uni_gamepad_t& gamepad) {
  if (gamepad.throttle) {
    robot->rotate(-gamepad.throttle / 1024.0);
  } else if (gamepad.brake) {
    robot->rotate(gamepad.brake / 1024.0);
  } else {
    const float speed_x = gamepad.axis_x / 512.0;
    const float speed_y = gamepad.axis_y / 512.0;
    robot->drive(speed_x, speed_y);
  }
}

int main() {
  stdio_init_all();

  robot = std::make_unique<Robot>(std::array<Omniwheel, 3>{
    Omniwheel(30, std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY, true)),
    Omniwheel(150, std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY)),
    Omniwheel(270, std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY)),
  });

  btcontrol::init();
  btcontrol::register_gampad_behavior(on_gamepad_data);
  btcontrol::run_loop();

  return 0;
}
