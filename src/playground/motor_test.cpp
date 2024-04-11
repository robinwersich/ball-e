#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"

const uint DIR1 = 4;
const uint PWM1 = 5;

int main() {
  stdio_init_all();

  auto motor = MotorDriverDRI0044(PWM1, DIR1, 1000);

  while (true) {
    sleep_ms(2000);
    motor.drive(0.2);
    sleep_ms(2000);
    motor.drive(0.5);
    sleep_ms(2000);
    motor.drive(1.0);
    sleep_ms(2000);
    motor.stop();
    sleep_ms(2000);
    motor.drive(-0.5);
  }

  return 0;
}
