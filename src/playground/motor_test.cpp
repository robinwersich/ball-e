#include "motor_drivers/l298n.h"
#include "pico/stdlib.h"

// GPIO pin numbers
const uint ENA_PIN = 2;
const uint IN1_PIN = 3;
const uint IN2_PIN = 4;

int main() {
  stdio_init_all();

  auto slice = PwmSlice::forPin(ENA_PIN);
  auto motor = MotorDriverL298N(ENA_PIN, IN1_PIN, IN2_PIN, slice);

  while (true) {
    sleep_ms(2000);
    motor.drive(100);
    sleep_ms(2000);
    motor.stop();
  }

  return 0;
}
