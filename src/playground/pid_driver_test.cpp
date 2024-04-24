#include "motor_driver_pid.h"
#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"
#include "parameters.h"

const uint PWM_FREQUENCY = 25000;
// -- motor 1 --
const uint DIR = 0;
const uint PWM = 1;
const uint ENCA = 2;
const uint ENCB = 3;

int main() {
  stdio_init_all();

  auto raw_driver = std::make_shared<MotorDriverDRI0044>(PWM, DIR, PWM_FREQUENCY);
  auto decoder = std::make_shared<MotorDecoder>(ENCA, ENCB);
  const MotorSpec motor_spec{.ticks_per_revolution = 6, .gear_ratio = 120, .max_rpm = 100};
  const PidGains pid_gains{.kp = 0.0, .ki = 0.0, .kd = 0.0};
  auto pid_driver = MotorDriverPid(raw_driver, decoder, motor_spec, pid_gains, "motor");

  parameters::register_parameter("speed", [&](float speed) {
    pid_driver.drive(speed);
  });
  parameters::start_updating();

  while (true) {
    sleep_ms(1000);
  }

  return 0;
}
