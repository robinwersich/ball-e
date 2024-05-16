#include "motor_decoder.h"
#include "motor_drivers/dri0044.h"
#include "motor_state.h"
#include "parameters.h"
#include "pico/stdlib.h"
#include "plot.h"

const uint DIR = 0;
const uint PWM = 1;
const uint ENC_SLOT = 1;

int main() {
  stdio_init_all();

  auto motor = std::make_shared<MotorDriverDRI0044>(PWM, DIR, 25000);
  auto decoder = MotorDecoder(ENC_SLOT);
  MotorState motor_state{&decoder.state(), 6, 115};

  parameters::register_parameter("frequency", [&](float frequency) {
    motor->set_pwm_frequency(frequency);
  });
  parameters::register_parameter("speed", [&](float speed) {
    motor->drive(speed);
  });
  parameters::start_updating();

  while (true) {
    sleep_ms(10);
    plot("speed", motor_state.compute_speed_rpm());
  }

  return 0;
}
