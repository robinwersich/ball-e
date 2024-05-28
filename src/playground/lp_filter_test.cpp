#include "lowpass.h"
#include "motor_decoder.h"
#include "motor_drivers/dri0044.h"
#include "motor_state.h"
#include "parameters.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "plot.h"

const uint DIR = 0;
const uint PWM = 1;
const uint ENC_SLOT = 1;

int main() {
  stdio_init_all();

  auto motor = std::make_shared<MotorDriverDRI0044>(PWM, DIR, 25000);
  auto decoder = MotorDecoder(ENC_SLOT);
  MotorState motor_state{&decoder.state(), 6, 115, 1};
  LowPassFilter filter{{.a1 = 0.7284895, .b0 = 0.13575525, .b1 = 0.13575525}};

  parameters::register_parameter("speed", [&](float speed) { motor->drive(speed); });
  multicore_launch_core1(parameters::start_updating);

  while (true) {
    sleep_ms(5);
    plot("speed", filter.filter(motor_state.compute_speed_rpm()));
  }

  return 0;
}
