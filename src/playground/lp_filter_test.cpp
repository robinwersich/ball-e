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
  LowPassCoefficients coefficients{.a1 = 0.88176521, .b0 = 0.0591174, .b1 = 0.0591174};
  MotorState motor_state_pre{
    &decoder.state(), 6, 115,
    coefficients
  };
  MotorState motor_state_post{&decoder.state(), 6, 115};
  LowPassFilter post_filter{coefficients};

  parameters::register_parameter("speed", [&](float speed) { motor->drive(speed); });
  multicore_launch_core1(parameters::start_updating);

  while (true) {
    sleep_ms(10);
    plot("speed_pre_filtered", motor_state_pre.compute_speed_rpm());
    plot("speed_post_filtered", post_filter.filter(motor_state_post.compute_speed_rpm()));
  }

  return 0;
}
