#include "motor_driver_kickstart.h"
#include "motor_drivers/dri0044.h"
#include "motor_state.h"
#include "parameters.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "plot.h"

const uint PWM_FREQUENCY = 25000;
// -- motor 1 --
const uint DIR = 0;
const uint PWM = 1;
const uint ENC_SLOT = 1;

int main() {
  stdio_init_all();

  auto raw_driver = std::make_shared<MotorDriverDRI0044>(PWM, DIR, PWM_FREQUENCY);
  const Kickstart kickstart{.start_threshold = 0.0, .end_threshold = 0.0, .duration_ms = 0};
  KickstartMotorDriver kickstart_driver{raw_driver, kickstart};

  auto decoder = MotorDecoder(ENC_SLOT);
  MotorState motor_state{&decoder.state(), 6, 115, 1};

  parameters::register_parameter("speed", [&](float speed) { kickstart_driver.drive(speed); });
  parameters::register_parameter("start_threshold", [&](float start_threshold) {
    kickstart_driver.kickstart().start_threshold = start_threshold;
  });
  parameters::register_parameter("end_threshold", [&](float end_threshold) {
    kickstart_driver.kickstart().end_threshold = end_threshold;
  });
  parameters::register_parameter("duration_ms", [&](uint32_t duration_us) {
    kickstart_driver.kickstart().duration_ms = duration_us;
  });

  sleep_ms(2000);
  multicore_launch_core1(parameters::start_updating);

  while (true) {
    sleep_ms(10);
    plot("speed", motor_state.compute_speed_rpm());
  }

  return 0;
}
