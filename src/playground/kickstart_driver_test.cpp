#include "motor_driver_kickstart.h"
#include "motor_drivers/dri0044.h"
#include "motor_state.h"
#include "parameters.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "plot.h"

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

int main() {
  stdio_init_all();

  auto raw_driver_1 = std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY, true);
  auto raw_driver_2 = std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY);
  auto raw_driver_3 = std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY);
  const Kickstart kickstart{.start_threshold = 0.0, .end_threshold = 0.0, .duration_ms = 0};
  KickstartMotorDriver kickstart_driver_1{std::move(raw_driver_1), kickstart};
  KickstartMotorDriver kickstart_driver_2{std::move(raw_driver_2), kickstart};
  KickstartMotorDriver kickstart_driver_3{std::move(raw_driver_3), kickstart};

  auto decoder_1 = MotorDecoder(ENC1_SLOT, true);
  auto decoder_2 = MotorDecoder(ENC2_SLOT);
  auto decoder_3 = MotorDecoder(ENC3_SLOT, true);
  MotorState motor_state_1{&decoder_1.state(), 6, 115};
  MotorState motor_state_2{&decoder_2.state(), 6, 115};
  MotorState motor_state_3{&decoder_3.state(), 6, 115};

  parameters::register_parameter("speed", [&](float speed) {
    kickstart_driver_1.drive(speed);
    kickstart_driver_2.drive(speed);
    kickstart_driver_3.drive(speed);
  });
  parameters::register_parameter("start_threshold", [&](float start_threshold) {
    kickstart_driver_1.kickstart().start_threshold = start_threshold;
    kickstart_driver_2.kickstart().start_threshold = start_threshold;
    kickstart_driver_3.kickstart().start_threshold = start_threshold;
  });
  parameters::register_parameter("end_threshold", [&](float end_threshold) {
    kickstart_driver_1.kickstart().end_threshold = end_threshold;
    kickstart_driver_2.kickstart().end_threshold = end_threshold;
    kickstart_driver_3.kickstart().end_threshold = end_threshold;
  });
  parameters::register_parameter("duration_ms", [&](uint32_t duration_us) {
    kickstart_driver_1.kickstart().duration_ms = duration_us;
    kickstart_driver_2.kickstart().duration_ms = duration_us;
    kickstart_driver_3.kickstart().duration_ms = duration_us;
  });

  sleep_ms(2000);
  multicore_launch_core1(parameters::start_updating);

  while (true) {
    sleep_ms(10);
    plot("speed 1", motor_state_1.compute_speed_rpm());
    plot("speed 2", motor_state_2.compute_speed_rpm());
    plot("speed 3", motor_state_3.compute_speed_rpm());
  }

  return 0;
}
