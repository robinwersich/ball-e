#include "motor_decoder.h"
#include "motor_drivers/dri0044.h"
#include "parameters.h"
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

  MotorDriverDRI0044 motor1{PWM1, DIR1, PWM_FREQUENCY, true};
  MotorDriverDRI0044 motor2{PWM2, DIR2, PWM_FREQUENCY};
  MotorDriverDRI0044 motor3{PWM3, DIR3, PWM_FREQUENCY};
  MotorDecoder decoder1{ENC1_SLOT, true};
  MotorDecoder decoder2{ENC2_SLOT};
  MotorDecoder decoder3{ENC3_SLOT, true};

  parameters::register_parameter("speed", [&](float speed) {
    motor1.drive(speed);
    motor2.drive(speed);
    motor3.drive(speed);
  });
  parameters::start_updating();

  while (true) {
    sleep_ms(50);
    plot("pos1", decoder1.state().count);
    plot("pos2", decoder2.state().count);
    plot("pos3", decoder3.state().count);
  }

  return 0;
}
