#include "motor_decoder.h"
#include "pico/stdlib.h"
#include "plot.h"

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

  auto decoder1 = MotorDecoder(ENC1_SLOT);
  auto decoder2 = MotorDecoder(ENC2_SLOT);
  auto decoder3 = MotorDecoder(ENC3_SLOT);

  while (true) {
    sleep_ms(50);
    plot("pos1", decoder1.state().count);
    plot("pos2", decoder2.state().count);
    plot("pos3", decoder3.state().count);
  }

  return 0;
}
