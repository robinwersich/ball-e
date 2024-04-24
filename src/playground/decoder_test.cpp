#include "motor_decoder.h"
#include "pico/stdlib.h"
#include "plot.h"

// -- motor 1 --
const uint DIR1 = 0;
const uint PWM1 = 1;
const uint ENC1A = 2;
const uint ENC1B = 3;
// -- motor 2 --
const uint ENC2B = 4;
const uint ENC2A = 5;
const uint DIR2 = 6;
const uint PWM2 = 7;
// -- motor 3 --
const uint PWM3 = 8;
const uint DIR3 = 9;
const uint ENC3A = 10;
const uint ENC3B = 11;

int main() {
  stdio_init_all();

  auto decoder1 = MotorDecoder::for_pins(ENC1A, ENC1B);
  auto decoder2 = MotorDecoder::for_pins(ENC2A, ENC2B);
  auto decoder3 = MotorDecoder::for_pins(ENC3A, ENC3B);

  while (true) {
    sleep_ms(50);
    plot("pos1", decoder1->count());
    plot("pos2", decoder2->count());
    plot("pos3", decoder3->count());
  }

  return 0;
}
