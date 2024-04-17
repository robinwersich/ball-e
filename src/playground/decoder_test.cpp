#include "motor_decoder.h"
#include "pico/stdlib.h"

const uint ENCA = 2;
const uint ENCB = 3;

int main() {
  stdio_init_all();

  auto decoder = MotorDecoder(ENCA, ENCB);

  while (true) {
    sleep_ms(1000);
    printf("Count: %lld\n", decoder.count());
  }

  return 0;
}
