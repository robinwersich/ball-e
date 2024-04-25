#include "pico/stdlib.h"
#include "btcontrol.h"

void on_gamepad_data(const uni_gamepad_t& gamepad) {
  uni_gamepad_dump(&gamepad);
}

int main() {
  stdio_init_all();

  if (btcontrol::init()) {
    return 1;
  }

  btcontrol::register_gampad_behavior(on_gamepad_data);

  btcontrol::run_loop();

  return 0;
}
