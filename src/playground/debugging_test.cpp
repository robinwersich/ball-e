#include <string>
#include <iostream>

#include "pico/stdlib.h"
#include "parameters.h"
#include "plot.h"

int main() {
  stdio_init_all();

  parameters::register_parameter("a", [](float value) {
    plot("a", value);
  });

  parameters::register_parameter("b", [](float value) {
    plot("b", value);
  });

  parameters::start_updating();
}
