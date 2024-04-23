#pragma once

#include "pico/stdio.h"
#include "pico/time.h"

inline void plot(const char* label, float value) {
  const auto millis = us_to_ms(time_us_32());
  printf("$ %s %lu %f\n", label, millis, value);
}
