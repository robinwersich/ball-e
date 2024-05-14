#pragma once

#include "pico/stdio.h"
#include "pico/time.h"

inline void plot(const char* label, float value) {
  const auto millis = us_to_ms(time_us_32());
  printf("$p %s,%lu,%f\n", label, millis, value);
}

inline void show_vector(const char* label, float x, float y, float z) {
  printf("$v %s,%f,%f,%f\n", label, x, y, z);
}
