#pragma once

#include <memory>

#include "pico/types.h"

/**
 * Object representation of an active PWM slice.
 * The slice is active as long as the object exists, so there should only ever be one instance
 * for each slice, meaning a shared pointer should be used.
 */
struct PwmSlice {
  PwmSlice(uint slice_num, uint frequency = 1000);
  ~PwmSlice();
  static std::shared_ptr<PwmSlice> forPin(uint pin, uint frequency = 1000);

  void set_frequency(uint frequency);

  uint slice_num;
  uint16_t wrap = 0xffff;
};
