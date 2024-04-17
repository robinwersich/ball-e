#pragma once

#include <memory>

#include "pico/types.h"

/**
 * Object representation of an active PWM slice.
 * The slice is active as long as the object exists, so there should only ever be one instance
 * for each slice, meaning a shared pointer should be used.
 */
class PwmSlice {
 public:
  static std::shared_ptr<PwmSlice> for_slice(uint slice_num, uint frequency);
  static std::shared_ptr<PwmSlice> for_pin(uint pin, uint frequency);
  /**
   * Construct a new PwmSlice object. Only one instance should exist for each slice,
   * so instead of using this constructor, use the for_slice or for_pin factory methods.
   * @param slice_num The PWM slice number to use.
   * @param frequency The frequency to set the PWM slice to.
  */
  PwmSlice(uint slice_num, uint frequency);
  /** Disables the PWM slice. */
  ~PwmSlice();

  void set_frequency(uint frequency);

  const uint slice_num;
  /** The number of clock ticks in one period of the PWM signal. */
  uint16_t period = 0xffff;
};
