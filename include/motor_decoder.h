#pragma once

#include <memory>

#include "pico/types.h"

/**
 * Counts the ticks of a motor encoder.
 * There should only be one instance of this class per encoder pin pair because interrupts are
 * registered and unregistered on construction and destruction.
 */
class MotorDecoder {
 public:
  static std::shared_ptr<MotorDecoder> for_pins(uint pin_a, uint pin_b);
  /**
   * Construct a new MotorDecoder object. Only one instance should exist for each encoder pin pair,
   * so instead of using this constructor, use the for_pins factory method.
   */
  MotorDecoder(uint pin_a, uint pin_b);
  /** Unregisters the interrupt handlers. */
  ~MotorDecoder();

  inline int64_t count() const { return _count; }

 private:
  const uint _pin_a, _pin_b;
  int64_t _count = 0;

  void on_a_change();
  void on_b_change();
};
