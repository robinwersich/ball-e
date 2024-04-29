#pragma once

#include <array>
#include <memory>

#include "pico/types.h"

/** Contains the current state of one MotorDecoder */
struct MotorDecoderState {
  // The (signed) number of ticks counted by the encoder.
  int32_t count = 0;
  // Same as _count, but only updated on rising edges to ensure consistent timing.
  int32_t syncronized_count = count;
  // The time in microseconds when _syncronized_count was last updated.
  uint32_t last_sync_micros = 0;
  // If true, the count will be updated in the opposite direction.
  bool swap_direction = false;
};

/**
 * Counts the ticks of a motor encoder.
 * There should only be one instance of this class per encoder pin pair because interrupts are
 * registered and unregistered on construction and destruction.
 * To let other objects use the information of this class, share a pointer to.
 */
class MotorDecoder {
  friend class MotorDecoderHandle;
 public:
  /**
   * Construct a new MotorDecoder object. This serves as a lifetime manager, meaning that callbacks
   * will be registered and unregistered on construction and destruction.
   * To access the state of the encoder, use the state() method.
   * @param slot The GPIO pin pair to use for the encoder, determined by A=2*slot and B=2*slot+1.
   *  Must be in the range [0, 7].
   * @param swap_direction If true, the encoder count will be negated.
   */
  MotorDecoder(uint8_t slot, bool swap_direction = false);
  /** Unregisters the interrupt handlers. */
  ~MotorDecoder();

  const MotorDecoderState& state() const;

  MotorDecoder(const MotorDecoder&) = delete;
  MotorDecoder& operator=(const MotorDecoder&) = delete;

 private:
  const uint8_t _slot;

  static std::array<MotorDecoderState, 8> _states;
  static void on_a_change(uint pin, uint32_t event);
};
