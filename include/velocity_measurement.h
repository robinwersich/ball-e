#pragma once

#include <memory>
#include <limits>

#include "motor_decoder.h"

/** Measures motor velocity based on decoder readings. */
class VelocityMeter {
 public:
  /**
   * Creates a new velocity meter.
   * @param decoder The decoder from which to read ticks.
   * @param ticks_per_revolution The number of ticks the encoder produces per motor revolution.
   * @param gear_ratio How many rotations of the motor are required for one shaft rotation.
   */
  VelocityMeter(
    std::shared_ptr<MotorDecoder> decoder, float ticks_per_revolution, float gear_ratio
  );

  /** Returns the current speed in ticks per second. */
  float compute_speed_tps();
  /** Returns the current speed in revolutions per second. */
  float compute_speed_rps();
  /** Returns the current speed in revolutions per minute. */
  float compute_speed_rpm();

 private:
  std::shared_ptr<MotorDecoder> _decoder;
  uint64_t _last_count_micros = 0;
  int64_t _last_count = 0;
  float _speed_factor;
};
