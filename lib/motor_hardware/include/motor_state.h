#pragma once

#include <memory>

#include "motor_decoder.h"

/** Measures motor position and velocity based on decoder readings. */
class MotorState {
 public:
  /**
   * Creates a new velocity meter.
   * @param decoder The decoder from which to read ticks.
   * @param ticks_per_revolution The number of ticks the encoder produces per motor revolution.
   * @param gear_ratio How many rotations of the motor are required for one shaft rotation.
   */
  MotorState(std::shared_ptr<MotorDecoder> decoder, float ticks_per_revolution, float gear_ratio);

  /** Returns the current position in terms of revolutions since start of measurement. */
  double current_position_revs();
  /** Returns the current position in terms of degrees since start of movement. */
  double current_position_deg();

  /** Computes the average speed since the last measurement in ticks per second. */
  float compute_speed_tps();
  /** Computes the average speed since the last measurement in revolutions per second. */
  float compute_speed_rps();
  /** Computes the average speed since the last measurement in revolutions per minute. */
  float compute_speed_rpm();

 private:
  std::shared_ptr<MotorDecoder> _decoder;
  uint64_t _last_count_micros = 0;
  int64_t _last_count = 0;
  float _revolutions_per_tick;  // shaft revolutions
};
