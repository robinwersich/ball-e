#pragma once

#include <memory>

#include "motor_decoder.h"

/** Measures motor position and velocity based on decoder readings. */
class MotorState {
 public:
  /**
   * Creates a new velocity meter.
   * @param decoder_state The decoder state from which to read ticks.
   * @param ticks_per_revolution The number of ticks the encoder produces per motor revolution.
   * @param gear_ratio How many rotations of the motor are required for one shaft rotation.
   * @param min_rpm The minimum RPM the motor can achieve. Based on this, the speed computation
   *  will avoid returning a false zero speed at high polling rates when the motor is moving slowly.
   */
  MotorState(const MotorDecoderState* decoder_state, float ticks_per_revolution, float gear_ratio, float min_rpm = 0.0);

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
  const MotorDecoderState* _decoder_state;
  uint32_t _last_count_micros = 0;
  int32_t _last_count = 0;
  float _last_speed_tps = 0;
  float _revolutions_per_tick;  // shaft revolutions
  uint32_t _max_sync_tick_interval;
};
