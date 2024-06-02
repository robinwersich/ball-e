#include "motor_state.h"

#include <limits>

#include "pico/time.h"

constexpr float seconds_per_micro = 1.0 / 1000000;

MotorState::MotorState(
  const MotorDecoderState* decoder_state, float ticks_per_revolution, float gear_ratio,
  const LowPassCoefficients& speed_filter
)
  : _decoder_state{decoder_state}
  , _revolutions_per_tick{1 / (ticks_per_revolution * gear_ratio)}
  , _speed_filter{speed_filter} {}

double MotorState::current_position_revs() {
  return _decoder_state->count * static_cast<double>(_revolutions_per_tick);
}

double MotorState::current_position_deg() { return current_position_revs() * 360; }

float MotorState::compute_speed_tps_filtered() {
  uint32_t current_count_micros = time_us_32();
  const auto delta_time = (current_count_micros - _last_count_micros) * seconds_per_micro;
  _last_count_micros = current_count_micros;
  return _speed_filter.filtered_derivative(_decoder_state->count, delta_time);
}

float MotorState::compute_speed_tps_unfiltered() {
  const auto current_count = _decoder_state->syncronized_count;
  if (current_count == _last_count) return 0;
  const auto delta_count = current_count - _last_count;
  _last_count = current_count;

  const auto current_count_micros = _decoder_state->last_sync_micros;
  const auto delta_time = (current_count_micros - _last_count_micros) * seconds_per_micro;
  _last_count_micros = current_count_micros;

  return delta_count / delta_time;
}

float MotorState::compute_speed_tps() {
  return _speed_filter.is_passthrough() ? compute_speed_tps_unfiltered()
                                        : compute_speed_tps_filtered();
}

float MotorState::compute_speed_rps() { return compute_speed_tps() * _revolutions_per_tick; }
float MotorState::compute_speed_rpm() { return compute_speed_rps() * 60; }
