#include "motor_state.h"

#include <limits>
#include "pico/time.h"

constexpr float seconds_per_micro = 1.0 / 1000000;

MotorState::MotorState(
  const MotorDecoderState* decoder_state, float ticks_per_revolution, float gear_ratio,
  float min_rpm
)
  : _decoder_state{decoder_state}
  , _revolutions_per_tick{1 / (ticks_per_revolution * gear_ratio)}
  , _max_sync_tick_interval{
      min_rpm == 0
        ? std::numeric_limits<uint32_t>::max()
        // 1 / ((min_rpm/(60*1000000)) * (ticks_per_revolution/2) * gear_ratio)
        : static_cast<uint32_t>(120000000 / (min_rpm * ticks_per_revolution * gear_ratio))
    } {}

double MotorState::current_position_revs() {
  return _decoder_state->count * static_cast<double>(_revolutions_per_tick);
}

double MotorState::current_position_deg() { return current_position_revs() * 360; }

float MotorState::compute_speed_tps() {
  // use syncronized count to avoid timing variations
  const auto current_count_micros = _decoder_state->last_sync_micros;
  const auto current_count = _decoder_state->syncronized_count;

  if (current_count_micros <= _last_count_micros) {
    // no encoder tick since last compute
    // possibly we are just polling too fast and the motor is still moving
    if (time_us_32() - current_count_micros < _max_sync_tick_interval) return _last_speed_tps;
    // otherwise it means motor has stopped
    // set last timestamp to now for more accurate velocity when motor starts again
    _last_count_micros = time_us_32();
    return _last_speed_tps = 0;
  }

  const auto delta_time = current_count_micros - _last_count_micros;
  const auto delta_count = current_count - _last_count;
  _last_count = current_count;
  _last_count_micros = current_count_micros;
  return _last_speed_tps = delta_count / (delta_time * seconds_per_micro);
}

float MotorState::compute_speed_rps() { return compute_speed_tps() * _revolutions_per_tick; }
float MotorState::compute_speed_rpm() { return compute_speed_rps() * 60; }
