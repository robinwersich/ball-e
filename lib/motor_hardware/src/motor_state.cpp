#include "motor_state.h"

static const uint64_t max_measurement_interval = 1000000;
static const uint64_t micros_per_second = 1000000;

MotorState::MotorState(
  std::shared_ptr<MotorDecoder> decoder, float ticks_per_revolution, float gear_ratio
)
  : _decoder{std::move(decoder)}, _revolutions_per_tick{1 / (ticks_per_revolution * gear_ratio)} {}

double MotorState::current_position_revs() {
  return _decoder->count() * static_cast<double>(_revolutions_per_tick);
}

double MotorState::current_position_deg() { return current_position_revs() * 360; }

float MotorState::compute_speed_tps() {
  const auto new_count = _decoder->count();
  const auto new_count_micros = _decoder->last_count_micros();
  const auto delta_count = new_count - _last_count;
  const auto delta_time = new_count_micros - _last_count_micros;
  _last_count = new_count;
  _last_count_micros = new_count_micros;

  if (delta_time == 0 || delta_time > max_measurement_interval) return 0;

  return delta_count / static_cast<float>(delta_time / micros_per_second);
}

float MotorState::compute_speed_rps() { return compute_speed_tps() * _revolutions_per_tick; }
float MotorState::compute_speed_rpm() { return compute_speed_rps() * 60; }
