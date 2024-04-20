#include "velocity_measurement.h"

static const uint64_t max_measurement_interval = 1000000;
static const uint64_t micros_per_second = 1000000;

VelocityMeter::VelocityMeter(
  std::shared_ptr<MotorDecoder> decoder, float ticks_per_revolution, float gear_ratio
)
  : _decoder{std::move(decoder)}, _speed_factor{1 / (ticks_per_revolution * gear_ratio)} {}

float VelocityMeter::compute_speed_tps() {
  const auto new_count = _decoder->count();
  const auto new_count_micros = _decoder->last_count_micros();
  const auto delta_count = new_count - _last_count;
  const auto delta_time = new_count_micros - _last_count_micros;
  _last_count = new_count;
  _last_count_micros = new_count_micros;

  if (delta_time == 0 || delta_time > max_measurement_interval) return 0;

  return delta_count / static_cast<float>(delta_time / micros_per_second);
}

float VelocityMeter::compute_speed_rps() { return compute_speed_tps() * _speed_factor; }
float VelocityMeter::compute_speed_rpm() { return compute_speed_rps() * 60; }
