#include "pid.h"

#include <algorithm>

#include "pico/stdlib.h"

PidController::PidController(
  float out_min, float out_max, uint32_t sample_time_millis, PidGains gains
)
  : _out_min{out_min}
  , _out_max{out_max}
  , _sample_time_millis{sample_time_millis}
  , _kp{gains.kp}  // Multiplying ki by sample time allows omitting it in the compute function
  , _ki{gains.ki * sample_time_millis}
  // Dividing kd by sample time allows omitting it in the compute function
  , _kd{gains.kd / sample_time_millis} {}

void PidController::set_gains(PidGains gains) {
  set_proportional_gain(gains.kp);
  set_integral_gain(gains.ki);
  set_derivative_gain(gains.kd);
}

void PidController::set_proportional_gain(float kp) { _kp = kp; }
void PidController::set_integral_gain(float ki) { _ki = ki * _sample_time_millis; }
void PidController::set_derivative_gain(float kd) { _kd = kd / _sample_time_millis; }

void PidController::set_target(float target) { _last_target = target; }

float PidController::compute_at_sample_time(float measurement, float target) {
  const auto error = target - measurement;

  // multiplication by (constant) dt is part of ki
  _scaled_error_sum += error * _ki;
  // prevent integral windup
  _scaled_error_sum = std::clamp(_scaled_error_sum, _out_min, _out_max);
  // derivative on measurement (not error) to prevent derivative kick
  // multiplication by (constant) dt is part of kd
  const auto measurement_change = measurement - _last_measurement;

  _previous_error = error;

  const auto output = _kp * error + _scaled_error_sum - _kd * measurement_change;
  return std::clamp(output, _out_min, _out_max);
}

float PidController::compute_at_sample_time(float measurement) {
  return compute_at_sample_time(measurement, _last_target);
}

std::optional<float> PidController::compute_if_sample_time(float measurement, float target) {
  const auto now = us_to_ms(time_us_64());
  const auto dt = now - _last_time_millis;

  if (dt < _sample_time_millis) return {};

  _last_time_millis = now;
  return compute_at_sample_time(measurement, target);
}

std::optional<float> PidController::compute_if_sample_time(float measurement) {
  return compute_if_sample_time(measurement, _last_target);
}
