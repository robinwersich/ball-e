#include "pid.h"

#include "pico/stdlib.h"

PidController::PidController(
  float out_min, float out_max, uint64_t sample_time_micros, float kp, float ki, float kd
)
  : _out_min{out_min}
  , _out_max{out_max}
  , _sample_time{sample_time_micros}
  , _kp{kp}  // Multiplying ki by sample time allows omitting it in the compute function
  , _ki{ki * sample_time_micros}
  // Dividing kd by sample time allows omitting it in the compute function
  , _kd{kd / sample_time_micros} {}

void PidController::set_gains(float kp, float ki, float kd) {
  set_proportional_gain(kp);
  set_integral_gain(ki);
  set_derivative_gain(kd);
}

void PidController::set_proportional_gain(float kp) { _kp = kp; }
void PidController::set_integral_gain(float ki) { _ki = ki * _sample_time; }
void PidController::set_derivative_gain(float kd) { _kd = kd / _sample_time; }

void PidController::set_target(float target) { _last_target = target; }

float PidController::compute_at_sample_time(float measurement, float target) {
  const auto error = target - measurement;

  // multiplication by (constant) dt is part of ki
  _scaled_error_sum += error * _ki;

  // prevent integral windup
  if (_scaled_error_sum > _out_max) _scaled_error_sum = _out_max;
  else if (_scaled_error_sum < _out_min) _scaled_error_sum = _out_min;

  // derivative on measurement (not error) to prevent derivative kick
  // multiplication by (constant) dt is part of kd
  const auto measurement_change = measurement - _last_measurement;

  _previous_error = error;

  return _kp * error + _ki * _scaled_error_sum - _kd * measurement_change;
}

float PidController::compute_at_sample_time(float measurement) {
  return compute_at_sample_time(measurement, _last_target);
}

std::optional<float> PidController::compute_if_sample_time(float measurement, float target) {
  const auto now = time_us_64();
  const auto dt = now - _last_time;

  if (dt < _sample_time) return {};

  _last_time = now;
  return compute_at_sample_time(measurement, target);
}

std::optional<float> PidController::compute_if_sample_time(float measurement) {
  return compute_if_sample_time(measurement, _last_target);
}
