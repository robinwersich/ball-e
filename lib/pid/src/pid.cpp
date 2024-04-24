#include "pid.h"

#include <algorithm>

#ifndef NDEBUG
#include "parameters.h"
#endif
#include "pico/stdlib.h"

PidController::PidController(
  float out_min, float out_max, uint32_t sample_time_millis, PidGains gains, const char* name
)
  : _out_min{out_min}
  , _out_max{out_max}
  , _sample_time_millis{sample_time_millis}
  , _kp{gains.kp}  // Multiplying ki by sample time allows omitting it in the compute function
  , _ki{gains.ki * sample_time_millis}
  // Dividing kd by sample time allows omitting it in the compute function
  , _kd{gains.kd / sample_time_millis}
#ifndef NDEBUG
  , _name{name} {
  register_parameters();
}
#else
{
}
#endif

PidController::~PidController() {
#ifndef NDEBUG
  unregister_parameters();
#endif
}

void PidController::set_gains(PidGains gains) {
  set_proportional_gain(gains.kp);
  set_integral_gain(gains.ki);
  set_derivative_gain(gains.kd);
}

void PidController::set_proportional_gain(float kp) { _kp = kp; }
void PidController::set_integral_gain(float ki) { _ki = ki * _sample_time_millis; }
void PidController::set_derivative_gain(float kd) { _kd = kd / _sample_time_millis; }

void PidController::set_target(float target) { _target = target; }

float PidController::compute_at_sample_time(float measurement, float target) {
  _target = target;
  return compute_at_sample_time(measurement);
}

float PidController::compute_at_sample_time(float measurement) {
  const auto error = _target - measurement;

  // multiplication by (constant) dt is part of ki
  _scaled_error_sum += (error + _last_error) / 2 * _ki;
  // prevent integral windup
  _scaled_error_sum = std::clamp(_scaled_error_sum, _out_min, _out_max);
  // derivative on measurement (not error) to prevent derivative kick
  // multiplication by (constant) dt is part of kd
  const auto measurement_change = measurement - _last_measurement;

  _last_error = error;
  _last_measurement = measurement;

  const auto output = _kp * error + _scaled_error_sum - _kd * measurement_change;
  return std::clamp(output, _out_min, _out_max);
}

std::optional<float> PidController::compute_if_sample_time(float measurement, float target) {
  const auto now = us_to_ms(time_us_64());
  const auto dt = now - _last_time_millis;

  if (dt < _sample_time_millis) return {};

  _last_time_millis = now;
  return compute_at_sample_time(measurement, target);
}

std::optional<float> PidController::compute_if_sample_time(float measurement) {
  return compute_if_sample_time(measurement, _target);
}

#ifndef NDEBUG
void PidController::register_parameters() {
  if (_name.empty()) return;

  auto register_gain_param = [&](const std::string& prefix, void (PidController::*setter)(float)) {
    parameters::register_parameter(prefix + _name, [this, setter](float value) {
      (this->*setter)(value);
    });
  };

  register_gain_param("kp_", &PidController::set_proportional_gain);
  register_gain_param("ki_", &PidController::set_integral_gain);
  register_gain_param("kd_", &PidController::set_derivative_gain);
}

void PidController::unregister_parameters() {
  parameters::unregister_parameter("kp_" + _name);
  parameters::unregister_parameter("ki_" + _name);
  parameters::unregister_parameter("kd_" + _name);
}
#endif
