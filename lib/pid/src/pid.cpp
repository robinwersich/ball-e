#include "pid.h"

#include <algorithm>

#ifndef NDEBUG
#include "parameters.h"
#include "plot.h"
#endif
#include "pico/stdlib.h"

PidController::PidController(float out_min, float out_max, PidGains gains, const char* name)
  : _out_min{out_min}
  , _out_max{out_max}
  , _kp{gains.kp}
  , _ki{gains.ki / 10e6f}  // convert from C/(M*s) to C/(M*us)
  , _kd{gains.kd * 10e6f}  // convert from C/(M/s) to C/(M/us)
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
void PidController::set_integral_gain(float ki) { _ki = ki / 10e6f; }
void PidController::set_derivative_gain(float kd) { _kd = kd * 10e6f; }

void PidController::set_target(float target) { _target = target; }

float PidController::compute(float measurement, float target) {
  _target = target;
  return compute(measurement);
}

float PidController::compute(float measurement) {
  const auto error = _target - measurement;
  const auto now = time_us_32();
  const auto dt = (now - _last_time_millis);
  _last_time_millis = now;

  _i_term += (error + _last_error) / 2 * dt * _ki;
  // prevent integral windup
  _i_term = std::clamp(_i_term, _out_min, _out_max);
  // derivative on measurement (not error) to prevent derivative kick
  const auto measurement_change = (measurement - _last_measurement) / dt;
  const auto p_term = error * _kp;
  const auto i_term = _i_term;
  const auto d_term = -measurement_change * _kd;

  _last_error = error;
  _last_measurement = measurement;

  const auto output = std::clamp(p_term + i_term + d_term, _out_min, _out_max);
#ifndef NDEBUG
  plot("p", p_term);
  plot("i", i_term);
  plot("d", d_term);
  plot("target", _target);
  plot("measurement", measurement);
  plot("output", output);
#endif

  return output;
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
