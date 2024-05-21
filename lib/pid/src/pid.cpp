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
  , _name{name}
#endif
{
#ifndef NDEBUG
  register_parameters();
#endif
}

PidController::~PidController() {
#ifndef NDEBUG
  unregister_parameters();
#endif
}

#ifndef NDEBUG
PidController::PidController(PidController&& other) : PidController(other) {
  other.unregister_parameters();
  other._name.clear();
  register_parameters();
}

PidController& PidController::operator=(PidController&& other) {
  *this = other;
  other.unregister_parameters();
  other._name.clear();
  register_parameters();
  return *this;
}
#endif

void PidController::reset() {
  _i_term = 0;
  _last_error = std::numeric_limits<float>::quiet_NaN();
  _last_measurement = std::numeric_limits<float>::quiet_NaN();
  _last_time_millis = 0;
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

bool PidController::is_initialized() const {
  return not (std::isnan(_last_error) || std::isnan(_last_measurement) || _last_time_millis == 0);
}

float PidController::compute(float measurement) {
  if (not is_initialized()) {
    _last_error = _target - measurement;
    _last_measurement = measurement;
    _last_time_millis = time_us_32();
    return 0;
  }

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
  if (not _name.empty()) {
    plot((_name + "_p").c_str(), p_term);
    plot((_name + "_i").c_str(), i_term);
    plot((_name + "_d").c_str(), d_term);
    plot((_name + "_target").c_str(), _target);
    plot((_name + "_measurement").c_str(), measurement);
    plot((_name + "_output").c_str(), output);
  }
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
  if (_name.empty()) return;
  parameters::unregister_parameter("kp_" + _name);
  parameters::unregister_parameter("ki_" + _name);
  parameters::unregister_parameter("kd_" + _name);
}
#endif
