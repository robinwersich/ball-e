#pragma once

#include <optional>
#ifndef NDEBUG
#include <map>
#include <string>
#endif

#include "pico/types.h"

/** The PID gains are defined in units based on the measurement unit M and control signal unit C. */
struct PidGains {
  /** Proportional gain in C/M. */
  float kp = 0.0;
  /** Integral gain in C/(M*s). */
  float ki = 0.0;
  /** Derivative gain in C/(M/s). */
  float kd = 0.0;
};

/**
 * A feedback controller that adjusts a control signal based on the error between a target and
 * a measurement. The control signal is the sum of the proportional (present), integral (past),
 * and derivative (future prediction) components of the error.
 */
class PidController {
 public:
  /**
   * Creates a new PID controller.
   * @param out_min The minimum value the controlled plant can accept.
   * @param out_max The maximum value the controlled plant can accept.
   * @param gains The proportional, integral, and derivative gain for the controller.
   * @param category The name of this controller.
   *   In debug mode, tuning parameters will be registered for each controller with a name:
   *   kp_<name>, ki_<name>, kd_<name>
   */
  PidController(float out_min, float out_max, PidGains gains = {}, const char* name = "");

  ~PidController();

  PidController(const PidController&) = delete;
  PidController& operator=(const PidController&) = delete;

  void set_gains(PidGains gains);
  void set_proportional_gain(float kp);
  void set_integral_gain(float ki);
  void set_derivative_gain(float kd);

  void set_target(float target);

  /** Computes the control signal based on a measurement and target and updates the target. */
  float compute(float measurement, float target);
  /** Computes the control signal based on the given measurement and the previously set target. */
  float compute(float measurement);

 private:
  uint32_t _sample_time_millis;
  float _kp, _ki, _kd;  // gains in C/M, C/(M*us), C/(M/us) to avoid unnecessary computations
  float _out_min, _out_max;
  float _i_term = 0.0;  // error sum multiplied by ki
  float _last_error = 0.0;
  float _target = 0.0;
  float _last_measurement = 0.0;
  uint32_t _last_time_millis = 0;
#ifndef NDEBUG
  std::string _name;

  void register_parameters();
  void unregister_parameters();
#endif
};
