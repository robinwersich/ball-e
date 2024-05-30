#pragma once

#include <cmath>
#ifndef NDEBUG
#include <map>
#include <string>
#endif

#include "lowpass.h"
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
   * @param filter An optional low-pass filter to apply to the control signal.
   * @param name The name of this controller.
   *   In debug mode, tuning parameters will be registered for each controller with a name:
   *   kp_<name>, ki_<name>, kd_<name>
   */
  PidController(float out_min, float out_max, PidGains gains = {}, const LowPassCoefficients& filter = {}, const char* name = "");

  void set_gains(PidGains gains);
  void set_proportional_gain(float kp);
  void set_integral_gain(float ki);
  void set_derivative_gain(float kd);

  void set_target(float target);

  /** Computes the control signal based on a measurement and target and updates the target. */
  float compute(float measurement, float target);
  /** Computes the control signal based on the given measurement and the previously set target. */
  float compute(float measurement);

  /**
   * Resets the controllers internal state (but not the target).
   * Should be called when compute was not called for a longer time.
   */
  void reset();

#ifndef NDEBUG
  /**
   * Registers tuning parameters for one or multiple controllers
   * @param controllers The controllers to register tuning parameters for.
   * @param name The suffix for the tuning parameters. If empty, the name of the controller is used.
   */
  static void register_parameters(
    std::initializer_list<PidController*> controllers, const std::string& name = ""
  );

  /** Unregisters tuning parameters for all controllers registered with the given name. */
  static void unregister_parameters(std::initializer_list<std::string> names);
#endif

 private:
  uint32_t _sample_time_millis;
  float _kp, _ki, _kd;  // gains in C/M, C/(M*us), C/(M/us) to avoid unnecessary computations
  float _out_min, _out_max;
  float _i_term = 0.0;  // error sum multiplied by ki
  float _last_error = 0;
  float _target = 0.0;
  uint32_t _last_time_millis = 0;  // 0 means uninitialized
  LowPassFilter _filter;

  bool is_initialized() const;

#ifndef NDEBUG
  std::string _name;
#endif
};
