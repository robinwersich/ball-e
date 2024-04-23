#pragma once

#include <optional>
#ifdef DEBUG
#include <map>
#include <string>
#endif

#include "pico/types.h"

struct PidGains {
  float kp = 0.0, ki = 0.0, kd = 0.0;
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
   * @param sample_time_millis The time between each control signal computation in milliseconds.
   * @param gains The proportional, integral, and derivative gain for the controller.
   * @param category The category of this controller.
   *   In debug mode, tuning parameters will be registered for each category:
   *   kp_<category>, ki_<category>, kd_<category>
   */
  PidController(
    float out_min, float out_max, uint32_t sample_time_millis = 10, PidGains gains = {},
    const char* category = ""
  );

  ~PidController();

  PidController(const PidController&) = delete;
  PidController& operator=(const PidController&) = delete;

  void set_gains(PidGains gains);
  void set_proportional_gain(float kp);
  void set_integral_gain(float ki);
  void set_derivative_gain(float kd);

  void set_target(float target);

  /**
   * Computes the control signal based on a measurement and target and updates the target.
   * This function must be called exactly once per sample time set by set_sample_time.
   */
  float compute_at_sample_time(float measurement, float target);
  /**
   * Computes the control signal based on the given measurement and the previously set target.
   * This function must be called exactly once per sample time set by set_sample_time.
   */
  float compute_at_sample_time(float measurement);
  /**
   * Computes the control signal based on a measurement and target and updates the target.
   * This function must be called in every loop iteration and checks if the sample time has passed.
   * @returns A bool if a new output has been computed and the new output value (if computed).
   */
  std::optional<float> compute_if_sample_time(float measurement, float target);
  /**
   * Computes the control signal based on the given measurement and the previously set target.
   * This function must be called in every loop iteration and checks if the sample time has passed.
   * @returns A bool if a new output has been computed and the new output value (if computed).
   */
  std::optional<float> compute_if_sample_time(float measurement);

 private:
  uint32_t _sample_time_millis;
  float _kp, _ki, _kd;
  float _out_min, _out_max;
  float _scaled_error_sum = 0.0;  // already multiplied by ki
  float _last_error = 0.0;
  float _target = 0.0;
  float _last_measurement = 0.0;
  uint32_t _last_time_millis = 0;

#ifdef DEBUG
  static std::multimap<std::string, PidController*> _controllers_by_category;
  static void register_controller(const std::string& category, PidController* controller);
  static void unregister_controller(PidController* controller);
#endif
};
