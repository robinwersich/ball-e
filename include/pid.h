#pragma once

#include <optional>

/**
 * A feedback controller that adjusts a control signal based on the error between a target and
 * a measurement. The control signal is the sum of the proportional (present), integral (past),
 * and derivative (future prediction) components of the error.
 */
class PidController {
 public:
  /** Creates a new PID controller with the given gains. */
  PidController(
    float out_min, float out_max, uint64_t sample_time_micros = 10000, float kp = 1.0,
    float ki = 0.0, float kd = 0.0
  );

  void set_gains(float kp, float ki, float kd);
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
  uint64_t _sample_time;
  float _kp, _ki, _kd;
  float _out_min, _out_max;
  float _scaled_error_sum = 0.0;  // already multiplied by ki
  float _previous_error = 0.0;
  float _last_target = 0.0;
  float _last_measurement = 0.0;
  uint64_t _last_time = 0;
};
