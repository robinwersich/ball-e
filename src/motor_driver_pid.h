#pragma once

#include <memory>
#include <type_traits>
#include <vector>

#include "motor_decoder.h"
#include "motor_driver.h"
#include "motor_state.h"
#include "pico/time.h"
#include "pid.h"

struct MotorSpec {
  // The number of encoder ticks per revolution of the motor.
  float ticks_per_revolution;
  // The number of motor revolutions required for one revolution of the output shaft.
  float gear_ratio;
  // The maximum speed the output shaft can achieve in revolutions per minute.
  float max_rpm;
};

/**
 * This is a convenience wrapper around a MotorDriver that adds PID control.
 * It automatically registeres the required interrupts for updating the PID controller and can
 * be used as a drop-in replacement for a MotorDriver.
 * If a high frequency control loop or timed interrupts are already used, it is recommended to
 * use a PidController and MotorDriver directly.
 */
class MotorDriverPid : public MotorDriver {
 public:
  /** Rate for sampling and adjusting the motor velocity */
  static const uint32_t sample_time_millis = 50;

  /**
   * Creates a new pid motor driver. As this will register an interrupt for the PID controller,
   * only one instance of this class should exist per motor. For this reason, a shared pointer
   * should be used.
   * @param driver The motor driver to control.
   * @param decoder_state The motor decoder state to use for measuring the motor speed.
   * @param motor_spec The specifications of the motor.
   * @param pid_gains The proportional, integral, and derivative gains for the PID controller.
   *  Units are VM/(rev/s), VM/rev, and VM/(rev/s²) for kp, ki, and kd respectively.
   *  where VM is the maximum output voltage of the motor driver.
   * @param name The name of the controller for debugging purposes.
   *  If given, the PID gains will be registered as tuning parameters with this name.
   */
  MotorDriverPid(
    std::shared_ptr<MotorDriver> driver, const MotorDecoderState* decoder_state,
    MotorSpec motor_spec, PidGains pid_gains, const char* name = ""
  );
  /** Unregisters the timer interrupt for updating the controller. */
  ~MotorDriverPid();

  MotorDriverPid(const MotorDriverPid&) = delete;
  MotorDriverPid& operator=(const MotorDriverPid&) = delete;

  /** Make the motor move at the given RPM (sign determines direction). */
  void drive_rpm(float rpm);
  /** Make the motor move at the given fraction of its maximum speed (sign determines direction). */
  void drive(float speed) override;

  /** Returns the PID controller so that it can be adjusted. */
  PidController& controller();

 private:
  std::shared_ptr<MotorDriver> _driver;
  PidController _controller;
  MotorState _motor_state;
  float _max_speed;

  repeating_timer_t _update_timer;

  void update_controller();
};
