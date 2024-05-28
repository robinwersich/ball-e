#pragma once

#include <limits>
#include <memory>

#include "motor_driver.h"
#include "pico/time.h"

struct Kickstart {
  /** The fraction of the motor's maximum input get the motor going from a standstill. */
  float start_threshold;
  /** The fraction of the motor's maximum input under which the motor will stop. */
  float end_threshold;
  /** The duration of the kickstart. */
  uint32_t duration_ms;
};

/**
 * This is a convenience wrapper around a MotorDriver that adds a kickstart to get the motor
 * going before switching to a simple proportional controller.
 */
class KickstartMotorDriver : public MotorDriver {
 public:
  /**
   * Creates a new pid motor driver. As this will register an interrupt for the PID controller,
   * only one instance of this class should exist per motor. For this reason, a shared pointer
   * should be used.
   * @param driver The motor driver to control.
   * @param decoder_state The motor decoder state to use for measuring the motor speed.
   * @param kickstart The strength of the kickstart in terms of the motor's maximum speed.
   */
  KickstartMotorDriver(std::unique_ptr<MotorDriver> driver, const Kickstart& kickstart);

  /** Make the motor move at the given fraction of its maximum speed (sign determines direction). */
  void drive(float speed) override;

  const Kickstart& kickstart() const { return _kickstart; }
  Kickstart& kickstart() { return _kickstart; }

 private:
  std::unique_ptr<MotorDriver> _driver;
  Kickstart _kickstart;
  float _current_speed = 0;
  alarm_id_t _current_kickstart_id = 0;

  void drive_at_set_speed();
};
