#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverDRI0044 final : public MotorDriver {
 public:
  MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> pwm_slice);
  MotorDriverDRI0044(uint pwm, uint direction, uint pwm_frequency);

  void drive(float speed) override;

  void set_pwm_frequency(uint frequency);

 private:
  uint _pin_pwm, _pin_direction;
  std::shared_ptr<PwmSlice> _pwm_slice;
  uint _pwm_channel;
  float _speed = 0.0;
};