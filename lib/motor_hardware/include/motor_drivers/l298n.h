#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverL298N final : public MotorDriver {
 public:
  MotorDriverL298N(uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> pwm_slice);
  MotorDriverL298N(uint enable, uint in1, uint in2, uint pwm_frequency);

  void drive(float speed) override;

 private:
  uint _pin_enable, _pin_in1, _pin_in2;
  std::shared_ptr<PwmSlice> _pwm_slice;
  uint _pwm_channel;
};