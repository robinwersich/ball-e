#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverL298N final : public MotorDriver {
 public:
  MotorDriverL298N(uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> pwm_slice);

  void drive(float speed) override;

 private:
  uint pin_enable, pin_in1, pin_in2;
  std::shared_ptr<PwmSlice> pwm_slice;
  uint pwm_channel;
};