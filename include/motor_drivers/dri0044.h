#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverDRI0044 final : public MotorDriver {
 public:
  MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> pwm_slice);

  void drive(float speed) override;

 private:
  uint pin_pwm, pin_direction;
  std::shared_ptr<PwmSlice> pwm_slice;
  uint pwm_channel;
};