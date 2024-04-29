#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverL298N final : public MotorDriver {
 public:
  MotorDriverL298N(
    uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> pwm_slice,
    bool swap_direction = false
  );
  MotorDriverL298N(
    uint enable, uint in1, uint in2, uint pwm_frequency, bool swap_direction = false
  );

  void drive(float speed) override;

  void set_pwm_frequency(uint frequency);

 private:
  uint _pin_enable, _pin_in1, _pin_in2;
  std::shared_ptr<PwmSlice> _pwm_slice;
  uint _pwm_channel;
  bool _swap_direction;
  float _speed = 0.0;
};