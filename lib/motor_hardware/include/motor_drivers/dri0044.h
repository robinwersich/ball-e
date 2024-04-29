#pragma once

#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

class MotorDriverDRI0044 final : public MotorDriver {
 public:
  MotorDriverDRI0044(
    uint pwm_pin, uint direction_pin, std::shared_ptr<PwmSlice> pwm_slice,
    bool swap_direction = false
  );
  MotorDriverDRI0044(
    uint pwm_pin, uint direction_pin, uint pwm_frequency, bool swap_direction = false
  );

  void drive(float speed) override;

  void set_pwm_frequency(uint frequency);

 private:
  uint _pin_pwm, _pin_direction;
  std::shared_ptr<PwmSlice> _pwm_slice;
  uint _pwm_channel;
  bool _swap_direction;
  float _speed = 0.0;
};