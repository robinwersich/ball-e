#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

struct MotorDriverDRI0044 final : MotorDriver {
  MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> pwm_slice);

  void drive(float speed) override;

  uint pin_pwm, pin_direction;
  std::shared_ptr<PwmSlice> pwm_slice;
  uint pwm_channel;
};