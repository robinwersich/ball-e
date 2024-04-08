#include <memory>

#include "motor_driver.h"
#include "pico/types.h"
#include "pwm_slice.h"

struct MotorDriverL298N final : MotorDriver {
  MotorDriverL298N(uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> slice, uint duty = 0);

  void set_duty(uint duty) override;
  void forward() override;
  void backward() override;
  void stop() override;

  uint pin_enable, pin_in1, pin_in2;
  std::shared_ptr<PwmSlice> slice;
  uint channel;
};