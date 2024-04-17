#include "motor_drivers/l298n.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverL298N::MotorDriverL298N(uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> slice)
  : pin_enable{enable}
  , pin_in1{in1}
  , pin_in2{in2}
  , pwm_slice{slice}
  , pwm_channel{pwm_gpio_to_channel(pin_enable)} {
  gpio_init(pin_enable);
  gpio_init(pin_in1);
  gpio_init(pin_in2);
  gpio_set_dir(pin_enable, GPIO_OUT);
  gpio_set_dir(pin_in1, GPIO_OUT);
  gpio_set_dir(pin_in2, GPIO_OUT);
  gpio_set_function(pin_enable, GPIO_FUNC_PWM);
  stop();
}

MotorDriverL298N::MotorDriverL298N(uint enable, uint in1, uint in2, uint pwm_frequency)
  : MotorDriverL298N(enable, in1, in2, PwmSlice::for_pin(enable, pwm_frequency)) {}

void MotorDriverL298N::drive(float speed) const {
  speed = std::clamp(speed, -1.0f, 1.0f);

  gpio_put(pin_in1, speed > 0);
  gpio_put(pin_in2, speed < 0);
  uint16_t level = pwm_slice->period * std::abs(speed);
  pwm_set_chan_level(pwm_slice->slice_num, pwm_channel, level);
}
