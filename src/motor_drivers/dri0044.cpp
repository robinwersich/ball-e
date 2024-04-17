#include "motor_drivers/dri0044.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverDRI0044::MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> slice)
  : pin_pwm{pwm}
  , pin_direction{direction}
  , pwm_slice{slice}
  , pwm_channel{pwm_gpio_to_channel(pin_pwm)} {
  gpio_init(pin_pwm);
  gpio_init(pin_direction);
  gpio_set_dir(pin_pwm, GPIO_OUT);
  gpio_set_dir(pin_direction, GPIO_OUT);
  gpio_set_function(pin_pwm, GPIO_FUNC_PWM);
  stop();
}

MotorDriverDRI0044::MotorDriverDRI0044(uint pwm, uint direction, uint pwm_frequency)
  : MotorDriverDRI0044(pwm, direction, PwmSlice::for_pin(pwm, pwm_frequency)) {}

void MotorDriverDRI0044::drive(float speed) const {
  speed = std::clamp(speed, -1.0f, 1.0f);

  gpio_put(pin_direction, speed > 0);
  uint16_t level = pwm_slice->period * std::abs(speed);
  pwm_set_chan_level(pwm_slice->slice_num, pwm_channel, level);
}
