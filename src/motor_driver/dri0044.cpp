#include "motor_driver/dri0044.h"

#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverDRI0044::MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> slice)
  : pin_pwm{pwm}
  , pin_direction{direction}
  , pwm_slice{std::make_shared<PwmSlice>(pwm_gpio_to_slice_num(pin_pwm), 1000)}
  , pwm_channel{pwm_gpio_to_channel(pin_pwm)} {
  gpio_init(pin_pwm);
  gpio_init(pin_direction);
  gpio_set_dir(pin_pwm, GPIO_OUT);
  gpio_set_dir(pin_direction, GPIO_OUT);
  gpio_set_function(pin_pwm, GPIO_FUNC_PWM);
  set_duty_percent(0);
}

void MotorDriverDRI0044::set_duty_percent(uint duty) {
  uint16_t level = pwm_slice->wrap * duty / 100;
  pwm_set_chan_level(pwm_slice->slice_num, pwm_channel, level - 1); // -1 because count starts at 0
}

void MotorDriverDRI0044::forward() {
  gpio_put(pin_direction, 1);
}

void MotorDriverDRI0044::backward() {
  gpio_put(pin_direction, 0);
}

void MotorDriverDRI0044::stop() {
  set_duty_percent(0);
}
