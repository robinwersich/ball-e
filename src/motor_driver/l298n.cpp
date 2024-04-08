#include "motor_driver/l298n.h"

#include <memory>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

MotorDriverL298N::MotorDriverL298N(
  uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> slice, uint duty
)
  : pin_enable{enable}
  , pin_in1{in1}
  , pin_in2{in2}
  , slice{std::make_shared<PwmSlice>(pwm_gpio_to_slice_num(pin_enable), 1000)}
  , channel{pwm_gpio_to_channel(pin_enable)} {
  gpio_init(pin_enable);
  gpio_init(pin_in1);
  gpio_init(pin_in2);
  gpio_set_dir(pin_enable, GPIO_OUT);
  gpio_set_dir(pin_in1, GPIO_OUT);
  gpio_set_dir(pin_in2, GPIO_OUT);
  gpio_set_function(pin_enable, GPIO_FUNC_PWM);
  set_duty(duty);
  stop();
}

void MotorDriverL298N::set_duty(uint duty) {
  uint16_t level = slice->wrap * duty / 100;
  pwm_set_chan_level(slice->slice_num, channel, level - 1);  // -1 because counter starts at 0
}

void MotorDriverL298N::forward() {
  gpio_put(pin_in1, 1);
  gpio_put(pin_in2, 0);
}

void MotorDriverL298N::backward() {
  gpio_put(pin_in1, 0);
  gpio_put(pin_in2, 1);
}

void MotorDriverL298N::stop() {
  gpio_put(pin_in1, 0);
  gpio_put(pin_in2, 0);
}
