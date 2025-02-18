#include "motor_drivers/l298n.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverL298N::MotorDriverL298N(
  uint enable, uint in1, uint in2, std::shared_ptr<PwmSlice> slice, bool swap_direction
)
  : _pin_enable{enable}
  , _pin_in1{in1}
  , _pin_in2{in2}
  , _pwm_slice{slice}
  , _pwm_channel{pwm_gpio_to_channel(_pin_enable)}
  , _swap_direction{swap_direction} {
  gpio_init(_pin_enable);
  gpio_init(_pin_in1);
  gpio_init(_pin_in2);
  gpio_set_dir(_pin_enable, GPIO_OUT);
  gpio_set_dir(_pin_in1, GPIO_OUT);
  gpio_set_dir(_pin_in2, GPIO_OUT);
  gpio_set_function(_pin_enable, GPIO_FUNC_PWM);
  stop();
}

MotorDriverL298N::MotorDriverL298N(
  uint enable, uint in1, uint in2, uint pwm_frequency, bool swap_direction
)
  : MotorDriverL298N(enable, in1, in2, PwmSlice::for_pin(enable, pwm_frequency), swap_direction) {}

void MotorDriverL298N::drive(float speed) {
  speed = std::clamp(speed, -1.0f, 1.0f);

  gpio_put(_pin_in1, speed > 0 xor _swap_direction);
  gpio_put(_pin_in2, speed < 0 xor _swap_direction);
  uint16_t level = _pwm_slice->period * std::abs(speed);
  pwm_set_chan_level(_pwm_slice->slice_num, _pwm_channel, level);
  _speed = speed;
}

void MotorDriverL298N::set_pwm_frequency(uint frequency) {
  _pwm_slice->set_frequency(frequency);
  drive(_speed);
}
