#include "motor_drivers/dri0044.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverDRI0044::MotorDriverDRI0044(uint pwm, uint direction, std::shared_ptr<PwmSlice> slice)
  : _pin_pwm{pwm}
  , _pin_direction{direction}
  , _pwm_slice{slice}
  , _pwm_channel{pwm_gpio_to_channel(_pin_pwm)} {
  gpio_init(_pin_pwm);
  gpio_init(_pin_direction);
  gpio_set_dir(_pin_pwm, GPIO_OUT);
  gpio_set_dir(_pin_direction, GPIO_OUT);
  gpio_set_function(_pin_pwm, GPIO_FUNC_PWM);
  stop();
}

MotorDriverDRI0044::MotorDriverDRI0044(uint pwm, uint direction, uint pwm_frequency)
  : MotorDriverDRI0044(pwm, direction, PwmSlice::for_pin(pwm, pwm_frequency)) {}

void MotorDriverDRI0044::drive(float speed) {
  speed = std::clamp(speed, -1.0f, 1.0f);

  gpio_put(_pin_direction, speed > 0);
  uint16_t level = _pwm_slice->period * std::abs(speed);
  pwm_set_chan_level(_pwm_slice->slice_num, _pwm_channel, level);
  _speed = speed;
}

void MotorDriverDRI0044::set_pwm_frequency(uint frequency) {
  _pwm_slice->set_frequency(frequency);
  drive(_speed);
}
