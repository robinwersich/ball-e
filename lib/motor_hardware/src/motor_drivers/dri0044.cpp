#include "motor_drivers/dri0044.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorDriverDRI0044::MotorDriverDRI0044(
  uint pwm_pin, uint direction_pin, std::shared_ptr<PwmSlice> slice, bool swap_direction
)
  : _pin_pwm{pwm_pin}
  , _pin_direction{direction_pin}
  , _pwm_slice{slice}
  , _pwm_channel{pwm_gpio_to_channel(_pin_pwm)}
  , _swap_direction{swap_direction} {
  gpio_init(_pin_pwm);
  gpio_init(_pin_direction);
  gpio_set_dir(_pin_pwm, GPIO_OUT);
  gpio_set_dir(_pin_direction, GPIO_OUT);
  gpio_set_function(_pin_pwm, GPIO_FUNC_PWM);
  stop();
}

MotorDriverDRI0044::MotorDriverDRI0044(
  uint pwm_pin, uint direction_pin, uint pwm_frequency, bool swap_direction
)
  : MotorDriverDRI0044(
    pwm_pin, direction_pin, PwmSlice::for_pin(pwm_pin, pwm_frequency), swap_direction
  ) {}

void MotorDriverDRI0044::drive(float speed) {
  speed = std::clamp(speed, -1.0f, 1.0f);

  gpio_put(_pin_direction, speed > 0 xor _swap_direction);
  uint16_t level = _pwm_slice->period * std::abs(speed);
  pwm_set_chan_level(_pwm_slice->slice_num, _pwm_channel, level);
  _speed = speed;
}

void MotorDriverDRI0044::set_pwm_frequency(uint frequency) {
  _pwm_slice->set_frequency(frequency);
  drive(_speed);
}
