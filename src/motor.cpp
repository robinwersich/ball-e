#include "motor.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

Slice::Slice(uint slice_num, uint frequency) : slice_num(slice_num) {
  set_frequency(frequency);
  pwm_set_enabled(slice_num, true);
}

Slice::~Slice() { pwm_set_enabled(slice_num, false); }

Slice Slice::forPin(uint pin, uint frequency) {
  return Slice(pwm_gpio_to_slice_num(pin), frequency);
}

void Slice::set_frequency(uint frequency) {
  uint32_t f_sys = clock_get_hz(clk_sys);  // typically 125'000'000 Hz
  uint32_t f_clock = 1000000UL;
  float divider = (float)f_sys / f_clock;
  pwm_set_clkdiv(slice_num, divider);
  wrap = f_clock / frequency;
  pwm_set_wrap(slice_num, wrap - 1);  // -1 because counter starts at 0
}

Motor::Motor(uint enable, uint in1, uint in2, Slice slice, uint duty)
  : pin_enable(enable)
  , pin_in1(in1)
  , pin_in2(in2)
  , slice(pwm_gpio_to_slice_num(pin_enable))
  , channel(pwm_gpio_to_channel(pin_enable)) {
  gpio_init(pin_enable);
  gpio_init(pin_in1);
  gpio_init(pin_in2);
  gpio_set_function(pin_enable, GPIO_FUNC_PWM);
  gpio_set_dir(pin_in1, GPIO_OUT);
  gpio_set_dir(pin_in2, GPIO_OUT);
  set_duty(duty);
  stop();
}

void Motor::set_duty(uint duty) {
  uint16_t level = slice.wrap * duty / 100;
  pwm_set_chan_level(slice.slice_num, channel, level - 1);  // -1 because counter starts at 0
}

void Motor::forward() {
  gpio_put(pin_in1, 1);
  gpio_put(pin_in2, 0);
}

void Motor::backward() {
  gpio_put(pin_in1, 0);
  gpio_put(pin_in2, 1);
}

void Motor::stop() {
  gpio_put(pin_in1, 0);
  gpio_put(pin_in2, 0);
}
