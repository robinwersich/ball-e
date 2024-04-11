#include "pwm_slice.h"

#include <cmath>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

PwmSlice::PwmSlice(uint slice_num, uint frequency) : slice_num{slice_num} {
  set_frequency(frequency);
  pwm_set_enabled(slice_num, true);
}

PwmSlice::~PwmSlice() { pwm_set_enabled(slice_num, false); }

std::shared_ptr<PwmSlice> PwmSlice::forPin(uint pin, uint frequency) {
  return std::make_shared<PwmSlice>(pwm_gpio_to_slice_num(pin), frequency);
}

void PwmSlice::set_frequency(uint frequency) {
  const uint32_t f_sys = clock_get_hz(clk_sys);  // typically 125'000'000 Hz
  period = f_sys / frequency;
  if (period > 0xffff) {
    // Period is too large to fit in 16bit int, need to reduce the frequency
    const uint32_t f_target = frequency * 0xffff;
    // make sure the clock frequency is low enough for the period to fit in 16bit int
    uint8_t divider = std::ceil(f_sys / f_target);
    pwm_set_clkdiv_int_frac(slice_num, divider, 0);
    period = f_sys / divider / frequency;
  }
  pwm_set_wrap(slice_num, period - 1);  // -1 because counter starts at 0
}
