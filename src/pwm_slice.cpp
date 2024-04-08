#include "pwm_slice.h"

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
  uint32_t f_sys = clock_get_hz(clk_sys);  // typically 125'000'000 Hz
  uint32_t f_clock = 1000000UL;
  float divider = (float)f_sys / f_clock;
  pwm_set_clkdiv(slice_num, divider);
  wrap = f_clock / frequency;
  pwm_set_wrap(slice_num, wrap - 1);  // -1 because counter starts at 0
}
