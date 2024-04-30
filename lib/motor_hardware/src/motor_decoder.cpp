#include "motor_decoder.h"

#include "pico/stdlib.h"

MotorDecoder::MotorDecoder(uint8_t slot, bool swap_direction) : _slot{slot} {
  _states[slot].swap_direction = swap_direction;

  const uint pin_a = 2 * slot;
  const uint pin_b = 2 * slot + 1;
  gpio_init(pin_a);
  gpio_set_dir(pin_a, GPIO_IN);
  gpio_init(pin_b);
  gpio_set_dir(pin_b, GPIO_IN);

  gpio_set_irq_enabled_with_callback(
    pin_a, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &on_a_change
  );
}

MotorDecoder::~MotorDecoder() {
  const uint pin_a = 2 * _slot;
  gpio_set_irq_enabled(pin_a, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
}

const MotorDecoderState& MotorDecoder::state() const { return _states[_slot]; }

std::array<MotorDecoderState, 8> MotorDecoder::_states{};

void MotorDecoder::on_a_change(uint pin, uint32_t event) {
  const bool a_rising = event & GPIO_IRQ_EDGE_RISE;
  const uint8_t slot = pin / 2;
  auto& state = _states[slot];

  if (a_rising != gpio_get(pin + 1) xor state.swap_direction) {
    ++state.count;
  } else {
    --state.count;
  }
  if (a_rising) {
    state.syncronized_count = state.count;
    state.last_sync_micros = time_us_32();
  }
}
