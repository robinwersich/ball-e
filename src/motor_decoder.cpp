#include "motor_decoder.h"

#include "interrupt_handler.h"
#include "pico/stdlib.h"

MotorDecoder::MotorDecoder(uint pin_a, uint pin_b) : _pin_a{pin_a}, _pin_b{pin_b} {
  gpio_init(_pin_a);
  gpio_set_dir(_pin_a, GPIO_IN);

  gpio_init(_pin_b);
  gpio_set_dir(_pin_b, GPIO_IN);

  interrupts::register_callback(_pin_a, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, [this] {
    this->on_a_change();
  });
}

MotorDecoder::~MotorDecoder() {
  interrupts::unregister_callback(_pin_a, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
}

std::shared_ptr<MotorDecoder> MotorDecoder::for_pins(uint pin_a, uint pin_b) {
  return std::make_shared<MotorDecoder>(pin_a, pin_b);
}

void MotorDecoder::on_a_change() {
  _last_count_micros = time_us_64();
  if (gpio_get(_pin_a) != gpio_get(_pin_b)) {
    _count++;
  } else {
    _count--;
  }
}
