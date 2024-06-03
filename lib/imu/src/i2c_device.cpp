#include "i2c_device.h"

#include <vector>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

i2c_inst_t* init_i2c_port(uint8_t slot, uint baudrate) {
  i2c_inst_t* i2c_port = slot % 2 == 0 ? i2c0 : i2c1;
  const auto pin_sda = 2 * slot;
  const auto pin_scl = 2 * slot + 1;

  i2c_init(i2c_port, baudrate);
  gpio_set_function(pin_sda, GPIO_FUNC_I2C);
  gpio_set_function(pin_scl, GPIO_FUNC_I2C);
  return i2c_port;
}

void deinit_i2c_port(i2c_inst_t* i2c_port) { i2c_deinit(i2c_port); }

I2CDevice::I2CDevice(i2c_inst_t* i2c_port, uint8_t address)
  : _i2c_port{i2c_port}, _address{static_cast<uint8_t>(address)} {}

void I2CDevice::read(uint8_t reg, uint8_t* data, size_t len) const {
  i2c_write_blocking(_i2c_port, _address, &reg, 1, true);
  i2c_read_blocking(_i2c_port, _address, data, len, false);
}

void I2CDevice::write(uint8_t reg, const uint8_t* data, size_t len) const {
  std::vector<uint8_t> buffer(1 + len);
  buffer[0] = reg;
  std::copy(data, data + len, buffer.begin() + 1);
  i2c_write_blocking(_i2c_port, _address, buffer.data(), buffer.size(), false);
}
