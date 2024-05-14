#include "lsm6.h"

#include <vector>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

const static uint8_t I2C_ADDRESS = 0b1101010;  // last bit has to be set to sa0
const static uint BAUDRATE = 400000;           // 400kHz

namespace {

enum Register : uint8_t {
  WHO_AM_I = 0x0F,
  CTRL1_XL = 0x10,
  CTRL2_G = 0x11,
  STATUS_REG = 0x1E,
  GYRO_OUT = 0x22,
  ACCEL_OUT = 0x28,
};

}

LSM6::LSM6(uint8_t slot, const AccelConfig& accel_config, const GyroConfig& gyro_config, bool sa0)
  : _i2c_port{slot % 2 == 0 ? i2c0 : i2c1}, _address{static_cast<uint8_t>(I2C_ADDRESS | sa0)} {
  const auto pin_sda = 2 * slot;
  const auto pin_scl = 2 * slot + 1;

  i2c_init(_i2c_port, BAUDRATE);
  gpio_set_function(pin_sda, GPIO_FUNC_I2C);
  gpio_set_function(pin_scl, GPIO_FUNC_I2C);

  const uint8_t CTRL1_XL_val = static_cast<uint8_t>(accel_config.odr) << 4
                             | static_cast<uint8_t>(accel_config.fs) << 2
                             | (accel_config.low_pass ? 1 : 0) << 1;
  const uint8_t CTRL2_G_val =
    static_cast<uint8_t>(gyro_config.odr) << 4 | static_cast<uint8_t>(gyro_config.fs) << 1;

  sleep_ms(100);
  write(CTRL1_XL, &CTRL1_XL_val, 1);
  write(CTRL2_G, &CTRL2_G_val, 1);
  sleep_ms(100);
}

LSM6::~LSM6() {
  uint8_t null = 0;
  write(CTRL1_XL, &null, 1);
  write(CTRL2_G, &null, 1);
  i2c_deinit(_i2c_port);
}

bool LSM6::is_connected() const {
  uint8_t who_am_i;
  read(WHO_AM_I, &who_am_i, 1);
  return who_am_i == 0x6C;
}

bool LSM6::is_new_data_available(bool acceleration, bool rotation) const {
  uint8_t status;
  read(STATUS_REG, &status, 1);
  return (acceleration && (status & 0x01)) || (rotation && (status & 0x02));
}

Eigen::Vector3<int16_t> LSM6::read_acceleration_raw() const {
  uint8_t data[6];
  read(ACCEL_OUT, data, 6);

  return {
    static_cast<int16_t>((data[1] << 8) | data[0]),
    static_cast<int16_t>((data[3] << 8) | data[2]),
    static_cast<int16_t>((data[5] << 8) | data[4]),
  };
}

Eigen::Vector3<int16_t> LSM6::read_rotation_raw() const {
  uint8_t data[6];
  read(GYRO_OUT, data, 6);

  return {
    static_cast<int16_t>((data[1] << 8) | data[0]),
    static_cast<int16_t>((data[3] << 8) | data[2]),
    static_cast<int16_t>((data[5] << 8) | data[4]),
  };
}

void LSM6::read(uint8_t reg, uint8_t* data, size_t len) const {
  i2c_write_blocking(_i2c_port, _address, &reg, 1, true);
  i2c_read_blocking(_i2c_port, _address, data, len, false);
}

void LSM6::write(uint8_t reg, const uint8_t* data, size_t len) const {
  std::vector<uint8_t> buffer(1 + len);
  buffer[0] = reg;
  std::copy(data, data + len, buffer.begin() + 1);
  i2c_write_blocking(_i2c_port, _address, buffer.data(), buffer.size(), false);
}
