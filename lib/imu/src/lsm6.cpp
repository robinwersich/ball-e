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
  CTRL4_C = 0x13,
  CTRL6_C = 0x15,
  CTRL8_XL = 0x17,
  STATUS_REG = 0x1E,
  GYRO_OUT = 0x22,
  ACCEL_OUT = 0x28,
};

}

LSM6::LSM6(
  uint8_t slot, const AccelConfig& accel_config, const GyroConfig& gyro_config, bool sa0,
  const ImuCalibration& calibration, const Eigen::Matrix3f& orientation
)
  : _i2c_port{slot % 2 == 0 ? i2c0 : i2c1}
  , _address{static_cast<uint8_t>(I2C_ADDRESS | sa0)}
  , _calibration{calibration}
  , _period_us{static_cast<uint64_t>(
      std::round(1e6f / std::max(accel_config.odr.frequency, gyro_config.odr.frequency))
    )} {
  // add scaling and orientation to the calibration
  const auto accel_scale = accel_config.fs.range / std::numeric_limits<int16_t>::max();
  const auto gyro_scale = gyro_config.fs.range / std::numeric_limits<int16_t>::max();
  // bias is applied before scaling
  _calibration.accel_bias /= accel_scale;
  _calibration.gyro_bias /= gyro_scale;
  _calibration.accel_transform = orientation * accel_scale * _calibration.accel_transform;
  _calibration.gyro_transform = orientation * gyro_scale * _calibration.gyro_transform;

  const auto pin_sda = 2 * slot;
  const auto pin_scl = 2 * slot + 1;

  i2c_init(_i2c_port, BAUDRATE);
  gpio_set_function(pin_sda, GPIO_FUNC_I2C);
  gpio_set_function(pin_scl, GPIO_FUNC_I2C);

  const uint8_t CTRL1_XL_val = static_cast<uint8_t>(accel_config.odr.code << 4)
                             | static_cast<uint8_t>(accel_config.fs.code << 2)
                             | static_cast<uint8_t>((accel_config.lp_cutoff & 0b1000) >> 2);
  const uint8_t CTRL2_G_val = static_cast<uint8_t>(gyro_config.odr.code << 4)
                            | static_cast<uint8_t>(gyro_config.fs.code << 1);
  const uint8_t CTRL4_C_val = static_cast<uint8_t>((gyro_config.lp_intensity & 0b1000) >> 2);
  const uint8_t CTRL6_C_val = static_cast<uint8_t>(gyro_config.lp_intensity & 0b0111);
  const uint8_t CTRL8_XL_val = static_cast<uint8_t>((accel_config.lp_cutoff & 0b0111) << 5);

  sleep_ms(100);
  write(CTRL1_XL, &CTRL1_XL_val, 1);
  write(CTRL2_G, &CTRL2_G_val, 1);
  write(CTRL4_C, &CTRL4_C_val, 1);
  write(CTRL6_C, &CTRL6_C_val, 1);
  write(CTRL8_XL, &CTRL8_XL_val, 1);
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

Eigen::Vector3f LSM6::read_acceleration() const {
  const auto& bias = _calibration.accel_bias;
  const auto& transform = _calibration.accel_transform;
  return transform * (read_acceleration_raw().cast<float>() - bias);
}

Eigen::Vector3f LSM6::read_rotation() const {
  const auto& bias = _calibration.gyro_bias;
  const auto& transform = _calibration.gyro_transform;
  return transform * (read_rotation_raw().cast<float>() - bias);
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
