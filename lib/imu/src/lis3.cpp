#include "lis3.h"

#include <vector>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

const static uint8_t I2C_ADDRESS = 0b0011100;  // last bit has to be set to sa0
const static uint BAUDRATE = 400000;           // 400kHz

namespace {

enum Register : uint8_t {
  WHO_AM_I = 0x0F,
  CTRL_REG1 = 0x20,
  CTRL_REG2 = 0x21,
  CTRL_REG3 = 0x22,
  STATUS_REG = 0x27,
  MAG_OUT = 0x28,
};

}

LIS3::LIS3(
  i2c_inst_t* i2c_port, const MagnetometerConfig& config, bool sa0,
  const LIS3Calibration& calibration,
  const Eigen::Matrix2f& orientation
)
  : I2CDevice{i2c_port, static_cast<uint8_t>(I2C_ADDRESS | sa0 << 1)}
  , _calibration{calibration}
  , _mag_scale{config.fs.range / std::numeric_limits<int16_t>::max()}
  , _period_us{static_cast<uint64_t>(std::round(1e6f / config.odr.frequency))} {
  // add scaling and orientation to the calibration
  const auto mag_scale = config.fs.range / std::numeric_limits<int16_t>::max();
  // bias is applied before transformation
  _calibration.mag_bias /= mag_scale;
  _calibration.mag_transform = orientation * mag_scale * _calibration.mag_transform;

  const uint8_t CTRL_REG1_val = static_cast<uint8_t>(config.odr.code << 1);
  const uint8_t CTRL_REG2_val = static_cast<uint8_t>(config.fs.code << 5);
  const uint8_t CTRL_REG3_val = config.odr.frequency <= 80 ? 0x01 : 0x00;

  write(CTRL_REG1, &CTRL_REG1_val, 1);
  write(CTRL_REG2, &CTRL_REG2_val, 1);
  write(CTRL_REG3, &CTRL_REG3_val, 1);
}

LIS3::~LIS3() {
  uint8_t off = 0x03;
  write(CTRL_REG3, &off, 1);
}

bool LIS3::is_connected() const {
  uint8_t who_am_i;
  read(WHO_AM_I, &who_am_i, 1);
  return who_am_i == 0x3D;
}

bool LIS3::is_new_data_available() const {
  uint8_t status;
  read(STATUS_REG, &status, 1);
  return (status & 0x03);
}

Eigen::Vector2<int16_t> LIS3::read_field_raw() const {
  uint8_t data[4];
  read(MAG_OUT, data, 4);

  return {
    static_cast<int16_t>((data[1] << 8) | data[0]),
    static_cast<int16_t>((data[3] << 8) | data[2]),
  };
}

Eigen::Vector2f LIS3::read_field() const {
  const auto& bias = _calibration.mag_bias;
  const auto& transform = _calibration.mag_transform;
  return transform * (read_field_raw().cast<float>() - bias);
}
