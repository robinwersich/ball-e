#pragma once

#include <Eigen/Core>

#include "hardware/i2c.h"
#include "i2c_device.h"
#include "imu_calibration.h"
#include "pico/types.h"

namespace lis3 {

struct ODR {
  float frequency;
  uint8_t code;
};
struct MagFS {
  float range;
  uint8_t code;
};

/** Output data rate. */
namespace odr {
constexpr ODR HZ_0_625 = {0.625, 0b000000};
constexpr ODR HZ_1_25 = {1.25, 0b000010};
constexpr ODR HZ_2_5 = {2.5, 0b000100};
constexpr ODR HZ_5 = {5, 0b000110};
constexpr ODR HZ_10 = {10, 0b001000};
constexpr ODR HZ_20 = {20, 0b001010};
constexpr ODR HZ_40 = {40, 0b001100};
constexpr ODR HZ_80 = {80, 0b001110};
constexpr ODR HZ_155 = {155, 0b110001};
constexpr ODR HZ_300 = {300, 0b100001};
constexpr ODR HZ_560 = {560, 0b010001};
constexpr ODR HZ_1000 = {1000, 0b000001};
}

/** Full scale range for magenetometer. */
namespace fs {
constexpr MagFS GS_4 = {4, 0b00};
constexpr MagFS GS_8 = {8, 0b01};
constexpr MagFS GS_12 = {12, 0b10};
constexpr MagFS GS_16 = {16, 0b11};
}

}

class LIS3 : I2CDevice {
 public:
  struct MagnetometerConfig {
    /** Magnetormeter output data rate. */
    lis3::ODR odr = lis3::odr::HZ_10;
    /** Range of the magnetic field strength. */
    lis3::MagFS fs = lis3::fs::GS_4;
  };

  /**
   * Sets up the IC2 connection to the LIS3MDL sensor and initializes it with the given config.
   * As a result, only one instance of this class should be created per physical sensor.
   * It is recommended to use a shared_ptr.
   * @param i2c_port The initialized I2C port to use for communication.
   * @param config The configuration for the magnetometer
   * @param sa0 The state of the SA0 pin. If true, the address is 0b0011110, otherwise 0b0011100.
   * @param calibration The calibration values for the sensor (in units of g and dps).
   */
  LIS3(
    i2c_inst_t* i2c_port, const MagnetometerConfig& config, bool sa0 = true,
    const LIS3Calibration& calibration = {},
    const Eigen::Matrix2f& orientation = Eigen::Matrix2f::Identity()
  );
  /** Turns off the sensor. */
  ~LIS3();

  LIS3(const LIS3&) = delete;
  LIS3& operator=(const LIS3&) = delete;

  /** Checks if the device ID can be read via the I2C interface. */
  bool is_connected() const;

  /** Returns true if any of the specified data is newly available. */
  bool is_new_data_available() const;

  /** Returns the raw value of the current magnetic field. */
  Eigen::Vector2<int16_t> read_field_raw() const;
  /** Returns the current magnetic field in gauss. */
  Eigen::Vector2f read_field() const;

  /** Returns the duration between two measurements. */
  uint64_t period_us() const { return _period_us; }

 private:
  LIS3Calibration _calibration;  // includes scaling, bias and orientation
  float _mag_scale;
  uint64_t _period_us;
};
