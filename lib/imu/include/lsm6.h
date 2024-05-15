#pragma once

#include <Eigen/Core>

#include "hardware/i2c.h"
#include "pico/types.h"

namespace lsm6 {

/** Output data rate. */
struct ODR {
  float frequency;
  uint8_t code;
};
/** Full scale range for accelerometer. */
struct AccFS {
  float range;
  uint8_t code;
};
/** Full scale range for gyroscope. */
struct GyroFS {
  float range;
  uint8_t code;
};

namespace odr {
constexpr ODR POWER_DOWN = {0, 0b0000};
constexpr ODR HZ_1_6 = {1, 0b1011};  // low power only
constexpr ODR HZ_12_5 = {12, 0b0001};
constexpr ODR HZ_26 = {26, 0b0010};
constexpr ODR HZ_52 = {52, 0b0011};
constexpr ODR HZ_104 = {104, 0b0100};
constexpr ODR HZ_208 = {208, 0b0101};
constexpr ODR HZ_416 = {416, 0b0110};
constexpr ODR HZ_833 = {833, 0b0111};
constexpr ODR KHZ_1_66 = {1666, 0b1000};
constexpr ODR KHZ_3_33 = {3333, 0b1001};
constexpr ODR KHZ_6_66 = {6666, 0b1010};
}

namespace fs::acc {
constexpr AccFS G_2 = {2, 0b00};
constexpr AccFS G_4 = {4, 0b10};
constexpr AccFS G_8 = {8, 0b11};
constexpr AccFS G_16 = {16, 0b01};
}

namespace fs::gyro {
constexpr GyroFS DPS_125 = {125, 0b001};
constexpr GyroFS DPS_250 = {250, 0b000};
constexpr GyroFS DPS_500 = {500, 0b010};
constexpr GyroFS DPS_1000 = {1000, 0b100};
}

}

class LSM6 {
 public:
  struct AccelConfig {
    /** Acceleration output data rate. */
    lsm6::ODR odr = lsm6::odr::HZ_104;
    /** Range of the acceleration. */
    lsm6::AccFS fs = lsm6::fs::acc::G_2;
    /** Enable low-pass filter for acceleration. */
    bool low_pass = false;
  };

  struct GyroConfig {
    /** Angular velocity output data rate. */
    lsm6::ODR odr = lsm6::odr::HZ_104;
    /** Range of the angular velocity. */
    lsm6::GyroFS fs = lsm6::fs::gyro::DPS_250;
  };

  /**
   * Sets up the IC2 connection to the LSM6DSO sensor and initializes it with the given config.
   * @param slot The I2C pin pair to use for communication. SDA=2*slot and SCL=2*slot+1.
   * @param config The configuration for the accelerometer.
   * @param gyro_config The configuration for the gyroscope.
   * @param sa0 The state of the SA0 pin. If true, the address is 0b1101011, otherwise 0b1101010.
   */
  LSM6(uint8_t slot, const AccelConfig& config, const GyroConfig& gyro_config, bool sa0 = true);
  /**
   * Turns off the LSM6DSO sensor and stops the I2C connection to the LSM6DSO sensor.
   */
  ~LSM6();

  LSM6(const LSM6&) = delete;
  LSM6& operator=(const LSM6&) = delete;

  /** Checks if the device ID can be read via the I2C interface. */
  bool is_connected() const;

  /** Returns true if any of the specified data is newly available. */
  bool is_new_data_available(bool acceleration = true, bool rotation = true) const;

  /** Returns the raw value of the current linear acceleration. */
  Eigen::Vector3<int16_t> read_acceleration_raw() const;
  /** Returns the current linear acceleration in g. */
  Eigen::Vector3f read_acceleration() const;

  /** Returns the raw value of the current angular velocity. */
  Eigen::Vector3<int16_t> read_rotation_raw() const;
  /**
   * Returns the current angular velocity in dps.
   * The vector can be interpreted as individual rotations about x, y and z,
   * but also as an axis-angle representation of the rotation.
   */
  Eigen::Vector3f read_rotation() const;

 private:
  i2c_inst_t* _i2c_port;
  uint8_t _address;
  float _accel_scale;
  float _gyro_scale;

  void read(uint8_t reg, uint8_t* data, size_t len) const;
  void write(uint8_t reg, const uint8_t* data, size_t len) const;
};
