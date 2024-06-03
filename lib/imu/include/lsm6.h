#pragma once

#include <Eigen/Core>

#include "hardware/i2c.h"
#include "i2c_device.h"
#include "imu_calibration.h"
#include "pico/types.h"

namespace lsm6 {

struct ODR {
  float frequency;
  uint8_t code;
};
struct AccFS {
  float range;
  uint8_t code;
};
struct GyroFS {
  float range;
  uint8_t code;
};

/** Output data rate. */
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

/** Full scale range for accelerometer. */
namespace fs::acc {
constexpr AccFS G_2 = {2, 0b00};
constexpr AccFS G_4 = {4, 0b10};
constexpr AccFS G_8 = {8, 0b11};
constexpr AccFS G_16 = {16, 0b01};
}

/** Full scale range for gyroscope. */
namespace fs::gyro {
constexpr GyroFS DPS_125 = {125, 0b001};
constexpr GyroFS DPS_250 = {250, 0b000};
constexpr GyroFS DPS_500 = {500, 0b010};
constexpr GyroFS DPS_1000 = {1000, 0b100};
}

/** Accelerometer LP filter cutoff frequency fraction (1/x) */
namespace cutoff::accel {
constexpr uint8_t CO_2 = 0b0000;
constexpr uint8_t CO_4 = 0b1000;
constexpr uint8_t CO_10 = 0b1001;
constexpr uint8_t CO_20 = 0b1010;
constexpr uint8_t CO_45 = 0b1011;
constexpr uint8_t CO_100 = 0b1100;
constexpr uint8_t CO_200 = 0b1101;
constexpr uint8_t CO_400 = 0b1110;
constexpr uint8_t CO_800 = 0b1111;
}

/** Gyroscope LP filter intensity (exact values can be found in manual) */
namespace cutoff::gyro {
constexpr uint8_t CO_OFF = 0b0000;
constexpr uint8_t CO_0 = 0b1000;
constexpr uint8_t CO_1 = 0b1001;
constexpr uint8_t CO_2 = 0b1010;
constexpr uint8_t CO_3 = 0b1011;
constexpr uint8_t CO_4 = 0b1100;
constexpr uint8_t CO_5 = 0b1101;
constexpr uint8_t CO_6 = 0b1110;
constexpr uint8_t CO_7 = 0b1111;
}

}

class LSM6 : I2CDevice {
 public:
  struct AccelConfig {
    /** Acceleration output data rate. */
    lsm6::ODR odr = lsm6::odr::HZ_104;
    /** Range of the acceleration. */
    lsm6::AccFS fs = lsm6::fs::acc::G_2;
    /** Cutoff frequency for the lowpass filter. */
    uint8_t lp_cutoff = lsm6::cutoff::accel::CO_2;
  };

  struct GyroConfig {
    /** Angular velocity output data rate. */
    lsm6::ODR odr = lsm6::odr::HZ_104;
    /** Range of the angular velocity. */
    lsm6::GyroFS fs = lsm6::fs::gyro::DPS_250;
    /** LP filter intensity. */
    uint8_t lp_intensity = lsm6::cutoff::gyro::CO_OFF;
  };

  const static uint MAX_BAUDRATE = 400000;  // 400kHz

  /**
   * Sets up the IC2 connection to the LSM6DSO sensor and initializes it with the given config.
   * As a result, only one instance of this class should be created per physical sensor.
   * It is recommended to use a shared_ptr.
   * @param i2c_port The initialized I2C port to use for communication.
   * @param config The configuration for the accelerometer.
   * @param gyro_config The configuration for the gyroscope.
   * @param sa0 The state of the SA0 pin. If true, the address is 0b1101011, otherwise 0b1101010.
   * @param calibration The calibration values for the sensor (in units of g and dps).
   */
  LSM6(
    i2c_inst_t* i2c_port, const AccelConfig& config, const GyroConfig& gyro_config, bool sa0 = true,
    const LSM6Calibration& calibration = {},
    const Eigen::Matrix3f& orientation = Eigen::Matrix3f::Identity()
  );
  /** Turns off the LSM6DSO sensor. */
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

  /** Returns the duration between two measurements. */
  uint64_t period_us() const { return _period_us; }

 private:
  LSM6Calibration _calibration;  // includes scaling, bias and orientation
  uint64_t _period_us;
};
