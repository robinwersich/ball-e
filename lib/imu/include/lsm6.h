#pragma once

#include "hardware/i2c.h"
#include "matrix.h"
#include "pico/types.h"

class LSM6 {
 public:
  /** Output Data Rate */
  enum class ODR : uint8_t {
    POWER_DOWN = 0b0000,
    HZ_1_6 = 0b1011,  // low power only
    HZ_12_5 = 0b0001,
    HZ_26 = 0b0010,
    HZ_52 = 0b0011,
    HZ_104 = 0b0100,
    HZ_208 = 0b0101,
    HZ_416 = 0b0110,
    HZ_833 = 0b0111,
    KHZ_1_66 = 0b1000,
    KHZ_3_33 = 0b1001,
    KHZ_6_66 = 0b1010,
  };

  struct AccelConfig {
    /** Full scale (range) of the acceleration. */
    enum class FS : uint8_t {
      G_2 = 0b00,
      G_4 = 0b10,
      G_8 = 0b11,
      G_16 = 0b01,
    };

    /** Acceleration output data rate. */
    ODR odr = ODR::HZ_104;
    /** Range of the acceleration. */
    FS fs = FS::G_2;
    /** Enable low-pass filter for acceleration. */
    bool low_pass = false;
  };

  struct GyroConfig {
    /** Full scale (range) of the angular velocity. */
    enum class FS : uint8_t {
      DPS_125 = 0b001,
      DPS_250 = 0b000,
      DPS_500 = 0b010,
      DPS_1000 = 0b100,
      DPS_2000 = 0b110,
    };

    /** Angular velocity output data rate. */
    ODR odr = ODR::HZ_104;
    /** Range of the angular velocity. */
    FS fs = FS::DPS_250;
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
  Vector3D<int16_t> read_acceleration_raw() const;
  /** Returns the current linear acceleration in g. */
  Vector3D<float> read_acceleration() const;

  /** Returns the raw value of the current angular velocity. */
  Vector3D<int16_t> read_rotation_raw() const;
  /** Returns the current angular velocity in dps. */
  Vector3D<float> read_rotation() const;

 private:
  i2c_inst_t* _i2c_port;
  uint8_t _address;

  void read(uint8_t reg, uint8_t* data, size_t len) const;
  void write(uint8_t reg, const uint8_t* data, size_t len) const;
};
