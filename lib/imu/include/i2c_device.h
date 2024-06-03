#pragma once

#include <memory>

#include "hardware/i2c.h"
#include "pico/types.h"

/** Initializes the I2C port and returns a pointer to it. */
i2c_inst_t* init_i2c_port(uint8_t slot, uint baudrate);
/** Deinitializes the I2C port. */
void deinit_i2c_port(i2c_inst_t* i2c_port);

class I2CDevice {
 public:
 /**
   * Sets up the IC2 connection. Only one instance of this class should be created per device.
   * It is recommended to use a shared_ptr.
   * @param i2c_port The initialized I2C port to use for communication.
   * @param address The 7-bit I2C address of the device.
   */
  I2CDevice(i2c_inst_t* i2c_port, uint8_t address);

 protected:
  void read(uint8_t reg, uint8_t* data, size_t len) const;
  void write(uint8_t reg, const uint8_t* data, size_t len) const;

 private:
  i2c_inst_t* _i2c_port;
  uint8_t _address;
};