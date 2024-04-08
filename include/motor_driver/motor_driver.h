#include "pico/types.h"

/**
 * Interface for motor drivers
 */
struct MotorDriver {
  virtual ~MotorDriver() = default;
  virtual void set_duty_percent(uint duty) = 0;
  virtual void forward() = 0;
  virtual void backward() = 0;
  virtual void stop() = 0;
};