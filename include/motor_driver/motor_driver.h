#include "pico/types.h"

/**
 * Interface for motor drivers
 */
struct MotorDriver {
  virtual ~MotorDriver() = default;
  
  /**
   * Makes the motor move at a specified speed and direction.
   * @param The desired speed as a number between 1 (forward) and -1 (backward).
   */
  virtual void drive(float speed) = 0;

  /** Convenience method for stopping the motor. */
  inline void stop() { drive(0); }
};