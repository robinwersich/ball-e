#include "motor_driver_kickstart.h"

KickstartMotorDriver::KickstartMotorDriver(
  std::shared_ptr<MotorDriver> driver, const Kickstart& kickstart
)
  : _driver(std::move(driver)), _kickstart(kickstart) {}

void KickstartMotorDriver::drive(float speed) {
  if (speed < _kickstart.start_threshold and speed >= _kickstart.end_threshold
      and _current_speed == 0) {
    _current_speed = speed;
    _driver->drive(_kickstart.start_threshold);
    add_alarm_in_ms(
      _kickstart.duration_ms,
      [](alarm_id_t alarm_id, void* driver) -> int64_t {
        static_cast<KickstartMotorDriver*>(driver)->drive_at_set_speed();
        return 0;
      },
      this, true
    );
  } else {
    _current_speed = speed;
    _driver->drive(speed < _kickstart.end_threshold ? 0 : speed);
  }
}

void KickstartMotorDriver::drive_at_set_speed() { _driver->drive(_current_speed); }
