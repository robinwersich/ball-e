#include "motor_driver_kickstart.h"

#include <cmath>

KickstartMotorDriver::KickstartMotorDriver(
  std::unique_ptr<MotorDriver> driver, const Kickstart& kickstart
)
  : _driver(std::move(driver)), _kickstart(kickstart) {}

void KickstartMotorDriver::drive(float speed) {
  if (not _current_kickstart_id and speed < _kickstart.start_threshold
      and speed >= _kickstart.end_threshold and _current_speed < _kickstart.end_threshold) {
    _current_speed = speed;
    _driver->drive(std::copysign(_kickstart.start_threshold, speed));
    _current_kickstart_id = add_alarm_in_ms(
      _kickstart.duration_ms,
      [](alarm_id_t alarm_id, void* driver_ptr) -> int64_t {
        auto& driver = *static_cast<KickstartMotorDriver*>(driver_ptr);
        driver.drive_at_set_speed();
        driver._current_kickstart_id = 0;
        return 0;
      },
      this, true
    );
  } else {
    _driver->drive(_current_speed = speed);
  }
}

void KickstartMotorDriver::drive_at_set_speed() { _driver->drive(_current_speed); }
