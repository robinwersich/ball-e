#include "motor_driver_kickstart.h"

#include <cmath>
#include <string>

KickstartMotorDriver::KickstartMotorDriver(
  std::unique_ptr<MotorDriver> driver, const Kickstart& kickstart
)
  : _driver(std::move(driver)), _kickstart(kickstart) {}

void KickstartMotorDriver::drive(float speed) {
  speed = std::abs(speed) < _kickstart.end_threshold ? 0 : speed;
  if (not _current_kickstart_id and std::abs(speed) < _kickstart.start_threshold and speed != 0
      and _current_speed == 0) {
    _current_speed = speed;
    _driver->drive(std::copysign(_kickstart.start_threshold, speed));
    _current_kickstart_id = add_alarm_in_ms(
      _kickstart.duration_ms,
      [](alarm_id_t alarm_id, void* driver_ptr) -> int64_t {
        static_cast<KickstartMotorDriver*>(driver_ptr)->drive_at_set_speed();
        return 0;
      },
      this, true
    );
  }
  _current_speed = speed;
  if (not _current_kickstart_id) _driver->drive(speed);
}

void KickstartMotorDriver::drive_at_set_speed() {
  _driver->drive(_current_speed);
  _current_kickstart_id = 0;
}
