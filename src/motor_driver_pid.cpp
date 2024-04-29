#include "motor_driver_pid.h"

MotorDriverPid::MotorDriverPid(
  std::shared_ptr<MotorDriver> driver, const MotorDecoderState* decoder_state, MotorSpec motor_spec,
  PidGains pid_gains, const char* name
)
  : _driver{std::move(driver)}
  , _max_speed{motor_spec.max_rpm}
  , _controller{-1.0, 1.0, MotorDriverPid::sample_time_millis, pid_gains, name}
  , _motor_state{decoder_state, motor_spec.ticks_per_revolution, motor_spec.gear_ratio} {
  add_repeating_timer_ms(
    MotorDriverPid::sample_time_millis,
    [](repeating_timer_t* timer) {
      auto driver = static_cast<MotorDriverPid*>(timer->user_data);
      driver->update_controller();
      return true;
    },
    this, &this->_update_timer
  );
}

MotorDriverPid::~MotorDriverPid() { cancel_repeating_timer(&_update_timer); }

void MotorDriverPid::drive_rpm(float rpm) { _controller.set_target(rpm); }
void MotorDriverPid::drive(float speed) { drive_rpm(speed * _max_speed); }

PidController& MotorDriverPid::controller() { return _controller; }

void MotorDriverPid::update_controller() {
  const auto rpm = _motor_state.compute_speed_rpm();
  const auto output = _controller.compute_at_sample_time(rpm);
  _driver->drive(output);
}