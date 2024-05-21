#include "btcontrol.h"
#include "imu_calibration_values.h"
#include "motor_drivers/dri0044.h"
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "robot.h"

const uint PWM_FREQUENCY = 25000;
// -- motor 1 --
const uint DIR1 = 0;
const uint PWM1 = 1;
const uint ENC1_SLOT = 1;
// -- motor 2 --
const uint DIR2 = 6;
const uint PWM2 = 7;
const uint ENC2_SLOT = 2;
// -- motor 3 --
const uint PWM3 = 8;
const uint DIR3 = 9;
const uint ENC3_SLOT = 5;
// -- IMU --
const uint IMU_SLOT = 7;

std::unique_ptr<Robot> robot;

critical_section_t create_critical_section() {
  critical_section_t cs;
  critical_section_init(&cs);
  return cs;
}

void on_gamepad_data(const uni_gamepad_t& gamepad) {
  static critical_section_t cs = create_critical_section();
  static bool balance_pressed = false;

  if (gamepad.buttons & BUTTON_B and !balance_pressed) {
    robot->toggle_balancing();
    printf("Balancing mode %s\n", robot->is_balancing() ? "enabled" : "disabled");
  }
  balance_pressed = gamepad.buttons & BUTTON_B;

  const float speed_x = gamepad.axis_x / 512.0;
  const float speed_y = gamepad.axis_y / 512.0;
  const float speed_rot = (gamepad.throttle - gamepad.brake) / 1024.0;

  // avoid robot update while speed is set
  critical_section_enter_blocking(&cs);
  robot->drive(speed_x, speed_y);
  robot->rotate(-speed_rot);  // robot should spin clockwise when throttle is pressed
  critical_section_exit(&cs);
}

int main() {
  stdio_init_all();

  // configure hardware timers to have lower priority than the default
  irq_set_priority(TIMER_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY + 1);
  irq_set_priority(TIMER_IRQ_1, PICO_DEFAULT_IRQ_PRIORITY + 1);
  irq_set_priority(TIMER_IRQ_2, PICO_DEFAULT_IRQ_PRIORITY + 1);
  irq_set_priority(TIMER_IRQ_3, PICO_DEFAULT_IRQ_PRIORITY + 1);

  using namespace lsm6;
  LSM6::AccelConfig accel_config{.odr = odr::HZ_104, .fs = fs::acc::G_2, .low_pass = true};
  LSM6::GyroConfig gyro_config{.odr = odr::HZ_104, .fs = fs::gyro::DPS_1000};
  const auto imu = std::make_shared<LSM6>(
    7, accel_config, gyro_config, true, IMU_CALIBRATION,
    Eigen::Matrix3f{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
  );

  robot = std::make_unique<Robot>(
    std::array<Omniwheel, 3>{
      Omniwheel(30, std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY, true)),
      Omniwheel(150, std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY)),
      Omniwheel(270, std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY)),
    },
    OrientationEstimator{imu}, PidGains{0.0, 0.0, 0.0}  // TODO: tune gains
  );

  robot->start_updating();

  btcontrol::init();
  btcontrol::register_gampad_behavior(on_gamepad_data);
  btcontrol::run_loop();

  return 0;
}
