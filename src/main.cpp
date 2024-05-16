#include "btcontrol.h"
#include "motor_drivers/dri0044.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
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

void on_gamepad_data(const uni_gamepad_t& gamepad) {
  static bool balance_pressed = false;
  if (gamepad.buttons & BUTTON_B and !balance_pressed) {
    robot->toggle_balancing();
    printf("Balancing mode %s\n", robot->is_balancing() ? "enabled" : "disabled");
  }
  balance_pressed = gamepad.buttons & BUTTON_B;

  const float speed_x = gamepad.axis_x / 512.0;
  const float speed_y = gamepad.axis_y / 512.0;
  const float speed_rot = (gamepad.throttle - gamepad.brake) / 1024.0;
  robot->drive(speed_x, speed_y);
  robot->rotate(-speed_rot);  // robot should spin clockwise when throttle is pressed
}

void run_bluetooth_loop() {
  btcontrol::init();
  btcontrol::register_gampad_behavior(on_gamepad_data);
  btcontrol::run_loop();
}

int main() {
  stdio_init_all();

  LSM6::AccelConfig accel_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::acc::G_2};
  LSM6::GyroConfig gyro_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::gyro::DPS_1000};
  const auto imu = std::make_shared<LSM6>(7, accel_config, gyro_config);

  robot = std::make_unique<Robot>(
    std::array<Omniwheel, 3>{
      Omniwheel(30, std::make_unique<MotorDriverDRI0044>(PWM1, DIR1, PWM_FREQUENCY, true)),
      Omniwheel(150, std::make_unique<MotorDriverDRI0044>(PWM2, DIR2, PWM_FREQUENCY)),
      Omniwheel(270, std::make_unique<MotorDriverDRI0044>(PWM3, DIR3, PWM_FREQUENCY)),
    },
    OrientationEstimator{imu, Eigen::Matrix3f{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}},
    PidGains{0.0, 0.0, 0.0}  // TODO: tune gains
  );

  multicore_launch_core1(run_bluetooth_loop);
  robot->run_control_loop();

  return 0;
}
