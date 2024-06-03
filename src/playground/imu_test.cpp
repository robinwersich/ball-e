#include <stdio.h>

#include "imu_calibration_values.h"
#include "lsm6.h"
#include "pico/stdlib.h"
#include "plot.h"

int main() {
  stdio_init_all();
  sleep_ms(2000);
  using namespace lsm6;

  const LSM6::AccelConfig accel_config{.odr = odr::HZ_104, .fs = fs::acc::G_2};
  const LSM6::GyroConfig gyro_config{.odr = odr::HZ_104, .fs = fs::gyro::DPS_1000};
  const auto i2c_port = init_i2c_port(7, LSM6::MAX_BAUDRATE);
  const LSM6 imu(i2c_port, accel_config, gyro_config, true, IMU_CALIBRATION);
  if (not imu.is_connected()) {
    printf("IMU not connected\n");
    return 1;
  }

  while (true) {
    if (imu.is_new_data_available(true, true)) {
      const auto acc = imu.read_acceleration();
      const auto rot = imu.read_rotation();
      plot("acc_x", acc.x());
      plot("acc_y", acc.y());
      plot("acc_z", acc.z());
      // Display rotation in rps to make axis scales more similar.
      plot("rot_x", rot.x() / 360);
      plot("rot_y", rot.y() / 360);
      plot("rot_z", rot.z() / 360);
    }
  }
}