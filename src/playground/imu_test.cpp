#include <stdio.h>

#include "imu_calibration_values.h"
#include "lis3.h"
#include "lsm6.h"
#include "pico/stdlib.h"
#include "plot.h"

int main() {
  stdio_init_all();
  sleep_ms(2000);

  const auto i2c_port = init_i2c_port(7, LSM6::MAX_BAUDRATE);

  const LSM6::AccelConfig accel_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::acc::G_2};
  const LSM6::GyroConfig gyro_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::gyro::DPS_1000};
  const LSM6 lsm6(i2c_port, accel_config, gyro_config, true, LSM6_CALIBRATION);
  if (not lsm6.is_connected()) {
    printf("LSM6 not connected\n");
    return 1;
  }

  const LIS3::MagnetometerConfig mag_config{.odr = lis3::odr::HZ_155, .fs = lis3::fs::GS_4};
  const LIS3 lis3(i2c_port, mag_config, true, LIS3_CALIBRATION);
  if (not lis3.is_connected()) {
    printf("LIS3 not connected\n");
    return 1;
  }

  while (true) {
    if (lsm6.is_new_data_available(true, true)) {
      const auto acc = lsm6.read_acceleration();
      const auto rot = lsm6.read_rotation();
      plot("acc_x", acc.x());
      plot("acc_y", acc.y());
      plot("acc_z", acc.z());
      // Display rotation in rps to make axis scales more similar.
      plot("rot_x", rot.x() / 360);
      plot("rot_y", rot.y() / 360);
      plot("rot_z", rot.z() / 360);
    }
    if (lis3.is_new_data_available()) {
      const auto mag = lis3.read_field();
      plot("mag_x", mag.x());
      plot("mag_y", mag.y());
      plot("mag_z", mag.z());
    }
  }
}