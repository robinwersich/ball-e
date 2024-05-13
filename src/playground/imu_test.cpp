#include <stdio.h>

#include "lsm6.h"
#include "pico/stdlib.h"
#include "plot.h"

int main() {
  stdio_init_all();
  sleep_ms(2000);

  const LSM6 imu(7, {}, {});
  if (not imu.is_connected()) {
    printf("IMU not connected\n");
    return 1;
  }

  while (true) {
    if (imu.is_new_data_available(true, true)) {
      const auto acc = imu.read_acceleration_raw();
      const auto rot = imu.read_rotation_raw();
      plot("acc_x", acc.x());
      plot("acc_y", acc.y());
      plot("acc_z", acc.z());
      plot("rot_x", rot.x());
      plot("rot_y", rot.y());
      plot("rot_z", rot.z());
    }
  }
}