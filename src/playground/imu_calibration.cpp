#include <stdio.h>

#include <array>
#include <iostream>
#include <string>

#include "lis3.h"
#include "lsm6.h"
#include "pico/stdlib.h"

Eigen::Vector3d measure_accel(const LSM6& lsm6, uint sample_count) {
  uint measured_sample_count = 0;
  Eigen::Vector3d acc_sum{};

  while (measured_sample_count < sample_count) {
    if (lsm6.is_new_data_available(true, false)) {
      const auto acc = lsm6.read_acceleration();
      acc_sum += acc.cast<double>();
      measured_sample_count++;
    }
  }
  return acc_sum / sample_count;
}

Eigen::Vector3d measure_gyro(const LSM6& lsm6, uint sample_count) {
  uint measured_sample_count = 0;
  Eigen::Vector3d gyro_sum{};

  while (measured_sample_count < sample_count) {
    if (lsm6.is_new_data_available(false, true)) {
      const auto gyro = lsm6.read_rotation();
      gyro_sum += gyro.cast<double>();
      measured_sample_count++;
    }
  }
  return gyro_sum / sample_count;
}

Eigen::Vector2f measure_mag(const LIS3& lis3) {
  while (!lis3.is_new_data_available()) {
    sleep_ms(1);
  }
  return lis3.read_field();
}

int main() {
  stdio_init_all();

  const LSM6::AccelConfig accel_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::acc::G_2};
  const LSM6::GyroConfig gyro_config{.odr = lsm6::odr::HZ_104, .fs = lsm6::fs::gyro::DPS_500};
  const auto i2c_port = init_i2c_port(7, LSM6::MAX_BAUDRATE);
  const LSM6 lsm6(i2c_port, accel_config, gyro_config);
  const LIS3::MagnetometerConfig mag_config{.odr = lis3::odr::HZ_155, .fs = lis3::fs::GS_4};
  const LIS3 lis3(i2c_port, mag_config);

  std::string command;
  while (true) {
    std::cin >> command;
    if (command == "gyro") {
      const auto gyro = measure_gyro(lsm6, 200);
      printf("%f %f %f\n", gyro.x(), gyro.y(), gyro.z());
    } else if (command == "accel") {
      const auto acc = measure_accel(lsm6, 200);
      printf("%f %f %f\n", acc.x(), acc.y(), acc.z());
    } else if (command == "mag") {
      const auto mag = measure_mag(lis3);
      printf("%f %f\n", mag.x(), mag.y());
    } else {
      printf("Unknown command '%s'\n", command.c_str());
    }
  }
}