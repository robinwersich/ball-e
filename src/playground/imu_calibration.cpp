#include <stdio.h>
#include <iostream>
#include <string>

#include "lsm6.h"
#include "pico/stdlib.h"

constexpr uint SAMPLE_COUNT = 200;

Eigen::Vector3d measure_accel(const LSM6& imu, uint sample_count) {
  uint measured_sample_count = 0;
  Eigen::Vector3d acc_sum{};

  while (measured_sample_count < sample_count) {
    if (imu.is_new_data_available(true, false)) {
      const auto acc = imu.read_acceleration();
      acc_sum += acc.cast<double>();
      measured_sample_count++;
    }
  }
  return acc_sum / sample_count;
}

Eigen::Vector3d measure_gyro(const LSM6& imu, uint sample_count) {
  uint measured_sample_count = 0;
  Eigen::Vector3d gyro_sum{};

  while (measured_sample_count < sample_count) {
    if (imu.is_new_data_available(false, true)) {
      const auto gyro = imu.read_rotation();
      gyro_sum += gyro.cast<double>();
      measured_sample_count++;
    }
  }
  return gyro_sum / sample_count;
}

int main() {
  stdio_init_all();
  using namespace lsm6;

  const LSM6::AccelConfig accel_config{.odr = odr::HZ_104, .fs = fs::acc::G_2};
  const LSM6::GyroConfig gyro_config{.odr = odr::HZ_104, .fs = fs::gyro::DPS_500};
  const LSM6 imu(7, accel_config, gyro_config);

  std::string command;
  while (true) {
    std::cin >> command;
    if (command == "gyro") {
      const auto gyro = measure_gyro(imu, SAMPLE_COUNT);
      printf("%f %f %f\n", gyro.x(), gyro.y(), gyro.z());
    } else if (command == "accel") {
      const auto acc = measure_accel(imu, SAMPLE_COUNT);
      printf("%f %f %f\n", acc.x(), acc.y(), acc.z());
    } else {
      printf("Unknown command '%s'\n", command.c_str());
    }
  }
}