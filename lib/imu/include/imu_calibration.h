#pragma once

#include <Eigen/Core>

struct ImuCalibration {
  Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f gyro_transform = Eigen::Matrix3f::Identity();
  Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f accel_transform = Eigen::Matrix3f::Identity();
};
