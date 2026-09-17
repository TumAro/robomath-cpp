#pragma once

#include <Eigen/Dense>

// Twist V = (omega, v) in R^6: angular velocity first, then linear
struct Twist {
  Eigen::Matrix<double, 6, 1> vec;

  explicit Twist(const Eigen::Matrix<double, 6, 1>& V) : vec(V) {}
  Twist(const Eigen::Vector3d& angular, const Eigen::Vector3d& linear) { vec << angular, linear; }

  Eigen::Vector3d angular() const { return vec.head<3>(); }
  Eigen::Vector3d linear() const { return vec.tail<3>(); }
};
