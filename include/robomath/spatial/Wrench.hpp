#pragma once

#include <Eigen/Dense>

// Wrench F = (m, f) in R^6: moment first, then force
struct Wrench {
  Eigen::Matrix<double, 6, 1> vec;

  explicit Wrench(const Eigen::Matrix<double, 6, 1>& F) : vec(F) {}
  Wrench(const Eigen::Vector3d& moment, const Eigen::Vector3d& force) { vec << moment, force; }

  Eigen::Vector3d moment() const { return vec.head<3>(); }
  Eigen::Vector3d force() const { return vec.tail<3>(); }
};
