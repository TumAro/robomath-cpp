#include "robomath/kinematics/forward.hpp"
#include "robomath/core/se3.hpp"
#include <stdexcept>

using Matrix4 = Eigen::Matrix4d;

Matrix4 fwd_space(Matrix4 M, const std::vector<Matrix4>& S_list, const Eigen::VectorXd& theta) {
  if (static_cast<Eigen::Index>(S_list.size()) != theta.size())
    throw std::invalid_argument("Size mismatch for given Screw Axis list and theta vector");

  Matrix4 T = M;
  for (Eigen::Index i = theta.size() - 1; i >= 0; --i) {
    if (std::abs(theta(i)) <= 1e-9) continue;
    T = mat_exp6(S_list[i], theta(i)) * T;
  }
  return T;
}

Matrix4 fwd_body(Matrix4 M, const std::vector<Matrix4>& B_list, const Eigen::VectorXd& theta) {
  if (static_cast<Eigen::Index>(B_list.size()) != theta.size())
    throw std::invalid_argument("Size mismatch for given Screw Axis list and theta vector");

  Matrix4 T = M;
  for (Eigen::Index i = 0; i < theta.size(); ++i) {
    if (std::abs(theta(i)) <= 1e-9) continue;
    T = T * mat_exp6(B_list[i], theta(i));
  }
  return T;
}
