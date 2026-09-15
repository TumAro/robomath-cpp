#include <cassert>
#include <cmath>
#include "robomath/core/so2.hpp"

int main() {
  assert(SO2_test(Eigen::Matrix2d::Identity()));

  Eigen::Matrix2d bad;
  bad << 1, 2, 3, 4;
  assert(!SO2_test(bad));

  assert(rot_matrix(0.0f).isApprox(Eigen::Matrix2d::Identity()));

  Eigen::Vector2d v(1, 0);
  assert((rot_matrix(M_PI / 2) * v).isApprox(Eigen::Vector2d(0, 1), 1e-5));

  Eigen::Matrix2d R = rot_matrix(M_PI / 4);
  assert(rot_inv(R).isApprox(R.transpose()));
  assert(rot_inv(bad).isApprox(Eigen::Matrix2d::Identity()));
}
