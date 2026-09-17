#include <cassert>
#include <cmath>
#include <vector>
#include "robomath/kinematics/forward.hpp"
#include "robomath/core/se3.hpp"
#include "robomath/core/so3.hpp"

using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;

int main() {
  // 2R planar arm: joint 1 at origin, joint 2 at (1, 0, 0), tool at (2, 0, 0)
  Matrix4 M = transform(Matrix3::Identity(), Vector3(2, 0, 0));

  Vector6 S1, S2;
  S1 << 0, 0, 1, 0, 0, 0;
  S2 << 0, 0, 1, 0, -1, 0;
  std::vector<Matrix4> S_list = {vec_to_se3(S1), vec_to_se3(S2)};

  Matrix4 Minv = trans_inverse(M);
  std::vector<Matrix4> B_list;
  for (const Matrix4& S : S_list)
    B_list.push_back(vec_to_se3(adjoint(Minv) * se3_to_vec(S)));

  Eigen::VectorXd zero = Eigen::VectorXd::Zero(2);
  assert(fwd_space(M, S_list, zero).isApprox(M));
  assert(fwd_body(M, B_list, zero).isApprox(M));

  // only joint 1 turns: tool swings about the origin to (0, 2, 0)
  Eigen::VectorXd th1(2);
  th1 << M_PI / 2, 0;
  assert(fwd_space(M, S_list, th1).isApprox(transform(rot_z(M_PI / 2), Vector3(0, 2, 0)), 1e-9));

  // only joint 2 turns: tool swings about (1, 0, 0) to (1, 1, 0)
  Eigen::VectorXd th2(2);
  th2 << 0, M_PI / 2;
  assert(fwd_space(M, S_list, th2).isApprox(transform(rot_z(M_PI / 2), Vector3(1, 1, 0)), 1e-9));

  // space and body forms describe the same arm, so they must agree
  Eigen::VectorXd th(2);
  th << M_PI / 2, -M_PI / 3;
  assert(fwd_space(M, S_list, th).isApprox(fwd_body(M, B_list, th), 1e-9));

  Eigen::VectorXd wrong_size(3);
  wrong_size << 0, 0, 0;
  try {
    fwd_space(M, S_list, wrong_size);
    assert(false);
  } catch (const std::invalid_argument&) {
  }
}
