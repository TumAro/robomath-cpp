#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#include "robomath/c_space.hpp"

int main() {
  assert(nDimensionalRigidBody::dim2dof(2) == 3);
  assert(nDimensionalRigidBody::dim2dof(3) == 6);
  assert(nDimensionalRigidBody::dof2rot_freedom(3) == 3);
  assert(nDimensionalRigidBody::dof2rot_freedom(3, 6) == 3);
  assert(nDimensionalRigidBody::dof2trans_freedom(3) == 3);
  assert(nDimensionalRigidBody::space_topology(3) == "R3 x S2 x S1");

  // planar 4-bar: 3*(4 - 1 - 4) + 4 revolute joints = 1 dof
  assert(Mechanism::grubler(4, 4, 3, {'R', 'R', 'R', 'R'}) == 1);
  // spatial 6R open chain
  assert(Mechanism::grubler(7, 6, 6, {'R', 'R', 'R', 'R', 'R', 'R'}) == 6);

  try {
    PlanarNBarClosedChain({1.0, 1.0, 1.0});
    assert(false);
  } catch (const std::invalid_argument&) {
  }

  PlanarNBarClosedChain square({1.0, 1.0, 1.0, 1.0});

  // unit square traversed counter-clockwise, one right angle per joint
  std::vector<double> closed = {M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2};
  assert(square.is_valid_config(closed));

  // phi includes theta[i], so the first link already sits at theta[0] = 90 deg
  std::vector<Eigen::Vector2d> pos = square.joint_positions(closed);
  assert(pos.size() == 5);
  assert(pos[1].isApprox(Eigen::Vector2d(0, 1), 1e-9));
  assert(pos[2].isApprox(Eigen::Vector2d(-1, 1), 1e-9));
  assert(pos[3].isApprox(Eigen::Vector2d(-1, 0), 1e-9));
  assert(pos[4].norm() < 1e-9);  // chain closes back onto the origin

  std::vector<double> open = {M_PI / 2, M_PI / 2, M_PI / 2, 0.0};
  assert(!square.is_valid_config(open));

  try {
    square.constraint_residual({0.0, 0.0});
    assert(false);
  } catch (const std::invalid_argument&) {
  }

  // the Pfaffian matrix must equal the finite differences of the residual it
  // claims to differentiate
  std::vector<double> th = {1.1, -0.4, 2.0, 0.7};
  Eigen::MatrixXd A = square.pfaffian(th);
  assert(A.rows() == 3 && A.cols() == 4);

  const double h = 1e-6;
  for (int j = 0; j < 4; ++j) {
    std::vector<double> up = th, down = th;
    up[j] += h;
    down[j] -= h;

    auto [u1, u2, u3] = square.constraint_residual(up);
    auto [d1, d2, d3] = square.constraint_residual(down);
    double fd[3] = {(u1 - d1) / (2 * h), (u2 - d2) / (2 * h), (u3 - d3) / (2 * h)};

    for (int k = 0; k < 3; ++k) {
      if (std::abs(A(k, j) - fd[k]) > 1e-5) {
        std::fprintf(stderr, "A(%d,%d) = %.9f but finite difference says %.9f\n", k, j, A(k, j), fd[k]);
        assert(false);
      }
    }
  }

  // a closed 4-bar has 1 dof, so the 3xn Pfaffian must leave a 1-D null space
  assert(square.pfaffian(closed).fullPivLu().dimensionOfKernel() == 1);
}
