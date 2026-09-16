#include "robomath/core/se2.hpp"
#include "robomath/core/so2.hpp"
using Matrix = Eigen::Matrix3d;
using Matrix2 = Eigen::Matrix2d;
using Vector2 = Eigen::Vector2d;
const Matrix I = Matrix::Identity();

bool SE2_test(Matrix T) {
  if (!SO2_test(T.block<2, 2>(0, 0))) return false;
  if (!T.row(2).isApprox(Eigen::RowVector3d(0, 0, 1))) return false;

  return true;
}

Matrix transform(double th, Vector2 p) {
  Matrix T = I;
  T.block<2, 2>(0, 0) = rot_matrix(th);
  T.block<2, 1>(0, 2) = p;
  return T;
}

Matrix trans_inverse(Matrix T) {
  if (!SE2_test(T)) return I;
  Matrix2 R = T.block<2, 2>(0, 0);
  Vector2 p = T.block<2, 1>(0, 2);

  Matrix invT = I;
  invT.block<2, 2>(0, 0) = R.transpose();
  invT.block<2, 1>(0, 2) = -R.transpose() * p;
  return invT;
}
