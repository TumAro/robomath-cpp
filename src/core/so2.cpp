#include "robomath/core/so2.hpp"
using Matrix = Eigen::Matrix2d;
const Matrix I = Matrix::Identity();

bool SO2_test(Matrix m) {
  if (!(m.transpose() * m).isApprox(I)) return false;
  if (std::abs(m.determinant() - 1) > 1e-6) return false;

  return true;
}

Matrix rot_inv(Matrix m) {
  if (SO2_test(m)) return m.transpose();
  return I;
}


Matrix rot_matrix(double th) {
  Matrix result;
  result << std::cos(th), -std::sin(th),
            std::sin(th), std::cos(th);
  return result;
}
