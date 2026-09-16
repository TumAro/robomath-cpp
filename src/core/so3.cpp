#include "robomath/core/so3.hpp"
using Matrix = Eigen::Matrix3d;
using Vector = Eigen::Vector3d;
const Matrix I = Matrix::Identity();

bool SO3_test(Matrix m) {
  if (!(m.transpose() * m).isApprox(I)) return false;
  if (std::abs(m.determinant() - 1) > 1e-6) return false;

  return true;
}

Matrix rot_inv(Matrix m) {
  if (SO3_test(m)) return m.transpose();
  return I;
}

Matrix rot_x(double th) {
  Matrix result;
  result << 1, 0, 0,
            0, std::cos(th), -std::sin(th),
            0, std::sin(th), std::cos(th);
  return result;
}

Matrix rot_y(double th) {
  Matrix result;
  result << std::cos(th), 0, std::sin(th),
            0, 1, 0,
            -std::sin(th), 0, std::cos(th);
  return result;
}

Matrix rot_z(double th) {
  Matrix result;
  result << std::cos(th), -std::sin(th), 0,
            std::sin(th), std::cos(th), 0,
            0, 0, 1;
  return result;
}

Matrix skew_symmetric(double x1, double x2, double x3) {
  Matrix result;
  result << 0, -x3, x2,
            x3, 0, -x1,
            -x2, x1, 0;
  return result;
}

bool check_skew_symmetry(Matrix m) {
  return m.transpose().isApprox(-m);
}

Vector skew_to_vec(Matrix m) {
  if (!check_skew_symmetry(m)) return Vector::Zero();
  return Vector(m(2, 1), m(0, 2), m(1, 0));
}

Matrix rodrigues(Vector w, double th) {
  Matrix w_hat = skew_symmetric(w.x(), w.y(), w.z());
  return I + std::sin(th) * w_hat + (1 - std::cos(th)) * (w_hat * w_hat);
}

Matrix skew_mat_to_SO3(Vector w) {
  double th = w.norm();
  if (th < 1e-9) return I;
  return rodrigues(w / th, th);
}

std::tuple<Matrix, double> logarithm(Matrix R) {
  if (R.isApprox(I)) return {Matrix::Zero(), 0.0};

  if (std::abs(R.trace() + 1) < 1e-9) {
    double theta = std::acos(-1.0);
    Vector w;
    if (std::abs(1 + R(2, 2)) > 1e-9) {
      w = Vector(R(0, 2), R(1, 2), 1 + R(2, 2)) / std::sqrt(2 * (1 + R(2, 2)));
    } else if (std::abs(1 + R(1, 1)) > 1e-9) {
      w = Vector(R(0, 1), 1 + R(1, 1), R(2, 1)) / std::sqrt(2 * (1 + R(1, 1)));
    } else {
      w = Vector(1 + R(0, 0), R(1, 0), R(2, 0)) / std::sqrt(2 * (1 + R(0, 0)));
    }
    return {skew_symmetric(w.x(), w.y(), w.z()), theta};
  }

  double theta = std::acos(0.5 * (R.trace() - 1));
  Matrix w = (R - R.transpose()) / (2 * std::sin(theta));
  return {w, theta};
}
