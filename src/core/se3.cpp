#include "robomath/core/se3.hpp"
#include "robomath/core/so3.hpp"
#include <limits>

using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
const Matrix4 I4 = Matrix4::Identity();
const Matrix3 I3 = Matrix3::Identity();

bool SE3_test(Matrix4 T) {
  if (!SO3_test(T.block<3, 3>(0, 0))) return false;
  if (!T.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1))) return false;

  return true;
}

Matrix4 transform(Matrix3 R, Vector3 p) {
  Matrix4 T = I4;
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

Matrix3 get_rotation(Matrix4 T) {
  if (!SE3_test(T)) return I3;
  return T.block<3, 3>(0, 0);
}

Vector3 get_translation(Matrix4 T) {
  if (!SE3_test(T)) return Vector3::Zero();
  return T.block<3, 1>(0, 3);
}

Matrix4 rotation(Vector3 w, double theta) {
  Matrix4 T = I4;
  T.block<3, 3>(0, 0) = rodrigues(w, theta);
  return T;
}

Matrix4 translation(Vector3 p) {
  Matrix4 T = I4;
  T.block<3, 1>(0, 3) = p;
  return T;
}

Matrix4 trans_inverse(Matrix4 T) {
  if (!SE3_test(T)) return I4;
  Matrix3 R = T.block<3, 3>(0, 0);
  Vector3 p = T.block<3, 1>(0, 3);

  Matrix4 invT = I4;
  invT.block<3, 3>(0, 0) = R.transpose();
  invT.block<3, 1>(0, 3) = -R.transpose() * p;
  return invT;
}

Matrix6 adjoint(Matrix4 T) {
  Matrix6 adj = Matrix6::Zero();
  if (!SE3_test(T)) return adj;

  Matrix3 R = T.block<3, 3>(0, 0);
  Vector3 p = T.block<3, 1>(0, 3);

  adj.block<3, 3>(0, 0) = R;
  adj.block<3, 3>(3, 3) = R;
  adj.block<3, 3>(3, 0) = skew_symmetric(p.x(), p.y(), p.z()) * R;
  return adj;
}

Vector6 wrench_transform(Vector6 F, Matrix4 T) {
  return adjoint(T).transpose() * F;
}

Matrix4 compose(Matrix4 A, Matrix4 B) {
  if (!SE3_test(A) || !SE3_test(B)) return I4;

  Matrix3 R_A = get_rotation(A);
  Vector3 p_A = get_translation(A);
  Matrix3 R_B = get_rotation(B);
  Vector3 p_B = get_translation(B);

  Matrix4 T = I4;
  T.block<3, 3>(0, 0) = R_A * R_B;
  T.block<3, 1>(0, 3) = R_A * p_B + p_A;
  return T;
}

bool se3_test(Matrix4 mat) {
  if (!mat.row(3).isApprox(Eigen::RowVector4d::Zero())) return false;
  if (!check_skew_symmetry(mat.block<3, 3>(0, 0))) return false;

  return true;
}

Matrix4 vec_to_se3(Vector6 V) {
  Vector3 w = V.head<3>();
  Vector3 v = V.tail<3>();

  Matrix4 result = Matrix4::Zero();
  result.block<3, 3>(0, 0) = skew_symmetric(w.x(), w.y(), w.z());
  result.block<3, 1>(0, 3) = v;
  return result;
}

Vector6 se3_to_vec(Matrix4 mat) {
  if (!se3_test(mat)) return Vector6::Zero();

  Vector3 w = skew_to_vec(mat.block<3, 3>(0, 0));
  Vector3 v = mat.block<3, 1>(0, 3);

  Vector6 V;
  V << w, v;
  return V;
}

Matrix4 screw_exp6(Matrix3 w, Vector3 v, double theta) {
  Matrix4 exp = I4;
  double w_norm = skew_to_vec(w).norm();

  if (std::abs(w_norm - 1) < 1e-9) {
    exp.block<3, 3>(0, 0) = rodrigues(skew_to_vec(w), theta);
    exp.block<3, 1>(0, 3) = (I3 * theta + (1 - std::cos(theta)) * w +
                              (theta - std::sin(theta)) * (w * w)) * v;
    return exp;
  }

  if (w_norm < 1e-9 && std::abs(v.norm() - 1) < 1e-9) {
    exp.block<3, 1>(0, 3) = v * theta;
    return exp;
  }

  return exp;
}

Matrix4 mat_exp6(Matrix4 se3mat, double theta) {
  if (!se3_test(se3mat)) return I4;
  return screw_exp6(se3mat.block<3, 3>(0, 0), se3mat.block<3, 1>(0, 3), theta);
}

std::tuple<Matrix3, Vector3, double> logarithm6(Matrix4 T) {
  if (!SE3_test(T)) return {Matrix3::Zero(), Vector3::Zero(), 0.0};

  Matrix3 R = T.block<3, 3>(0, 0);
  Vector3 p = T.block<3, 1>(0, 3);

  if (R.isApprox(I3)) {
    if (p.isApprox(Vector3::Zero())) return {Matrix3::Zero(), Vector3::Zero(), 0.0};
    double theta = p.norm();
    return {Matrix3::Zero(), p / theta, theta};
  }

  auto [w, theta] = logarithm(R);
  Vector3 v = (I3 / theta - w / 2 +
               (1 / theta - std::cos(theta / 2) / (2 * std::sin(theta / 2))) * (w * w)) * p;
  return {w, v, theta};
}

std::tuple<Vector3, Vector3, double> axis_to_screw(Vector6 S) {
  Vector3 w = S.head<3>();
  Vector3 v = S.tail<3>();

  if (std::abs(w.norm() - 1) < 1e-9) {
    Vector3 s_hat = w;
    double h = w.dot(v);
    Vector3 q = w.cross(v - h * w);
    return {q, s_hat, h};
  }

  if (w.norm() < 1e-9 && std::abs(v.norm() - 1) < 1e-9) {
    return {Vector3::Zero(), v, std::numeric_limits<double>::infinity()};
  }

  return {Vector3::Zero(), Vector3::Zero(), 0.0};
}

Vector6 screw_to_axis(Vector3 q, Vector3 s_hat, double h) {
  Vector3 w, v;
  if (std::isinf(h)) {
    w = Vector3::Zero();
    v = s_hat;
  } else {
    w = s_hat;
    v = (-s_hat).cross(q) + h * s_hat;
  }

  Vector6 S;
  S << w, v;
  return S;
}
