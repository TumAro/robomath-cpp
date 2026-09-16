#pragma once

#include <tuple>
#include <cmath>
#include <Eigen/Dense>

// SE3 is the Euclidean group of rigid body transformations
bool SE3_test(Eigen::Matrix4d T);
Eigen::Matrix4d transform(Eigen::Matrix3d R, Eigen::Vector3d p);
Eigen::Matrix3d get_rotation(Eigen::Matrix4d T);
Eigen::Vector3d get_translation(Eigen::Matrix4d T);
Eigen::Matrix4d rotation(Eigen::Vector3d w, double theta);
Eigen::Matrix4d translation(Eigen::Vector3d p);
Eigen::Matrix4d trans_inverse(Eigen::Matrix4d T);
Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix4d T);
Eigen::Matrix<double, 6, 1> wrench_transform(Eigen::Matrix<double, 6, 1> F, Eigen::Matrix4d T);
Eigen::Matrix4d compose(Eigen::Matrix4d A, Eigen::Matrix4d B);

// se3 is the tangent space of SE3, its Lie algebra
bool se3_test(Eigen::Matrix4d mat);
Eigen::Matrix4d vec_to_se3(Eigen::Matrix<double, 6, 1> V);
Eigen::Matrix<double, 6, 1> se3_to_vec(Eigen::Matrix4d mat);
Eigen::Matrix4d screw_exp6(Eigen::Matrix3d w, Eigen::Vector3d v, double theta);
Eigen::Matrix4d mat_exp6(Eigen::Matrix4d se3mat, double theta);
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> logarithm6(Eigen::Matrix4d T);
std::tuple<Eigen::Vector3d, Eigen::Vector3d, double> axis_to_screw(Eigen::Matrix<double, 6, 1> S);
Eigen::Matrix<double, 6, 1> screw_to_axis(Eigen::Vector3d q, Eigen::Vector3d s_hat, double h);
