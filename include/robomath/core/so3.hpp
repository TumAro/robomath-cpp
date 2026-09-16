#pragma once

#include <tuple>
#include <cmath>
#include <Eigen/Dense>

bool SO3_test(Eigen::Matrix3d m);

Eigen::Matrix3d rot_inv(Eigen::Matrix3d m);
Eigen::Matrix3d rot_x(double th);
Eigen::Matrix3d rot_y(double th);
Eigen::Matrix3d rot_z(double th);

Eigen::Matrix3d rodrigues(Eigen::Vector3d w, double th);

Eigen::Matrix3d skew_mat_to_SO3(Eigen::Vector3d w);
Eigen::Matrix3d skew_symmetric(double x1, double x2, double x3);
bool check_skew_symmetry(Eigen::Matrix3d m);
Eigen::Vector3d skew_to_vec(Eigen::Matrix3d m);
std::tuple<Eigen::Matrix3d, double> logarithm(Eigen::Matrix3d R);
