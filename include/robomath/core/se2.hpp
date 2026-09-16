#pragma once

#include <Eigen/Dense>

bool SE2_test(Eigen::Matrix3d T);
Eigen::Matrix3d transform(double th, Eigen::Vector2d p);
Eigen::Matrix3d trans_inverse(Eigen::Matrix3d T);
