#pragma once

#include <Eigen/Dense>
#include <cmath>

bool SO2_test(Eigen::Matrix2d m);
Eigen::Matrix2d rot_inv(Eigen::Matrix2d m);
Eigen::Matrix2d rot_matrix(double th);
