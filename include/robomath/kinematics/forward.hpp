#pragma once

#include <vector>
#include <Eigen/Dense>

Eigen::Matrix4d fwd_space(Eigen::Matrix4d M, const std::vector<Eigen::Matrix4d>& S_list, const Eigen::VectorXd& theta);
Eigen::Matrix4d fwd_body(Eigen::Matrix4d M, const std::vector<Eigen::Matrix4d>& B_list, const Eigen::VectorXd& theta);
