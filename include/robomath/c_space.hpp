#pragma once

#include <vector>
#include <string>
#include <optional>
#include <tuple>
#include <Eigen/Dense>

class nDimensionalRigidBody {
public:
  static int dim2dof(int n);
  static int dof2rot_freedom(int n, std::optional<int> m = std::nullopt);
  static int dof2trans_freedom(int n);
  static std::string space_topology(int n);
};

class Mechanism {
public:
  static int grubler(int N, int J, int m, const std::vector<char>& joints);
};

class PlanarNBarClosedChain {
public:
  explicit PlanarNBarClosedChain(std::vector<double> lengths);

  std::tuple<double, double, double> constraint_residual(const std::vector<double>& theta) const;
  Eigen::MatrixXd pfaffian(const std::vector<double>& theta) const;
  bool is_valid_config(const std::vector<double>& theta, double tolerance = 1e-8) const;
  std::vector<Eigen::Vector2d> joint_positions(const std::vector<double>& theta) const;

private:
  std::vector<double> link_lengths_;
  int n_;
  int dof_;

  std::vector<double> cumulative_angles(const std::vector<double>& theta) const;
};
