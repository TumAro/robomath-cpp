#include "robomath/c_space.hpp"
#include <cmath>
#include <map>
#include <stdexcept>

int nDimensionalRigidBody::dim2dof(int n) {
  return static_cast<int>(std::floor(n * (n + 1) / 2.0));
}

int nDimensionalRigidBody::dof2rot_freedom(int n, std::optional<int> m) {
  if (m) return *m - n;
  return static_cast<int>(std::floor(n * (n - 1) / 2.0));
}

int nDimensionalRigidBody::dof2trans_freedom(int n) {
  return n;
}

std::string nDimensionalRigidBody::space_topology(int n) {
  std::string topology;
  for (int power = n; power > 0; --power) {
    if (power == n) {
      topology += "R" + std::to_string(power);
      continue;
    }
    topology += " x S" + std::to_string(power);
  }
  return topology;
}

int Mechanism::grubler(int N, int J, int m, const std::vector<char>& joints) {
  static const std::map<char, int> f_values = {
    {'R', 1}, {'P', 1}, {'H', 1}, {'C', 2}, {'U', 2}, {'S', 3}
  };

  if (J != static_cast<int>(joints.size()))
    throw std::invalid_argument("number of joints size mismatch");

  int freedom = 0;
  for (char joint : joints) {
    auto it = f_values.find(joint);
    if (it == f_values.end())
      throw std::invalid_argument(std::string("unknown joint type: ") + joint);
    freedom += it->second;
  }

  return m * (N - 1 - J) + freedom;
}

PlanarNBarClosedChain::PlanarNBarClosedChain(std::vector<double> lengths)
    : link_lengths_(std::move(lengths)), n_(static_cast<int>(link_lengths_.size())) {
  if (n_ < 4)
    throw std::invalid_argument("Minimum closed chain needs 4 links, given: " + std::to_string(n_));

  dof_ = Mechanism::grubler(n_, n_, 3, std::vector<char>(n_, 'R'));
}

std::vector<double> PlanarNBarClosedChain::cumulative_angles(const std::vector<double>& theta) const {
  // phi[i] includes theta[i]: link i sits at the sum of every joint up to and
  // including its own, which is what MR eq 2.7 assumes
  std::vector<double> phi(theta.size(), 0.0);
  double running = 0.0;
  for (size_t i = 0; i < theta.size(); ++i) {
    running += theta[i];
    phi[i] = running;
  }
  return phi;
}

std::tuple<double, double, double>
PlanarNBarClosedChain::constraint_residual(const std::vector<double>& theta) const {
  if (static_cast<int>(theta.size()) != n_)
    throw std::invalid_argument("Number of angles mismatch with number of links!");

  std::vector<double> phi = cumulative_angles(theta);

  double g1 = 0.0, g2 = 0.0, g3 = 0.0;
  for (int i = 0; i < n_; ++i) {
    g1 += link_lengths_[i] * std::cos(phi[i]);
    g2 += link_lengths_[i] * std::sin(phi[i]);
    g3 += theta[i];
  }

  return {g1, g2, g3 - 2 * M_PI};
}

Eigen::MatrixXd PlanarNBarClosedChain::pfaffian(const std::vector<double>& theta) const {
  if (static_cast<int>(theta.size()) != n_)
    throw std::invalid_argument("Number of angles mismatch with number of links!");

  std::vector<double> phi = cumulative_angles(theta);
  std::vector<double> s(n_), c(n_);
  for (int i = 0; i < n_; ++i) {
    s[i] = -link_lengths_[i] * std::sin(phi[i]);
    c[i] = link_lengths_[i] * std::cos(phi[i]);
  }

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n_);
  J(0, n_ - 1) = s[n_ - 1];
  J(1, n_ - 1) = c[n_ - 1];

  for (int j = n_ - 2; j >= 0; --j) {
    J(0, j) = s[j] + J(0, j + 1);
    J(1, j) = c[j] + J(1, j + 1);
  }

  J.row(2).setOnes();
  return J;
}

bool PlanarNBarClosedChain::is_valid_config(const std::vector<double>& theta, double tolerance) const {
  auto [g1, g2, g3] = constraint_residual(theta);
  return std::sqrt(g1 * g1 + g2 * g2 + g3 * g3) < tolerance;
}

std::vector<Eigen::Vector2d>
PlanarNBarClosedChain::joint_positions(const std::vector<double>& theta) const {
  if (static_cast<int>(theta.size()) != n_)
    throw std::invalid_argument("Number of angles mismatch with number of links!");

  std::vector<double> phi = cumulative_angles(theta);
  std::vector<Eigen::Vector2d> pos = {Eigen::Vector2d::Zero()};

  for (int i = 0; i < n_; ++i)
    pos.push_back(pos[i] + link_lengths_[i] * Eigen::Vector2d(std::cos(phi[i]), std::sin(phi[i])));

  return pos;
}
