#include "Path.h"

LinearPath::LinearPath(const Eigen::Vector6d &from, const Eigen::Vector6d &to) {
  if (from.size() != to.size()) {
    throw std::invalid_argument("Start and end position must share the same dimension.");
  }
  if ((from.array() == to.array()).all()) {
    throw std::invalid_argument("Start and end position must be different.");
  }

  this->from = from;
  this->direction = to - from;
}

Eigen::VectorXd LinearPath::clamp(const Eigen::VectorXd &v, double lowerLimit, double upperLimit) {
  if (lowerLimit >= upperLimit) {
    throw std::invalid_argument("Upper limit must be bigger than lower limit.");
  }
  return v.cwiseMin(Eigen::VectorXd::Constant(v.size(), upperLimit))
      .cwiseMax(Eigen::VectorXd::Constant(v.size(), lowerLimit));
}

Eigen::Vector6d LinearPath::at(double s) {
  s = std::max(std::min(s, 1.), 0.); // std::clamp(s, 0, 1); // limit s to be in [0,1]
  return from + s * direction;
}

Eigen::Matrix6dynd LinearPath::at(const Eigen::VectorXd &s) {
  auto s_ = this->clamp(s, 0, 1);
  Eigen::Matrix6dynd p =
      from * Eigen::VectorXd::Ones(s_.size()).transpose() + direction * s_.transpose();
  return p;
}

Eigen::Vector6d LinearPath::ds_at(double) { return direction; }

Eigen::Matrix6dynd LinearPath::ds_at(const Eigen::VectorXd &s) {
  Eigen::Matrix6dynd dp = direction * Eigen::VectorXd::Ones(s.size()).transpose();
  return dp;
}

std::size_t LinearPath::dim() { return this->from.size(); }

double LinearPath::getL() { return direction.norm(); }