#pragma once
// Generic B-spline message validation/evaluation. Mission remains the only setpoint publisher.
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <rm_drone_msgs/msg/local_trajectory.hpp>

namespace rm_drone
{
class SplineTrajectory
{
public:
  bool assign(const rm_drone_msgs::msg::LocalTrajectory & msg)
  {
    valid_ = false;
    const size_t n = msg.control_points.size();
    const size_t degree = msg.degree;
    if (degree != 3 || n < degree + 1 || n > 512 || msg.knots.size() != n + degree + 1) {return false;}
    for (size_t i = 0; i < msg.knots.size(); ++i) {
      if (!std::isfinite(msg.knots[i]) || (i && msg.knots[i] < msg.knots[i - 1])) {return false;}
    }
    if (std::abs(msg.knots[degree]) > 1e-6 || msg.knots[n] <= 0 || msg.knots[n] > 120) {return false;}
    p_.clear();
    for (const auto & point : msg.control_points) {
      Eigen::Vector3d p(point.x, point.y, point.z);
      if (!p.allFinite()) {return false;}
      p_.push_back(p);
    }
    knots_ = msg.knots;
    if (!derivative(p_, knots_, 3, v_) || !derivative(v_, trimmed(knots_), 2, a_)) {return false;}
    duration_ = knots_[n];
    valid_ = true;
    return true;
  }
  bool valid() const {return valid_;}
  double duration() const {return duration_;}
  Eigen::Vector3d position(double t) const {return evaluate(p_, knots_, 3, t);}
  Eigen::Vector3d velocity(double t) const {return evaluate(v_, trimmed(knots_), 2, t);}
  Eigen::Vector3d acceleration(double t) const {return evaluate(a_, trimmed(trimmed(knots_)), 1, t);}
private:
  static std::vector<double> trimmed(const std::vector<double> & u) {return {u.begin() + 1, u.end() - 1};}
  static bool derivative(const std::vector<Eigen::Vector3d> & p, const std::vector<double> & u,
    int degree, std::vector<Eigen::Vector3d> & result)
  {
    result.clear();
    for (size_t i = 0; i + 1 < p.size(); ++i) {
      const double denominator = u[i + degree + 1] - u[i + 1];
      if (denominator < 1e-8) {return false;}
      result.push_back(degree * (p[i + 1] - p[i]) / denominator);
    }
    return true;
  }
  static Eigen::Vector3d evaluate(const std::vector<Eigen::Vector3d> & p,
    const std::vector<double> & u, int degree, double t)
  {
    t = std::clamp(t, u[degree], u[p.size()]);
    int span = static_cast<int>(std::upper_bound(u.begin(), u.end(), t) - u.begin()) - 1;
    span = std::clamp(span, degree, static_cast<int>(p.size()) - 1);
    Eigen::Vector3d work[4];
    for (int j = 0; j <= degree; ++j) {work[j] = p[span - degree + j];}
    for (int r = 1; r <= degree; ++r) {
      for (int j = degree; j >= r; --j) {
        const int index = span - degree + j;
        const double den = u[index + degree - r + 1] - u[index];
        const double alpha = den > 1e-12 ? (t - u[index]) / den : 0;
        work[j] = (1 - alpha) * work[j - 1] + alpha * work[j];
      }
    }
    return work[degree];
  }
  bool valid_{false};
  double duration_{0};
  std::vector<double> knots_;
  std::vector<Eigen::Vector3d> p_, v_, a_;
};
}
