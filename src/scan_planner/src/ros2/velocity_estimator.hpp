#pragma once
#include <Eigen/Core>

namespace scan_planner {
class VelocityEstimator {
public:
  explicit VelocityEstimator(double alpha=0.35, double max_speed=3.0)
    : alpha_(alpha), max_speed_(max_speed) {}
  bool update(const Eigen::Vector3d &position, double stamp, Eigen::Vector3d &velocity);
  void reset() { ready_=false; velocity_.setZero(); }
private:
  double alpha_, max_speed_, stamp_{0};
  bool ready_{false};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()}, velocity_{Eigen::Vector3d::Zero()};
};

// Time-based low-pass for an odometry twist that contains periodic body motion.
// A zero time constant passes measurements through unchanged.
class TwistVelocityFilter {
public:
  void setTimeConstant(double seconds) { time_constant_=seconds; reset(); }
  bool update(const Eigen::Vector3d &measurement, double stamp, Eigen::Vector3d &velocity);
  void reset() { ready_=false; velocity_.setZero(); }
private:
  double time_constant_{0.0}, stamp_{0.0};
  bool ready_{false};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
};
}  // namespace scan_planner
