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
}  // namespace scan_planner
