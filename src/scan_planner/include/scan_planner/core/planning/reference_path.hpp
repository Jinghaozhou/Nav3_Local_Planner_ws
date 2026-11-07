#pragma once
#include "scan_planner/core/types.hpp"

namespace scan_planner {
class ReferencePath {
public:
  void set(const Path &path);
  Path localSegment(const Eigen::Vector3d &position, double horizon);
  bool empty() const { return points_.empty(); }
  const Eigen::Vector3d &goal() const { return points_.back(); }
  double remaining() const { return lengths_.empty() ? 0.0 : lengths_.back() - progress_; }
  void reanchor() { progress_=0.0; initialized_=false; }
private:
  Eigen::Vector3d at(double progress) const;
  Path points_;
  std::vector<double> lengths_;
  double progress_{0.0};
  bool initialized_{false};
};
Path resample(const Path &points, double spacing);
double distanceToPath(const Eigen::Vector3d &point, const Path &path);
}  // namespace scan_planner
