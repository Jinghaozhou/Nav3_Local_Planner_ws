#include "scan_planner/core/planning/reference_path.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace scan_planner {
void ReferencePath::set(const Path &path) {
  points_.clear(); lengths_.clear(); progress_ = 0; initialized_ = false;
  if (path.empty() || path.size() > 200000) throw std::invalid_argument("Empty or oversized reference path");
  for (const auto &p : path) {
    if (!p.allFinite()) throw std::invalid_argument("Non-finite path point");
    if (points_.empty() || (p - points_.back()).norm() > 1e-6) points_.push_back(p);
  }
  lengths_.push_back(0.0);
  for (std::size_t i = 1; i < points_.size(); ++i)
    lengths_.push_back(lengths_.back() + (points_[i] - points_[i - 1]).norm());
}
Eigen::Vector3d ReferencePath::at(double s) const {
  if (s >= lengths_.back() || points_.size() == 1) return points_.back();
  auto end = std::upper_bound(lengths_.begin(), lengths_.end(), s);
  const auto i = static_cast<std::size_t>(end - lengths_.begin());
  const double ratio = (s - lengths_[i-1]) / (lengths_[i] - lengths_[i-1]);
  return points_[i-1] + ratio * (points_[i] - points_[i-1]);
}
Path ReferencePath::localSegment(const Eigen::Vector3d &position, double horizon) {
  if (empty() || !position.allFinite() || !std::isfinite(horizon) || horizon <= 0)
    throw std::invalid_argument("Invalid local path request");
  if (points_.size() == 1) return points_;
  double best = std::numeric_limits<double>::infinity(), selected = progress_;
  const double upper = initialized_ ? progress_ + horizon : lengths_.back();
  for (std::size_t i = 1; i < points_.size(); ++i) {
    if (lengths_[i] < progress_ || lengths_[i-1] > upper) continue;
    const Eigen::Vector3d delta = points_[i] - points_[i-1];
    const double fraction = std::clamp((position - points_[i-1]).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
    const double s = std::clamp(lengths_[i-1] + fraction * delta.norm(), progress_, std::min(upper, lengths_.back()));
    const double distance = (position - at(s)).squaredNorm();
    if (distance < best - 1e-10) { best = distance; selected = s; }
  }
  progress_ = std::max(progress_, selected); initialized_ = true;
  const double end = std::min(lengths_.back(), progress_ + horizon);
  Path segment{at(progress_)};
  for (std::size_t i = 1; i < points_.size(); ++i)
    if (lengths_[i] > progress_ && lengths_[i] < end) segment.push_back(points_[i]);
  if ((segment.back() - at(end)).norm() > 1e-6) segment.push_back(at(end));
  return segment;
}
Path resample(const Path &points, double spacing) {
  if (points.empty() || !std::isfinite(spacing) || spacing <= 0) return {};
  std::vector<double> lengths{0.0};
  for (std::size_t i = 1; i < points.size(); ++i) lengths.push_back(lengths.back() + (points[i] - points[i-1]).norm());
  if (lengths.back() < 1e-6) return {points.front()};
  const auto count = std::max<std::size_t>(6, static_cast<std::size_t>(std::ceil(lengths.back() / spacing)));
  if (count > 2000) throw std::invalid_argument("Local path is too long to resample");
  Path result; std::size_t j = 1;
  for (std::size_t i = 0; i <= count; ++i) {
    const double s = lengths.back() * i / count;
    while (j + 1 < points.size() && lengths[j] < s) ++j;
    const double ratio = (s - lengths[j-1]) / std::max(1e-9, lengths[j] - lengths[j-1]);
    result.push_back(points[j-1] + ratio * (points[j] - points[j-1]));
  }
  return result;
}
double distanceToPath(const Eigen::Vector3d &point, const Path &path) {
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    distance = std::min(distance, (point - path[i]).norm());
    if (i == 0) continue;
    const Eigen::Vector3d delta = path[i] - path[i-1];
    const double ratio = std::clamp((point - path[i-1]).dot(delta) / std::max(1e-12, delta.squaredNorm()), 0.0, 1.0);
    distance = std::min(distance, (point - path[i-1] - ratio * delta).norm());
  }
  return distance;
}
}  // namespace scan_planner
