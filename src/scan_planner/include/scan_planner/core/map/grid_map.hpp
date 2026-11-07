#pragma once
#include <Eigen/Core>
#include <memory>
#include <vector>
#include "scan_planner/core/parameters.hpp"

// Snapshot obstacle map used by the original projected A* and optimizer.
// A snapshot contains occupied points, not evidence of free-space rays.
class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;
  explicit GridMap(const scan_planner::MapParameters &parameters = {});
  void updateSnapshot(const std::vector<Eigen::Vector3d> &points,
                      const Eigen::Vector3d &center);
  bool isInMap(const Eigen::Vector3d &position) const;
  int getInflateOccupancy(const Eigen::Vector3d &position, double yaw) const;
  double getResolution() const { return parameters_.resolution; }
  Eigen::Vector3d getOrigin() const { return origin_; }
  Eigen::Vector3d size() const { return dimensions_.cast<double>() * parameters_.resolution; }
  std::size_t occupiedCount() const { return occupied_count_; }
private:
  bool index(const Eigen::Vector3d &point, Eigen::Vector3i &id) const;
  std::size_t address(const Eigen::Vector3i &id) const;
  int cylinder(const Eigen::Vector3d &center) const;
  scan_planner::MapParameters parameters_;
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3i dimensions_;
  std::vector<unsigned char> inflated_;
  std::vector<Eigen::Vector3i> offsets_;
  std::size_t occupied_count_{0};
};
