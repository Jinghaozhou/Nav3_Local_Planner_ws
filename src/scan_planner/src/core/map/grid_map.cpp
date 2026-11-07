#include "scan_planner/core/map/grid_map.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

GridMap::GridMap(const scan_planner::MapParameters &p) : parameters_(p) {
  if (!std::isfinite(p.resolution) || p.resolution < 0.02 ||
      !p.size.allFinite() || (p.size.array() <= 0).any() ||
      !std::isfinite(p.radius) || p.radius < 0 ||
      !std::isfinite(p.cylinder_offset) || p.cylinder_offset < 0 ||
      !std::isfinite(p.body_above) || p.body_above < 0 ||
      !std::isfinite(p.body_below) || p.body_below < 0 ||
      p.radius + p.cylinder_offset >= p.size.head<2>().minCoeff() * 0.5 ||
      p.body_above + p.body_below >= p.size.z())
    throw std::invalid_argument("Invalid map dimensions or robot envelope");
  const Eigen::Array3d counts = (p.size / p.resolution).array().ceil();
  if (counts.prod() > 20000000.0) throw std::invalid_argument("Map exceeds 20 million voxels");
  dimensions_ = counts.cast<int>();
  inflated_.resize(static_cast<std::size_t>(dimensions_.prod()));
  const int radius = static_cast<int>(std::ceil(p.radius / p.resolution)) + 1;
  // Minkowski sum: an obstacle below the robot forbids a reference ABOVE it.
  const int down = static_cast<int>(std::ceil(p.body_above / p.resolution));
  const int up = static_cast<int>(std::ceil(p.body_below / p.resolution));
  for (int x = -radius; x <= radius; ++x)
    for (int y = -radius; y <= radius; ++y) {
      // Both the obstacle and the query may lie anywhere within their voxels.
      // Inflate using the minimum distance between the two cells, rather than
      // assuming both positions are at voxel centers.
      if (std::hypot(std::max(0,std::abs(x)-1),std::max(0,std::abs(y)-1)) * p.resolution > p.radius) continue;
      for (int z = -down; z <= up; ++z) offsets_.emplace_back(x, y, z);
    }
}

bool GridMap::index(const Eigen::Vector3d &p, Eigen::Vector3i &id) const {
  if (!p.allFinite()) return false;
  const auto local = ((p - origin_) / parameters_.resolution).array();
  if ((local < 0).any() || (local >= dimensions_.cast<double>().array()).any()) return false;
  id = local.floor().cast<int>();
  return true;
}
std::size_t GridMap::address(const Eigen::Vector3i &id) const {
  return (static_cast<std::size_t>(id.x()) * dimensions_.y() + id.y()) * dimensions_.z() + id.z();
}
bool GridMap::isInMap(const Eigen::Vector3d &p) const {
  Eigen::Vector3i id;
  return index(p, id);
}
void GridMap::updateSnapshot(const std::vector<Eigen::Vector3d> &points,
                             const Eigen::Vector3d &center) {
  if (!center.allFinite()) throw std::invalid_argument("Non-finite map center");
  origin_ = ((center - size() * 0.5) / parameters_.resolution).array().floor().matrix() * parameters_.resolution;
  std::fill(inflated_.begin(), inflated_.end(), 0);
  occupied_count_ = 0;
  std::vector<unsigned char> seen(inflated_.size(), 0);
  for (const auto &point : points) {
    Eigen::Vector3i id;
    if (!index(point, id) || seen[address(id)]) continue;
    seen[address(id)] = 1;
    ++occupied_count_;
    for (const auto &offset : offsets_) {
      const Eigen::Vector3i cell = id + offset;
      if ((cell.array() >= 0).all() && (cell.array() < dimensions_.array()).all())
        inflated_[address(cell)] = 1;
    }
  }
}
int GridMap::cylinder(const Eigen::Vector3d &p) const {
  // The whole envelope must fit inside the configured snapshot bounds.
  if (!isInMap(p + Eigen::Vector3d(parameters_.radius, parameters_.radius, parameters_.body_above)) ||
      !isInMap(p - Eigen::Vector3d(parameters_.radius, parameters_.radius, parameters_.body_below))) return -1;
  Eigen::Vector3i id;
  if (!index(p, id)) return -1;
  if (inflated_[address(id)]) return 1;
  return parameters_.allow_unobserved_space ? 0 : -1;
}
int GridMap::getInflateOccupancy(const Eigen::Vector3d &p, double yaw) const {
  if (!std::isfinite(yaw)) return -1;
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  const int front = cylinder(p + parameters_.cylinder_offset * heading);
  return front != 0 ? front : cylinder(p - parameters_.cylinder_offset * heading);
}
