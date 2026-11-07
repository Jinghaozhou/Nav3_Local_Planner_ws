#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>
#include "scan_planner/core/trajectory/uniform_bspline.hpp"

namespace scan_planner {
using Path = std::vector<Eigen::Vector3d>;
struct RobotState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double yaw() const {
    const auto heading = orientation.toRotationMatrix().col(0).eval();
    return std::atan2(heading.y(), heading.x());
  }
};
struct Trajectory {
  UniformBspline position;
  double duration{0.0};
};
struct PlanResult {
  bool success{false};
  std::string reason;
  Trajectory trajectory;
};
struct VelocityCommand {
  double vx{0.0}, vy{0.0}, yaw_rate{0.0};
  bool frozen{false};
};
}  // namespace scan_planner
