#pragma once
#include <Eigen/Core>

namespace scan_planner {
struct MapParameters {
  double resolution{0.1};
  Eigen::Vector3d size{10.0, 10.0, 5.0};
  double radius{0.25}, cylinder_offset{0.18};
  // Robot extent above/below its planning reference, not obstacle extents.
  double body_above{0.1}, body_below{0.4};
  bool allow_unobserved_space{false};
};
struct OptimizationParameters {
  double smoothness{1.0}, collision{2.0}, feasibility{0.1}, fitness{1.0};
  double clearance{0.2}, max_velocity{0.75}, max_acceleration{0.5};
};
struct PlannerParameters {
  double horizon{3.0}, point_spacing{0.15}, reference_corridor{0.8};
  double max_velocity{0.75}, max_acceleration{0.5};
  double goal_xy_tolerance{0.15}, goal_z_tolerance{0.15}, stopped_velocity{0.1};
};
struct TrackerParameters {
  double kp_position{0.8}, kp_yaw{1.5}, lookahead{0.8};
  double max_vx{0.75}, max_vy{0.35}, max_yaw_rate{1.0};
  double heading_threshold{0.8};
};
}  // namespace scan_planner
