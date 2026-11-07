#include "scan_planner/core/control/trajectory_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace scan_planner {
VelocityCommand track(const RobotState &state, Trajectory trajectory, double time,
                      const TrackerParameters &p) {
  VelocityCommand command;
  if (!std::isfinite(time) || time < 0 || time >= trajectory.duration) return command;
  const Eigen::Vector3d desired = trajectory.position.evaluateDeBoorT(time);
  const Eigen::Vector3d future = trajectory.position.evaluateDeBoorT(std::min(time + p.lookahead, trajectory.duration));
  auto velocity = trajectory.position.getDerivative();
  Eigen::Vector3d direction = future - desired;
  if (direction.head<2>().norm() < 1e-4) direction = velocity.evaluateDeBoorT(time);
  const double yaw = state.yaw();
  const double desired_yaw = direction.head<2>().norm() < 1e-4 ? yaw : std::atan2(direction.y(), direction.x());
  const double error = std::remainder(desired_yaw - yaw, 2 * M_PI);
  command.yaw_rate = std::clamp(p.kp_yaw * error, -p.max_yaw_rate, p.max_yaw_rate);
  command.frozen = std::abs(error) > p.heading_threshold;
  if (command.frozen) return command;
  const Eigen::Vector3d world_velocity = velocity.evaluateDeBoorT(time) + p.kp_position * (desired - state.position);
  command.vx = std::clamp(std::cos(yaw)*world_velocity.x() + std::sin(yaw)*world_velocity.y(), -p.max_vx, p.max_vx);
  command.vy = std::clamp(-std::sin(yaw)*world_velocity.x() + std::cos(yaw)*world_velocity.y(), -p.max_vy, p.max_vy);
  return command;
}
}  // namespace scan_planner
