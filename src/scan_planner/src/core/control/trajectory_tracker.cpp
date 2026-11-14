#include "scan_planner/core/control/trajectory_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace scan_planner {
VelocityCommand track(const RobotState &state, Trajectory trajectory, double time,
                      const TrackerParameters &p) {
  VelocityCommand command;
  if (!std::isfinite(time) || time < 0 || time >= trajectory.duration) return command;
  const Eigen::Vector3d desired = trajectory.position.evaluateDeBoorT(time);
  const double preview_time=std::min(time+p.lookahead,trajectory.duration);
  Eigen::Vector3d future=trajectory.position.evaluateDeBoorT(preview_time);
  auto velocity = trajectory.position.getDerivative();
  Eigen::Vector3d direction = future - desired;
  // A few centimetres of contact motion are too short to define a useful
  // heading. Search farther along the same collision-checked trajectory.
  if(p.heading_preview_distance>0 && direction.head<2>().norm()<p.heading_preview_distance) {
    double travelled=direction.head<2>().norm();
    const double step=std::max(0.05,(trajectory.duration-preview_time)/100.0);
    for(double t=preview_time+step; t<trajectory.duration+step*0.5; t+=step) {
      const double sample_time=std::min(t,trajectory.duration);
      const Eigen::Vector3d next=trajectory.position.evaluateDeBoorT(sample_time);
      travelled+=(next-future).head<2>().norm();
      future=next;
      direction=future-desired;
      if(travelled>=p.heading_preview_distance &&
         direction.head<2>().norm()>=0.5*p.heading_preview_distance) break;
      if(sample_time>=trajectory.duration) break;
    }
  }
  if (direction.head<2>().norm() < 1e-4) direction = velocity.evaluateDeBoorT(time);
  const double yaw = state.yaw();
  const double desired_yaw = direction.head<2>().norm() < 1e-4 ? yaw : std::atan2(direction.y(), direction.x());
  const double error = std::remainder(desired_yaw - yaw, 2 * M_PI);
  command.yaw_rate = std::clamp(p.kp_yaw * error, -p.max_yaw_rate, p.max_yaw_rate);
  command.frozen = std::abs(error) > p.heading_threshold;
  if (command.frozen) return command;
  Eigen::Vector3d world_velocity = velocity.evaluateDeBoorT(time) + p.kp_position * (desired - state.position);
  // Receding-horizon replans may replace a zero-speed spline before its first
  // acceleration span is executed. A small path-relative preview term gives
  // the physical gait a forward command while the temporal target catches up.
  world_velocity.head<2>() += p.preview_position_gain * direction.head<2>();
  command.vx = std::clamp(std::cos(yaw)*world_velocity.x() + std::sin(yaw)*world_velocity.y(), -p.max_vx, p.max_vx);
  command.vy = std::clamp(-std::sin(yaw)*world_velocity.x() + std::cos(yaw)*world_velocity.y(), -p.max_vy, p.max_vy);
  return command;
}
}  // namespace scan_planner
