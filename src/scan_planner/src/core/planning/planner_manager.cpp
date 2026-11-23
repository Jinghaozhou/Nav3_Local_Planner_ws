#include "scan_planner/core/planning/planner_manager.hpp"
#include "scan_planner/core/planning/reference_path.hpp"
#include <algorithm>
#include <stdexcept>

namespace scan_planner {
PlannerManager::PlannerManager(const MapParameters &map, const PlannerParameters &p,
                               const OptimizationParameters &optimization, bool check_local_collision)
  : map_(std::make_shared<GridMap>(map)), parameters_(p), check_local_collision_(check_local_collision) {
  if (!(p.horizon > 0 && p.point_spacing > 0 && p.max_velocity > 0 && p.max_acceleration > 0 &&
        p.reference_corridor > 0) || !std::isfinite(p.horizon + p.point_spacing +
        p.max_velocity + p.max_acceleration + p.reference_corridor +
        p.start_velocity_xy_deadband + p.start_velocity_z_deadband) ||
      p.start_velocity_xy_deadband < 0 || p.start_velocity_z_deadband < 0 ||
      p.start_velocity_xy_deadband >= p.max_velocity ||
      p.start_velocity_z_deadband >= p.max_velocity)
    throw std::invalid_argument("Invalid planning limits");
  optimizer_.setParam(optimization);
  optimizer_.setEnvironment(map_);
  optimizer_.a_star_ = std::make_shared<AStar>();
  optimizer_.a_star_->initGridMap(map_, Eigen::Vector3i(81, 81, 51));
}
bool PlannerManager::reached(const RobotState &state, const Eigen::Vector3d &goal) const {
  const Eigen::Vector3d error = state.position - goal;
  return error.head<2>().norm() <= parameters_.goal_xy_tolerance &&
    std::abs(error.z()) <= parameters_.goal_z_tolerance &&
    state.velocity.norm() <= parameters_.stopped_velocity;
}
bool PlannerManager::collisionFree(const Trajectory &trajectory, double from) const {
  return scan_planner::collisionFree(trajectory, *map_, parameters_.max_velocity, from);
}
bool collisionFree(const Trajectory &trajectory, const GridMap &map, double max_velocity, double from) {
  auto position = trajectory.position;
  if (!std::isfinite(trajectory.duration) || trajectory.duration <= 0) return false;
  const double dt = std::min(0.02, map.getResolution() / (4 * max_velocity));
  for (double t = std::max(0.0, from); t < trajectory.duration + dt; t += dt) {
    const double sample = std::min(t, trajectory.duration);
    const Eigen::Vector3d point = position.evaluateDeBoorT(sample);
    const Eigen::Vector3d next = position.evaluateDeBoorT(std::min(sample + dt, trajectory.duration));
    Eigen::Vector3d delta = next - point;
    // At the final sample next == point. Preserve the approach direction for
    // the yaw-dependent robot envelope instead of checking an arbitrary yaw 0.
    if (delta.head<2>().squaredNorm() < 1e-12 && sample > 0.0) {
      const Eigen::Vector3d previous = position.evaluateDeBoorT(std::max(0.0, sample - dt));
      delta = point - previous;
    }
    if (!point.allFinite() || map.getInflateOccupancy(point, std::atan2(delta.y(), delta.x())) != 0) return false;
  }
  return true;
}
PlanResult PlannerManager::plan(const RobotState &state, const Path &reference, const Path &cloud) {
  PlanResult result;
  if (reference.empty() || cloud.empty() || !state.position.allFinite() || !state.velocity.allFinite()) {
    result.reason = "Missing valid robot state, reference path or local map"; return result;
  }
  // In the explicit PCT-path experiment, keep map bounds but omit local obstacle points.
  map_->updateSnapshot(check_local_collision_ ? cloud : Path{}, state.position);
  if (distanceToPath(state.position, reference) > parameters_.reference_corridor) {
    result.reason = "Robot is outside the reference corridor"; return result;
  }
  if (map_->getInflateOccupancy(state.position, state.yaw()) != 0) {
    result.reason = "Robot envelope is occupied, unobserved, or outside map bounds"; return result;
  }
  Path seed{state.position};
  for (const auto &p : reference) {
    if (!p.allFinite()) { result.reason = "Non-finite reference"; return result; }
    if ((seed.back() - p).norm() > 1e-5) seed.push_back(p);
  }
  if (seed.size() < 2) { result.reason = "Reference has no remaining motion"; return result; }
  const auto samples = resample(seed, parameters_.point_spacing);
  double length = 0;
  for (std::size_t i=1; i<samples.size(); ++i) length += (samples[i] - samples[i-1]).norm();
  // Rest at every local endpoint; never extrapolate motion into an unplanned region.
  double interval = std::max(0.15, length / (samples.size()-1) / (parameters_.max_velocity * 0.6));
  Eigen::Vector3d start_velocity=state.velocity;
  if(start_velocity.head<2>().norm()<parameters_.start_velocity_xy_deadband)
    start_velocity.head<2>().setZero();
  if(std::abs(start_velocity.z())<parameters_.start_velocity_z_deadband)
    start_velocity.z()=0.0;
  if(start_velocity.norm()>parameters_.max_velocity+1e-3) {
    result.reason="Measured start velocity exceeds the configured planning limit";return result;
  }
  std::vector<Eigen::Vector3d> derivatives{start_velocity, Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  for(int attempt=0;attempt<8;++attempt) {
    Eigen::MatrixXd control_points;
    UniformBspline::parameterizeToBspline(interval, samples, derivatives, control_points);
    // The upstream least-squares fit only approximates endpoint constraints.
    // Fix the three boundary control points before optimizing the interior.
    control_points.col(0)=state.position-start_velocity*interval;
    control_points.col(1)=state.position;
    control_points.col(2)=state.position+start_velocity*interval;
    control_points.col(control_points.cols()-3)=samples.back();
    control_points.col(control_points.cols()-2)=samples.back();
    control_points.col(control_points.cols()-1)=samples.back();
    if (!control_points.allFinite()) { result.reason = "Invalid initial spline"; return result; }
    optimizer_.initControlPoints(control_points, true);
    if (!optimizer_.BsplineOptimizeTrajRebound(control_points, interval) || !control_points.allFinite()) {
      result.reason = "Projected A*/rebound optimization failed"; return result;
    }
    UniformBspline position(control_points, 3, interval);
    position.setPhysicalLimits(parameters_.max_velocity, parameters_.max_acceleration, 0.0);
    double ratio = 1.0;
    const bool component_feasible=position.checkFeasibility(ratio);
    const auto dynamic_ratio=[&]() {
      auto velocity=position.getDerivative(); auto acceleration=velocity.getDerivative();
      const auto v=velocity.getControlPoint(); const auto a=acceleration.getControlPoint();
      if(!v.allFinite()||!a.allFinite()) return std::numeric_limits<double>::infinity();
      // Derivative curves remain inside their control-point convex hulls.
      // Bounding vector norms here covers the entire curve, between samples too.
      return std::max({1.0,v.colwise().norm().maxCoeff()/parameters_.max_velocity,
        std::sqrt(a.colwise().norm().maxCoeff()/parameters_.max_acceleration)});
    };
    ratio=std::max(component_feasible?1.0:ratio,dynamic_ratio());
    if(!std::isfinite(ratio)) {result.reason="Non-finite trajectory derivatives";return result;}
    if(!component_feasible||ratio>1.001) {
      // Refit ALL knot intervals and reapply the measured boundary velocity.
      // Stretching only interior knots cannot cure an infeasible first span;
      // simply scaling all knots would silently change the measured velocity.
      interval*=std::max(1.15,ratio*1.05);
      continue;
    }
    result.trajectory = {position, position.getTimeSum()};
    if (!collisionFree(result.trajectory)) { result.reason = "Final whole trajectory is in collision"; return result; }
    for (double t=0; t<result.trajectory.duration; t+=0.05) {
      const Eigen::Vector3d sample = position.evaluateDeBoorT(t);
      if (distanceToPath(sample, seed) > parameters_.reference_corridor) {
        result.reason = "Optimized trajectory left the reference corridor"; return result;
      }
    }
    result.success = true; result.reason = "Local trajectory validated";
    return result;
  }
  result.reason="Trajectory remained dynamically infeasible after time refitting";
  return result;
}
}  // namespace scan_planner
