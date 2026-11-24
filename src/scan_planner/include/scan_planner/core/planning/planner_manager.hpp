#pragma once
#include "scan_planner/core/map/grid_map.hpp"
#include "scan_planner/core/optimization/bspline_optimizer.hpp"
#include "scan_planner/core/types.hpp"

namespace scan_planner {
bool collisionFree(const Trajectory &trajectory, const GridMap &map,
                   double max_velocity, double from = 0.0,
                   Eigen::Vector3d *first_collision = nullptr, double *collision_time = nullptr);
class PlannerManager {
public:
  PlannerManager(const MapParameters &map, const PlannerParameters &planner,
                 const OptimizationParameters &optimization, bool check_local_collision = true);
  PlanResult plan(const RobotState &state, const Path &local_reference, const Path &cloud);
  bool collisionFree(const Trajectory &trajectory, double from = 0.0) const;
  bool reached(const RobotState &state, const Eigen::Vector3d &goal) const;
  GridMap::Ptr map() const { return map_; }
private:
  GridMap::Ptr map_;
  PlannerParameters parameters_;
  bool check_local_collision_;
  BsplineOptimizer optimizer_;
};
}  // namespace scan_planner
