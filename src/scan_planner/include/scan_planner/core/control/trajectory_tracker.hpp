#pragma once
#include "scan_planner/core/types.hpp"
#include "scan_planner/core/parameters.hpp"
namespace scan_planner {
VelocityCommand track(const RobotState &state, Trajectory trajectory, double time,
                      const TrackerParameters &parameters);
}
