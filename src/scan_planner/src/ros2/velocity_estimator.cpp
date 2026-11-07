#include "velocity_estimator.hpp"
#include <cmath>
namespace scan_planner {
bool VelocityEstimator::update(const Eigen::Vector3d &position, double stamp, Eigen::Vector3d &velocity) {
  if (!position.allFinite() || !std::isfinite(stamp)) { reset(); return false; }
  if (!ready_) { position_=position; stamp_=stamp; ready_=true; velocity.setZero(); return false; }
  const double dt=stamp-stamp_;
  if (dt==0) return false;  // Repeated messages are not new observations.
  const Eigen::Vector3d raw=(position-position_)/dt;
  position_=position; stamp_=stamp;
  if (dt<0.001 || dt>0.5 || !raw.allFinite() || raw.norm()>max_speed_) {
    velocity_.setZero(); velocity.setZero(); return false;
  }
  velocity_=alpha_*raw+(1-alpha_)*velocity_; velocity=velocity_; return true;
}
}  // namespace scan_planner
