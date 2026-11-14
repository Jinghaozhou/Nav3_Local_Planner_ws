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
bool TwistVelocityFilter::update(const Eigen::Vector3d &measurement, double stamp,
                                 Eigen::Vector3d &velocity) {
  if (!measurement.allFinite() || !std::isfinite(stamp)) { reset(); return false; }
  if (time_constant_==0.0) { velocity=measurement; return true; }
  if (!ready_) { velocity_=measurement; stamp_=stamp; ready_=true; velocity=velocity_; return true; }
  const double dt=stamp-stamp_;
  // Gazebo may repeat a sample timestamp. Keep the last estimate instead of
  // briefly revoking planning permission; freshness is checked separately.
  if (dt==0.0) { velocity=velocity_; return true; }
  if (dt<0.0) { reset(); stamp_=stamp; velocity_=measurement; ready_=true; return false; }
  if (dt>0.5) { velocity_=measurement; stamp_=stamp; velocity=velocity_; return true; }
  const double alpha=-std::expm1(-dt/time_constant_);
  velocity_+=alpha*(measurement-velocity_);
  stamp_=stamp;
  velocity=velocity_;
  return true;
}
}  // namespace scan_planner
