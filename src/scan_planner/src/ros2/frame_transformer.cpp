#include "frame_transformer.hpp"
#include <stdexcept>
namespace scan_planner {
FrameTransformer::FrameTransformer(rclcpp::Node &node)
  : clock_(node.get_clock()), buffer_(node.get_clock()), listener_(buffer_) {}
Eigen::Isometry3d FrameTransformer::pose(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  Eigen::Vector3d position(p.position.x,p.position.y,p.position.z);
  if (!position.allFinite() || !q.coeffs().allFinite() || q.norm()<1e-6)
    throw std::invalid_argument("Invalid pose or quaternion");
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
  result.linear()=q.normalized().toRotationMatrix(); result.translation()=position;
  return result;
}
Eigen::Isometry3d FrameTransformer::lookup(const std::string &target, const std::string &source,
                                           const rclcpp::Time &stamp, double max_age) {
  if (target.empty() || source.empty()) throw std::invalid_argument("Empty TF frame");
  if (target==source) return Eigen::Isometry3d::Identity();
  const auto transform=buffer_.lookupTransform(target,source,stamp,rclcpp::Duration::from_seconds(0.0));
  const rclcpp::Time tf_stamp(transform.header.stamp,stamp.get_clock_type());
  const auto reference=stamp.nanoseconds()==0?clock_->now():stamp;
  if (max_age>0 && tf_stamp.nanoseconds()!=0 &&
      std::abs((reference-tf_stamp).seconds())>max_age) throw std::runtime_error("TF is too old");
  geometry_msgs::msg::Pose p; p.position.x=transform.transform.translation.x;
  p.position.y=transform.transform.translation.y; p.position.z=transform.transform.translation.z;
  p.orientation=transform.transform.rotation; return pose(p);
}
}  // namespace scan_planner
