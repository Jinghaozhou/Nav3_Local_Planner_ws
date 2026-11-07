#pragma once
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scan_planner {
class FrameTransformer {
public:
  explicit FrameTransformer(rclcpp::Node &node);
  Eigen::Isometry3d lookup(const std::string &target, const std::string &source,
                           const rclcpp::Time &stamp, double max_age);
  static Eigen::Isometry3d pose(const geometry_msgs::msg::Pose &pose);
private:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};
}  // namespace scan_planner
