#pragma once
#include "frame_transformer.hpp"
#include "velocity_estimator.hpp"
#include "scan_planner/core/types.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace scan_planner {
class InputAdapter {
public:
  InputAdapter(rclcpp::Node &node, FrameTransformer &frames);
  bool odometry(const nav_msgs::msg::Odometry &message, RobotState &state);
  Path cloud(const sensor_msgs::msg::PointCloud2 &message);
  const std::string &planningFrame() const { return planning_frame_; }
  const std::string &globalFrame() const { return global_frame_; }
  const std::string &robotFrame() const { return robot_frame_; }
private:
  FrameTransformer &frames_;
  std::string planning_frame_, global_frame_, robot_frame_, velocity_source_;
  double tf_age_;
  VelocityEstimator estimator_;
  TwistVelocityFilter twist_filter_;
};
}  // namespace scan_planner
