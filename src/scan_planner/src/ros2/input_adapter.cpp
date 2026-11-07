#include "input_adapter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <cstring>

namespace scan_planner {
InputAdapter::InputAdapter(rclcpp::Node &node,FrameTransformer &frames)
  : frames_(frames), estimator_(node.declare_parameter<double>("velocity_filter_alpha",0.35),
                                node.declare_parameter<double>("velocity_max_speed",3.0)) {
  planning_frame_=node.declare_parameter<std::string>("planning_frame","odom");
  global_frame_=node.declare_parameter<std::string>("global_frame","map");
  robot_frame_=node.declare_parameter<std::string>("robot_frame","base_link");
  velocity_source_=node.declare_parameter<std::string>("velocity_source","pose_difference");
  tf_age_=node.declare_parameter<double>("tf_max_age",0.5);
  const double alpha=node.get_parameter("velocity_filter_alpha").as_double();
  const double speed=node.get_parameter("velocity_max_speed").as_double();
  if ((velocity_source_!="pose_difference" && velocity_source_!="odometry_twist") ||
      !(alpha>0 && alpha<=1 && speed>0 && tf_age_>0)) throw std::invalid_argument("Invalid velocity/TF parameters");
}
bool InputAdapter::odometry(const nav_msgs::msg::Odometry &m,RobotState &state) {
  const rclcpp::Time stamp(m.header.stamp,RCL_ROS_TIME);
  if (m.child_frame_id.empty()) throw std::invalid_argument("Odometry child_frame_id is empty");
  const auto parent=frames_.lookup(planning_frame_,m.header.frame_id,stamp,tf_age_);
  const auto measured=parent*FrameTransformer::pose(m.pose.pose);
  const auto offset=frames_.lookup(m.child_frame_id,robot_frame_,stamp,tf_age_);
  const auto robot=measured*offset;
  state.position=robot.translation(); state.orientation=Eigen::Quaterniond(robot.linear());
  if (velocity_source_=="pose_difference") return estimator_.update(state.position,stamp.seconds(),state.velocity);
  const Eigen::Vector3d linear(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z);
  const Eigen::Vector3d angular(m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z);
  if (!linear.allFinite() || !angular.allFinite()) throw std::invalid_argument("Invalid odometry twist");
  state.velocity=measured.linear()*(linear+angular.cross(offset.translation()));
  return true;
}
Path InputAdapter::cloud(const sensor_msgs::msg::PointCloud2 &m) {
  if (m.point_step==0 || m.width==0 || m.height==0 ||
      m.row_step<static_cast<uint64_t>(m.width)*m.point_step ||
      m.data.size()<static_cast<uint64_t>(m.height)*m.row_step ||
      static_cast<uint64_t>(m.width)*m.height>5000000 || m.is_bigendian)
    throw std::invalid_argument("Empty, oversized, big-endian or malformed local map");
  std::size_t offsets[3]{};
  for(int axis=0;axis<3;++axis) {
    unsigned count=0;
    for(const auto &field:m.fields) {
      if(field.name!=std::string(1,"xyz"[axis])) continue;
      ++count;
      if(field.datatype!=sensor_msgs::msg::PointField::FLOAT32 || field.count!=1 ||
         static_cast<uint64_t>(field.offset)+4>m.point_step)
        throw std::invalid_argument("Local map requires scalar FLOAT32 x/y/z");
      offsets[axis]=field.offset;
    }
    if(count!=1) throw std::invalid_argument("Local map coordinates must have exactly one field each");
  }
  const auto transform=frames_.lookup(planning_frame_,m.header.frame_id,rclcpp::Time(m.header.stamp,RCL_ROS_TIME),tf_age_);
  Path points; points.reserve(static_cast<std::size_t>(m.width)*m.height);
  // Respect organized row padding instead of assuming tightly packed storage.
  for(uint32_t row=0;row<m.height;++row) for(uint32_t col=0;col<m.width;++col) {
    const auto *data=m.data.data()+static_cast<std::size_t>(row)*m.row_step+static_cast<std::size_t>(col)*m.point_step;
    float values[3]; for(int i=0;i<3;++i) std::memcpy(&values[i],data+offsets[i],4);
    Eigen::Vector3d p(values[0],values[1],values[2]); if(p.allFinite()) points.push_back(transform*p);
  }
  if(points.empty()) throw std::invalid_argument("Local map contains no finite points");
  return points;
}
}  // namespace scan_planner
