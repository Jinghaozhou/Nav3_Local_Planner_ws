// ROS 2 demo adapter for the CHAMP gait library (BSD-3-Clause).
// The upstream library and its copyright notices are in third_party/champ.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <champ/body_controller/body_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/utils/urdf_loader.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>

namespace scan_planner_demo {
class Go2ChampGait final : public rclcpp::Node {
 public:
  Go2ChampGait()
      : Node("go2_champ_gait"), body_controller_(base_),
        leg_controller_(base_, 0), kinematics_(base_) {
    const auto positive = [this](const char *name, double fallback) {
      const double value = declare_parameter<double>(name, fallback);
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) + " must be positive and finite");
      return value;
    };
    const double loop_rate = positive("loop_rate", 50.0);
    const double trajectory_duration = positive("trajectory_duration", 1.0 / 60.0);
    const double cmd_timeout = positive("cmd_timeout", 0.3);
    if (loop_rate > 200.0 || trajectory_duration > 0.2 || cmd_timeout > 2.0)
      throw std::invalid_argument("Invalid CHAMP gait timing");
    timeout_ = cmd_timeout;
    trajectory_duration_ = trajectory_duration;

    knee_orientation_ = declare_parameter<std::string>("gait.knee_orientation", ">>");
    if (knee_orientation_.size() != 2 ||
        knee_orientation_.find_first_not_of("<>") != std::string::npos)
      throw std::invalid_argument("gait.knee_orientation must contain two '<' or '>' characters");
    config_.knee_orientation = knee_orientation_.c_str();
    config_.pantograph_leg = declare_parameter<bool>("gait.pantograph_leg", false);
    config_.max_linear_velocity_x = positive("gait.max_linear_velocity_x", 0.22);
    config_.max_linear_velocity_y = positive("gait.max_linear_velocity_y", 0.10);
    config_.max_angular_velocity_z = positive("gait.max_angular_velocity_z", 0.40);
    config_.swing_height = positive("gait.swing_height", 0.04);
    config_.stance_duration = positive("gait.stance_duration", 0.25);
    config_.nominal_height = positive("gait.nominal_height", 0.30);
    const double stance_depth = declare_parameter<double>("gait.stance_depth", 0.01);
    const double com_x = declare_parameter<double>("gait.com_x_translation", 0.0);
    if (!std::isfinite(stance_depth) || stance_depth < 0.0 ||
        !std::isfinite(com_x) || std::abs(com_x) > 0.2)
      throw std::invalid_argument("Invalid CHAMP stance/CoM parameters");
    config_.stance_depth = stance_depth;
    config_.com_x_translation = com_x;
    base_.setGaitConfig(config_);
    pose_.position.z = config_.nominal_height;

    const std::array<std::string, 4> leg_labels = {
        "left_front", "right_front", "left_hind", "right_hind"};
    const std::array<std::string, 4> prefixes = {"FL", "FR", "RL", "RR"};
    declare_parameter<std::string>("links_map.base", "base");
    for (std::size_t i = 0; i < leg_labels.size(); ++i) {
      const auto &prefix = prefixes[i];
      declare_parameter<std::vector<std::string>>(
          "links_map." + leg_labels[i],
          {prefix + "_hip", prefix + "_thigh", prefix + "_calf", prefix + "_foot"});
      declare_parameter<std::vector<std::string>>(
          "joints_map." + leg_labels[i],
          {prefix + "_hip_joint", prefix + "_thigh_joint", prefix + "_calf_joint",
           prefix + "_foot_fixed"});
    }
    const std::string description = declare_parameter<std::string>("robot_description", "");
    urdf::Model model;
    if (!model.initString(description))
      throw std::invalid_argument("CHAMP gait needs a valid robot_description");
    champ::URDF::loadFromString(base_, get_node_parameters_interface(), description);
    joint_names_ = champ::URDF::getJointNames(get_node_parameters_interface());
    if (joint_names_.size() != 12)
      throw std::invalid_argument("CHAMP gait needs twelve leg joints");

    const std::string topic = declare_parameter<std::string>(
        "joint_controller_topic", "/joint_trajectory_controller/joint_trajectory");
    joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        topic, rclcpp::QoS(10).reliable());
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
              !std::isfinite(message->angular.z)) {
            have_command_ = false;
            return;
          }
          vx_ = std::clamp(message->linear.x, -double(config_.max_linear_velocity_x),
                           double(config_.max_linear_velocity_x));
          vy_ = std::clamp(message->linear.y, -double(config_.max_linear_velocity_y),
                           double(config_.max_linear_velocity_y));
          wz_ = std::clamp(message->angular.z, -double(config_.max_angular_velocity_z),
                           double(config_.max_angular_velocity_z));
          last_command_ = now();
          have_command_ = true;
        });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / loop_rate));
    timer_ = create_wall_timer(period, [this] { tick(); });
  }

 private:
  void tick() {
    const auto stamp = now();
    if (stamp.nanoseconds() == 0) return;
    const double age = have_command_ ? (stamp - last_command_).seconds() : timeout_ + 1.0;
    const bool fresh = age >= 0.0 && age <= timeout_;
    champ::Velocities desired;
    if (fresh) {
      desired.linear.x = vx_;
      desired.linear.y = vy_;
      desired.angular.z = wz_;
    }
    geometry::Transformation feet[4];
    body_controller_.poseCommand(feet, pose_);
    leg_controller_.velocityCommand(feet, desired,
                                    static_cast<champ::PhaseGenerator::Time>(stamp.nanoseconds() / 1000));
    float joints[12];
    std::fill(std::begin(joints), std::end(joints),
              std::numeric_limits<float>::quiet_NaN());
    kinematics_.inverse(joints, feet);
    if (!std::all_of(std::begin(joints), std::end(joints),
                     [](float joint) { return std::isfinite(joint) && std::abs(joint) < 3.14f; })) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "CHAMP returned an invalid joint target; holding the last valid posture");
      if (!have_valid_joints_) return;
      std::copy(last_joints_.begin(), last_joints_.end(), joints);
    } else {
      std::copy(std::begin(joints), std::end(joints), last_joints_.begin());
      have_valid_joints_ = true;
    }
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(std::begin(joints), std::end(joints));
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_duration_);
    trajectory.points.push_back(std::move(point));
    joint_pub_->publish(trajectory);
  }

  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;
  champ::GaitConfig config_;
  champ::Pose pose_;
  std::string knee_orientation_;
  std::vector<std::string> joint_names_;
  std::array<float, 12> last_joints_{};
  bool have_valid_joints_{false};
  bool have_command_{false};
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  double timeout_{0.3}, trajectory_duration_{1.0 / 60.0};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_planner_demo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<scan_planner_demo::Go2ChampGait>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_champ_gait"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
