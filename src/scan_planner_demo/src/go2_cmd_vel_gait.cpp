#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace scan_planner_demo {
class Go2CmdVelGait final : public rclcpp::Node {
 public:
  Go2CmdVelGait() : Node("go2_cmd_vel_gait") {
    frequency_ = declare_parameter<double>("gait_frequency", 1.6);
    height_ = declare_parameter<double>("standing_height", 0.30);
    lift_ = declare_parameter<double>("swing_height", 0.07);
    max_vx_ = declare_parameter<double>("max_vx", 0.22);
    max_vy_ = declare_parameter<double>("max_vy", 0.10);
    max_wz_ = declare_parameter<double>("max_wz", 0.40);
    timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    if (!(frequency_ > 0.0 && height_ > 0.22 && height_ < 0.40 && lift_ >= 0.0 &&
          max_vx_ > 0.0 && max_vy_ > 0.0 && max_wz_ > 0.0 && timeout_ > 0.0))
      throw std::invalid_argument("Invalid Go2 gait parameters");
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
              !std::isfinite(message->angular.z)) {
            vx_ = vy_ = wz_ = 0.0;
            have_command_ = false;
            return;
          }
          vx_ = std::clamp(message->linear.x, -max_vx_, max_vx_);
          vy_ = std::clamp(message->linear.y, -max_vy_, max_vy_);
          wz_ = std::clamp(message->angular.z, -max_wz_, max_wz_);
          last_command_ = now();
          have_command_ = true;
        });
    joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kThigh = 0.213;
  static constexpr double kCalf = 0.213;
  static constexpr std::array<const char *, 12> kJoints = {
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  static std::array<double, 3> inverseLeg(double x, double y, double z) {
    // x / z are relative to the thigh joint; y is the lateral foot offset.
    const double hip = std::clamp(std::atan2(y, -z), -0.65, 0.65);
    const double planar_z = -std::hypot(y, z);
    const double r2 = x * x + planar_z * planar_z;
    const double c = std::clamp((r2 - kThigh * kThigh - kCalf * kCalf) /
                                    (2.0 * kThigh * kCalf), -0.98, 0.98);
    const double calf = -std::acos(c);
    const double thigh = std::atan2(-x, -planar_z) -
                          std::atan2(kCalf * std::sin(calf), kThigh + kCalf * c);
    return {hip, std::clamp(thigh, -1.2, 3.0), std::clamp(calf, -2.6, -0.9)};
  }

  void tick() {
    const auto stamp = now();
    if (stamp.nanoseconds() == 0) return;
    if (last_tick_.nanoseconds() != 0 && stamp < last_tick_) phase_ = 0.0;
    const double dt = last_tick_.nanoseconds() == 0 ? 0.0 :
        std::clamp((stamp - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = stamp;
    const double age = have_command_ ? (stamp - last_command_).seconds() : timeout_ + 1.0;
    const bool fresh = age >= 0.0 && age <= timeout_;
    const double vx = fresh ? vx_ : 0.0;
    const double vy = fresh ? vy_ : 0.0;
    const double wz = fresh ? wz_ : 0.0;
    const bool moving = std::hypot(vx, vy) > 0.015 || std::abs(wz) > 0.025;
    if (moving) phase_ = std::fmod(phase_ + frequency_ * dt, 1.0);
    else phase_ = 0.0;

    trajectory_msgs::msg::JointTrajectory message;
    message.header.stamp = stamp;
    message.joint_names.assign(kJoints.begin(), kJoints.end());
    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.positions.resize(12);
    target.time_from_start.sec = 0;
    target.time_from_start.nanosec = 60000000;
    for (std::size_t leg = 0; leg < 4; ++leg) {
      const double front = leg < 2 ? 1.0 : -1.0;
      const double side = leg % 2 == 0 ? 1.0 : -1.0;
      const double leg_phase = std::fmod(phase_ + (leg == 1 || leg == 2 ? 0.5 : 0.0), 1.0);
      const double effective_vx = vx - wz * side * 0.14;
      const double effective_vy = vy + wz * front * 0.1934;
      const double stride_time = 0.5 / frequency_;
      const double stride_x = std::clamp(effective_vx * stride_time, -0.09, 0.09);
      const double stride_y = std::clamp(effective_vy * stride_time, -0.05, 0.05);
      double x = 0.0, y = side * 0.015, z = -height_;
      if (moving && leg_phase < 0.5) {
        const double u = 2.0 * leg_phase;
        x = stride_x * (0.5 - u);
        y += stride_y * (0.5 - u);
      } else if (moving) {
        const double u = 2.0 * (leg_phase - 0.5);
        const double smooth = u * u * (3.0 - 2.0 * u);
        x = stride_x * (smooth - 0.5);
        y += stride_y * (smooth - 0.5);
        z += lift_ * std::sin(kPi * u);
      }
      const auto joints = inverseLeg(x, y, z);
      for (std::size_t axis = 0; axis < 3; ++axis)
        target.positions[3 * leg + axis] = joints[axis];
    }
    message.points.push_back(std::move(target));
    joint_pub_->publish(message);
  }

  double frequency_{1.6}, height_{0.30}, lift_{0.07};
  double max_vx_{0.22}, max_vy_{0.10}, max_wz_{0.40}, timeout_{0.3};
  double vx_{0.0}, vy_{0.0}, wz_{0.0}, phase_{0.0};
  bool have_command_{false};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_planner_demo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<scan_planner_demo::Go2CmdVelGait>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_cmd_vel_gait"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
