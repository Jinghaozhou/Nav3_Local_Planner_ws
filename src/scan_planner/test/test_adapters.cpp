#include <gtest/gtest.h>
#include "velocity_estimator.hpp"
#include "execution_guard.hpp"
using namespace scan_planner;
TEST(Velocity, DerivesOdomVelocityAndRejectsDuplicateJumpAndBackwardsTime) {
  VelocityEstimator estimator(1.0,3.0); Eigen::Vector3d velocity;
  EXPECT_FALSE(estimator.update({0,0,0},10.0,velocity));
  ASSERT_TRUE(estimator.update({0.1,0,0},10.1,velocity));
  EXPECT_NEAR(velocity.x(),1.0,1e-6);
  EXPECT_FALSE(estimator.update({0.1,0,0},10.1,velocity));
  EXPECT_FALSE(estimator.update({20,0,0},10.2,velocity));
  EXPECT_FALSE(estimator.update({20,0,0},5.0,velocity));
  EXPECT_FALSE(estimator.update({NAN,0,0},5.1,velocity));
}
TEST(Guard, UsesAcquisitionTimeAndRejectsFutureAndUnstampedSamples) {
  const rclcpp::Time now(100,0,RCL_ROS_TIME);
  EXPECT_TRUE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(99,800000000,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(98,0,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(102,0,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,builtin_interfaces::msg::Time(),0.5));
}
