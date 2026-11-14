#include <gtest/gtest.h>
#include <cmath>
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
TEST(Velocity, TwistFilterSuppressesShortPulseButKeepsSustainedMotion) {
  TwistVelocityFilter filter;
  Eigen::Vector3d velocity;
  filter.setTimeConstant(0.3);
  ASSERT_TRUE(filter.update({0.18,0,0},1.0,velocity));
  ASSERT_TRUE(filter.update({0.32,0,-0.24},1.02,velocity));
  EXPECT_LT(velocity.norm(),0.22);
  const Eigen::Vector3d before_duplicate=velocity;
  ASSERT_TRUE(filter.update({0.9,0,0},1.02,velocity));
  EXPECT_NEAR((velocity-before_duplicate).norm(),0.0,1e-9);
  for(int i=2;i<=60;++i)
    ASSERT_TRUE(filter.update({0.28,0,0},1.0+0.02*i,velocity));
  EXPECT_GT(velocity.x(),0.26);
  EXPECT_FALSE(filter.update({0.28,0,0},1.5,velocity));
  filter.setTimeConstant(0.0);
  ASSERT_TRUE(filter.update({0.3,0,-0.2},2.0,velocity));
  EXPECT_NEAR(velocity.norm(),std::sqrt(0.13),1e-6);
}
TEST(Guard, UsesAcquisitionTimeAndRejectsFutureAndUnstampedSamples) {
  const rclcpp::Time now(100,0,RCL_ROS_TIME);
  EXPECT_TRUE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(99,800000000,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(98,0,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(102,0,RCL_ROS_TIME)),0.5));
  EXPECT_FALSE(fresh(now,builtin_interfaces::msg::Time(),0.5));
}
