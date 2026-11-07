#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace scan_planner {
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();
  ~PlannerNode() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
