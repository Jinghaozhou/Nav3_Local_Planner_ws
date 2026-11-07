#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace scan_planner {
class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ~ControllerNode() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
