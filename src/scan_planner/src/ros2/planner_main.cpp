#include "planner_node.hpp"
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<scan_planner::PlannerNode>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("scan_planner"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
