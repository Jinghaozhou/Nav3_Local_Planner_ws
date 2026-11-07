#include "controller_node.hpp"
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<scan_planner::ControllerNode>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("scan_controller"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
