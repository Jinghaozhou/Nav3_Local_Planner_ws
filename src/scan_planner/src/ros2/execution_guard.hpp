#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <string>

namespace scan_planner {
inline bool fresh(const rclcpp::Time &now, const builtin_interfaces::msg::Time &stamp, double timeout) {
  const rclcpp::Time sample(stamp,now.get_clock_type());
  const double age=(now-sample).seconds();
  return sample.nanoseconds()>0 && std::isfinite(age) && age>=-0.1 && age<=timeout;
}
}  // namespace scan_planner
