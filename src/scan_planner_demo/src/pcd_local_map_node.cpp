#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace scan_planner_demo {
class PcdLocalMapNode final : public rclcpp::Node {
 public:
  PcdLocalMapNode() : Node("pcd_local_map") {
    const auto file = declare_parameter<std::string>("pcd_file", "");
    frame_ = declare_parameter<std::string>("frame_id", "odom");
    const auto leaf = declare_parameter<double>("downsample_resolution", 0.1);
    half_xy_ = declare_parameter<double>("crop_half_xy", 4.5);
    half_z_ = declare_parameter<double>("crop_half_z", 2.0);
    const auto rate = declare_parameter<double>("publish_rate", 4.0);
    const auto bounds = declare_parameter<std::vector<double>>(
        "world_bounds", std::vector<double>{});
    if (file.empty() || leaf <= 0.0 || half_xy_ <= 0.0 || half_z_ <= 0.0 || rate <= 0.0)
      throw std::invalid_argument("PCD file and positive map parameters are required");
    if (!bounds.empty() && (bounds.size() != 6 ||
        !std::all_of(bounds.begin(), bounds.end(), [](double v) { return std::isfinite(v); }) ||
        bounds[0] >= bounds[1] || bounds[2] >= bounds[3] || bounds[4] >= bounds[5]))
      throw std::invalid_argument("world_bounds must be [xmin, xmax, ymin, ymax, zmin, zmax]");

    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(file, *loaded) != 0)
      throw std::runtime_error("Cannot load PCD file: " + file);
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr finite(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*loaded, *finite, indices);
    loaded.reset();
    if (!bounds.empty()) {
      auto &points = finite->points;
      points.erase(std::remove_if(points.begin(), points.end(), [&bounds](const auto &point) {
        return point.x < bounds[0] || point.x > bounds[1] ||
               point.y < bounds[2] || point.y > bounds[3] ||
               point.z < bounds[4] || point.z > bounds[5];
      }), points.end());
      finite->width = static_cast<std::uint32_t>(points.size());
      finite->height = 1;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(finite);
    voxel.setLeafSize(leaf, leaf, leaf);
    map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*map_);
    finite.reset();
    if (map_->empty()) throw std::runtime_error("PCD contains no finite XYZ points");
    tree_.setInputCloud(map_);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", rclcpp::SensorDataQoS());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          if (message->header.frame_id != frame_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "Odometry frame '%s' differs from PCD frame '%s'",
                message->header.frame_id.c_str(), frame_.c_str());
            have_odom_ = false;
            return;
          }
          const auto &p = message->pose.pose.position;
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
          center_.x = static_cast<float>(p.x);
          center_.y = static_cast<float>(p.y);
          center_.z = static_cast<float>(p.z);
          have_odom_ = true;
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { publish(); });
    RCLCPP_INFO(get_logger(), "Loaded %zu downsampled PCD points from %s", map_->size(), file.c_str());
  }

 private:
  void publish() {
    if (!have_odom_ || now().nanoseconds() == 0) return;
    std::vector<int> candidates;
    std::vector<float> distances;
    tree_.radiusSearch(center_, std::sqrt(2.0 * half_xy_ * half_xy_ + half_z_ * half_z_),
                       candidates, distances);
    pcl::PointCloud<pcl::PointXYZ> local;
    local.reserve(candidates.size());
    for (const auto index : candidates) {
      const auto &point = map_->points[index];
      if (std::abs(point.x - center_.x) <= half_xy_ &&
          std::abs(point.y - center_.y) <= half_xy_ &&
          std::abs(point.z - center_.z) <= half_z_) local.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(local, message);
    message.header.stamp = now();
    message.header.frame_id = frame_;
    cloud_pub_->publish(message);
  }

  std::string frame_;
  double half_xy_{4.5}, half_z_{2.0};
  bool have_odom_{false};
  pcl::PointXYZ center_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_planner_demo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<scan_planner_demo::PcdLocalMapNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_local_map"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
