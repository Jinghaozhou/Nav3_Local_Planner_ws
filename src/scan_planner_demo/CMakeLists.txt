cmake_minimum_required(VERSION 3.16)
project(scan_planner_demo)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(scan_planner REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(pcl_conversions REQUIRED)

add_executable(pcd_local_map_node src/pcd_local_map_node.cpp)
target_include_directories(pcd_local_map_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_local_map_node ${PCL_LIBRARIES})
ament_target_dependencies(pcd_local_map_node rclcpp nav_msgs sensor_msgs pcl_conversions)

add_executable(go2_cmd_vel_gait src/go2_cmd_vel_gait.cpp)
ament_target_dependencies(go2_cmd_vel_gait rclcpp geometry_msgs trajectory_msgs)
add_executable(go2_champ_gait src/go2_champ_gait.cpp)
target_include_directories(go2_champ_gait PRIVATE
  third_party/champ/include third_party/champ/include/champ)
ament_target_dependencies(go2_champ_gait rclcpp geometry_msgs trajectory_msgs urdf)

install(TARGETS pcd_local_map_node go2_cmd_vel_gait go2_champ_gait DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz description DESTINATION share/${PROJECT_NAME})
install(FILES third_party/champ/include/champ/LICENSE third_party/champ/README.md
  DESTINATION share/${PROJECT_NAME}/third_party/champ)
install(PROGRAMS scripts/keypoint_recorder.py scripts/publish_test_path.py
  scripts/path_publisher.py scripts/pct_path_adapter.py scripts/pcd_to_sdf.py
  scripts/scale_pcd_z.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_test(NAME pcd_to_sdf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pcd_to_sdf.py)
  add_launch_test(test/test_pcd_local_map.py TIMEOUT 35)
  add_launch_test(test/test_pct_path_adapter.py TIMEOUT 35)
endif()
ament_package()
