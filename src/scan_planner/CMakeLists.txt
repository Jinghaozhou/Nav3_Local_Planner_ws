cmake_minimum_required(VERSION 3.16)
project(scan_planner VERSION 0.2.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SCAN_CORE_STANDALONE "Build and test only the C++ core without ROS" OFF)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Keep the algorithm independent of ROS even though it lives in this ROS package.
add_library(scan_planner_core
  src/core/map/grid_map.cpp src/core/map/raycast.cpp
  src/core/search/projected_astar.cpp
  src/core/trajectory/uniform_bspline.cpp src/core/trajectory/polynomial_trajectory.cpp
  src/core/optimization/bspline_optimizer.cpp src/core/optimization/gradient_descent_optimizer.cpp
  src/core/planning/reference_path.cpp src/core/planning/planner_manager.cpp
  src/core/control/trajectory_tracker.cpp)
target_include_directories(scan_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_planner_core PUBLIC Eigen3::Eigen)
target_compile_options(scan_planner_core PRIVATE -Wall -Wextra)
install(TARGETS scan_planner_core EXPORT export_scan_planner_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE scan_planner_core GTest::gtest_main)
  add_test(NAME core_contracts COMMAND test_core)
endif()

if(NOT SCAN_CORE_STANDALONE)
  find_package(ament_cmake REQUIRED)
  find_package(scan_planner_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_library(scan_nodes
    src/ros2/frame_transformer.cpp src/ros2/velocity_estimator.cpp
    src/ros2/input_adapter.cpp src/ros2/planner_node.cpp src/ros2/controller_node.cpp)
  target_include_directories(scan_nodes PRIVATE src/ros2)
  target_compile_options(scan_nodes PRIVATE -Wall -Wextra)
  ament_target_dependencies(scan_nodes scan_planner_msgs rclcpp
    geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)
  target_link_libraries(scan_nodes scan_planner_core Threads::Threads)

  add_executable(planner_node src/ros2/planner_main.cpp)
  add_executable(controller_node src/ros2/controller_main.cpp)
  target_include_directories(planner_node PRIVATE src/ros2)
  target_include_directories(controller_node PRIVATE src/ros2)
  target_link_libraries(planner_node scan_nodes)
  target_link_libraries(controller_node scan_nodes)
  install(TARGETS planner_node controller_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_adapters test/test_adapters.cpp src/ros2/velocity_estimator.cpp)
    target_include_directories(test_adapters PRIVATE src/ros2)
    ament_target_dependencies(test_adapters rclcpp builtin_interfaces)
    target_link_libraries(test_adapters Eigen3::Eigen)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_integration.py TIMEOUT 100)
  endif()

  ament_export_targets(export_scan_planner_core HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
