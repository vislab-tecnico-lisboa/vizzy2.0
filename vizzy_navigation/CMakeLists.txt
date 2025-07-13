# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.22)

# Set the project name.
project(vizzy_navigation)

# Default to C++23.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all the required packages.
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED) 
find_package(nav2_amcl REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED) 
find_package(interactive_markers REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)

# Twistsimulator target.
add_executable(twistsimulator src/twistsimulator.cpp)
ament_target_dependencies(twistsimulator rclcpp geometry_msgs)

add_library(nav2_check_path_cost_bt_node SHARED
  src/check_path_cost_condition.cpp
)

# Specify the include directories for the nav2_check_path_cost_bt_node.
target_include_directories(nav2_check_path_cost_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav2_behavior_tree_INCLUDE_DIRS}
)

# Link the nav2_check_path_cost_bt_node with the required libraries.
ament_target_dependencies(nav2_check_path_cost_bt_node PUBLIC
  "behaviortree_cpp_v3"
  "nav2_behavior_tree"
  "nav2_costmap_2d"
  "nav2_util"
  "nav_msgs"
  "rclcpp"
  "tf2_ros"
)

# Install the executables and libraries.
install(TARGETS
  twistsimulator
  nav2_check_path_cost_bt_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the directories to be accessible via the package.
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# Install the Behaviour Tree plugin description file.
pluginlib_export_plugin_description_file(nav2_check_path_cost_bt_node config/bt_plugins.xml)

# Install the package.xml and register the package with ament.
ament_package()