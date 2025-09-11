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
find_package(VTK REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED) 
find_package(nav2_amcl REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(vizzy_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(behaviortree_cpp REQUIRED) 
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(opennav_docking_bt REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)

# TODO: Find a way to include this package as a dependency
# without causing a Ceres dependency issue.
# Althout it is dependent on the package.xml file (and therefore installed
# with rosdep, we should fix this). 
# find_package(nav2_constrained_smoother REQUIRED)

find_package(nav2_regulated_pure_pursuit_controller REQUIRED)

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
  behaviortree_cpp
  nav2_behavior_tree
  nav2_costmap_2d
  vizzy_msgs
  nav2_util
  nav_msgs
  rclcpp
  tf2_ros
)

# Create a library from the class implementation files.
add_library(docking_lib
  src/dock_pose_estimator.cpp
  src/pattern_pose_estimation.cpp
)

# Tell the library where to find ALL header files.
target_include_directories(docking_lib PUBLIC
  # This tells it where to find its own headers.
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>

  # These tell it where to find PCL and VTK headers.
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

# Link the library against all of its ROS 2 dependencies.
ament_target_dependencies(docking_lib PUBLIC
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  laser_geometry
  sensor_msgs
  geometry_msgs
)

# Link the library against ALL required libraries.
target_link_libraries(docking_lib PUBLIC
  # These link against the actual PCL and VTK library files.
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

# Create the executable from ONLY the main() file.
add_executable(dock_pose_estimator_node src/dock_pose_estimator_node.cpp)

# Link the executable against the new library.
# This automatically handles all includes and links from docking_lib.
target_link_libraries(dock_pose_estimator_node PUBLIC docking_lib)

# Create the executable for the charging_action_server_node.
add_executable(charging_action_server_node src/charging_action_server_node.cpp src/charging_action_server.cpp)

# Link the executable against its dependencies.
ament_target_dependencies(charging_action_server_node PUBLIC
  rclcpp
  rclcpp_action
  geometry_msgs
  vizzy_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  behaviortree_cpp
  nav2_behavior_tree
)

# Specify the include directories for the charging_action_server_node.
target_include_directories(charging_action_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav2_behavior_tree_INCLUDE_DIRS}
)

# Create the nav2_lifecycle_manager_bt_node library.
add_library(nav2_lifecycle_manager_bt_node SHARED
  src/docking_lifecycle_state_manager.cpp
)

# Specify the include directories for the nav2_lifecycle_manager_bt_node.
target_include_directories(nav2_lifecycle_manager_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav2_behavior_tree_INCLUDE_DIRS}
)

# Link the nav2_lifecycle_manager_bt_node library against its dependencies.
ament_target_dependencies(nav2_lifecycle_manager_bt_node PUBLIC
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp
  nav2_msgs
  opennav_docking_bt
)

# Create the library with the opennav plugin registrations.
add_library(opennav_docking_bt_plugin SHARED
  src/opennav_docking_bt_plugin_register.cpp
)

# Specify the include directories for the opennav_docking_bt_plugin.
target_include_directories(opennav_docking_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the opennav_docking_bt_plugin against its dependencies.
ament_target_dependencies(opennav_docking_bt_plugin PUBLIC
  rclcpp
  rclcpp_lifecycle
  opennav_docking_bt
)

# Install the executables and libraries.
install(TARGETS
  nav2_check_path_cost_bt_node
  dock_pose_estimator_node
  charging_action_server_node
  docking_lib 
  nav2_lifecycle_manager_bt_node
  opennav_docking_bt_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the directories to be accessible via the package.
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch maps models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  script/battery_state_simulator_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Behaviors tree plugin description file.
install(FILES
  config/bt_plugins.xml # This is the main plugin description file that lists all plugins.
  DESTINATION share/${PROJECT_NAME}/
)

# Export the plugin description file so that it is discoverable by Nav2.
pluginlib_export_plugin_description_file(${PROJECT_NAME} config/bt_plugins.xml)

ament_export_dependencies(rosidl_default_runtime)

# Install the package.xml and register the package with ament.
ament_package()