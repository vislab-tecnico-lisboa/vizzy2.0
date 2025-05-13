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
find_package(visualization_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

# Include directories from the package.
include_directories(${PCL_INCLUDE_DIRS})

add_executable(twistsimulator src/twistsimulator.cpp)

ament_target_dependencies(twistsimulator
  rclcpp
  geometry_msgs)

install(TARGETS
  twistsimulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch/config/rviz files.
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install the package.xml and register the package with ament.
ament_package()