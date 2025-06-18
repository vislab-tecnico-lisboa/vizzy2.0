# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.22)

# Set the project name.
project(vizzy_sensors)

# Default to C++23.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(laser_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

add_library(vizzy_sensors_filters src/vizzy_sensors_filters.cpp)
ament_target_dependencies(vizzy_sensors_filters
  rclcpp
  pluginlib
  laser_filters
  Boost
)
target_include_directories(vizzy_sensors_filters PUBLIC include)

add_library(suppression src/nodelets/saccadic_suppression_nodelet.cpp)
ament_target_dependencies(suppression
  rclcpp
  Boost
  OpenCV
  sensor_msgs
  image_transport
)
target_include_directories(suppression PUBLIC include)

install(
  TARGETS
    vizzy_sensors_filters
    suppression
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vizzy_sensors_plugins.xml")
  install(
    FILES vizzy_sensors_plugins.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()