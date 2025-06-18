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

# Load ament_cmake and all dependencies required for this package.
find_package(ament_cmake REQUIRED
  COMPONENTS
    laser_filters
    message_generation
    nodelet
    pcl_ros
    pluginlib
    roscpp
    roslint
    sensor_msgs
    image_transport
    camera_info_manager
    dynamic_reconfigure
    geometry_msgs
)

find_package(OpenCV REQUIRED)

# Install the directories to be accessible via the package.
install(
  DIRECTORY launch 
  config
  DESTINATION share/${PROJECT_NAME}
) 

# Install the package.xml and register the package with ament.
ament_package()