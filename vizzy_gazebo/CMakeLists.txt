# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.22)

# Set the project name.
project(vizzy_gazebo)

# Default to C++23.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Load ament_cmake and all dependencies required for this package.
find_package(ros_gz_sim REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(vizzy_description REQUIRED)
find_package(vizzy_navigation REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install the directories to be accessible via the package.
install(
  DIRECTORY launch 
  worlds
  media
  pessoa1
  pessoa2
  pessoa3
  DESTINATION share/${PROJECT_NAME}
) 

ament_package()

