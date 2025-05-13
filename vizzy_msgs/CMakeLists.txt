# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.22)

# Set the project name.
project(vizzy_msgs)

# Default to C++23.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare message files.
set(msg_files
  "msg/Tactile.msg"
  "msg/TacVector.msg"
  "msg/KokamBatteryPower.msg"
)

# Declare service files.
set(srv_files
  "srv/ArmDown.srv"
  "srv/BatteryState.srv"
  "srv/MotorsShutdown.srv"
  "srv/BatteryChargingState.srv"
  "srv/ShutdownStartYarpRunProcess.srv"
)

# Declare action files.
set(action_files
  "action/Gaze.action"
  "action/Charge.action"
  "action/Cartesian.action"
)

# Register the interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs actionlib_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Install the package.xml and register the package with ament.
ament_package()
