export ROS2_WS="~/VisLab-Vizzy-Ros2"
export GAZEBO_MODEL_PATH=$ROS2_WS/src/vizzy/vizzy_gazebo:\$GAZEBO_MODEL_PATH
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix vizzy_gazebo)/share:$(ros2 pkg prefix vizzy_description)/share:$IGN_GAZEBO_RESOURCE_PATH