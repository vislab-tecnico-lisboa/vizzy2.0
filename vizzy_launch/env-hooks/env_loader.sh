export GAZEBO_MODEL_PATH=$ROS2_WS/src/vizzy/vizzy_gazebo:\$GAZEBO_MODEL_PATH
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix vizzy_gazebo)/share:$(ros2 pkg prefix vizzy_description)/share:$IGN_GAZEBO_RESOURCE_PATH
export ROS_MASTER_URI=http://10.1.3.1:11311
export ROS_IP=10.1.3.1