# Launches slam_toolbox (online async) to build a new 2D map from the merged
# laser scan. Mapping-only: this brings up just the SLAM node. The robot should
# be driven with teleop_twist_keyboard (publishes /cmd_vel) and the map saved when done:
#
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>
#
# slam_toolbox publishes the map->odom transform, so AMCL / map_server must NOT
# run at the same time (use vizzy_slam_launch.xml, which skips navigation).

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vizzy_navigation')
    default_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_mapping.yaml')

    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='slam_toolbox mapping parameters file.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_merged',
                              description='LaserScan topic to map from (the merged front+rear scan).'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odometry'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': False,
                    'scan_topic': scan_topic,
                    'base_frame': base_frame_id,
                    'odom_frame': odom_frame_id,
                    'map_frame': map_frame_id,
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
