# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# The code for this file is heavily modified, although based on the navigation_launch.py file from the
# following repository:
#
# https://github.com/utexas-bwi/bwi-ros2/blob/main/src/nav2_bringup/launch/navigation_launch.py
# As seen above, the original file is licensed under the Apache License, Version 2.0.
#
# This file has all the necessary code to launch the navigation stack for the Vizzy robot.
# Due to the complexity of the navigation stack, this python launcher was created directly and 
# not based on the XML launch files. The original XML launch files were used as a reference to
# obtain the necessary parameters and nodes to be launched.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # Get the launch directory
    pkg_dir = get_package_share_directory('vizzy_navigation')

    # TODO: The 'map' frame is not recognized when using a namespace other than empty string.
    # TODO: Correct this in the future, so everything vizzy related can be in the 'vizzy' namespace.
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Ign Gazebo) clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level of the nodes. Applies to all nodes launched in this file.')
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='map',
        description='Name of the map topic.'
    )
    scan_topic_front_arg = DeclareLaunchArgument(
        'scan_topic_front',
        default_value='nav_hokuyo_laser/front/scan',
        description='Name of the laser scan topic.'
    )
    scan_topic_rear_arg = DeclareLaunchArgument(
        'scan_topic_rear',
        default_value='nav_hokuyo_laser/front/scan',
        description='Name of the laser scan topic.'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(pkg_dir, 'maps', 'isr_7th_floor_simulation.yaml'),
        description='Full path to map YAML file.'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_topic   = LaunchConfiguration('map_topic')
    scan_topic_front  = LaunchConfiguration('scan_topic_front')
    scan_topic_rear   = LaunchConfiguration('scan_topic_rear')
    map_yaml    = LaunchConfiguration('map_yaml')

    # Substitutions for the parameters file.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    # Load the parameters file with the substitutions.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Remap topics due to the namespace (if it is different than empty string).
    remappings = [('tf', '/tf'),
                ('tf_static', '/tf_static')]

    # Load the navigation nodes.
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('scan_topics' , [scan_topic_front, scan_topic_rear])]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_yaml,
                             'frame_id': 'map',
                             'topic_name': map_topic,
                             'use_sim_time': use_sim_time}],
                remappings=[('map', map_topic)]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params]),
        ]
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(namespace),
            LoadComposableNodes(
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params_file, 
                            {'use_sim_time':  use_sim_time},
                    {  
                        'scan_topics': [scan_topic_front, scan_topic_rear]
                    }],
                remappings=remappings + [('map',  map_topic)]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[configured_params]),
        
            ])])

    # return the launch description
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_composition_arg,
        container_name_arg,
        use_respawn_arg,
        log_level_arg,
        map_topic_arg,
        scan_topic_front_arg,
        scan_topic_rear_arg,
        map_yaml_arg,
        load_nodes,
        load_composable_nodes
    ])