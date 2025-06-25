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

# ? This file was refactored to adopt a Generate-Then-Use approach instead of performing in-memory substitutions.
# ? This allows for the parameters to be saved to a file, which can be used later for debugging or inspection.

import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

# --- Opaque Function to save substituted params ---
def set_nested_item(data_dict, keys, value):
    """Helper function to set a value in a nested dictionary."""
    for key in keys[:-1]:
        data_dict = data_dict.setdefault(key, {})
    data_dict[keys[-1]] = value

def save_rewritten_yaml(context: LaunchContext, output_file_path: str = None):
    """
    OpaqueFunction to perform substitutions and save the result to a file.
    This function is executed at launch time.
    """
    # Get the original parameters file path.
    params_file_path = LaunchConfiguration('params_file').perform(context)

    # Load the original YAML file.
    with open(params_file_path, 'r') as f:
        original_params = yaml.safe_load(f)

    # Get the dictionary of substitutions defined in the launch file.
    substitutions_dict = {
        # General substitutions.
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'autostart': LaunchConfiguration('autostart').perform(context),
        'map_server.ros__parameters.yaml_filename': LaunchConfiguration('map_yaml').perform(context),

        # AMCL substitutions.
        'amcl.ros__parameters.base_frame_id': LaunchConfiguration('base_frame_id').perform(context),
        'amcl.ros__parameters.global_frame_id': LaunchConfiguration('map_frame_id').perform(context),
        'amcl.ros__parameters.odom_frame_id': LaunchConfiguration('odom_frame_id').perform(context),
        'amcl.ros__parameters.scan_topic': LaunchConfiguration('scan_topic_front').perform(context),
        'amcl.ros__parameters.map_topic': LaunchConfiguration('map_topic').perform(context),

        # BT Navigator substitutions.
        'bt_navigator.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'bt_navigator.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'bt_navigator.ros__parameters.odom_topic': LaunchConfiguration('odom_topic').perform(context),

        # Behavior Server substitutions.
        'behavior_server.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'behavior_server.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),

        # Global and Local Costmaps substitutions.
        'global_costmap.global_costmap.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_front.topic': LaunchConfiguration('scan_topic_front').perform(context),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_rear.topic': LaunchConfiguration('scan_topic_rear').perform(context),
        'local_costmap.local_costmap.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan_front.topic': LaunchConfiguration('scan_topic_front').perform(context),
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan_rear.topic': LaunchConfiguration('scan_topic_rear').perform(context),

        # Velocity Smoother substitutions.
        'velocity_smoother.ros__parameters.odom_topic': LaunchConfiguration('odom_topic').perform(context),
    }
    
    # Apply substitutions to the loaded params.
    for key_path, value in substitutions_dict.items():
        # The 'use_sim_time' and 'autostart' keys are special cases at the top level.
        # of each node's 'ros__parameters' block, but they are not targeted with.
        # dot notation in the RewrittenYaml class in the same way. We will apply them manually.
        if key_path == 'use_sim_time' or key_path == 'autostart':
             for node_name in original_params:
                 if 'ros__parameters' in original_params[node_name]:
                     # Special handling for boolean strings.
                     if isinstance(value, str):
                        actual_value = value.capitalize() in ['True', '1', 'T']
                     else:
                        actual_value = value
                     original_params[node_name]['ros__parameters'][key_path] = actual_value
        else:
            # Handle dot-notated nested keys.
            keys = key_path.split('.')
            set_nested_item(original_params, keys, value)

    # Write the modified dictionary to the new YAML file.
    with open(output_file_path, 'w') as f:
        yaml.dump(original_params, f, default_flow_style=False)
        
    # Log a message to the console.
    print(f"[Launch Info] Successfully wrote substituted params to {output_file_path}")
    
    # The OpaqueFunction must return a list of nodes or actions (can be empty).
    return []

def generate_launch_description():

    # Get the launch directory
    pkg_dir = get_package_share_directory('vizzy_navigation')

    # TODO: The 'map' frame is not recognized when using a namespace other than empty string.
    # TODO: Correct this in the future, so everything vizzy related can be in the 'vizzy' namespace.

    # --- Launch Arguments ---
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
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_footprint',
        description='Base frame ID for the robot. This is used by the AMCL node to localize the robot in the map. It should match the frame ID of the robot\'s base in the TF tree.'
    )
    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Map frame ID for the robot. This is used by the AMCL node to localize the robot in the map. It should match the frame ID of the map in the TF tree.'
    )
    odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odometry',
        description='Odometry frame ID for the robot. This is used by the AMCL node to localize the robot in the map. It should match the frame ID of the odometry in the TF tree.'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom',
        description='Odometry topic for the robot. This is used by the AMCL node to localize the robot in the map. It should match the topic name of the odometry in the ROS2 system.'
    )

    # --- Launch Configurations ---
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    package_name = 'vizzy_navigation' 

    # Get the path to the package's install share directory.
    install_share_path = get_package_share_directory(package_name)

    # Define the output path within this packages directory.
    output_dir = os.path.join(install_share_path, 'params')
    os.makedirs(output_dir, exist_ok=True) 
    output_file_path = os.path.join(output_dir, 'nav2_params_substituted.yaml')

    # --- Actions for Logging and Saving ---
    log_sim_time_action = LogInfo(msg=['[Launch Info] Using simulation time: ', use_sim_time])
    save_params_action = OpaqueFunction(function=save_rewritten_yaml,
                                        args=[output_file_path])

    # Create one ParameterFile object with all substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=output_file_path,
            root_key=namespace,
            param_rewrites=[],
            convert_types=True),
        allow_substs=True)

    # Remap topics due to the namespace (if it is different than empty string).
    remappings = [('tf', '/tf'),
                ('tf_static', '/tf_static')]

    load_nodes = GroupAction(
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
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params], 
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': ['map_server', 'amcl', 'planner_server',
                                            'controller_server', 'smoother_server',
                                            'bt_navigator', 'behavior_server',
                                            'waypoint_follower', 'velocity_smoother']}])
        ]
    )

    # --- Launch Description ---
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        map_topic_arg,
        scan_topic_front_arg,
        scan_topic_rear_arg,
        map_yaml_arg,
        base_frame_id_arg,
        map_frame_id_arg,
        odom_frame_id_arg,
        odom_topic_arg,
        
        # Actions
        log_sim_time_action,
        save_params_action,
        load_nodes,
    ])