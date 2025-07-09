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
# The code for this file is *heavily* modified (regarding code and concepts), although based on the navigation_launch.py file from the
# following repository:
#
# https://github.com/utexas-bwi/bwi-ros2/blob/main/src/nav2_bringup/launch/navigation_launch.py
# As seen above, the original file is licensed under the Apache License, Version 2.0.
#
# This file has all the necessary code to launch the navigation stack for the Vizzy robot.
# Due to the complexity of the navigation stack, this python launcher was created directly and 
# not based on the XML launch files. The original XML launch files were used as a reference to
# obtain the necessary parameters and nodes to be launched.

# * This file was refactored to adopt a Generate-Then-Use approach for configuration parameters instead of performing in-memory substitutions.
# * This allows for the parameters to be saved to a file, which can be used later for debugging or inspection.

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

# This function is used to parse the pose string from the launch configuration.
def Parse_Pose_Str(context):
    pose_str = LaunchConfiguration('pose').perform(context)
    # The string is e.g., "-x 0.0 -y 0.0 ...", so we just split it by spaces.
    pose_args = pose_str.split()
    return pose_args

# --- Opaque Function to save rewritten BT XML ---
def save_rewritten_bt_xml(context: LaunchContext, output_file_path: str = None):
    """
    OpaqueFunction to substitute the controller_id in a BT XML template
    and save the result to a file.
    """
    # Get the package share directory.
    pkg_dir = get_package_share_directory('vizzy_navigation')
    
    # Define the path to your template BT XML file.
    template_file_path = os.path.join(pkg_dir, 'config', 'custom_navigate_to_pose_bt_navigator_nav2.xml')

    # Get the controller plugin choice from the launch argument.
    controller_plugin_type = LaunchConfiguration('controller_plugin').perform(context)

    # Map the simple launch argument to the required controller name.
    controller_mapping = {
        "DWB": "dwb_controller",
        "RPP": "rpp_controller",
        "MPPI": "mppi_controller"
    }
    
    # Get the specific controller name, e.g., "dwb_controller".
    active_controller_id = controller_mapping.get(controller_plugin_type)
    
    if not active_controller_id:
        raise RuntimeError(f"[Launch Error] Unknown controller_plugin for BT XML: {controller_plugin_type}.")

    # Read the template file.
    with open(template_file_path, 'r') as f:
        xml_content = f.read()

    # Perform the substitution.
    xml_content = xml_content.replace('CONTROLLER_PLACEHOLDER', active_controller_id)

    # Write the modified content to the new XML file.
    with open(output_file_path, 'w') as f:
        f.write(xml_content)
        
    print(f"[Launch Info] Successfully wrote substituted BT XML to {output_file_path}")
    
    return []

# --- Opaque Function to save substituted params ---
def set_nested_item(data_dict, keys, value):
    """Helper function to set a value in a nested dictionary."""
    for key in keys[:-1]:
        data_dict = data_dict.setdefault(key, {})
    data_dict[keys[-1]] = value

def save_rewritten_yaml(context: LaunchContext, output_file_path: str = None, output_bt_path: str = None):
    """
    OpaqueFunction to perform substitutions and save the result to a file.
    This function is executed at launch time.
    """
    # Get the original parameters file path.
    params_file_path = LaunchConfiguration('params_file').perform(context)

    # Load the original YAML file.
    with open(params_file_path, 'r') as f:
        original_params = yaml.safe_load(f)

    # Call the function to parse the pose string from the launch configuration.
    pose_args = Parse_Pose_Str(context)

    # Parse the values of pose_args into floats.
    pose_args = [float(arg) if i % 2 == 1 else arg for i, arg in enumerate(pose_args)]

    # Get the controller plugin type from launch configuration.
    controller_plugin_type = LaunchConfiguration('controller_plugin').perform(context)

    # Assign the controller plugin based on the type specified in the launch configuration.
    if controller_plugin_type == "DWB":
        original_params['controller_server']['ros__parameters']['controller_plugins'] = ["dwb_controller"] 
    elif controller_plugin_type == "RPP":
        original_params['controller_server']['ros__parameters']['controller_plugins'] = ["rpp_controller"]
    elif controller_plugin_type == "MPPI":
        original_params['controller_server']['ros__parameters']['controller_plugins'] = ["mppi_controller"]
    else:
        # Fallback or error if an unknown plugin is specified.
        print(f"[Launch Error] Unknown controller_plugin: {controller_plugin_type}. Using default or raising error.")
        pass

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
        'amcl.ros__parameters.initial_pose.x': -float(pose_args[1]) if len(pose_args) > 1 else 0.0,
        'amcl.ros__parameters.initial_pose.y': -float(pose_args[3]) if len(pose_args) > 3 else 0.0,
        'amcl.ros__parameters.initial_pose.z': -float(pose_args[5]) if len(pose_args) > 5 else 0.0,
        'amcl.ros__parameters.initial_pose.yaw': float(pose_args[7]) if len(pose_args) > 7 else 0.0,
        'amcl.ros__parameters.update_min_d': float(LaunchConfiguration('update_min_d').perform(context)),
        'amcl.ros__parameters.update_min_a': float(LaunchConfiguration('update_min_a').perform(context)),
        'amcl.ros__parameters.laser_max_range': float(LaunchConfiguration('laser_max_range').perform(context)),
        'amcl.ros__parameters.beam_skip_distance': float(LaunchConfiguration('beam_skip_distance').perform(context)),
        'amcl.ros__parameters.beam_skip_error_threshold': float(LaunchConfiguration('beam_skip_error_threshold').perform(context)),
        'amcl.ros__parameters.beam_skip_threshold': float(LaunchConfiguration('beam_skip_threshold').perform(context)),
        'amcl.ros__parameters.do_beamskip': LaunchConfiguration('do_beamskip').perform(context),

        # BT Navigator substitutions.
        'bt_navigator.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'bt_navigator.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'bt_navigator.ros__parameters.odom_topic': LaunchConfiguration('odom_topic').perform(context),
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': output_bt_path,

        # Behavior Server substitutions.
        'behavior_server.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'behavior_server.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),

        # Controller Server substitutions.
        'controller_server.ros__parameters.odom_topic': LaunchConfiguration('odom_topic').perform(context),
        'controller_server.ros__parameters.controller_frequency': float(LaunchConfiguration('controller_frequency').perform(context)),
        'controller_server.ros__parameters.dwb_controller.PathAlign.forward_point_distance': float(LaunchConfiguration('path_align_forward_point_distance').perform(context)),
        'controller_server.ros__parameters.dwb_controller.GoalAlign.forward_point_distance': float(LaunchConfiguration('goal_align_forward_point_distance').perform(context)),
        'controller_server.ros__parameters.dwb_controller.PathAlign.scale': float(LaunchConfiguration('path_align_scale').perform(context)),
        'controller_server.ros__parameters.dwb_controller.GoalAlign.scale': float(LaunchConfiguration('goal_align_scale').perform(context)),
        'controller_server.ros__parameters.dwb_controller.PathDist.scale': float(LaunchConfiguration('path_dist_scale').perform(context)),
        'controller_server.ros__parameters.dwb_controller.GoalDist.scale': float(LaunchConfiguration('goal_dist_scale').perform(context)),
        'controller_server.ros__parameters.dwb_controller.BaseObstacle.scale': float(LaunchConfiguration('base_obstacle_scale').perform(context)),

        # Map Server substitutions.
        'map_server.ros__parameters.topic_name': LaunchConfiguration('map_topic').perform(context),
        'map_server.ros__parameters.frame_id': LaunchConfiguration('map_frame_id').perform(context),

        # Global and Local Costmaps substitutions.
        'global_costmap.global_costmap.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_front.topic': LaunchConfiguration('scan_topic_front').perform(context),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_rear.topic': LaunchConfiguration('scan_topic_rear').perform(context),
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': LaunchConfiguration('map_topic').perform(context),
        'global_costmap.global_costmap.ros__parameters.use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius': float(LaunchConfiguration('inflation_radius').perform(context)),
        'global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor': float(LaunchConfiguration('cost_scaling_factor').perform(context)),
        'local_costmap.local_costmap.ros__parameters.global_frame': LaunchConfiguration('map_frame_id').perform(context),
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': LaunchConfiguration('base_frame_id').perform(context),
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan_front.topic': LaunchConfiguration('scan_topic_front').perform(context),
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan_rear.topic': LaunchConfiguration('scan_topic_rear').perform(context),
        'local_costmap.local_costmap.ros__parameters.static_layer.map_topic': LaunchConfiguration('map_topic').perform(context),
        'local_costmap.local_costmap.ros__parameters.use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius': float(LaunchConfiguration('inflation_radius').perform(context)),
        'local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor': float(LaunchConfiguration('cost_scaling_factor').perform(context)),

        # Velocity Smoother substitutions.
        'velocity_smoother.ros__parameters.odom_topic': LaunchConfiguration('odom_topic').perform(context),
    }

    # Apply substitutions to the loaded params.
    for key_path, value in substitutions_dict.items():
        # The 'use_sim_time' and 'autostart' keys are special cases at the top level.
        # of each node's 'ros__parameters' block, but they are not targeted with.
        # dot notation in the RewrittenYaml class in the same way. We will apply them manually.
        if key_path == 'use_sim_time' or key_path == 'autostart' or key_path == 'do_beamskip':
             for node_name in original_params:
                 if 'ros__parameters' in original_params[node_name]:
                     # Special handling for boolean strings.
                     if isinstance(value, str):
                        actual_value = value.lower() in ['true', '1', 't']
                     else:
                        actual_value = value
                     original_params[node_name]['ros__parameters'][key_path] = actual_value
        else:
            keys = key_path.split('.')

            # Check if we are handling a parameter within the active_controller_param_key (dwb_controller).
            # This is where DWB or other specific controller params would be set.
            # We explicitly check that the key_path matches the expected structure.
            if len(keys) > 3 and keys[2] == "dwb_controller":
                # Path to the parent dict, ['controller_server', 'ros__parameters', 'dwb_controller'].
                parent_keys = keys[:3]
                # The final key which contains dots, e.g., 'BaseObstacle.scale'.
                final_key = '.'.join(keys[3:])

                # Navigate to the parent dictionary.
                d = original_params
                for k in parent_keys:
                    d = d[k]

                # Set the value using the complete, un-split final key.
                d[final_key] = value
            else:
                if key_path.endswith('use_sim_time') or key_path.endswith('autostart') or key_path.endswith('do_beamskip'):
                    if isinstance(value, str):
                        value = value.lower() in ['true', '1', 't']
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
    initial_pose_arg = DeclareLaunchArgument(
        'initial_pose',
        default_value='-x 0.0 -y 0.0 -z 0.0 -Y 0.0',
        description='Initial pose of the robot in the map frame. This is used by the AMCL node to localize the robot in the map. The format is "-x <x> -y <y> -z <z> -R <roll> -P <pitch> -Y <yaw>". The values are in meters for x, y, and z, and in radians for roll, pitch, and yaw.'
    )
    update_min_d_arg = DeclareLaunchArgument(
        'update_min_d',
        default_value='0.01',
        description='Minimum distance for localization updates. This is used by the AMCL node to determine when to update the robot\'s pose based on the laser scan data. The value is in meters.'
    )
    update_min_a_arg = DeclareLaunchArgument(
        'update_min_a',
        default_value='0.01',
        description='Minimum angle for localization updates. This is used by the AMCL node to determine when to update the robot\'s pose based on the laser scan data. The value is in radians.'
    )
    laser_max_range_arg = DeclareLaunchArgument(
        'laser_max_range',
        default_value='-1.0',
        description='Maximum range for the laser scanner. This is used by the AMCL node to filter out laser scan data that is beyond the specified range. The value is in meters. A value of -1.0 means no limit, and the laser scan data will not be filtered based on range.'
    )
    beam_skip_distance_arg = DeclareLaunchArgument(
        'beam_skip_distance',
        default_value='0.5',
        description='Distance threshold for beam skipping. This is used by the AMCL node to ignore beams that most particles disagree with in Likelihood field model. Maximum distance to consider skipping for (m).'
    )
    beam_skip_error_threshold_arg = DeclareLaunchArgument(
        'beam_skip_error_threshold',
        default_value='0.9',
        description='Error threshold for beam skipping. This is used by the AMCL node as the percentage of beams after not matching map to force full update due to bad convergence.'
    )
    beam_skip_threshold_arg = DeclareLaunchArgument(
        'beam_skip_threshold',
        default_value='0.3',
        description='Threshold for beam skipping. This is used by the AMCL node as the percentage of beams required to skip.'
    )  
    do_beamskip_arg = DeclareLaunchArgument(
        'do_beamskip',
        default_value='true',
        description='Enable or disable beam skipping in the laser scanner.'
    )
    controller_frequency_arg = DeclareLaunchArgument(
        'controller_frequency',
        default_value='20.0',
        description='Frequency of the controller in Hz.',
    )
    path_align_forward_point_distance_arg = DeclareLaunchArgument(
        'path_align_forward_point_distance',
        default_value='0.325',
        description='Forward point distance for the PathAlign critic in the controller.',
    )
    goal_align_forward_point_distance_arg = DeclareLaunchArgument(
        'goal_align_forward_point_distance',
        default_value='0.325',
        description='Forward point distance for the GoalAlign critic in the controller.',
    )
    inflation_radius_arg = DeclareLaunchArgument(
        'inflation_radius',
        default_value='0.50',
        description='Inflation radius for the costmap. This is used by the local and global costmaps.'
    )
    path_align_scale_arg = DeclareLaunchArgument(
        'path_align_scale',
        default_value='32.0',
        description='Scale factor for the PathAlign critic in the controller.'
    )
    goal_align_scale_arg = DeclareLaunchArgument(
        'goal_align_scale',
        default_value='24.0',
        description='Scale factor for the GoalAlign critic in the controller.'
    )
    path_dist_scale_arg = DeclareLaunchArgument(
        'path_dist_scale',
        default_value='32.0',
        description='Scale factor for the PathDist critic in the controller.'
    )
    goal_dist_scale_arg = DeclareLaunchArgument(
        'goal_dist_scale',
        default_value='24.0',
        description='Scale factor for the GoalDist critic in the controller.'
    )
    cost_scaling_factor_arg = DeclareLaunchArgument(
        'cost_scaling_factor',
        default_value='3.0',
        description='Cost scaling factor for the inflation layer in the costmap.'
    )
    base_obstacle_scale_arg = DeclareLaunchArgument(
        'base_obstacle_scale',
        default_value='0.02',
        description='Base obstacle scale for the costmap. This is used by the local and global costmaps to scale the cost of obstacles.'
    )
    controller_plugin_arg = DeclareLaunchArgument(
        'controller_plugin',
        default_value='MPPI',
        description='Controller plugin to use for the navigation stack.'
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

    output_bt_dir = os.path.join(get_package_share_directory(package_name), 'behavior_trees')
    os.makedirs(output_bt_dir, exist_ok=True)
    output_bt_path = os.path.join(output_bt_dir, 'custom_navigate_to_pose_bt_navigator_nav2.xml')
    
    # Add this new action to generate the XML file
    save_bt_xml_action = OpaqueFunction(function=save_rewritten_bt_xml,
                                        args=[output_bt_path])

    # Define the output path within this packages directory.
    output_dir = os.path.join(install_share_path, 'params')
    os.makedirs(output_dir, exist_ok=True) 
    output_file_path = os.path.join(output_dir, 'nav2_params_substituted.yaml')

    # --- Actions for Logging and Saving ---
    log_sim_time_action = LogInfo(msg=['[Launch Info] Using simulation time: ', use_sim_time])
    save_params_action = OpaqueFunction(function=save_rewritten_yaml,
                                        args=[output_file_path, output_bt_path])

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
                ('tf_static', '/tf_static'),
                ('map', LaunchConfiguration('map_topic')),]

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
        initial_pose_arg,
        update_min_d_arg,
        update_min_a_arg,
        laser_max_range_arg,
        beam_skip_distance_arg,
        beam_skip_error_threshold_arg,
        beam_skip_threshold_arg,
        do_beamskip_arg,
        controller_frequency_arg,
        path_align_forward_point_distance_arg,
        goal_align_forward_point_distance_arg,
        inflation_radius_arg,
        path_align_scale_arg,
        goal_align_scale_arg,
        path_dist_scale_arg,
        goal_dist_scale_arg,
        cost_scaling_factor_arg,
        base_obstacle_scale_arg,
        controller_plugin_arg,
        
        # Actions
        log_sim_time_action,
        save_params_action,
        save_bt_xml_action,
        load_nodes,
    ])