# This file had to be created to replace the former "vizzy_spawn_launch.xml" file.
# The original file was not working properly due to the xacro command execution through the xml launcher, 
# and the new file is a complete rewrite in python.
# The principle is exactly the same as the original file, but with a few key changes:
# 1. The resulting URDF file from the Command() execution is now written to a temporary file before being passed to the robot_state_publisher and spawn_node.
# 2. The URDF file is now read and printed after it has been written, allowing for better debugging and verification of the file contents.
# 3. The use of OpaqueFunction allows for better handling of the context and ensures that the URDF file is written before being used by the nodes.
# 4. The bridge node is now started before the robot_state_publisher and joint_state_publisher nodes, ensuring that the necessary topics are available for the nodes to subscribe to.

# Graphically, the launch order is as follows:
#
#   +---------------------+        +---------------------+        +---------------------+
#   |                     |        |                     |        |                     |  
#   |     ROS2 Bridge     |------->|joint_state_publisher|------->|robot_state_publisher|
#   |                     |        |                     |        |                     |
#   +---------------------+        +---------------------+        +---------------------+ 

import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart

# Helper function to write the URDF to a file.
def write_urdf_to_file(context, robot_description_substitution, file_path):
    # Evaluate the substitution using the valid launch context.
    urdf_content = robot_description_substitution.perform(context)
    if urdf_content is None:
        raise RuntimeError("URDF content evaluated to None. Check your substitutions.")
    # Write the evaluated URDF content to a file.
    with open(file_path, 'w') as f:
        f.write(urdf_content)
    # Log that we wrote the file
    print(f"URDF written to: {file_path}")
    return urdf_content # Return content for other nodes to use

# Helper function to print the URDF file.
def print_urdf_file(context, file_path):
    # Read and log the content of the file after it has been written
    try:
        with open(file_path, 'r') as f:
            file_contents = f.read()
        print("Created URDF file contents:")
    except Exception as e:
        print(f"Error reading file: {e}")
    return []

# This function is used to parse the pose string from the launch configuration.
def Parse_Pose_Str(context):
    pose_str = LaunchConfiguration('pose').perform(context)
    # The string is e.g., "-x 0.0 -y 0.0 ...", so we just split it by spaces.
    pose_args = pose_str.split()
    return pose_args

# This OpaqueFunction contains all the logic that needs to be deferred until launch time.
def launch_setup(context, *args, **kwargs):
    
    # Generate the robot_description by processing the URDF file with xacro.
    robot_description_command = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('vizzy_description'), 'robots', LaunchConfiguration('urdf_file')]), ' ',
        'use_yarp:=', LaunchConfiguration('use_yarp'), ' ',
        'disable_laser:=', LaunchConfiguration('disable_laser'), ' ',
        'disable_3d_sensor:=', LaunchConfiguration('disable_3d_sensor'), ' ',
        'base_frame_id:=', LaunchConfiguration('base_frame_id'), ' ',
        'odom_frame_id:=', LaunchConfiguration('odom_frame_id'), ' ',
        'odom_topic:=', LaunchConfiguration('odom_topic')
    ])
    
    urdf_temp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, prefix='vizzy_urdf_')
    urdf_file_path = urdf_temp_file.name
    
    robot_description_content = write_urdf_to_file(context, robot_description_command, urdf_file_path)
    print_urdf_file(context, urdf_file_path)
    
    # Call the function to parse the pose string from the launch configuration.
    pose_args = Parse_Pose_Str(context)

    # The robot_state_publisher uses the URDF content directly.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_tf_static': True,
                'robot_description': robot_description_content
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # Refactor the pose_args to include "-R 0.0 and -P 0.0", because they are not included in the original pose string.
    # This is necessary because the robot_state_publisher expects these arguments to be present.
    if len(pose_args) < 8:
        pose_args += ['-R', '0.0', '-P', '0.0']

    # Sum Pi to the Yaw value.
    # TODO: Check why does ros_gz_sim has a Pi offset from the original pose.
    if '-Y' in pose_args:
        yaw_index = pose_args.index('-Y')
        if yaw_index + 1 < len(pose_args):
            try:
                # Convert the Yaw value to float, add pi, and convert back to string.
                pose_args[yaw_index + 1] = str(float(pose_args[yaw_index + 1]) + 3.141592653589793)
            except ValueError:
                print(f"Invalid Yaw value: {pose_args[yaw_index + 1]}")

    # Spawn the robot using ros_gz_sim with the file path and parsed pose arguments.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_vizzy',
        output='screen',
        arguments=[
            '-urdf',
            '-file', urdf_file_path,
            '-name', LaunchConfiguration('robot').perform(context),
            '-v4'
        ] + pose_args # Append the parsed pose arguments from your function
    )
    
    # The OpaqueFunction must return a list of launch actions to execute.
    return [
        LogInfo(msg=f"Spawning robot with pose arguments: {pose_args}"),
        robot_state_publisher_node,
        spawn_node
    ]

def generate_launch_description():
    
    # Declare launch arguments.
    # The individual pose_x, pose_y, etc., are no longer needed as they cannot be set
    # dynamically from the 'pose' string during this description phase.
    declared_arguments = [
        DeclareLaunchArgument(
            'robot', default_value='vizzy', description='Name of the robot model'
        ),
        DeclareLaunchArgument(
            'pose',
            default_value='-x 0.0 -y 0.0 -z 0.0 -Y 0.0',
            description='Full spawn pose string for Gazebo (e.g., "-x 1.0 -y 2.0")'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'urdf_file', default_value='vizzy.urdf.xacro', description='URDF xacro file'
        ),
        DeclareLaunchArgument(
            'use_yarp', default_value='false', description='Use YARP'
        ),
        DeclareLaunchArgument(
            'disable_laser', default_value='false', description='Disable laser sensor'
        ),
        DeclareLaunchArgument(
            'disable_3d_sensor', default_value='true', description='Disable 3D sensor'
        ),
        DeclareLaunchArgument(
            'base_frame_id', default_value='base_footprint', description='Base frame ID'
        ),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='odometry', description='Odometry frame ID'
        ),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom', description='Odometry topic'
        ),
        DeclareLaunchArgument(
            'scan_front_topic', default_value='/nav_hokuyo_laser/front/scan', description='Front scan topic'
        ),
        DeclareLaunchArgument(
            'scan_rear_topic', default_value='/nav_hokuyo_laser/rear/scan', description= 'Rear scan topic'
        )

    ]

    # Create the joint_state_publisher node.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}], # Use LaunchConfiguration here
        output='screen'
    )

    # Bridge  
    #       /clock  ->  /clock
    #       /model/vizzy/cmd_vel  <-  /cmd_vel
    #       /model/vizzy/odometry  ->  /odom
    #       /model/vizzy/tf  ->  /tf
    #       /model/vizzy/joint_states  ->  /joint_states
    #       /model/vizzy/camera/l/image_raw  ->  /camera/image_raw
    #       /model/vizzy/camera/l/camera_info  ->  /camera/camera_info
    #       /model/vizzy/camera/r/image_raw  ->  /camera/image_raw
    #       /model/vizzy/camera/r/camera_info  ->  /camera/camera_info
    #       /model/vizzy/nav_hokuyo_laser/scan  ->  /nav_hokuyo_laser/front/scan
    #       /model/vizzy/nav_hokuyo_rear_laser/scan  ->  /nav_hokuyo_laser/rear/scan
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/vizzy/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/vizzy/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vizzy/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/vizzy/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/vizzy/camera/l/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/vizzy/camera/r/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/vizzy/nav_hokuyo_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/vizzy/camera/l/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/vizzy/camera/r/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/vizzy/nav_hokuyo_rear_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/vizzy/tf', '/tf'),
            ('/model/vizzy/odometry', LaunchConfiguration('odom_topic')),
            ('/model/vizzy/cmd_vel', '/cmd_vel'),
            ('/model/vizzy/joint_states', '/joint_states'),
            ('/model/vizzy/camera/l/image_raw', '/camera/l/image_raw'),
            ('/model/vizzy/camera/r/image_raw', '/camera/r/image_raw'),
            ('/model/vizzy/camera/l/camera_info', '/camera/l/camera_info'),
            ('/model/vizzy/camera/r/camera_info', '/camera/r/camera_info'),
            ('/model/vizzy/nav_hokuyo_laser/scan', LaunchConfiguration('scan_front_topic')),
            ('/model/vizzy/nav_hokuyo_rear_laser/scan', LaunchConfiguration('scan_rear_topic')),
            ('/clock', '/clock')
        ],
        output='screen',
        emulate_tty=True,
    )

    # Use OpaqueFunction for the actions that must run after arguments are evaluated.
    deferred_actions = OpaqueFunction(function=launch_setup)

    # Only start TF publishers once the bridge is up (so /clock is available).
    tf_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[joint_state_publisher_node]
        )
    )

    # Return the complete launch description.
    return LaunchDescription(
        declared_arguments + 
        [
            bridge,
            tf_start_handler,
            deferred_actions
        ]
    )