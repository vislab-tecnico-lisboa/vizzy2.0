# This file had to be created to replace the former "vizzy_spawn_launch.xml" file.
# The original file was not working properly due to the xacro command execution through the xml launcher, 
# and the new file is a complete rewrite in python.
# The principle is exactly the same as the original file, but with a few key changes:
# 1. The resulting URDF file from the Command() execution is now written to a permanent file before being passed to the robot_state_publisher and spawn_node.
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart

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
    return []

def print_urdf_file(context, file_path):
    # Read and log the content of the file after it has been written
    try:
        with open(file_path, 'r') as f:
            file_contents = f.read()
        print("Created URDF file contents:")
        print(file_contents)
    except Exception as e:
        print(f"Error reading file: {e}")
    return []

def generate_launch_description():
    
    # Declare launch arguments.
    robot = DeclareLaunchArgument(
        'robot', default_value='vizzy', description='Name of the robot model'
    )
    pose = DeclareLaunchArgument(
        'pose',
        default_value='-x 0.0 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0',
        description='Spawn pose of the robot'
    )

    pose_x = DeclareLaunchArgument('pose_x', default_value='0.0', description='X position')
    pose_y = DeclareLaunchArgument('pose_y', default_value='0.0', description='Y position')
    pose_z = DeclareLaunchArgument('pose_z', default_value='0.0', description='Z position')
    pose_R = DeclareLaunchArgument('pose_R', default_value='0.0', description='Roll')
    pose_P = DeclareLaunchArgument('pose_P', default_value='0.0', description='Pitch')
    pose_Y = DeclareLaunchArgument('pose_Y', default_value='3.14', description='Yaw')

    urdf_file = DeclareLaunchArgument(
        'urdf_file', default_value='vizzy.urdf.xacro', description='URDF xacro file'
    )
    use_yarp = DeclareLaunchArgument(
        'use_yarp', default_value='false', description='Use YARP'
    )
    use_full_gazebo_model = DeclareLaunchArgument(
        'use_full_gazebo_model', default_value='false', description='Use full Gazebo model'
    )
    use_full_hand_model = DeclareLaunchArgument(
        'use_full_hand_model', default_value='false', description='Use full hand model'
    )
    disable_laser = DeclareLaunchArgument(
        'disable_laser', default_value='true', description='Disable laser sensor'
    )
    disable_3d_sensor = DeclareLaunchArgument(
        'disable_3d_sensor', default_value='true', description='Disable 3D sensor'
    )

    # Generate the robot_description by processing the URDF file with xacro.
    robot_description_command = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('vizzy_description'), 'robots', LaunchConfiguration('urdf_file')]), ' ',
        'use_yarp:=', LaunchConfiguration('use_yarp'), ' ',
        'use_full_gazebo_model:=', LaunchConfiguration('use_full_gazebo_model'), ' ',
        'use_full_hand_model:=', LaunchConfiguration('use_full_hand_model'), ' ',
        'disable_laser:=', LaunchConfiguration('disable_laser'), ' ',
        'disable_3d_sensor:=', LaunchConfiguration('disable_3d_sensor')
    ])

    # Use the ament index to get the package share directory and build the file path string.
    package_share = get_package_share_directory('vizzy_description')
    urdf_file_path = os.path.join(package_share, 'vizzy_final_model.urdf')

    # Write the URDF to the temporary file.
    write_file_action = OpaqueFunction(
        function=lambda context: write_urdf_to_file(context, robot_description_command, urdf_file_path)
    )

    # Create the joint_state_publisher node.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Create the robot_state_publisher node.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': True,
                'enable_tf_static': True,
                'robot_description': ParameterValue(
                    robot_description_command, value_type=str
                )
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # Spawn the robot using ros_gz_sim with the file path.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-urdf',
            '-file', urdf_file_path,
            '-x', LaunchConfiguration('pose_x'),
            '-y', LaunchConfiguration('pose_y'),
            '-z', LaunchConfiguration('pose_z'),
            '-R', LaunchConfiguration('pose_R'),
            '-P', LaunchConfiguration('pose_P'),
            '-Y', LaunchConfiguration('pose_Y'),
            '-model', LaunchConfiguration('robot'),
            '-v4'
        ]
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
    #       /model/vizzy/nav_hokuyo_laser/scan/points  ->  /nav_hokuyo_laser/front/points
    #       /model/vizzy/nav_hokuyo_rear_laser/scan/points  ->  /nav_hokuyo_laser/rear/points
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
            '/model/vizzy/nav_hokuyo_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/vizzy/nav_hokuyo_rear_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/vizzy/tf', '/tf'),
            ('/model/vizzy/odometry', '/odom'),
            ('/model/vizzy/cmd_vel', '/cmd_vel'),
            ('/model/vizzy/joint_states', '/joint_states'),
            ('/model/vizzy/camera/l/image_raw', '/camera/l/image_raw'),
            ('/model/vizzy/camera/r/image_raw', '/camera/r/image_raw'),
            ('/model/vizzy/camera/l/camera_info', '/camera/l/camera_info'),
            ('/model/vizzy/camera/r/camera_info', '/camera/r/camera_info'),
            ('/model/vizzy/nav_hokuyo_laser/scan', '/nav_hokuyo_laser/front/scan'),
            ('/model/vizzy/nav_hokuyo_rear_laser/scan', '/nav_hokuyo_laser/rear/scan'),
            ('/model/vizzy/nav_hokuyo_laser/scan/points', '/nav_hokuyo_laser/front/points'),
            ('/model/vizzy/nav_hokuyo_rear_laser/scan/points', '/nav_hokuyo_laser/rear/points'),
            ('/clock', '/clock')
        ],
        output='screen',
        emulate_tty=True,
    )

    # Only start TF publishers once the bridge is up (so /clock is available).
    tf_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[joint_state_publisher_node, robot_state_publisher_node]
        )
    )

    return LaunchDescription([
        pose,
        pose_x,
        pose_y,
        pose_z,
        pose_R,
        pose_P,
        pose_Y,
        robot,
        bridge,
        use_yarp,
        urdf_file,
        spawn_node,
        disable_laser,
        disable_3d_sensor,
        write_file_action,
        use_full_hand_model,
        use_full_gazebo_model,
        tf_start_handler,
    ])
