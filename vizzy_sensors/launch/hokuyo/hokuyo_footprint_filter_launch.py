from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # --- Declare Launch Arguments ---
    scan_topic_front_arg = DeclareLaunchArgument(
        'scan_topic_front',
        default_value='/nav_hokuyo_laser/front/scan',
        description='The input laser scan topic.'
    )

    scan_topic_filtered_front_arg = DeclareLaunchArgument(
        'scan_topic_filtered_front',
        default_value='/scan_filtered',
        description='The output filtered laser scan topic.'
    )

    scan_front = LaunchConfiguration('scan_topic_front')
    scan_filtered_front = LaunchConfiguration('scan_topic_filtered_front')

    return LaunchDescription([

        scan_topic_front_arg,
        scan_topic_filtered_front_arg,

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain_node', 
            name='hokuyo_footprint_filter',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory('vizzy_sensors'),
                    'config', 'footprint_filter.yaml',
                ])],

            remappings=[
                ('scan', scan_front),
                ('scan_filtered', scan_filtered_front),
            ],
        )
    ])