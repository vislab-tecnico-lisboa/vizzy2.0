# This file had to be created as a dedicated way to launch RViz2 with custom configuration.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction
from nav2_common.launch import ReplaceString

def generate_launch_description():

    # Build the URDF path as a Substitution.
    urdf_path = os.path.join(
        get_package_share_directory('vizzy_description'),
        'vizzy_final_model.urdf'
    )

    # Use ReplaceString to patch your rviz file.
    patched_rviz = ReplaceString(
        source_file=os.path.join(
            get_package_share_directory('vizzy_launch'),
            'config',
            'custom_config.rviz'
        ),
        # Perform a simple find/replace on the raw file.
        replacements={
          # match the exact line in your .rviz
          "Description File: mock/value":
          f"Description File: {urdf_path}"
        },
    )

    # Pass the *path* that ReplaceString generates into rviz2.
    rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', patched_rviz],
      output='screen'
    )

    # Timer action to delay the launch of RViz2 for 10 seconds.
    # This is useful to ensure that all other nodes are up and running before RViz2 starts.
    # This also helps to reduce the number of 
    # "[rviz2]: Message Filter dropping message: frame 'nav_hokuyo_laser_link' at time 0.150 for reason 'discarding message because the queue is full'"
    # messages that are printed in the terminal.
    rviz2_timer = TimerAction(
        period=10.0,
        actions=[rviz2_node]
    )

    # Return the launch description with the timer action.
    return LaunchDescription([
        rviz2_timer
    ])
