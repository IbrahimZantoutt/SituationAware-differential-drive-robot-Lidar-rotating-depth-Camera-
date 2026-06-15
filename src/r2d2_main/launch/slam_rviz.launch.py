import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # RViz config showing the SLAM map, laser scan, robot model and TF.
    rviz_config_path = os.path.join(
        get_package_share_directory('r2d2_main'),
        'rviz', 'robot.rviz'
    )

    # RViz ONLY. The robot_state_publisher and joint states come from
    # gazebo.launch.py, so this launch starts no other nodes and can be
    # run alongside it without conflicts.
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
