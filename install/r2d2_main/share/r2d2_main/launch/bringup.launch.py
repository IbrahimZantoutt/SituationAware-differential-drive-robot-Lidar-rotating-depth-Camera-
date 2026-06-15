import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_launch = os.path.join(
        get_package_share_directory('r2d2_main'), 'launch'
    )

    # Gazebo + world + robot_state_publisher + spawn + slam_toolbox
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, 'gazebo.launch.py')
        )
    )

    # RViz showing the map / scan / costmaps / path
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, 'slam_rviz.launch.py')
        )
    )

    # Nav2 stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, 'nav2.launch.py')
        )
    )

    # Give Gazebo + SLAM a few seconds to start publishing TF / scan / odom
    # before Nav2 comes up, so its costmaps initialize cleanly.
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2])

    return LaunchDescription([
        gazebo,
        rviz,
        delayed_nav2,
    ])
