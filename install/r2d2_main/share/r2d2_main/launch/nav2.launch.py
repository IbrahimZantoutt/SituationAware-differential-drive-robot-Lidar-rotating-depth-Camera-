import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_params_path = os.path.join(
        get_package_share_directory('r2d2_main'),
        'config', 'nav2_params.yaml'
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Brings up the Nav2 navigation stack (planner, controller, bt_navigator,
    # behaviors, costmaps...). No map_server / AMCL: slam_toolbox publishes the
    # live map and the map->odom transform.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params_path,
        }.items()
    )

    return LaunchDescription([
        nav2,
    ])
