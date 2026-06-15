import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('r2d2_main'),
        'urdf', 'robot.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Start Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                os.path.join(get_package_share_directory('r2d2_main'),
                'worlds', 'world.sdf')],
            output='screen'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'nav_robot'],
            output='screen'
        ),

        # add this node to your LaunchDescription:
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(
                    get_package_share_directory('r2d2_main'),
                    'config', 'mapper_params.yaml'
                ),
                {'use_sim_time': True},
            ],
            output='screen'
        ),
    ])