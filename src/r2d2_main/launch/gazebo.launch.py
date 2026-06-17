import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('r2d2_main')
    urdf_path = os.path.join(share_dir, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # robot.urdf is loaded raw (not processed through xacro). Kept for any
    # remaining $(find r2d2_main) references; harmless if there are none.
    robot_description = robot_description.replace('$(find r2d2_main)', share_dir)

    # Spawn robot in Gazebo. The camera rotator is driven by the standalone
    # gazebo_ros joint plugins baked into the URDF (no controller_manager).
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'nav_robot',
                   '-timeout', '120.0'],   # safety net: don't give up before Gazebo is ready
        output='screen'
    )

    return LaunchDescription([
        # The default online model DB (models.gazebosim.org) is defunct; leaving it
        # set makes Gazebo hang ~30s on startup trying to reach it, which pushes
        # /spawn_entity past spawn_entity.py's timeout. Disable it and point Gazebo
        # at the local models (sun, ground_plane live here) so startup is fast.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'),

        # Start Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',      # publishes /clock (needed for use_sim_time)
                '-s', 'libgazebo_ros_factory.so',   # spawn/delete entity services
                os.path.join(share_dir, 'worlds', 'world.sdf')],
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
        spawn_entity,

        # add this node to your LaunchDescription:
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(share_dir, 'config', 'mapper_params.yaml'),
                {'use_sim_time': True},
            ],
            output='screen'
        ),
    ])
