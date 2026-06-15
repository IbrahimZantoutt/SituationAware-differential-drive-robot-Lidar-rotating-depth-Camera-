import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('r2d2_main')
    urdf_path = os.path.join(share_dir, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # robot.urdf is loaded raw (not processed through xacro), so the
    # $(find r2d2_main) used by the gazebo_ros2_control <parameters> tag
    # won't expand on its own. Resolve it to the installed share path here.
    robot_description = robot_description.replace('$(find r2d2_main)', share_dir)

    # Spawn robot in Gazebo (controller_manager comes up with it)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'nav_robot'],
        output='screen'
    )

    # Publishes /joint_states (read the rotator angle here)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Position-commands the rotator joint
    rotator_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rotator_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
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

        # Spawn controllers only after the robot (and its controller_manager)
        # is up, then chain the position controller after the broadcaster.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rotator_position_controller_spawner],
            )
        ),
    ])
