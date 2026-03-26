"""
slam_launch.py
Standalone SLAM mapping launch – use this to build a map first,
then save it and switch to nav2_launch.py for navigation.

Usage:
  ros2 launch warehouse_robot slam_launch.py
  # Drive robot around to map the warehouse
  # Then save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


PKG = 'warehouse_robot'


def generate_launch_description():
    pkg_dir  = get_package_share_directory(PKG)
    gz_dir   = get_package_share_directory('gazebo_ros')
    slam_dir = get_package_share_directory('slam_toolbox')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'warehouse.world'),
            'verbose': 'false',
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'warehouse_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
            'use_sim_time': use_sim_time,
        }.items()
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'warehouse_robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        gazebo,
        robot_state_publisher,
        TimerAction(period=1.0, actions=[spawn_robot]),
        TimerAction(period=3.0, actions=[slam_toolbox]),
        TimerAction(period=4.0, actions=[teleop]),
        rviz_node,
    ])
