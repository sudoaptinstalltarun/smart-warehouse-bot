"""
nav2_launch.py
Navigation launch using a pre-built map (AMCL localization + Nav2).
Use this after you have mapped the warehouse with slam_launch.py.

Usage:
  ros2 launch warehouse_robot nav2_launch.py map:=/path/to/warehouse_map.yaml
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
    nav2_dir = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml'),
        description='Full path to map yaml file')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'warehouse.world'),
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

    # Localization (AMCL) + Nav2
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file':  os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            'autostart':    'true',
        }.items()
    )

    # Warehouse nodes
    camera_node = Node(
        package=PKG, executable='camera_node',
        name='camera_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    vision_node = Node(
        package=PKG, executable='vision_node',
        name='vision_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    task_scheduler = Node(
        package=PKG, executable='task_scheduler',
        name='task_scheduler', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_map_file': os.path.join(pkg_dir, 'maps', 'marker_positions.json'),
        }])

    robot_controller = Node(
        package=PKG, executable='robot_controller',
        name='robot_controller', output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'warehouse_robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        map_arg, use_sim_time_arg, rviz_arg,
        gazebo,
        robot_state_publisher,
        TimerAction(period=1.0, actions=[spawn_robot]),
        TimerAction(period=3.0, actions=[nav2_stack]),
        TimerAction(period=5.0, actions=[
            camera_node, vision_node, task_scheduler, robot_controller]),
        rviz_node,
    ])
