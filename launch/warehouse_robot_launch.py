"""
warehouse_robot_launch.py
Master launch file – starts everything:
  1. Gazebo with warehouse world
  2. Robot state publisher (URDF)
  3. Spawn robot in Gazebo
  4. SLAM Toolbox (mapping mode)
  5. Nav2 stack
  6. warehouse_robot nodes (camera, vision, scheduler, controller)
  7. RViz2

Usage:
  ros2 launch warehouse_robot warehouse_robot_launch.py
  ros2 launch warehouse_robot warehouse_robot_launch.py use_sim_time:=true slam:=true rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, RegisterEventHandler, LogInfo,
    TimerAction, GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution,
    PythonExpression, Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Package paths ─────────────────────────────────────────────
PKG = 'warehouse_robot'


def generate_launch_description():
    pkg_dir   = get_package_share_directory(PKG)
    nav2_dir  = get_package_share_directory('nav2_bringup')
    slam_dir  = get_package_share_directory('slam_toolbox')
    gz_dir    = get_package_share_directory('gazebo_ros')

    # ── Launch arguments ─────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Run SLAM Toolbox for mapping')

    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Run Nav2 navigation stack')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 visualization')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'warehouse.world'),
        description='Gazebo world file')

    robot_x_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    robot_y_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    robot_z_arg = DeclareLaunchArgument('z_pose', default_value='0.05')

    # ── Configurations ───────────────────────────────────────
    use_sim_time  = LaunchConfiguration('use_sim_time')
    slam          = LaunchConfiguration('slam')
    nav2          = LaunchConfiguration('nav2')
    rviz          = LaunchConfiguration('rviz')
    world         = LaunchConfiguration('world')

    # ── URDF via xacro ───────────────────────────────────────
    urdf_file = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')
    robot_description_cmd = Command(['xacro ', urdf_file])

    # ── 1. Gazebo ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'pause': 'false',
        }.items()
    )

    # ── 2. Robot State Publisher ─────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': use_sim_time,
        }]
    )

    # ── 3. Joint State Publisher ─────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 4. Spawn Robot in Gazebo ─────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_warehouse_robot',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'warehouse_robot',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
        ],
        output='screen'
    )

    # ── 5. SLAM Toolbox ──────────────────────────────────────
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(slam)
    )

    # ── 6. Nav2 ──────────────────────────────────────────────
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(nav2)
    )

    # ── 7. Warehouse Robot Nodes ─────────────────────────────
    camera_node = Node(
        package=PKG,
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    vision_node = Node(
        package=PKG,
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'use_sim_time':   use_sim_time,
            'confirm_frames': 3,
            'max_marker_dist': 3.0,
            'reproj_error_thresh': 2.5,
            'debug_image': True,
        }]
    )

    task_scheduler = Node(
        package=PKG,
        executable='task_scheduler',
        name='task_scheduler',
        output='screen',
        parameters=[{
            'use_sim_time':     use_sim_time,
            'replan_tolerance': 0.05,
            'goal_timeout':     60.0,
            'map_frame':        'map',
            'base_frame':       'base_footprint',
            'marker_map_file':  os.path.join(pkg_dir, 'maps', 'marker_positions.json'),
        }]
    )

    robot_controller = Node(
        package=PKG,
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'stop_distance':         0.30,
            'slow_distance':         0.60,
            'max_linear_vel':        0.26,
            'max_angular_vel':       0.80,
            'vision_watchdog_timeout': 3.0,
        }]
    )

    # ── 8. RViz2 ─────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'rviz', 'warehouse_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    # ── Assemble launch description ───────────────────────────
    return LaunchDescription([
        # Args
        use_sim_time_arg,
        slam_arg,
        nav2_arg,
        rviz_arg,
        world_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,

        # Infrastructure
        gazebo,
        robot_state_publisher,
        joint_state_publisher,

        # Spawn robot after Gazebo is up (1 second delay)
        TimerAction(period=1.0, actions=[spawn_robot]),

        # SLAM + Nav2 (after robot is spawned)
        TimerAction(period=3.0, actions=[slam_toolbox]),
        TimerAction(period=5.0, actions=[nav2_stack]),

        # Warehouse nodes
        TimerAction(period=4.0, actions=[
            camera_node,
            vision_node,
            task_scheduler,
            robot_controller,
        ]),

        # Visualization
        rviz_node,
    ])
