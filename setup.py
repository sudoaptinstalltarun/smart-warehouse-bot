from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # urdf
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # rviz
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Nexus',
    maintainer_email='team.nexus@example.com',
    description='Intelligent Warehouse Picking Robot – Vision-Based Dynamic Task Optimization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Each entry maps to a ROS2 node executable ──────────────
            # Usage: ros2 run warehouse_robot <node_name>
            'camera_node     = warehouse_robot.camera_node:main',
            'vision_node     = warehouse_robot.vision_node:main',
            'task_scheduler  = warehouse_robot.task_scheduler:main',
            'robot_controller= warehouse_robot.robot_controller:main',
        ],
    },
)
