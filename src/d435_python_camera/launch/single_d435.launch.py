"""Launch one D435/D435i color camera through the Python RealSense SDK."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('d435_python_camera')
    config_path = os.path.join(package_share, 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='d435_python_camera',
            executable='d435_python_camera_node',
            name='d435_python_camera_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
