import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cargo_locator'),
        'config',
        'config.yaml'
    )
    
    cargo_locator_node = Node(
        package='cargo_locator',
        executable='cargo_locator_node',
        name='cargo_locator_node',
        parameters=[config]
    )

    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
        }]
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view'
    )

    return LaunchDescription([
        aruco_detector_node,
        cargo_locator_node,
        rqt_image_view_node,
    ])
