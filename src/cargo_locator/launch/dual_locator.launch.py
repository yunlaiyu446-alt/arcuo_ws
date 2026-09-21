import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dual_config = os.path.join(
        get_package_share_directory("cargo_locator"),
        "config",
        "dual_config.yaml",
    )

    dual_d435_launch = os.path.join(
        get_package_share_directory("dual_d435_bringup"),
        "launch",
        "dual_d435.launch.py",
    )

    aruco_dictionary_name = LaunchConfiguration("aruco_dictionary_name")
    marker_size = LaunchConfiguration("marker_size")
    image_is_rectified = LaunchConfiguration("image_is_rectified")

    dual_d435 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_d435_launch),
    )

    left_aruco_detector_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="left_aruco_detector_node",
        output="screen",
        parameters=[{
            "aruco_dictionary_name": aruco_dictionary_name,
            "marker_size": marker_size,
            "image_topic": "/cam_left/color/image_raw",
            "camera_info_topic": "/cam_left/color/camera_info",
            "image_is_rectified": image_is_rectified,
            "marker_topic": "/left/aruco_markers",
            "marker_tf_prefix": "left_aruco_marker_",
            "debug_image_topic": "/left/aruco_debug_image",
        }],
    )

    right_aruco_detector_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="right_aruco_detector_node",
        output="screen",
        parameters=[{
            "aruco_dictionary_name": aruco_dictionary_name,
            "marker_size": marker_size,
            "image_topic": "/cam_right/color/image_raw",
            "camera_info_topic": "/cam_right/color/camera_info",
            "image_is_rectified": image_is_rectified,
            "marker_topic": "/right/aruco_markers",
            "marker_tf_prefix": "right_aruco_marker_",
            "debug_image_topic": "/right/aruco_debug_image",
        }],
    )

    left_cargo_locator_node = Node(
        package="cargo_locator",
        executable="cargo_locator_node",
        name="left_cargo_locator_node",
        output="screen",
        parameters=[dual_config],
    )

    right_cargo_locator_node = Node(
        package="cargo_locator",
        executable="cargo_locator_node",
        name="right_cargo_locator_node",
        output="screen",
        parameters=[dual_config],
    )

    hanger_pose_fusion_node = Node(
        package="cargo_locator",
        executable="hanger_pose_fusion_node",
        name="hanger_pose_fusion_node",
        output="screen",
        parameters=[dual_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "aruco_dictionary_name",
            default_value="DICT_6X6_50",
            description="Name of the ArUco dictionary to use.",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.3",
            description="Size of the ArUco marker in meters.",
        ),
        DeclareLaunchArgument(
            "image_is_rectified",
            default_value="False",
            description="Set to true if the input images are already rectified.",
        ),
        dual_d435,
        left_aruco_detector_node,
        right_aruco_detector_node,
        left_cargo_locator_node,
        right_cargo_locator_node,
        hanger_pose_fusion_node,
    ])
