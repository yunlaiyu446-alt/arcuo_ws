from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("red_cargo_localization"))
    default_parameters = str(package_share / "config" / "red_cargo.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_parameters),
        DeclareLaunchArgument(
            "input_topic", default_value="/camera/depth_registered/points"
        ),
        Node(
            package="red_cargo_localization",
            executable="red_cargo_localizer_node",
            name="red_cargo_localizer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"input_topic": LaunchConfiguration("input_topic")},
            ],
        ),
    ])
