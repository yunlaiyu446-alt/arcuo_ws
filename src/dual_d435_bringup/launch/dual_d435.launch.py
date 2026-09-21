from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _camera_launch(name_cfg, namespace_cfg, serial_cfg):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("realsense2_camera") + "/launch/rs_launch.py"
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration(name_cfg),
            "camera_namespace": LaunchConfiguration(namespace_cfg),
            "serial_no": LaunchConfiguration(serial_cfg),
            "log_level": LaunchConfiguration("log_level"),
            "output": LaunchConfiguration("output"),
            "enable_color": LaunchConfiguration("enable_color"),
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "enable_depth": LaunchConfiguration("enable_depth"),
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            "publish_tf": LaunchConfiguration("publish_tf"),
            "tf_publish_rate": "0.0",
            "enable_sync": LaunchConfiguration("enable_sync"),
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("left_camera_name", default_value="cam_left"),
            DeclareLaunchArgument("right_camera_name", default_value="cam_right"),
            DeclareLaunchArgument("left_camera_namespace", default_value=""),
            DeclareLaunchArgument("right_camera_namespace", default_value=""),
            DeclareLaunchArgument("left_serial_no", default_value="'_102422073387'"),
            DeclareLaunchArgument("right_serial_no", default_value="'_337322073533'"),
            DeclareLaunchArgument("color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("enable_sync", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("output", default_value="screen"),
            _camera_launch("left_camera_name", "left_camera_namespace", "left_serial_no"),
            _camera_launch("right_camera_name", "right_camera_namespace", "right_serial_no"),
        ]
    )
