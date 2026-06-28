import os 

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_launch_file = PathJoinSubstitution(
        FindPackageShare("realsense2_camera", "launch", "rs_launch.py")
    )
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
                # 配置 D435 相机参数
                'device_type': 'd435', # 指定设备类型
                'enable_color': 'true', # 启用 RGB 相机
                'color_width': '640', # 设置分辨率
                'color_height': '480',
                'color_fps': '30', # 设置帧率
                'enable_depth': 'false', # 如果不需要深度信息，可以禁用以节省资源
                'enable_infra1': 'false', # 禁用红外1
                'enable_infra2': 'false', # 禁用红外2
                # 'initial_reset': 'true', # 如果遇到连接问题，可以尝试设置为 true
                # 可以根据需要添加更多参数...
            }.items()
    )
    config = os.path.join(get_package_share_directory("aruco_detector"),
                        "conifg",
                        ""
                        )

    aruco_detector_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        parameters=[config]
    )

    ld = LaunchDescription()

    ld.add_action(realsense_node)
    ld.add_action(aruco_detector_node)

    return ld

