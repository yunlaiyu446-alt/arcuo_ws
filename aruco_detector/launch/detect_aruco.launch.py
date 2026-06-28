from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # --- 配置参数 ---
    aruco_dictionary_name = LaunchConfiguration('aruco_dictionary_name')
    marker_size = LaunchConfiguration('marker_size')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    image_is_rectified = LaunchConfiguration('image_is_rectified')
    marker_topic = LaunchConfiguration('marker_topic')
    marker_tf_prefix = LaunchConfiguration('marker_tf_prefix')

    # --- 声明可从命令行覆盖的 Launch 参数 ---
    declare_aruco_dictionary_name_cmd = DeclareLaunchArgument(
        'aruco_dictionary_name',
        default_value='DICT_6X6_50',
        description='Name of the ArUco dictionary to use (e.g., DICT_4X4_50)'
    )

    declare_marker_size_cmd = DeclareLaunchArgument(
        'marker_size',
        default_value='0.3',
        description='Size of the ArUco marker in meters'
    )

    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='Topic name for the input image stream'
    )

    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Topic name for the camera info'
    )

    declare_image_is_rectified_cmd = DeclareLaunchArgument(
        'image_is_rectified',
        default_value='False',
        description='Set to true if the input image is already rectified'
    )

    declare_marker_topic_cmd = DeclareLaunchArgument(
        'marker_topic',
        default_value='aruco_markers',
        description='Topic name to publish detected markers'
    )

    declare_marker_tf_prefix_cmd = DeclareLaunchArgument(
        'marker_tf_prefix',
        default_value='aruco_marker',
        description='Prefix for the TF frame of detected markers'
    )

    # --- 启动 RealSense D435 节点 ---
    # 假设您已经安装了 realsense2_camera 包
    # 此部分包含启动 D435 相机的必要配置
    realsense_launch_file = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py' # 或者 rs_camera.launch.py，取决于包版本
    ])

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

    # --- 启动 ArUco 检测节点 ---
    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node', # 确保 CMakeLists.txt 中定义了此可执行文件
        name='aruco_detector_node',
        output='screen', # 将日志输出到终端
        parameters=[{
            'aruco_dictionary_name': aruco_dictionary_name,
            'marker_size': marker_size,
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'image_is_rectified': image_is_rectified,
            'marker_topic': marker_topic,
            'marker_tf_prefix': marker_tf_prefix,
        }],
        arguments=['--ros-args', '--log-level', 'info'] # 可设置日志级别
    )

    # --- 创建 Launch 描述 ---
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_aruco_dictionary_name_cmd)
    ld.add_action(declare_marker_size_cmd)
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_camera_info_topic_cmd)
    ld.add_action(declare_image_is_rectified_cmd)
    ld.add_action(declare_marker_topic_cmd)
    ld.add_action(declare_marker_tf_prefix_cmd)

    # 添加节点启动
    ld.add_action(realsense_node)
    ld.add_action(aruco_detector_node)

    return ld



