# 单 D435 Python SDK ROS 2 相机节点设计

## 目标

创建一个可单独复制到 Ubuntu ROS 2 Humble 工作空间 `src/` 下构建的 `ament_python` 包。节点使用 `pyrealsense2` 直接打开一台 Intel RealSense D435/D435i，只采集彩色图像，并保持现有 ArUco 检测链路的 ROS 2 接口不变。

## 不在本次范围内

- 不启用深度、红外、点云或 IMU。
- 不在相机节点内执行 ArUco 检测或定位。
- 不修改现有 `aruco_detector`、`aruco_interfaces` 或 `cargo_locator`。
- 不实现双相机同步；双相机版应在单相机验证通过后另行设计。

## 目录与交付形式

包根目录固定为：

```text
/Users/yuyunlai/Desktop/d435_python_camera
```

目录结构：

```text
d435_python_camera/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── PROJECT_CONTEXT.md
├── resource/
│   └── d435_python_camera
├── d435_python_camera/
│   ├── __init__.py
│   ├── camera_info.py
│   └── camera_node.py
├── config/
│   └── camera.yaml
├── launch/
│   └── single_d435.launch.py
├── test/
│   └── test_camera_info.py
└── docs/superpowers/specs/
    └── 2026-09-20-d435-python-camera-design.md
```

用户将整个 `d435_python_camera` 文件夹复制到 Ubuntu 的 `/home/lucky/arcuo_ws_1/arcuo_ws/src/`，再通过 `colcon` 构建。

## ROS 2 接口

节点名：`d435_python_camera_node`

发布话题：

- `/camera/color/image_raw`：`sensor_msgs/msg/Image`，编码为 `bgr8`。
- `/camera/color/camera_info`：`sensor_msgs/msg/CameraInfo`。

两种消息使用相同时间戳与 `frame_id=camera_color_optical_frame`，采用传感器数据 QoS，以兼容现有 C++ `aruco_detector_node` 的 `SensorDataQoS` 订阅。

参数：

- `serial_no`：可选相机序列号；空字符串表示使用发现的第一台设备。
- `width`：默认 `640`。
- `height`：默认 `480`。
- `fps`：默认 `30`。
- `frame_id`：默认 `camera_color_optical_frame`。
- `image_topic`：默认 `/camera/color/image_raw`。
- `camera_info_topic`：默认 `/camera/color/camera_info`。
- `frame_timeout_ms`：默认 `1000`。

## 数据处理

1. 用 `pyrealsense2.pipeline` 和 `config` 选择设备并只启用彩色流。
2. 从启动后的彩色流 profile 读取真实内参及畸变参数。
3. 将每帧 BGR8 数据直接写入 `sensor_msgs/Image.data`，不依赖 Python `cv_bridge`。
4. 根据内参生成 `CameraInfo` 的 `k`、`r`、`p`、`d`、宽度和高度。
5. 每帧发布一对时间戳一致的 `Image` 和 `CameraInfo`。

首版使用 ROS 节点时钟作为消息时间戳，避免直接混用 RealSense 硬件毫秒时间和 ROS 时钟。单相机定位跑通后，再评估是否需要硬件时间同步。

## 异常与退出处理

- 找不到设备或请求的彩色流配置不可用时，打印明确错误并退出，不能保持一个无数据的假运行节点。
- 等待帧超时时节流打印警告并继续重试。
- 收到退出信号或节点销毁时调用 `pipeline.stop()`，释放 D435，避免产生脱离终端的残留相机进程。
- 启动前要求停止 `realsense2_camera`，因为同一台 D435 不能同时被两个进程独占。

## 依赖与环境

- ROS 2 Humble。
- `rclpy`、`sensor_msgs`、`launch`、`launch_ros`。
- `pyrealsense2` 和 NumPy。
- 不在系统全局或 Conda `base` 中直接安装依赖；Ubuntu 侧应使用项目约定的 `rosmaster` 环境，并先验证该环境的 Python 版本与 ROS 2 Humble Python 3.10 兼容。

## 验证标准

无硬件测试：

- `camera_info.py` 的内参映射单元测试通过。
- ROS 包可由 `colcon build --symlink-install --packages-select d435_python_camera` 构建。
- `ros2 pkg executables d435_python_camera` 能列出节点。

D435 实机测试：

- 相机节点启动后不存在重复或残留的 `/camera` 节点。
- `ros2 topic hz /camera/color/image_raw` 有稳定输出。
- `Image` 与 `CameraInfo` 的宽高、时间戳和 `frame_id` 一致。
- 现有 `aruco_detector_node` 能收到相机标定并持续发布 `/aruco_debug_image`。
- 显示 `DICT_6X6_50` 标志物后，`/aruco_markers` 中出现有效 ID 和位姿。

## 已知的独立问题

当前 Ubuntu 出现的 `The message type 'aruco_interfaces/msg/MarkerArray' is invalid` 属于 `aruco_interfaces` 的 Python 类型支持或环境加载问题，与相机节点采用 C++ 驱动还是 Python SDK 无关。相机包完成后仍需单独修复并验证该接口包。
