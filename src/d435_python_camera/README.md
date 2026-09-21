# d435_python_camera

使用 `pyrealsense2` 直接读取一台 Intel RealSense D435/D435i 的彩色流，并发布现有 ArUco 检测节点所需的 ROS 2 话题。该包只启用 RGB，不启用深度、红外、点云或 IMU。

## 默认接口

- 图像：`/camera/color/image_raw`（`sensor_msgs/msg/Image`，`bgr8`）
- 相机内参：`/camera/color/camera_info`（`sensor_msgs/msg/CameraInfo`）
- 坐标系：`camera_color_optical_frame`
- 分辨率和帧率：`640x480@30Hz`

## 1. 复制到 Ubuntu

将整个文件夹复制到工作空间：

```text
/home/lucky/arcuo_ws_1/arcuo_ws/src/d435_python_camera
```

不要只复制 `camera_node.py`，ROS 2 构建还需要 `package.xml`、`setup.py`、`resource/`、`config/` 和 `launch/`。

## 2. 准备环境

不要在 Conda `base` 或系统全局环境中安装 Python 包。使用项目环境：

```bash
conda activate rosmaster
source /opt/ros/humble/setup.bash
python3 --version
```

ROS 2 Humble 的 Ubuntu 22.04 二进制环境以 Python 3.10 为基准。确认同一个 `python3` 能加载全部运行依赖：

```bash
python3 -c "import rclpy, sensor_msgs, numpy, pyrealsense2; print('Python dependencies OK')"
```

如果只有 `pyrealsense2` 缺失，应安装到 `rosmaster` 环境，而不是全局环境：

```bash
python3 -m pip install pyrealsense2
```

如果 `rclpy` 在 `rosmaster` 中无法导入，先不要构建；这表示 Conda Python 与系统 ROS 2 Python 尚未兼容，需要先统一到 Python 3.10。

## 3. 确保 D435 未被占用

启动本节点前，必须停止所有 `realsense2_camera` 进程，否则 Python SDK 无法独占打开同一台 D435：

```bash
ps -eo pid,ppid,tty,stat,cmd | grep -E '[r]ealsense2_camera|[r]s_launch.py'
```

仅在确认 PID 属于旧 RealSense 驱动后再停止对应进程。

## 4. 构建

```bash
cd /home/lucky/arcuo_ws_1/arcuo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d435_python_camera
source install/local_setup.bash
```

确认 ROS 2 能找到节点：

```bash
ros2 pkg executables d435_python_camera
```

预期包含：

```text
d435_python_camera d435_python_camera_node
```

## 5. 启动单相机

```bash
ros2 launch d435_python_camera single_d435.launch.py
```

如需指定相机序列号，可修改 `config/camera.yaml` 中的 `serial_no`。保持空字符串时使用发现的第一台 RealSense 设备。

## 6. 验证相机话题

保持相机节点运行，另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lucky/arcuo_ws_1/arcuo_ws/install/local_setup.bash

timeout 5 ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

图像应持续有频率；`CameraInfo` 的 `width`、`height`、`k`、`d` 和 `frame_id` 应为非空有效值。

## 7. 接入现有 ArUco 检测节点

在新终端启动检测节点，并显式使用项目实际标志物字典和尺寸：

```bash
source /opt/ros/humble/setup.bash
source /home/lucky/arcuo_ws_1/arcuo_ws/install/local_setup.bash

ros2 run aruco_detector aruco_detector_node --ros-args \
  -p aruco_dictionary_name:=DICT_6X6_50 \
  -p marker_size:=0.3 \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p publish_empty_markers:=true
```

检查检测节点是否持续处理图像：

```bash
timeout 5 ros2 topic hz /aruco_debug_image
```

## 参数

参数文件位于 `config/camera.yaml`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `serial_no` | `""` | 指定设备序列号；空值使用第一台设备 |
| `width` | `640` | 彩色图像宽度 |
| `height` | `480` | 彩色图像高度 |
| `fps` | `30` | 请求帧率 |
| `frame_id` | `camera_color_optical_frame` | 两种消息共同的坐标系 |
| `image_topic` | `/camera/color/image_raw` | 图像输出话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参输出话题 |
| `frame_timeout_ms` | `1000` | 等待一帧的最大时间 |

## 已知的独立问题

如果执行 `ros2 topic echo /aruco_markers` 时出现：

```text
The message type 'aruco_interfaces/msg/MarkerArray' is invalid
```

这是 `aruco_interfaces` 的 Python 消息类型支持或环境加载问题，与本相机节点采用 Python SDK 无关，需要单独重建和验证接口包。

## 本地便携测试

这些测试不需要 ROS 2 或 D435：

```bash
python3 -m unittest discover -s test -v
python3 -m compileall setup.py d435_python_camera launch
```

macOS 只能执行上述便携测试；真实相机、ROS 2 构建和话题频率必须在 Ubuntu 上验证。
