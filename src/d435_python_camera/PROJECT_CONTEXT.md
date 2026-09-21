# PROJECT_CONTEXT

## 项目定位

独立的 ROS 2 Humble `ament_python` 包，使用 `pyrealsense2` 直接读取单台 Intel RealSense D435/D435i 彩色流，并发布与现有 ArUco 定位链兼容的 ROS 图像和相机内参话题。

## 关键路径

- 包根目录：`/Users/yuyunlai/Desktop/d435_python_camera`
- 设计规格：`docs/superpowers/specs/2026-09-20-d435-python-camera-design.md`
- 目标 Ubuntu 安装位置：`/home/lucky/arcuo_ws_1/arcuo_ws/src/d435_python_camera`

## 当前进展

- 已确认首版只支持单相机和 RGB，不使用深度、红外、点云或 IMU。
- 已确认保持 `/camera/color/image_raw` 与 `/camera/color/camera_info` 接口，以复用现有 C++ ArUco 检测节点。
- 已实现 `ament_python` 包骨架、RealSense 内参映射、单相机 ROS 2 发布节点、默认参数、launch 文件和中文 README。
- 节点入口为 `d435_python_camera_node = d435_python_camera.camera_node:main`，默认发布 `640x480@30Hz` 的 `bgr8` 彩色图像。
- 便携测试使用 Python 标准库 `unittest`，避免在 macOS 全局环境安装 `pytest`。
- macOS 完成 8 个便携测试、Python 语法编译检查，以及使用伪造 ROS/RealSense 模块进行的一次消息发布与幂等关停集成检查；这些结果不能替代 Ubuntu ROS 2 和真实 D435 验证。
- 2026-09-20 已在 Ubuntu ROS 2 Humble 与真实 D435 上完成构建和运行验证：纯 `pyrealsense2` 取帧约 `29.28 Hz` 且无超时；节点成功发布图像和内参，C++ 检测节点识别 `DICT_APRILTAG_36h11` ID 1，`cargo_locator` 最终发布 `frame_id=cargo` 的 `/target_frame/pose`。
- 性能测试发现 Humble Python 消息在调试模式下执行 `message.data = image.tobytes()` 会逐字节校验，导致相机节点约占满一个 CPU 核且链路约 `3–4 Hz`；使用 `PYTHONOPTIMIZE=1` 后相机节点降至约 `4.6% CPU`、端到端链路约 `10 Hz`。永久源码优化尚未实施。
- 最终 `/target_frame/pose` 的 10 秒实测平均约 `7–9 Hz`，消息间隔最短 `0.033 s`、最长 `0.667 s`、标准差约 `0.127 s`，表明链路存在突发和长时间断档；这不是可供现有 `20 Hz` 控制器直接使用的稳定输出。

## 重要决策

- 使用 `pyrealsense2` 直接读取相机。
- 不依赖 Python `cv_bridge`，直接构造 `sensor_msgs/msg/Image`。
- 首版使用 ROS 节点时钟，不引入硬件时间同步。
- 包保持独立，便于用户从 macOS 桌面复制到 Ubuntu 工作空间。

## 运行与测试

- macOS 便携测试：`python3 -m unittest discover -s test -v`。
- Python 语法检查：`python3 -m compileall setup.py d435_python_camera launch`。
- Ubuntu 构建：`colcon build --symlink-install --packages-select d435_python_camera`。
- 当前 Ubuntu 验证启动：`PYTHONOPTIMIZE=1 ros2 launch d435_python_camera single_d435.launch.py`。

## 待办事项

- Ubuntu 现有 Conda `rosmaster` 是 Python 3.13.12，与 ROS 2 Humble 的 CPython 3.10 `rclpy` ABI 不兼容；当前实测必须使用 `/usr/bin/python3` 3.10，GUI 工具也应先 `conda deactivate` 或直接用 `/usr/bin/python3` 启动。
- 按测试驱动方式将图像数据改为高效的 `array.array('B')` 路径，避免依赖全局 `PYTHONOPTIMIZE=1`，并加入节点内部采集/发布 FPS 统计以区分发布端和 DDS 传输端频率。
- 在真实码板尺寸和安装外参确定后更新 `marker_size`、标记布局及相机外参，再做定位精度验证。

## 跨项目备注

- 现有视觉定位工作空间副本：`/Users/yuyunlai/Desktop/arcuo_ws`
- 控制与固件项目：`/Users/yuyunlai/Desktop/V3.5.1`
