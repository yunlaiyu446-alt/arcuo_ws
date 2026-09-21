# PROJECT_CONTEXT

## 项目定位

该目录是香橙派 `/home/orangepi/arcuo_ws` 的 ROS 2 Humble 工作空间副本，负责双 Intel RealSense D435i 彩色相机启动、ArUco 标志物检测、基于已知标志物布局反算吊挂机体位姿，以及双相机结果融合。

## 关键路径

- `/Users/yuyunlai/Desktop/arcuo_ws/src/dual_d435_bringup`：按序列号启动左右两台 D435i。
- `/Users/yuyunlai/Desktop/arcuo_ws/src/aruco_detector`：用 OpenCV ArUco 从 RGB 图像估算标志物相对相机的六自由度位姿。
- `/Users/yuyunlai/Desktop/arcuo_ws/src/aruco_interfaces`：`Marker` 和 `MarkerArray` 自定义消息。
- `/Users/yuyunlai/Desktop/arcuo_ws/src/cargo_locator`：根据已知标志物位置计算吊挂机体在 `cargo` 坐标系中的位姿，并融合左右相机结果。
- `/Users/yuyunlai/Desktop/arcuo_ws/src/red_cargo_localization`：独立的 Orbbec 彩色点云红色货物定位包，不属于双 D435i ArUco 主链路。

## 当前进展

- 已确认双相机正式启动入口为 `cargo_locator/launch/dual_locator.launch.py`。
- 左右相机分别发布 `/cam_left/color/image_raw`、`/cam_left/color/camera_info` 和对应右路话题；检测结果进入 `/left/aruco_markers`、`/right/aruco_markers`。
- 默认双相机流程使用 `DICT_6X6_50`、边长 `0.3 m` 的 ArUco 标志物，不是普通 QR Code。
- 2026-09-20 用户提供并用于当前单相机实测的码图经 OpenCV 解码，实际是 `DICT_APRILTAG_36h11` 的 ID 1，而不是 `DICT_6X6_50`。当前验证该码时检测节点必须改用 `DICT_APRILTAG_36h11`；标记外侧还需保留浅色留白，避免黑色外框与深色屏幕背景融为一体。`marker_size` 应在正式位姿验证前改成实物黑色方框的实际边长。
- 每路 `cargo_locator_node` 根据 ID 1 至 4 的已知位置，计算 `/left/hanger_pose_in_cargo` 或 `/right/hanger_pose_in_cargo`；`hanger_pose_fusion_node` 最终发布 `/target_frame/pose`，`frame_id` 为 `cargo`。
- 该流程只使用 RGB 和 `CameraInfo`。双 D435i 启动配置中深度、红外、点云、IMU、相机 TF 和硬件同步均未使用或未启用。
- `cargo_locator` 内部可融合同一图像里的多个标志物测量，并使用静态位姿 EKF 平滑；双相机融合层在 50 ms 时间窗、5 cm 位置差和 15 deg 姿态差以内做等权平均，否则发布最新单路观测。
- 香橙派保留的 2026-07-22 构建日志显示 `aruco_detector_node`、`cargo_locator_node` 和 `hanger_pose_fusion_node` 编译成功，但尚未在当前 macOS 副本重新构建或运行。
- 2026-09-19 已将当前 `src`（包含 `.git`、已跟踪修改和未跟踪文件）完整备份为 `/Users/yuyunlai/Desktop/arcuo_ws_backups/arcuo_ws_src_20260919_184814.tar.gz`；SHA-256 为 `d5927b1eec020c9e8df9b936a51fba89cae2a0979d9893af9e9b7676e0f78b33`，解压后的 182 个目录/文件条目已与原目录通过 `diff -qr` 校验一致。
- 用户提供的两张操作说明截图记录了另一份路径 `/home/lucky/arcuo_ws_1/arcuo_ws` 下的单 D435 实机验证流程：启动 `realsense2_camera`，订阅 `/camera/color/image_raw`，用手机向相机展示 ArUco 图案，并以 `/aruco_markers` 非空和 `/target_frame/pose` 约 30 Hz 作为通过标准。截图属于操作步骤与预期判据，不是终端实测日志，不能单独证明流程已经实际跑通；它也不包含 Isaac Sim。
- 已在 `/Users/yuyunlai/Desktop/d435_python_camera` 创建独立的单相机替代包，使用 `pyrealsense2` 发布 `/camera/color/image_raw` 和 `/camera/color/camera_info`，用于替代实机上的 `realsense2_camera` C++ 驱动而不改动本工作空间的 ArUco 与定位节点。该包已复制到 `/home/lucky/arcuo_ws_1/arcuo_ws/src/d435_python_camera` 并完成 Ubuntu 实机验证。
- 2026-09-20 已在 Ubuntu 实机跑通单相机完整链路：`d435_python_camera` -> `/camera/color/image_raw`、`/camera/color/camera_info` -> `aruco_detector`（`DICT_APRILTAG_36h11`，检测到 ID 1）-> `/aruco_markers` -> `cargo_locator` -> `/target_frame/pose`。最新验证消息的 `frame_id` 为 `cargo`，证明话题和节点连接已贯通；当前手机显示码、临时 `marker_size:=0.3` 与配置外参不代表真实标定，因此输出数值不能作为正式定位精度结论。
- 最终 `/target_frame/pose` 的 10 秒频率测试平均约 `7–9 Hz`，最短消息间隔 `0.033 s`、最长 `0.667 s`，存在明显突发和断档。该结果足以证明流程连通，但不满足当前 `20 Hz` 控制器和 `0.25 s` 位姿过期阈值；正式闭环前必须继续优化 Python 图像发布/DDS 传输链路。

## 重要决策

- Git 仓库根目录统一为 `/Users/yuyunlai/Desktop/arcuo_ws`，而不是其下的 `src`；仓库跟踪 `src`、根目录文档和工作空间级脚本，忽略可通过 `colcon build` 重建的 `build/`、`install/`、`log/`。
- Isaac Sim 首版不必模拟深度数据：只需让两台仿真 RGB 相机发布与实机相同的 `Image`、`CameraInfo` 话题，即可复用现有 ArUco 检测与定位代码。
- Isaac 真值只用于对比 `/target_frame/pose` 的定位误差，不作为控制器正式输入。
- 仿真时不启动 `dual_d435_bringup` 的真实硬件驱动，应单独启动两个检测节点、两个定位节点和融合节点，或新增仿真专用 launch。
- 两台相机的外参当前仅通过各自 `hanger_position_in_camera_frame` 三维平移配置，且左右配置暂时相同；代码没有相机到吊挂的旋转外参。接入仿真或实机闭环前必须核对真实安装方向和标定结果。
- 标志物到 `cargo` 当前只配置位置并假设方向一致；若四块标志物不共面或朝向不同，需要补充完整姿态外参。
- 使用 conda 环境 `rosmaster`，不在全局环境安装依赖。

## 运行与测试

- 原工作空间针对 ROS 2 Humble 构建；当前 `build/install/log` 是香橙派路径下的历史产物，不能视为 macOS 可运行环境。
- 双相机实机入口：`ros2 launch cargo_locator dual_locator.launch.py`。
- 核心输出：`/target_frame/pose`（`geometry_msgs/msg/PoseStamped`，`frame_id=cargo`）。
- 图像调试输出：`/left/aruco_debug_image`、`/right/aruco_debug_image`。

## 待办事项

- 后续源码修改前先检查 `git status` 并提交当前可用节点；较大改动前额外创建完整工作空间压缩备份。
- 核对两台相机相对吊挂机体的完整六自由度外参，不能只凭相同平移参数进入正式闭环。
- 核对四个 ArUco 板在 `cargo` 坐标系中的真实位置、方向、尺寸和字典。
- 为 Isaac Sim 新增仿真专用 launch，绕过真实 `realsense2_camera` 驱动并接收模拟相机话题。
- 录制或保存实机标志板照片、相机内参和典型运行高度，用于仿真视觉一致性验证。

## 跨项目备注

- 控制器与固件项目：`/Users/yuyunlai/Desktop/V3.5.1`。
- 香橙派控制工作空间副本：`/Users/yuyunlai/Desktop/zz_ws`。
- 2026-09-21 已在 GitHub 创建仓库 `https://github.com/yunlaiyu446-alt/arcuo_ws`，当前可见性为 `PUBLIC`、描述为 `camera`，但远程尚无默认分支或提交；本地 Git 根目录 `/Users/yuyunlai/Desktop/arcuo_ws/src` 也尚未配置 `origin`，因此代码还没有推送。
- 2026-09-21 Git 根目录迁移前已创建完整快照 `/Users/yuyunlai/Desktop/arcuo_ws_backups/arcuo_ws_full_before_root_git_20260921_100602.tar.gz`，包含源码、生成目录和原 `src/.git`；SHA-256 为 `bdc63e5781603f97216c34de08c98a4b69acf425dcb75b2c6fb93b2a2843db25`，压缩包已通过 `tar -tzf` 完整性检查。
- `src/d435_python_camera` 原来是独立 Git 仓库；并入主仓库前已将完整历史保存为 `/Users/yuyunlai/Desktop/arcuo_ws_backups/d435_python_camera_history_20260921_1010.bundle`，其原 `.git` 同时移至同目录下的 `d435_python_camera_dot_git_20260921_1010`。
