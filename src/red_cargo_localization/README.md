# red_cargo_localization

该 ROS 2 Humble 包从 Orbbec 彩色点云中提取最大的红色三维点簇，并发布该点簇所有原始点的算术平均位置。它是独立的下游包，不修改 Orbbec 驱动。

## 接口

默认订阅：

- `/camera/depth_registered/points` (`sensor_msgs/msg/PointCloud2`)，要求包含 `x/y/z` 和 `rgb` 字段。

节点 `red_cargo_localizer` 发布：

- `~/centroid` (`geometry_msgs/msg/PointStamped`)：检测成功时的货物质心。
- `~/red_points` (`sensor_msgs/msg/PointCloud2`)：最大红色点簇，用于 RViz 调试。
- `~/detected` (`std_msgs/msg/Bool`)：本帧是否检测成功。
- `~/point_count` (`std_msgs/msg/UInt32`)：本帧质心使用的点数。

没有有效目标时不发布新的 `centroid`，只发布 `detected=false` 和 `point_count=0`。输出时间戳和坐标系与输入点云相同。Orbbec 默认使用彩色相机光学坐标系：x 向右、y 向下、z 向前；相机朝下时 z 通常大致指向地面。

## 构建与运行

```bash
cd /root/wen_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select red_cargo_localization
source install/setup.bash
```

先按相机型号启动 Orbbec 驱动，并确保启用彩色点云。例如 Gemini 330 系列：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_color:=true enable_depth:=true enable_frame_sync:=true \
  enable_colored_point_cloud:=true
```

再启动定位节点：

```bash
ros2 launch red_cargo_localization red_cargo_localization.launch.py
```

若相机命名空间不同，可覆盖输入话题：

```bash
ros2 launch red_cargo_localization red_cargo_localization.launch.py \
  input_topic:=/my_camera/depth_registered/points
```

观察结果：

```bash
ros2 topic echo /red_cargo_localizer/centroid
ros2 topic echo /red_cargo_localizer/detected
rviz2 -d /root/wen_ws/install/red_cargo_localization/share/red_cargo_localization/rviz/red_cargo.rviz
```

## 参数调节

参数文件位于 `config/red_cargo.yaml`。建议先录制典型飞行高度、光照和货物姿态下的点云，再按以下顺序调节：

1. 用 `min_depth/max_depth` 和 `min_x/max_x/min_y/max_y` 限制吊挂装置下方的有效空间。
2. 调节两个红色色相区间以及 `min_saturation/min_value`，在 RViz 中确认红色货物被保留而地面被排除。
3. `cluster_tolerance` 应略大于货物表面相邻点的典型间距；过小会把货物切碎，过大会连接不同物体。
4. 提高 `min_cluster_points` 可排除小红色杂物。飞行高度增加、有效点变少时需要相应降低。
5. `voxel_size` 只用于建立聚类索引；最终质心仍由所选点簇的全部原始点计算。

`voxel_size` 必须小于或等于 `cluster_tolerance`，HSV 色相上下限必须在 0 到 360 度内。非法参数会使节点直接拒绝启动，以避免产生含义不明确的位置结果。

推荐使用三维 ROI 屏蔽无人机或吊挂结构中固定的红色部件。自动曝光和白平衡可能使 RGB 阈值随环境变化，实机参数应以目标场景采集的数据为准。
