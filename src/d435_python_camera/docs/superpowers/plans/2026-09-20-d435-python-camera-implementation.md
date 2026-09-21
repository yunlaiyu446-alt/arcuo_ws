# D435 Python Camera Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone ROS 2 Humble `ament_python` package that reads one D435/D435i color stream with `pyrealsense2` and publishes the exact image and calibration topics consumed by the existing ArUco detector.

**Architecture:** Keep hardware access in one `rclpy` node and keep camera-calibration math in a ROS-independent helper module. The node opens only the BGR8 color stream, derives calibration from the active RealSense profile, and publishes synchronized `Image` and `CameraInfo` messages with sensor-data QoS.

**Tech Stack:** Python 3.10, ROS 2 Humble, `rclpy`, `sensor_msgs`, NumPy, `pyrealsense2`, `ament_python`, `pytest`.

## Global Constraints

- Package root is `/Users/yuyunlai/Desktop/d435_python_camera` and the whole directory must be copyable into `/home/lucky/arcuo_ws_1/arcuo_ws/src/`.
- Support one D435/D435i and RGB only; do not enable depth, infrared, point cloud, or IMU.
- Publish `/camera/color/image_raw` and `/camera/color/camera_info` with `frame_id=camera_color_optical_frame` by default.
- Default stream is BGR8 at `640x480@30Hz`.
- Do not depend on Python `cv_bridge`.
- Stop the RealSense pipeline during every controlled shutdown path.
- Do not modify `aruco_detector`, `aruco_interfaces`, or `cargo_locator`.
- Do not install dependencies globally or into Conda `base`; Ubuntu validation uses the project environment after verifying Python 3.10 compatibility with ROS 2 Humble.

---

### Task 1: Package Metadata and Standalone Repository

**Files:**
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/package.xml`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/setup.py`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/setup.cfg`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/resource/d435_python_camera`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/d435_python_camera/__init__.py`

**Interfaces:**
- Produces: ROS package `d435_python_camera` and console executable `d435_python_camera_node` mapped to `d435_python_camera.camera_node:main`.

- [ ] **Step 1: Initialize version control and save the approved design**

Run:

```bash
cd /Users/yuyunlai/Desktop/d435_python_camera
git init
git add PROJECT_CONTEXT.md docs/superpowers/specs/2026-09-20-d435-python-camera-design.md docs/superpowers/plans/2026-09-20-d435-python-camera-implementation.md
git commit -m "docs: define single D435 Python camera package"
```

Expected: a root commit containing only the design, plan, and project context.

- [ ] **Step 2: Write the package metadata**

Create `package.xml` with package format 3, build type `ament_python`, and runtime dependencies `rclpy`, `sensor_msgs`, `launch`, `launch_ros`, and `python3-numpy`. Document `pyrealsense2` as an external runtime dependency in the description because its installation source varies by Ubuntu/librealsense setup.

Create `setup.py` with these package-data entries:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/d435_python_camera']),
    ('share/d435_python_camera', ['package.xml']),
    (os.path.join('share', 'd435_python_camera', 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', 'd435_python_camera', 'config'), glob('config/*.yaml')),
]
```

and this console script:

```python
'd435_python_camera_node = d435_python_camera.camera_node:main'
```

Create `setup.cfg` so scripts install under `$base/lib/d435_python_camera`. Create an empty resource marker and an `__init__.py` containing only the package docstring.

- [ ] **Step 3: Verify metadata syntax**

Run:

```bash
cd /Users/yuyunlai/Desktop/d435_python_camera
python3 -m compileall setup.py d435_python_camera
python3 setup.py --name
```

Expected: compile succeeds and the name printed is `d435_python_camera`.

- [ ] **Step 4: Commit the scaffold**

```bash
git add package.xml setup.py setup.cfg resource d435_python_camera/__init__.py
git commit -m "build: scaffold D435 Python camera package"
```

### Task 2: Calibration Mapping with Unit Tests

**Files:**
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/test/test_camera_info.py`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/d435_python_camera/camera_info.py`

**Interfaces:**
- Produces: immutable `CameraCalibration` and `calibration_from_values(width, height, fx, fy, ppx, ppy, coeffs)`.
- Consumes: numeric values copied from a `pyrealsense2.intrinsics` object by the ROS node.

- [ ] **Step 1: Write the failing calibration test**

```python
from d435_python_camera.camera_info import calibration_from_values


def test_calibration_from_values_maps_realsense_intrinsics():
    calibration = calibration_from_values(
        width=640,
        height=480,
        fx=615.0,
        fy=616.0,
        ppx=319.5,
        ppy=239.5,
        coeffs=[0.1, -0.2, 0.003, 0.004, 0.05],
    )

    assert calibration.width == 640
    assert calibration.height == 480
    assert calibration.distortion_model == 'plumb_bob'
    assert calibration.d == (0.1, -0.2, 0.003, 0.004, 0.05)
    assert calibration.k == (615.0, 0.0, 319.5, 0.0, 616.0, 239.5, 0.0, 0.0, 1.0)
    assert calibration.r == (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    assert calibration.p == (
        615.0, 0.0, 319.5, 0.0,
        0.0, 616.0, 239.5, 0.0,
        0.0, 0.0, 1.0, 0.0,
    )


def test_calibration_rejects_invalid_dimensions():
    try:
        calibration_from_values(0, 480, 615.0, 616.0, 319.5, 239.5, [0.0] * 5)
    except ValueError as exc:
        assert 'width and height must be positive' in str(exc)
    else:
        raise AssertionError('expected ValueError')
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cd /Users/yuyunlai/Desktop/d435_python_camera
python3 -m pytest test/test_camera_info.py -v
```

Expected: FAIL because `d435_python_camera.camera_info` does not exist.

- [ ] **Step 3: Implement the pure calibration helper**

Implement a frozen dataclass whose collection fields are tuples. Validate positive dimensions and focal lengths, require at least five distortion coefficients, and return:

```python
k = (fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0)
r = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
p = (fx, 0.0, ppx, 0.0, 0.0, fy, ppy, 0.0, 0.0, 0.0, 1.0, 0.0)
```

Use `distortion_model='plumb_bob'` and the first five coefficients for `d`.

- [ ] **Step 4: Run the test to verify it passes**

Run:

```bash
python3 -m pytest test/test_camera_info.py -v
```

Expected: 2 tests pass.

- [ ] **Step 5: Commit the helper**

```bash
git add d435_python_camera/camera_info.py test/test_camera_info.py
git commit -m "feat: map RealSense color intrinsics to CameraInfo"
```

### Task 3: RealSense ROS 2 Publisher Node

**Files:**
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/test/test_camera_node_contract.py`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/d435_python_camera/camera_node.py`

**Interfaces:**
- Consumes: `calibration_from_values(...)` from Task 2 and one `pyrealsense2` color stream.
- Produces: `D435PythonCameraNode.capture_and_publish()` and `main(args=None)`; publishes `sensor_msgs.msg.Image` and `sensor_msgs.msg.CameraInfo`.

- [ ] **Step 1: Write the failing source-contract test**

Because macOS does not provide the target ROS 2 Humble or D435 runtime, use an AST test to verify the portable source contract without importing ROS modules:

```python
import ast
from pathlib import Path


def test_camera_node_defines_required_entry_points():
    source_path = Path(__file__).parents[1] / 'd435_python_camera' / 'camera_node.py'
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    class_names = {node.name for node in tree.body if isinstance(node, ast.ClassDef)}
    function_names = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert 'D435PythonCameraNode' in class_names
    assert 'main' in function_names


def test_camera_node_has_no_cv_bridge_dependency():
    source_path = Path(__file__).parents[1] / 'd435_python_camera' / 'camera_node.py'
    source = source_path.read_text(encoding='utf-8')
    assert 'cv_bridge' not in source
    assert 'rs.stream.color' in source
    assert 'rs.format.bgr8' in source
    assert 'pipeline.stop()' in source
```

- [ ] **Step 2: Run the contract test to verify it fails**

Run:

```bash
python3 -m pytest test/test_camera_node_contract.py -v
```

Expected: FAIL because `camera_node.py` does not exist.

- [ ] **Step 3: Implement the camera node**

The implementation must:

- declare and read `serial_no`, `width`, `height`, `fps`, `frame_id`, `image_topic`, `camera_info_topic`, and `frame_timeout_ms`;
- create both publishers with `qos_profile_sensor_data`;
- configure only `rs.stream.color` with `rs.format.bgr8`;
- optionally call `config.enable_device(serial_no)`;
- start the pipeline and read color intrinsics from the active profile;
- convert frame data with `numpy.asanyarray`;
- construct `Image` directly with `height`, `width`, `encoding='bgr8'`, `is_bigendian=0`, `step=width*3`, and `data=image.tobytes()`;
- construct `CameraInfo` from the pure calibration helper;
- stamp both messages with the same `node.get_clock().now().to_msg()` value;
- publish exactly one image/info pair for each valid color frame;
- warn at most once every five seconds on frame timeout;
- make `shutdown_camera()` idempotent and call `pipeline.stop()` only after a successful start;
- run a `while rclpy.ok()` capture loop in `main`, and use `finally` to stop the pipeline, destroy the node, and shut down rclpy.

Use the official librealsense Python sequence `pipeline -> config -> enable_stream -> pipeline.start -> wait_for_frames -> get_color_frame`, consistent with the upstream examples.

- [ ] **Step 4: Run portable tests and syntax checks**

Run:

```bash
python3 -m pytest test/test_camera_info.py test/test_camera_node_contract.py -v
python3 -m compileall d435_python_camera
```

Expected: all tests pass and compileall succeeds without importing ROS or RealSense.

- [ ] **Step 5: Commit the node**

```bash
git add d435_python_camera/camera_node.py test/test_camera_node_contract.py
git commit -m "feat: publish D435 color frames through rclpy"
```

### Task 4: Launch, Configuration, and Operator Documentation

**Files:**
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/config/camera.yaml`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/launch/single_d435.launch.py`
- Create: `/Users/yuyunlai/Desktop/d435_python_camera/README.md`
- Modify: `/Users/yuyunlai/Desktop/d435_python_camera/PROJECT_CONTEXT.md`

**Interfaces:**
- Produces: `ros2 launch d435_python_camera single_d435.launch.py` and documented copy/build/run/verification workflow.

- [ ] **Step 1: Create the parameter file**

Use this exact parameter contract:

```yaml
d435_python_camera_node:
  ros__parameters:
    serial_no: ""
    width: 640
    height: 480
    fps: 30
    frame_id: "camera_color_optical_frame"
    image_topic: "/camera/color/image_raw"
    camera_info_topic: "/camera/color/camera_info"
    frame_timeout_ms: 1000
```

- [ ] **Step 2: Create the launch file**

Use `get_package_share_directory('d435_python_camera')` to load `config/camera.yaml`, then create one `launch_ros.actions.Node` with package `d435_python_camera`, executable `d435_python_camera_node`, name `d435_python_camera_node`, output `screen`, and the YAML parameter file.

- [ ] **Step 3: Write the README**

Document:

1. Stop every `realsense2_camera` process before starting the Python node.
2. Copy the entire package to `/home/lucky/arcuo_ws_1/arcuo_ws/src/d435_python_camera`.
3. Activate the project environment rather than Conda `base`, source `/opt/ros/humble/setup.bash`, and verify that the same Python can import `rclpy`, `sensor_msgs`, `numpy`, and `pyrealsense2`.
4. Build with `colcon build --symlink-install --packages-select d435_python_camera`.
5. Source `install/local_setup.bash` and launch the node.
6. Verify `/camera/color/image_raw`, `/camera/color/camera_info`, and then the existing ArUco debug topic.
7. Explain that `aruco_interfaces/msg/MarkerArray is invalid` is a separate interface-build problem.

- [ ] **Step 4: Update project context**

Record the implemented files, entry point, default topics, Ubuntu destination, test commands, and remaining hardware verification.

- [ ] **Step 5: Verify package contents**

Run:

```bash
python3 -m compileall setup.py d435_python_camera launch
python3 -m pytest test -v
git status --short
```

Expected: compile succeeds, every portable test passes, and only intended documentation/configuration files remain uncommitted.

- [ ] **Step 6: Commit documentation and launch files**

```bash
git add README.md PROJECT_CONTEXT.md config launch
git commit -m "docs: add launch and Ubuntu deployment workflow"
```

### Task 5: Final Portable Verification and Handoff

**Files:**
- Verify all files under `/Users/yuyunlai/Desktop/d435_python_camera`

**Interfaces:**
- Produces: a clean standalone package ready to copy to Ubuntu.

- [ ] **Step 1: Run the complete portable test suite**

```bash
cd /Users/yuyunlai/Desktop/d435_python_camera
python3 -m pytest test -v
python3 -m compileall setup.py d435_python_camera launch
git status --short
```

Expected: all tests pass, all Python files compile, and the worktree is clean.

- [ ] **Step 2: Inspect the transfer payload**

```bash
find /Users/yuyunlai/Desktop/d435_python_camera -path '*/.git' -prune -o -type f -print | sort
```

Expected: package metadata, Python module, resource marker, config, launch file, tests, README, design, plan, and project context are present; no build/install/log artifacts are included.

- [ ] **Step 3: Record Ubuntu-only verification**

Do not claim hardware success on macOS. After copying to Ubuntu, run the README commands to verify Python imports, colcon build, executable discovery, D435 image rate, CameraInfo values, and compatibility with `aruco_detector_node`.
