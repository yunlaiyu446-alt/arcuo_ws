"""ROS 2 node that publishes one RealSense color stream through pyrealsense2."""

from __future__ import annotations

import sys
import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from d435_python_camera.camera_info import (
    CameraCalibration,
    calibration_from_values,
)


class D435PythonCameraNode(Node):
    """Read a D435/D435i color stream and publish ROS 2 sensor messages."""

    def __init__(self) -> None:
        super().__init__('d435_python_camera_node')

        self.declare_parameter('serial_no', '')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('frame_timeout_ms', 1000)

        self._serial_no = self.get_parameter('serial_no').value.strip()
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = int(self.get_parameter('fps').value)
        self._frame_id = self.get_parameter('frame_id').value.strip()
        self._image_topic = self.get_parameter('image_topic').value.strip()
        self._camera_info_topic = (
            self.get_parameter('camera_info_topic').value.strip()
        )
        self._frame_timeout_ms = int(
            self.get_parameter('frame_timeout_ms').value
        )
        self._validate_parameters()

        self._pipeline = rs.pipeline()
        self._pipeline_started = False
        self._active_serial_no = ''
        self._calibration: Optional[CameraCalibration] = None
        self._last_timeout_warning = 0.0

        self._image_publisher = self.create_publisher(
            Image,
            self._image_topic,
            qos_profile_sensor_data,
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            self._camera_info_topic,
            qos_profile_sensor_data,
        )

        try:
            self._start_camera()
        except Exception:
            self.shutdown_camera()
            raise

    def _validate_parameters(self) -> None:
        if self._width <= 0 or self._height <= 0:
            raise ValueError('width and height must be positive')
        if self._fps <= 0:
            raise ValueError('fps must be positive')
        if self._frame_timeout_ms <= 0:
            raise ValueError('frame_timeout_ms must be positive')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')
        if not self._image_topic or not self._camera_info_topic:
            raise ValueError('image topics must not be empty')

    def _start_camera(self) -> None:
        config = rs.config()
        if self._serial_no:
            config.enable_device(self._serial_no)
        config.enable_stream(
            rs.stream.color,
            self._width,
            self._height,
            rs.format.bgr8,
            self._fps,
        )

        profile = self._pipeline.start(config)
        self._pipeline_started = True

        device = profile.get_device()
        self._active_serial_no = device.get_info(rs.camera_info.serial_number)
        color_profile = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        self._calibration = calibration_from_values(
            width=intrinsics.width,
            height=intrinsics.height,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            ppx=intrinsics.ppx,
            ppy=intrinsics.ppy,
            coeffs=intrinsics.coeffs,
        )

        self.get_logger().info(
            'Started RealSense color stream: '
            f'serial={self._active_serial_no}, '
            f'{intrinsics.width}x{intrinsics.height}@{self._fps}Hz, '
            f'image_topic={self._image_topic}, '
            f'camera_info_topic={self._camera_info_topic}'
        )

    def _device_is_connected(self) -> bool:
        context = rs.context()
        for device in context.query_devices():
            try:
                serial_no = device.get_info(rs.camera_info.serial_number)
            except RuntimeError:
                continue
            if serial_no == self._active_serial_no:
                return True
        return False

    def _warn_frame_timeout(self, error: RuntimeError) -> None:
        now = time.monotonic()
        if now - self._last_timeout_warning >= 5.0:
            self.get_logger().warning(
                f'Waiting for a color frame timed out: {error}'
            )
            self._last_timeout_warning = now

    def _new_camera_info_message(self, stamp) -> CameraInfo:
        if self._calibration is None:
            raise RuntimeError('camera calibration is unavailable')

        calibration = self._calibration
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.width = calibration.width
        message.height = calibration.height
        message.distortion_model = calibration.distortion_model
        message.d = list(calibration.d)
        message.k = list(calibration.k)
        message.r = list(calibration.r)
        message.p = list(calibration.p)
        return message

    def _new_image_message(self, stamp, image: np.ndarray) -> Image:
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(
                f'expected a BGR image with shape HxWx3, got {image.shape}'
            )

        height, width, _ = image.shape
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.height = int(height)
        message.width = int(width)
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = int(width * 3)
        message.data = image.tobytes(order='C')
        return message

    def capture_and_publish(self) -> bool:
        """Wait for one valid color frame and publish its ROS message pair."""

        try:
            frames = self._pipeline.wait_for_frames(self._frame_timeout_ms)
        except RuntimeError as error:
            if not self._device_is_connected():
                raise RuntimeError(
                    f'RealSense device {self._active_serial_no} disconnected'
                ) from error
            self._warn_frame_timeout(error)
            return False

        color_frame = frames.get_color_frame()
        if not color_frame:
            self._warn_frame_timeout(RuntimeError('frameset has no color frame'))
            return False

        image = np.asanyarray(color_frame.get_data())
        stamp = self.get_clock().now().to_msg()
        camera_info_message = self._new_camera_info_message(stamp)
        image_message = self._new_image_message(stamp, image)

        self._camera_info_publisher.publish(camera_info_message)
        self._image_publisher.publish(image_message)
        return True

    def shutdown_camera(self) -> None:
        """Stop the pipeline once so another process can open the D435."""

        if not self._pipeline_started:
            return
        try:
            self._pipeline.stop()
        except Exception as error:  # Best-effort cleanup during shutdown.
            self.get_logger().warning(
                f'Failed to stop the RealSense pipeline cleanly: {error}'
            )
        finally:
            self._pipeline_started = False


def main(args=None) -> None:
    """Run the camera node until ROS requests shutdown or capture fails."""

    rclpy.init(args=args)
    node: Optional[D435PythonCameraNode] = None
    exit_code = 0

    try:
        node = D435PythonCameraNode()
        while rclpy.ok():
            node.capture_and_publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        if node is not None:
            node.get_logger().fatal(f'Camera node stopped: {error}')
        else:
            print(f'Failed to start camera node: {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.shutdown_camera()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
