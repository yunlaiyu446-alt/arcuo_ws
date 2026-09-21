"""Pure helpers for mapping RealSense intrinsics to ROS CameraInfo fields."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class CameraCalibration:
    """ROS CameraInfo-compatible calibration values without ROS dependencies."""

    width: int
    height: int
    distortion_model: str
    d: Tuple[float, ...]
    k: Tuple[float, ...]
    r: Tuple[float, ...]
    p: Tuple[float, ...]


def calibration_from_values(
    width: int,
    height: int,
    fx: float,
    fy: float,
    ppx: float,
    ppy: float,
    coeffs: Iterable[float],
) -> CameraCalibration:
    """Convert RealSense color intrinsics into ROS CameraInfo matrix values."""

    if width <= 0 or height <= 0:
        raise ValueError('width and height must be positive')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('focal lengths must be positive')

    distortion = tuple(float(value) for value in coeffs)
    if len(distortion) < 5:
        raise ValueError('at least five distortion coefficients are required')

    fx = float(fx)
    fy = float(fy)
    ppx = float(ppx)
    ppy = float(ppy)

    return CameraCalibration(
        width=int(width),
        height=int(height),
        distortion_model='plumb_bob',
        d=distortion[:5],
        k=(fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0),
        r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p=(fx, 0.0, ppx, 0.0, 0.0, fy, ppy, 0.0, 0.0, 0.0, 1.0, 0.0),
    )
