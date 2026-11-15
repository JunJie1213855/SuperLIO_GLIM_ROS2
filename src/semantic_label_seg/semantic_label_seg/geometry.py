"""Geometry shared by library clients and ROS nodes; no ROS/OpenCV imports."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if (
            not np.isfinite([self.fx, self.fy, self.cx, self.cy]).all()
            or min(self.fx, self.fy) <= 0
        ):
            raise ValueError(
                "camera intrinsics must be finite with positive focal lengths"
            )


def rgbd_to_points(depth, intrinsics, depth_scale=0.001, min_depth=0.1, max_depth=20.0):
    """Back-project aligned, rectified depth into its camera optical frame.

    Integer depth is multiplied by depth_scale (typically mm -> m). Floating
    depth is already in metres and is never multiplied by depth_scale.
    Invalid depths remain in place as NaN; the output is organized (H,W,3).
    """
    depth = np.asarray(depth)
    if depth.ndim != 2 or depth.dtype.kind not in "uif":
        raise ValueError("depth must be a numeric HxW array")
    if not np.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError("depth_scale must be positive and finite")
    if not 0 <= min_depth < max_depth:
        raise ValueError("expected 0 <= min_depth < max_depth")
    z = depth.astype(np.float32, copy=True)
    if depth.dtype.kind in "ui":
        z *= depth_scale
    valid = np.isfinite(z) & (z > 0) & (z >= min_depth) & (z <= max_depth)
    v, u = np.indices(z.shape, dtype=np.float32)
    xyz = np.stack(
        (
            (u - intrinsics.cx) * z / intrinsics.fx,
            (v - intrinsics.cy) * z / intrinsics.fy,
            z,
        ),
        axis=-1,
    )
    xyz[~valid] = np.nan
    return xyz


def transform_points(points, translation, quaternion_xyzw):
    """Apply target_T_source, preserving point ordering and non-finite points."""
    points = np.asarray(points, dtype=np.float32)
    translation = np.asarray(translation, dtype=np.float64)
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or translation.shape != (3,)
        or q.shape != (4,)
    ):
        raise ValueError("expected Nx3 points, xyz translation and xyzw quaternion")
    if (
        not np.isfinite(q).all()
        or not np.isfinite(translation).all()
        or np.linalg.norm(q) < 1e-12
    ):
        raise ValueError("invalid rigid transform")
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return (points @ rotation.T + translation).astype(np.float32)
