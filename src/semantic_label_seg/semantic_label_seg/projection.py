"""Spherical projection derived from SLIDE-SLAM/PRBonn LaserScan.

Unlike the original inference wrapper, never moves or drops input points, and
keeps the inverse projection for all valid points (including input index zero).
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RangeProjection:
    features: np.ndarray  # (5,H,W): range,x,y,z,remission
    mask: np.ndarray  # (H,W), occupied pixels
    point_indices: np.ndarray  # original indices of valid points
    u: np.ndarray
    v: np.ndarray


def project_scan(points, intensity, height, width, fov_up, fov_down, max_range=0.0):
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if height <= 0 or width <= 0 or not -90 <= fov_down < fov_up <= 90:
        raise ValueError("invalid range image size or vertical field of view")
    if not np.isfinite(max_range) or max_range < 0:
        raise ValueError("max_range must be finite and nonnegative; 0 disables it")
    intensity = np.asarray(intensity, dtype=np.float32)
    if intensity.shape != (len(points),):
        raise ValueError("intensity must have shape (N,)")

    depth = np.linalg.norm(points, axis=1)
    valid = np.isfinite(points).all(axis=1) & np.isfinite(depth) & (depth > 1e-6)
    if max_range > 0:
        valid &= depth <= max_range
    indices = np.flatnonzero(valid)
    pitch = np.arcsin(np.clip(points[indices, 2] / depth[indices], -1, 1))
    lower, upper = np.deg2rad([fov_down, fov_up])
    in_fov = (pitch >= lower - 1e-7) & (pitch <= upper + 1e-7)
    indices, pitch = indices[in_fov], pitch[in_fov]
    xyz = points[indices]
    yaw = -np.arctan2(xyz[:, 1], xyz[:, 0])
    u = np.clip(np.floor(0.5 * (yaw / np.pi + 1) * width), 0, width - 1).astype(
        np.int64
    )
    v = np.clip(
        np.floor((upper - pitch) / (upper - lower) * height), 0, height - 1
    ).astype(np.int64)

    features = np.full((5, height, width), -1, dtype=np.float32)
    mask = np.zeros((height, width), dtype=bool)
    # Keep the nearest observation per pixel, including a deterministic tie rule.
    order = np.argsort(depth[indices], kind="stable")
    _, first = np.unique((v * width + u)[order], return_index=True)
    chosen = order[first]
    point_ids = indices[chosen]
    vv, uu = v[chosen], u[chosen]
    features[0, vv, uu] = depth[point_ids]
    features[1:4, vv, uu] = points[point_ids].T
    features[4, vv, uu] = np.nan_to_num(
        intensity[point_ids], nan=0.0, posinf=0.0, neginf=0.0
    )
    mask[vv, uu] = True
    return RangeProjection(features, mask, indices, u, v)
