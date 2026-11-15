"""Label arrays have exactly the same ordering/shape as their input."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PointLabels:
    labels: np.ndarray  # uint32, (N,), category IDs
    confidence: np.ndarray  # float32, (N,), maximum model score, not calibrated


@dataclass(frozen=True)
class ImageLabels:
    labels: np.ndarray  # uint32, (H,W), category IDs; 0 is unknown/background
    instance_ids: np.ndarray  # uint32, (H,W), frame-local IDs; not tracking IDs
    confidence: np.ndarray  # float32, (H,W), detector score
