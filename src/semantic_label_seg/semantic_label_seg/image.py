"""YOLO instance masks to semantic labels, adapted from SLIDE-SLAM detect.py."""

from pathlib import Path

import numpy as np

from .types import ImageLabels


def _numpy(value):
    return (
        value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)
    )


def labels_from_detections(detections, names, image_shape, class_map, threshold=0.4):
    """Resolve overlapping instance masks by confidence, not detector order."""
    height, width = image_shape[:2]
    if height <= 0 or width <= 0 or not 0 <= threshold <= 1:
        raise ValueError("invalid image shape or confidence threshold")
    labels = np.zeros((height, width), dtype=np.uint32)
    instances = np.zeros((height, width), dtype=np.uint32)
    confidence = np.zeros((height, width), dtype=np.float32)
    next_id = 1
    for result in detections:
        if result.masks is None or result.boxes is None:
            continue
        # Current Ultralytics exposes .data; older SLIDE-SLAM used .masks.
        masks = _numpy(
            getattr(result.masks, "data", getattr(result.masks, "masks", None))
        )
        classes = _numpy(result.boxes.cls).reshape(-1)
        scores = _numpy(result.boxes.conf).reshape(-1)
        if masks.ndim != 3 or len(masks) != len(classes) or len(classes) != len(scores):
            raise ValueError("inconsistent YOLO instance masks and boxes")
        for mask, class_id, score in zip(masks, classes, scores):
            if not np.isfinite(score) or score < threshold:
                continue
            name = names[int(class_id)]
            label = int(class_map.get(name, 0))
            if label == 0:
                continue
            if label < 0 or label > np.iinfo(np.uint32).max:
                raise ValueError("class_map values must fit uint32")
            if mask.shape != (height, width):
                # Only resizing is needed: avoid a mandatory OpenCV dependency.
                rows = np.minimum(
                    (np.arange(height) * mask.shape[0] / height).astype(int),
                    mask.shape[0] - 1,
                )
                cols = np.minimum(
                    (np.arange(width) * mask.shape[1] / width).astype(int),
                    mask.shape[1] - 1,
                )
                mask = mask[np.ix_(rows, cols)]
            active = (mask > 0.5) & (score > confidence)
            labels[active] = label
            instances[active] = next_id
            confidence[active] = score
            next_id += 1
    return ImageLabels(labels, instances, confidence)


class YoloSegmenter:
    def __init__(
        self, model_path, class_map=None, device="cpu", confidence_threshold=0.4
    ):
        model_path = Path(model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"YOLO segmentation checkpoint not found: {model_path}"
            )
        if not 0 <= confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be in [0,1]")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "YOLO inference needs the optional ultralytics package"
            ) from exc
        self._model = YOLO(str(model_path), task="segment")
        if self._model.task != "segment":
            raise ValueError(
                "YOLO checkpoint must be a segmentation model, not a box-only detector"
            )
        self.device = device
        self.threshold = confidence_threshold
        self.class_map = (
            {"chair": 1, "dining table": 2, "tv": 3}
            if class_map is None
            else dict(class_map)
        )

    def segment(self, bgr_image):
        image = np.asarray(bgr_image)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("expected an HxWx3 uint8 BGR image")
        detections = self._model.predict(
            image,
            device=self.device,
            conf=self.threshold,
            retina_masks=True,
            verbose=False,
        )
        return labels_from_detections(
            detections, self._model.names, image.shape, self.class_map, self.threshold
        )
