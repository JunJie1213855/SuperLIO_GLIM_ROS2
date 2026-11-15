"""RangeNet inference extracted from SLIDE-SLAM's scan2shape frontend."""

from pathlib import Path

import numpy as np
import yaml

from .projection import project_scan
from .types import PointLabels


class RangeNetSegmenter:
    def __init__(
        self,
        model_dir,
        device="auto",
        max_range=40.0,
        confidence_threshold=0.0,
        label_space="learning",
        label_map=None,
        unknown_label=0,
        cpu_threads=None,
    ):
        directory = Path(model_dir).expanduser().resolve()
        required = [
            "arch_cfg.yaml",
            "data_cfg.yaml",
            "backbone",
            "segmentation_decoder",
            "segmentation_head",
        ]
        missing = [name for name in required if not (directory / name).is_file()]
        if missing:
            raise FileNotFoundError(
                f"RangeNet model directory {directory}: missing {', '.join(missing)}"
            )
        with (directory / "arch_cfg.yaml").open() as file:
            self.arch = yaml.safe_load(file)
        with (directory / "data_cfg.yaml").open() as file:
            self.data = yaml.safe_load(file)
        crf = self.arch.get("post", {}).get("CRF", {})
        if crf.get("use", False) and not (directory / "segmentation_CRF").is_file():
            raise FileNotFoundError(
                f"missing CRF checkpoint: {directory / 'segmentation_CRF'}"
            )
        if not 0 <= confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be in [0,1]")
        if not np.isfinite(max_range) or max_range < 0:
            raise ValueError("max_range must be finite and nonnegative")
        if label_space not in {"learning", "original"}:
            raise ValueError("label_space must be learning or original")
        if not 0 <= unknown_label <= np.iinfo(np.uint32).max:
            raise ValueError("unknown_label must fit uint32")
        inverse = {int(k): int(v) for k, v in self.data["learning_map_inv"].items()}
        self.nclasses = len(inverse)
        if set(inverse) != set(range(self.nclasses)):
            raise ValueError(
                "learning_map_inv must have consecutive network IDs starting at 0"
            )
        if isinstance(label_map, (str, Path)):
            with Path(label_map).expanduser().open() as file:
                label_map = yaml.safe_load(file)
        label_map = {int(k): int(v) for k, v in (label_map or {}).items()}
        if any(v < 0 or v > np.iinfo(np.uint32).max for v in label_map.values()):
            raise ValueError("mapped labels must fit uint32")
        output_ids = [
            i if label_space == "learning" else inverse[i] for i in range(self.nclasses)
        ]
        self.output_ids = np.asarray(
            [label_map.get(i, i) for i in output_ids], dtype=np.uint32
        )
        raw_names = {int(k): str(v) for k, v in self.data.get("labels", {}).items()}
        self.label_names = {
            int(self.output_ids[i]): raw_names.get(inverse[i], str(inverse[i]))
            for i in range(self.nclasses)
        }
        self.max_range = float(max_range)
        self.confidence_threshold = float(confidence_threshold)
        self.unknown_label = int(unknown_label)
        self.sensor = self.arch["dataset"]["sensor"]
        self.means = np.asarray(self.sensor["img_means"], dtype=np.float32).reshape(
            -1, 1, 1
        )
        self.stds = np.asarray(self.sensor["img_stds"], dtype=np.float32).reshape(
            -1, 1, 1
        )
        if self.means.shape != (5, 1, 1) or self.stds.shape != (5, 1, 1):
            raise ValueError(
                "RangeNet expects five normalization channels: range,x,y,z,intensity"
            )
        if (
            not np.isfinite(self.means).all()
            or not np.isfinite(self.stds).all()
            or (self.stds <= 0).any()
        ):
            raise ValueError("invalid normalization statistics")
        try:
            import torch
        except ImportError as exc:
            raise ImportError(
                "RangeNet inference needs PyTorch >=2.0 in the current Python environment"
            ) from exc
        from .models.segmentator import Segmentator

        if cpu_threads is not None:
            if int(cpu_threads) < 1:
                raise ValueError("cpu_threads must be positive")
            torch.set_num_threads(int(cpu_threads))
        self.device = torch.device(
            ("cuda" if torch.cuda.is_available() else "cpu")
            if device == "auto"
            else device
        )
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable; choose device=cpu")
        self._torch = torch
        self._model = Segmentator(self.arch, self.nclasses)
        self._model.load_weights(directory)
        self._model.to(self.device).eval()

    def segment(self, points, intensity=None):
        """Label Nx3 points in the trained LiDAR frame; preserve N and ordering."""
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N,3)")
        if intensity is None:
            intensity = np.zeros(len(points), dtype=np.float32)
        projection = project_scan(
            points,
            intensity,
            self.sensor["img_prop"]["height"],
            self.sensor["img_prop"]["width"],
            self.sensor["fov_up"],
            self.sensor["fov_down"],
            self.max_range,
        )
        labels = np.full(len(points), self.unknown_label, dtype=np.uint32)
        confidence = np.zeros(len(points), dtype=np.float32)
        if len(projection.point_indices) == 0:
            return PointLabels(labels, confidence)
        inputs = ((projection.features - self.means) / self.stds) * projection.mask[
            None
        ]
        torch = self._torch
        with torch.inference_mode():
            output = self._model(
                torch.from_numpy(inputs[None]).to(self.device),
                torch.from_numpy(projection.mask[None]).to(self.device),
            )
            if tuple(output.shape) != (1, self.nclasses, *projection.mask.shape):
                raise RuntimeError(
                    f"unexpected RangeNet output shape: {tuple(output.shape)}"
                )
            scores = output[0, :, projection.v, projection.u]
            point_confidence, point_classes = scores.max(dim=0)
            point_classes = point_classes.cpu().numpy()
            point_confidence = point_confidence.cpu().numpy().astype(np.float32)
        if not np.isfinite(point_confidence).all():
            raise RuntimeError("RangeNet returned non-finite scores")
        accepted = point_confidence >= self.confidence_threshold
        ids = projection.point_indices[accepted]
        labels[ids] = self.output_ids[point_classes[accepted]]
        confidence[ids] = point_confidence[accepted]
        return PointLabels(labels, confidence)
