"""ROS-independent SLIDE-SLAM RangeNet architecture and strict checkpoint loader.

Module names are preserved so existing split SLIDE-SLAM checkpoints load without
renaming keys. Backbone, decoder and CRF implementations retain upstream code.
"""

from importlib import import_module
from pathlib import Path
import torch
from torch import nn
from torch.nn import functional as F
from .crf import CRF


class Segmentator(nn.Module):
    def __init__(self, arch, nclasses):
        super().__init__()
        name = arch["backbone"]["name"]
        if name not in {"darknet", "basic_darknet"}:
            raise ValueError(f"unsupported SLIDE-SLAM backbone: {name}")
        backbone = import_module(f".backbone.{name}", __package__)
        decoder = import_module(f".decoder.{name}", __package__)
        self.backbone = backbone.Backbone(params=arch["backbone"])
        # Dimension discovery stays on CPU and must not update BatchNorm buffers.
        self.backbone.eval()
        sensor = arch["dataset"]["sensor"]
        with torch.no_grad():
            _, skips = self.backbone(
                torch.zeros(
                    1, 5, sensor["img_prop"]["height"], sensor["img_prop"]["width"]
                )
            )
        self.decoder = decoder.Decoder(
            params=arch["decoder"],
            stub_skips=skips,
            OS=arch["backbone"]["OS"],
            feature_depth=self.backbone.get_last_depth(),
        )
        self.head = nn.Sequential(
            nn.Dropout2d(p=arch["head"]["dropout"]),
            nn.Conv2d(
                self.decoder.get_last_depth(), nclasses, kernel_size=3, padding=1
            ),
        )
        crf_config = arch.get("post", {}).get("CRF", {})
        self.CRF = (
            CRF(crf_config["params"], nclasses)
            if crf_config.get("use", False)
            else None
        )

    def load_weights(self, directory):
        parts = [
            ("backbone", self.backbone),
            ("segmentation_decoder", self.decoder),
            ("segmentation_head", self.head),
        ]
        if self.CRF is not None:
            parts.append(("segmentation_CRF", self.CRF))
        for filename, module in parts:
            path = Path(directory) / filename
            try:
                state = torch.load(path, map_location="cpu", weights_only=True)
                module.load_state_dict(state, strict=True)
            except Exception as exc:
                raise RuntimeError(
                    f"cannot load RangeNet checkpoint {path}: {exc}"
                ) from exc

    def forward(self, inputs, mask):
        output, skips = self.backbone(inputs)
        output = self.head(self.decoder(output, skips))
        output = F.softmax(output, dim=1)
        return self.CRF(inputs, output, mask) if self.CRF is not None else output
