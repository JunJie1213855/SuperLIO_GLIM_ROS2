"""Actual CPU network/checkpoint tests using synthetic, non-trained weights."""

import numpy as np
import pytest
import yaml

torch = pytest.importorskip("torch")
from semantic_label_seg.lidar import RangeNetSegmenter
from semantic_label_seg.models.segmentator import Segmentator


@pytest.fixture(scope="module")
def checkpoint(tmp_path_factory):
    torch.set_num_threads(1)
    path = tmp_path_factory.mktemp("synthetic_rangenet")
    arch = {
        "backbone": {
            "name": "darknet",
            "input_depth": {"range": True, "xyz": True, "remission": True},
            "dropout": 0.0,
            "bn_d": 0.1,
            "OS": 8,
            "extra": {"layers": 13},
        },
        "decoder": {"dropout": 0.0, "bn_d": 0.1, "lightweight_decoder": True},
        "head": {"dropout": 0.0},
        "post": {"CRF": {"use": False}},
        "dataset": {
            "max_points": 1,
            "sensor": {
                "img_prop": {"height": 4, "width": 32},
                "fov_up": 30.0,
                "fov_down": -30.0,
                "img_means": [0.0] * 5,
                "img_stds": [1.0] * 5,
            },
        },
    }
    (path / "arch_cfg.yaml").write_text(yaml.safe_dump(arch))
    (path / "data_cfg.yaml").write_text(
        yaml.safe_dump(
            {"learning_map_inv": {0: 0, 1: 10}, "labels": {0: "unknown", 10: "chair"}}
        )
    )
    network = Segmentator(arch, 2)
    with torch.no_grad():
        for parameter in network.parameters():
            parameter.zero_()
        network.head[1].bias[1] = 4
    for filename, module in [
        ("backbone", network.backbone),
        ("segmentation_decoder", network.decoder),
        ("segmentation_head", network.head),
    ]:
        torch.save(module.state_dict(), path / filename)
    return path


def test_actual_network_unprojects_to_original_order_and_handles_invalid_points(
    checkpoint,
):
    engine = RangeNetSegmenter(
        checkpoint, device="cpu", max_range=10, label_space="original"
    )
    points = np.array(
        [[1, 0, 0], [0, 2, 0], [20, 0, 0], [np.nan, 0, 0], [0, 0, 0]], dtype=np.float32
    )
    result = engine.segment(points, np.arange(5, dtype=np.float32))
    np.testing.assert_array_equal(result.labels, [10, 10, 0, 0, 0])
    assert (result.confidence[:2] > 0.98).all()
    assert (result.confidence[2:] == 0).all()
    assert result.labels.dtype == np.uint32
    # The original max_points=1 is irrelevant: there is no fixed point padding.
    assert len(result.labels) == 5
    assert len(engine.segment(np.empty((0, 3))).labels) == 0


def test_label_remapping_and_confidence_rejection(checkpoint):
    engine = RangeNetSegmenter(checkpoint, device="cpu", label_map={1: 7})
    assert engine.segment([[1, 0, 0]]).labels[0] == 7
    engine.confidence_threshold = 0.999
    result = engine.segment([[1, 0, 0]])
    assert result.labels[0] == 0 and result.confidence[0] == 0


def test_incompatible_checkpoint_is_fatal(checkpoint, monkeypatch):
    # Preserve the fixture on disk; force a real state-dict mismatch at load time.
    monkeypatch.setattr(torch, "load", lambda *args, **kwargs: {})
    with pytest.raises(RuntimeError, match="cannot load RangeNet checkpoint"):
        RangeNetSegmenter(checkpoint, device="cpu")
