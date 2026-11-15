from types import SimpleNamespace

import numpy as np
import pytest
from semantic_label_seg.geometry import rgbd_to_points, transform_points
from semantic_label_seg.image import labels_from_detections
from semantic_label_seg.projection import project_scan

from semantic_label_seg import CameraIntrinsics, RangeNetSegmenter, YoloSegmenter


def test_projection_preserves_input_and_index_zero_and_uses_nearest_point():
    xyz = np.array(
        [
            [1, 0, 0],
            [2, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
            [np.nan, 0, 0],
            [100, 0, 0],
            [1, 0, 10],
        ],
        dtype=np.float32,
    )
    before = xyz.copy()
    intensity = np.arange(len(xyz), dtype=np.float32)
    projected = project_scan(xyz, intensity, 8, 16, 30, -30, 40)
    np.testing.assert_equal(xyz, before)
    np.testing.assert_array_equal(projected.point_indices, [0, 1, 2])
    assert projected.mask.sum() == 2
    u, v = projected.u[0], projected.v[0]
    assert projected.mask[v, u]
    assert projected.features[0, v, u] == 1
    assert projected.features[4, v, u] == 0


def test_empty_projection_and_validation():
    projected = project_scan(np.empty((0, 3)), np.empty(0), 4, 8, 20, -20)
    assert not projected.mask.any()
    with pytest.raises(ValueError):
        project_scan(np.ones((3, 3)), np.ones(2), 4, 8, 20, -20)


def test_depth_units_and_invalid_points():
    camera = CameraIntrinsics(2, 2, 0, 0)
    integer = np.array([[1000, 2000], [0, 1000]], dtype=np.uint16)
    floating = integer.astype(np.float32) / 1000
    a = rgbd_to_points(integer, camera)
    b = rgbd_to_points(floating, camera)
    np.testing.assert_allclose(a, b, equal_nan=True)
    np.testing.assert_allclose(a[0, 1], [1, 0, 2])
    assert np.isnan(a[1, 0]).all()
    assert integer[0, 0] == 1000
    with pytest.raises(ValueError):
        CameraIntrinsics(0, 1, 0, 0)


def test_transform_uses_xyzw_and_does_not_mutate_points():
    points = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    result = transform_points(points, [1, 2, 3], [0, 0, np.sqrt(0.5), np.sqrt(0.5)])
    np.testing.assert_allclose(result, [[1, 3, 3], [0, 2, 3]], atol=1e-6)
    np.testing.assert_array_equal(points, [[1, 0, 0], [0, 1, 0]])


def detection(masks, classes, confidence):
    return SimpleNamespace(
        masks=SimpleNamespace(data=np.asarray(masks)),
        boxes=SimpleNamespace(cls=np.asarray(classes), conf=np.asarray(confidence)),
    )


def test_image_overlap_uses_confidence_and_unknown_stays_zero():
    detections = [
        detection(
            [np.ones((2, 2)), np.ones((2, 2)), np.ones((2, 2))],
            [0, 1, 2],
            [0.9, 0.6, 0.99],
        )
    ]
    result = labels_from_detections(
        detections, {0: "chair", 1: "tv", 2: "other"}, (4, 4), {"chair": 1, "tv": 3}
    )
    assert (result.labels == 1).all()
    assert (result.instance_ids == 1).all()
    np.testing.assert_allclose(result.confidence, 0.9)
    empty = labels_from_detections([], {}, (4, 4), {})
    assert not empty.labels.any() and not empty.instance_ids.any()


def test_missing_weights_fail_instead_of_random_predictions(tmp_path):
    with pytest.raises(FileNotFoundError, match="arch_cfg.yaml"):
        RangeNetSegmenter(tmp_path)
    with pytest.raises(FileNotFoundError, match="checkpoint"):
        YoloSegmenter(tmp_path / "absent.pt")
