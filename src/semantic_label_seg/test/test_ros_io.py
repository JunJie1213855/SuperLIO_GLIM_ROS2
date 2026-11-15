import numpy as np
import pytest

pytest.importorskip("sensor_msgs")
from semantic_label_seg.ros_io import (
    array_to_image,
    cloud_to_arrays,
    cloud_view,
    image_to_array,
    with_labels,
    xyz_cloud,
)
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


def make_padded_cloud(big_endian=False):
    dtype = np.dtype(
        [
            ("x", ">f4" if big_endian else "<f4"),
            ("y", ">f4" if big_endian else "<f4"),
            ("z", ">f4" if big_endian else "<f4"),
            ("intensity", ">f4" if big_endian else "<f4"),
            ("ring", ">u2" if big_endian else "<u2"),
            ("padding", "V2"),
        ]
    )
    values = np.zeros((2, 2), dtype=dtype)
    values["x"] = [[1, 2], [3, np.nan]]
    values["y"] = 0
    values["z"] = 1
    values["intensity"] = [[10, 20], [30, 40]]
    values["ring"] = [[1, 2], [3, 4]]
    data = b"".join(row.tobytes() + b"PAD!" for row in values)
    header = Header(frame_id="lidar")
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    fields = [
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(["x", "y", "z", "intensity"])
    ]
    fields.append(
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1)
    )
    return PointCloud2(
        header=header,
        height=2,
        width=2,
        fields=fields,
        is_bigendian=big_endian,
        point_step=20,
        row_step=44,
        data=data,
        is_dense=False,
    )


@pytest.mark.parametrize("big_endian", [False, True])
def test_cloud_conversion_preserves_fields_order_stamp_and_endianness(big_endian):
    cloud = make_padded_cloud(big_endian)
    original_bytes = bytes(cloud.data)
    points, intensity = cloud_to_arrays(cloud)
    np.testing.assert_equal(points[:, 0], [1, 2, 3, np.nan])
    np.testing.assert_equal(intensity, [10, 20, 30, 40])
    output = with_labels(
        cloud,
        np.array([1, 2, 3, 0], dtype=np.uint32),
        np.array([0.9, 0.8, 0.7, 0]),
        colors=np.array(
            [[255, 0, 0], [0, 255, 0], [0, 0, 255], [96, 96, 96]], dtype=np.uint8
        ),
    )
    assert output.header == cloud.header
    assert output.width == 2 and output.height == 2
    assert bytes(cloud.data) == original_bytes
    result = cloud_view(output)
    np.testing.assert_equal(result["ring"], [[1, 2], [3, 4]])
    np.testing.assert_equal(result["label"], [[1, 2], [3, 0]])
    np.testing.assert_equal(result["intensity"], [[10, 20], [30, 40]])
    np.testing.assert_equal(result["x"], cloud_view(cloud)["x"])
    np.testing.assert_equal(result["rgb"], [[0xFF0000, 0x00FF00], [0x0000FF, 0x606060]])
    replaced = with_labels(output, np.zeros(4, dtype=np.uint32), np.zeros(4))
    assert sum(f.name == "label" for f in replaced.fields) == 1
    assert not cloud_view(replaced)["label"].any()


def test_ros_cloud_organized_rgbd_and_empty_cloud():
    points = np.array([[[1, 2, 3], [np.nan, np.nan, np.nan]]], dtype=np.float32)
    cloud = xyz_cloud(points, Header(frame_id="camera_optical"))
    result = with_labels(
        cloud,
        np.array([1, 0], dtype=np.uint32),
        np.array([0.7, 0]),
        np.array([4, 0], dtype=np.uint32),
    )
    assert not result.is_dense
    np.testing.assert_equal(cloud_view(result)["instance_id"], [[4, 0]])
    empty = xyz_cloud(np.empty((0, 3)), Header(frame_id="lidar"))
    labeled = with_labels(empty, np.empty(0, dtype=np.uint32), np.empty(0))
    assert labeled.width == 0 and len(labeled.data) == 0


def test_padded_big_endian_image_and_label_image_roundtrip():
    data = np.array([[1000, 2000], [3000, 4000]], dtype=">u2")
    msg = Image(
        height=2,
        width=2,
        encoding="16UC1",
        is_bigendian=True,
        step=6,
        data=data[0].tobytes() + b"xx" + data[1].tobytes() + b"xx",
    )
    np.testing.assert_equal(image_to_array(msg), [[1000, 2000], [3000, 4000]])
    labels = np.array([[0, 42]], dtype=np.uint32)
    image = array_to_image(labels, "32SC1", Header(frame_id="camera"))
    np.testing.assert_equal(image_to_array(image), labels)


def test_invalid_cloud_and_mismatched_labels_are_rejected():
    cloud = make_padded_cloud()
    with pytest.raises(ValueError, match="exactly"):
        with_labels(cloud, np.zeros(3, dtype=np.uint32), np.zeros(4))
    cloud.data = b""
    with pytest.raises(ValueError, match="truncated"):
        cloud_to_arrays(cloud)
