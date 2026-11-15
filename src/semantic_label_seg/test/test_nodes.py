"""ROS transport tests use explicit stubs, never a production random model."""

import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from semantic_label_seg.ros_io import (
    array_to_image,
    cloud_to_arrays,
    cloud_view,
    xyz_cloud,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

from semantic_label_seg import ImageLabels, PointLabels, lidar_node, rgbd_node


@pytest.mark.parametrize("world_input", [False, True])
def test_lidar_ros_transport_and_preserved_header(monkeypatch, world_input):
    model_inputs = []

    def segment(xyz, intensity):
        model_inputs.append(xyz.copy())
        return PointLabels(
            np.ones(len(xyz), dtype=np.uint32),
            np.full(len(xyz), 0.9, dtype=np.float32),
        )

    def create_stub(*args, **kwargs):
        return SimpleNamespace(
            device="test-only",
            label_names={1: "test"},
            segment=segment,
        )

    monkeypatch.setattr(lidar_node, "RangeNetSegmenter", create_stub)
    context = Context()
    rclpy.init(context=context)
    name = "/test_semantic_label_seg_" + uuid4().hex
    node = lidar_node.LidarSegNode(
        context=context,
        parameter_overrides=[
            Parameter("input_topic", value=name + "/input"),
            Parameter("output_topic", value=name + "/output"),
            Parameter("inference_frame", value="lidar"),
            Parameter("desired_frequency", value=0.0),
        ],
    )
    client = rclpy.create_node("semantic_label_seg_test_client", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(client)
    received = []
    subscription = client.create_subscription(
        PointCloud2, name + "/output", received.append, qos_profile_sensor_data
    )
    publisher = client.create_publisher(
        PointCloud2, name + "/input", qos_profile_sensor_data
    )
    header = Header(frame_id="world" if world_input else "lidar")
    header.stamp.sec = 42
    header.stamp.nanosec = 123
    points = [[1, 1, 0], [1, 2, 0]] if world_input else [[1, 0, 0], [2, 0, 0]]
    cloud = xyz_cloud(np.array(points, dtype=np.float32), header)
    try:
        deadline = time.monotonic() + 5
        tf_time = time.monotonic() + 0.2
        tf_sent = False
        while not received and time.monotonic() < deadline:
            if world_input and not tf_sent and time.monotonic() >= tf_time:
                assert not received, "world cloud must wait for the timestamped TF"
                tf = TransformStamped(header=header, child_frame_id="lidar")
                tf.transform.translation.x = 1.0
                tf.transform.rotation.z = np.sqrt(0.5)
                tf.transform.rotation.w = np.sqrt(0.5)
                node.tf_buffer.set_transform(tf, "test")
                tf_sent = True
            publisher.publish(cloud)
            executor.spin_once(timeout_sec=0.05)
        assert received, "ROS publisher/subscriber did not deliver labeled cloud"
        assert received[0].header == cloud.header
        np.testing.assert_equal(
            cloud_to_arrays(received[0])[0], cloud_to_arrays(cloud)[0]
        )
        assert (cloud_view(received[0])["label"] == 1).all()
        assert "rgb" in cloud_view(received[0]).dtype.names
        np.testing.assert_allclose(model_inputs[-1], [[1, 0, 0], [2, 0, 0]], atol=1e-6)
    finally:
        client.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        client.destroy_node()
        context.shutdown()


def test_rgbd_node_respects_depth_units_invalid_pixels_and_optical_frame(monkeypatch):
    monkeypatch.setattr(
        rgbd_node,
        "YoloSegmenter",
        lambda *args, **kwargs: SimpleNamespace(
            segment=lambda image: ImageLabels(
                np.ones((1, 2), dtype=np.uint32),
                np.ones((1, 2), dtype=np.uint32),
                np.full((1, 2), 0.8, dtype=np.float32),
            )
        ),
    )
    context = Context()
    rclpy.init(context=context)
    node = rgbd_node.RgbdSegNode(context=context)
    header = Header(frame_id="camera_optical")
    header.stamp.sec = 42
    info = CameraInfo(
        header=header,
        height=1,
        width=2,
        k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )
    node.receive_info(info)
    color = Image(
        header=header, height=1, width=2, encoding="bgr8", step=6, data=bytes(6)
    )
    depth = array_to_image(np.array([[1000, 0]], dtype=np.uint16), "16UC1", header)
    clouds = []
    images = []
    node.cloud_pub = SimpleNamespace(publish=clouds.append)
    node.label_pub = SimpleNamespace(publish=images.append)
    node.instance_pub = SimpleNamespace(publish=images.append)
    node.confidence_pub = SimpleNamespace(publish=images.append)
    try:
        node.process_rgbd(color, depth)
        assert len(clouds) == 1 and len(images) == 3
        result = cloud_view(clouds[0])
        np.testing.assert_equal(result["label"], [[1, 0]])
        np.testing.assert_allclose(result["z"], [[1, np.nan]], equal_nan=True)
        assert clouds[0].header == depth.header
        assert clouds[0].header.frame_id == "camera_optical"
    finally:
        node.destroy_node()
        context.shutdown()
