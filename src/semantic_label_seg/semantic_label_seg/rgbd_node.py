"""ROS 2 interface for aligned RGB-D YOLO labels and camera-frame points."""

from pathlib import Path

import numpy as np
import rclpy
import yaml
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from .geometry import CameraIntrinsics, rgbd_to_points
from .image import YoloSegmenter
from .ros_io import array_to_image, image_to_array, with_labels, xyz_cloud


class RgbdSegNode(Node):
    def __init__(self, **kwargs):
        super().__init__("rgbd_seg", **kwargs)
        defaults = {
            "model_path": "",
            "class_map_file": "",
            "device": "cpu",
            "confidence_threshold": 0.4,
            "desired_frequency": 2.0,
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "label_topic": "/semantic_label_seg/image_labels",
            "instance_topic": "/semantic_label_seg/image_instances",
            "confidence_topic": "/semantic_label_seg/image_confidence",
            "cloud_topic": "/semantic_label_seg/rgbd_points",
            "sync_slop": 0.05,
            "depth_scale": 0.001,
            "min_depth": 0.1,
            "max_depth": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda key: self.get_parameter(key).value
        class_map = None
        if get("class_map_file"):
            with Path(get("class_map_file")).expanduser().open() as file:
                class_map = yaml.safe_load(file)
        self.segmenter = YoloSegmenter(
            get("model_path"), class_map, get("device"), get("confidence_threshold")
        )
        self.frequency = float(get("desired_frequency"))
        if self.frequency < 0 or get("sync_slop") < 0:
            raise ValueError("desired_frequency and sync_slop must be nonnegative")
        self.last_stamp = None
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo,
            get("camera_info_topic"),
            self.receive_info,
            qos_profile_sensor_data,
        )
        self.label_pub = self.create_publisher(
            Image, get("label_topic"), qos_profile_sensor_data
        )
        self.instance_pub = self.create_publisher(
            Image, get("instance_topic"), qos_profile_sensor_data
        )
        self.confidence_pub = self.create_publisher(
            Image, get("confidence_topic"), qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, get("cloud_topic"), qos_profile_sensor_data
        )
        self.rgb_sub = Subscriber(
            self, Image, get("rgb_topic"), qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, get("depth_topic"), qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 5, get("sync_slop")
        )
        self.sync.registerCallback(self.process_rgbd)

    def receive_info(self, message):
        self.camera_info = message

    def process_rgbd(self, rgb, depth):
        stamp = rgb.header.stamp.sec * 1_000_000_000 + rgb.header.stamp.nanosec
        if (
            self.frequency > 0
            and self.last_stamp is not None
            and stamp >= self.last_stamp
            and stamp - self.last_stamp < 1e9 / self.frequency
        ):
            return
        if self.camera_info is None:
            self.get_logger().warning(
                "waiting for camera_info", throttle_duration_sec=5.0
            )
            return
        try:
            info = self.camera_info
            if not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id:
                raise ValueError(
                    "RGB and aligned depth must use the same camera optical frame"
                )
            if info.header.frame_id != rgb.header.frame_id:
                raise ValueError("CameraInfo frame differs from RGB optical frame")
            if rgb.encoding not in {"bgr8", "rgb8"} or depth.encoding not in {
                "16UC1",
                "32FC1",
            }:
                raise ValueError(
                    "expected bgr8/rgb8 color and 16UC1/32FC1 aligned depth"
                )
            color, distance = image_to_array(rgb), image_to_array(depth)
            if (
                color.shape[:2] != distance.shape
                or (info.height, info.width) != distance.shape
            ):
                raise ValueError(
                    "aligned RGB, depth and CameraInfo dimensions must agree"
                )
            if rgb.encoding == "rgb8":
                color = color[:, :, ::-1].copy()
            # The input must already be rectified. P is the rectified camera model.
            intrinsics = (
                CameraIntrinsics(info.p[0], info.p[5], info.p[2], info.p[6])
                if info.p[0] > 0 and info.p[5] > 0
                else CameraIntrinsics(info.k[0], info.k[4], info.k[2], info.k[5])
            )
            result = self.segmenter.segment(color)
            points = rgbd_to_points(
                distance,
                intrinsics,
                self.get_parameter("depth_scale").value,
                self.get_parameter("min_depth").value,
                self.get_parameter("max_depth").value,
            )
            valid = np.isfinite(points).all(axis=2)
            labels = np.where(valid, result.labels, 0).astype(np.uint32)
            ids = np.where(valid, result.instance_ids, 0).astype(np.uint32)
            confidence = np.where(valid, result.confidence, 0).astype(np.float32)
            self.label_pub.publish(array_to_image(result.labels, "32SC1", rgb.header))
            self.instance_pub.publish(
                array_to_image(result.instance_ids, "32SC1", rgb.header)
            )
            self.confidence_pub.publish(
                array_to_image(result.confidence, "32FC1", rgb.header)
            )
            self.cloud_pub.publish(
                with_labels(xyz_cloud(points, depth.header), labels, confidence, ids)
            )
            self.last_stamp = stamp
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(
                f"RGB-D segmentation failed: {exc}", throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RgbdSegNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
