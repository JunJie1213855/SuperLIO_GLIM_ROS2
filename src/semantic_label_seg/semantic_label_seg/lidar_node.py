"""ROS 2 Humble LiDAR interface; labels never overwrite reflectance."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .colors import label_colors, load_color_map
from .geometry import transform_points
from .lidar import RangeNetSegmenter
from .ros_io import cloud_to_arrays, with_labels


class LidarSegNode(Node):
    def __init__(self, **kwargs):
        super().__init__("lidar_seg", **kwargs)
        defaults = {
            "model_dir": "",
            "device": "auto",
            "max_range": 40.0,
            "confidence_threshold": 0.0,
            "label_space": "learning",
            "label_map_file": "",
            "unknown_label": 0,
            "cpu_threads": 4,
            "input_type": "pointcloud",
            "input_topic": "/semantic_label_seg/input_points",
            "output_topic": "/semantic_label_seg/points",
            "inference_frame": "",
            "desired_frequency": 2.0,
            "publish_rgb": True,
            "color_map_file": "",
            "tf_wait_timeout": 2.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        get = lambda key: self.get_parameter(key).value
        self.inference_frame = get("inference_frame")
        self.frequency = float(get("desired_frequency"))
        if self.frequency < 0:
            raise ValueError("desired_frequency must be nonnegative")
        self.last_stamp = None
        self.pending_cloud = None
        self.waiting_cloud = None
        self.wait_started = None
        self.tf_wait_timeout = float(get("tf_wait_timeout"))
        if self.tf_wait_timeout <= 0:
            raise ValueError("tf_wait_timeout must be positive")
        self.publish_rgb = get("publish_rgb")
        self.color_map = load_color_map(get("color_map_file"))
        self.unknown_label = get("unknown_label")
        self.segmenter = RangeNetSegmenter(
            get("model_dir"),
            device=get("device"),
            max_range=get("max_range"),
            confidence_threshold=get("confidence_threshold"),
            label_space=get("label_space"),
            label_map=get("label_map_file") or None,
            unknown_label=get("unknown_label"),
            cpu_threads=get("cpu_threads"),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, get("output_topic"), qos_profile_sensor_data
        )
        input_type = get("input_type")
        if input_type == "pointcloud":
            message_type, callback = PointCloud2, self.receive_cloud
        elif input_type == "superlio_keyframe":
            try:
                from super_lio_slam.msg import KeyFrame
            except ImportError as exc:
                raise ImportError(
                    "source the Super-LIO install/setup.bash before using superlio_keyframe"
                ) from exc
            message_type, callback = KeyFrame, lambda msg: self.receive_cloud(msg.cloud)
        else:
            raise ValueError("input_type must be pointcloud or superlio_keyframe")
        self.subscription = self.create_subscription(
            message_type, get("input_topic"), callback, qos_profile_sensor_data
        )
        # Let TF callbacks run before inference; retry the latest scan when TF
        # for its timestamp arrives later than the cloud on another DDS topic.
        self.timer = self.create_timer(0.02, self.process_pending)
        self.get_logger().info(
            f"RangeNet ready on {self.segmenter.device}; labels: {self.segmenter.label_names}"
        )
        if not self.inference_frame:
            self.get_logger().warning(
                "inference_frame is empty: input XYZ must already be in the trained LiDAR frame"
            )

    def receive_cloud(self, cloud):
        self.pending_cloud = cloud

    def process_pending(self):
        cloud = (
            self.waiting_cloud if self.waiting_cloud is not None else self.pending_cloud
        )
        if cloud is None:
            return
        if self.process_cloud(cloud):
            self.waiting_cloud = None
            self.wait_started = None
            if self.pending_cloud is cloud:
                self.pending_cloud = None
        else:
            if self.waiting_cloud is None:
                # Keep this scan until its TF catches up. Replacing it with
                # every newer scan can starve output if TF is always late.
                self.waiting_cloud = cloud
                self.wait_started = time.monotonic()
                if self.pending_cloud is cloud:
                    self.pending_cloud = None
            elif time.monotonic() - self.wait_started >= self.tf_wait_timeout:
                self.waiting_cloud = None
                self.wait_started = None

    def process_cloud(self, cloud):
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if (
            self.frequency > 0
            and self.last_stamp is not None
            and stamp >= self.last_stamp
            and stamp - self.last_stamp < 1e9 / self.frequency
        ):
            return True
        try:
            if not cloud.header.frame_id:
                raise ValueError("input cloud has no frame_id")
            if not self.inference_frame and cloud.header.frame_id in {
                "map",
                "world",
                "odom",
            }:
                raise ValueError(
                    "world-frame cloud requires inference_frame and a valid TF to the LiDAR"
                )
            xyz, intensity = cloud_to_arrays(cloud)
            if self.inference_frame and cloud.header.frame_id != self.inference_frame:
                tf = self.tf_buffer.lookup_transform(
                    self.inference_frame,
                    cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                )
                t, q = tf.transform.translation, tf.transform.rotation
                xyz = transform_points(xyz, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            result = self.segmenter.segment(xyz, intensity)
            colors = (
                label_colors(result.labels, self.color_map, self.unknown_label)
                if self.publish_rgb
                else None
            )
            self.publisher.publish(
                with_labels(cloud, result.labels, result.confidence, colors=colors)
            )
            self.last_stamp = stamp
            return True
        except TransformException as exc:
            self.get_logger().warning(
                f"waiting for cloud TF: {exc}", throttle_duration_sec=5.0
            )
            return False
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(
                f"cloud segmentation failed: {exc}", throttle_duration_sec=5.0
            )
            return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarSegNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
