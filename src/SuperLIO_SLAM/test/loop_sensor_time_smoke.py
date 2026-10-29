#!/usr/bin/env python3
"""Exercise production backend TF with sensor epochs, pauses and delayed keyframes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from super_lio_slam.msg import KeyFrame

def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec

def stamp(seconds):
    return Time(nanoseconds=round(seconds * 1e9), clock_type=ClockType.ROS_TIME).to_msg()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", type=Path, required=True)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    runtime = args.backend.resolve().parent / "runtime_library_dirs.txt"
    if runtime.exists():
        env["LD_LIBRARY_PATH"] = ":".join(runtime.read_text().splitlines() + [env.get("LD_LIBRARY_PATH", "")])
    log = (args.out / "backend.log").open("w")
    command = [str(args.backend), "--ros-args", "-p", "loop_candidate_mode:=distance",
               "-p", "loop_registration_mode:=std_gicp", "-p", "map_publish_stride:=1",
               "-p", "loop_min_time_separation_s:=10000.0", "-p", "tf_publish_rate_hz:=40.0",
               "-p", "use_sim_time:=" + str(args.use_sim_time).lower()]
    process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("loop_sensor_time_smoke")
    buffer = Buffer(cache_time=Duration(seconds=30))
    transforms, poses = [], []
    def receive_tf(msg):
        for tf in msg.transforms:
            buffer.set_transform(tf, "sensor_time_smoke")
            if tf.header.frame_id == "map" and tf.child_frame_id == "world":
                transforms.append(tf)
    subscriptions = [
        node.create_subscription(TFMessage, "/tf", receive_tf, 100),
        node.create_subscription(Odometry, "/lio_slam/odom", poses.append, 100)]
    keyframes = node.create_publisher(KeyFrame, "/lio_slam/keyframe", 10)
    odometry = node.create_publisher(Odometry, "/lio/odom", 10)
    imu = node.create_publisher(Odometry, "/lio/imu/odom", 10)
    tfpub = node.create_publisher(TFMessage, "/tf", 100)
    clock = node.create_publisher(Clock, "/clock", 10) if args.use_sim_time else None
    def spin_until(condition, timeout=8):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            assert process.poll() is None, "Backend exited"
            rclpy.spin_once(node, timeout_sec=.02)
        assert condition(), "Timed out waiting for backend data-time TF"
    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
    rng = np.random.default_rng(19)
    xyz = rng.uniform(-3, 3, (1000, 4)).astype("<f4")
    xyz[:, 3] = 1
    def keyframe(ident, seconds):
        msg = KeyFrame()
        msg.id = ident
        msg.header.frame_id = "world"
        msg.header.stamp = stamp(seconds)
        msg.odom_pose.orientation.w = 1.0
        cloud = PointCloud2()
        cloud.header.frame_id = "imu"
        cloud.header.stamp = msg.header.stamp
        cloud.height, cloud.width = 1, len(xyz)
        cloud.point_step, cloud.row_step = 16, 16 * len(xyz)
        cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                        for i, name in enumerate(("x", "y", "z", "intensity"))]
        cloud.data = xyz.tobytes()
        msg.cloud = cloud
        keyframes.publish(msg)
        spin_until(lambda: any(ns(o.header.stamp) == round(seconds*1e9) for o in poses))
    def frontend(seconds):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = "world", "imu"
        msg.header.stamp = stamp(seconds)
        msg.pose.pose.orientation.w = 1.0
        odometry.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "imu"
        tf.transform.rotation.w = 1.0
        tfpub.publish(TFMessage(transforms=[tf]))
        prediction = Odometry()
        prediction.header.frame_id = "world"
        prediction.header.stamp = stamp(seconds + .02)
        prediction.pose.pose.orientation.w = 1.0
        imu.publish(prediction)
        spin_until(lambda: transforms and ns(transforms[-1].header.stamp) >= round((seconds+.02)*1e9))
        at = Time(nanoseconds=round(seconds*1e9), clock_type=ClockType.ROS_TIME)
        spin_until(lambda: buffer.can_transform("map", "imu", at))
        buffer.lookup_transform("map", "imu", at)
    try:
        spin_until(lambda: keyframes.get_subscription_count() > 0 and odometry.get_subscription_count() > 0)
        keyframe(0, 1000.0)
        spin_until(lambda: bool(transforms))
        for seconds in (1000.1, 1000.2, 1000.3, 1000.4, 1000.5):
            frontend(seconds)
        latest = ns(transforms[-1].header.stamp)
        assert latest == round(1000.52 * 1e9)
        before = len(transforms)
        keyframe(1, 1000.15)  # Backend work is older than live frontend data.
        spin_until(lambda: len(transforms) > before)
        assert ns(transforms[-1].header.stamp) == latest, "Delayed keyframe rewound TF"
        before = len(transforms)
        if clock:
            for seconds in (2000000000.0, 1000000000.0):
                msg = Clock(); msg.clock = stamp(seconds); clock.publish(msg); spin_for(.15)
        spin_for(.3)
        assert len(transforms) == before, "Paused sensor TF advanced on wall/simulated time"
        old = Odometry(); old.header.frame_id = "world"; old.header.stamp = stamp(999)
        old.pose.pose.orientation.w = 1.0; odometry.publish(old)
        unrelated = Odometry(); unrelated.header.frame_id = "odom_local"; unrelated.header.stamp = stamp(9000)
        unrelated.pose.pose.orientation.w = 1.0; odometry.publish(unrelated)
        spin_for(.15)
        assert len(transforms) == before, "Stale/unrelated odometry changed data time"
        frontend(1000.8)
        stamps = [ns(tf.header.stamp) for tf in transforms]
        assert all(a <= b for a,b in zip(stamps, stamps[1:])), "TF timestamps regressed"
        assert all(round(1000e9) <= t <= round(1000.82e9) for t in stamps), "TF used wall or /clock epoch"
        result = dict(passed=True, use_sim_time=args.use_sim_time,
                      clock_published=bool(clock), sensor_first_ns=stamps[0],
                      sensor_last_ns=stamps[-1], map_tf_messages=len(stamps),
                      checks=["sensor_epoch", "map_imu_lookup", "live_imu_coverage",
                              "delayed_keyframe", "pause", "stale_input", "unrelated_frame"])
        (args.out/"result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        log.close()

if __name__ == "__main__":
    main()
