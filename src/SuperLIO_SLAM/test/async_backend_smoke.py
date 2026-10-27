#!/usr/bin/env python3
"""Burst real keyframes while registration is busy; require lossless graph ingress.

Use a private ROS_DOMAIN_ID and source the workspace before running. This exercises
actual STD/TEASER/GICP workers, task eviction, map coalescing and delayed loops.
"""
import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2, PointField
from super_lio_slam.msg import KeyFrame
from visualization_msgs.msg import MarkerArray
from online_loop_smoke import scene_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', required=True)
    parser.add_argument('--frames', type=int, default=24)
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '143')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/super_lio_async_test_logs')
    log = tempfile.NamedTemporaryFile(prefix='superlio_async_burst_', suffix='.log', delete=False)
    process = subprocess.Popen([args.backend, '--ros-args',
        '-p', 'loop_registration_mode:=std_teaser_gicp', '-p', 'loop_candidate_mode:=std',
        '-p', 'registration_queue_capacity:=1', '-p', 'skip_near_num:=2',
        '-p', 'loop_min_interval:=0', '-p', 'loop_min_time_separation_s:=0.0', '-p', 'registration_submap_max_keyframes:=2',
        '-p', 'coarse_min_inlier_ratio:=0.2', '-p', 'coarse_max_inlier_rmse_m:=0.35',
        '-p', 'graph_loop_translation_sigma:=0.02', '-p', 'map_publish_stride:=1',
        '-p', 'map_max_points_per_keyframe:=30000', '-p', 'map_voxel_size:=0.1'],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('async_backend_burst')
    paths, odometry, loops, maps = [], [], [], []
    node.create_subscription(NavPath, '/lio_slam/path', paths.append, 100)
    node.create_subscription(Odometry, '/lio_slam/odom', odometry.append, 100)
    node.create_subscription(MarkerArray, '/lio_slam/loop_markers', loops.append, 100)
    node.create_subscription(PointCloud2, '/lio_slam/map', maps.append, 10)
    publisher = node.create_publisher(KeyFrame, '/lio_slam/keyframe', 200)
    try:
        deadline = time.monotonic()+15
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        assert publisher.get_subscription_count(), 'Missing backend subscription'
        points = scene_points(None)
        fields = [PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1)
                  for i,name in enumerate(('x','y','z','intensity'))]
        # Every keyframe sees identical geometry but accumulates 0.25 m raw drift.
        blob = np.ascontiguousarray(points,dtype='<f4').tobytes()
        stamps = []
        begin = time.monotonic()
        for index in range(args.frames):
            msg = KeyFrame()
            msg.id = index
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.odom_pose.orientation.w = 1.0
            msg.odom_pose.position.x = .25*index
            msg.cloud = PointCloud2(header=msg.header,height=1,width=len(points),fields=fields,
                is_bigendian=False,point_step=16,row_step=16*len(points),is_dense=True,data=blob)
            msg.cloud.header.frame_id = 'imu'
            stamps.append((msg.header.stamp.sec,msg.header.stamp.nanosec))
            publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=.01)
        deadline = time.monotonic()+90
        while (not paths or len(paths[-1].poses)<args.frames) and time.monotonic()<deadline:
            assert process.poll() is None, 'Backend exited during burst'
            rclpy.spin_once(node,timeout_sec=.05)
        assert paths and len(paths[-1].poses)==args.frames, 'A keyframe/odometry graph node was lost'
        ingest_seconds = time.monotonic()-begin
        # Let the running task and newest pending task finish, including map rebuild.
        deadline = time.monotonic()+20
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        seen = {(o.header.stamp.sec,o.header.stamp.nanosec) for o in odometry}
        assert set(stamps).issubset(seen), 'Not every input keyframe produced odometry'
        assert process.poll() is None, 'Backend crashed after delayed registration'
        assert len(paths[-1].poses)==args.frames, 'Delayed loop truncated the trajectory'
        assert maps and maps[-1].width>0 and maps[-1].header.frame_id=='map', 'Missing asynchronous map'
        assert (maps[-1].header.stamp.sec,maps[-1].header.stamp.nanosec)==stamps[-1], 'Final map snapshot omitted latest frame'
        assert loops and any(m.points for a in loops for m in a.markers), 'No delayed loop accepted'
        assert (odometry[-1].header.stamp.sec,odometry[-1].header.stamp.nanosec)==stamps[-1], 'Delayed loop published old current pose'
        accepted = []
        for array in loops:
            for marker in array.markers:
                accepted.extend(marker.points)
        assert accepted, 'Empty loop markers'
        text = Path(log.name).read_text()
        assert 'Keyframe ID gap' not in text and 'failed:' not in text, text
        dropped = [int(n) for n in re.findall(r'dropped_tasks=(\d+)',text)]
        assert dropped and max(dropped)>0, 'Burst did not exercise bounded-queue eviction'
        # The graph mutex never protects a TEASER call: observed ingress should
        # finish ahead of a serial execution of the logged registration work.
        waits = re.findall(r'Registration timing frame=(\d+) queue_wait=([\d.]+) ms work=([\d.]+) ms',text)
        assert waits, 'Missing worker timing diagnostics'
        print(f'PASS: {args.frames} graph nodes retained in {ingest_seconds:.2f} s; '
              f'dropped detection tasks={max(dropped)}; delayed loop keeps latest pose; map published')
        print(f'Backend log: {log.name}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        log.close()
        print(Path(log.name).read_text())


if __name__ == '__main__':
    main()
