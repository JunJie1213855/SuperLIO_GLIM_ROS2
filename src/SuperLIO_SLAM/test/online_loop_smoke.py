#!/usr/bin/env python3
"""Exercise the real STD -> GICP -> iSAM2 ROS node on a repeated static scene.

Run after sourcing the package overlay. An optional binary XYZI PCD supplies a
real scene; the default scene is deterministic and generated in memory.
"""
import argparse
import os
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from super_lio_slam.msg import KeyFrame
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage


def scene_points(pcd):
    if pcd:
        with open(pcd, 'rb') as stream:
            header = {}
            while True:
                line = stream.readline().decode('ascii').strip()
                if not line or line.startswith('#'):
                    continue
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    break
            if header['DATA'] != ['binary'] or set(header['SIZE']) != {'4'}:
                raise ValueError('Test PCD must use binary float32 scalar fields')
            fields = header['FIELDS']
            raw = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, len(fields))
            return raw[:, [fields.index(name) for name in ('x', 'y', 'z', 'intensity')]].copy()
    rng = np.random.default_rng(47)
    planes = []
    for lo, hi in [((-15, -12, 0), (15, 12, 5)),
                   ((-8, -5, 0), (-5, -2, 3)), ((1, -7, 0), (4, -3, 2)),
                   ((7, 3, 0), (10, 6, 4)), ((-4, 5, 0), (-1, 8, 2.5))]:
        lo, hi = np.array(lo), np.array(hi)
        for axis in range(3):
            for side in (lo[axis], hi[axis]):
                xyz = rng.uniform(lo, hi, (4000, 3))
                xyz[:, axis] = side
                planes.append(np.column_stack((xyz, np.ones(len(xyz)))))
    return np.concatenate(planes).astype('<f4')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', required=True, help='loop_backend_node executable')
    parser.add_argument('--pcd', help='Optional binary PCD scene')
    parser.add_argument('--moving', action='store_true', help='Use nonzero true translation and rotation')
    parser.add_argument('--reject-registration', action='store_true', help='Force the registration minimum-point gate to reject candidates')
    parser.add_argument('--mode', choices=['std_gicp', 'std_teaser_gicp'], default='std_gicp')
    parser.add_argument('--candidate-mode', choices=['std','distance'], default='std')
    parser.add_argument('--candidate-radius', type=float, default=15.0)
    parser.add_argument('--reject-distance', action='store_true')
    parser.add_argument('--submap-frames', type=int, default=4, help='New-mode causal window size')
    parser.add_argument('--coarse-inlier-ratio', type=float, default=0.3)
    parser.add_argument('--coarse-max-rmse', type=float, default=0.3)
    parser.add_argument('--loop-min-time-separation', type=float, default=0.0, help='Test override; runtime default is 30 s')
    parser.add_argument('--timestamp-step', type=float, default=0.0, help='Synthetic sensor seconds per frame; zero uses node time')
    parser.add_argument('--sensor-stamp-base', type=float, help='Sensor epoch independent of the node clock; requires a positive timestamp step')
    parser.add_argument('--reject-time-window', action='store_true')
    args = parser.parse_args()
    expect_rejection = args.reject_registration or args.reject_time_window or args.reject_distance
    os.environ.setdefault('ROS_DOMAIN_ID', '127')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/super_lio_slam_test_logs')
    points = scene_points(args.pcd)
    points = points[np.isfinite(points).all(axis=1)]
    log = tempfile.NamedTemporaryFile(prefix='super_lio_loop_test_', suffix='.log', delete=False)
    command = [args.backend, '--ros-args', '-p', 'skip_near_num:=2',
               '-p', 'loop_min_interval:=1', '-p', 'gicp_submap_half_width:=0',
               '-p', 'graph_loop_translation_sigma:=0.02',
               '-p', 'map_publish_stride:=1',
               '-p', f'loop_registration_mode:={args.mode}',
               '-p', f'loop_candidate_mode:={args.candidate_mode}',
               '-p', f'loop_candidate_radius_m:={args.candidate_radius}',
               '-p', f'registration_submap_max_keyframes:={args.submap_frames}',
               '-p', f'coarse_min_inlier_ratio:={args.coarse_inlier_ratio}',
               '-p', f'coarse_max_inlier_rmse_m:={args.coarse_max_rmse}',
               '-p', f'loop_min_time_separation_s:={args.loop_min_time_separation}']
    if args.reject_registration:
        command += ['-p', 'registration_min_points:=100000']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('online_loop_smoke')
    received, loops, std_triangles = [], [], []
    paths, maps, transforms, raw_positions = [], [], [], []
    node.create_subscription(Odometry, '/lio_slam/odom', received.append, 10)
    node.create_subscription(MarkerArray, '/lio_slam/loop_markers', loops.append, 10)
    node.create_subscription(MarkerArray, '/lio_slam/std_triangles', std_triangles.append, 10)
    node.create_subscription(NavPath, '/lio_slam/path', paths.append, 10)
    node.create_subscription(PointCloud2, '/lio_slam/map', maps.append, 10)
    node.create_subscription(TFMessage, '/tf', lambda msg: transforms.extend(
        t for t in msg.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'world'), 10)
    publisher = node.create_publisher(KeyFrame, '/lio_slam/keyframe', 10)
    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publisher.get_subscription_count(), 'Backend subscription unavailable'
        if args.sensor_stamp_base is not None:
            assert args.timestamp_step > 0, 'Sensor-epoch test requires increasing timestamps'
        base_stamp = round(args.sensor_stamp_base*1e9) if args.sensor_stamp_base is not None else node.get_clock().now().nanoseconds
        for index in range(5):
            msg = KeyFrame()
            msg.header.stamp = (Time(nanoseconds=base_stamp+round(index*args.timestamp_step*1e9)).to_msg()
                                if args.timestamp_step else node.get_clock().now().to_msg())
            msg.header.frame_id = 'world'
            msg.id = index
            # A grid-aligned 90-degree revisit preserves the procedural scene's
            # corners while exposing an inverted GICP/BetweenFactor transform.
            true_translation = np.array([2.0, 2.0, 0.0]) if args.moving and index >= 3 else np.zeros(3)
            true_yaw = np.pi / 2 if args.moving and index >= 3 else 0.0
            raw_yaw = true_yaw + (0.02 * index if args.moving else 0.0)
            msg.odom_pose.position.x = float(true_translation[0] + 0.25 * index)
            msg.odom_pose.position.y = float(true_translation[1])
            msg.odom_pose.orientation.z = float(np.sin(raw_yaw / 2))
            msg.odom_pose.orientation.w = float(np.cos(raw_yaw / 2))
            c, s = np.cos(true_yaw), np.sin(true_yaw)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            local_points = points.copy()
            local_points[:, :3] = (points[:, :3] - true_translation) @ rotation
            cloud = PointCloud2()
            cloud.header.stamp = msg.header.stamp
            cloud.header.frame_id = 'imu'
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            cloud.is_bigendian = False
            cloud.point_step, cloud.row_step = 16, 16 * len(points)
            cloud.is_dense = True
            cloud.data = np.ascontiguousarray(local_points, dtype='<f4').tobytes()
            msg.cloud = cloud
            raw_positions.append(np.array([msg.odom_pose.position.x, msg.odom_pose.position.y, 0.0]))
            publisher.publish(msg)
            deadline = time.monotonic() + 30
            while not any(o.header.stamp == msg.header.stamp for o in received) and time.monotonic() < deadline:
                assert process.poll() is None, 'Backend exited unexpectedly'
                rclpy.spin_once(node, timeout_sec=0.1)
            assert any(o.header.stamp == msg.header.stamp for o in received), f'No optimized odometry for keyframe {index}'
        deadline = time.monotonic() + (2 if expect_rejection else 20)
        while not loops and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        deadline = time.monotonic() + 2
        while len(std_triangles) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(std_triangles) >= 5, 'Missing online STD triangle markers'
        edges, vertices = std_triangles[-1].markers
        assert edges.header.frame_id == 'map' and edges.type == Marker.LINE_LIST
        assert vertices.header.frame_id == 'map' and vertices.type == Marker.SPHERE_LIST
        if args.candidate_mode=='std':
            assert 0 < len(edges.points) <= 600 and len(edges.points) % 6 == 0
        else:
            assert not edges.points and edges.action==Marker.DELETE, 'Distance mode generated STD geometry'
        assert len(vertices.points) * 2 == len(edges.points)
        # Wait for delayed loop corrections and their asynchronous map snapshot.
        deadline = time.monotonic() + (0.5 if expect_rejection else 15)
        while not expect_rejection and time.monotonic() < deadline:
            if received:
                position=received[-1].pose.pose.position
                q=received[-1].pose.pose.orientation
                measured_yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                angle_error=np.arctan2(np.sin(measured_yaw-true_yaw),np.cos(measured_yaw-true_yaw))
                if (np.linalg.norm(np.array([position.x,position.y,position.z])-true_translation)<0.5
                        and (not args.moving or abs(angle_error)<0.06)):
                    break
            rclpy.spin_once(node,timeout_sec=0.05)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert paths and len(paths[-1].poses) == 5, 'Incomplete optimized history'
        assert maps and maps[-1].header.frame_id == 'map' and maps[-1].width > 0, 'Missing rebuilt map'
        assert transforms, 'Missing map -> world correction'
        if args.sensor_stamp_base is not None:
            assert all(base_stamp <= t.header.stamp.sec*1000000000+t.header.stamp.nanosec <= base_stamp+round(4*args.timestamp_step*1e9) for t in transforms), 'Map correction TF used a non-sensor epoch'
            assert transforms[-1].header.stamp == received[-1].header.stamp, 'Delayed correction did not use latest keyframe time'
        has_loop = bool(loops and any(m.points for a in loops for m in a.markers))
        pose = received[-1].pose.pose
        corrected = float(np.linalg.norm(np.array([pose.position.x, pose.position.y, pose.position.z]) - true_translation))
        def yaw(q):
            return np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        def rotation(q):
            x,y,z,w=np.array([q.x,q.y,q.z,q.w])/np.linalg.norm([q.x,q.y,q.z,q.w])
            return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        cr,sr=np.cos(raw_yaw),np.sin(raw_yaw)
        raw_rotation=np.array([[cr,-sr,0],[sr,cr,0],[0,0,1]])
        def correction_matches_latest_pose():
            current_pose=received[-1].pose.pose
            expected_rotation=rotation(current_pose.orientation) @ raw_rotation.T
            expected_translation=np.array([current_pose.position.x,current_pose.position.y,current_pose.position.z])-expected_rotation @ raw_positions[-1]
            current_tf=transforms[-1].transform
            actual_translation=np.array([current_tf.translation.x,current_tf.translation.y,current_tf.translation.z])
            return (np.linalg.norm(actual_translation-expected_translation)<1e-5 and
                    np.linalg.norm(rotation(current_tf.rotation)-expected_rotation)<1e-5)
        # Odometry and TF are separate topics; wait for the same asynchronous graph revision.
        deadline=time.monotonic()+2
        while not correction_matches_latest_pose() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        assert correction_matches_latest_pose(), 'TF correction direction or timestamp mismatch'
        pose=received[-1].pose.pose
        corrected=float(np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-true_translation))
        last_path = paths[-1].poses[-1].pose.position
        assert np.linalg.norm(np.array([last_path.x,last_path.y,last_path.z])-np.array([pose.position.x,pose.position.y,pose.position.z])) < 1e-5, 'Path and graph odometry disagree'
        if expect_rejection:
            assert not has_loop, 'Rejected GICP candidate created a loop marker'
            assert abs(corrected - 1.0) < 0.01, f'Rejected GICP changed odometry: {corrected}'
            print('PASS: candidate gate rejection left the 1.000 m raw drift unchanged' if args.reject_time_window or args.reject_distance
                  else 'PASS: GICP rejection left the 1.000 m raw drift unchanged')
        else:
            assert has_loop, 'No accepted loop'
            history = paths[-1].poses[1].pose.position
            assert np.linalg.norm(np.array([history.x,history.y,history.z])-raw_positions[1]) > 0.02, 'Loop did not update historical poses'
            assert corrected < 0.5, f'PGO did not reduce artificial 1 m drift: {corrected}'
            if args.moving:
                estimated_yaw = yaw(pose.orientation)
                yaw_error = np.arctan2(np.sin(estimated_yaw-true_yaw), np.cos(estimated_yaw-true_yaw))
                assert abs(yaw_error) < 0.06, f'Incorrect loop rotation direction: estimated={estimated_yaw:.6f}, true={true_yaw:.6f}, error={yaw_error:.6f}'
            print(f'PASS: 5 keyframes, accepted loop, 1.000 m raw drift -> {corrected:.4f} m corrected')
        log.flush()
        diagnostic_log = re.sub(r'\x1b\[[0-9;]*m', '', Path(log.name).read_text())
        assert 'Loop search input=' not in diagnostic_log, 'Verbose search summary leaked into INFO'
        assert 'hash_candidates=' not in diagnostic_log, 'Verbose candidate counts leaked into INFO'
        if args.reject_time_window or args.reject_distance:
            assert 'matched:' not in diagnostic_log and 'accepted:' not in diagnostic_log and 'Distance candidate ' not in diagnostic_log, 'Candidate passed rejection gate'
        elif args.candidate_mode=='distance':
            assert 'Distance candidate ' in diagnostic_log and 'STD ' not in diagnostic_log, 'Distance retrieval was not exercised'
        else:
            assert 'STD ' in diagnostic_log and 'matched:' in diagnostic_log, 'Missing verified STD match'
        if args.reject_registration:
            assert ('stage=few_points' in diagnostic_log if args.mode == 'std_teaser_gicp'
                    else 'stage=few_gicp_points' in diagnostic_log), 'Missing registration gate diagnostic'
        if args.mode == 'std_teaser_gicp' and not expect_rejection:
            assert 'TEASER/GICP' in diagnostic_log, 'New registration mode was not exercised'
            assert 'coarse_rmse=' in diagnostic_log and 'gicp=executed' in diagnostic_log, 'Missing registration quality summary'
        elif args.mode == 'std_gicp' and not expect_rejection:
            assert 'accepted: GICP' in diagnostic_log, 'Direct GICP was not exercised'
            assert 'stage=accepted' in diagnostic_log, 'Missing direct GICP quality decision'
        print(f'Backend log: {log.name}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        print(Path(log.name).read_text())


if __name__ == '__main__':
    main()
