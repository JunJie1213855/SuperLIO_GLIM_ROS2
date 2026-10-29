#!/usr/bin/env python3
"""Exercise the real save service and SIGINT autosave in an isolated ROS domain."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointField
from std_srvs.srv import Trigger
from super_lio_slam.msg import KeyFrame


def check(value, message):
    if not value:
        raise RuntimeError(message)


def main():
    executable, runtime_file = sys.argv[1:]
    os.environ['ROS_DOMAIN_ID'] = '53'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = ':'.join(Path(runtime_file).read_text().splitlines() +
                                      [env.get('LD_LIBRARY_PATH', '')])
    with tempfile.TemporaryDirectory(prefix='optimized-map-service-') as directory:
        directory = Path(directory)
        output = directory / 'map.pcd'
        env['ROS_LOG_DIR'] = str(directory / 'ros_logs')
        rclpy.init()
        node = rclpy.create_node('optimized_map_service_test')
        publisher = node.create_publisher(KeyFrame, '/lio_slam/keyframe', 10)
        client = node.create_client(Trigger, '/lio_slam/save_optimized_map')
        paths = []
        node.create_subscription(RosPath, '/lio_slam/path', paths.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def wait(predicate, timeout=15):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            check(predicate(), 'Timed out waiting for backend')

        def call():
            future = client.call_async(Trigger.Request())
            wait(future.done)
            return future.result()

        def send(index):
            frame = KeyFrame()
            frame.id = index
            frame.header.stamp.sec = index + 1
            frame.header.frame_id = 'world'
            frame.odom_pose.orientation.w = 1.
            frame.odom_pose.position.x = 10. * index
            frame.cloud.header.stamp = frame.header.stamp
            frame.cloud.header.frame_id = 'imu'
            frame.cloud.height = 1
            frame.cloud.width = 20
            frame.cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            frame.cloud.point_step = 16
            frame.cloud.row_step = 320
            frame.cloud.is_dense = True
            frame.cloud.data = b''.join(struct.pack('<ffff', i * .1, 0., 0., 7.) for i in range(20))
            publisher.publish(frame)
            wait(lambda: bool(paths) and len(paths[-1].poses) == index + 1)

        def read():
            header, binary = output.read_bytes().split(b'DATA binary\n', 1)
            count = int(next(line.split()[1] for line in header.splitlines() if line.startswith(b'POINTS ')))
            check(len(binary) == count * 16, 'Invalid binary PCD length')
            return list(struct.iter_unpack('<ffff', binary))

        with (directory / 'backend.log').open('w') as log:
            process = subprocess.Popen([
                executable, '--ros-args', '-p', 'loop_candidate_mode:=distance',
                '-p', 'save_map_path:=' + str(output), '-p', 'save_map_voxel_size:=0.0',
                '-p', 'save_map_on_shutdown:=true', '-p', 'keyframe_cloud_voxel_size:=0.0',
                '-p', 'map_max_keyframes:=1', '-p', 'map_max_points_per_keyframe:=1'],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                check(client.wait_for_service(timeout_sec=15), 'Save service missing')
                wait(lambda: publisher.get_subscription_count() > 0)
                check(not call().success and not output.exists(), 'Empty graph must fail without a PCD')
                send(0)
                send(1)
                check(call().success, 'Save service failed')
                points = read()
                check(len(points) == 40, 'Display limits truncated exported map')
                check(abs(points[20][0] - 10.) < 1e-5 and points[20][3] == 7., 'Wrong map transform/intensity')
                check(call().success, 'Repeated atomic replacement failed')
                send(2)
                process.send_signal(signal.SIGINT)
                check(process.wait(timeout=20) == 0, 'Backend failed on graceful shutdown')
                check(len(read()) == 60, 'Shutdown did not export latest keyframe')
                print('Service, empty-graph failure, complete export, replacement and SIGINT autosave: passed')
            except BaseException:
                print((directory / 'backend.log').read_text())
                raise
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
