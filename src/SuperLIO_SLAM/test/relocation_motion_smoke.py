#!/usr/bin/env python3
"""Real relocation node: stationary bootstrap, deskewed moving scans, async world handoff.

Requires sourced ROS/package overlays and compatible runtime library paths.
The temporary PCD and logs are isolated; no user map or sensor is modified.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def stamp(seconds):
    from builtin_interfaces.msg import Time
    value=round(seconds*1e9)
    return Time(sec=value//1000000000,nanosec=value%1000000000)


def motion(seconds):
    u=np.maximum(np.asarray(seconds)-1.0,0)
    x=np.where(u<2,.1*u*u,.4+.4*(u-2))
    acceleration=np.where((u>0)&(u<2),.2,0)
    yaw=.08*u
    rate=np.where(u>0,.08,0)
    return x,acceleration,yaw,rate


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--node',required=True)
    parser.add_argument('--frames',type=int,default=100)
    args=parser.parse_args()
    os.environ.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID','49')
    os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
    os.environ.setdefault('ROS_LOG_DIR','/tmp/superlio_relocation_test_logs')
    folder=Path(tempfile.mkdtemp(prefix='superlio_relocation_motion_'))
    rng=np.random.default_rng(47)
    pieces=[]
    for box in range(5):
        low=np.array([box*2.3-5,box%2*3.1-3,0])
        extent=np.array([1.5+box*.2,2.1,1.0+box*.4])
        for face in range(6):
            points=low+rng.random((50,3))*extent
            points[:,face//2]=low[face//2]+(face%2)*extent[face//2]
            pieces.append(points)
    scene=np.concatenate(pieces).astype(np.float32)
    angle=.35
    A=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    translation=np.array([3,-2,0])
    world=scene@A.T+translation
    xyzi=np.column_stack((world,np.ones(len(world)))).astype('<f4')
    map_path=folder/'reference.pcd'
    header=f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyzi)}\nHEIGHT 1\nPOINTS {len(xyzi)}\nDATA binary\n'
    map_path.write_bytes(header.encode()+xyzi.tobytes())
    params={
        'relocation.map_path':str(map_path), 'lio.ros.lidar_topic':'/test/relocation/cloud',
        'lio.ros.imu_topic':'/test/relocation/imu', 'lio.sensor.lidar_type':4,
        'lio.sensor.blind':.1, 'lio.sensor.maxrange':60.0, 'lio.sensor.filter_rate':1,
        'lio.sensor.voxel_fliter_size':.3, 'lio.sensor.gravity_norm':9.81,
        'lio.extrinsic.lidar_imu':[0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,1.],
        'lio.relocation.init_pose':[3.,-2.,0.,0.,0.,float(angle*180/np.pi)],
        'lio.output.map':True, 'lio.output.dense':True, 'lio.output.pub_step':1,
        'lio.output.robot':False, 'lio.map.save_map':False,
        'lio.hash_map.hash_capacity':2000000, 'lio.hash_map.vox_resolution':.5,
        'lio.kf.kf_max_iterations':4, 'lio.kf.kf_quit_eps':.001,
        'lio.sensor.imu_na':.1,'lio.sensor.imu_ng':.1,
        'lio.sensor.imu_nba':.0001,'lio.sensor.imu_nbg':.0001,
        'relocation.registration_min_frames':8,
        'relocation.registration_interval_s':.5, 'registration_voxel_size_m':.3,
        'relocation.local_map_visualization_interval_s':.1,
        'relocation.local_map_visualization_max_points':200,
        'fpfh_normal_radius_m':.8, 'fpfh_feature_radius_m':1.6,
    }
    import yaml
    config=folder/'parameters.yaml'
    config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    log_path=folder/'node.log'
    def log_tail():
        return '\n'.join(line for line in log_path.read_text().splitlines() if any(key in line for key in ('Relocation','LIO sync','ERROR','WARN','FAIL')))[-14000:]
    log=log_path.open('w')
    rclpy.init()
    node=rclpy.create_node('relocation_motion_smoke')
    poses=[]; states=[]; transforms=[]; previews=[]; paths=[]
    preview_qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Marker,'/lio/relocation/local_map_preview',previews.append,preview_qos)
    node.create_subscription(Odometry,'/lio/odom',poses.append,1000)
    node.create_subscription(RosPath,'/lio/path',paths.append,1000)
    node.create_subscription(String,'/lio/relocation/status',lambda msg:states.append(msg.data),
        QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage,'/tf',lambda msg:transforms.extend(msg.transforms),1000)
    sensor_qos=QoSProfile(depth=500,reliability=ReliabilityPolicy.BEST_EFFORT)
    imu_pub=node.create_publisher(Imu,'/test/relocation/imu',sensor_qos)
    cloud_pub=node.create_publisher(PointCloud2,'/test/relocation/cloud',sensor_qos)
    process=subprocess.Popen([args.node,'--ros-args','--log-level','super_lio:=debug','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+15
        while (not imu_pub.get_subscription_count() or not cloud_pub.get_subscription_count()) and time.monotonic()<deadline:
            assert process.poll() is None,log_tail()
            rclpy.spin_once(node,timeout_sec=.05)
        assert imu_pub.get_subscription_count() and cloud_pub.get_subscription_count(),'Sensor subscriptions unavailable'
        ready=time.monotonic()+1.0
        while time.monotonic()<ready:
            rclpy.spin_once(node,timeout_sec=.02)
        base=node.get_clock().now().nanoseconds/1e9+.1
        last_imu=-.005
        offsets=np.linspace(0,.1,len(scene),dtype=np.float32)
        fields=[PointField(name=name,offset=4*k,datatype=PointField.FLOAT32,count=1)
                for k,name in enumerate(('x','y','z','intensity','time'))]
        for frame in range(args.frames):
            start=frame*.1
            while last_imu<start+.11:
                last_imu+=.005
                _,acc,yaw,rate=motion(last_imu)
                imu=Imu(); imu.header.stamp=stamp(base+last_imu); imu.header.frame_id='imu'
                imu.angular_velocity.z=float(rate)
                imu.linear_acceleration.x=float(acc*np.cos(yaw))
                imu.linear_acceleration.y=float(-acc*np.sin(yaw))
                imu.linear_acceleration.z=9.81
                imu_pub.publish(imu)
            x,_,yaw,_=motion(start+offsets.astype(float))
            points=scene.copy(); points[:,0]-=x
            c=np.cos(yaw); sn=np.sin(yaw)
            local=points.copy()
            local[:,0]=c*points[:,0]+sn*points[:,1]
            local[:,1]=-sn*points[:,0]+c*points[:,1]
            data=np.column_stack((local,np.ones(len(scene)),offsets)).astype('<f4')
            cloud=PointCloud2(); cloud.header.stamp=stamp(base+start); cloud.header.frame_id='lidar'
            cloud.height=1; cloud.width=len(data); cloud.fields=fields
            cloud.point_step=20; cloud.row_step=20*len(data); cloud.is_dense=True
            cloud.data=data.tobytes(); cloud_pub.publish(cloud)
            until=time.monotonic()+.08
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.002)
            assert process.poll() is None,log_tail()
        deadline=time.monotonic()+30
        expected_end=base+args.frames*.1
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if poses and 'WORLD_MAP_TRACKING' in states and poses[-1].header.stamp.sec+poses[-1].header.stamp.nanosec*1e-9>expected_end-.15:
                break
        assert 'LOCAL_TRACKING' in states and 'WORLD_MAP_TRACKING' in states,log_tail()
        additions=[marker for marker in previews if marker.action==Marker.ADD]
        assert additions and all(0<len(marker.points)<=200 for marker in additions),'Missing/bounded local-map visualization failed'
        assert all(marker.header.frame_id=='world' for marker in previews),'Preview must be visible in the RViz canvas'
        # This scene starts near x=-5 in S; the supplied W<-S hint moves its
        # minimum x above -3.5. Only registration may apply that hint.
        assert min(p.x for p in additions[0].points)<-3.5,'Prior moved preview away from provisional odometry'
        assert previews[-1].action==Marker.DELETE,'Successful handoff left the local map visible'
        late_previews=[]
        late_sub=node.create_subscription(Marker,'/lio/relocation/local_map_preview',late_previews.append,preview_qos)
        deadline=time.monotonic()+3
        while not late_previews and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert late_previews and late_previews[-1].action==Marker.DELETE,'Late subscriber received stale local map'
        node.destroy_subscription(late_sub)
        def seconds(stamped):
            return stamped.header.stamp.sec+stamped.header.stamp.nanosec*1e-9
        clears=[path for path in paths if not path.poses]
        assert len(clears)==1,'Missing explicit provisional-path clear at handoff'
        handoff=seconds(clears[0])
        assert paths[-1].poses and seconds(paths[-1].poses[0])>=handoff,'World path retained provisional poses'
        assert all(msg.header.frame_id=='world' for msg in poses+paths),'Outputs must use world throughout'
        local_poses=[pose for pose in poses if seconds(pose)<handoff]
        world_poses=[pose for pose in poses if seconds(pose)>=handoff]
        assert len(local_poses)>=8 and len(world_poses)>=5,'Missing local/world scan tracking'
        times=[pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9 for pose in poses]
        assert all(b>a for a,b in zip(times,times[1:])),'Scan time went backwards or was reset'
        assert times[-1]>expected_end-.15,'Latest sensor scan was lost'
        errors=[]
        for pose in world_poses:
            t=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9-base
            x,_,_,_=motion(t)
            expected=A@np.array([x,0,0])+translation
            p=pose.pose.pose.position
            errors.append(np.linalg.norm(np.array([p.x,p.y,p.z])-expected))
        assert max(errors)<.35,f'World-frame motion error {max(errors):.4f} m'
        imu_transforms=[tf for tf in transforms if tf.child_frame_id=='imu']
        assert imu_transforms and all(tf.header.frame_id=='world' and tf.child_frame_id=='imu' for tf in transforms),'Expected only world -> imu TF'
        text=log_tail()
        match=re.search(r'Relocation committed: snapshot=([0-9.]+) latest=([0-9.]+) preserved_time=([0-9.]+)',text)
        assert match,'No atomic handoff diagnostic'
        old,latest,preserved=map(float,match.groups())
        assert latest>old and abs(latest-preserved)<1e-5,'Delayed registration rewound the filter'
        print(f'PASS: {len(local_poses)} local / {len(world_poses)} world poses; moving scan error max={max(errors):.4f} m; delayed handoff preserves time and TF')
        print(f'PASS: {len(additions)} local-map previews, point cap, handoff DELETE and late-subscriber clearing')
        print(f'Artifacts: {folder}')
    finally:
        node.destroy_node(); rclpy.shutdown()
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close()
        print('\n'.join(line for line in log_tail().splitlines() if any(key in line for key in ('Relocation','LIO sync','ERROR','WARN','FAIL')))[-14000:])

if __name__=='__main__':
    main()
