#!/usr/bin/env python3
"""Export recorded SuperLIO keyframes using the last optimized Path. Never overwrite output."""
import argparse,json,os,shutil,tempfile
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as RosPath
from super_lio_slam.msg import KeyFrame

def stamp(t): return t.sec*1000000000+t.nanosec

def rotation(q):
    v=np.array([q.x,q.y,q.z,q.w],dtype=float)
    n=np.linalg.norm(v)
    if not np.isfinite(n) or n<1e-12: raise ValueError('Invalid quaternion')
    x,y,z,w=v/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]),v/n

def xyz(c):
    fields={f.name:f for f in c.fields}
    endian='>' if c.is_bigendian else '<'
    a=[]
    for name in ('x','y','z'):
        f=fields[name]
        if f.datatype not in (7,8) or f.count!=1: raise ValueError('Unsupported XYZ field')
        arr=np.ndarray((c.height,c.width),dtype=endian+('f4' if f.datatype==7 else 'f8'),buffer=bytes(c.data),offset=f.offset,strides=(c.row_step,c.point_step))
        a.append(arr.reshape(-1))
    p=np.column_stack(a).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]

def export(bag,out):
    if out.exists(): raise ValueError(f'Output already exists: {out}')
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    frames={}; last=None; paths=0
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/lio_slam/keyframe':
            f=deserialize_message(data,KeyFrame); t=stamp(f.header.stamp)
            if t in frames: raise ValueError('Duplicate keyframe timestamp')
            frames[t]=f
        elif topic=='/lio_slam/path': last=deserialize_message(data,RosPath); paths+=1
    if last is None or not last.poses: raise ValueError('No final path')
    if last.header.frame_id!='map': raise ValueError('Expected map path frame')
    poses={stamp(p.header.stamp):p.pose for p in last.poses}
    if len(poses)!=len(last.poses): raise ValueError('Duplicate path timestamps')
    missing=set(poses)-set(frames)
    if missing: raise ValueError(f'{len(missing)} final path poses lack recorded clouds')
    if not frames: raise ValueError('No keyframes')
    # A truncated final path must not silently drop early frames.
    if min(poses)>min(frames): raise ValueError('Final path omits earliest recorded keyframe; investigate truncation/rejection')
    report={'bag':str(bag),'keyframes':len(frames),'path_messages':paths,'exported':len(poses),'excluded_keyframes_not_in_final_path':len(set(frames)-set(poses)),'frame':'map','pose_frame':'imu','points':0,'frames':[]}
    out.parent.mkdir(parents=True,exist_ok=True)
    temp=Path(tempfile.mkdtemp(prefix=out.name+'.partial-',dir=out.parent))
    try:
        (temp/'clouds').mkdir()
        with (temp/'poses.txt').open('w') as tum:
            for i,t in enumerate(sorted(poses)):
                f=frames[t]; pose=poses[t]
                if f.cloud.header.frame_id!='imu': raise ValueError(f'Unexpected cloud frame: {f.cloud.header.frame_id}')
                if stamp(f.cloud.header.stamp)!=t: raise ValueError('Cloud timestamp mismatch')
                p=xyz(f.cloud)
                if len(p)==0: raise ValueError('Empty cloud')
                R,q=rotation(pose.orientation)
                trans=np.array([pose.position.x,pose.position.y,pose.position.z])
                if not np.isfinite(trans).all(): raise ValueError('Invalid translation')
                world=np.asarray(p@R.T+trans,dtype='<f4')
                if not np.isfinite(world).all(): raise ValueError('Nonfinite transformed cloud')
                name=f'{i:06d}.pcd'
                header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n'
                with (temp/'clouds'/name).open('wb') as fp: fp.write(header.encode()); fp.write(world.tobytes())
                tum.write(f'{t//1000000000}.{t%1000000000:09d} '+ ' '.join(f'{x:.17g}' for x in [*trans,*q])+'\n')
                report['points']+=len(p)
                report['frames'].append({'file':name,'stamp_ns':t,'keyframe_id':int(f.id)})
        (temp/'export_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
        temp.rename(out)
    except BaseException:
        shutil.rmtree(temp); raise
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--bag',required=True,type=Path)
    a.add_argument('--out',required=True,type=Path)
    args=a.parse_args()
    export(args.bag.resolve(),args.out.resolve())
