#!/usr/bin/env python3
"""
把 GLIM dump 转回外部 LIO 的通用结果（PCD 序列 + TUM 位姿）—— `lio2glim.py`
的逆过程。

输入
----
一个 GLIM dump 目录（`offline_viewer` 能打开的那种）：

    dump/
    ├── graph.txt                     # num_submaps / num_all_frames / ...
    ├── config/                       # 忽略
    ├── 000000/{data.txt, points_compact.bin, covs_compact.bin, imu_rate.txt}
    └── ...

坐标约定（与 GLIM 源码一致，见 sub_mapping.cpp:459-461, 485）
------------------------------------------------------------
    子图点云存的是 **origin 系** 的点：p_origin = T_world_origin^-1 @ p_world
    因此还原到世界系：                   p_world  = T_world_origin   @ p_origin
    data.txt 里每帧的 T_world_lidar 已是世界系位姿，直接可用。

产出
----
    out/
    ├── clouds/000000.pcd ...         # 每个子图一张世界系点云（与 poses 1:1）
    ├── poses.txt                     # TUM：每个子图一行，取 T_world_origin
    ├── frames.txt                    # TUM：dump 内每一帧一行（T_world_lidar）
    ├── merged.pcd                    # 可选（--global-cloud）：整幅地图合成一张世界系点云
    └── glim2lio_manifest.json

`clouds/` 与 `poses.txt` 按子图一一对应，可直接喂回 lio2glim.py 形成闭环；
`frames.txt` 保留 dump 里的完整轨迹（帧数多于子图数，不与点云对应）。

注意：GLIM 把每个子图内的帧合并、降采样成一张点云，逐帧点云已不可恢复，
所以这里按「子图」输出点云，而位姿既有子图粒度（poses.txt）也有帧粒度
（frames.txt）。
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------- data.txt 解析

def parse_data_txt(path):
    """按 SubMap::load() 的顺序解析 data.txt（token 流，标签名不敏感）。

    返回 {id, T_world_origin, frames:[{id, stamp, T_world_lidar}]}。
    """
    tokens = Path(path).read_text().split()
    pos = 0

    def next_tokens(n=1):
        nonlocal pos
        vals = tokens[pos:pos + n]
        if len(vals) < n:
            raise ValueError(f"{path}: 内容在读取 {n} 个字段时提前结束")
        pos += n
        return vals

    def expect(label):
        tok = next_tokens()[0]
        if tok != label:
            raise ValueError(f"{path}: 期望标签 {label!r}，实际为 {tok!r}")

    def matrix(rows, cols):
        return np.array([float(v) for v in next_tokens(rows * cols)],
                        dtype=np.float64).reshape(rows, cols)

    expect("id:")
    sub_id = int(next_tokens()[0])

    expect("T_world_origin:")
    T_world_origin = matrix(4, 4)
    expect("T_origin_endpoint_L:")
    matrix(4, 4)                      # 不需要，但要吃掉 token
    expect("T_origin_endpoint_R:")
    matrix(4, 4)

    # 这些块在无帧时 save() 会省略，所以按标签出现与否可选读取
    T_lidar_imu = np.eye(4)
    if pos < len(tokens) and tokens[pos] == "T_lidar_imu:":
        expect("T_lidar_imu:")
        T_lidar_imu = matrix(4, 4)
        if pos < len(tokens) and tokens[pos] == "imu_bias:":
            expect("imu_bias:")
            next_tokens(6)
        if pos < len(tokens) and tokens[pos] == "frame_id:":
            expect("frame_id:")
            next_tokens(1)

    expect("num_frames:")
    num_frames = int(next_tokens()[0])

    frames = []
    for i in range(num_frames):
        expect("frame_%d" % i)
        expect("id:")
        fid = int(next_tokens()[0])
        expect("stamp:")
        stamp = float(next_tokens()[0])
        expect("T_odom_lidar:")
        matrix(4, 4)                  # odom 位姿：本脚本不需要（T_world 已优化）
        expect("T_world_lidar:")
        T_world_lidar = matrix(4, 4)
        expect("v_world_imu:")
        next_tokens(3)
        frames.append({"id": fid, "stamp": stamp, "T_world_lidar": T_world_lidar})

    return {
        "id": sub_id,
        "T_world_origin": T_world_origin,
        "T_lidar_imu": T_lidar_imu,
        "frames": frames,
    }


# ---------------------------------------------------------------- 二进制读入

def read_points_bin(path):
    """points_compact.bin：N×3 float32，无文件头（gtsam_points PointCloudCPU）。"""
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 3:
        raise ValueError(f"{path}: 字节数不是 3 的整数倍，可能不是 points_compact.bin")
    return raw.reshape(-1, 3).astype(np.float64)


# ---------------------------------------------------------------- 位姿 / PCD 写出

def rot2quat(R):
    """旋转矩阵 → 四元数 (qx, qy, qz, qw)。"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s]
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s]
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s]


def write_tum_line(f, stamp, T):
    qx, qy, qz, qw = rot2quat(T[:3, :3])
    t = T[:3, 3]
    f.write("%.9f %s\n" % (stamp, " ".join("%.17g" % v for v in (t[0], t[1], t[2], qx, qy, qz, qw))))


def write_pcd(path, pts, binary=True):
    n = len(pts)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA %s\n"
        % (n, n, "binary" if binary else "ascii")
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        if binary:
            f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())
        else:
            np.savetxt(f, pts, fmt="%.6f %.6f %.6f")


def maybe_downsample(pts, voxel_size):
    if voxel_size <= 0 or len(pts) == 0:
        return pts
    try:
        import open3d as o3d
    except ImportError:
        sys.exit("需要 open3d 做体素降采样: pip install open3d（或用 --voxel-size 0 关闭）")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return np.asarray(pcd.voxel_down_sample(voxel_size).points)


# ---------------------------------------------------------------- 寻找子图

def find_submaps(dump):
    """返回按编号升序排列的子图目录列表。"""
    subs = []
    for d in sorted(dump.iterdir()):
        if d.is_dir() and d.name.isdigit() and (d / "data.txt").exists():
            subs.append(d)
    return subs


def read_graph_txt(dump):
    """读 graph.txt 里的头部计数（缺省时返回 None）。"""
    f = dump / "graph.txt"
    if not f.exists():
        return {}
    info = {}
    for line in f.read_text().splitlines():
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if key in ("num_submaps", "num_all_frames", "num_matching_cost_factors") and val:
            info[key] = int(val)
    return info


# ---------------------------------------------------------------- 主流程

def main():
    ap = argparse.ArgumentParser(
        description="GLIM dump → PCD 序列 + TUM 位姿（lio2glim.py 的逆过程）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例：\n  python3 tools/glim2lio.py "
               "--dump /path/firstpart_dump --out /tmp/firstpart_reverse")
    ap.add_argument("--dump", required=True, help="GLIM dump 目录")
    ap.add_argument("--out", required=True, help="输出目录（不得已存在）")
    ap.add_argument("--voxel-size", type=float, default=0.0,
                    help="对输出点云做体素降采样，0 = 不降采样（默认 0）")
    ap.add_argument("--ascii", action="store_true", help="写 ASCII PCD（默认二进制）")
    ap.add_argument("--global-cloud", action="store_true",
                    help="额外写一张全局点云 <out>/merged.pcd（所有子图拼成一张世界系点云）")
    ap.add_argument("--global-voxel-size", type=float, default=None,
                    help="全局点云的体素降采样边长，缺省沿用 --voxel-size（0 = 不降采样）")
    ap.add_argument("--force", action="store_true", help="允许覆盖已存在的输出目录")
    a = ap.parse_args()

    dump = Path(a.dump)
    out = Path(a.out)
    if not dump.is_dir():
        sys.exit(f"不是目录：{dump}")
    if out.exists() and any(out.iterdir()) and not a.force:
        sys.exit(f"输出目录已存在且非空：{out}（先移走或加 --force）")
    if a.force and out.exists():
        shutil.rmtree(out)            # 清掉上次残留，避免子图数变少时留下旧点云

    subs = find_submaps(dump)
    if not subs:
        sys.exit(f"{dump} 下没找到子图目录（如 000000/data.txt）")

    graph = read_graph_txt(dump)
    if graph.get("num_submaps", len(subs)) != len(subs):
        print(f"⚠ graph.txt 记 num_submaps={graph.get('num_submaps')}，实际找到 {len(subs)} 个",
              file=sys.stderr)

    # 先全部解析成功，再落盘：出错时不会留下半成品
    parsed = []
    for d in subs:
        info = parse_data_txt(d / "data.txt")
        bin_path = d / "points_compact.bin"
        pts_origin = read_points_bin(bin_path) if bin_path.exists() else np.zeros((0, 3))
        parsed.append((d.name, info, pts_origin))

    out.mkdir(parents=True, exist_ok=True)
    (out / "clouds").mkdir(exist_ok=True)

    print(f"共 {len(parsed)} 个子图，{sum(len(i['frames']) for _, i, _ in parsed)} 帧，输出到 {out}")

    total_pts = 0
    manifest_frames = []
    global_chunks = []                # --global-cloud 时累积全部世界系点
    with (out / "poses.txt").open("w") as poses_f, (out / "frames.txt").open("w") as frames_f:
        for name, info, pts_origin in parsed:
            T_wo = info["T_world_origin"]
            # origin 系 → 世界系
            pts_world = pts_origin @ T_wo[:3, :3].T + T_wo[:3, 3] if len(pts_origin) else pts_origin
            pts_world = maybe_downsample(pts_world, a.voxel_size)

            pcd_name = "%s.pcd" % name
            write_pcd(out / "clouds" / pcd_name, pts_world, binary=not a.ascii)
            if a.global_cloud and len(pts_world):
                global_chunks.append(pts_world)

            # poses.txt：子图粒度，与 clouds/ 一一对应
            write_tum_line(poses_f, info["frames"][len(info["frames"]) // 2]["stamp"]
                           if info["frames"] else 0.0, T_wo)

            # frames.txt：dump 内的每一帧
            for fr in info["frames"]:
                write_tum_line(frames_f, fr["stamp"], fr["T_world_lidar"])

            total_pts += len(pts_world)
            manifest_frames.append({
                "submap": name, "cloud": pcd_name, "points": int(len(pts_world)),
                "frames": len(info["frames"]),
            })
            print(f"  子图 {name}: {len(info['frames'])} 帧 → {len(pts_world)} 点")

    manifest = {
        "dump": str(dump.resolve()),
        "submaps": len(parsed),
        "num_all_frames": graph.get("num_all_frames", sum(len(i["frames"]) for _, i, _ in parsed)),
        "points": total_pts,
        "voxel_size": a.voxel_size,
        "clouds": manifest_frames,
    }

    if a.global_cloud:
        if global_chunks:
            merged = np.vstack(global_chunks)
            gvox = a.voxel_size if a.global_voxel_size is None else a.global_voxel_size
            merged = maybe_downsample(merged, gvox)
            write_pcd(out / "merged.pcd", merged, binary=not a.ascii)
            manifest["merged_points"] = int(len(merged))
            manifest["global_voxel_size"] = gvox
            print("\n全局点云：%d 点 → %s" % (len(merged), out / "merged.pcd"))
        else:
            print("\n⚠ 无点云，跳过全局点云", file=sys.stderr)

    (out / "glim2lio_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    print("\n完成：%d 个点云，%d 点" % (len(parsed), total_pts))
    print("  %s  —— 子图点云（世界系），与 poses.txt 1:1" % (out / "clouds"))
    print("  %s  —— 子图位姿（TUM）" % (out / "poses.txt"))
    print("  %s  —— 全部帧位姿（TUM，不与点云对应）" % (out / "frames.txt"))
    if a.global_cloud:
        print("  %s  —— 全局点云（所有子图拼成一张世界系点云）" % (out / "merged.pcd"))
    print("\n闭环回 lio2glim.py：")
    print("  python3 tools/lio2glim.py --poses %s --clouds %s --out <dump2>"
          % (out / "poses.txt", out / "clouds"))


if __name__ == "__main__":
    main()
