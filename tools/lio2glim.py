#!/usr/bin/env python3
"""
把外部 LIO 的结果（PCD 序列 + TUM 位姿）转成 GLIM dump，供 offline_viewer
加载与「合并会话」使用。

输入约定
--------
点云 : 每帧一个 .pcd，点已是 **世界系**（LIO 已配准，如 /cloud_registered）
位姿 : TUM 格式 poses.txt，每行 "t x y z qx qy qz qw"，位姿是 **IMU/body 系**

由于点云已是世界系、位姿也是世界系，二者在同一世界系对齐，无需外参，
T_lidar_imu 填单位阵即可自洽。

坐标约定（与 GLIM 源码一致，见 sub_mapping.cpp:459-461, 485）
------------------------------------------------------------
    T_world_origin      = T_world_imu[center],  center = N // 2
    T_origin_endpoint_L = T_world_origin^-1 @ T_world_imu[first]
    T_origin_endpoint_R = T_world_origin^-1 @ T_world_imu[last]
    点云                p_origin = T_world_origin^-1 @ p_world

产出
----
    out/
    ├── graph.txt
    ├── 000000/{data.txt, points_compact.bin, covs_compact.bin, imu_rate.txt}
    └── ...

默认生成 graph.bin / values.bin 和 config，用各子图 data.txt 的
T_world_origin 定位；--no-graph 只用于调试，不保证正常加载。
"""

import argparse
import os
import sys
from glob import glob
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("需要 open3d: pip install open3d")


# ---------------------------------------------------------------- TUM 位姿

def read_tum(path):
    """读 TUM 格式位姿 → (stamps, poses)，poses 为 N×4×4 的 T_world_imu。"""
    stamps, poses = [], []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            if len(v) < 8:
                print(f"  ⚠ {path}:{lineno} 字段不足 8 个，跳过", file=sys.stderr)
                continue
            t, x, y, z, qx, qy, qz, qw = (float(s) for s in v[:8])
            n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if n < 1e-12:
                print(f"  ⚠ {path}:{lineno} 四元数为零，跳过", file=sys.stderr)
                continue
            qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

            # 四元数 → 旋转矩阵
            R = np.array([
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
            ])
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            stamps.append(t)
            poses.append(T)
    return np.asarray(stamps), np.asarray(poses)


# ---------------------------------------------------------------- 二进制写出

def write_points_bin(path, xyz):
    """N×3 float32，无文件头（对应 gtsam_points 的 points_compact.bin）。"""
    np.ascontiguousarray(xyz, dtype=np.float32).tofile(path)


def write_covs_bin(path, covs):
    """N×6 float32，对称协方差上三角 c00 c01 c02 c11 c12 c22。"""
    c = np.asarray(covs, dtype=np.float64)          # N×3×3
    tri = np.stack([c[:, 0, 0], c[:, 0, 1], c[:, 0, 2],
                    c[:, 1, 1], c[:, 1, 2], c[:, 2, 2]], axis=1)
    np.ascontiguousarray(tri, dtype=np.float32).tofile(path)


# ---------------------------------------------------------------- data.txt

def fmt_matrix(m):
    return "\n".join(" ".join("%.17g" % v for v in row) for row in m)


def write_data_txt(path, sub, T_lidar_imu):
    """按 SubMap::load() 的顺序写（顺序 token 解析，标签名不敏感）。"""
    lines = []
    lines.append("id: %d" % sub["id"])
    lines.append("T_world_origin: \n" + fmt_matrix(sub["T_world_origin"]))
    lines.append("T_origin_endpoint_L: \n" + fmt_matrix(sub["T_origin_endpoint_L"]))
    lines.append("T_origin_endpoint_R: \n" + fmt_matrix(sub["T_origin_endpoint_R"]))
    lines.append("T_lidar_imu: \n" + fmt_matrix(T_lidar_imu))
    lines.append("imu_bias: " + " ".join(["0"] * 6))
    lines.append("frame_id: 0")
    lines.append("num_frames: %d" % len(sub["frames"]))

    for i, f in enumerate(sub["frames"]):
        lines.append("frame_%d" % i)
        lines.append("id: %d" % i)
        lines.append("stamp: %.9f" % f["stamp"])
        # 没有因子图优化，odom 与优化位姿等同
        lines.append("T_odom_lidar: \n" + fmt_matrix(f["T_world"]))
        lines.append("T_world_lidar: \n" + fmt_matrix(f["T_world"]))
        lines.append("v_world_imu: 0 0 0")

    Path(path).write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------- 主流程

def build_submaps(stamps, poses, cloud_paths, frames_per_submap):
    """按 frames_per_submap 切分子图，返回 [{frames:[{stamp,T_world}], ...}]。"""
    n = len(poses)
    subs = []
    for s in range(0, n, frames_per_submap):
        idx = list(range(s, min(s + frames_per_submap, n)))
        frames = [{"stamp": float(stamps[i]), "T_world": poses[i],
                   "cloud": cloud_paths[i]} for i in idx]
        center = idx[len(idx) // 2]
        T_wo = poses[center]

        subs.append({
            "id": len(subs),
            "T_world_origin": T_wo,
            "T_origin_endpoint_L": np.linalg.inv(T_wo) @ poses[idx[0]],
            "T_origin_endpoint_R": np.linalg.inv(T_wo) @ poses[idx[-1]],
            "frames": frames,
        })
    return subs


def load_and_merge(sub, voxel_size):
    """把子图内所有帧的点（已在世界系）转到 origin 系并合并，可选体素降采样。"""
    inv = np.linalg.inv(sub["T_world_origin"])
    chunks = []
    for f in sub["frames"]:
        pcd = o3d.io.read_point_cloud(f["cloud"])
        pts = np.asarray(pcd.points)
        if pts.size == 0:
            continue
        # 世界系 → origin 系（同一点云已在同一世界系，无需外参）
        chunks.append(pts @ inv[:3, :3].T + inv[:3, 3])

    if not chunks:
        return np.zeros((0, 3))

    pts = np.vstack(chunks)
    if voxel_size > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = pcd.voxel_down_sample(voxel_size)
        pts = np.asarray(pcd.points)
    return pts


def estimate_covs(pts, knn):
    """kNN 邻域 PCA 估计逐点协方差，返回 N×3×3。"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    param = o3d.geometry.KDTreeSearchParamKNN(knn=knn)
    pcd.estimate_covariances(param)
    covs = np.asarray(pcd.covariances)
    if covs.shape != (len(pts), 3, 3):
        # 点太少时 estimate_covariances 可能不写满，退化为各向同性
        covs = np.tile(np.eye(3) * 1e-3, (len(pts), 1, 1))
    covs = np.nan_to_num(covs, nan=0.0)
    return covs


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


def workspace_root():
    """Support tools/ and src/tools/ without assuming a fixed parent depth."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/glim/src/glim/util/export_factors.cpp").is_file():
            return parent
    raise FileNotFoundError("Cannot locate workspace src/glim/src/glim/util/export_factors.cpp")


def build_graph_writer(force=False):
    """按需编译 tools/glim_graph_writer.cpp，返回可执行文件路径。

    需要一并编入 glim 的 export_factors.cpp —— 它注册了各因子的
    BOOST_CLASS_EXPORT_GUID，缺了会在序列化时抛 "unregistered class"。
    """
    import subprocess
    tools = Path(__file__).resolve().parent
    repo = workspace_root() / "src/glim"
    src = tools / "glim_graph_writer.cpp"
    exports = repo / "src/glim/util/export_factors.cpp"
    exe = tools / "glim_graph_writer"

    if not force and exe.exists() and exe.stat().st_mtime >= max(src.stat().st_mtime, exports.stat().st_mtime, Path(__file__).stat().st_mtime):
        return exe

    if not exports.exists():
        sys.exit(f"找不到 {exports}（需要 glim 源码树）")

    cmd = ["g++", "-std=c++17", "-O2", str(src), str(exports), "-o", str(exe),
           "-I" + str(repo / "include"),
           "-I/usr/local/include", "-isystem", "/usr/include/eigen3",
           "-L/usr/local/lib", "-lgtsam", "-lgtsam_unstable", "-lgtsam_points",
           "-lboost_serialization", "-Wl,-rpath,/usr/local/lib"]
    print("编译 graph_writer …")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr, file=sys.stderr)
        sys.exit("glim_graph_writer 编译失败（需要 g++ / GTSAM / gtsam_points）")
    return exe


def write_graph(out, subs, helper):
    """写 graph.bin / values.bin —— 结构与 GlobalMapping::recover_graph() 一致。"""
    import subprocess
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for s in subs:
            qx, qy, qz, qw = rot2quat(s["T_world_origin"][:3, :3])
            t = s["T_world_origin"][:3, 3]
            f.write("%.17g %.17g %.17g %.17g %.17g %.17g %.17g\n"
                    % (t[0], t[1], t[2], qx, qy, qz, qw))
        poses_file = f.name
    # 必须把 /usr/local/lib 前置：本机 ROS Humble 也装了 GTSAM 4.2，
    # 会通过继承来的 LD_LIBRARY_PATH 抢占 libgtsam.so.4，导致符号缺失。
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/usr/local/lib:" + env.get("LD_LIBRARY_PATH", "")
    try:
        r = subprocess.run([str(helper), str(out), poses_file],
                           capture_output=True, text=True, env=env)
        if r.returncode != 0:
            print(r.stderr or r.stdout, file=sys.stderr)
            sys.exit("写 graph.bin/values.bin 失败")
        print(r.stdout.strip())
    finally:
        os.unlink(poses_file)


def default_config_dir():
    """优先用 install 里那份（与包内一致），退回源码目录。"""
    ws = workspace_root()
    repo = ws / "src/glim"
    for c in (ws / "install/glim/share/glim/config", repo / "config"):
        if c.is_dir():
            return c
    return None


def strip_json_comments(text):
    """去掉 JSON 里的 // 与 /* */ 注释（GLIM 的 config 是带注释的 JSON）。

    带字符串状态机，避免误伤字符串里的 "//"（例如 URL）。
    """
    out, i, n = [], 0, len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:      # 转义，跳过下一个字符
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
        elif c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            i += 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def write_config(out, src):
    """把 GLIM 的 config 拷进 <out>/config，并把 config_path 指向自身。

    offline_viewer 用 GlobalConfig::instance(<map>/config, true) 加载，
    该函数会用这个路径覆盖 config_path，所以目录必须自包含。
    """
    import json
    dst = (out / "config").resolve()
    dst.mkdir(exist_ok=True)
    for f in Path(src).glob("*.json"):
        (dst / f.name).write_text(f.read_text())

    cfg = dst / "config.json"
    if cfg.exists():
        try:
            data = json.loads(strip_json_comments(cfg.read_text()))
            data.setdefault("global", {})["config_path"] = str(dst)
            cfg.write_text(json.dumps(data, indent=2))
        except json.JSONDecodeError as e:
            print(f"  ⚠ config.json 解析失败（{e}），原样保留", file=sys.stderr)
    return dst


def main():
    ap = argparse.ArgumentParser(
        description="PCD 序列 + TUM 位姿 → GLIM dump",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例：\n  python3 src/tools/lio2glim.py --poses poses.txt "
               "--clouds clouds/ --out /tmp/dump_out")
    ap.add_argument("--poses", required=True, help="TUM 格式位姿文件")
    ap.add_argument("--clouds", required=True,
                    help="PCD 目录或 glob（按文件名排序，与位姿行 1:1）")
    ap.add_argument("--out", required=True, help="输出 dump 目录")
    ap.add_argument("--frames-per-submap", type=int, default=50,
                    help="每个子图的帧数（默认 50）")
    ap.add_argument("--voxel-size", type=float, default=0.5,
                    help="子图内体素降采样边长，0 = 不降采样（默认 0.5 m）")
    ap.add_argument("--knn", type=int, default=10, help="协方差估计的 kNN（默认 10）")
    ap.add_argument("--config", default=None,
                    help="GLIM config 目录，拷进 <out>/config（默认自动从工作区查找）")
    ap.add_argument("--no-config", action="store_true", help="不写 config 目录")
    ap.add_argument("--no-graph", action="store_true",
                    help="不写 graph.bin/values.bin（仅调试用，正常加载会失败）")
    a = ap.parse_args()

    # 点云列表
    if os.path.isdir(a.clouds):
        cloud_paths = sorted(glob(os.path.join(a.clouds, "*.pcd")))
        cloud_paths += sorted(glob(os.path.join(a.clouds, "*.PCD")))
    else:
        cloud_paths = sorted(glob(a.clouds))
    if not cloud_paths:
        sys.exit(f"在 {a.clouds} 下没找到 .pcd")

    stamps, poses = read_tum(a.poses)
    if len(poses) == 0:
        sys.exit(f"{a.poses} 里没有有效位姿")

    n = min(len(poses), len(cloud_paths))
    if len(poses) != len(cloud_paths):
        print(f"⚠ 位姿 {len(poses)} 条 vs 点云 {len(cloud_paths)} 个，取前 {n} 个配对",
              file=sys.stderr)
    stamps, poses, cloud_paths = stamps[:n], poses[:n], cloud_paths[:n]

    # Fail before processing point clouds if the helper cannot be built.
    helper = build_graph_writer() if not a.no_graph else None
    subs = build_submaps(stamps, poses, cloud_paths, a.frames_per_submap)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    print(f"共 {n} 帧 → {len(subs)} 个子图，输出到 {out}")

    T_lidar_imu = np.eye(4)   # 点云已世界系，无需外参
    total_pts = 0
    for sub in subs:
        d = out / ("%06d" % sub["id"])
        d.mkdir(exist_ok=True)

        pts = load_and_merge(sub, a.voxel_size)
        if len(pts) == 0:
            print(f"  ⚠ 子图 {sub['id']:06d} 无点云", file=sys.stderr)

        covs = estimate_covs(pts, a.knn) if len(pts) else np.zeros((0, 3, 3))
        write_points_bin(d / "points_compact.bin", pts)
        write_covs_bin(d / "covs_compact.bin", covs)
        write_data_txt(d / "data.txt", sub, T_lidar_imu)
        (d / "imu_rate.txt").write_text("")
        total_pts += len(pts)
        print(f"  子图 {sub['id']:06d}: {len(sub['frames'])} 帧 → {len(pts)} 点")

    # graph.txt：matching_cost 留空，合并时用界面上的
    # 「Find overlapping submaps」跨会话建约束
    (out / "graph.txt").write_text(
        "num_submaps: %d\n"
        "num_all_frames: %d\n"
        "num_matching_cost_factors: 0\n" % (len(subs), n)
    )

    if not a.no_graph:
        write_graph(out, subs, helper)

    if not a.no_config:
        cfg_src = Path(a.config) if a.config else default_config_dir()
        if cfg_src and cfg_src.is_dir():
            write_config(out, cfg_src)
            print(f"config 取自 {cfg_src}")
        else:
            print("⚠ 没找到 GLIM config 目录，offline_viewer 可能无法打开",
                  file=sys.stderr)

    print(f"\n完成：{len(subs)} 个子图，{total_pts} 点")
    print("打开验证：")
    print("  source /opt/ros/humble/setup.bash && source install/setup.bash")
    print("  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH")
    print(f"  ros2 run glim_ros offline_viewer {out}")


if __name__ == "__main__":
    main()
