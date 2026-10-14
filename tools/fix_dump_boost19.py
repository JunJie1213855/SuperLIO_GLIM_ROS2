#!/usr/bin/env python3
"""
让本机 GLIM (Boost 1.74) 能打开 Boost>=1.76 生成的 dump。

背景：dump 里 graph.bin / values.bin 的 Boost 归档头记录的库版本是 19，
而 Ubuntu 22.04 的 Boost 1.74 只支持到 18，basic_binary_iarchive::init()
会直接抛 archive_exception::unsupported_version 并放弃。

做法：把版本字节 (偏移 0x1E) 从 0x13 改成 0x12。
此时解析会在正文深处失败并抛出 std::exception，正好落进
global_mapping.cpp 的第二个 catch，令 needs_recover=true，
GLIM 随即走 recover_graph()：
  - 位姿 X(i) 从各子图 data.txt 里的 T_world_origin 填回（几何正确）
  - 配准因子从子图体素图重建

注意：原始因子图（闭环/配准的历史记录）无法恢复，是重建而非还原。
"""
import argparse, shutil, sys, pathlib

VERSION_OFFSET = 0x1E
VER_19, VER_18 = 0x13, 0x12
TARGETS = ("graph.bin", "values.bin")


def patch(path, revert, dry):
    p = pathlib.Path(path)
    if not p.exists():
        print(f"  跳过（不存在）: {p}")
        return False
    b = bytearray(p.read_bytes())
    cur = b[VERSION_OFFSET]
    want = VER_19 if revert else VER_18
    if cur == want:
        print(f"  已是目标状态 ({cur:#04x}): {p.name}")
        return False
    if cur not in (VER_18, VER_19):
        print(f"  ⚠ 意外的版本字节 {cur:#04x}，跳过: {p.name}")
        return False
    if dry:
        print(f"  [dry-run] {p.name}: {cur:#04x} -> {want:#04x}")
        return True
    b[VERSION_OFFSET] = want
    p.write_bytes(b)
    print(f"  {p.name}: {cur:#04x} -> {want:#04x}")
    return True


def main():
    ap = argparse.ArgumentParser(description="修正 GLIM dump 的 Boost 归档版本字节")
    ap.add_argument("dumps", nargs="+", help="dump 目录")
    ap.add_argument("--revert", action="store_true", help="改回 19（还原原始文件）")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--backup", action="store_true", help="修改前备份为 *.bak")
    a = ap.parse_args()

    for d in a.dumps:
        d = pathlib.Path(d)
        print(f"{d}:")
        if not d.is_dir():
            print("  ⚠ 不是目录，跳过")
            continue
        for name in TARGETS:
            f = d / name
            if a.backup and f.exists() and not a.dry_run:
                bak = f.with_suffix(f.suffix + ".bak")
                if not bak.exists():
                    shutil.copy2(f, bak)
            patch(f, a.revert, a.dry_run)
    print("\n完成。之后用 offline_viewer 打开即可：")
    print("  source /opt/ros/humble/setup.bash && source install/setup.bash")
    print("  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH")
    print("  ros2 run glim_ros offline_viewer <dump目录>")


if __name__ == "__main__":
    sys.exit(main())
