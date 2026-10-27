# SuperLIO + GLIM 多会话地图合并

本工作区将 Unitree L1 数据分别交给 SuperLIO 建图和闭环优化，再把每段优化后的关键帧点云与位姿转换为 GLIM `dump`，最后在 GLIM `Offline Viewer` 中配准并合并。


## 工作区内容

```bash
  SuperLIO_GLIM_ROS2/
  ├── src/
  │   ├── SuperLIO_SLAM/  # SuperLIO、STD 回环检测、ICP 和 iSAM2 后端
  │   ├── glim/           # GLIM v1.2.2
  │   └── glim_ros2/      # GLIM ROS 2 节点与 offline_viewer
  └── tools/
      ├── export_superlio.py
      ├── lio2glim.py
      ├── glim2lio.py       # 反向：dump → PCD + TUM（见第 5 节）
      ├── glim_graph_writer.cpp
      └── fix_dump_boost19.py
```

## 数据集

Unitree 数据集下载地址：

[Google Drive：Unitree 数据集](https://drive.google.com/drive/folders/1hCyqttluoQbxpVullFv-Bdlff1dptu37?usp=drive_link)

下载后确认有 `firstpart`、`secondpart` 两个 ROS 2 bag 目录（目录内含 `metadata.yaml` 和 SQLite `db3` 文件），放到：

```
/home/ros/dataset/unitree/
├── firstpart/
└── secondpart/
```

两段数据包含 `/unilidar/cloud` 和 `/unilidar/imu`，时间范围约重叠 130 秒。README 中的命令按上述数据位置编写；若保存位置不同，替换对应路径。

## 环境要求与构建
当前配置针对 Ubuntu 22.04、ROS 2 Humble、Unitree UniLiDAR L1。需要安装项目依赖、CUDA 13.1，以及 GLIM 使用的 `GTSAM`、`gtsam_points` 和 `Iridescence`。SuperLIO 与 GLIM 的依赖说明分别见 `src/SuperLIO_SLAM/README.md`、`src/glim/docs/installation.md` 或者 `src/glim/docs/quickstart.md`。

本机有两套 GTSAM：ROS Humble 的 4.2 和 `/usr/local` 的 4.3a1。GLIM 与 gtsam_points 需要使用 4.3a1；构建和运行时都要避免 ROS 的 4.2 抢占。

安装源
```bash
sudo apt install curl gpg
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
```

安装依赖库
```bash
sudo apt update
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

# Choose one of the follows
sudo apt install -y libgtsam-points-dev           # without CUDA
sudo apt install -y libgtsam-points-cuda12.2-dev  # with CUDA 12.2 (Ubuntu 22.04 only)
sudo apt install -y libgtsam-points-cuda12.6-dev  # with CUDA 12.6
sudo apt install -y libgtsam-points-cuda13.1-dev  # with CUDA 13.1
```


注意不要安装 ros2 humble 的 gtsam 库，它是 4.2 版本的（安装了也有方法转换，见下面），需要自己编译，查看下方命令，安装 ros2 依赖：
```bash
# Install dependencies
sudo apt install libomp-dev libboost-all-dev libmetis-dev \
                 libfmt-dev libspdlog-dev \
                 libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install

# Make shared libraries visible to the system
sudo ldconfig
```

开始编译代码
```bash
cd /home/ros/ros_ws/SLAM_Merge_ws
source /opt/ros/humble/setup.bash

# 如果安装了 ros2 humble 的 gtsam4.2 就带上这个
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  


MAKEFLAGS="-j4" colcon build --symlink-install \
--cmake-args \
-DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
-DGTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM_UNSTABLE
```

每个新终端运行 GLIM 前：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SLAM_Merge_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

可用以下命令确认实际加载的 GTSAM：

```bash
ldd /home/ros/ros_ws/SLAM_Merge_ws/install/glim/lib/libglim.so | grep libgtsam.so.4
```

输出应指向 `/usr/local/lib/libgtsam.so.4`。

## 1. 分别运行 SuperLIO

每段数据单独启动一次 SuperLIO 和 `loop backend`，避免两个会话的关键帧编号和图状态混在一起。录制 `/lio_slam/keyframe` 与 `/lio_slam/path`；后者会持续发布优化路径，导出器使用结果 bag 中最后一条路径。

终端 A，启动 Unitree L1 配置：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SLAM_Merge_ws/install/setup.bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  use_sim_time:=true rviz:=true
```

终端 B，录制 `firstpart` 的 SuperLIO 输出：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SLAM_Merge_ws/install/setup.bash
mkdir -p /home/ros/dataset/unitree/superlio_results
ros2 bag record \
  -o /home/ros/dataset/unitree/superlio_results/firstpart_result \
  /lio_slam/keyframe /lio_slam/path
```

终端 C，回放 `firstpart`：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/ros/dataset/unitree/firstpart --clock --rate 0.5
```

回放结束后等待后端处理完最后关键帧、路径停止更新。停止 bag record，再停止 launch。

重新执行终端 A、B、C，录制目录与输入 bag 换成 `secondpart`：

```bash
ros2 bag record \
  -o /home/ros/dataset/unitree/superlio_results/secondpart_result \
  /lio_slam/keyframe /lio_slam/path
```

```bash
ros2 bag play /home/ros/dataset/unitree/secondpart --clock --rate 0.5
```

不要在同一个后端进程中连续回放两段数据。

## 2. 导出优化后的 PCD 和 TUM 轨迹

导出器读取关键帧消息和最终优化路径，按纳秒时间戳匹配。关键帧点云在 IMU 局部系；脚本用对应的最终 map→IMU 位姿把点云变换到 map 系，并同时写出 TUM 位姿。

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SLAM_Merge_ws/install/setup.bash
```

```bash
python3 /home/ros/ros_ws/SLAM_Merge_ws/tools/export_superlio.py \
  --bag /home/ros/dataset/unitree/superlio_results/firstpart_result \
  --out /home/ros/dataset/unitree/superlio_results/firstpart_export
```

```bash
python3 /home/ros/ros_ws/SLAM_Merge_ws/tools/export_superlio.py \
  --bag /home/ros/dataset/unitree/superlio_results/secondpart_result \
  --out /home/ros/dataset/unitree/superlio_results/secondpart_export
```

输出结构：

    firstpart_export/                 secondpart_export/
    ├── clouds/                        ├── clouds/
    │   ├── 000000.pcd                 │   ├── 000000.pcd
    │   └── ...                        │   └── ...
    ├── poses.txt                      ├── poses.txt
    └── export_manifest.json           └── export_manifest.json

`poses.txt` 每行格式为 `timestamp x y z qx qy qz qw`，与同编号 PCD 一一对应。脚本不覆盖已有输出目录；若需要重导出，先将旧输出改名或移走。检查 `export_manifest.json` 中 `exported` 数量等于 `clouds/` 文件数，且没有遗漏的关键帧。

之前处理的结果为 `firstpart` 261 帧、234,484 点；`secondpart` 317 帧、267,668 点。重新运行后数量可能因代码、配置或数据变化而不同。

## 3. 转换为 GLIM `dump`

`lio2glim.py` 将 map 系 PCD 和位姿切成子图，计算协方差并生成 GLIM `dump`。首次运行会编译 `glim_graph_writer` 辅助程序，需要 `g++`、`GTSAM` 和 `gtsam_points` 开发文件。两段要分别生成 dump：

```bash
python3 /home/ros/ros_ws/SLAM_Merge_ws/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/firstpart_export/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/firstpart_export/clouds \
  --out /home/ros/dataset/unitree/superlio_results/firstpart_dump \
  --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/ros_ws/SLAM_Merge_ws/src/glim/config
```

```bash
python3 /home/ros/ros_ws/SLAM_Merge_ws/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/secondpart_export/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/secondpart_export/clouds \
  --out /home/ros/dataset/unitree/superlio_results/secondpart_dump \
  --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/ros_ws/SLAM_Merge_ws/src/glim/config
```

转换完成后，每个 `dump` 都应有 `graph.txt`、`graph.bin`、`values.bin`、`config/` 和按编号排列的子图目录。子图中应有 `data.txt`、`points_compact.bin`、`covs_compact.bin`。不要对已有的非空输出目录直接重跑；改用新的输出目录，或先备份旧目录。

此前出现过工具目录调整后辅助程序路径仍按旧布局计算，造成只有子图、没有 graph.bin/values.bin/config 的半成品。当前仓库里的工具应从工作区 `tools/` 目录调用。若日志报找不到 `export_factors.cpp`，说明运行的不是此工作区中的最新 `lio2glim.py`。

## 4. 在 GLIM 中合并

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SLAM_Merge_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

```bash
ros2 run glim_ros offline_viewer \
  /home/ros/dataset/unitree/superlio_results/firstpart_dump
```

在界面中：

1. 首次加载 `firstpart` 时启用优化。
2. File → Open Additional Map，加载 `secondpart_dump`。
3. 点击 `Merge sessions`。运行全局粗配准；如果 `source` 与 `target` 没对齐，手动把绿色 source 调到红色 target 的重叠区域。
4. 点击 `Run fine registration` 执行 ICP/GICP。确认点云结构、方向和重叠区域贴合；`inlier rate` 只是参考。
5. 点击 `Create Factor` 创建跨会话锚点约束。
6. 点击 `Find overlapping submaps`，创建重叠子图之间的 VGICP 匹配代价因子。
7. 点击 `Optimize` 数次，确认地图在重叠区域保持对齐，再保存为新的合并 dump，保留两个输入 dump。

成功检查：优化过程中没有 `xN does not exist in the Values` 或 `out_of_range`；两个 `session` 的点云同时显示，重叠区域贴合；Find overlapping submaps 能找到候选并完成优化。

结果可视化

|点云图一|点云图二|地图合并|
|---|---|---|
|![alt text](images/image.png)|![alt text](images/image-1.png)|![alt text](images/image-2.png)|



## 5. 反向导出：`dump` → PCD + TUM（`glim2lio.py`）

`glim2lio.py` 是 `lio2glim.py` 的逆过程：把 GLIM `dump` 转回通用的「PCD 序列 + TUM 位姿」，便于用其它工具查看、重采样，或经 `lio2glim.py` 再转回 `dump` 形成闭环。子图内点云存的是 origin 系（`T_world_origin^-1 @ p_world`），脚本按 `p_world = T_world_origin @ p_origin` 还原到世界系；`data.txt` 里的逐帧 `T_world_lidar` 已是世界系位姿，直接沿用。

```bash
python3 /home/ros/ros_ws/SuperLIO_GLIM_ROS2/tools/glim2lio.py \
  --dump /home/ros/dataset/unitree/superlio_results/firstpart_dump \
  --out  /home/ros/dataset/unitree/superlio_results/firstpart_reverse
```

输出结构：

    firstpart_reverse/
    ├── clouds/000000.pcd ...         # 每个子图一张世界系点云，与 poses.txt 1:1
    ├── poses.txt                     # TUM：每个子图一行（取 T_world_origin）
    ├── frames.txt                    # TUM：dump 内每一帧一行（T_world_lidar）
    └── glim2lio_manifest.json

`clouds/` 与 `poses.txt` 按子图一一对应，可直接喂回 `lio2glim.py`；`frames.txt` 保留 dump 里的完整帧轨迹（帧数多于子图数，不与点云对应）。GLIM 已把每个子图内的帧合并、降采样成一张点云，逐帧点云无法恢复，所以点云只按子图输出。

### 5.1 同时保存全局点云

加 `--global-cloud`，会把所有子图按 `T_world_origin` 变换到世界系后拼成一整幅地图，写成 `<out>/merged.pcd`：

```bash
python3 /home/ros/ros_ws/SuperLIO_GLIM_ROS2/tools/glim2lio.py \
  --dump /home/ros/dataset/unitree/superlio_results/firstpart_dump \
  --out  /home/ros/dataset/unitree/superlio_results/firstpart_reverse \
  --global-cloud --global-voxel-size 0.2
```

`merged.pcd` 是「所有子图点云求并集」，只用于整体查看——一张图没有位姿，不能喂回 `lio2glim.py`。子图点云与全局点云各自独立降采样：`--voxel-size` 控制 `clouds/`，`--global-voxel-size` 控制 `merged.pcd`（缺省沿用 `--voxel-size`，`0` = 不降采样）。地图很大时先给 `--global-voxel-size` 一个较大值再跑，可降低内存占用。

### 5.2 常用参数与闭环

| 参数 | 作用 |
|---|---|
| `--dump <dir>` | 输入：GLIM `dump` 目录 |
| `--out <dir>` | 输出目录（不得已存在，防覆盖） |
| `--voxel-size 0.3` | 子图点云体素降采样，默认 `0` = 不降采样（不需要 open3d） |
| `--ascii` | 写 ASCII PCD，默认二进制 |
| `--global-cloud` | 额外写全局点云 `<out>/merged.pcd` |
| `--global-voxel-size 0.2` | 全局点云单独的体素降采样 |
| `--force` | 覆盖已存在的输出目录（先删旧目录） |

闭环回 `dump`：把 `clouds/` + `poses.txt` 交给 `lio2glim.py` 即可（两者子图数一致）：

```bash
python3 /home/ros/ros_ws/SuperLIO_GLIM_ROS2/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/firstpart_reverse/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/firstpart_reverse/clouds \
  --out /tmp/firstpart_round --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/ros_ws/SuperLIO_GLIM_ROS2/src/glim/config
```

检查转换是否正确：用 open3d 打开 `merged.pcd`（或任一子图点云），点云应连成整条轨迹，而不是挤成一团；挤成一团说明坐标方向反了。

> 已知问题：本仓库把 glim 嵌套在 `src/glim/` 下，而 `lio2glim.py` 按 `src/glim/util/export_factors.cpp` 找源码，闭环那一步会报 `FileNotFoundError`。反向导出本身不受影响（可用 open3d 直接核对点云与位姿）。

## 故障排查

- `libgtsam_points_cuda.so.1` 报 `undefined symbol ... rekey`：当前终端的 `LD_LIBRARY_PATH` 没有把 `/usr/local/lib` 放在前面。
- `lio2glim.py` 报找不到 `export_factors.cpp`：确认运行的是本工作区 `tools/lio2glim.py`，且辅助 graph writer 源文件和 GLIM 源码都在仓库中。
- 优化时报 `xN does not exist in the Values`：对应 `dump` 的 `values.bin` 缺失、无效或不含该子图 pose。检查 `graph.bin`、`values.bin` 是否存在且非空；两个 `session` 的子图数量应与各自 values 中的 X pose 数一致。
- 导出器报最终轨迹缺少关键帧：先让 SuperLIO 后端处理完整并录制完最后一条路径，再重新导出。
- 配准分数看起来不错但地图仍错位：回到 `Merge sessions` 调整 `source` 粗对齐，再运行 `fine registration`。


## 项目整理与第三方依赖

本项目采用同一工作区的两阶段流程：

```text
SuperLIO-SLAM 在线建图与回环优化
    → 导出各会话的优化关键帧和位姿
    → 转换为 GLIM dump
    → GLIM Offline Viewer 配准、优化与合并子图
```

`src/SuperLIO_SLAM`、`src/glim`、`src/glim_ros2` 和 `tools` 都属于本项目，应一起保留。
TEASER++ 上游源码位于 `src/SuperLIO_SLAM/third_party/TEASER-plusplus`，通过 Git submodule 固定版本。
克隆后执行 `git submodule update --init --recursive`；具体说明见
[src/SuperLIO_SLAM/third_party/README.md](src/SuperLIO_SLAM/third_party/README.md)。
TEASER++ 已接入可选在线模式 `std_teaser_gicp`：单帧 STD 检索 → deque 多帧子图 FPFH → TEASER++ → GICP → iSAM2。默认仍为旧 `std_icp` 模式。构建前执行 `python3 src/tools/build_teaserpp.py --jobs 1`，运行时增加 `loop_registration_mode:=std_teaser_gicp`。详见 [SuperLIO 包说明](src/SuperLIO_SLAM/README.md)。

2026-10-27 整理后，仅 Funny 参考源码和实验结果 `test/`、`savemap/`、`liomap/`
保留在相邻 `../SuperLIO_GLIM_ROS2_archive_20261027/`，没有删除数据。
该目录的 `moves.json` 标明哪些项目已经恢复。包内 `src/SuperLIO_SLAM/test/` 仍是在线测试代码。
构建产物 `build/`、`install/`、`log/` 不进入源码版本管理。

在线回环后端现已拆分关键帧/STD、配准和地图重建流水线。检测任务有界排队，
过载淘汰旧检测任务，保留已接收的关键帧和里程计图节点；地图使用最新待处理快照。
实现与性能日志说明见 [异步回环后端](docs/async_loop_backend.md)。
