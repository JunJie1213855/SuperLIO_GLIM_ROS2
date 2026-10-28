# SuperLIO + GLIM 多会话地图合并

本工作区将 Unitree L1 数据分别交给 SuperLIO 建图和闭环优化，再把每段优化后的关键帧点云与位姿转换为 GLIM `dump`，最后在 GLIM `Offline Viewer` 中配准并合并。


## 工作区内容

```bash
  SuperLIO_GLIM_ROS2/
  ├── src/
  │   ├── SuperLIO_SLAM/  # SuperLIO、距离/STD 检索、GICP/TEASER++、iSAM2
  │   ├── glim/           # GLIM v1.2.2
  │   └── glim_ros2/      # GLIM ROS 2 节点与 offline_viewer
  └── tools/
      ├── build_teaserpp.py  # 构建并安装固定版本 TEASER++
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

本机工作区位于 `/home/ros/rosws/SuperLIO_GLIM_ROS2`；以下命令均从工作区根目录执行。
Livox 驱动工作区为 `/home/ros/rosws/livox_ros_ws`，换机器后按实际位置调整。

先构建 TEASER++，再编译 ROS 包。`SUPERLIO_WITH_TEASER` 默认开启，
默认回环为 `std` 候选检索 + `std_teaser_gicp` 配准，因此需要 TEASER++。
仅初始化源码子模块不会生成 CMake 所需的安装配置。

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash

git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1

# 保证 GLIM 使用构建时选定的 /usr/local GTSAM
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential \
  --cmake-args \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM_UNSTABLE \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

构建脚本位于 `tools/build_teaserpp.py`，不是 `src/tools/`。它从脚本位置定位
工作区，在 `build/teaserpp_vendor` 缓存固定版本依赖，在 `build/teaserpp` 编译，
无需 sudo 即安装到 `install/teaserpp`。首次运行需要联网；缓存完整后可加 `--offline`。
安装成功后应存在：

```text
install/teaserpp/lib/cmake/teaserpp/teaserppConfig.cmake
```

若 GLIM 已编译，只需修复或重编 SuperLIO，可将上方 colcon 命令改为：

```bash
MAKEFLAGS=-j1 colcon build --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

若暂时只使用直接 GICP，可以跳过 TEASER++ 构建脚本，并将 colcon 的选项改为
`-DSUPERLIO_WITH_TEASER=OFF`。此时启动必须显式指定 `loop_registration_mode:=std_gicp`，不能运行默认的 `std_teaser_gicp`。
之后启用新模式时，先构建依赖，再显式传入 `-DSUPERLIO_WITH_TEASER=ON`，避免沿用旧 CMake 缓存。

2026-10-28 本机验证：TEASER++ 构建安装、上述 `super_lio_slam` 单包构建及
`submap_registration_test`（1/1）通过；此记录不代表本次重新编译了 GLIM 或完成实测 bag 精度评估。
迁移前的额外在线合成测试未全部通过：默认四帧配置被粗配准门限拒绝，两帧实验配置后续因 ROS 发现超时未进入配准。
详细结果已记录在 [包说明的验证章节](src/SuperLIO_SLAM/README.md#validation-and-limitations)，不将编译通过等同于在线回环测试通过。
2026-10-28 GICP 与默认流程迁移后的验证：单包构建和 CTest（1/1）通过。
使用已安装的 Cyclone DDS、独立 ROS domain 运行在线合成测试，STD → 时间/距离筛选 →
TEASER++ → GICP 正例、严格 30 s 边界拒绝、STD 空间半径拒绝和 STD + 直接 GICP
含旋转正例均通过。默认质量门限正例使用 10 s 的合成关键帧间隔，因此 6 s 子图窗口
在该用例内裁剪为单帧；它验证的是判决链路，不证明默认四帧强漂移场景或真实 bag 精度。

测试前 source 上述工作区，并为直接运行的测试程序设置依赖库：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:/home/ros/rosws/SuperLIO_GLIM_ROS2/install/teaserpp/lib:${LD_LIBRARY_PATH:-}
ctest --test-dir build/super_lio_slam -R submap_registration_test --output-on-failure
```

每个新终端运行 GLIM 前：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/SuperLIO_GLIM_ROS2/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

可用以下命令确认实际加载的 GTSAM：

```bash
ldd /home/ros/rosws/SuperLIO_GLIM_ROS2/install/glim/lib/libglim.so | grep libgtsam.so.4
```

输出应指向 `/usr/local/lib/libgtsam.so.4`。

## 1. 分别运行 SuperLIO

每段数据单独启动一次 SuperLIO 和 `loop backend`，避免两个会话的关键帧编号和图状态混在一起。录制 `/lio_slam/keyframe` 与 `/lio_slam/path`；后者会持续发布优化路径，导出器使用结果 bag 中最后一条路径。

终端 A，启动 Unitree L1 配置：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/SuperLIO_GLIM_ROS2/install/setup.bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  use_sim_time:=true rviz:=true
```

终端 B，录制 `firstpart` 的 SuperLIO 输出：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/SuperLIO_GLIM_ROS2/install/setup.bash
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

### 1.1 不同回环方式的启动命令

默认流程已经统一为：

```text
STD 描述子检索历史候选
  → 时间差 > 30 s 且原始里程计位置距离 ≤ 15 m
  → 构建当前/历史局部子图，FPFH 对应 + TEASER++ 粗配准
  → 粗配准内点数、比例、残差 RMSE 检查
  → GICP 精配准及距离、误差、覆盖率、退化检查
  → 通过后插入 iSAM2 回环因子，更新优化轨迹与地图
```

STD 在默认流程中负责候选检索；三角形位姿提示可用时辅助 FPFH，
不可用时仍允许无提示粗配准。候选的时间和空间门限在检索后、后台执行前及最终提交时检查。
空间距离统一比较原始 LIO 位姿，不混用优化位姿。

| 判决阶段 | 默认门限及含义 |
| --- | --- |
| 候选时间 | `loop_min_time_separation_s: 30.0`，时间差严格大于 30 s |
| 候选空间距离 | `loop_candidate_radius_m: 15.0`，位置距离不超过 15 m，STD 和距离检索都生效 |
| TEASER++ 内点距离 | `coarse_residual_threshold_m: 0.6`，对应点残差不超过 0.6 m 计为粗内点 |
| TEASER++ 内点质量 | 最少 6 个内点、比例至少 0.3、内点 RMSE 不超过 0.3 m |
| GICP 对应距离 | `gicp_max_correspondence_distance_m: 2.0`，求解时最大对应距离 |
| GICP 独立质量距离 | `quality_nn_radius_m: 1.0`，最近邻距离不超过 1 m 计入质量内点 |
| GICP 质量检查 | 内点 MSE 严格小于 0.1 m²；源子图和当前帧覆盖率都严格大于 80%；至少 100 个内点 |
| GICP 修正及退化 | 相对初值修正不超过 5 m / 20°，信息特征值比至少 `1e-5`；还需收敛且为有效刚体变换 |

粗配准的 RMSE 单位是 m，精配准的 MSE 单位是 m²；这些是几何残差，
不是相对于真值的轨迹误差。GICP 的接受条件是内点 MSE <0.1 m² 且两项覆盖率都 >0.8；
恰好 0.1 m² 或恰好 80% 都拒绝，不再增加独立的“分数 >0.8”条件。
PCL 的 `getFitnessScore()` 是越小越好的均方距离，并非越大越好的 0–1 置信度。
这里的内点 MSE 只统计质量半径内的最近邻点对，覆盖率为内点数除以参与检查的点数。
候选筛选后仍受重复回环冷却和源/目标子图不共享成员等约束。
本次收紧门限后，单包编译、配准单测和两条 GICP 路径的在线合成测试均通过。
边界单测确认误差等于上限、源子图覆盖率恰好 80%、当前帧覆盖率恰好 80% 都会拒绝。
下面命令保留四种组合，供对照或排查使用。

候选检索和点云配准由两个独立参数选择；`std_gicp` / `std_teaser_gicp`
是配准模式名称，前缀 `std_` 不会强制使用 STD 检索。

| 方式 | `loop_candidate_mode` | `loop_registration_mode` |
| --- | --- | --- |
| 距离检索 + GICP（对照方式） | `distance` | `std_gicp` |
| STD 检索 + GICP | `std` | `std_gicp` |
| 距离检索 + 多帧 TEASER++ / GICP | `distance` | `std_teaser_gicp` |
| STD 检索 + 多帧 TEASER++ / GICP（当前默认） | `std` | `std_teaser_gicp` |

先在终端 A 加载环境：

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
source install/setup.bash
```

以下命令任选一条，替换前面的终端 A 启动命令；终端 B 的录制与终端 C 的
bag 回放步骤不变。命令针对 Unitree L1 的 bag，统一使用模拟时间并启动 RViz。
切换方式时停止当前 launch，再启动新的方式；已用 `SUPERLIO_WITH_TEASER=ON`
构建时，四种方式之间切换不需要重新编译。

**距离检索 + GICP（对照方式）**

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_gicp use_sim_time:=true rviz:=true
```

**STD 检索 + GICP**

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_gicp \
  use_sim_time:=true rviz:=true
```

**距离检索 + 多帧 TEASER++ / GICP**

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true rviz:=true
```

**STD 检索 + 多帧 TEASER++ / GICP**

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_teaser_gicp \
  use_sim_time:=true rviz:=true
```

原 `std_icp` 回环模式已移除；直接精配准使用 `std_gicp`。默认组合为 `std` + `std_teaser_gicp`。
两条路径的精配准都调用 GICP，`std_teaser_gicp` 额外执行多帧子图构建和 TEASER++ 粗配准。
共用的 `registration_*`、`gicp_*` 和 `quality_*` 参数在 `registration_config` 指定的 YAML 中，
直接 GICP 的历史子图邻域由 `slam_config` 内的 `gicp_submap_half_width` 控制。
STD + 直接 GICP 会分别尝试 STD 和原始里程计初值，两次使用相同质量门限，
按包含未匹配点惩罚的最近邻误差选择结果，以减少重复结构下错误局部最优的影响。
旧 `icp_*` 精配准参数已移除；STD 的 `icp_threshold` 是沿用名称的平面匹配分数门限，不调用 ICP。

两个 `std_teaser_gicp` 命令要求已安装 TEASER++ 并用
`-DSUPERLIO_WITH_TEASER=ON` 构建。距离模式不生成 STD 描述子；STD 模式要求
描述子匹配。两种检索都要经过配准验证和 iSAM2 才能形成回环，默认候选两端
传感器时间差必须严格大于 30 秒，连续接受回环还受关键帧冷却限制。

**关闭回环，仅运行 SuperLIO 前端**

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  enable_loop_backend:=false use_sim_time:=true rviz:=true
```

此时不启动回环后端，也不发布 SLAM 关键帧；RViz 自动使用前端配置，
因此不能沿用前面的 `/lio_slam/keyframe` + `/lio_slam/path` 导出流程。

### 1.2 切换传感器和实验配准配置

上面四种组合也适用于其他传感器入口，仅需替换 launch 文件名：

| 传感器 | launch 文件 |
| --- | --- |
| Unitree L1 | `unitree_slam.launch.py` |
| Velodyne VLP32 / UrbanNav | `vlp32.launch.py` |
| Livox MID-360 | `Livox_mid360.launch.py` |
| Robosense Airy | `robosense_airy.launch.py` |

例如 VLP32 使用距离检索和多帧配准：

```bash
ros2 launch super_lio_slam vlp32.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true rviz:=true
```

默认使用 `config/loop_teaser_gicp.yaml`。如需显式使用 UrbanNav 距离检索的实验配置：

```bash
ros2 launch super_lio_slam vlp32.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_teaser_gicp \
  registration_config:=/home/ros/rosws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/loop_distance_teaser_gicp.yaml \
  use_sim_time:=true rviz:=true
```

当前实验文件设置粗内点 RMSE 上限 0.4 m、最少粗内点 30，与默认粗配准参数不同。
两份配置的 GICP 门限统一为内点 MSE <0.1 m²、源子图和当前帧覆盖率都 >80%。
它只是可选配准参数，不代表精度已验证；VLP32 数据逐点时间及外参限制见
[包内 UrbanNav 说明](src/SuperLIO_SLAM/README.md#urbannav-2020-03-14-16-45-35-inspection)。
使用实时传感器时改为 `use_sim_time:=false`；使用 bag 时保留 `true`，并在回放终端使用 `--clock`。

## 2. 导出优化后的 PCD 和 TUM 轨迹

导出器读取关键帧消息和最终优化路径，按纳秒时间戳匹配。关键帧点云在 IMU 局部系；脚本用对应的最终 map→IMU 位姿把点云变换到 map 系，并同时写出 TUM 位姿。

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/SuperLIO_GLIM_ROS2/install/setup.bash
```

```bash
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/export_superlio.py \
  --bag /home/ros/dataset/unitree/superlio_results/firstpart_result \
  --out /home/ros/dataset/unitree/superlio_results/firstpart_export
```

```bash
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/export_superlio.py \
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
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/firstpart_export/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/firstpart_export/clouds \
  --out /home/ros/dataset/unitree/superlio_results/firstpart_dump \
  --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/rosws/SuperLIO_GLIM_ROS2/src/glim/config
```

```bash
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/secondpart_export/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/secondpart_export/clouds \
  --out /home/ros/dataset/unitree/superlio_results/secondpart_dump \
  --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/rosws/SuperLIO_GLIM_ROS2/src/glim/config
```

转换完成后，每个 `dump` 都应有 `graph.txt`、`graph.bin`、`values.bin`、`config/` 和按编号排列的子图目录。子图中应有 `data.txt`、`points_compact.bin`、`covs_compact.bin`。不要对已有的非空输出目录直接重跑；改用新的输出目录，或先备份旧目录。

此前出现过工具目录调整后辅助程序路径仍按旧布局计算，造成只有子图、没有 graph.bin/values.bin/config 的半成品。当前仓库里的工具应从工作区 `tools/` 目录调用。若日志报找不到 `export_factors.cpp`，说明运行的不是此工作区中的最新 `lio2glim.py`。

## 4. 在 GLIM 中合并

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/SuperLIO_GLIM_ROS2/install/setup.bash
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
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/glim2lio.py \
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
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/glim2lio.py \
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
python3 /home/ros/rosws/SuperLIO_GLIM_ROS2/tools/lio2glim.py \
  --poses /home/ros/dataset/unitree/superlio_results/firstpart_reverse/poses.txt \
  --clouds /home/ros/dataset/unitree/superlio_results/firstpart_reverse/clouds \
  --out /tmp/firstpart_round --frames-per-submap 10 --voxel-size 0.2 \
  --config /home/ros/rosws/SuperLIO_GLIM_ROS2/src/glim/config
```

检查转换是否正确：用 open3d 打开 `merged.pcd`（或任一子图点云），点云应连成整条轨迹，而不是挤成一团；挤成一团说明坐标方向反了。

> 已知问题：本仓库把 glim 嵌套在 `src/glim/` 下，而 `lio2glim.py` 按 `src/glim/util/export_factors.cpp` 找源码，闭环那一步会报 `FileNotFoundError`。反向导出本身不受影响（可用 open3d 直接核对点云与位姿）。

## 故障排查

- `Could not find ... teaserppConfig.cmake`：先在工作区根目录执行 `python3 tools/build_teaserpp.py --jobs 1`，再运行 colcon。若依赖安装在其他位置，传入 `-Dteaserpp_DIR=/实际安装位置/lib/cmake/teaserpp`。
- 构建脚本报找不到 `/home/ros/rosws/src/SuperLIO_SLAM/third_party/TEASER-plusplus`：旧脚本把工作区多向上取了一层。当前 `tools/` 布局应使用 `Path(__file__).resolve().parents[1]`，仓库脚本已修正。
- `glim_ros` 的 `ament_auto_package` scoped-header 提示只是头文件安装布局警告；该包显示 `Finished` 时，不是本次构建失败原因。
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
TEASER++ 已接入可选在线模式 `std_teaser_gicp`：距离或 STD 检索 → deque 多帧子图 FPFH → TEASER++ → GICP → iSAM2。候选检索默认 `std`，通过严格大于 30 s 的时间差和不超过 15 m 的原始里程计距离筛选后，默认进入 `std_teaser_gicp`；两项模式也可独立切换作对照。构建前执行 `python3 tools/build_teaserpp.py --jobs 1`，默认启动即使用该流程；显式命令为 `loop_candidate_mode:=std loop_registration_mode:=std_teaser_gicp`。详见 [SuperLIO 包说明](src/SuperLIO_SLAM/README.md)。

2026-10-27 整理后，仅 Funny 参考源码和实验结果 `test/`、`savemap/`、`liomap/`
保留在相邻 `../SuperLIO_GLIM_ROS2_archive_20261027/`，没有删除数据。
该目录的 `moves.json` 标明哪些项目已经恢复。包内 `src/SuperLIO_SLAM/test/` 仍是在线测试代码。
构建产物 `build/`、`install/`、`log/` 不进入源码版本管理。

在线回环后端现已拆分关键帧/STD、配准和地图重建流水线。检测任务有界排队，
过载淘汰旧检测任务，保留已接收的关键帧和里程计图节点；地图使用最新待处理快照。
实现与性能日志说明见 [异步回环后端](docs/async_loop_backend.md)。
