# SuperLIO SLAM

ROS 2 LiDAR / IMU 前端、STD 回环后端与运动补偿重定位。完整运行命令见 [Quickstart](../../docs/quickstart.md)，项目概览见[工作区 README](../../README.md)。

## 当前实现

- 回环默认使用 **STD → 时间/距离筛选 → 多帧 FPFH / TEASER++ → GICP → iSAM2**；普通 ICP 已移除，直接 GICP 保留为对照。
- 关键帧/STD、异步配准、地图重建分开执行。队列过载只淘汰旧检测任务，不删除关键帧和图节点；地图请求合并为最新快照。
- 重定位在固定局部系累积经逐帧位姿补偿的点云，配准后变换最新 KF 状态和协方差，切换世界参考地图。
- 点云、位姿、关键帧和 TF 保留传感器时间。默认无需 `/clock`，旧时间对齐参数弃用并忽略。
- 室内/室外配准 YAML 独立；RViz 默认强度伪彩色、3 像素、Alpha 1，地图独立体素滤波。
- 日志对 `accepted` / `rejected` 和最终 `stage` 理由使用绿/红色，数值保持原色。

## 构建

在工作区根目录执行：

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/livox_ros/install/setup.bash
git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1

MAKEFLAGS=-j1 colcon build --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

构建助手按脚本位置定位工作区，在 `build/teaserpp_vendor` 缓存固定版本源码，安装到 `install/teaserpp`；
完整缓存可使用 `--offline`。缺少 `teaserppConfig.cmake` 时先运行该助手。
关闭 TEASER++ 的构建只能使用 `std_gicp`，不支持 TEASER++ 回环或当前重定位流程。

## 启动与配置

各终端先加载上述 ROS 和工作区环境，使用相同 DDS 配置；每个 bag 会话单独启动算法。

| 入口 | 传感器配置 | 默认配准配置 |
|---|---|---|
| `unitree_slam.launch.py` | `unitree_l1.yaml` | `loop_indoor_teaser_gicp.yaml` |
| `robosense_airy.launch.py` | `robosense_airy.yaml` | `loop_indoor_teaser_gicp.yaml` |
| `vlp32.launch.py` | `vlp32.yaml` | `loop_outdoor_teaser_gicp.yaml` |
| `Livox_mid360.launch.py` / `slam.launch.py` | `livox_360.yaml` | `loop_teaser_gicp.yaml` |

```bash
ros2 launch super_lio_slam robosense_airy.launch.py rviz:=true
# 另一个终端
ros2 bag play /mnt/d/dataset/SLAM/robosense/mapping
```

| 参数 | 含义 |
|---|---|
| `sensor_config` | 话题、点云字段/单位、外参、前端参数 |
| `slam_config` | 关键帧、STD、图优化与地图发布；Unitree 使用 `unitree_slam.yaml` |
| `registration_config` | TEASER++、GICP 和接受门限；可覆盖入口默认配置 |
| `enable_loop_backend` | 默认 true；false 同时关闭后端与关键帧发布 |
| `loop_candidate_mode` | `std`（默认）或 `distance` |
| `loop_registration_mode` | `std_teaser_gicp`（默认）或 `std_gicp` |
| `loop_candidate_radius_m` | 两种候选模式的原始里程计距离门限，默认 15 m |
| `use_sim_time` | 默认 false；需要 ROS 仿真时间时仍可与播放器 `--clock` 配套使用 |
| `rviz` / `rviz_config` | 是否启动及显示配置；关闭后端时自动选择 `lio.rviz` |
| `save_map_path` | 服务与退出保存的优化地图 PCD 路径 |
| `save_map_voxel_size` | 保存体素大小，默认 0.2 m；0 逐点流式写入 |
| `save_map_on_shutdown` | 正常退出自动保存；默认 false |

### 四种回环组合

```bash
# STD + TEASER++ + GICP（默认）
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_teaser_gicp rviz:=true
# 距离检索 + TEASER++ + GICP
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_registration_mode:=std_teaser_gicp rviz:=true
# STD + 直接 GICP
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_gicp rviz:=true
# 距离检索 + 直接 GICP
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_registration_mode:=std_gicp rviz:=true
```

候选方式与配准方式独立。`loop_distance_teaser_gicp.yaml` 只是显式选择的实验配置，不会自动启用距离检索。
两种检索均要求传感器时间差 **>30 s**、原始里程计距离 **≤15 m**；
还受关键帧冷却及源/目标子图不共享成员的限制。

STD 位姿提示是可选初值，提示失败后可重试无提示粗配准。源/目标子图分别在锚帧下按位姿累积，GICP 返回将当前帧点变换到历史帧的完整相对变换供 iSAM2 使用；坐标方向见[配准推导](../../docs/loop_pgo_path.md)。

### 配准门限

| 参数 | 室内 | 室外 |
|---|---:|---:|
| `registration_voxel_size_m` | 0.3 | 0.5 |
| `coarse_min_inliers` | 3 | 10 |
| `coarse_min_inlier_ratio` | 0.1 | 0.15 |
| `coarse_max_inlier_rmse_m` | 0.4 | 0.4 |
| `quality_min_source_overlap` | 0.9 | 0.9 |
| `quality_min_current_frame_overlap` | 0.9 | 0.9 |
| `quality_max_inlier_mse_m2` | 0.05 | 0.1 |

粗配准 RMSE 单位为 m，GICP 内点 MSE 单位为 m²，`RMSE = sqrt(MSE)`。
覆盖率按质量半径内的最近邻内点占比计算，子图和当前帧分别检查。
GICP 的覆盖率必须严格大于下限，MSE 必须严格小于上限；没有独立的“分数 >0.8”条件。
阈值以所选 YAML 为准，重定位使用独立配置。

共享检查还包括至少 100 个 GICP 内点、有效刚体变换、收敛、精修幅度和信息特征值比。
STD 内部保留的 `icp_threshold` 是历史命名的平面评分参数，不代表启用普通 ICP。

## 坐标系与时间

建图 TF：`map → world → imu`。前端在 world 系连续估计；后端发布 map 系优化轨迹和修正，
**回环不重置前端 ESKF**。

`map → world` 从 `/lio/odom`、`/lio/imu/odom` 获取最新数据时间，独立关键帧客户端用关键帧时间兜底。
暂停时不推进 TF 时间，延迟回环在最新数据时刻发布修正；旧参数
`lio.ros.align_sensor_time_to_ros_clock=true` 会警告并忽略。
重放或 seek 到过去时重启节点；回放命令见 [Quickstart](../../docs/quickstart.md)。

## 重定位

用于重定位的参考地图可以直接从在线后端保存，无需录制关键帧 rosbag：

```bash
ros2 launch super_lio_slam vlp32.launch.py \
  save_map_path:=/home/ros/dataset/urbannav/slam_results/reference_map.pcd \
  save_map_voxel_size:=0.2 save_map_on_shutdown:=true

# 另一个已加载环境的终端：停止数据输入，等关键帧接收完后调用。
ros2 service call /lio_slam/save_optimized_map std_srvs/srv/Trigger '{}'
```

服务等待当前运行和已排队的回环配准结束，再按优化位姿保存全部后端关键帧。
服务与关键帧入口串行执行；请先停止输入，以免保存期间接收队列积压。
`save_map_on_shutdown=true` 在正常 Ctrl+C 退出时也执行保存；崩溃或 SIGKILL 不保证保存。
launch 为后端预留 120 秒再升级退出信号，大地图优先在退出前调用服务并确认成功。

输出是 map 系二进制 XYZI PCD。相同路径重复保存会原子替换旧文件，失败保留旧文件。
`save_map_voxel_size=0` 逐点流式保存；正数采用增量体素质心融合，额外内存随占用体素数增长。
它不受 `map_max_keyframes`、`map_max_points_per_keyframe` 或 `path_max_poses` 限制，
但输入是已接收并经过前端/后端关键帧过滤的点云，不能恢复原始扫描中已丢弃的点。
默认输出 `map/optimized_map.pcd`（相对启动目录），体素 0.2 m，默认关闭退出自动保存。
也可在 `slam.yaml` 中设置这三个参数；未显式提供 launch 参数时保留 YAML 值。

```bash
ros2 launch super_lio_slam relocation.launch.py \
  sensor_config:=/home/ros/ros_ws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/robosense_airy.yaml \
  map_path:=/absolute/path/to/reference_map.pcd rviz:=true
```

先短暂静止完成 IMU 初始化，再在局部系 S 中运行 LIO，并累积 `M_S = union(T_S_Ik * P_k)`。
异步执行 FPFH / TEASER++ / GICP；提交前复核最新扫描，以 `T_W_Ilatest = T_W_S * T_S_Ilatest`
变换 KF 的姿态、位置、速度、重力、协方差和预测缓存，保留偏置及传感器时间。

成功后使用参考地图跟踪，默认不更新地图。TF 为 `world → odom_local → imu`；
输出在定位前为 odom_local、成功后为 world。重定位不向回环后端发布关键帧。

参数使用 `relocation_config` / `config/relocation.yaml`，初值为世界系
`[x,y,z,roll,pitch,yaw]`（m / 度）。`/initialpose` 非零时间戳须属于传感器时间域，
零时间戳关联当前状态；状态见 `/lio/relocation/status`。
完整参数与初始化约束见[重定位说明](../../docs/relocation_math.md)。

## 消息、显示与诊断

| 话题 | 内容 / 坐标系 |
|---|---|
| `/lio/odom`、`/lio/imu/odom` | 扫描更新 / IMU 预测的位姿；建图时为 world |
| `/lio/cloud_world` | 当前扫描；`lio.output.dense=false` 使用体素滤波结果 |
| `/lio/cloud_world_dense` | `lio.output.dense=true` 时的完整去畸变扫描 |
| `/lio_slam/keyframe` | `KeyFrame`：原始 world→IMU 位姿及 IMU 系扫描 |
| `/lio_slam/odom`、`/lio_slam/path`、`/lio_slam/map` | map 系优化输出 |
| `/lio_slam/loop_markers`、`/lio_slam/std_triangles` | 已接受回环 / 当前关键帧 STD 可视化 |

Robosense 默认显示滤波后的当前扫描。优化地图由 `map_voxel_size`、`map_max_points_per_keyframe`
及 `map_publish_stride` 独立控制，发布周期按关键帧数计算，不是扫描频率。
`map_max_keyframes` 只限制重建范围，不会清理全部历史检索数据。

`std_pairs` 是 STD 三角形匹配对数，`coarse_inliers=N/M` 的分母是 FPFH 对应数。
`sensor_lag` 是最新前端数据时间减当前关键帧时间；配准耗时查看 `Registration timing`。
接受/拒绝和末尾 `stage` 理由着色；未执行的后续质量检查可能保留零值，不应当作实际测量。

| 常见拒绝理由 | 含义 |
|---|---|
| `few_coarse_inliers` | 粗配准内点数或比例不足 |
| `coarse_rmse_exceeded` | 粗配准 RMSE 超限 |
| `degenerate_coarse_inliers` | 粗内点空间跨度不足或近共线 |
| `excessive_refinement` | GICP 相对初值的平移或旋转修正超限 |
| `low_overlap` | 质量内点数或子图/当前帧覆盖率不足 |
| `high_inlier_mse` | GICP 内点 MSE 超限 |
| `degenerate_geometry` | 精配准几何约束退化 |

## 数据与依赖约束

- LiDAR 和 IMU 输入须同步；缺少逐点时间时只能填零偏移，不能正常恢复帧内运动补偿。
- Unitree 的 float32 `time` 是相对消息头的秒偏移；Robosense 使用逐点 `timestamp`，单位由传感器 YAML 指定。
- 重定位需要短时静止初始化，不支持完全动态冷启动；重新定位或换图时重启节点。
- 当前 `relocation.yaml` 关闭初始位姿先验，全图尝试；可启用配置初值或使用 `/initialpose`。没有独立的全局地点检索模块。

### GTSAM runtime isolation

CMake 记录实际选定的 GTSAM / TEASER++ 库目录，SLAM 后端和重定位 launch 自动应用。
直接执行测试程序时先加载工作区，并使用相同库路径：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:/home/ros/ros_ws/SuperLIO_GLIM_ROS2/install/teaserpp/lib:${LD_LIBRARY_PATH:-}
ctest --test-dir build/super_lio_slam --output-on-failure
```

`test/optimized_map_service_smoke.py` 检查保存服务与正常退出自动保存；`test/relocation_motion_smoke.py` 检查运动累积重定位。数学推导放在工作区 `docs/`。
