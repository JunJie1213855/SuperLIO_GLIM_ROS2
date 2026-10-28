# 运动补偿重定位使用说明

日期：2026-10-28。实现采用局部 LIO + 固定 S 系累积 + FPFH/TEASER++ + GICP，成功后切换至已有地图跟踪。

## 构建

在工作区根目录执行：

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
python3 tools/build_teaserpp.py --jobs 1
MAKEFLAGS=-j1 colcon build --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

新重定位必须启用 TEASER++；未启用时节点启动会明确报错，不回退到普通 ICP。

## 启动和参数

下面路径是示例，请将地图参数替换为已有的世界地图 PCD：

```bash
ros2 launch super_lio_slam relocation.launch.py \
  sensor_config:=/home/ros/rosws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/vlp32.yaml \
  map_path:=/absolute/path/to/reference_map.pcd \
  use_sim_time:=true rviz:=true
```

在另一个已加载 ROS 环境的终端回放对应 bag。以上示例启用了仿真时间，需配套 `--clock`；默认可省略示例中的 `use_sim_time:=true` 和播放器的 `--clock`，使用原始传感器时间。详见 [时间策略更新](sensor_time_playback.md)。
实时传感器使用 `use_sim_time:=false`。

`relocation_config` 默认加载包内 `config/relocation.yaml`，与传感器参数分开。
如需更改初值、窗口和配准参数，复制该 YAML 后传入：

```bash
ros2 launch super_lio_slam relocation.launch.py \
  sensor_config:=/home/ros/rosws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/unitree_l1.yaml \
  relocation_config:=/absolute/path/to/my_relocation.yaml \
  map_path:=/absolute/path/to/reference_map.pcd \
  use_sim_time:=true rviz:=true
```

非空 `map_path` 启动参数覆盖 YAML 的 `relocation.map_path`。
两者都为空时使用传感器 YAML 中的 `lio.map.save_map_dir` / `lio.map.map_name`。
显式绝对 `map_path` 更适合工作区外的地图。

| 参数 | 默认值 / 含义 |
| --- | --- |
| `lio.relocation.init_pose` | 首帧 IMU 世界位姿 `[x,y,z,roll,pitch,yaw]`，角度单位为度 |
| `relocation.use_initial_pose` | true；采用配置初值选取地图区域和辅助对应点匹配 |
| `relocation.map_crop_radius_m` | 80；相对先验预测的当前 IMU 世界位置裁剪；0 禁用 |
| `relocation.window_min_frames` / `window_max_frames` | 5 / 10 个选中帧 |
| `relocation.window_max_duration_s` / `window_max_travel_m` | 3 s / 5 m 累计路程 |
| `relocation.keyframe_distance_m` / `keyframe_angle_deg` | 0.2 m / 5° |
| `relocation.keyframe_max_interval_s` | 0.3 s |
| `relocation.registration_interval_s` | 1 s；只有一个运行任务和一个最新待处理快照 |
| `relocation.local_min_plane_points` | 30；局部观测不足时不新增配准任务、不提交重定位 |
| `relocation.imu_init_samples` / `imu_init_duration_s` | 至少 50 个样本、0.3 s 的低动态初始化 |
| `relocation.max_gravity_angle_deg` | 10°；配准后重力与世界向下方向的一致性 |
| `relocation.registration_position_sigma_m` / `registration_rotation_sigma_deg` | 0.2 m / 2°；切换时保守增加位姿协方差，不代表已标定配准协方差 |
| `lio.relocation.update_map` | false；定位后默认不修改参考地图 |

`registration_*`、`fpfh_*`、`teaser_*`、`coarse_*`、`gicp_*` 和 `quality_*`
同样在独立重定位 YAML 中配置，不读取在线回环的关键帧年龄或 30 s 时间门限。
默认严格要求内点 MSE <0.1 m²，源子图和当前帧覆盖率均 >80%；等号拒绝。
还检查最少内点、有效变换、精修幅度、几何退化和重力方向。

## 运行过程

1. 加载 PCD，建立不可变配准目标和世界地图近邻索引；加载失败直接退出。
2. 短时静止/低动态 IMU 初始化后，只用一帧作为局部地图种子。
3. 后续扫描逐帧进行 IMU 传播、去畸变和局部地图观测。每帧点云已经位于扫描结束 IMU 坐标系中。
4. 根据每帧更新后的局部位姿累积 `M_S = union(T_S_Ik * P_k)`，雷达到 IMU 的外参不重复应用。
5. 异步执行 FPFH/TEASER++/GICP。在有先验时，将候选地图暂时表达在提示的 S 系中计算特征和体素网格，避免两个原点造成法向定向不一致；解出的增量恢复成完整 W<-S 后再精配准。
6. 配准失败继续局部 LIO；待处理任务只保留最新快照，不中断运行中的求解器。
7. 成功结果只在扫描结束的主线程提交。先检查最新扫描在世界地图中的 MSE/覆盖率，再执行 `T_W_Ilatest = T_W_S * T_S_Ilatest`。
8. 同时变换 KF 的 R/p/v/g、协方差和预测缓存，保留 IMU 偏置、传感器时钟及原始 IMU 样本；切换活动地图到世界地图。

配准成功不会把 KF 回退到快照帧，也不会将时间设为 -1。
协方差保留交叉项并按参考系变换，额外加入可配置的保守位姿不确定性。

## 初始位姿和坐标输出

运行中可在 RViz 向 `/initialpose` 发布世界系先验：

- frame_id 必须为 `world`（空值按 world 处理），使用消息中的实际 z，不再固定为 0.2 m。
- 非零时间戳必须与局部滑窗中的关键帧在 0.25 s 内匹配；使用原始传感器时间，不再施加设备到 ROS 时钟的偏移。
- 时间戳为零时关联当前局部状态。
- 新先验会增加任务代号，使已运行的旧先验结果失效；不会重置局部 LIO。
- 当前版本为启动重定位，成功后如需重新定位/更换地图，重启节点开始新会话。

输出状态位于 `/lio/relocation/status`：

```text
LOAD_REFERENCE_MAP -> WAITING_FOR_STATIONARY_IMU -> LOCAL_TRACKING
  -> WORLD_MAP_TRACKING
```

局部观测不足时为 `LOCAL_TRACKING_DEGRADED`。已交付扫描的时间倒退会进入错误状态。
回放同一 bag 前应重启节点。

未定位时 `/lio/odom`、`/lio/path` 和 `/lio/cloud_world` 的 frame_id 是 `odom_local`；
定位后为 `world`，路径在切换时清空，避免混合两个坐标系。
参考地图 `/lio/global_map` 始终表达在 world。

TF 始终保持 `world -> odom_local -> imu` 的结构：未定位时没有 world 连接，
定位后新增 W<-S。内部 KF 切换到 W 后，用 `inverse(T_W_S) * T_W_I` 生成局部 TF。
不会同时再广播 world -> imu。重定位节点不向在线 SLAM 后端发布关键帧。

RViz 在 world 固定坐标系下，初始化前无法将局部扫描叠加到世界地图上，这是未知坐标关系的正常表现。
检查局部跟踪时可选择 odom_local 为固定坐标系。

## 边界和验证入口

需要短时可靠的 IMU 初始条件；该版本不声称支持从上电就持续剧烈运动的动态冷启动。
IMU 低动态检查也不能仅凭惯性数据证明线速度为零，启动时应实际保持短时静止。
移动阶段必须有可靠逐点时间；缺失时间字段的点云不能恢复真实帧内去畸变。

有先验时，取消对应点提示不会恢复已裁剪掉的地图；先验错误超出裁剪区域需更换先验或扩大范围。
`use_initial_pose=false` 对整幅地图做有界点数的特征配准，不包含离线地点索引，不能保证未知地点的大地图检索成功。
地图侧特征缓存、动态 IMU 冷启动及运行中失锁自动重定位尚未实现。

```bash
ctest --test-dir build/super_lio_slam --output-on-failure
```

`test/relocation_state_test.cpp` 检查完整 KF 状态/交叉协方差、前向预测连续性、固定 S 系运动累积、完整 W<-S 配准和重力拒绝。
`test/relocation_motion_test.cpp` 通过同一进程的生产 ROSWrapper/SuperLIOReLoc 注入移动扫描，覆盖完整重定位管线并避免外部 DDS 传输干扰。
`test/relocation_motion_smoke.py` 通过真实节点注入带逐点运动畸变的模拟雷达与 IMU，检查延迟提交、世界跟踪和单一 TF 父级。
直接运行测试前需加载工作区并配置兼容库路径。移动 ROS 用例默认 Fast DDS，独立 ROS domain，避免影响正常节点：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:/home/ros/rosws/SuperLIO_GLIM_ROS2/install/teaserpp/lib:${LD_LIBRARY_PATH:-}
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=49 ROS_LOCALHOST_ONLY=1 \
  python3 src/SuperLIO_SLAM/test/relocation_motion_smoke.py \
  --node install/super_lio_slam/lib/super_lio_slam/relocation_node
```

## 本机验证结果

完整构建、VLP32 launch 加载参考地图和受控退出检查通过，CTest 3/3 通过。进程内生产 ROS 管线用例得到 25 个局部位姿和 132 个世界位姿，
合成运动轨迹的最大位置误差约 0.0084 m。配准快照时间 1002.1 s，提交时使用最新 1002.9 s 状态，
KF 时间保持 1002.9 s，TF 的 imu 子节点只有 odom_local 一个父级。地图缺失启动报错检查通过。
结构化记录见 [验证报告](validation/2026-10-28/motion_relocalization_report.json)。

外部 Python/C++ 跨进程烟测在本机尚未通过：Cyclone DDS 运行中 IMU 到达而点云未到达，
Fast DDS 也观察到订阅发现或输入接收不稳定；曾有运行推进到粗精配准，但没有完成最终的跨进程验收。
不能把进程内管线通过表述为跨进程部署已经验证。未修改系统 DDS/网络配置来掩盖该结果。
这些结果只覆盖合成数据，不替代真实传感器 bag 和真值精度验收。
