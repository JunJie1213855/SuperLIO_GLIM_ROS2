# Quickstart

已完成构建后使用。以下以 VLP32 / UrbanNav 为例；更换传感器时同步修改启动入口、传感器 YAML 和数据路径。基础建图是在线 LIO 位姿估计与建图；已有地图定位使用第 4 节的重定位入口。

## 0. 环境与传感器

每个终端先执行：

```bash
export WS=/home/ros/ros_ws/SuperLIO_GLIM_ROS2
export OUT="$WS/maps"
export BAG=/home/ros/dataset/urbannav/2020-03-14-16-45-35
cd "$WS"
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/livox_ros/install/setup.bash
source install/setup.bash
mkdir -p "$OUT"
```

| 传感器 | 建图启动文件 | 重定位 `sensor_config` 文件（位于 `src/SuperLIO_SLAM/config/`） |
|---|---|---|
| VLP32 / UrbanNav | `vlp32.launch.py` | `vlp32.yaml` |
| Robosense Airy | `robosense_airy.launch.py` | `robosense_airy.yaml` |
| Unitree L1 | `unitree_slam.launch.py` | `unitree_l1.yaml` |
| Livox MID-360 | `Livox_mid360.launch.py` | `livox_360.yaml` |

下文默认不使用仿真时钟。需要时，在 launch 增加 `use_sim_time:=true`，播放器配套增加 `--clock`。每次从头重放或更换会话，都重启算法。

## 1. 基础建图定位（无回环）

终端 A：

```bash
ros2 launch super_lio_slam vlp32.launch.py enable_loop_backend:=false rviz:=true
```

终端 B：

```bash
ros2 bag play "$BAG" --rate 0.5
```

使用实时传感器时，启动雷达/IMU 驱动替代 bag 播放。检查 YAML 中的话题、外参和点云时间单位。

## 2. 建图定位 + 回环

以下二选一，终端 B 仍按第 1 节播放数据。

```bash
# 默认：STD → 多帧 TEASER++ → GICP → iSAM2
ros2 launch super_lio_slam vlp32.launch.py rviz:=true

# 距离检索 → 多帧 TEASER++ → GICP → iSAM2
ros2 launch super_lio_slam vlp32.launch.py loop_candidate_mode:=distance rviz:=true
```

两种方法默认均要求候选时间差 **>30 s**、原始里程计距离 **≤15 m**，并接受配准质量检查。距离半径可用 `loop_candidate_radius_m:=15.0` 覆盖。

## 3. 保存地图

### 回环后端地图（推荐用于重定位）

建图启动时增加保存参数，例如：

```bash
ros2 launch super_lio_slam vlp32.launch.py \
  save_map_path:="$OUT/reference_map.pcd" \
  save_map_voxel_size:=0.2 save_map_on_shutdown:=true rviz:=true
```

需要距离回环时再加 `loop_candidate_mode:=distance`。建图结束，停止输入并确认关键帧已接收完，在另一终端调用：

```bash
ros2 service call /lio_slam/save_optimized_map std_srvs/srv/Trigger '{}'
```

服务等待剩余配准任务完成，按 iSAM2 最新位姿保存全部已保留关键帧，不受显示范围限制；无需录制 rosbag。`save_map_voxel_size=0` 流式保存，正数执行增量体素融合。相同路径重复保存会替换旧文件；正常 Ctrl+C 也可自动保存。

### 纯前端地图

第 1 节关闭后端后没有上述服务。要保存原始 LIO 地图，在所用传感器 YAML 中设置后重新启动：

```yaml
lio.map.save_map: true
lio.map.save_map_dir: "map"
lio.map.map_name: "basic_map.pcd"
lio.map.save_interval: 500
```

正常退出后输出到 `$WS/src/SuperLIO_SLAM/map/basic_map.pcd`；该地图不含 PGO 校正。

## 4. 已有地图重定位

退出建图节点后启动，传感器配置应与建图一致：

```bash
ros2 launch super_lio_slam relocation.launch.py \
  sensor_config:="$WS/src/SuperLIO_SLAM/config/vlp32.yaml" \
  relocation_config:="$WS/src/SuperLIO_SLAM/config/relocation.yaml" \
  map_path:="$OUT/reference_map.pcd" rviz:=true
```

另一终端启动传感器或重新播放 `ros2 bag play "$BAG" --rate 0.5`，查看状态：

```bash
ros2 topic echo /lio/relocation/status --qos-durability transient_local --qos-reliability reliable
```

`WAITING_FOR_STATIONARY_IMU → LOCAL_TRACKING → WORLD_MAP_TRACKING` 表示初始化、局部 LIO、定位成功并进入参考地图跟踪。开始时需要短暂静止或低动态；默认不修改参考地图。

在 `relocation.yaml` 中配置初值：

```yaml
relocation.use_initial_pose: false  # 未知起点：全图尝试，不能保证定位成功
# 已知近似起点时改为 true，并填写首帧 IMU 的地图位姿：
lio.relocation.init_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 米 / 度
```

也可通过 RViz 的 **2D Pose Estimate** 提供 `world` 系初值。`relocation.map_crop_radius_m` 仅在有初值时生效，设为 `0.0` 关闭裁剪。`excessive_refinement` 限制的是 GICP 相对粗配准的修正量，不是地图中的绝对位置；相关参数为 `quality_max_refinement_translation_m`、`quality_max_refinement_rotation_deg`，当前代码要求正数，不能用 `0` 禁用。

## 5. GLIM 多会话地图合并

此流程需要逐关键帧和轨迹；单独保存的参考 PCD 不包含 GLIM 所需的完整图信息。

**每个会话分别执行**：启动第 2 节的建图，先在另一终端录制，再开始播放对应 bag。第一段用 `firstpart`，第二段改为 `secondpart`，两段之间重启建图节点。

```bash
ros2 bag record -o "$OUT/firstpart_result" /lio_slam/keyframe /lio_slam/path
```

每段播放结束后等待后端处理完成，先停止录制，再退出建图。随后导出并转换：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
for part in firstpart secondpart; do
  python3 tools/export_superlio.py --bag "$OUT/${part}_result" --out "$OUT/${part}_export"
  python3 tools/lio2glim.py \
    --poses "$OUT/${part}_export/poses.txt" --clouds "$OUT/${part}_export/clouds" \
    --out "$OUT/${part}_dump" --frames-per-submap 10 --voxel-size 0.2 --config src/glim/config
done
ros2 run glim offline_viewer "$OUT/firstpart_dump"
```

导出点云已经在地图坐标系，不要再次乘位姿。首次转换会编译图写入工具，需要匹配的 GTSAM / gtsam_points；各输出目录使用新名称。

在 Viewer 中：**Open Additional Map** 加载第二段 → **Merge sessions** → **Run fine registration** → 检查贴合后 **Create Factor** → **Find overlapping submaps** → **Optimize**，保存为 `$OUT/merged_dump`。

```bash
python3 tools/glim2lio.py --dump "$OUT/merged_dump" --out "$OUT/merged_export" \
  --global-cloud --global-voxel-size 0.2
```

得到 `$OUT/merged_export/merged.pcd` 后，将第 4 节的 `map_path` 换成该文件即可在合并地图中定位。初值应使用合并地图坐标。

更多参数见 [包 README](../src/SuperLIO_SLAM/README.md)；理论见 [回环与 PGO](loop_pgo_path.md)、[重定位](relocation_math.md)。
