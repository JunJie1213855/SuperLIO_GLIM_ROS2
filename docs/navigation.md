# SuperLIO + SCAN 差速小车规划

LIO、回环、规划、控制和 RViz 分别启动；组合流程放在 `scripts/start_navigation.sh`，单个 tmux session 内每个模块独立 window。职责和退出方式见 [启动说明](launches.md)。规划、占据地图和控制轨迹均在连续前端 `world` 系。

## 构建

LIO 不再依赖 Planner，需要同时构建两个入口：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/livox_ros/install/setup.bash
colcon build --packages-up-to super_lio_slam scan_planner --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

规划核心为 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`scan_planner_msgs` 和 `scan_planner`。新入口不依赖 Go2 仿真。

## 启动

```bash
# 默认 Airy；回放自动配套仿真时钟，bag 窗口按空格开始。
BAG_PATH=/home/ros/dataset/robosense/mapping ./scripts/start_navigation.sh lio

# 建图加回环、规划；候选也可设置 LOOP_CANDIDATE_MODE=distance。
./scripts/start_navigation.sh slam

# 先验地图重定位加规划，接管前不接收局部预览数据。
./scripts/start_navigation.sh relocation /absolute/reference_map.pcd

# 已有 LIO 在运行，只启动规划器（不启动控制器和 RViz）。
ros2 launch scan_planner planner.launch.py require_world_tracking:=false
```

使用 RViz 的 **2D Goal Pose** 发送 `/goal_pose`。脚本开头直接列出 `SENSOR_CONFIG`、`PLANNER_CONFIG` 等路径，修改变量即可切换配置，也支持同名环境变量。传感器 YAML 的 `lio.output.map` 必须开启，供规划器获取 `/lio/cloud_world`。

默认控制器关闭。需要时设置 `ENABLE_CONTROLLER=true`，通过独立 `controller` 窗口发布 `/cmd_vel`；`CMD_VEL_TOPIC` 可修改命令接口。回放运动不受该命令控制。当前 Robosense bag 的录制时间和传感器时间基准不同，尚未对齐，回放保持控制器关闭。

输入机器人里程计来自 `lio.extrinsic.odom_robo`；规划高度偏移直接修改 `planner.yaml` 的 `fsm.odom_height_offset`，脚本不覆盖该值，修改后重启节点。`PLANNER_CONFIG`、`CONTROLLER_CONFIG` 可指定独立 YAML。

## 接口及坐标

| 输入/输出 | 约定 |
|---|---|
| `/lio/robo/odom` | 机器人位姿，线速度为 world 表达，包含旋转杆臂项；不是标准机体系 twist 适配接口 |
| `/lio/lidar/odom` | 帧末雷达位姿，由 `T_world_imu * T_imu_lidar` 得到 |
| `/lio/cloud_world` | 已去畸变的世界系当前扫描，与雷达位姿严格同戳配对；不重复应用外参 |
| `/goal_pose` | 手动目标；支持 world 或可经 TF 转换的坐标系 |
| `/initial_path` | `navi_mode:=3` 时的外部参考路径；节点接收时转换为 world |
| `/planning/bspline` | world 系 B-spline，差速控制器按 XY 跟踪 |
| `/lio/relocation/world_epoch` | latched Header，给出重定位接管的传感器时间边界 |

重定位模式下，地图、规划器和控制器都等待 `world_epoch`。收到后清空旧栅格、目标、轨迹和可视化，并丢弃时间早于边界的旧消息；手动目标/参考路径需重新发送，预设航点则按配置重新规划。单独启动规划器连接重定位时设置 `require_world_tracking:=true`。

回环不重置连续的前端 world。地图系目标/参考路径在**接收时**转换，当前没有全局路线自动更新模块；回环校正后，上层应重新发送地图系路线或目标。预设航点使用 world 坐标，不能直接填写 map 坐标。

`navi_mode:=2` 需要 `keypoints_file:=/absolute/waypoints.yaml`；`navi_mode:=3` 需外部发布 `/initial_path`。这套模块提供局部避障和轨迹跟踪，不自动从完整 PCD 中生成跨区域全局路线。

## 验证

```bash
colcon test --packages-select bspline_opt scan_planner
colcon test-result --verbose
```

部分测试会启动 ROS 节点，需要可用 DDS 通信；纯算法测试可分别运行对应 build 目录中的 gtest 程序。
