# 启动职责与 tmux

组合流程由工作区 `scripts/*.sh` 管理；一个 tmux session，多个独立 window。新 launch 均直接启动本模块进程，不 include 其他 launch。

| 职责 | 独立 launch | tmux window |
|---|---|---|
| IMU/LiDAR 融合、连续里程计、局部建图、可选关键帧发布 | `super_lio_slam lio.launch.py` | `lio` |
| 在线回环、iSAM2、优化地图及保存服务 | `super_lio_slam loop_backend.launch.py` | `loop_backend` |
| 局部 LIO、全局配准、先验地图跟踪 | `super_lio_slam relocation.launch.py` | `relocation` |
| 滑动占据地图、避障和 B-spline 规划 | `scan_planner planner.launch.py` | `planner` |
| 根据里程计跟踪样条，发布 `/cmd_vel` | `scan_planner controller.launch.py` | `controller` |
| 显示点云、位姿、栅格和轨迹 | `super_lio_slam rviz.launch.py` | `rviz` |
| GLIM 多会话 dump 加载、配准与图优化界面 | `glim map_merge.launch.py` | `map_merge` |
| 原始数据回放 / 合并所需结果录制 | `ros2 bag play / record` | `bag` / `record` |

`relocation` 已包含局部 LIO，不能与 `lio` 同时运行。GLIM 合并是离线阶段，输入为 dump，不直接订阅在线 LIO 点云。只有 `slam` 模式会启动在线回环后端，同时为 LIO 打开关键帧发布。

## 脚本入口

在工作区执行（也支持使用脚本绝对路径从任意目录调用）：

```bash
# 基础建图：lio + rviz
./scripts/start_mapping.sh

# 建图加回环：loop_backend + lio + rviz
./scripts/start_mapping.sh slam

# 导航回放：lio + planner + rviz + bag
BAG_PATH=/home/ros/dataset/robosense/mapping ./scripts/start_navigation.sh lio

# 已有地图定位加规划：relocation + planner + rviz
./scripts/start_navigation.sh relocation /absolute/reference.pcd

# 只定位，不启动 Planner
./scripts/start_relocation.sh /absolute/reference.pcd

# 离线合并：仅 GLIM Viewer；在界面加载其他会话并建立跨会话约束
./scripts/start_merge.sh /absolute/first_dump
```

四个脚本各自独立，只保留简单的 `new_win` 函数。配置变量和每条 `ros2 launch` 命令都写在本文件，按注释从上到下执行，不再 source `_session.sh` 或调用 `_run.sh`。无需通用参数解析器，也不自动 attach。

默认 Robosense Airy，对应 indoor 配准配置。修改脚本开头的 `SENSOR_CONFIG`、`REGISTRATION_CONFIG`、`PLANNER_CONFIG` 等变量即可更换参数；也支持同名环境变量。VLP32 示例：

```bash
SENSOR_CONFIG="$PWD/src/SuperLIO_SLAM/config/common/vlp32.yaml" \
REGISTRATION_CONFIG="$PWD/src/SuperLIO_SLAM/config/loop/loop_outdoor_teaser_gicp.yaml" \
BAG_PATH=/home/ros/dataset/urbannav/2020-03-14-16-45-35 BAG_RATE=0.5 \
  ./scripts/start_mapping.sh slam
```

节点 window 先进入工作区并 `source install/setup.bash`，再运行命令。结束后 `exec bash` 保留 shell 和输出。主 `terminal` window 可手工运行保存地图服务等命令。脚本默认关闭缺失点云 `time` 字段的警告。

`BAG_PATH` 非空时，各算法节点使用仿真时间，播放器带 `--clock --start-paused`；等节点初始化后在 `bag` 窗口按空格开始。实时驱动清空 `BAG_PATH`，默认系统时钟。外部另行回放时，设置 `USE_SIM_TIME=true`，播放器手动加 `--clock`。Robosense 当前数据的传感器时间与录制时间不同；尚未实现两者对齐，回放保持控制器关闭。

导航默认只规划；RViz 用 **2D Goal Pose** 发送 `/goal_pose`。设置 `ENABLE_CONTROLLER=true` 才创建 `controller` window，同时为规划器启用控制进度反馈。高度偏移直接修改 `planner.yaml` 的 `fsm.odom_height_offset`，脚本不覆盖。`NAVI_MODE=3` 需要外部路径发布器，DDDMR 尚未接入启动脚本。

回环默认 `USE_TEASERPP=true`。用 `USE_TEASERPP=false ./scripts/start_mapping.sh slam`（或 `start_navigation.sh slam`）可在相同多帧窗口下跳过 FPFH/TEASER++，直接使用 LIO 初值做 GICP。该开关不传给重定位节点；修改后需重启，不能通过 `ros2 param set` 热切换。

## 窗口与退出

session 名直接写在每个脚本开头：`superlio_mapping`、`superlio_navigation`、`superlio_relocation`、`superlio_merge`。重复执行脚本会先结束同名 session，再重新创建；运行前先保存已有地图、停止结果录制。不同 session 不自动互斥，避免同时启动两个前端。

```bash
tmux attach -t superlio_navigation  # 返回会话
tmux list-windows -t superlio_navigation
```

tmux 默认按键：`Ctrl+B` 然后 `n/p` 切换 window，`Ctrl+B` 然后 `w` 选择窗口，`Ctrl+B` 然后 `d` 脱离而保持运行。在某窗口按 `Ctrl+C` 只停止该模块；结束后保留 shell 查看日志。

有回环保存或 GLIM 结果录制时，先停止数据输入，等待后端处理完，再保存/退出：

```bash
./scripts/stop.sh superlio_mapping input
# 等关键帧与配准队列处理完成；如使用实时驱动，需另外停止驱动。
ros2 service call /lio_slam/save_optimized_map std_srvs/srv/Trigger '{}'
./scripts/stop.sh superlio_mapping all
# 确认保存成功、进程退出后，删除留下的日志窗口：
tmux kill-session -t superlio_mapping
```

`stop.sh ... all` 向各窗口发送 Ctrl+C，不直接 kill 进程，允许后端按配置自动保存。停止时保留录制到后端处理结束，避免丢失最后的关键帧结果。

## 不使用 tmux 时

每条命令独立终端执行，不需要启动任何上层 launch：

```bash
ros2 launch super_lio_slam lio.launch.py publish_keyframes:=true
ros2 launch super_lio_slam loop_backend.launch.py
ros2 launch scan_planner planner.launch.py
ros2 launch super_lio_slam rviz.launch.py \
  rviz_config:=/home/ros/ros_ws/SuperLIO_GLIM_ROS2/src/Scan-Planner-Car/src/planner/plan_manage/launch/superlio.rviz
```

接入重定位的独立 Planner / 控制器需传 `require_world_tracking:=true`。独立控制器通过 `controller.launch.py` 启动，同时给 Planner 传 `use_controller_progress:=true`。

原 `navigation.launch.py` 和 Planner 的 `superlio.launch.py` 已由脚本及独立 launch 替代。旧传感器 launch、`slam.launch.py` 和 Planner 仿真演示保留兼容；新脚本不调用它们。`relocation.launch.py` 现在只启动重定位节点，显示需单独启动 `rviz.launch.py profile:=relocation`。
