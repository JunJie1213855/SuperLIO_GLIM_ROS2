# SuperLIO + GLIM

ROS 2 LiDAR / IMU 建图定位、在线回环、参考地图重定位与 GLIM 多会话地图合并。

**运行命令见 [Quickstart](docs/quickstart.md)**：基础建图 → STD / 距离回环 → 保存地图 → 重定位 → 多会话合并。

| 功能 | 当前流程 |
|---|---|
| 基础建图定位 | SuperLIO 前端 LIO，输出连续位姿与点云 |
| 在线回环 | STD（默认）或距离检索 → 多帧 TEASER++ → GICP → iSAM2 |
| 保存地图 | 服务保存完整优化关键帧地图，可配置退出自动保存，无需录制 rosbag |
| 重定位 | S 系累积局部地图快照 → TEASER++ / GICP → 状态与协方差变换 → 参考地图跟踪 |
| 地图合并 | 导出各会话关键帧 → GLIM dump → Offline Viewer 配准优化 → merged.pcd |

## 构建

本机使用 Ubuntu 22.04 / ROS 2 Humble；其他机器请调整工作区和 Livox 驱动路径。

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/livox_ros/install/setup.bash

git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential \
  --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM_UNSTABLE \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

仅重编 SuperLIO 时增加 `--packages-select super_lio_slam`。GLIM 与 SuperLIO 应使用一致的 GTSAM；TEASER++ 由构建助手安装到 `install/teaserpp`。

## 文档与目录

- [Quickstart：完整运行流程](docs/quickstart.md)
- [SuperLIO 参数、话题与诊断](src/SuperLIO_SLAM/README.md)
- [闭环配准与 iSAM2 PGO 理论](docs/loop_pgo_path.md)
- [重定位坐标系、状态与协方差理论](docs/relocation_math.md)

```text
src/SuperLIO_SLAM/   SuperLIO 前端、回环、保存地图与重定位
src/glim/           GLIM 子图优化
src/glim/ros2/      ROS 2 接口与 Offline Viewer
tools/              TEASER++ 构建、关键帧导出与地图格式转换
docs/               使用说明与数学推导
```

### GLIM ROS 2 接口合并

ROS 2 节点、组件和离线查看器已纳入 `glim` 包，源码位于 `src/glim/ros2/`；不再单独构建 `glim_ros`。入口统一为 `ros2 run glim glim_rosnode`、`ros2 run glim glim_rosbag <bag目录>` 和 `ros2 run glim offline_viewer <dump目录>`。节点名、话题、配置键和 `glim::GlimROS` 组件类名保持不变。

从旧工作区升级时，请在新终端仅 source ROS 环境后重新构建；旧的 `install/glim_ros` 是过期产物，不应继续使用。

SuperLIO 配置分为 `config/common/`（传感器与前端）、`config/loop/`（关键帧与回环）、`config/relocation/`（重定位）。启动文件默认路径已同步；显式指定 YAML 时需使用新路径。
