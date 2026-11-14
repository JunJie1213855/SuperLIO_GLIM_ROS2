# SuperLIO + GLIM

ROS 2 LiDAR / IMU 建图定位、在线回环、参考地图重定位与 GLIM 多会话地图合并。

**运行命令见 [Quickstart](docs/quickstart.md)**：基础建图 → Scan Context 回环 → 保存地图 → 重定位 → 多会话合并。

扫描滤波支持 `fixed`（默认，原固定体素）和 `adaptive`（GenZ-LIO 场景尺度 + PD 控制，观测 / 入图双分辨率）。建图或重定位 launch 可加 `downsample_mode:=adaptive`。

LIO 与重定位仅采用点到面观测，平面匹配失败的点直接丢弃；在 RViz 用蓝色显示参与估计的观测点。

支持通过 CMake 选项 `WITH_JEMALLOC=ON` 为算法节点直接链接 jemalloc，默认关闭。安装、编译和验证方法见 [Quickstart](docs/quickstart.md#可选jemalloc-内存分配器)。

| 功能 | 当前流程 |
|---|---|
| 基础建图定位 | SuperLIO 前端 LIO，输出连续位姿与点云 |
| 在线回环 | Scan Context → 多帧子图 GICP → iSAM2 |
| 保存地图 | 服务保存完整优化关键帧地图，可配置退出自动保存，无需录制 rosbag |
| 重定位 | S 系累积局部地图快照 → TEASER++ / GICP → 状态与协方差变换 → 参考地图跟踪 |
| 地图合并 | 导出各会话关键帧 → GLIM dump → Offline Viewer 配准优化 → merged.pcd |

## 构建

本机使用 Ubuntu 22.04 / ROS 2 Humble；其他机器请调整工作区和 Livox 驱动路径。

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash

git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential \
  --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM_UNSTABLE \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

仅重编 SuperLIO 时增加 `--packages-select super_lio_slam`。GLIM 与 SuperLIO 应使用一致的 GTSAM。只做建图和在线回环时，可跳过 TEASER++ 安装，并使用 `-DSUPERLIO_WITH_TEASER=OFF`。参考地图重定位需要 `-DSUPERLIO_WITH_TEASER=ON`。

## 文档与目录

- [Quickstart：完整运行流程](docs/quickstart.md)
- [从旧版回环命令迁移](docs/quickstart.md#旧版命令迁移)
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
