# GLIM 使用说明（本工作区）

本工作区：`~/ros_ws/glim_ws`，包含 `glim` 与 `glim_ros2` 两个包，均固定在 **v1.2.2**。

> 官方的通用文档在 [`docs/`](docs/)，包含参数详解、扩展模块开发等。本文档只讲**在本机跑起来**所需要的、以及本机特有的坑。

## 0. 先读这一段：本机的 GTSAM 冲突

本机装了两套 GTSAM，这是几乎所有编译/运行问题的根源：

| 位置 | 版本 | 来源 | 用途 |
|---|---|---|---|
| `/usr/local/lib/cmake/GTSAM` | **4.3a1** | koide3 PPA `libgtsam-notbb-dev` | ✅ glim 必须用这个 |
| `/opt/ros/humble/lib/cmake/GTSAM` | 4.2.0 | `ros-humble-gtsam` | ROS 自带，会抢占 |

`gtsam_points 1.2.2`（PPA 包，`Depends: libgtsam-notbb-dev`）是按 GTSAM 4.3 编译的，所以 **glim 也必须用 4.3**。但 ROS 的 `CMAKE_PREFIX_PATH` 和 `LD_LIBRARY_PATH` 都排在 `/usr/local` 前面，会默认把 4.2 选走，于是：

- **编译期**报 `the type 'const boost::none_t' ... is not literal`
- **运行期**报 `symbol lookup error: ... undefined symbol: gtsam::NonlinearFactor::rekey`

两者都是同一个原因。按下面的命令显式指定即可绕开。

## 1. 安装编译
安装依赖
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

现在开始编译代码：
```bash
cd ~/ros2_ws/src
git clone https://github.com/JunJie1213855/glim
git clone https://github.com/koide3/glim_ros2

cd ~/ros2_ws
MAKEFLAGS="-j4" colcon build --cmake-args \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM_UNSTABLE
```

**那两个 `-D` 每次都要带。** 漏掉就会出现上面说的 `boost::none_t` 报错。

> 换装 gtsam_points 的变体（如非 CUDA ↔ CUDA 版）后，必须 `rm -rf build/glim install/glim` 再编。因为 `glim/CMakeLists.txt` 在检测到 gtsam_points 无 CUDA 时会用 `FORCE` 把 `BUILD_WITH_CUDA=OFF` 写进 CMakeCache，而 `option()` 不会覆盖已有缓存值，不删缓存就永远不会重新启用 GPU。

## 2. 每次运行前

如果安装了 ros2 的 4.2版本的 gtsam，需要运行如下命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH   # ← 每个新终端都要
```

注意，最后这行是把 GTSAM 4.3 顶到 4.2 前面。**新开终端就得重新执行**（`~/.bashrc` 里 source ROS 时会重置它）。

自检是否生效：

```bash
ldd install/glim/lib/libglim.so | grep libgtsam.so.4
# 期望  /usr/local/lib/libgtsam.so.4          ✅
# 错误  /opt/ros/humble/lib/x86_64-linux-gnu/... ❌ 漏了 export
```

## 3. 运行
数据集如下：
* https://staff.aist.go.jp/k.koide/projects/glim/datasets/mmm.tar.gz (安装这个可以直接运行地图合并)
* https://koide3.github.io/glim/demo.html

本工作区提供 5 个可执行文件（均在 `glim_ros` 包下）：

| 命令 | 用途 |
|---|---|
| `glim_rosnode` | 在线：订阅 ROS 话题跑建图 |
| `glim_rosbag` | 离线：直接读 rosbag，自动调速 |
| `offline_viewer` | 打开已保存的 dump，支持手动编辑与导出 |
| `map_editor` | 地图编辑器，命令行参数与 `offline_viewer` 相同（传 dump 目录） |
| `validator_node` | 数据校验节点，订阅 `imu` / `points` 话题并输出统计 |

### 3.1 在线（实时数据）

```bash
ros2 run glim_ros glim_rosnode
# 另一个终端播放数据
ros2 bag play <your_bag>
```

注意 `glim_rosnode` 默认 **不会**在退出时保存 dump，需要显式开启：

```bash
ros2 run glim_ros glim_rosnode --ros-args -p dump_on_unload:=true
# 保存到 /tmp/dump
```

### 3.2 离线（推荐，最省事）

```bash
ros2 run glim_ros glim_rosbag <rosbag2目录>
```

- 取 **rosbag2** 路径（含 `metadata.yaml` 的目录），也支持 `.mcap` 文件
- 参数支持通配符，可一次传多个（内部用 `glob()` 展开并按名称排序）：
  ```bash
  ros2 run glim_ros glim_rosbag '/path/to/bag_*'
  ```
- **ROS1 的 `.bag` 不能直接用**（内部只用 `rosbag2_cpp` reader，`storage_id` 仅 mcap / sqlite3），需先转换：
  ```bash
  pip install rosbags
  rosbags-convert <your.bag> --dst /tmp/bag_out
  ```
- 会自动调速，避免丢帧
- 结束时**自动保存**到 `/tmp/dump`，可用参数改：
  ```bash
  ros2 run glim_ros glim_rosbag <bag> --ros-args -p dump_path:=/tmp/my_dump
  ```

### 3.3 查看 / 编辑地图

```bash
ros2 run glim_ros offline_viewer <dump目录>
```

- 直接传 dump 目录路径；不传的话启动后可在界面里选
- 支持手动闭环修正、点云导出
- 另有 RViz 插件可用：`rviz2 -d src/glim_ros2/rviz/glim_ros.rviz`

## 4. 配置

配置根目录默认是 `glim/config`（安装后为 `install/glim/share/glim/config`）。

`config.json` 只是索引，指向各子模块的配置：

```json
"global": {
  "config_ros":           "config_ros.json",
  "config_sensors":       "config_sensors.json",
  "config_preprocess":    "config_preprocess.json",
  "config_odometry":      "config_odometry_gpu.json",
  "config_sub_mapping":   "config_sub_mapping_gpu.json",
  "config_global_mapping":"config_global_mapping_gpu.json",
  ...
}
```

### 换自己的传感器，主要改两处

**`config_ros.json`** — 话题名与帧：

```json
"imu_topic":    "/os_cloud_node/imu",     // 默认是 Ouster 的
"points_topic": "/os_cloud_node/points",
"acc_scale":    0.0                        // 0 = 自动。Livox 需设 9.80665
```

> `ang_scale`（角速度单位倍率）是 v1.2.2 之后才加入的字段，本工作区没有。若用 Hesai 这类需要把角速度乘 π/180 的雷达，需自行升级或改用扩展模块处理。

**`config_sensors.json`** — 外参（LiDAR↔IMU）：

```json
"T_lidar_imu": [0.006, -0.012, 0.008, 0.0, 0.0, 0.0, 1.0]
```

格式是 TUM 的 `[x, y, z, qx, qy, qz, qw]`，含义是 `p_lidar = T_lidar_imu * p_imu`。文件注释里列了几种常见雷达的参考值，但**不要盲信，按自己标定的来**。

### 加速 / 降级预设

| 场景 | config_odometry | config_sub_mapping | config_global_mapping |
|---|---|---|---|
| GPU（默认，最快最好） | `config_odometry_gpu.json` | `config_sub_mapping_gpu.json` | `config_global_mapping_gpu.json` |
| CPU（无 CUDA 时） | `config_odometry_cpu.json` | `config_sub_mapping_passthrough.json` | `config_global_mapping_pose_graph.json` |
| 纯 LiDAR（不用 IMU） | `config_odometry_ct.json` | `config_sub_mapping_gpu.json` | `config_global_mapping_gpu.json` |

纯 LiDAR 模式还需把 `config_sub_mapping_gpu.json` 与 `config_global_mapping_gpu.json` 里的 `enable_imu` 改成 `false`（见两文件第 33 / 29 行）。

### 改配置后必须重新编译

配置是**拷贝**进 `install/` 的，不是软链。改完要：

```bash
MAKEFLAGS="-j4" colcon build --cmake-args -DGTSAM_DIR=... -DGTSAM_UNSTABLE_DIR=...
```

想避免反复编译，可以编译时加 `--symlink-install`，或用 `config_path` 参数直接指向工作目录外的配置：

```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ./my_config)
```

`config_path` 以 `/` 开头按绝对路径解释，否则相对 `glim` 包目录。

## 5. 输出

dump 目录结构：

```
/tmp/dump/
├── config/            # 本次运行用的配置快照
├── 000000/            # 每个子图一个目录
│   ├── points_compact.bin   # 点云
│   ├── covs_compact.bin     # 协方差
│   └── data.txt             # 子图位姿（T_world_origin 等）
├── graph.bin          # 因子图（GTSAM 序列化）
├── values.bin         # 位姿值
├── graph.txt          # 因子图的可读摘要
├── odom_{imu,lidar}.txt     # 未闭环的轨迹
└── traj_{imu,lidar}.txt     # 闭环后的轨迹
```

轨迹文件是 TUM 格式，每行 `[t x y z qx qy qz qw]`。

## 6. 打开旧版本产生的 dump

**症状**：`offline_viewer` 打开某个 dump 时：

```
failed to deserialize factor graph!!   unsupported version
terminate called ... std::out_of_range: Requested variable 'x0' is not in this VectorValues.
```

**原因**：dump 的 `graph.bin` / `values.bin` 是 **Boost ≥ 1.76** 生成的（归档头里库版本号是 19），而 Ubuntu 22.04 自带 **Boost 1.74** 只支持到 18。Boost 的归档格式只保证向后兼容，读取端遇到更高的版本号会直接拒绝。

判断方法（第 0x1E 字节为 `13` 即版本 19）：

```bash
xxd -s 30 -l 1 -p <dump>/graph.bin
```

**处理**：`tools/fix_dump_boost19.py`

```bash
cd ~/ros_ws/glim_ws
python3 tools/fix_dump_boost19.py --backup /path/to/dump_*
# --revert   还原
# --dry-run  只看会改什么
# --backup   改前备份为 *.bak
```

它把版本字节 `0x13` 改成 `0x12`，**故意**让反序列化在正文深处失败并抛出 `std::exception`。这个异常正好落进 `global_mapping.cpp` 的第二个 `catch`，把 `needs_recover` 置真，GLIM 随即调用 `recover_graph()`：

- 位姿 `X(i)` 从各子图 `data.txt` 的 `T_world_origin` 填回 → 几何位置正确
- 配准因子从子图体素图重建

**代价**：原始因子图（历史闭环、配准记录）无法还原，是重建而非恢复。看地图、做演示够用；若要可继续优化的完整 session，请用自己的数据重新跑一遍 mapping。

运行后日志里这些是**正常的**，不用管：

```
failed to deserialize factor graph!!   std::bad_alloc            ← 故意触发的
X0 is missing ... X15 is missing                                  ← 位姿正被填回
recovering done / optimize / done
```

## 7. 把其它 LIO 的结果导入 GLIM

GLIM 的会话合并（`File → Open Additional Map` → `Merge sessions`）**只认 dump 目录**，
不认通用的点云+轨迹格式。`tools/lio2glim.py` 负责把别的算法的结果逆向构造成 dump。

### 输入要求

| | 要求 |
|---|---|
| 点云 | 每帧一个 `.pcd`，点**已是世界系**（已配准，如 `/cloud_registered`） |
| 位姿 | TUM 格式 `poses.txt`，每行 `t x y z qx qy qz qw`，位姿是 **IMU/body 系** |

点云和位姿在同一世界系对齐，因此**不需要外参**，`T_lidar_imu` 填单位阵即自洽。

### 用法

```bash
cd ~/ros_ws/glim_ws
python3 tools/lio2glim.py \
  --poses poses.txt \
  --clouds clouds/ \
  --out /tmp/my_session \
  --frames-per-submap 50 \
  --voxel-size 0.5
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--frames-per-submap` | 50 | 每个子图的帧数，影响合并时的配准粒度 |
| `--voxel-size` | 0.5 | 子图内体素降采样边长（m），`0` = 不降采样 |
| `--knn` | 10 | 协方差估计的 kNN |
| `--config` | 自动查找 | GLIM config 目录，拷进 `<out>/config` |

首次运行会用 `g++` 编译一次 `tools/glim_graph_writer.cpp`（约几秒，结果缓存）。
若编译或运行报 `undefined symbol: gtsam::...`，是 GTSAM 4.2/4.3 冲突，脚本已自动
处理子进程的 `LD_LIBRARY_PATH`；若仍出现，检查 `/usr/local/lib` 下是否有 GTSAM 4.3。

### 产出与原理

生成的是**完整合法的 dump**，含真实的 `graph.bin` / `values.bin`：

```
my_session/
├── graph.txt  graph.bin  values.bin
├── config/
└── 000000/{data.txt, points_compact.bin, covs_compact.bin, imu_rate.txt}
```

坐标约定严格对齐 `sub_mapping.cpp:459-461,485`：

```
T_world_origin      = T_world_imu[center],  center = ⌊N/2⌋
T_origin_endpoint_L = T_world_origin⁻¹ · T_world_imu[first]
点云                p_origin = T_world_origin⁻¹ · p_world
```

因子图结构与 `GlobalMapping::recover_graph()` 一致：`X(i) = Pose3(T_world_origin)`，
加 `LinearDampingFactor(X0)` 与相邻子图的 `BetweenFactor<Pose3>`。
逐点协方差由 kNN 邻域 PCA 估计（GICP 配准的依据，不能填常数）。

**注意**：`graph.bin` / `values.bin` 必须写。`GlobalMapping::load()` 对
`gtsam::deserializeFromBinaryFile()` 的返回值不作检查，而该函数在文件打不开时是
`return false` 而非抛异常——所以"省略 graph.bin 让它自动恢复"是不成立的，
会让 isam2 拿到空 Values 并在 `optimize()` 时崩在 `'x0' is not in this VectorValues`。

### 验证是否可用

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run glim_ros offline_viewer /tmp/my_session
```

日志应出现 `Load submaps` → `deserializing factor graph` → `optimize` → `done`，
**不应**出现 `terminated`。之后用 `File → Open Additional Map` 叠加另一个会话，
再 `Merge sessions` 做跨会话配准。

## 8. 排错速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `the type 'const boost::none_t' ... is not literal` | 编译时选中了 GTSAM 4.2 | 编译加 `-DGTSAM_DIR=/usr/local/lib/cmake/GTSAM` |
| `symbol lookup error: ... gtsam::NonlinearFactor::rekey` | 运行时加载了 GTSAM 4.2 | `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` |
| `failed to load libodometry_estimation_gpu.so` | 装的是非 CUDA 版 gtsam_points | 装 `libgtsam-points-cuda13.1-dev` 并 `rm -rf build/glim install/glim` 重编 |
| `unsupported version` 打开 dump 时 | Boost 归档版本 19 | 见第 6 节 |
| `unregistered class - derived class not registered` | 因子类型未注册 | 确认 glim 与 gtsam_points 版本配套 |
| 改了配置不生效 | 配置是拷贝进 install 的 | 重新 `colcon build`，或用 `config_path` 参数 |
| `config_logging.json` 打不开 | 旧 dump 早于该配置文件 | 无害，走默认值 |
| `'x0' is not in this VectorValues` | dump 缺 `graph.bin`/`values.bin` | 见第 7 节；不要省略这两个文件 |
| `unregistered class - derived class not registered` | 因子类型未导出 | 确认编译时带上了 glim 的 `export_factors.cpp` |

---

**本机环境**：Ubuntu 22.04 / ROS 2 Humble / CUDA 13.1 / GTX 1650 Ti
**依赖版本**：GLIM v1.2.2，GTSAM 4.3a1 (`libgtsam-notbb-dev`)，gtsam_points 1.2.2 (`libgtsam-points-cuda13.1-dev`)，iridescence 1.0.2，Boost 1.74
