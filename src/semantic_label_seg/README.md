# semantic_label_seg

从 SLIDE-SLAM 提取的独立语义标签库，提供纯 Python API 和 ROS 2 Humble 节点。
支持原前端的 LiDAR RangeNet 路线，以及 YOLO 实例分割 + 对齐深度路线。
模型架构源文件和来源许可证保留于包内；不依赖原 SLIDE-SLAM 运行环境。

## 范围与当前验证边界

- LiDAR：输入 Nx3 点和反射强度，输出等长、同点序的 `uint32` 类别和 `float32` 分数。
- RGB-D：图像实例分割输出类别图、帧内实例 ID、检测分数；对齐深度生成相机光学系语义点云。
- 点云 ROS 输出增加 `label`、`confidence` 字段；LiDAR 节点默认另加 `rgb` 供 RViz 着色。保留输入 XYZ、intensity、ring、time 等其他字段，以及 Header 与组织结构。
- 坐标系变换仅用于 LiDAR 模型推理，ROS 输出的点仍使用原输入坐标。
- 缺少或不兼容的权重直接报错；不会继续使用随机权重生成标签。
- 本包不承担状态估计、对象跟踪、几何对象建模、拓扑建图或导航。

当前仓库未提供训练权重。CPU 合成权重测试只能验证模型装载和计算流程，不能证明真实模型的分割精度。
YOLO 掩码处理与 RGB-D 几何可单独测试；真实 YOLO 推理还需安装 Ultralytics 并提供本地分割权重。

## 构建

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_label_seg --symlink-install
source install/setup.bash
```

基础库需要 NumPy 和 PyYAML；LiDAR 推理另需当前 Python 环境中的 `torch>=2.0`；
图像推理另需 `ultralytics`。安装示例：

```bash
python3 -m pip install 'torch>=2.0'  # 按实际 CPU/CUDA 平台选择 PyTorch 发行包
python3 -m pip install ultralytics  # 仅使用图像路线时需要
```

本包自身不使用 `cv_bridge` 或 `ros_numpy`，避免当前机器上 NumPy 2 与 Humble 二进制 cv_bridge 的 ABI 冲突。
不会替换你的系统 NumPy、CUDA 或 PyTorch。第三方推理库仍应使用相互兼容的依赖版本。

## Super-LIO + 语义标签可视化（当前推荐入口）

只需要下面这条链路：

```text
LiDAR + IMU → Super-LIO → /lio/cloud_world_dense（去畸变，world 系）
                              ↓ 同时间戳 TF：world → imu → semantic_lidar
                         转回雷达系做 RangeNet 推理
                              ↓ 标签按原始点序附回 world 系点云
                   /semantic_label_seg/points → RViz RGB8
```

Super-LIO 提供里程计、去畸变点云和 `world → imu` TF；分割节点提供逐点类别和颜色。
这条启动链不使用 Hydra，也不启动 SLIDE-SLAM 后端、对象跟踪、拓扑图或回环后端。
Super-LIO 自身用于定位的几何局部地图仍正常工作。

先准备下一节所列的**真实训练权重目录**。仓库内没有这些权重，测试权重不能用来获得有效语义。
以下以 Airy 配置为例；`sensor_config` 必须换成与你的雷达、IMU、话题及标定一致的配置。
例如 VLP32 数据使用 `config/common/vlp32.yaml`，不要仅为了启动成功而套用其他传感器外参。

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
# super_lio_slam 已经编译过时，只需构建这一个 Python 包
colcon build --packages-select semantic_label_seg --symlink-install
source install/setup.bash

ros2 launch semantic_label_seg superlio_semantic.launch.py \
  sensor_config:=/home/ros/ros_ws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/common/robosense_airy.yaml \
  model_dir:=/absolute/path/my_model \
  device:=auto desired_frequency:=2.0
```

随后启动传感器驱动或播放对应 rosbag。若使用 `use_sim_time:=true`，播放时还需要 `ros2 bag play /path/to/bag --clock`。
`device:=auto` 自动选择可用 CPU/CUDA；也可显式设置 `device:=cpu`。先用 2 Hz 检查效果，再按实际算力提高频率。
Airy、Livox 等雷达与训练数据的扫描模式、视场、强度分布可能不同，模型能运行不代表分割精度合格。

联合启动文件会为此次启动覆盖 `lio.output.dense=true`，并关闭关键帧发布、机器人位姿输出与 PCD 保存；
不修改原传感器 YAML。它从同一个 YAML 的 `lio.extrinsic.lidar_imu` 读取完整外参，广播 `imu → semantic_lidar`。
`semantic_lidar` 是用于推理的雷达坐标系，不要求原始驱动已发布相同名称的 TF。
分割查询点云时间戳对应的 TF；变换晚到时暂存扫描重试，输出坐标始终保持原来的 `world` 系。

**如果 Super-LIO 已经在运行**，加 `start_lio:=false`，避免重复启动。此时必须用同一份传感器配置，
且已有 Super-LIO 必须已启用 `lio.output.dense=true`，确实发布 `/lio/cloud_world_dense`。
`rviz:=false` 可关闭 GUI。该联合启动入口需要安装好的 `super_lio_slam`，普通库 API 和单独分割节点仍可独立使用。

RViz 配置已设置：`Fixed Frame=world`、订阅 `/semantic_label_seg/points`、`Color Transformer=RGB8`、
`Reliability=Best Effort`。默认只显示最新一帧，另显示 `/lio/path` 轨迹。
若想看最近几秒的彩色点云，可把 `Semantic labels → Decay Time` 改为 3～10 秒；这是显示缓存，
不会进行跨帧语义融合、地图持久化或回环修正。

同一最终类别 ID 始终使用同一默认颜色，未知类别默认灰色；默认调色板每 20 类循环，不表示固定类别含义。
类别名称由模型 `data_cfg.yaml` 决定，节点启动日志会打印 ID 对应名称。
可通过 `color_map_file:=/absolute/path/colors.yaml` 指定最终输出 ID 的 RGB 颜色，例如：

```yaml
0: [96, 96, 96]    # unknown
1: [255, 0, 0]    # 示例 ID；实际含义查模型配置
2: [0, 255, 0]
```

`rgb: UINT32` 只供显示，`label: UINT32` 才是语义 ID，`confidence: FLOAT32` 是模型分数。
单独启动 LiDAR 节点时也默认输出这三个字段；`publish_rgb:=false` 可关闭配色字段的生成。

排查时先检查 `ros2 topic hz /lio/cloud_world_dense`，再检查 `ros2 topic hz /semantic_label_seg/points`。
前者无数据先检查 Super-LIO 输入与初始化；后者无数据检查模型加载、日志中的 TF 提示及输入/推理坐标系。
若出现 TF 缺失，可运行 `ros2 run tf2_ros tf2_echo world semantic_lidar` 检查整条坐标链。

## LiDAR 模型文件

使用 SLIDE-SLAM README 指向的 RangeNet 权重，或相同架构训练得到的权重目录：

```text
my_model/
├── arch_cfg.yaml
├── data_cfg.yaml
├── backbone
├── segmentation_decoder
├── segmentation_head
└── segmentation_CRF       # 仅 arch_cfg.yaml 开启 CRF 时需要
```

支持原前端的 `darknet` 和 `basic_darknet` 架构。权重按 PyTorch state_dict 严格加载。
输入通道顺序为 `[range, x, y, z, intensity]`，归一化、垂直视场和投影尺寸来自 `arch_cfg.yaml`。
模型训练时的 LiDAR 类型、视场和强度尺度应与实际数据匹配；迁移到 Livox 等不同采样模式时需验证精度。
本实现使用球面投影反投影，没有增加 RangeNet 的 kNN 后处理；同一投影像素内的点可能共享类别预测。

```python
from semantic_label_seg import RangeNetSegmenter

segmenter = RangeNetSegmenter('/absolute/path/my_model', device='cpu')
result = segmenter.segment(points_xyz, intensity)  # NumPy: Nx3、N
labels = result.labels                           # uint32, N
scores = result.confidence                       # float32, N
```

非有限、零距离、视场外、距离阈值外的点保留原位置，标签设为 `unknown_label`，分数为零。
`confidence` 是模型最大输出分数，不是经过校准的定位或语义正确率。
学习类别 ID 默认直接输出；`label_space='original'` 使用 `learning_map_inv` 转换为原数据集类别 ID。
可选 `label_map`/`label_map_file` 在所选类别空间之后进一步映射到你的应用类别。

## LiDAR ROS 节点

```bash
ros2 run semantic_label_seg lidar_seg_node --ros-args \
  -p model_dir:=/absolute/path/my_model \
  -p input_topic:=/your_lidar_points \
  -p inference_frame:=your_lidar_frame \
  -p output_topic:=/semantic_label_seg/points \
  -p device:=cpu
```

也可复制并编辑 `config/lidar.yaml` 后启动：

```bash
ros2 launch semantic_label_seg lidar.launch.py params_file:=/absolute/path/lidar.yaml
```

订阅和发布使用 SensorDataQoS。`desired_frequency=0.0` 表示不节流，正数按传感器时间戳节流。
数据时间倒退时会重置该限频判断，适合重新播放录制数据。
分割节点只保留一帧等待 TF 的扫描和最新一帧待处理扫描；推理速度不足时会丢弃中间扫描。

`inference_frame` 指训练输入使用的 LiDAR 原点和轴向；输入 frame 不同时，在点云时间戳处查询 TF。
找不到变换时暂存该帧重试，最多等待 `tf_wait_timeout` 秒（默认 2 秒墙上时间），同时缓存后续最新帧。
这样 TF 比点云稍晚到达时不会因不断替换待处理帧而持续丢失输出。
为空时假定输入已经是正确 LiDAR 系；`world/map/odom` 输入会被拒绝。
这是数值坐标变换，不是简单更改 `frame_id`。

### 接 Super-LIO

逐帧可视化优先使用上面的 `superlio_semantic.launch.py`。下面保留可选的关键帧接法，适合只处理稀疏扫描的用途。

当前项目 `/lio_slam/keyframe` 中 `cloud` 位于扫描末时刻的 IMU 系。可以直接选择消息类型：

```bash
ros2 run semantic_label_seg lidar_seg_node --ros-args \
  -p model_dir:=/absolute/path/my_model \
  -p input_type:=superlio_keyframe \
  -p input_topic:=/lio_slam/keyframe \
  -p inference_frame:=your_actual_lidar_frame
```

需要先 source 当前 Super-LIO 安装环境，并提供正确的 `LiDAR ← IMU` TF；不要使用猜测的单位外参。
原始消息中的位姿/协方差不参与单帧语义推理，标签点云保持 cloud 原时间戳和 IMU frame。
`super_lio_slam` 是这个可选输入分支的运行依赖，纯库和普通 PointCloud2 模式不依赖它。
关键帧是否过于稀疏、是否需要改接逐帧去畸变点云，取决于后续重建密度与算力。

## 图像和 RGB-D

```python
from semantic_label_seg import YoloSegmenter, CameraIntrinsics, rgbd_to_points

model = YoloSegmenter('/absolute/path/yolov8n-seg.pt', device='cpu')
result = model.segment(bgr_uint8_image)
points = rgbd_to_points(aligned_depth, CameraIntrinsics(fx, fy, cx, cy))
```

默认采用 SLIDE-SLAM 室内映射：`chair=1, dining table=2, tv=3`。其他类别为 0。
`config/image_classes.yaml` 可扩展类别；标签空间应与应用统一。
实例 ID 仅在当前图像内有效，不代表跨帧对象 ID。重叠掩码由较高置信度覆盖。
必须使用分割权重（如 `*-seg.pt`）；仅有检测框的模型不提供像素级类别。

```bash
ros2 run semantic_label_seg rgbd_seg_node --ros-args \
  -p model_path:=/absolute/path/yolov8n-seg.pt \
  -p rgb_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
```

也可使用 `rgbd.launch.py params_file:=...`。需要预先去畸变且与 RGB 对齐的深度、匹配分辨率的 CameraInfo，
三者应使用同一相机光学坐标系。使用 CameraInfo 的有效 P 内参，P 未设置时退回 K；本包不执行图像去畸变或深度配准。
`16UC1` 乘 `depth_scale`（默认毫米转米），`32FC1` 已是米，不重复缩放。
类别图、实例图使用 `32SC1`；分数图使用 `32FC1`；RGB-D 点云中另有 `instance_id: UINT32`。
标签图保留 RGB 时间戳，点云保留深度时间戳；两者由 `sync_slop` 限定近似同步。
无效深度以 NaN 保留对应像素，其点云类别、实例和分数置零。

## 接 Hydra

本包生成可供 PointcloudReceiver 读取的 `label: UINT32`，不会把反射强度或扫描线 ring 当类别。
必须将本包类别空间映射到 Hydra 的 label_space，并配置 object/dynamic/invalid 等类别组。
不同模型输出的数字即使相同，也不意味着语义相同。YOLO 的对象分割不会自动产生墙、地面等完整背景标签。
标签值为未知时，是否还参与 Hydra 几何积分由其配置决定。

接点云还需要独立提供传感器模型和真实传感器位姿；有标签不等于已打通 Hydra 运行管线。

## 测试

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SuperLIO_GLIM_ROS2/install/setup.bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2/src/semantic_label_seg
python3 -m pytest test -q
```

测试覆盖球面投影及原点索引、深度单位、类别/实例掩码重叠、真实 PyTorch 网络的合成权重载入/反投影、
缺失权重报错，以及 PointCloud2/Image 大小端、行填充、时间戳、附加字段保留等契约。
还覆盖 RGB 打包、标定旋转的方向/行主序/180 度情形，以及 world 系输入等待 TF 后推理并保留输出坐标的 ROS 消息传输。

本机已通过全部 22 项测试，并用已有 UrbanNav VLP32 rosbag 对联合启动做了短时检查：
Super-LIO 正常输出逐帧稠密点云，分割节点经完整 TF 链输出带 `label/confidence/rgb` 的 world 系点云。
该检查关闭 RViz，使用合成权重，只验证节点与消息接线；真实类别精度和 GUI 显示效果仍需用实际权重检查。
