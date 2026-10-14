# Unitree L1：SuperLIO 分段建图并合并到 GLIM

本文记录 /home/ros/dataset/unitree/firstpart 与 secondpart 的实际处理流程：分别用 SuperLIO 建图并运行闭环后端，导出关键帧点云和最终优化位姿，转换成 GLIM dump，再通过 GLIM Offline Viewer 对齐和合并。

> 当前工作区：SuperLIO 在 /home/ros/ros_ws/super_lio_slam，GLIM 在 /home/ros/ros_ws/mapping_merge_ws。运行 GLIM 程序时必须让 /usr/local/lib 排在 LD_LIBRARY_PATH 前面，否则 ROS Humble 的 GTSAM 4.2 会抢占编译所用的 GTSAM 4.3a1。

## 1. 数据和坐标约定

两个输入都是 ROS 2 SQLite bag，包含 /unilidar/cloud (sensor_msgs/PointCloud2) 与 /unilidar/imu (sensor_msgs/Imu)。firstpart 时长约 261.4 秒、2587 帧点云；secondpart 时长约 305.0 秒、3018 帧点云。两段记录时间约重叠 130.7 秒，为配准提供重叠区域。

本例使用 super_lio_slam/launch/unitree_slam.launch.py、config/unitree_l1.yaml 和 config/unitree_slam.yaml。Unitree L1 的点云 time 字段按秒解释；传感器外参是当前配置中的初始值，精度要求高时应先按实物标定。

数据链路的坐标关系：

- /lio_slam/keyframe 的 cloud 是去畸变后的 IMU 局部系点云。
- /lio_slam/path 是后端输出的最终优化轨迹，header.frame_id 为 map，每个 pose 与关键帧时间戳对应。
- 导出器用 T_map_imu 变换局部点云，生成 map 系 PCD；同时将同一 T_map_imu 写进 TUM 位姿文件。
- GLIM 转换器据此设单位 T_lidar_imu，因为输入 PCD 已在世界系。

## 2. 分两次运行 SuperLIO

每段必须启动一套新的 SuperLIO 和 loop backend，让关键帧编号与图优化状态从头开始。使用模拟时钟回放 bag，录制后端关键帧与路径。

终端 A：

    source /opt/ros/humble/setup.bash
    source /home/ros/ros_ws/super_lio_slam/install/setup.bash
    ros2 launch super_lio_slam unitree_slam.launch.py use_sim_time:=true rviz:=true

终端 B：

    source /opt/ros/humble/setup.bash
    source /home/ros/ros_ws/super_lio_slam/install/setup.bash
    mkdir -p /home/ros/dataset/unitree/superlio_results
    ros2 bag record -o /home/ros/dataset/unitree/superlio_results/firstpart_result \
      /lio_slam/keyframe /lio_slam/path

终端 C：

    source /opt/ros/humble/setup.bash
    ros2 bag play /home/ros/dataset/unitree/firstpart --clock --rate 0.5

等待回放结束，并等后端处理完最后关键帧、/lio_slam/path 不再更新。停止结果录制，再停止 launch。然后重新执行终端 A、B、C，把结果目录改成 secondpart_result、bag 路径改成 secondpart：

    ros2 bag record -o /home/ros/dataset/unitree/superlio_results/secondpart_result \
      /lio_slam/keyframe /lio_slam/path

    ros2 bag play /home/ros/dataset/unitree/secondpart --clock --rate 0.5

不要在同一后端进程中连续跑两份 bag；那会把会话的关键帧编号和图状态混在一起。不要在路径还没更新完时结束录制，否则导出的轨迹可能不完整。

## 3. 导出关键帧 PCD 与 TUM 位姿

导出器位于 /home/ros/dataset/unitree/superlio_results/export_superlio.py。它读取结果 bag 中所有 /lio_slam/keyframe，并使用最后一条 /lio_slam/path 作为最终优化轨迹，按整数纳秒时间戳配对。对最终轨迹中每个 pose 执行：

    p_map = T_map_imu(t) * p_imu

分别导出两段；输出目录必须尚不存在，脚本会拒绝覆盖：

    source /opt/ros/humble/setup.bash
    source /home/ros/ros_ws/super_lio_slam/install/setup.bash

    python3 /home/ros/dataset/unitree/superlio_results/export_superlio.py \
      --bag /home/ros/dataset/unitree/superlio_results/firstpart_result \
      --out /home/ros/dataset/unitree/superlio_results/firstpart_export

    python3 /home/ros/dataset/unitree/superlio_results/export_superlio.py \
      --bag /home/ros/dataset/unitree/superlio_results/secondpart_result \
      --out /home/ros/dataset/unitree/superlio_results/secondpart_export

输出结构：

    firstpart_export/                 secondpart_export/
    ├── clouds/                        ├── clouds/
    │   ├── 000000.pcd                 │   ├── 000000.pcd
    │   └── ...                        │   └── ...
    ├── poses.txt                      ├── poses.txt
    └── export_manifest.json           └── export_manifest.json

poses.txt 每行是 timestamp x y z qx qy qz qw；000000.pcd 对应第一行，依此类推。manifest 记录每帧时间戳、SuperLIO keyframe ID 和点数。脚本遇到时间戳重复、最终轨迹缺关键帧点云、空点云或输出已存在时会报错。

本次导出结果：

| 结果 | 关键帧/导出帧 | 路径消息 | 点数 |
|---|---:|---:|---:|
| firstpart | 261 / 261 | 261 | 234,484 |
| secondpart | 317 / 317 | 317 | 267,668 |

## 4. 转为 GLIM dump

转换器位于 mapping_merge_ws/src/glim/tools/lio2glim.py。它根据 TUM pose 将 PCD 切成子图，估计每点协方差，并生成 GLIM 的 data.txt、紧凑点云、因子图与配置。每段单独生成 dump：

    python3 /home/ros/ros_ws/mapping_merge_ws/src/glim/tools/lio2glim.py \
      --poses /home/ros/dataset/unitree/superlio_results/firstpart_export/poses.txt \
      --clouds /home/ros/dataset/unitree/superlio_results/firstpart_export/clouds \
      --out /home/ros/dataset/unitree/superlio_results/firstpart_dump \
      --frames-per-submap 10 --voxel-size 0.2 \
      --config /home/ros/ros_ws/mapping_merge_ws/src/glim/config

    python3 /home/ros/ros_ws/mapping_merge_ws/src/glim/tools/lio2glim.py \
      --poses /home/ros/dataset/unitree/superlio_results/secondpart_export/poses.txt \
      --clouds /home/ros/dataset/unitree/superlio_results/secondpart_export/clouds \
      --out /home/ros/dataset/unitree/superlio_results/secondpart_dump \
      --frames-per-submap 10 --voxel-size 0.2 \
      --config /home/ros/ros_ws/mapping_merge_ws/src/glim/config

本次设置生成 firstpart 27 个子图、secondpart 32 个子图。首次运行会编译 tools/glim_graph_writer.cpp，需要 g++ 和 GTSAM/gtsam_points 开发文件；脚本会为辅助进程前置 /usr/local/lib。

转换后检查每个 dump 都有 graph.txt、graph.bin、values.bin、config/，并且每个编号子目录含 data.txt、points_compact.bin、covs_compact.bin。

曾发生过转换器移动到 src/glim/tools 后仍使用旧目录计算 export_factors.cpp 路径，导致点云子图写完，但 graph.bin、values.bin、config/ 未生成。GLIM 仍能显示子图点云，却在合并创建涉及 x27 的因子时因 Values 缺少该 key 而崩溃。修复后的脚本会先定位并编译 graph writer，再处理点云。若已有半成品，不要直接拿它合并；应重新生成完整 dump，或从各子图 data.txt 补齐 graph/values 并复制 config。

## 5. 在 GLIM 中合并

新终端加载 ROS 和 GLIM，并指定 GTSAM 4.3：

    source /opt/ros/humble/setup.bash
    source /home/ros/ros_ws/mapping_merge_ws/install/setup.bash
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

打开 firstpart：

    ros2 run glim_ros offline_viewer \
      /home/ros/dataset/unitree/superlio_results/firstpart_dump

界面操作顺序：

1. 首次打开选择启用优化。
2. File → Open Additional Map，加载 secondpart_dump。
3. 点 Merge sessions，选择 Indoor/Outdoor 配准参数。先运行全局粗配准；若结果不稳，手动移动 source（绿色）到 target（红色）重叠区域。
4. 点 Run fine registration 执行 ICP/GICP 精配准。检查重叠区域，不要只看 inlier rate；变换位置、方向和点云贴合都应合理。
5. 点 Create Factor 建立两个会话之间的锚点约束。
6. 点 Find overlapping submaps 建立重叠子图的 VGICP 匹配代价因子。
7. 点击 Optimize 数次，观察地图是否保持对齐并改善全局一致性。

两个分段各有自己的初始 map 原点，由配准估计会话间变换。当前粗配准日志里的 T_target_source 是 source 到 target 的变换；target_id、source_id 表示建立会话约束的锚点子图。

成功判据：Create Factor 后没有 does not exist in Values / out_of_range 崩溃；两个会话在重叠区域贴合；Find overlapping submaps 找到候选并完成优化；viewer 中能同时查看两段地图。保存合并后的结果到新的 dump 目录，保留原始两个 dump。

注意：当前记录证明两段 dump 已补齐且变量存在；最终 GUI 合并还需按上述步骤完成。不要把粗配准成功日志等同于整体合并完成。

## 6. 常见问题

- libgtsam_points_cuda.so.1: undefined symbol ... rekey：新终端忘记 export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH。
- 转换器生成子图后报找不到 export_factors.cpp：使用了旧版 lio2glim.py；更新后重跑，或从既有 data.txt 补齐 graph/values 和 config。
- x27 does not exist in the Values：第二会话 dump 的 values.bin 缺失或不含第二会话 X pose。重新生成完整 dump。
- 最终轨迹没覆盖所有 keyframe：等 SuperLIO 后端处理完再结束录制；重新录制结果 bag 并导出。
- 配准的 inlier rate 高但地图仍错位：检查重叠区域的点云结构、方向和轨迹；调整 source 粗对齐，再运行 fine registration。

