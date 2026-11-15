# 残差点着色与直方图诊断

建图 RViz 的 `current (plane blue)` 显示实际参与最后一次 ESKF 线性化的查询点：蓝色为点到面；未采用的点不包含在这个显示里。当前已移除点距离观测。累积地图使用 GenZ-LIO 风格的半透明白色背景（`FlatColor`、透明度 0.2、1 像素、`Flat Squares`）。逐帧世界点云仍发布在 `/lio/cloud_world`，LIO 预设以 `Decay Time: 100000` 在 RViz 中累积，重开 RViz 后从新收到的帧开始显示。回环预设的 `/lio_slam/map` 是完整优化地图，采用相同外观，但保持 `Decay Time: 0`，每次替换以避免残留回环前的地图。重定位预设维持原来的显示和局部地图清除逻辑。

这里只记录**实际采用的平面观测**；近邻不足、平面拟合失败或残差门限失败的点均丢弃。最近地图点距离仅用于分析平面支撑，不构成观测。

## 运行

重新编译安装后，重启算法和 RViz。终端 A：

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source install/setup.bash
ros2 launch super_lio_slam vlp16.launch.py enable_loop_backend:=false \
  downsample_mode:=adaptive rviz:=true
```

终端 B，先启动统计，再播放 bag：

```bash
cd /home/ros/ros_ws/SuperLIO_GLIM_ROS2
source install/setup.bash
python3 tools/plot_observation_residuals.py record \
  --output diagnostics/narrowwide_plane_01 --live
```

`--live` 每 5 秒传感器时间刷新 Matplotlib 图窗，直方图默认显示最近 15 秒（`--live-window-s`），覆盖更新 `_live.png`；去掉它可减少绘图开销，停止后统一生成全程图片。`--duration 150` 表示记录 150 秒传感器时间后自动停止；`--wall-timeout` 可限制实际等待时间。已有记录目录不会被覆盖，重复测试请换新目录。

终端 C：

```bash
ros2 bag play ~/dataset/narrowwide/vlp16_handheld_a_01/ \
  --topics /velodyne_points /vectornav/IMU --rate 1.0
```

统计进程 Ctrl+C 后生成：

- `frames.csv`：每帧平面残差数量、RMSE、P95、权重总和、加权残差平方、最近地图点距离、信息方向性、位姿校正量及自适应采样状态。
- `histograms.csv`：每帧各类型的直方图；保留溢出计数，不把超出绘图区间的值静默丢弃。不会在内存里累计全程逐点数据。
- `residual_histograms.png`：更新前 / 后绝对距离残差、最近地图点距离、加权残差、信息权重等直方图。
- `observation_timeline.png`：残差数量、权重与代价占比、位姿校正、自适应边长、信息特征值比及估计轨迹。
- `report.md`：数值汇总和大校正帧列表。

针对漂移前后的区间重新作图，例如：

```bash
python3 tools/plot_observation_residuals.py plot \
  --output diagnostics/narrowwide_plane_01 --start-s 110 --end-s 140
```

时间是相对于**记录到的第一帧观测**的传感器时间，不是 ROS 日志的系统时间。报告同时保留绝对传感器时间。应在播放 bag 前启动记录，便于多次回放对齐。

## 统计口径

`residual` 和 `information` 对应最后一轮真正送入 ESKF 的线性化点。点到面保留有符号距离，直方图显示绝对值。`residual_after` 保持原匹配目标，仅用本帧最终更新位姿重算，不重新匹配，因此可以检查最后一次校正是否真的减小这些残差。

`nearest_distance` 是最终位姿下查询点到缓存的最近地图代表点的欧氏距离。点到面可能因切向距离较大而仍有很小的法向残差；这个量用于辅助发现平面支撑过远 / 外推风险，不能直接当作平面匹配错误的判据。

每点用原代码实际权重累加 `H=Σ w JᵀJ`，平面信息权重保持 1000。工具保留旧 CSV 和 v1 ROS 协议的点距离统计字段以便读取历史记录；当前节点的 `points` 数量和对应占比恒为 0，新记录只绘制平面残差。

位姿 H 的转动和平移列量纲不同，因此先按对角线归一化再计算特征值比；同时提供平移子块的最小 / 最大特征值比及最弱方向。小比值提示方向约束弱，不等同于已完成可观性证明；平移子块也没有消除旋转耦合。低残差或 `unused=0` 都不能证明轨迹正确。

## ROS 数据

- `/lio/observation/points`：`PointCloud2`，`world` 系，SensorDataQoS（Best Effort），一帧只含实际采用的观测点。包含 `xyz/rgb/residual/residual_after/information/nearest_distance/jrx..jtz/kind/source_index`；当前仅发布 kind 1（平面）；kind 2 仅在旧版记录中存在。坐标和 `residual_after` 使用最终位姿，J 和 `residual` 属于最后线性化。`source_index` 对应本帧估计点云索引。
- `/lio/observation/metrics`：`Float64MultiArray`，首个 layout label 用 `superlio_observation_v1:` 加逗号分隔的列名描述数据，包含同一传感器时间戳、数量、边长、尺度、位姿校正、速度、协方差迹及 IMU 模长；保留的 `points` 列恒为 0。

只有 RViz 或统计程序订阅时才采集、复制、发布诊断样本；没有订阅者时保留原算法路径。统计程序不向算法写参数。录制采用流式 CSV，ROS 诊断链路可能掉帧，应结合 CSV 的时间间隔检查；它不是无损 rosbag 记录。

重定位也复用该采集路径；Local → World 接管会改变地图参考，在分析跨接管轨迹时需区分两个阶段。当前可视化坐标发布于观测更新之后、重定位提交之前，接管附近的一帧可能仍处于临时世界显示约定。

## 历史混合观测验证记录（2026-11-13，移除前）

已编译安装 LIO / 重定位节点，7 项相关 CTest 与两项 Python 统计、带行填充点云解码测试通过；实际 ROS 消息检查了蓝 / 红 RGB 编码、world 坐标与分类数量。使用 NarrowWide 前 155 秒完成 hybrid / point_to_plane 同配置对照，并保留了地图分辨率副本的负面对照。上述过程没有修改生产观测权重、接受门限或地图分辨率。

当前点距离分支已删除；上述 hybrid 对照记录只描述移除前的版本，不代表当前还支持该模式。

## 移除点距离后的验证（2026-11-14）

已重新编译安装 LIO 和重定位节点。6 项 CTest（自适应采样、采样集成、平面观测及诊断、重定位状态、固定 / 自适应重定位运动）和两项 Python 统计 / 解码测试通过。观测测试确认孤立近邻不产生更新，正常平面可修正法向位移并更新协方差，诊断点云仅含蓝色 kind 1，v1 `points` 数量恒为 0。旧版纯平面记录已验证可生成仅含平面曲线的直方图；本次未重新回放完整数据集评估漂移。
