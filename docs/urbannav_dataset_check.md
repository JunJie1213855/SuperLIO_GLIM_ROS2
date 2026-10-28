# UrbanNav 数据适配检查

日期：2026-10-27。数据目录：`/home/ros/dataset/urbannav/2020-03-14-16-45-35`。

## 实测数据

| 项目 | 结果 |
| --- | --- |
| bag 大小/时长 | 25.1 GiB / 300.752 秒 |
| 点云 | `/velodyne_points`，3005 帧，约 10 Hz |
| IMU | `/imu/data`，60152 条，约 200 Hz |
| 点云字段 | float32 x/y/z/intensity、uint16 ring；point_step=32 |
| ring | 第一帧覆盖 0–31，符合 32 线输入 |
| 抽样帧点数 | 64808–65578，抽查前 30 帧 |
| 逐点时间 | 未发现 time/t/timestamp 字段 |
| 消息时钟 | LiDAR 与 IMU header 同一 epoch；无需 Robosense 的设备 uptime 平移 |
| header 与录制时刻差 | 点云约提前 21–52 ms；IMU 约提前 0.1–3.6 ms，前段抽样 |
| IMU 加速度 | 前 1000 条模长均值约 9.801 m/s²，含重力 |

header 与 bag 时刻差不是 LiDAR–IMU 标定时间偏差，不能直接用来补偿外参/时间。前帧 azimuth 顺序单调扫过约一周，可用于后续评估角度重建逐点时间，但这只能是近似，必须核对扫描起止和转速。

`ros2 bag info` 和选定话题的短时播放、订阅已通过；未修改 bag 数据或 metadata。

## 已修正

1. `vlp32.yaml` 原来使用类型 3（VELO16），已改为 4（VELO32）。
2. 后端编译使用 GTSAM 4.3a1，而普通 ROS 环境优先找到 GTSAM 4.2，导致 `NonlinearFactor::rekey` 未定义符号。CMake 记录所选库目录，公共 launch 只为后端进程设置正确搜索优先级。已在父进程显式优先 ROS 4.2 的条件下启动 VLP32，并从 `/proc` 确认后端加载 4.3a1。

## 尚不能确认可正确建图

当前 Velodyne 分支通过 `pcl::fromROSMsg` 读取 float32 time，并以其作为逐点秒偏移。该数据缺少这个字段，因此不能假定点时间有效，也不能把点时间设为零后宣称完成了去畸变适配。

需要后续补充经验证的逐点时间重建，或获取带逐点时间的原始输入。当前配置中的单位 LiDAR–IMU 外参也尚未由该数据的标定文件核实。本次没有执行完整地图质量/回环精度验收。

## 播放方式

在缺失时间问题解决前，下列命令只说明正确的 ROS 仿真时钟和话题选择方式，不表示已经完成点云适配：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/SuperLIO_GLIM_ROS2/install/setup.bash
ros2 launch super_lio_slam vlp32.launch.py \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true
```

另一个终端：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/ros/dataset/urbannav/2020-03-14-16-45-35/ \
  --clock --topics /velodyne_points /imu/data
```

[机器可读抽样结果](validation/2026-10-27/urbannav_inspection.json)。
