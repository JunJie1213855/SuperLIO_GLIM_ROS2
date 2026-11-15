# GenZ-LIO 自适应体素滤波接入方案

状态：已实现 `fixed` / `adaptive` 两种模式；普通 LIO 和重定位共用采样模块。核对日期：2026-11-13。

运行：传感器 YAML 设置 `lio.downsample.mode: adaptive`，或 launch 追加 `downsample_mode:=adaptive`；默认 `fixed`，不传 launch 参数时保留 YAML 的选择。模式和自适应控制参数为启动参数，切换需重启节点。

## 1. 目标与依据

在 SuperLIO 的扫描降采样阶段复现 GenZ-LIO 的尺度感知、增益调度 PD 和双分辨率点云输出。保持 SuperLIO 的 IMU 传播、ESKF 点到面观测、OctVoxMap、回环和地图合并架构。

依据：

- [本地论文](/home/ros/文档/SLAM/LIO/GenZ-LIO.pdf)：第 IV 节、Algorithm 1（第 7 页）、实验参数（第 15 页）。
- [用户笔记](/home/ros/文档/note/应用学科/SLAM/2.激光雷达SLAM/11.GenZ-LIO/1.简介.md)：第 2 节。
- 上游固定版本 `bf444f7a747ed3da8548b56b49c14f207e8d49bb`：
  [控制器](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/ScaleAwareVoxelizer.cpp)、
  [降采样](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/Downsample.cpp)、
  [参数](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/Config.hpp)、
  [主流程](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/GenZLIO.cpp)。

复现范围为扫描采样模块，不能据此宣称复现完整 GenZ-LIO 或其精度结果。混合残差、协方差模型和搜索剪枝属于其他模块。

## 2. 代码与接入点

当前正常帧流程：

```text
Propagation_Undistort() -> DownSample() -> Observe() -> UpdateMap()
```

关键位置：

| 位置 | 接入前行为 | 本次实现 |
|---|---|---|
| `src/SuperLIO_SLAM/src/lio/super_lio.cpp::DownSample` | 固定体素，输出 `ds_undistort_` | 增加模式分支和自适应滤波器 |
| `SuperLIO::Observe` | 用 `ds_undistort_` 建立残差 | 继续使用较稀的 estimation 点云 |
| `SuperLIO::UpdateMap` | 使用 Observe 的 `points_body_v3_` 插入地图 | 自适应模式从独立 mapping 点云转换并插入 |
| `include/lio/super_lio.h` | 一个滤波结果，部分缓存固定 20000 | 新增 mapping 缓存，观测工作缓存按实际点数扩容 |
| `SuperLIOReLoc` | 复用 DownSample/Observe/UpdateMap，另有 limitScan | 初始化和正常帧同步接入，处理截断与缓存关系 |

当前 `lio.sensor.enable_downsample` 被加载，但 `DownSample()` 没有使用它进行分支控制。新模式使用明确参数，不依赖这个尚未接通的旧开关。

## 3. 每帧算法

设输入为本帧去畸变扫描 \(S_t\)，当前控制状态为上一帧边长 \(d_{t-1}\)。

### 3.1 临时降采样与尺度

\[
V_{temp,t}=Voxelize(S_t,d_{t-1}),\quad N_{temp,t}=|V_{temp,t}|.
\]

在雷达局部系计算点的量程中位数，最近 5 帧取均值：

\[
m_t=median\{\|p_L\|:p_L\in V_{temp,t}\},\qquad
\bar m_t=\frac1{|W|}\sum_{j\in W}m_j.
\]

与上游一致，可对平方距离用 `nth_element`，再对选中的中间元素开平方。偶数点采用上中位数，以便逐帧对齐源码结果。

### 3.2 场景尺度决定点数预算

\[
\phi_t=\min(\bar m_t/\tau_m,1),\qquad
N_t^*=N_{min}+(N_{max}-N_{min})[1-(1-\phi_t)^p].
\]

默认 \(N_{min}=1000,N_{max}=4000,\tau_m=30m,p=2\)。例如尺度为 5/15/30 m 时，目标约为 1917/3250/4000 点。

这是尺度相关的点数预算；它不直接衡量几何可观性，也不保证单帧输出等于预算。更大的场景目标点数更高，不应把“开阔处增大体素”硬编码为控制方向。

### 3.3 增益调度 PD

实现采用上游源码的误差符号，避免与代码对照时混淆：

\[
e_t=N_{temp,t}-N_t^*,\qquad \dot e_t=(e_t-e_{t-1})/\Delta t.
\]

\[
\psi_p=\min\left(\frac{|e_t|}{\lambda_pN_t^*},1\right),\quad
\psi_d=\min\left(\frac{|\dot e_t|}{\lambda_dN_t^*/\Delta t},1\right).
\]

\[
K_p=K_{p,min}+(K_{p,max}-K_{p,min})\sqrt{\phi_t\psi_p},
\]
\[
K_d=K_{d,min}+(K_{d,max}-K_{d,min})\sqrt{\phi_t\psi_d}.
\]

\[
d_t=clamp(d_{t-1}+K_pe_t+K_d\dot e_t,d_{min},d_{max}).
\]

论文和笔记采用相反的误差定义，因此更新使用减号；两种写法等价，不能混用符号。\(e_t>0\) 表示点数过多，比例项增加体素边长。

\(\Delta t\) 使用相邻扫描结束时刻之差，单位秒，不使用程序耗时或 ROS 当前时间。正常首帧按上游使用 0.1 s、前次误差 0。每帧只更新一次，不在同一帧循环搜索点数。

### 3.4 重新从完整扫描生成双分辨率点云

\[
V_{map,t}=Voxelize(S_t,d_t/2),\qquad
V_{est,t}=Voxelize(V_{map,t},d_t).
\]

```text
完整去畸变扫描 S_t
  ├─ 旧边长 d_(t-1) -> probe -> 尺度、点数、PD -> 新边长 d_t
  └─ 新边长 d_t/2  -> mapping -> 新边长 d_t -> estimation
                         │                            │
                         └─ 更新后位姿插入地图        └─ ESKF 观测更新
```

减小边长时，必须回到完整扫描重新滤波，不能继续滤波旧 probe，否则先前丢掉的细节无法恢复。

上游使用质心：每个体素输出 XYZ、强度的均值。虽然源码注释称 estimation 为 mapping 的子集，实际二次质心通常会产生新坐标，因此不能依赖严格子集或点索引对应关系。

## 4. 坐标系与网格定义

SuperLIO 的 `scan_undistort_full_` 已在帧末 IMU 系；上游采样与量程统计使用雷达局部坐标。令已知外参为 \(T_{I\leftarrow L}=(R_{IL},t_{IL})\)：

\[
p_L=R_{IL}^{\top}(p_I-t_{IL}).
\]

推荐在雷达系计算量程及网格索引，仍对输入 IMU 系点累加质心并输出：

\[
k=\lfloor p_L/d\rfloor,\qquad
\bar p_I(k)=\frac1{n_k}\sum_{i\in k}p_{I,i}.
\]

刚体变换与求质心可交换，因此可复现雷达系网格，而不改变 Observe 所需的 IMU 系输出。只修正量程、却继续在 IMU 轴向网格分箱，不是完全相同的采样定义。

不在世界系计算尺度；世界系原点随会话或重定位改变，不代表场景范围。重定位接管仅改变全局参考，IMU 局部输入约定未变，无需因此重置控制器。

地图根体素 `lio.hash_map.vox_resolution` 保持固定；改变扫描边长不能重新划分已存在的 OctVoxMap。

## 5. 不能原样复用的两处实现

### 5.1 质心与体素 key

现有 `VoxelGridClosest` 用 `round()` 分箱，保留最靠近中心的原始点；GenZ 用 `floor()` 分箱并计算质心。仅调用 `setLeafSize(d_t)` 可做工程变体，但不等同于上游采样。

现有 key 在三个索引加 1000 后位移、按位或，未完整处理负值。小体素下索引容易超出假定范围；例如加偏移前 x=-1001 的两个不同体素 `(-1001,0,0)`、`(-1001,250,100)` 在 64 位 key 上都变成全 1。边长 0.02 m 时，x=-20.02 m 就可能到达这种情况。

新滤波内核使用三元 `int64_t` key、完整坐标相等比较和哈希，不复制这种打包方式。哈希值相同由容器的键比较区分，不能只把哈希值当唯一体素身份。

### 5.2 20000 点观测缓存

接入前 `effect_mask_`、`effect_knn_mask_` 为固定数组，`abcd_vec_`、`effect_knn_idxs_` 只在初始化 resize 到 20000。控制器暂态、极密点云或边长饱和时，输出仍可能超过 20000；4000 不是硬上限。

两种模式均已改为按实际 estimation 点数扩容。并行写入的掩码使用 `vector<uint8_t>` 独立元素存储，不用存在位打包共享写问题的 `vector<bool>`。

重定位的 `limitScan()` 在 `fixed` 模式保留原有 20000 点限制，`adaptive` 模式跳过截断，使用动态缓存。若以后增加计算预算硬限点，须单独记录其触发与丢点数，明确属于工程扩展。

## 6. 类接口与数据归属

新增 `include/lio/AdaptiveVoxelFilter.h`、`src/lio/AdaptiveVoxelFilter.cpp`。纯 C++ / PCL / Eigen，直接复用已有哈希容器，不引入整个 GenZ-LIO 工程。

```cpp
struct AdaptiveVoxelOutput {
  BASIC::CloudPtr estimation; // IMU 系，交给 Observe
  BASIC::CloudPtr mapping;    // IMU 系，交给 UpdateMap
  AdaptiveVoxelStats stats;
};

AdaptiveVoxelOutput process(const BASIC::CloudPtr& deskewed_imu,
                            double scan_end_time,
                            const BASIC::SE3& imu_from_lidar);
void reset();
```

每个 LIO 实例持有自己的控制器、中位数平滑窗口及历史时间，不能使用函数级 static 共享控制状态。这里的窗口仅存量程标量，不存多帧点云。基础扫描仍保留在 `scan_undistort_full_`，原关键帧、回环及稠密显示的输入不随这次采样方案替换。

新增 `map_undistort_`。`UpdateMap()` 必须从它读取每个点并乘观测更新后的位姿；不能继续按 estimation 点数遍历 `points_body_v3_`。过滤后的前端 PCD 保存分支可使用 mapping；固定兼容模式令 mapping 与 estimation 指向同一个结果，保持原行为。

普通建图初始化目前使用原始启动扫描，上游首次建图也使用首次去畸变扫描；第一版保留此行为。重定位初始化已调用 DownSample，需要将初始化插图分支也改用 mapping。后续 Local 建图复用新 UpdateMap；先验 World 跟踪依然遵循 `g_update_map`，不得因自适应采样而打开先验地图写入。

SuperLIO 的采样后点型为 `PointXYZI`，已经完成按时间去畸变，故不需要照搬上游输出按 time_offset 排序。若将来更换为含时间字段的点型，再明确字段的聚合与排序规则。

## 7. 参数与对照模式

新增 `lio.downsample.mode`，仅支持 `fixed` 和 `adaptive` 两种模式：

| 模式 | 采样内核 | 边长 | 观测 / 入图 |
|---|---|---|---|
| `fixed`（默认） | 现有 closest / round | 原固定值 | 同一份点云，保留现有基线 |
| `adaptive`（GenZ-LIO） | centroid / floor | PD 自适应 | d_t / d_t/2 |

`fixed` 保留现有固定体素流程；`adaptive` 使用完整的 GenZ-LIO 自适应流程，包括场景尺度估计、PD 边长控制和观测 / 入图双分辨率点云。两组对比评估整套采样策略的效果。未指定模式时默认使用 `fixed`，保持已有配置的行为。

保留已有拼写 `lio.sensor.voxel_fliter_size` 作为固定边长及控制器初值，避免再增加第二个初值参数。

```yaml
lio.downsample.mode: adaptive
lio.sensor.voxel_fliter_size: 0.5  # d_0；沿用现有传感器配置也可以
lio.downsample.adaptive.window_size: 5
lio.downsample.adaptive.min_points: 1000
lio.downsample.adaptive.max_points: 4000
lio.downsample.adaptive.scale_threshold_m: 30.0
lio.downsample.adaptive.setpoint_exponent: 2
lio.downsample.adaptive.error_sensitivity: 0.1
lio.downsample.adaptive.error_rate_sensitivity: 0.2
lio.downsample.adaptive.p_gain_min: 5.0e-6
lio.downsample.adaptive.p_gain_max: 5.0e-5
lio.downsample.adaptive.d_gain_min: 5.0e-8
lio.downsample.adaptive.d_gain_max: 5.0e-7
lio.downsample.adaptive.min_leaf_size_m: 0.02
lio.downsample.adaptive.max_leaf_size_m: 1.0
```

这些值是论文起点，不是对当前 VLP32 / Airy、SuperLIO 地图结构和观测模型已验证的最优设置。`filter_rate`、盲区和最大量程在对比组之间保持一致；原始输入不足时，缩小体素也不能恢复预处理已经丢掉的点。

## 8. 边界处理与复现口径

正常输入按照上述公式、默认首帧 0.1 s 和原始增益执行，不额外加误差死区、平滑微分或每帧边长变化限幅，否则对比的不再是相同控制器。

必要的工程保护单独标记：过滤非有限 XYZ；无有效点时返回空结果并跳过观测/地图写入，不以空扫描的 0 距离污染尺度窗口；会话重启或扫描时间回退时清空历史。正常帧不使用系统时钟纠正扫描间隔。异常时间戳要记录，不能静默继续积累 PD 历史。

所有参数校验有限性与合法范围：正时间/正边长、min<=max、正点数、窗口>=1、p>1、正归一化系数、非负增益。d_min/2 也须能安全分箱。

## 9. 验证与实现顺序

1. 完成纯滤波内核和控制器：测试正负坐标、体素边界、XYZ/强度质心、外参、NaN/空输入及参数校验。
2. 用同一批人工窄场/宽场扫描与固定版本上游逐帧对照：probe 点数、量程中位数、目标、Kp/Kd、边长、两份点云；上游可处理的正常输入需在容差内一致。
3. 测试控制方向、边长上下界和场景切换；输出点数只要求趋势合理，不要求每帧等于目标，更不能假设单调收敛。
4. 改造动态观测缓存和两份点云数据流；用超过 20000 点的估计扫描做内存检查，并验证入图确实使用 mapping。
5. 在 VLP32 / UrbanNav、Airy 上分别运行 `fixed` 和 `adaptive` 两组同起点、同速率回放。记录有效平面数、状态更新耗时、滤波耗时、地图插入耗时、内存和轨迹质量；有真值时计算轨迹误差。先分别测试 LIO，再用相同回环配置进行 SLAM 对比。
6. 单独验证重定位 Local 建图、成功接管和只读先验地图跟踪。检查成功率、快照点数与内存，而不改动重定位配准门限。

每帧 `AdaptiveVoxelStats` 保留点数、尺度、目标、误差、Kp/Kd、边长、限幅与滤波耗时。终端按传感器时间每 5 秒输出一次 `Adaptive voxel: ...`，包含 `input/probe/estimation/mapping/skipped/median/scale/target/leaf/dt/filter/clamped`；不依赖 `/clock`。当前未添加 CSV 导出，整帧各阶段耗时继续使用 `lio.eva.timer`。非有限 XYZ / 越界索引会被丢弃并计入 skipped，非有限强度按 0 参与均值。

自适应增加了 probe 和双分辨率过滤，可能节省观测时间，也可能增加入图成本；需以总处理时间判断收益。当前复用 hash、probe 点云容量和量程数组，保持原始算法顺序；两份输出点云独立分配，后续根据回放测量再评估复用。

### 已完成验证（2026-11-13）

- `adaptive_voxel_filter_test`：用原版上游源码生成的 10 帧窄场 / 宽场结果对照点数、尺度、目标、误差、边长；并验证质心、负坐标、外参、无效点、控制方向、上下界、时间回退和空扫描。
- `lio_downsample_test`：固定模式与原滤波器逐点一致；自适应扫描 45000 点生成 22500 个观测点及 45000 个入图点，验证动态缓存和观测更新后独立变换入图。
- `relocation_state_test`、`relocation_motion_test`、`relocation_motion_adaptive_test`：状态 / 协方差 / 时间传递以及两种模式下的 Local → World 接管通过。
- 滤波内核通过 AddressSanitizer / UndefinedBehaviorSanitizer；三个入口的启动实测验证了显式模式、YAML 继承和 launch 覆盖 YAML。

LIO 与重定位节点已编译并安装到当前工作区。尚未完成 UrbanNav / Airy 整包精度和性能对比，合成测试结果不代表真实数据集收益。
