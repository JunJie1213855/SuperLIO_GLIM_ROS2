# 多帧回环首版实现与验证

日期：2026-10-27。实现为可选模式 `std_teaser_gicp`，默认保留 `std_icp`。

## 已实现

- STD 对单关键帧提取描述子，提供未经旧位姿硬门限筛选的历史候选。
- deque 管理因果窗口，按帧数、时间、累计路程裁剪；原始单帧点云保留用于地图及 GLIM。
- 两侧子图分别锚定当前/历史中心帧，原始 odom 相对位姿对齐；拒绝共享成员。
- 子图 FPFH 互选/距离比筛选、有限数量且空间分布充分的对应点进入固定尺度 TEASER++。
- 可选 STD 几何提示限制匹配的宽邻域；提示失败重试无提示版本，不阻止原始候选进入求解。
- GICP 使用粗变换作为初值，独立质量指标验证后将完整相对变换加入原 iSAM2 后端。
- 保留轨迹发布和 map→world 更新；后续已实现独立配准与地图工作线程，详见本文异步流水线章节。历史特征缓存仍未实现。
- 依赖构建脚本固定上游及 PMC/tinyply/Spectra 版本；支持完整缓存下离线构建。

## 构建与单测

`super_lio_slam` 构建通过；TEASER++ 固定版本构建安装通过，缓存离线构建通过。

`ctest --test-dir build/super_lio_slam -R submap_registration_test --output-on-failure` 通过。
覆盖：锚变换方向、当前帧单位位姿、队列快照、帧数/时间/累计路程裁剪、时间倒退拒绝、共享帧、非法刚体变换、真实 FPFH→TEASER→GICP 已知变换、点数不足和低覆盖拒绝、单平面退化拒绝。

## 实际 ROS 后端测试

| 场景 | 参数 | 结果 |
|---|---|---|
| 新模式静态重访 | 4 帧、默认内点比例 0.3 | 通过；1.000 m 人工漂移降至 0.3686 m |
| 新模式非零平移/90°旋转 | 2 帧、内点比例 0.2 | 通过；1.000 m 人工漂移降至 0.1371 m |
| 新模式强制点数不足 | 4 帧，registration_min_points=100000 | 通过；无回环，1.000 m 原始漂移保留 |
| 旧模式非零平移/90°旋转 | std_icp | 通过；1.000 m 人工漂移降至 0.0097 m |

上述最终 ROS 测试额外断言历史关键帧确实被优化、Path 与优化 Odometry 一致、地图非空且处于 map、map→world 与同一帧位姿组合一致。

### 已知压力场景限制

新模式使用默认 4 帧、0.3 内点比例运行 `--moving` 时，未通过正例预期：拒绝原因为 `few_coarse_inliers`。该合成场景每个关键帧增加 0.25 m 平移漂移及 0.02 rad 偏航漂移，远处结构在累计子图内产生明显非刚性错位。

离线检查表明，其粗配准能接近正确旋转方向，但四帧平均几何使中心帧位姿存在偏差。该失败没有通过修改默认门限或更改原测试的 0.5 m 误差断言掩盖。上表旋转通过结果明确使用两帧和 0.2 比例的实验配置，不代表默认四帧方案在该压力场景通过。

这也是保留旧默认模式、要求按传感器标定窗口及噪声参数的原因。累计点数更多并不自动带来更高精度。

## Robosense 完整回放

数据：`/home/ros/dataset/robosense/mapping`，约 98 秒，1 倍速。使用 `robo_slam.launch.py`、`use_sim_time=true`、新模式默认运行参数。

- 录制 50 条关键帧、50 条优化 Path，最终 Path 含 50 个位姿；等待后端处理完再停止。
- 接受约束 0→11、1→31；当前源窗口均为 4 帧。

| 约束 | 源/目标帧数 | 粗内点/对应点 | GICP 内点 MSE (m²) | 源覆盖率 | 粗/精耗时 (ms) |
|---|---|---|---|---|---|
| 0→11 | 4/1 | 94/160 | 0.0807 | 0.909 | 135.8/29.5 |
| 1→31 | 4/2 | 11/24 | 0.1270 | 0.827 | 144.5/35.9 |

31 号帧还拒绝了其他候选，全部尝试合计约 0.64 秒；500 ms 是候选间检查的软预算，不是硬截止。运行耗时随点数和负载变化，不能由该短回放宣称硬实时。

这里记录的是算法接受的约束，没有真值支持其回环精确率或绝对轨迹精度。未完成 Unitree 长序列、独立实际负例集和多会话人工配准验收。

## 导出与 GLIM 格式回归

完整回放结果通过 `src/tools/export_superlio.py` 导出：50 个单帧点云、50 个 TUM 位姿，共 302903 点，无缺失关键帧。

工具迁移到 `src/tools/` 后，`lio2glim.py` 的 GLIM 源码/config 路径定位已改为向上查找工作区，兼容 `tools/` 和 `src/tools/`。

按每子图 10 帧转换后生成 5 个 GLIM 子图，共 10986 点，以及非空的 graph.bin、values.bin、graph.txt 和 config。未通过图形界面执行多会话合并，因此本项证明格式生成链路，不声称已验证全部 Offline Viewer 交互。

## 可复查记录

[机器可读报告](validation/2026-10-27/registration_report.json) 包含参数来源代码哈希、数据 metadata 哈希、接受/拒绝日志和测试结果；临时 bag、PCD、dump 及完整日志的路径记录在其中。临时产物不作为源码数据提交。

更多坐标推导见 [局部子图数学说明](loop_submap_registration_math.md)，运行方式与参数见 [包说明](../src/SuperLIO_SLAM/README.md)。

### STD admission and concise logs (2026-10-27)

`std_teaser_gicp` now requires `CandidatePoseHint` to pass STD triangle consistency
(at least four votes and a finite transform) before attempting registration. This
is a triangle-consistency match, not proof of a true loop or the legacy plane-score
acceptance gate. Failed STD candidates and per-keyframe search summaries are DEBUG.
INFO reports matched candidates, registration decisions and final accepted loops.
The hinted attempt may still retry unguided; both attempts must pass all quality gates.

Coarse registration retains the minimum inlier count, minimum inlier fraction and
spatial spread checks, and additionally rejects inlier RMSE above
`coarse_max_inlier_rmse_m` (default 0.3 m). RMSE is computed over descriptor-pair
residuals within `coarse_residual_threshold_m`; it is not ground-truth pose accuracy.
Only passing coarse results enter GICP; existing fine-registration and observability
gates remain active. Ordinary logs omit uncomputed GICP quality values when skipped.

Validation of this change: build and CTest passed (including an explicit coarse
RMSE rejection case). The original four-frame synthetic smoke fixture no longer
accepts a loop with the stricter STD admission and default quality thresholds;
its final candidate had only 24/200 coarse inliers. Two-frame synthetic data with
inlier fraction 0.2 still rejects at RMSE limit 0.3 m (measured about 0.304 m).
With explicitly configured limit 0.35 m and the same two-frame/0.2 settings, the
online pipeline accepts loops and reduces artificial 1 m drift to about 0.134 m.
These test overrides do not change runtime defaults and are not UrbanNav tuning.
A forced insufficient-points run preserves the original 1 m drift and skips GICP.
INFO log checks now assert absence of per-frame search counts and presence of
verified STD matches and registration decisions. Logs are in
`/tmp/superlio_quiet_loop_positive.log` and `/tmp/superlio_quiet_loop_rejection.log`.

### 异步流水线验证（2026-10-27）

当前实现见 [async_loop_backend.md](async_loop_backend.md)，结构化结果见
[async_backend_report.json](validation/2026-10-27/async_backend_report.json)。

- 编译及 CTest 通过；VLP32 launch 参数和 GTSAM 4.3a1 实际运行库检查通过。
- 真实 ROS 突发测试：24 个关键帧/图节点全部保留，入口处理约 0.62 s；第一个配准任务耗时约 813 ms。观察到有界检测队列淘汰旧任务，迟到回环提交时 `latest=23`，轨迹未被截断或回退。
- 同一突发测试将地图每关键帧点数上限设为 30000、体素设为 0.1 m，检查最终异步地图包含最新帧时间戳。
- 新模式合成正例：1 m 人工漂移降至约 0.1339 m；旧模式含 90° 转动正例降至约 0.0097 m，旋转方向检查通过。
- 强制点数不足：没有加入回环，1 m 原始漂移保留。
- 新模式成功测试采用 2 帧、粗内点比例 0.2、粗 RMSE 上限 0.35 m；没有修改运行默认的 4 帧、0.3 和 0.3 m。不是 UrbanNav 的调参或精度结论。

### 回环两端时间间隔限制

默认参数 `loop_min_time_separation_s: 30.0`，新旧两种配准模式共同生效。
编译及 CTest 通过；ROS 集成测试在两种模式下各验证三组传感器时间差：
29,999,999,999 ns 拒绝、30,000,000,000 ns 拒绝、30,000,000,001 ns 接受。
拒绝时五个关键帧和图节点仍保留，原始里程计未被回环校正。
这些测试使用同一重复场景，只有时间戳条件不同；大于门限的用例仍需通过实际配准质量门限。
结果见 [loop_time_gate_report.json](validation/2026-10-27/loop_time_gate_report.json)。

### 距离候选检索和 UrbanNav 实测

`loop_candidate_mode` 默认改为 `distance`（15 m 半径），STD 仍可显式启用。
编译、CTest、VLP32 启动及公共 launch 参数检查通过。合成测试验证距离检索
在新旧配准模式均能产生约束，并验证 30 秒时间门限、距离半径拒绝及 STD 回归。

UrbanNav 数据 `2020-03-14-16-45-35`，只回放点云/IMU、`--clock --rate 3.0`：

- 默认粗 RMSE 上限 0.3 m，完整 bag：714 个后端关键帧、无 ID 跳号，44 次候选配准，0 条接受回环。41 次内点不足；其余因粗 RMSE 超限拒绝。例如 67/200 内点、RMSE 0.341 m。
- 独立试验从 bag 第 100 秒重新初始化 LIO，粗 RMSE 上限设为 0.4 m；其他配准门限不变。513 个关键帧、无 ID 跳号、21 次候选配准，接受 3 条回环。其 GICP MSE 为 0.0962、0.0919、0.1007 m²，覆盖率分别为 0.946、0.956、0.932。
- 此片段试验与整包初始状态不同，不作为单变量因果对照，也没有真值 ATE/RPE 或回环标签审计。
- 新增可选 `config/loop_distance_teaser_gicp.yaml` 复现 0.4 m 粗 RMSE 门限；原默认配置仍为 0.3 m。逐点时间缺失及未验证的外参没有在本次修改中解决。

结构化记录及日志位置见
[distance_candidates_report.json](validation/2026-10-27/distance_candidates_report.json)。
