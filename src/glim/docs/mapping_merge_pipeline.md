# Mapping / Merge 各阶段处理流程

本文按数据流顺序拆解 GLIM 从原始帧到「多会话合并地图」的每个阶段，标注对应的源码位置与中间数据结构。适用于排查 dump 损坏、理解 `graph.bin/values.bin` 的作用，以及编写外部格式转换工具（如 `tools/lio2glim.py`、`tools/glim2lio.py`）。

> 阅读前提：GLIM 中**子图位姿与因子图节点位姿是同一个量**。因子图的节点 `X(i)` 代表第 i 个子图，每次优化后由 `update_submaps()` 回写进 `SubMap::T_world_origin`。不存在「每帧一个图节点」的设计。

## 0. 总览

```
 IMU/LiDAR
    │
    ▼
 [阶段一] 里程计 (odometry_estimation)
    │  逐帧 EstimationFrame：stamp / T_world_lidar / T_world_imu / v / imu_bias
    ▼
 [阶段一] 子图构建 (sub_mapping)  ── 关键帧选择 + 子图内局部优化 + 点云合并
    │  SubMap(T_world_origin, T_origin_endpoint_L/R, frames[], frame=合并点云)
    ▼
 [阶段二] 全局建图 (global_mapping) ── 因子图 isam2，节点 X(i) = 子图位姿
    │  变量：X(i) 子图位姿 / E 端点位姿 / V 速度 / B IMU bias
    ▼
 [阶段三] 落盘 (GlobalMapping::save)
    │  graph.bin values.bin graph.txt  traj_*.txt odom_*.txt  000000/...  config/
    ▼
 [阶段四] 加载与合并 (GlobalMapping::load) ── 反序列化 + rekey + 重建体素/因子
    │
    ▼
 [阶段五] Viewer 交互合并 (OfflineViewer) ── 粗配准 → ICP → 建锚点 → 找重叠 → 优化
    │
    ▼
 [阶段六] 图恢复 (recover_graph) ── 缺值时由 submap 数据重建图与初值
```

内存中始终是三层结构：

| 层 | 类型 | 关键成员 | 是否为图节点 |
|---|---|---|---|
| 帧 | `EstimationFrame` | `stamp`、`T_world_lidar`、`T_world_imu`、`v_world_imu`、`imu_bias` | 否 |
| 子图 | `SubMap` | `T_world_origin`、`T_origin_endpoint_L/R`、`frames[]`、`frame`（合并点云）、`voxelmaps` | 是（作为 `X(i)`） |
| 因子图 | `gtsam_points::ISAM2Ext` | 变量 `X/E/V/B`，因子为里程计/IMU/配准代价 | — |

因子图变量符号（`using gtsam::symbol_shorthand::...`，`global_mapping.cpp:44-47`）：

| 符号 | 含义 | 编号规律 |
|---|---|---|
| `X(i)` | 第 i 个子图的 origin 位姿 | 每子图一个 |
| `E(k)` | 子图端点位姿（左/右） | `k = 2i` 左端、`2i+1` 右端 |
| `V(k)` | 端点的世界系速度 | 同上 |
| `B(k)` | 端点的 IMU bias | 同上 |

---

## 阶段一：帧 → 子图（`sub_mapping`）

源码：`src/glim/src/glim/mapping/sub_mapping.cpp`

1. **逐帧入图**（`insert_frame`）。每个里程计帧 `EstimationFrame` 在子图**局部**因子图中作为 `X(current)`（`current = odom_frames.size()`），初值取 `odom_frame->T_world_sensor()`。`current == 0` 时加先验因子固定第一帧；`create_between_factors` 开启时在相邻帧间加 between 因子（`sub_mapping.cpp:189-215`）。

2. **关键帧选择**（`sub_mapping.cpp:245-267`）。由 `keyframe_update_strategy` 决定：
   - `OVERLAP`（默认）：与上一个关键帧的体素图重叠度 `< max_keyframe_overlap` 时插入新关键帧（默认阈值 `0.6`）。
   - `DISPLACEMENT`：相对上一关键帧的平移 `> keyframe_update_interval_trans` 或转角 `> keyframe_update_interval_rot` 时插入。
   - 帧点数不足 `keyframe_update_min_points`（默认 500）时不作为关键帧。

3. **关键帧内建因子**（`sub_mapping.cpp:270-320`）。新关键帧与其**之前的所有关键帧**建立 VGICP 配准因子（全连接），保证子图内一致。

4. **子图切分**（`create_submap`，`sub_mapping.cpp:421`）。当关键帧数 `>= max_num_keyframes`（默认 15）时切出一个子图；序列结束时 `submit_end_of_sequence()` 强制切出最后一个。

5. **子图内局部优化**。对子图局部的因子图跑 `LevenbergMarquardtOptimizerExt`（最多 20 次迭代），得到子图内各帧位姿。

6. **组装子图**（`create_submap`，`sub_mapping.cpp:459-495`）。以**中间帧**为原点建立子图坐标系：

   ```
   center                 = odom_frames.size() / 2
   T_world_origin         = X(center)
   T_origin_endpoint_L    = T_world_origin⁻¹ · X(0)
   T_origin_endpoint_R    = T_world_origin⁻¹ · X(last)
   ```

   再把关键帧点云按 `T_world_origin⁻¹ · X(关键帧)` 变换后合并、降采样，得到 `submap->frame`。

> 所以子图点云存的是 **origin 系**（原点帧的 lidar 系）点，`T_origin_endpoint_*` 存的是**相对**位姿。还原世界系需要 `p_world = T_world_origin · p_origin`。

---

## 阶段二：子图 → 因子图（`global_mapping` 在线）

源码：`src/glim/src/glim/mapping/global_mapping.cpp`

1. **入图与初值**（`insert_submap`，`global_mapping.cpp:138-152`）。子图按时序插入，初值由「上一子图位姿 × 拼接相对位姿」推送：

   ```
   T_origin0_origin1   = T_origin0_endpointR0 · T_endpointR0_endpointL1 · T_origin1_endpointL1⁻¹
   current_T_world_submap = last_T_world_submap · T_origin0_origin1     // 首个子图为 identity
   X(current) = current_T_world_submap
   submap->T_world_origin = current_T_world_submap
   ```

   其中帧间相对量 `T_endpointR0_endpointL1` 来自两子图端点的里程计位姿。

2. **建因子**：
   - `create_between_factors`（相邻子图里程计约束，`global_mapping.cpp:379`）；
   - `create_matching_cost_factors`（当前子图与其它子图的重叠配准代价，`global_mapping.cpp:430`）；
   - `enable_imu` 时补 IMU 相关变量与因子（`E/V/B`、`ImuFactor`、`RotateVector3Factor` 等，`global_mapping.cpp:168-220`）；
   - `X(0)` 上加 `LinearDampingFactor` 固定全局原点（`global_mapping.cpp:162`）。

3. **isam2 增量优化**（`update_isam2`）。

4. **回写子图位姿**（`update_submaps`，`global_mapping.cpp:486`）：

   ```cpp
   submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
   ```

   **这一步是「子图位姿」与「节点位姿」合一的关键**：优化结果写回 `T_world_origin`，同时它也就是 `X(i)` 的当前估计。

5. **回环 / 重叠检测**（`find_overlapping_submaps`，`global_mapping.cpp:285`）。按世界系距离 `max_implicit_loop_distance` 筛候选，再用体素图算重叠度 `overlap_auto`，超过 `min_implicit_loop_overlap` 才建立 VGICP 代价因子。

---

## 阶段三：落盘 dump（`GlobalMapping::save`）

源码：`global_mapping.cpp:546`。触发点：`glim_rosnode.cpp:24`、`glim_rosbag.cpp:398`（进程结束）、`GlimROS::~GlimROS`（`/tmp/dump`）、Viewer `Save Map`。

1. `optimize()` 跑一次 isam2，`update_submaps()` 同步位姿。
2. **`graph.bin`**：遍历 `isam2->getFactorsUnsafe()`，只序列化**可 Boost 序列化的普通 GTSAM 因子**。
3. **`values.bin`**：`isam2->calculateEstimate()`，即全部 `X/E/V/B`。
4. **`graph.txt`**：三个计数 + 每个匹配代价因子一行。
5. **`odom_lidar.txt` / `odom_imu.txt`**：未优化的里程计轨迹（`submap->odom_frames`）。
6. **`traj_lidar.txt` / `traj_imu.txt`**：优化后轨迹。逐帧由「子图 origin 位姿 + 帧相对位姿」推出（`global_mapping.cpp:618-628`）：

   ```
   T_world_endpoint_L = T_world_origin · T_origin_endpoint_L
   T_world_imu        = T_world_endpoint_L · T_odom_imu0⁻¹ · frame->T_world_imu
   T_world_lidar      = T_world_imu · T_lidar_imu⁻¹
   ```

7. **每个子图** `SubMap::save(path/000000)`（`sub_map.cpp:24`）：`data.txt` + `points_compact.bin` + `covs_compact.bin` + `imu_rate.txt`。
8. **`config/`**：`GlobalConfig::instance()->dump(path + "/config")`。

### 关键设计：匹配代价因子不落盘

`IntegratedGICPFactor` / `IntegratedVGICPFactor(GPU)` 等挂着体素图指针，**无法 Boost 序列化**，save 时被剔除，只在 `graph.txt` 里记一行：

```
matching_cost <type> <i> <j>       # type ∈ {gicp, vgicp, vgicp_gpu}
```

加载时再依据当前子图点云重建体素图、重新构造这些因子补回图中（见阶段四）。

`graph.txt` 完整格式（`global_mapping.cpp:576`）：

```
num_submaps: <N>
num_all_frames: <Σ frames>
num_matching_cost_factors: <M>
matching_cost <type> <i> <j>       # 共 M 行
```

`data.txt` 格式（`SubMap::save`/`SubMap::load`，`sub_map.cpp:24,77`），按 token 流顺序解析：

```
id: <submap_id>
T_world_origin: <4x4>
T_origin_endpoint_L: <4x4>
T_origin_endpoint_R: <4x4>
T_lidar_imu: <4x4>                 # frames 非空时才写
imu_bias: <6>
frame_id: <int>
num_frames: <N>
frame_0
  id: <int>
  stamp: <%.9f>
  T_odom_lidar: <4x4>              # 未优化
  T_world_lidar: <4x4>             # 优化后
  v_world_imu: <3>
frame_1
  ...
```

点云文件（`gtsam_points::PointCloudCPU`，见 `point_cloud_cpu.cpp`）：`points_compact.bin` 为 N×3 `float32` 无头，`covs_compact.bin` 为 N×6 `float32`（对称协方差上三角 `c00 c01 c02 c11 c12 c22`）。

---

## 阶段四：加载与会话合并（`GlobalMapping::load`）

源码：`global_mapping.cpp:690`。

1. **读计数**：`graph.txt` 得到 `num_submaps / num_all_frames / num_matching_cost_factors` 及匹配代价因子列表。
2. **逐子图加载**：`SubMap::load(path/000000)`（`sub_map.cpp:77`）解析 `data.txt` 与点云。同时按点云中位距离自适应确定体素分辨率，**重建每个子图的体素图**（`global_mapping.cpp:715-763`），供配准使用。
3. **读图与值**：反序列化 `graph.bin` → `loaded_graph`，`values.bin` → `loaded_values`。任何一步抛异常则置 `needs_recover = true`。
4. **多会话 rekey**（`start_from_frame_id > 0`，即加载第二个及以后的 dump 时，`global_mapping.cpp:795-844`）：把新 dump 的所有键整体平移，避免与已有会话冲突：

   ```
   X(i)   → X(i + start_from_frame_id)
   E/V/B  → 对应整体平移
   ```

   同时**删除** `PoseTranslationPrior`、`LinearDampingFactor`、`PriorFactor<Pose3>`——否则两个会话各自的先验会把对方拉回各自原点，合并必然失败。
5. **重建匹配代价因子**：按 `graph.txt` 的 `matching_cost` 行，用第 2 步的体素图与子采样点云重新构造 `IntegratedVGICPFactor(GPU)` 补回图（`global_mapping.cpp:846-878`）。
6. **空因子清理**：剔除 `nullptr` 因子，数量变化则置 `needs_recover`。
7. **需要恢复则调用 `recover_graph`**（见阶段六）。
8. **首个会话（`start_from_frame_id <= 0`）**：直接 `update_isam2(graph, values)` 优化并 `update_submaps()`。**后续会话**：不立即优化，把图与值塞入 `new_factors/new_values` 待合并操作时统一优化（`global_mapping.cpp:892-908`）。
9. `session_id++`。

---

## 阶段五：Viewer 交互合并（`OfflineViewer`）

源码：`src/glim/src/glim/viewer/offline_viewer.cpp`、`map_editor.cpp`。对应 README 第 4 节的界面操作：

| 界面操作 | 背后动作 |
|---|---|
| `File → Open New Map` | `load_map` 新建 `GlobalMapping` 并 `load(path)`（`offline_viewer.cpp:181`） |
| `File → Open Additional Map` | 再次 `load_map`，走阶段四的 rekey 分支（`start_from_frame_id > 0`） |
| `Merge sessions` | 对 source/target 会话点云做全局粗配准 |
| `Run fine registration` | ICP/GICP 精配准，得到会话间变换 |
| `Create Factor` | 在因子图中插入会话间锚点约束 |
| `Find overlapping submaps` | 对每次 `find_overlapping_submaps`，跨会话寻找重叠子图并建 VGICP 代价因子（阶段二第 5 步） |
| `Optimize` | `AsyncGlobalMapping::optimize` 反复跑 `update_isam2` + `update_submaps` |
| `Save Map` | `OfflineViewer::save` → `AsyncGlobalMapping::save`（同阶段三） |
| `Recover graph` | 触发阶段六 |

---

## 阶段六：图恢复（`recover_graph`）

源码：`global_mapping.cpp:912`（入口）/ `918`（实现）。

当 `graph.bin`/`values.bin` 缺失、反序列化失败（如 Boost 版本不兼容）或因子被剔除时触发。它**只用子图自身的 `data.txt` 数据**把图与初值补全——这也是外部工具**不写 `graph.bin/values.bin` 也能被 Viewer 打开**的原因。

1. 扫描已有值与因子，判断是否启用 IMU（存在 `e/v/b` 键或 `ImuFactor`）。
2. 建立连通性表 `connectivity_map`，记录哪些键对已有因子约束。
3. 对每个子图 `i`：
   - `X(i)` 缺失 → 用 `submaps[i]->T_world_origin` 补值；
   - `X(i) → X(i+1)` 缺因子 → 用两子图 origin 的里程计相对位姿补 between 因子；
   - `X(0)` 缺先验 → 补 `LinearDampingFactor`；
   - 启用 IMU 时，按 `T_world_origin · T_origin_endpoint_L/R` 补 `E` 初值、按 `T_world_origin.linear() · v_origin_imu*` 补 `V` 初值、按子图端帧 `imu_bias` 补 `B` 初值，并补上对应的 between / prior / `RotateVector3Factor`。
4. 返回补齐后的 `{new_factors, new_values}`，由调用方 `update_isam2` 合并。

> 因此坏图的典型成因是：子图 `data.txt` 本身不完整（缺少 `T_world_origin`），此时 recover 也无从下手。README 提到的 `X34 -> E69 is missing` 一类警告，就是这里在提示补了哪些键。

---

## 附录 A：dump 目录文件速查

| 文件 | 内容 | 缺失后果 |
|---|---|---|
| `graph.txt` | 计数 + 匹配代价因子列表 | 无法加载（load 直接返回 false） |
| `graph.bin` | 可序列化的 GTSAM 因子 | 触发 `recover_graph` |
| `values.bin` | 全部变量 `X/E/V/B` 的估计 | 触发 `recover_graph` |
| `odom_lidar.txt` / `odom_imu.txt` | 未优化里程计轨迹 | 仅影响可视化 |
| `traj_lidar.txt` / `traj_imu.txt` | 优化后轨迹 | 仅影响可视化 |
| `config/` | 建图时的参数快照 | Viewer 用自身 config 兜底 |
| `000000/data.txt` | 子图位姿 + 逐帧位姿 + 外参 | 该子图加载失败，整图加载失败 |
| `000000/points_compact.bin` | origin 系点云 N×3 f32 | 该子图加载失败 |
| `000000/covs_compact.bin` | 逐点协方差 N×6 f32 | 配准精度下降 |
| `000000/imu_rate.txt` | 帧内 IMU 频率轨迹 | 仅影响可视化 |

## 附录 B：与外部转换工具的对应

| 逻辑量 | dump 中的位置 | `lio2glim.py`（写入） | `glim2lio.py`（读出） |
|---|---|---|---|
| 子图位姿 | `data.txt` `T_world_origin` | 取子图中心帧位姿 | → `poses.txt` 每子图一行 |
| 逐帧位姿 | `data.txt` `frame_i.T_world_lidar` | 由 TUM 逐帧写入 | → `frames.txt` |
| 子图点云（世界系） | `points_compact.bin`（origin 系） | 世界系点 × `T_world_origin⁻¹` | `T_world_origin` × origin 点 → `clouds/`、`merged.pcd` |
| 图/值 | `graph.bin` / `values.bin` | **不写**，靠加载时 `recover_graph` | — |
| 匹配代价 | `graph.txt` `matching_cost` 行 | 写 0 条，合并时由界面重建 | — |

坐标还原关系（贯穿全流程）：

```
p_origin = T_world_origin⁻¹ · p_world
p_world  = T_world_origin  · p_origin
T_world_imu   = T_world_origin · T_origin_endpoint_L · T_odom_imu0⁻¹ · frame->T_world_imu
T_world_lidar = T_world_imu · T_lidar_imu⁻¹
```
