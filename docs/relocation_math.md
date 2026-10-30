# 重定位的坐标系、运动补偿、状态接管与协方差理论

本文依据当前 SuperLIO_SLAM 重定位实现整理，更新于 2026-10-31。统一从参考地图世界系、局部 LIO 坐标系出发，推导多帧累积、粗精配准、异步结果接管和 ESKF 协方差变换。

文中明确区分三类内容：

- **当前实现**：可在代码中直接对应到的行为。
- **条件成立时的理论公式**：例如把配准变换视为准确已知，或假设两类误差独立。
- **尚未实现的扩展**：完整配准协方差、相关性传播和延迟测量融合。

重定位不是回环后端的 iSAM2 图优化。当前重定位先运行局部 LIO，估计局部系到参考地图系的刚体变换，再把最新 ESKF 状态切换到参考地图中继续定位。

## 1. 坐标系与变换约定

### 1.1 坐标系

| 符号 | 含义 | 当前 ROS 表达 |
|---|---|---|
| \(W\) | 参考 PCD 的世界坐标系 | world |
| \(S\) | 重定位前建立的固定局部 LIO 坐标系 | 内部数学坐标系；定位前直接在 world 中预览 |
| \(I_i\) | 第 \(i\) 个扫描参考时刻的 IMU 坐标系 | imu 在时刻 \(t_i\) 的坐标系 |
| \(L\) | LiDAR 坐标系 | 由 LiDAR–IMU 外参关联 |

这里的“世界”表示参考地图采用的坐标系，不意味着一定是地理坐标系。地图原点和朝向由建图、地图合并及导出过程决定。

局部系 \(S\) 在局部 LIO 初始化后固定，不随着滑窗最新帧移动。初始化一般令位置为零，并依据重力确定横滚、俯仰方向；偏航仍由局部坐标约定决定。因此，初始 IMU 位姿 \(T_{S,I_0}\) 不必严格为单位矩阵，尤其初始化存在重力对齐旋转时。

### 1.2 变换方向

全文统一使用：

$$
T_{A,B}=
\begin{bmatrix}
R_{A,B}&p_{A,B}\\
0&1
\end{bmatrix},
\qquad
{}^Aq=R_{A,B}\,{}^Bq+p_{A,B}.
$$

即 \(T_{A,B}\) 将 \(B\) 系坐标变换到 \(A\) 系。组合顺序为：

$$
T_{A,C}=T_{A,B}T_{B,C}.
$$

重定位要求解：

$$
T_{W,S}=
\begin{bmatrix}
A&t\\
0&1
\end{bmatrix}.
$$

局部 LIO 已知当前状态 \(T_{S,I}(t)\)，因此地图系中的当前状态满足：

$$
\boxed{T_{W,I}(t)=T_{W,S}T_{S,I}(t).}
$$

不能将乘法顺序颠倒，也不能把配准结果再乘到已经处于世界系的点云上。

## 2. 整体流程与状态机

~~~text
加载参考 PCD，建立参考体素地图与近邻索引
    ↓
等待短暂静止 / 低动态 IMU 初始化
    ↓
固定局部系 S 中运行 LIO
    ↓
去畸变、局部地图匹配、更新 S 系累积体素地图、冻结配准快照
    ↓
后台 FPFH → TEASER++ → GICP，估计 T_W_S
    ↓
粗精配准、几何退化、重力方向等验收
    ↓
用返回的 T_W_S 复查当前最新扫描
    ↓
变换最新 ESKF 状态与协方差，切换参考地图
    ↓
WORLD_MAP_TRACKING
~~~

主要状态包括：

- LOAD_REFERENCE_MAP：加载参考地图。
- WAITING_FOR_STATIONARY_IMU：等待初始化条件满足。
- LOCAL_TRACKING：在局部地图中持续跟踪。
- LOCAL_TRACKING_DEGRADED：局部有效匹配不足，暂不接管重定位结果。
- WORLD_MAP_TRACKING：已经切换到参考地图。
- ERROR_SENSOR_TIME_ROLLBACK：检测到扫描时间不递增，停止当前会话处理。

**当前实现**使用独立配准线程。LIO 不等待后台求解结束；最多保留一个运行任务和一个最新待处理任务，新待处理快照覆盖旧待处理快照。

每个任务和结果都携带 generation。初始位姿更新或重定位完成后改变 generation，以阻止旧任务结果被当前会话错误接收。成功后停止新的重定位任务；当前没有自动丢失恢复和再次全局搜索状态机。

## 3. IMU 初始化和局部 LIO

### 3.1 初始化的物理意义

在短暂静止或足够低动态时，平均角速度可用于估计陀螺偏置，平均加速度方向可用于估计重力方向。

当前实现检查样本数、持续时间、角速度幅值，以及加速度和角速度的波动。默认至少 50 个 IMU 样本、0.3 s；满足条件后执行重力对齐。加速度计初始偏置设为零，并用重力模长与平均加速度模长的比值设置加速度尺度。

这不是完全动态的惯性冷启动。若初始化期间存在明显线加速度，直接把平均加速度方向解释为重力会产生姿态误差。

### 3.2 局部状态传播

忽略离散积分细节，名义状态可以写为：

$$
x_S=(R_S,p_S,v_S,b_g,b_a,g_S).
$$

其中 \(R_S=R_{S,I}\)。连续传播的基本形式为：

$$
\begin{aligned}
\dot R_S&=R_S[\omega_m-b_g]_\times,\\
\dot p_S&=v_S,\\
\dot v_S&=R_S(s_a a_m-b_a)+g_S.
\end{aligned}
$$

\(s_a\) 表示实现中的加速度尺度，偏置和重力的过程模型由滤波器定义。LiDAR 对局部体素地图的匹配提供观测修正。

局部 LIO 的作用是提供连续相对运动和扫描去畸变，使设备在后台配准期间仍能移动。它尚未确定局部坐标系相对于参考地图的全局变换。

## 4. 单帧去畸变与多帧运动补偿

### 4.1 单帧内部的运动补偿

设一个原始 LiDAR 点在 \(t_{ij}\) 时刻采集，第 \(i\) 帧选取参考时刻 \(t_i\)。理论上，该点可以表达为：

$$
{}^{I_i}q_{ij}
=
T_{S,I}(t_i)^{-1}
T_{S,I}(t_{ij})
T_{I,L}\,{}^Lq_{ij},
$$

这里采用齐次坐标记法，\(T_{I,L}\) 是外参。

这一步把同一扫描中不同采集时刻的点统一到 \(I_i\) 系。当前局部 LIO 的传播与去畸变负责该过程。

如果输入没有逐点时间，将时间偏移全部置零无法恢复真实扫描运动。关闭缺少 time 字段的日志只改变打印行为，不解决去畸变问题。

### 4.2 固定局部系中的累积地图

设扫描参考系为 \(I_n\)，其 LIO 位姿为 \(T_{S,I_n}\)。截至第 M 帧，理想累积地图为：

$$
\boxed{
\mathcal P_S^{(M)}
=\operatorname{Voxel}\left(\bigcup_{n=1}^{M}T_{S,I_n}\mathcal P_{I_n}\right).
}
$$

当前重定位直接提取局部 LIO 的 `local_ivox_` 全部保留点，生成不可变 S 系点云快照，不再保存或拼接滑动窗口。它沿用 LIO 的点筛选、体素表示与容量策略，因此不是原始扫描的无损并集，也不能恢复已过滤或容量淘汰的点；没有另外施加最近帧数、时间跨度或行程裁剪。

每个后台任务同时保存：地图快照 \(\mathcal P_S^{(M)}\)、同一时刻的 \(T_{S,I_M}\)、时间 \(t_M\)、当前扫描和 generation。扫描仅用于质量验收；配准源是累积地图，不能再乘一次 \(T_{S,I_M}\)，也不以 M 帧为新原点。

### 4.3 异步快照与最新帧接管

默认成功跟踪至少 5 帧后可提交任务（`relocation.registration_min_frames`），随后按传感器时间间隔提交。运行中的任务不被新扫描中断，待处理任务只保留最新快照，地图和快照之间不共享可变点云。

若第 M 帧地图配准得到 \(T_{\mathrm{reg}}=T_{W,S}\)，返回时前端已运行到第 N 帧，则：

$$
\boxed{
T_{W,I_N}
=\left(T_{\mathrm{reg}}T_{S,I_M}\right)
 \left(T_{S,I_M}^{-1}T_{S,I_N}\right)
=T_{\mathrm{reg}}T_{S,I_N}.
}
$$

M 小于 N 不构成拒绝原因；新任务失败也不会在前端消费前覆盖一个成功待提交结果。仍须检查 generation、局部跟踪质量和最新扫描，再变换最新 ESKF 状态及协方差，不回退滤波器时间。

`registration_max_source_points`、`registration_max_target_points` 是对完整地图体素过滤后的空间采样上限，0 表示不追加点数限制。抽点不会改成最近帧窗口。累积地图越大，快照复制和配准开销越高，可通过体素大小、点数上限和任务间隔控制；LIO 漂移造成的地图内部畸变也不会由一个刚体变换完全消除。

## 5. 初始位姿与参考地图范围

设用户提供某个局部关键帧 \(k\) 的世界位姿先验：

$$
\widehat T_{W,I_k},
$$

该关键帧的局部位姿为 \(T_{S,I_k}\)，则局部系到地图系的提示为：

$$
\boxed{
\widehat T_{W,S}
=
\widehat T_{W,I_k}T_{S,I_k}^{-1}.
}
$$

于是最新扫描的预测世界位姿为：

$$
\widehat T_{W,I}(t)
=
\widehat T_{W,S}T_{S,I}(t).
$$

当前支持配置中的初始位姿以及 /initialpose。带非零时间戳的运行时初始位姿，通过最近 5 秒的纯位姿记录匹配时间接近的局部状态，容差为 0.25 s。这些记录不含点云，不用于限制配准地图范围。

有先验且裁剪半径 \(r>0\) 时，参考点云按最新预测位置裁剪：

$$
\mathcal M_{\mathrm{crop}}
=
\left\{
q\in\mathcal M_W:
\|q-\widehat p_{W,I}(t)\|\le r
\right\}.
$$

默认 \(r=80\) m；无先验或裁剪半径为零时不裁剪。当前 YAML 关闭初始位姿；启用后全零初值也是一个被使用的先验，不等于“没有先验”。

先验失败后的无提示重试仍使用已经选出的目标区域；它不会自动扩展为整张地图。要进行整图尝试，应明确关闭先验或关闭裁剪。

## 6. 粗配准与精配准

### 6.1 FPFH 与 TEASER++

源点 \(a_j\in\mathcal P_S\)，目标对应点 \(b_j\in\mathcal M_W\)。固定尺度刚体模型为：

$$
b_j=Aa_j+t+\epsilon_j.
$$

FPFH 用于产生候选对应；TEASER++ 对存在大量错误对应的集合求鲁棒旋转和平移。它需要足够的正确对应、合理的空间分布和适当噪声模型，不保证任意重复场景都能正确定位。

当前重定位默认使用全局粗配准配置：FPFH 单向最近邻、不做比率和双向一致性筛选，按源点索引均匀取最多 2000 对，使用 PMC_EXACT（最大团阶段 30 秒预算）和 GNC_TLS；体素、法向与描述子半径分别为 0.3、0.9、1.5 m。参数与独立 `teaserpp_registration` 工具对齐。粗几何内点仍以 0.6 m 残差判断，再执行现有数量、比例和 RMSE 验收。

在线回环继续采用 mutual_ratio、空间分散的 200 对、PMC_HEU 短预算。算法共享实现，配置分别加载；不能把重定位的全局配置自动视作回环参数。

得到粗变换：

$$
T_{\mathrm{coarse}}\approx T_{W,S}.
$$

当前检查粗内点数量、内点比例、残差 RMSE，以及内点空间分布和刚体变换合法性。

### 6.2 使用先验时的坐标关系

当前实现会把裁剪参考点云先变换到近似局部系：

$$
\widetilde{\mathcal M}
=
\widehat T_{W,S}^{-1}\mathcal M_W.
$$

这样源、目标特征在近似一致的坐标和法向视点下计算。若 TEASER++ 求得：

$$
\Delta T:\mathcal P_S\rightarrow\widetilde{\mathcal M},
$$

则世界系粗变换为：

$$
\boxed{
T_{\mathrm{coarse}}
=
\widehat T_{W,S}\Delta T.
}
$$

重试无提示时，直接在局部源子图和世界目标点云之间求 \(T_{W,S}\)。

### 6.3 GICP

GICP 使用粗变换初始化，在局部范围内细化刚体配准。概念目标可写为：

$$
\min_{A,t}
\sum_j r_j^\top
\left(C_j^b+AC_j^aA^\top\right)^{-1}r_j,
\qquad
r_j=b_j-(Aa_j+t).
$$

这里 \(C_j^a,C_j^b\) 表示点邻域的几何协方差模型，对应关系与权重会随迭代变化。

当前读取 GICP 的完整最终变换：

$$
T_{\mathrm{reg}}=T_{\mathrm{GICP,final}}.
$$

该结果已经包含初值，不能再额外乘一次粗配准变换。

### 6.4 配准质量不等于变换协方差

当前验收包括：

- 收敛、有限值、合法旋转。
- GICP 相对粗初值的修正幅度。
- 累积源地图覆盖率、快照单帧覆盖率和有效近邻数量。
- 有效近邻 MSE。
- 根据目标法向形成的归一化信息矩阵特征值比，用于拒绝明显退化。
- 配准后重力方向与地图负 Z 方向的夹角，默认不超过 10°。

设参考近邻距离为 \(d(q,\mathcal M_W)\)，质量半径为 \(r_q\)，则：

$$
\mathcal I=
\{q\in T_{\mathrm{reg}}\mathcal P_S:
d(q,\mathcal M_W)\le r_q\},
$$

$$
\operatorname{overlap}
=\frac{|\mathcal I|}{|T_{\mathrm{reg}}\mathcal P_S|},
\qquad
\operatorname{MSE}
=\frac{1}{|\mathcal I|}
\sum_{q\in\mathcal I}d(q,\mathcal M_W)^2.
$$

RMSE 是 MSE 的平方根，二者单位分别是 m 与 m²。MSE、覆盖率和信息特征值比都不是完整的 6×6 位姿协方差。

当前覆盖率检查针对源子图及最新源单帧，不等于源—目标双向覆盖率。GICP 相对粗配准的修正限制，也不等于最终变换与 LIO / 用户先验的一致性约束。

## 7. 异步结果为什么必须作用于最新状态

设后台任务快照时刻为 \(\tau\)，结果返回时 LIO 已运行到 \(t>\tau\)。只要局部系 \(S\) 没有更换，配准结果仍表示同一个固定变换 \(T_{W,S}\)。

最新世界位姿应为：

$$
\boxed{
T_{W,I}(t)=T_{W,S}T_{S,I}(t).
}
$$

将其展开，也可以写成：

$$
T_{W,I}(t)
=
\underbrace{T_{W,S}T_{S,I}(\tau)}_{\text{快照时刻的世界位姿}}
\underbrace{T_{S,I}(\tau)^{-1}T_{S,I}(t)}_{\text{快照后的局部相对运动}}.
$$

不能直接把当前滤波器位置设置为快照时刻的配准位置，否则会丢失 \(\tau\) 到 \(t\) 的运动，并造成位姿或时间回退。

当前程序在接管前还要求：

1. 任务 generation 与当前会话一致。
2. 后台配准结果已通过验收。
3. 当前局部 LIO 跟踪有效。
4. 用 \(T_{\mathrm{reg}}T_{S,I}(t)\) 变换最新扫描后，最新扫描相对于参考地图的内点数量、覆盖率和 MSE 仍合格。

最新扫描复查是对迟到结果的额外保护，但不构成严格的多假设定位或完整时序一致性证明。

## 8. 名义状态的接管

令最新局部状态为：

$$
x_S(t)=
\bigl(R_S,p_S,v_S,b_g,b_a,g_S\bigr),
$$

并令 \(T_{\mathrm{reg}}=[A,t_{W,S}]\)。为避免与时间变量混淆，本节把平移写为 \(t_{W,S}\)。世界状态为：

$$
\boxed{
\begin{aligned}
R_W&=AR_S,\\
p_W&=Ap_S+t_{W,S},\\
v_W&=Av_S,\\
g_W&=Ag_S,\\
b_{g,W}&=b_g,\\
b_{a,W}&=b_a.
\end{aligned}
}
$$

速度和重力只旋转，不加平移。偏置在 IMU 机体系表达，不因外部参考系改变而旋转。

当前还同步变换全局加速度和前向传播缓存的姿态、位置、速度，清除旧坐标系下的点云和传播缓存，以免后续处理混用坐标系。

滤波器时间、IMU 时间和机体系 IMU 历史量保持不变。滤波器误差增量清零，不是把已有协方差清零。

## 9. ESKF 的误差状态与协方差

当前代码的误差状态排列为：

$$
\delta x=
\begin{bmatrix}
\delta\theta\\
\delta p\\
\delta v\\
\delta b_g\\
\delta b_a\\
\delta g
\end{bmatrix}\in\mathbb R^{18},
\qquad
P=\operatorname{Cov}(\delta x).
$$

姿态采用右乘局部误差：

$$
R_{\mathrm{true}}=R\operatorname{Exp}([\delta\theta]_\times).
$$

其余状态采用加性误差，例如：

$$
p_{\mathrm{true}}=p+\delta p,
\qquad
v_{\mathrm{true}}=v+\delta v.
$$

这里 \([a]_\times b=a\times b\)。姿态误差在局部机体切空间表达，位置、速度、重力误差在参考坐标系表达；两种误差基底不能混淆。

当前协方差实际按 18×18 实现。重力模长固定时物理上只有两个方向自由度，但本推导遵循当前三分量重力误差实现，不将其替换成另一套 17 维流形滤波器。

## 10. 配准变换准确已知时，协方差怎样变换

本节把 \(A,t_{W,S}\) 看成准确已知的确定量，真实状态和名义状态都使用同一变换。

### 10.1 姿态误差不需要直接旋转

$$
\begin{aligned}
R_{W,\mathrm{true}}
&=A R_{S,\mathrm{true}}\\
&=A R_S\operatorname{Exp}([\delta\theta_S]_\times)\\
&=R_W\operatorname{Exp}([\delta\theta_S]_\times).
\end{aligned}
$$

所以：

$$
\boxed{\delta\theta_W=\delta\theta_S.}
$$

这依赖当前的右乘误差定义。若采用左乘、参考系表达的角度误差，姿态块的变换就不同。

### 10.2 位置、速度、重力误差

$$
\begin{aligned}
p_{W,\mathrm{true}}
&=A(p_S+\delta p_S)+t_{W,S},\\
\delta p_W
&=p_{W,\mathrm{true}}-p_W
=A\delta p_S.
\end{aligned}
$$

因此：

$$
\delta v_W=A\delta v_S,\quad
\delta g_W=A\delta g_S,\quad
\delta b_{g,W}=\delta b_{g,S},\quad
\delta b_{a,W}=\delta b_{a,S}.
$$

### 10.3 完整雅可比

$$
\delta x_W=J\delta x_S,
\qquad
\boxed{J=\operatorname{diag}(I,A,A,I,I,A).}
$$

据协方差定义：

$$
\boxed{P_W=JP_SJ^\top.}
$$

这会同时变换对角块和交叉块，例如：

$$
\begin{aligned}
P_{pp,W}&=AP_{pp,S}A^\top,\\
P_{\theta p,W}&=P_{\theta p,S}A^\top,\\
P_{pv,W}&=AP_{pv,S}A^\top.
\end{aligned}
$$

只旋转部分对角块、保留交叉块原值是不一致的。只改变状态、不改变协方差，会使后续滤波器在错误方向上表达置信度。

纯确定性坐标旋转只改变同一份不确定性的表达，不凭空引入新观测。这里平移不进入 \(J\)，因为位置误差采用参考系加性误差，且平移被假设准确已知；不能据此推论“不确定平移对协方差没有影响”。

### 10.4 为什么不直接套一个 SE(3) 伴随矩阵

SE(3) 左、右扰动、平移切向量以及加性位置误差的定义不同，所对应的协方差表达也不同。本项目使用“右角度误差 + 参考系加性位置误差”的混合状态定义。

因此应从上述状态定义求雅可比，而不能直接把一个 6×6 的位姿伴随矩阵放进 18×18 状态中。改用统一的 SE(3) 扰动参数化时，必须同时改变误差定义、雅可比和协方差表达。

## 11. 配准变换本身不确定时的完整一阶传播

本节是**理论扩展，当前没有完整实现**。

### 11.1 明确配准误差的参数化

选择：

$$
A_{\mathrm{true}}=A\operatorname{Exp}([\delta\phi]_\times),
\qquad
t_{\mathrm{true}}=t_{W,S}+\delta t.
$$

其中 \(\delta\phi\) 在 \(S\) 系表达，\(\delta t\) 在 \(W\) 系表达。定义：

$$
\xi=
\begin{bmatrix}\delta\phi\\\delta t\end{bmatrix},
\qquad
\Sigma_T=\operatorname{Cov}(\xi).
$$

这是旋转右扰动、平移加性扰动；它不是未经转换就能使用的任意 SE(3) 六维协方差约定。

### 11.2 位置的一阶展开

忽略二阶小量：

$$
\begin{aligned}
p_{W,\mathrm{true}}
&=
A\operatorname{Exp}([\delta\phi]_\times)(p_S+\delta p_S)
+t_{W,S}+\delta t\\
&\approx
Ap_S+t_{W,S}+A\delta p_S
-A[p_S]_\times\delta\phi+\delta t.
\end{aligned}
$$

因此：

$$
\boxed{
\delta p_W
=A\delta p_S-A[p_S]_\times\delta\phi+\delta t.
}
$$

配准旋转不确定性会通过局部位置产生位置误差，不能仅给位置添加一个独立平移方差就完整描述。

### 11.3 其他状态的一阶展开

由：

$$
A\operatorname{Exp}([\delta\phi]_\times)
R_S\operatorname{Exp}([\delta\theta_S]_\times)
\approx
AR_S\operatorname{Exp}
\left([R_S^\top\delta\phi+\delta\theta_S]_\times\right),
$$

得到：

$$
\begin{aligned}
\delta\theta_W&\approx\delta\theta_S+R_S^\top\delta\phi,\\
\delta v_W&\approx A\delta v_S-A[v_S]_\times\delta\phi,\\
\delta g_W&\approx A\delta g_S-A[g_S]_\times\delta\phi.
\end{aligned}
$$

整体写成：

$$
\boxed{\delta x_W\approx J\delta x_S+G\xi,}
$$

$$
\boxed{
G=
\begin{bmatrix}
R_S^\top&0\\
-A[p_S]_\times&I\\
-A[v_S]_\times&0\\
0&0\\
0&0\\
-A[g_S]_\times&0
\end{bmatrix}.
}
$$

### 11.4 独立误差假设下

如果 \(\delta x_S\) 与 \(\xi\) 独立：

$$
\boxed{
P_W=JP_SJ^\top+G\Sigma_TG^\top.
}
$$

若进一步假设配准旋转和平移误差独立，位置协方差块为：

$$
\boxed{
P_{pp,W}
=
AP_{pp,S}A^\top
+A[p_S]_\times\Sigma_\phi[p_S]_\times^\top A^\top
+\Sigma_t.
}
$$

完整的 \(G\Sigma_TG^\top\) 还会形成姿态—位置、位置—速度等交叉协方差。

例如，在这种参数化及相应独立性假设下，局部位置距离 \(S\) 原点约 10 m，角度标准差为 2°，垂直于该距离方向的位置误差量级约为：

$$
10\times 2\pi/180\approx0.35\ \mathrm m.
$$

这个估算不能脱离协方差约定使用。若改变局部原点、改变扰动参数化，必须同时变换 \(\Sigma_T\) 及其中的旋转—平移相关项，不能仅凭坐标原点较远就断言真实误差更大。

### 11.5 不独立时的交叉项

配准源子图由 LIO 位姿累积而来，因此两类误差通常相关。令：

$$
C=\operatorname{Cov}(\delta x_S,\xi)\in\mathbb R^{18\times6},
$$

则更一般的一阶传播为：

$$
\boxed{
P_W=
JP_SJ^\top+G\Sigma_TG^\top
+JCG^\top+GC^\top J^\top.
}
$$

忽略相关性不保证一定偏保守，也可能重复计算不确定性。

一维例子：若独立地图测量为 \(z\)，局部位置估计为 \(p\)，由二者构造平移 \(t=z-p\)，则：

$$
p_W=p+t=z.
$$

正确方差为 \(\operatorname{Var}(z)\)。若错误地将 \(p\) 与 \(t\) 当成独立量：

$$
\operatorname{Var}(p)+\operatorname{Var}(t)
=2\operatorname{Var}(p)+\operatorname{Var}(z).
$$

实际交叉协方差 \(\operatorname{Cov}(p,t)=-\operatorname{Var}(p)\) 会消去重复项。实际多帧配准比该例复杂，但它说明了不能直接假定独立。

## 12. 当前代码实际采用的协方差更新

当前 ESKF::ChangeReferenceFrame() 实施：

$$
\boxed{
P_{\mathrm{impl}}
=JP_SJ^\top+Q_{\mathrm{heuristic}},
}
$$

$$
Q_{\mathrm{heuristic}}
=
\operatorname{diag}
\left(
\sigma_R^2I,\,
\sigma_p^2I,\,
0,\,
0,\,
0,\,
0
\right).
$$

代码核心等价于：

~~~cpp
J = Identity18();
J.block<3,3>(3,3) = A;    // position
J.block<3,3>(6,6) = A;    // velocity
J.block<3,3>(15,15) = A;  // gravity
P = J * P * J.transpose();
P.block<3,3>(0,0) += rotation_variance * I;
P.block<3,3>(3,3) += position_variance * I;
P = 0.5 * (P + P.transpose());
~~~

默认配置为：

~~~yaml
relocation.registration_position_sigma_m: 0.2
relocation.registration_rotation_sigma_deg: 2.0
~~~

程序先将角度标准差转换为弧度，再将标准差平方作为方差：

$$
\sigma_p^2=0.04\ \mathrm{m}^2,
\qquad
\sigma_R^2=(2\pi/180)^2\approx0.001218\ \mathrm{rad}^2.
$$

**已经实现：**

- 按当前误差定义完整变换既有协方差及其交叉块。
- 给位置、姿态块增加人工设定的方差。
- 同步变换名义状态和前向传播缓存。
- 保留最新时间、机体系偏置和 IMU 历史。
- 对协方差进行数值对称化。

**尚未实现：**

- 从配准估计完整且经过标定的 \(\Sigma_T\)。
- 完整的 \(G\Sigma_TG^\top\)。
- LIO 与配准结果的交叉协方差 \(C\)。
- 包含历史状态的严格延迟观测融合。

这是一种避免过度自信的工程性膨胀模型，不应称为“已经证明统计保守的完整传播”。它可能漏掉旋转引起的位置、速度和重力不确定性；在相关性不明时，也可能重复计入某些误差。

对称化只修正浮点计算中的非对称性，不估计额外信息，也不修复配准误匹配。

## 13. 配准协方差可以怎样估计

本节为**后续扩展方向，当前代码没有实现**。

若配准在所选六维扰动参数 \(\eta\) 下，残差局部线性化为：

$$
r_j(\eta+\delta\eta)\approx r_j(\eta)+H_j\delta\eta,
$$

且残差噪声独立、权重 \(W_j\) 正确反映噪声协方差逆，则局部信息近似为：

$$
\Lambda_T\approx\sum_jH_j^\top W_jH_j,
\qquad
\Sigma_\eta\approx\Lambda_T^{-1}.
$$

如果权重没有包含实际噪声尺度，还需要相应的尺度估计。该近似还依赖正确对应、局部单峰分布、地图和源点误差模型等假设。

使用前必须将 \(\Sigma_\eta\) 转换成第 11 节定义的 \(\Sigma_T\) 参数化，并考虑源子图和地图的不确定性、相关性及退化方向。

不能把下列量直接当作位置或姿态方差：

- 配准近邻 MSE。
- 粗对应点 RMSE。
- 覆盖率。
- 当前代码使用的归一化信息特征值比。

例如，MSE 的单位是 m²，而角度方差单位是 rad²；即使单位相同，点残差方差也不等于位姿参数方差。较小的点残差仍可能出现在位姿不可观的走廊、平面或重复场景中。

## 14. 与卡尔曼测量更新的区别

坐标系切换是对状态分布重新表达。卡尔曼测量更新则是把新的观测信息与先验融合。

若以地图配准位姿构造测量模型：

$$
z=h(x)+n,\qquad n\sim\mathcal N(0,V),
$$

在相容的切空间中线性化后，得到残差 \(r\) 和雅可比 \(H\)，则：

$$
K=PH^\top(HPH^\top+V)^{-1},
\qquad
\widehat{\delta x}=Kr.
$$

Joseph 形式的协方差更新为：

$$
P_{\mathrm{post}}
=(I-KH)P(I-KH)^\top+KVK^\top.
$$

随后将误差修正注入名义状态，例如：

$$
R\leftarrow R\operatorname{Exp}([\widehat{\delta\theta}]_\times).
$$

误差注入后还需重置误差均值，并依据所选误差约定进行协方差重置。对当前右角度误差，其姿态重置雅可比一阶近似为：

$$
G_{\mathrm{reset},\theta}
\approx I-\frac12[\widehat{\delta\theta}]_\times.
$$

不要把此重置雅可比与第 10 节参考系变换的 \(J\) 混为一谈。外部参考系整体左乘 \(A\)，并不等于把 \(A\) 的完整旋转当作一次右误差状态修正注入。

此外，上述普通测量更新公式默认测量噪声与先验误差满足相应独立性条件。当前配准复用了 LIO 累积点云，若直接构造测量更新，也必须处理相关性和重复使用信息的问题。

**当前实现没有通过该 Kalman 增益融合配准结果。** 它把验收通过的变换作为参考系接管结果，再执行坐标变换和人工方差膨胀。

## 15. 迟到结果的协方差问题

名义位姿的最新状态接管：

$$
T_{W,I}(t)
=
T_{W,I}(\tau)
\left(T_{S,I}(\tau)^{-1}T_{S,I}(t)\right)
$$

在几何上保持了快照后的运动。严格协方差还涉及：

- 快照时刻地图定位的不确定性。
- 从 \(\tau\) 到 \(t\) 的相对运动不确定性。
- 快照状态、最新状态与配准结果之间的交叉协方差。

常见更完整的处理方式包括：保存历史状态和交叉协方差的延迟测量更新；在历史时刻更新后重新传播 IMU；状态克隆、滑窗估计或联合平滑。

当前程序只在最新状态上做参考变换和简化协方差膨胀，并用最新扫描几何复查保护接管。它没有维护上述完整历史联合分布。

## 16. 接管后的地图、TF 与时间

当前成功接管后：

1. 将最新 ESKF 状态及前向预测状态转换到 \(W\)。
2. 切换为预先加载的参考体素地图，释放局部地图。
3. 清除旧坐标系点云缓存和局部位姿记录，发布 Marker DELETE 删除局部地图预览。
4. 释放旧局部 Path 的存储并发布空轨迹，从最新世界位姿重新生成轨迹。
5. 进入 WORLD_MAP_TRACKING，停止新的后台重定位任务。

默认不更新参考地图；若显式启用地图更新，则属于后续参考地图跟踪行为。

ROS 输出与 TF 在接管前后统一使用 world：

~~~text
接管前：world → imu   （数值为局部 S 系下的 LIO 位姿）
接管后：world → imu   （数值为配准后的真实 W 系位姿）
~~~

定位前，局部地图预览、当前扫描、里程计及轨迹都直接使用 S 系数值并标记为 world，便于在同一 RViz 中观察。内部 S 系仍固定，不额外发布局部 TF，也不单独按初始位姿移动预览地图。初始位姿只参与配准先验。

接管时，输出位姿从：

$$
T_{S,I_N}\quad\longrightarrow\quad T_{W,I_N}=T_{\mathrm{reg}}T_{S,I_N}.
$$

因此接管时位置和朝向可以跳变，定位前的 world 标签仅用于预览，不表示已经求出世界对齐关系。清除局部地图和旧轨迹后，显示由参考地图、已对齐的当前扫描及新的世界轨迹接续；TF 始终只有 world → imu。

先验 PCD 加载后先剔除无效 XYZ，再按 `relocation.reference_map_voxel_size_m` 做一次体素质心降采样（YAML 默认 0.3 m，0 关闭降采样）。降采样结果共同用于地图显示、参考体素索引、最新帧质量复查以及后台配准，不改变磁盘 PCD 和世界坐标系。`registration_voxel_size_m` 仍控制每次配准的源/目标采样分辨率；两者可分别设置。

点云、位姿和 TF 保留对应的传感器时间。计算耗时、线程等待与传感器状态时间是不同概念，不能因为后台计算晚了，就把状态时间改成返回时的电脑时间。

注意：ESKF 内部更新了 \(P\)，不代表 nav_msgs/Odometry 的 pose/twist covariance 字段也已经填入。当前 ROSWrapper::pub_odom() 主要发布状态值，没有把内部 18×18 协方差转换并填入这些字段；下游需要协方差时，应另行实现符合消息坐标约定的导出。

## 17. 参数与理论量对应

以下为当前 relocation.yaml 的值，启动时可能被显式配置覆盖：

| 参数 | 默认值 | 理论意义 |
|---|---:|---|
| relocation.reference_map_voxel_size_m | 0.3 m | 先验 PCD 加载时的统一体素降采样；0 关闭 |
| relocation.registration_min_frames | 5 | 首次配准前的最少成功局部跟踪帧数；不是窗口大小 |
| relocation.registration_interval_s | 1.0 s | 基于传感器时间的任务提交间隔 |
| relocation.map_crop_radius_m | 80.0 m | 有先验时的参考地图裁剪半径 |
| relocation.max_gravity_angle_deg | 10° | 世界重力方向验收 |
| coarse_min_inliers | 3 | 最少粗对应内点 |
| coarse_min_inlier_ratio | 0.01 | 最低粗内点比例 |
| coarse_max_inlier_rmse_m | 0.4 m | 粗对应内点 RMSE 上限 |
| quality_min_source_overlap | 0.8 | 源子图覆盖率严格大于该值 |
| quality_min_current_frame_overlap | 0.8 | 最新单帧覆盖率严格大于该值 |
| quality_max_inlier_mse_m2 | 0.1 m² | 精配准及最新扫描复查 MSE 严格小于该值 |
| relocation.registration_position_sigma_m | 0.2 m | 当前人工位置膨胀标准差 |
| relocation.registration_rotation_sigma_deg | 2° | 当前人工姿态膨胀标准差，先转换为弧度 |

这些重定位参数与回环的室内、室外配置是不同的加载入口，不应把回环配置中的数值自动视为重定位门限。

## 18. 代码对应与实现边界

| 环节 | 当前代码 |
|---|---|
| 地图加载、IMU 初始化、局部跟踪、关键帧、任务与接管 | [super_lio_reloc.cpp](../src/SuperLIO_SLAM/src/lio/super_lio_reloc.cpp) |
| 固定 S 系点云、先验裁剪、TEASER++ / GICP 组合、重力检查 | [RelocationRegistration.cpp](../src/SuperLIO_SLAM/src/lio/RelocationRegistration.cpp) |
| 配准快照与结果定义 | [RelocationRegistration.h](../src/SuperLIO_SLAM/include/lio/RelocationRegistration.h) |
| 误差状态排列与 18×18 协方差 | [ESKF.h](../src/SuperLIO_SLAM/include/lio/ESKF.h) |
| ChangeReferenceFrame()、状态注入、传播缓存 | [ESKF.cpp](../src/SuperLIO_SLAM/src/lio/ESKF.cpp) |
| 粗配准与几何内点检查 | [TeaserCoarseRegistration.cpp](../src/SuperLIO_SLAM/src/loop/TeaserCoarseRegistration.cpp) |
| 精配准、覆盖率、MSE 和退化诊断 | [GicpRegistration.cpp](../src/SuperLIO_SLAM/src/loop/GicpRegistration.cpp) |
| 点云、位姿、TF 和参考系切换 | [ROSWrapper.cpp](../src/SuperLIO_SLAM/src/ros/ROSWrapper.cpp) |
| 启动和独立重定位参数 | [relocation.launch.py](../src/SuperLIO_SLAM/launch/relocation.launch.py)、[relocation.yaml](../src/SuperLIO_SLAM/config/relocation/relocation.yaml) |

当前实现的确定性参考系变换与其右乘姿态误差定义一致。完整的配准不确定性与相关性传播仍是后续工作。

配准质量门限、最新扫描复查、generation 校验和协方差更新分别解决不同问题。增大协方差不能修复错误的重定位变换；较低配准 MSE 也不能证明全局位置唯一正确。

## 19. 理论与实现检查清单

- [ ] LiDAR–IMU 外参方向正确，单帧点云已经统一到对应 IMU 参考时刻。
- [ ] 多帧按各自 LIO 位姿累积，固定 \(S\) 系未与当前锚点系混用。
- [ ] 配准源为 \(S\) 系、目标为 \(W\) 系，结果确实是 \(T_{W,S}\)。
- [ ] 提示坐标变换只组合一次，GICP 完整最终变换未重复乘初值。
- [ ] 迟到结果作用于最新状态，并保留最新滤波时间。
- [ ] 姿态采用右乘误差时，使用本项目的 \(J\)，而不是另一种误差定义的雅可比。
- [ ] 既有协方差及交叉块整体执行 \(JPJ^\top\)。
- [ ] 人工膨胀标准差先转换单位再平方，未把 MSE、RMSE 和角度方差混用。
- [ ] 不把当前人工膨胀描述成已标定的 GICP 协方差或完整 Bayesian 融合。
- [ ] 若引入完整变换协方差，先明确扰动坐标、参数顺序和状态—配准相关性。
- [ ] 接管后地图缓存、传播缓存、输出参考系和 TF 链同步更新。
- [ ] 内部协方差更新与 ROS 消息协方差发布分别检查。

## 20. 参考资料

以下资料用于理解误差状态、旋转扰动及位姿不确定性；本文的具体 \(J\)、\(G\) 和当前行为以项目状态定义和代码为准。

1. Joan Solà. Quaternion kinematics for the error-state Kalman filter. 2017. [arXiv:1711.02508](https://arxiv.org/abs/1711.02508)。
2. Timothy D. Barfoot and Paul T. Furgale. Associating Uncertainty With Three-Dimensional Poses for Use in Estimation Problems. IEEE Transactions on Robotics, 30(3), 679–693, 2014. [DOI:10.1109/TRO.2014.2298059](https://doi.org/10.1109/TRO.2014.2298059)。
