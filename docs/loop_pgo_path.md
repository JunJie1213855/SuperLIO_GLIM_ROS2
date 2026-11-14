# Scan Context 闭环检测、GICP 配准与 iSAM2 PGO 的理论和实现

本文对应当前在线回环实现，介绍 Scan Context 候选筛选、多帧子图配准与 iSAM2 优化。参考地图重定位使用独立的 [TEASER++ 与 GICP 流程](relocation_math.md#6-粗配准与精配准)。

文中“闭环”和“回环”含义相同。**检索到候选、得到配准变换、接收图约束、完成地图更新是四个不同阶段。** 数学推导、当前实现和未实现的扩展分别说明，参数以实际加载 YAML 为准。

## 1. 整体流程

前端发布 IMU 系关键帧点云和原始 LIO 位姿。后端建立里程计因子，并对符合时间与距离门限的历史关键帧做 Scan Context 匹配。候选通过多帧子图 GICP 和质量检查后，作为相对位姿约束加入 iSAM2。优化结果用于发布轨迹、重建地图以及计算 map→world 修正。

## 2. 坐标系与变量

### 2.1 统一变换方向

使用列向量和左乘：

$$
{}^A p=T_{A,B}\,{}^B p,
\qquad
T_{A,C}=T_{A,B}T_{B,C}.
$$

即 \(T_{A,B}\) 把 \(B\) 系的坐标变换到 \(A\) 系。刚体变换属于：

$$
SE(3)=
\left\{
\begin{bmatrix}R&t\\0&1\end{bmatrix}
:
R^\top R=I,\ \det R=1
\right\}.
$$

### 2.2 前端和后端的两个世界系

| 符号 | 含义 | 当前实现中的量 |
|---|---|---|
| \(O\) | 原始 LIO 里程计参考系 | world |
| \(M\) | PGO 优化后的地图参考系 | map |
| \(C_i\) | 第 \(i\) 个关键帧参考时刻的 IMU 系 | 关键帧局部点云的坐标系 |
| \(O_i=T_{O,C_i}\) | 原始 LIO 位姿 | Frame::odom_pose |
| \(X_i=T_{M,C_i}\) | 优化关键帧位姿 | Frame::map_pose |
| \(\mathcal P_i\) | 第 \(i\) 帧局部点云 | Frame::cloud，IMU 系 |
| \(c,h\) | 当前锚帧、历史锚帧索引 | 配准任务保存的两个索引 |

单帧点云应已经过 LiDAR–IMU 外参变换和扫描去畸变。若把原始 LiDAR 点误当成 IMU 系点，或者把已变到 world 的点云再次乘 \(O_i\)，会造成坐标错误。

\(O_i\) 和 \(\mathcal P_i\) 在保存后作为原始数据使用；PGO 更新 \(X_i\)，不覆盖前端的原始位姿和单帧点云。

## 3. 关键帧与里程计图的建立

前端根据平移、旋转和最长时间间隔选择关键帧。公共 slam.yaml 当前设置为 1.5 m、10°、3 s；传感器和启动参数可以覆盖这些值。

收到关键帧后，后端先检查输入合法性、点数和 ID 顺序，再保存局部点云及原始位姿。后端索引是自身连续索引，不一定等于输入消息 ID。

相邻关键帧的里程计测量为：

$$
\boxed{
Z^{\mathrm{odom}}_{i-1,i}=O_{i-1}^{-1}O_i.
}
$$

因此可以持续给图添加节点和里程计边，即使当前没有任何回环。

新节点的优化初值为：

$$
\boxed{
X_i^{(0)}
=
X_{i-1}^{\mathrm{stored}}Z^{\mathrm{odom}}_{i-1,i}.
}
$$

该式把最新原始相对运动接到已有优化轨迹后面。首节点初值为 \(O_0\)，并添加首帧位姿先验。

## 4. Scan Context 闭环候选检测

每个关键帧生成一张 20 环 × 60 扇区的极坐标高度描述图，最大半径 80 m。沿扇区方向对每环求平均得到 ring key。后端在通过时间和距离筛选的历史帧中，按 ring key 欧氏距离选出最近的 10 帧，再使用循环移位对齐描述图，按列比较余弦相似度。默认只保留 SC 距离严格小于 0.13 的候选。

循环移位给出把当前帧转到历史帧的相对偏航角。扇区宽度为 \(360^\circ / 60 = 6^\circ\)。当前帧完成检索后才加入描述子历史，因此不会被自身召回。

## 5. 候选时间、距离和回环间隔

Scan Context 检索使用原始 LIO 位姿和传感器时间戳筛选历史帧：

$$
\boxed{
t_c-t_h>\Delta t_{\min},
\qquad
\|p_c^O-p_h^O\|\le r_{\max}.
}
$$

公共默认值为：

$$
\Delta t_{\min}=30\ \mathrm s,
\qquad
r_{\max}=15\ \mathrm m.
$$

时间差严格大于门限，恰好 30 s 也拒绝；空间距离允许等于门限。时间差来自消息的传感器时间，而非电脑时间或后台任务等待时间。

时间和距离筛选完成后，候选按 Scan Context 距离排序。

当前还包含：

- registration_top_k：每个任务最多保留若干候选，默认 3。
- 已选候选邻近两个索引的重复候选抑制。
- loop_min_interval：接受一个回环后，以当前源帧索引为基准的关键帧数冷却，默认 20。
- registration_soft_budget_ms：运行超过软预算后不再启动下一个候选，默认 500 ms；不强行中断运行中的求解器。

距离门限依赖 LIO 漂移仍处于合理范围；它可以减少远处误匹配，也可能漏掉漂移超过半径的真实回访。

时间门限、帧数冷却、候选空间距离和子图内部累计路程是四个不同参数，不能混用。

## 6. 当前锚定的多帧源子图

定义：

$$
S=C_c.
$$

第 \(i\) 帧点满足：

$$
{}^O p=O_i\,{}^{C_i}p
=O_c\,{}^S p.
$$

所以：

$$
\boxed{
T_{S,C_i}=O_c^{-1}O_i.
}
$$

若 \(O_i=[R_i,p_i]\)，则：

$$
R_{S,C_i}=R_c^\top R_i,
\qquad
p_{S,C_i}=R_c^\top(p_i-p_c).
$$

源子图为：

$$
\boxed{
\mathcal Q_S
=
\operatorname{Voxel}
\left(
\bigcup_{i\in\mathcal W_c}
(O_c^{-1}O_i)\mathcal P_i
\right).
}
$$

当前锚帧满足：

$$
T_{S,C_c}=O_c^{-1}O_c=I.
$$

**单位位姿只发生在子图的坐标重表达中。** 不会把保存的 \(O_c\) 或优化变量 \(X_c\) 赋值为单位矩阵，也不会重置前端 ESKF。

对任意共同世界变换 \(G\)：

$$
(GO_c)^{-1}(GO_i)=O_c^{-1}O_i.
$$

因此相对构图不依赖 LIO 世界系原点和整体朝向。但窗口内各帧不同的估计误差并不会抵消；相对运动不准会使子图发生内部畸变。

## 7. 历史锚定目标子图和 deque

定义目标局部系：

$$
H=C_h.
$$

历史因果窗口使用锚帧及其之前的关键帧：

$$
\boxed{
\mathcal Q_H
=
\operatorname{Voxel}
\left(
\bigcup_{j\in\mathcal W_h}
(O_h^{-1}O_j)\mathcal P_j
\right).
}
$$

当前多帧路径的窗口限制通常为最多 4 帧、6 s、6 m 累计路程。滑窗裁剪累计路程时使用：

$$
D=\sum_{k}\|p_k^O-p_{k-1}^O\|,
$$

而不是只比较首尾位置。因此来回运动也会消耗路程预算。

SubmapWindow::cloud() 根据窗口末帧作为锚点重建点云；锚点变化后，相对变换也相应变化。源和历史窗口不能共享关键帧，否则会因重复使用同一数据而得到虚假的高覆盖。

\(S\) 和 \(H\) 的锚帧在各自坐标系中均为单位位姿，但它们不是同一个坐标系。待估计的正是：

$$
T_{H,S}.
$$

配准任务保存独立窗口快照和共享不可变点云引用。即使入口继续添加关键帧，该任务的源锚 \(c\)、历史锚 \(h\) 和窗口成员也不会改变。

## 8. GICP 初始位姿

记原始 LIO 相对位姿为 \(T_{HC}^{\mathrm{LIO}}=(T_{WH})^{-1}T_{WC}\)，其中 \(H\) 是历史锚帧，\(C\) 是当前锚帧。Scan Context 给出 \(\psi_{\mathrm{SC}}\)，从 LIO 相对旋转提取 \(\psi_{\mathrm{LIO}}\)，用以下初值对齐两个子图：

\[
R_0=R_z(\psi_{\mathrm{SC}}-\psi_{\mathrm{LIO}})R_{HC}^{\mathrm{LIO}},
\qquad t_0=t_{HC}^{\mathrm{LIO}}.
\]

若该初值未通过 GICP 验收且不同于原始 LIO 初值，后端可用 \(T_{HC}^{\mathrm{LIO}}\) 再验证同一对点云。若子图或当前帧点数不足以进入 GICP，则直接拒绝。两次尝试使用同一套质量门限。

## 9. GICP 精配准的理论

### 9.1 点到分布的几何模型

设当前变换下的对应为 \(a_k\leftrightarrow b_{\pi(k)}\)，点邻域协方差为 \(C_k^a,C_{\pi(k)}^b\)。残差和组合协方差为：

$$
r_k=b_{\pi(k)}-(Ra_k+t),
$$

$$
\Sigma_k=C_{\pi(k)}^b+RC_k^aR^\top.
$$

GICP 的基本目标为：

$$
\boxed{
\min_{R,t}
\sum_k r_k^\top\Sigma_k^{-1}r_k.
}
$$

局部平面沿切向和法向具有不同权重，能同时利用两侧表面结构。实际算法还涉及邻域协方差正则化、对应点更新、距离裁剪和停止条件。

它是局部优化，初值决定其进入哪个收敛域。错误初值可能使 GICP 在错误但相似的结构上收敛。

### 9.2 变换组合必须一致

当前实现保留原始源子图，将 Scan Context 偏航和 LIO 位姿组合为 GICP 初值：

~~~text
input source = Q_S
initial      = T_initial
final        = getFinalTransformation()
~~~

输出已经是完整变换：

$$
\boxed{T_{\mathrm{final}}=T_{H,S}^{*}.}
$$

不能再计算 \(T_{\mathrm{final}}T_{\mathrm{initial}}\)，否则重复施加初值。

只有在事先把源点云显式变换为 \(T_{\mathrm{initial}}\mathcal Q_S\)，并以单位初值求左修正 \(\Delta_H\) 时，才有：

$$
T_{\mathrm{final}}=\Delta_H T_{\mathrm{initial}}.
$$

当前代码使用前一种方式。

### 9.3 左修正与右修正

定义：

$$
\Delta_S=T_{\mathrm{initial}}^{-1}T_{\mathrm{final}},
\qquad
\Delta_H=T_{\mathrm{final}}T_{\mathrm{initial}}^{-1}.
$$

它们满足：

$$
\Delta_H=T_{\mathrm{initial}}\Delta_ST_{\mathrm{initial}}^{-1}.
$$

旋转角一致，但平移分量会受表达参考点影响。当前代码采用右修正 \(\Delta_S\)，检查其平移模长和旋转角。

这限制的是“GICP 结果相对初值”的变化，不是：

$$
(O_h^{-1}O_c)^{-1}T_{\mathrm{final}}
$$

的变化。因此，候选距离筛选和精配准修正限制，不能替代最终变换与 LIO 的一致性检查。

## 10. 精配准质量与退化验收

### 10.1 近邻覆盖和 MSE

对精对齐点：

$$
q_k=T_{\mathrm{final}}a_k,
\qquad
d_k=\min_{b\in\mathcal Q_H}\|q_k-b\|.
$$

质量半径为 \(r_q\)，有效集合为：

$$
\mathcal I_f=\{k:d_k\le r_q\}.
$$

定义：

$$
N_f=|\mathcal I_f|,
\quad
\alpha_f=\frac{|\mathcal I_f|}{|\mathcal Q_S|},
\quad
\operatorname{MSE}_f=
\frac{1}{|\mathcal I_f|}\sum_{k\in\mathcal I_f}d_k^2.
$$

还单独把当前单帧 \(\mathcal P_c\) 变到 \(H\) 中，计算当前帧覆盖率 \(\alpha_{\mathrm{current}}\)，避免仅窗口里的旧帧对齐而当前帧不匹配。

当前代码要求：

$$
N_f\ge N_{\min},
\quad
\alpha_f>\alpha_{\min},
\quad
\alpha_{\mathrm{current}}>\alpha_{\mathrm{current,min}},
\quad
\operatorname{MSE}_f<m_{\max}.
$$

覆盖率为严格大于，MSE 为严格小于。

MSE 使用 m²，RMSE 使用 m。例如 MSE 0.1 m² 对应 RMSE 约 0.316 m，不等于 RMSE 0.1 m。

当前覆盖率为源到目标的近邻覆盖，另加当前源单帧检查，尚非源—目标双向覆盖。

### 10.2 几何退化

单一大平面即使残差很小，也可能无法约束全部位姿自由度。当前实现从目标邻域协方差提取法向 \(n_k\)，对源点作中心化和尺度归一化：

$$
\widetilde q_k=\frac{q_k-\bar q}{s_q}.
$$

构造诊断雅可比及信息矩阵：

$$
j_k=
\begin{bmatrix}
\widetilde q_k\times n_k\\
n_k
\end{bmatrix},
\qquad
\mathcal H_{\mathrm{diag}}
=
\sum_{k\in\mathcal I_f}j_kj_k^\top.
$$

检查：

$$
\gamma=
\frac{\lambda_{\min}(\mathcal H_{\mathrm{diag}})}
{\lambda_{\max}(\mathcal H_{\mathrm{diag}})}.
$$

比例过小则拒绝。它是归一化的几何退化诊断，不是经过标定的位姿信息矩阵，不能直接当成图因子协方差的逆。

## 11. 从配准结果到图约束

因为 \(S=C_c,\ H=C_h\)：

$$
T_{\mathrm{final}}
=
T_{C_h,C_c}.
$$

而图变量满足：

$$
X_h^{-1}X_c
=
T_{M,C_h}^{-1}T_{M,C_c}
=
T_{C_h,C_c}.
$$

因此回环测量直接为：

$$
\boxed{
Z^{\mathrm{loop}}_{h,c}=T_{\mathrm{final}},
\qquad
Z^{\mathrm{loop}}_{h,c}\approx X_h^{-1}X_c.
}
$$

对应：

~~~text
BetweenFactor<Pose3>(h, c, Z_hc, loop_noise)
~~~

不能把 \(T_{\mathrm{final}}\) 作为 \(X_c\) 直接写入，也不能把它当作 map→world。

若已知 \(X_h\)，可形成预测 \(X_c=X_hZ_{h,c}\)，但优化中两端和其他节点均可能移动；最终只是带噪约束下的折中，不要求每个回环等式严格成立。

### 11.1 目标若改为 map 系

若目标点云在 \(M\) 中，配准结果为 \(B=T_{M,C_c}\)，应转换为：

$$
Z_{h,c}
=
(X_h^{\mathrm{snapshot}})^{-1}B.
$$

\(X_h^{\mathrm{snapshot}}\) 必须与构建该目标所使用的位姿快照一致。若目标是包含多个已发生不同优化修正的历史片段的全局地图，所获结果也不一定能视为独立的单帧对观测。

当前多帧实现使用局部 \(H\) 系目标，避免把全局地图形变直接混入观测定义。

### 11.2 若反转测量方向

交换因子节点顺序时，测量必须同时取逆，且协方差要按扰动约定变换。

例如定义右扰动：

$$
Z_{\mathrm{true}}=Z\operatorname{Exp}(\xi^\wedge),
$$

则：

$$
Z_{\mathrm{true}}^{-1}
=
Z^{-1}\operatorname{Exp}
\left((-\operatorname{Ad}_{Z}\xi)^\wedge\right).
$$

相应协方差为：

$$
\Sigma_{\mathrm{inverse}}
=
\operatorname{Ad}_{Z}\Sigma\operatorname{Ad}_{Z}^{\top}.
$$

不能在反转一个一般各向异性观测后，无条件保留原协方差。当前代码通过一致的历史到当前契约，不需要额外反转。

## 12. PGO 的概率模型和规范自由度

### 12.1 优化变量与因子

未知量为全部关键帧位姿：

$$
\mathcal X=\{X_0,\ldots,X_N\}.
$$

图包含三类因子：

1. 首帧位姿先验。
2. 相邻节点里程计边。
2. 验收通过的非相邻回环边。

若只有相对位姿边，对任意共同变换 \(G\)：

$$
(GX_i)^{-1}(GX_j)=X_i^{-1}X_j.
$$

因此全图有共同平移和旋转的六自由度规范自由度。先验因子固定全局参考，避免线性系统因整体规范自由度而奇异。当前先验以 \(O_0\) 为中心，并使用较小但非零噪声。

### 12.2 因子残差

以标准 SE(3) 对数形式表达：

$$
E_{ij}=Z_{ij}^{-1}X_i^{-1}X_j,
\qquad
e_{ij}=\operatorname{Log}(E_{ij})^\vee\in\mathbb R^6.
$$

满足测量时 \(E_{ij}=I,\ e_{ij}=0\)。

GTSAM 实际通过 Pose3 的 Between 和 Local 计算残差。其具体局部 chart 受版本与构建选项影响；这里的 Log 形式用于说明标准李群残差，不无条件断言所有构建的 Local 都与某个手写完整 SE(3) Log 逐项相同。

当前六维噪声顺序为：

$$
e=
\begin{bmatrix}
e_{\mathrm{rotation}}\\
e_{\mathrm{translation}}
\end{bmatrix}.
$$

旋转尺度单位是 rad，平移尺度单位是 m。图噪声参数是标准差，不是直接的权重或方差。

### 12.3 最大后验与加权最小二乘

在因子独立、噪声模型成立的近似下：

$$
p(\mathcal X\mid\mathcal Z)
\propto
p(X_0)\prod_{(i,j)}p(Z_{ij}\mid X_i,X_j).
$$

高斯噪声对应负对数目标：

$$
\frac12 e^\top\Sigma^{-1}e.
$$

当前图的概念目标为：

$$
\boxed{
\min_{\mathcal X}
\frac12\|L_0e_0\|^2
+
\frac12\sum_{(i,j)\in\mathcal E_o}\|L_{ij}^{o}e_{ij}^{o}\|^2
+
\sum_{(h,c)\in\mathcal E_l}
\rho_\kappa\left(\|L_{hc}^{l}e_{hc}^{l}\|\right),
}
$$

其中 \(L^\top L=\Sigma^{-1}\)，\(\mathcal E_o,\mathcal E_l\) 分别是里程计边和回环边，回环使用 Huber 鲁棒损失。

这也解释了优化后一般：

$$
X_c^*\ne X_h^*Z_{h,c}
$$

的严格等式：图在所有有噪测量之间折中，并非把每个回环硬焊接。

## 13. Huber 鲁棒核和当前图噪声

定义白化残差范数：

$$
s=\|\Sigma^{-1/2}e\|.
$$

Huber 损失为：

$$
\rho_\kappa(s)=
\begin{cases}
\frac12s^2,&s\le\kappa,\\
\kappa s-\frac12\kappa^2,&s>\kappa.
\end{cases}
$$

对应迭代重加权的权重为：

$$
w(s)=
\begin{cases}
1,&s\le\kappa,\\
\kappa/s,&s>\kappa.
\end{cases}
$$

当前回环使用 GTSAM 的默认块重加权方式，对白化后的整个六维残差块计算鲁棒权重。Huber 参数是无量纲的白化残差尺度，不是“多少米就拒绝”。

**Huber 只降低大残差影响，不会自动删除错误回环。** 初始残差不大的错误约束、成组一致的错误回环，仍可能明显拉坏轨迹。

当前公共 slam.yaml 的噪声如下；实际加载配置可覆盖：

| 因子 | 平移标准差 | 旋转标准差 |
|---|---:|---:|
| 首帧先验 | 0.001 m | 0.001 rad |
| 里程计边 | 0.1 m | 0.05 rad |
| 回环边 | 0.2 m | 0.1 rad |

回环 Huber 参数为 1.345。代码通过 Diagonal::Sigmas() 使用这些标准差。

当前图权重不是从每次 GICP 的协方差自动计算的，配准 MSE、覆盖率和退化信息比主要用于验收和日志。图因子仍使用配置的固定噪声。

更小的标准差意味着更强的约束，不等于更可靠；将错误回环的标准差调小，只会让图更相信错误测量。

### 13.1 一个一维折中例子

固定 \(x_0=0\)，里程计测量 \(x_1=10\)，回环测量 \(x_1=9\)。暂不加鲁棒核：

$$
\min_{x_1}
\frac{(x_1-10)^2}{2\sigma_o^2}
+\frac{(x_1-9)^2}{2\sigma_l^2}.
$$

解为：

$$
x_1^*
=
\frac{10/\sigma_o^2+9/\sigma_l^2}
{1/\sigma_o^2+1/\sigma_l^2}.
$$

当 \(\sigma_o=0.1,\sigma_l=0.2\) 时，\(x_1^*=9.8\)，不是直接等于回环测量 9。多节点 SE(3) 图中的优化同样是整体加权折中，只是具有非线性和稀疏结构。

## 14. SE(3) 上的线性化与稀疏求解

### 14.1 局部增量

采用角度在前的六维切向量：

$$
\delta\xi=
\begin{bmatrix}\delta\theta\\\delta u\end{bmatrix},
\qquad
\delta\xi^\wedge=
\begin{bmatrix}
[\delta\theta]_\times&\delta u\\0&0
\end{bmatrix}.
$$

用右扰动说明优化更新：

$$
X_i(\delta\xi_i)=\bar X_i\operatorname{Exp}(\delta\xi_i^\wedge).
$$

增量为局部切向量，不是直接对旋转矩阵九个元素作独立加法。

因子一阶展开：

$$
e_{ij}(\delta)
\approx
\bar e_{ij}+A_{ij}\delta\xi_i+B_{ij}\delta\xi_j.
$$

每条相对位姿因子只关联两个节点，因此整体雅可比是稀疏的。

### 14.2 标准 Log 残差的局部雅可比

令 \(\bar Y=\bar X_i^{-1}\bar X_j,\ \bar E=Z^{-1}\bar Y\)。右扰动后：

$$
E(\delta)
=
\operatorname{Exp}
\left((-\operatorname{Ad}_{Z^{-1}}\delta\xi_i)^\wedge\right)
\bar E
\operatorname{Exp}(\delta\xi_j^\wedge).
$$

若使用标准 Lie Log，记 \(J_l,J_r\) 为 SE(3) 左、右雅可比，则：

$$
A_{ij}=-J_l^{-1}(\bar e)\operatorname{Ad}_{Z^{-1}},
\qquad
B_{ij}=J_r^{-1}(\bar e).
$$

当残差接近零：

$$
A_{ij}\approx-\operatorname{Ad}_{Z^{-1}},
\qquad
B_{ij}\approx I.
$$

对角度在前的约定：

$$
\operatorname{Ad}_{[R,t]}
=
\begin{bmatrix}
R&0\\
[t]_\times R&R
\end{bmatrix}.
$$

这些式子说明相对位姿线性化的坐标耦合。当前代码交由 GTSAM 计算其所选 chart 的雅可比，不额外手写并替换上述矩阵。

### 14.3 白化、重加权和增量求解

将各因子白化并乘鲁棒权重的平方根后，可写成：

$$
\min_\delta\frac12\|\mathcal A\delta+\mathcal b\|^2.
$$

概念正规方程为：

$$
\mathcal A^\top\mathcal A\,\delta
=-\mathcal A^\top\mathcal b.
$$

实际稀疏实现可使用消元和平方根分解，不需要显式构造稠密逆矩阵。求得增量后在流形上更新节点，重新线性化必要因子，再继续迭代。

鲁棒权重依赖残差，因此图中加入回环后，重线性化和额外迭代也会改变鲁棒权重，不能将一次线性求解等同于完整非线性收敛。

## 15. iSAM2 和 Bayes tree

PGO 定义要优化的目标；iSAM2 是高效增量求解该目标的方法，两者不是两个独立的位姿优化目标。

对当前线性化图进行变量消元，可形成条件概率分解。Bayes tree 将相关条件组织为团和分离变量，表达稀疏高斯结构。

加入新节点或回环因子后，iSAM2 主要执行：

1. 找到与新增因素及需重线性化变量有关的树区域。
2. 移除受影响部分，保留可复用的子树。
2. 重新线性化相关非线性因子。
3. 必要时重新排序受影响变量以控制填充。
4. 对受影响区域重新消元并接回树。
6. 求解或传播增量，更新当前估计。

它并非每一帧都从头构造和求解整个批量问题。但大跨度回环可能影响大范围轨迹，不能保证任意回环都只有常数耗时。

### 15.1 当前参数和调用

当前设置：

~~~text
relinearizeThreshold = 0.01
relinearizeSkip      = 1
~~~

新增里程计节点：

~~~text
isam.update(new_odometry_factors, new_node_initial_values)
estimate current node
~~~

迟到回环提交：

~~~text
isam.update(loop_factor, empty_initial_values,
            force_relinearize=true, forceFullSolve=true)
repeat 3 times:
    isam.update(empty_factors, empty_initial_values, same_update_parameters)
estimate all existing nodes
~~~

空更新不重复加入回环因子，也不重复创建节点。

force_relinearize 使该次更新检查重线性化而不受常规检查间隔限制，并不表示忽略全部阈值、强制每一个历史变量都重新线性化。forceFullSolve 控制增量求解范围，不等于保证全局最优或每次进行完整批量非线性优化。

固定的额外三次更新用于改善迟到约束下的线性化和鲁棒权重，仍不是经过收敛判据验证的任意精度保证。

## 16. 异步回环的节点和时间一致性

配准源锚为旧节点 \(c\)，结果返回时图中最新节点可能已经是 \(k>c\)。

正确处理：

- 因子仍连接 \(h,c\)，不把源端索引换成 \(k\)。
- 两个端点已经存在，不给它们重新插入 initial value。
- 验收并提交后刷新全部历史 \(X_i\)。
- 当前实时输出使用最新节点 \(k\)，而不是回退到 \(c\)。

后台执行前和最终提交前都会复查候选时间和距离条件。回环冷却在工作线程执行时也会检查，以避免排队任务继续密集插入约束。

图状态和 iSAM2 由互斥锁保护；点云配准及地图点云重建不持有该图锁。图优化和快照发布仍可能短时阻塞关键帧入口，不能将当前实现称为完全无锁或硬实时系统。

## 17. PGO 如何改变轨迹

回环边给相距较远的历史节点建立新相对约束，优化同时考虑全部里程计边、先验和已有回环。因此它会把误差分布到相关节点，而不是只移动最后一帧。

可以为每个节点定义修正：

$$
C_i=X_i^*O_i^{-1}.
$$

通常：

$$
C_i\ne C_j.
$$

这意味着历史轨迹校正不是对整个原始轨迹施加同一个刚体变换。它可以表现为不同片段的平移、旋转及路径形状变化，但每个单帧点云仍按一个刚体位姿放置。

当前只优化关键帧六自由度位姿，没有在 PGO 图中联合优化速度、IMU 偏置、重力、外参或原始点坐标。

## 18. 最新 map→world 修正和前端输出

图中最新节点为 \(k\)，定义：

$$
\boxed{
T_{M,O}=X_k^*O_k^{-1}.
}
$$

之后某个前端时刻 \(t\) 的地图系位姿可表达为：

$$
\widehat T_{M,C(t)}
=
T_{M,O}O(t)
=
X_k^*\left(O_k^{-1}O(t)\right).
$$

这将最新优化位姿与之后的原始相对运动接起来。异步回环完成后应使用最新节点 \(k\) 计算该修正，而不是使用旧源帧 \(c\)。

当前 TF 使用最新前端里程计数据的传感器时间；延迟回环只更新变换内容，不把 TF 时间退回旧关键帧时刻，也不按电脑当前时间向前推进。回放暂停时，数据时间停止。

这个修正不代表前端 ESKF 被重新初始化。当前 PGO 不将优化结果或图协方差直接写回前端滤波器，也不重建前端内部用于里程计跟踪的体素地图。

这与重定位模块接管 ESKF 的行为不同，后者见 [重定位数学说明](relocation_math.md)。

## 19. 优化地图怎样重建

正确的关键帧地图为：

$$
\boxed{
\mathcal M_M
=
\operatorname{Voxel}
\left(
\bigcup_i X_i^*\mathcal P_i
\right).
}
$$

各帧必须分别使用自己的优化位姿。只把原始全局地图整体乘最新 \(T_{M,O}\)，无法表达历史各节点不同的修正 \(C_i\)。

当前地图线程读取一致的优化位姿与单帧云快照，先按点数上限采样，再变换、合并及体素滤波。公共配置限制最多 2000 个关键帧、每帧最多 1000 点，地图体素 0.5 m；这些是发布地图的近似，不是图节点删除策略。

注册子图 \(\mathcal Q_S,\mathcal Q_H\) 是配准的临时输入，不能再作为新的独立关键帧重复加入地图，否则会重复累积窗口中的同一帧。

回环接收后立即请求重建；普通关键帧按 map_publish_stride 请求。地图最多一个任务运行、一个最新任务等待，新的待处理快照覆盖旧待处理快照。

运行中的旧地图可以先完成发布，随后由更新快照替换；单地图线程避免不同任务完成顺序倒置。此显示过程不是增量地图形变。

### 19.1 保存完整优化地图

`/lio_slam/save_optimized_map`（`std_srvs/srv/Trigger`）服务与关键帧入口位于同一个互斥回调组。
停止数据输入、确认关键帧接收完成后调用，服务等待运行中及已排队的回环任务完成，然后在图锁内获取全部关键帧优化位姿与不可变点云指针快照，锁外执行磁盘写入。

导出逐帧应用 \(X_i^*\)，不使用最新统一修正代替历史位姿，不受显示帧数、每帧显示点数或路径长度上限限制。
“完整”指全部已保留关键帧的点，不能恢复关键帧进入后端前后已过滤或采样丢弃的原始测量。

参数 `save_map_path` 指定输出 PCD，`save_map_voxel_size=0` 直接流式写盘；正数用哈希体素增量累加世界系坐标与强度，最终输出每个体素的质心。
因此流式模式无需额外完整世界点云副本；体素模式额外内存主要取决于占用体素数。
文件在输出目录中暂存，全部成功后原子替换目标 PCD，失败不覆盖旧地图。

`save_map_on_shutdown=true` 时，正常退出也先排空配准队列再保存；强制终止不能保证保存。
服务或退出等待只涵盖后端已接收的关键帧和未被过载淘汰的检测任务，不会补回 DDS 接收队列尚未处理或先前丢弃的消息。

## 20. 为什么可视化中的两条轨迹可能不重合

原始轨迹在 \(O\) 中。如果 RViz 通过当前统一 TF 把整条原始轨迹显示到 \(M\)，第 \(i\) 个历史点显示为：

$$
T_{M,O}^{\mathrm{latest}}O_i.
$$

优化轨迹则是：

$$
X_i^*.
$$

由于各帧修正一般不同：

$$
T_{M,O}^{\mathrm{latest}}O_i\ne X_i^*.
$$

因此原始 Path 和优化 Path 同时显示时，不重合并不自动证明所有回环错误。应区分：

- /lio/path：原始前端轨迹，经 TF 显示。
- /lio_slam/path：逐关键帧优化轨迹。
- /lio/cloud_world：前端点云。
- /lio_slam/map：后端按优化位姿重建的地图。

反过来，不能把明显的地图重影和轨迹突变都解释成显示问题。应检查被接收回环的端点、源目标子图对齐、配准变换和图残差；小 MSE 或鲁棒核都不排除错误约束。

## 21. 图协方差、配准协方差和 ESKF 协方差

三者不能混用：

| 量 | 表达对象 | 当前用途 |
|---|---|---|
| 点邻域协方差 | 点云局部表面结构 | GICP 代价的几何权重 |
| 配准位姿协方差 | 一个六自由度相对变换的不确定性 | 当前未完整标定并用于因子赋权 |
| PGO 节点边缘协方差 | 所有图因子条件下某个 \(X_i\) 的不确定性 | 当前未作为 ESKF 接管输入 |
| ESKF 协方差 | 姿态、位置、速度、偏置和重力误差 | 前端状态估计 |
| 退化诊断信息比 | 几何约束方向相对强弱 | 拒绝明显退化配准 |

在线性化点和固定鲁棒权重下，图联合协方差可以近似联系到：

$$
\Sigma_{\mathcal X}
\approx
(\mathcal A^\top\mathcal A)^{-1}.
$$

实际求边缘协方差应利用稀疏结构；并非显式求整个大矩阵逆。该近似还受规范固定、线性化、噪声模型和鲁棒权重影响，不能保证正确描述误匹配导致的多峰误差。

相邻里程计边和重复窗口回环通常存在数据相关性；当前图以固定对角噪声近似，并没有完整维护这些交叉相关性。增加高度重复的边不应被解释为同比例增加独立信息。

## 22. 当前配置的分工

- slam.yaml 和 unitree_slam.yaml 管理关键帧、Scan Context、时间与距离筛选、图优化和地图发布。
- registration_config 指定子图窗口、GICP 和质量门限。
- VLP32 和 S3E 默认使用室外配置，体素 0.5 m，子图与当前帧覆盖率均大于 0.90，MSE 小于 0.15 m²。
- Unitree 和 Airy 默认使用室内配置，体素 0.3 m，两个覆盖率均大于 0.90，MSE 小于 0.10 m²。
- 通用配置对应 0.5 m 体素，两个覆盖率均大于 0.80，MSE 小于 0.10 m²。

## 23. 队列、运行预算与观测信息

配准待处理队列默认容量为 2，另有最多一个运行任务。队列满时淘汰最旧待处理检测任务，不删除已保存的关键帧、描述子或里程计节点。

这保护入口吞吐，但会减少过载时的回环检测覆盖率。ROS 接收历史本身也是有限的；不能据此保证任意输入速度下绝不丢消息。

单候选运行超出 500 ms 软预算时，代码不再启动下一个候选，但不会强杀当前求解或已经进入的提示重试。

阶段日志用于区分：

- 描述子生成。
- 检索和几何提示计算。
- 入口总处理与数据时间滞后。
- 配准排队和工作耗时。
- 地图快照排队和重建。
- 被丢弃的旧检测任务数。

减少日志不会改变求解复杂度；增大队列也不能解决长期处理速率低于输入速率的问题。

## 24. 实现边界与误回环防护

当前已经有：传感器时间和原始位置筛选、当前单帧与子图覆盖率、MSE、精配准修正量、几何退化检查、图鲁棒核和异步索引一致性。

当前仍不能等价声称已经实现：

- 最终配准相对 LIO 预测的完整统计一致性门控。
- 源—目标双向覆盖和动态物体分离。
- 连续多帧候选的联合时序验证。
- 可切换约束、回环隔离或自动回滚错误因子。
- 完整数据相关性建模或逐回环配准协方差标定。
- 将优化状态和协方差反馈到前端 ESKF。
- 前端体素地图跟随 PGO 形变重建。

因此，不能只因 GICP 收敛、局部残差低或 iSAM2 返回估计，就宣称闭环真实正确。应同时核对相对位姿、子图几何、端点语义和整条轨迹的变化；有真值时再评价 ATE / RPE。

## 25. 数学和代码对应

| 环节 | 当前代码 |
|---|---|
| 关键帧局部点云与原始位姿发布 | [ROSWrapper.cpp](../src/SuperLIO_SLAM/src/ros/ROSWrapper.cpp) |
| Scan Context 描述子和相对偏航 | [Scancontext.cpp](../src/SuperLIO_SLAM/SC/cpp/module/Scancontext/Scancontext.cpp) |
| 当前锚定的滑窗构图 | [SubmapWindow.cpp](../src/SuperLIO_SLAM/src/loop/SubmapWindow.cpp) |
| GICP、质量和退化验收 | [GicpRegistration.cpp](../src/SuperLIO_SLAM/src/loop/GicpRegistration.cpp) |
| 候选、图构建、异步回环提交、路径和地图 | [LoopBackendNode.cpp](../src/SuperLIO_SLAM/src/loop/LoopBackendNode.cpp) |
| 后端状态与快照结构 | [LoopBackendNode.hpp](../src/SuperLIO_SLAM/include/loop/LoopBackendNode.hpp)、[SubmapRegistration.hpp](../src/SuperLIO_SLAM/include/loop/SubmapRegistration.hpp) |
| 图与候选配置 | [slam.yaml](../src/SuperLIO_SLAM/config/loop/slam.yaml) |
| 室内 / 室外配准配置 | [室内](../src/SuperLIO_SLAM/config/loop/loop_indoor_gicp.yaml)、[室外](../src/SuperLIO_SLAM/config/loop/loop_outdoor_gicp.yaml) |
| 公共启动参数 | [slam.launch.py](../src/SuperLIO_SLAM/launch/slam.launch.py) |


## 26. 参考资料

以下是算法背景的原始论文和官方资料；本项目具体的变换方向、门限与线程语义以当前代码为准。本文推导不等于所引用库对任意输入的性能承诺。

2. Aleksandr Segal, Dirk Haehnel, Sebastian Thrun. Generalized-ICP. Robotics: Science and Systems, 2009. [原始论文入口](https://roboticsproceedings.org/rss05/p21.html)。
3. Michael Kaess et al. iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree. IJRR, 2012. [作者公开论文](https://people.csail.mit.edu/kaess/pub/Kaess12ijrr.pdf)。
4. [GTSAM 官方 ISAM2 文档](https://borglab.github.io/gtsam/isam2/)。
