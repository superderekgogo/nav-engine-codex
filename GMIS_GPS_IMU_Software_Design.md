# GMIS GPS/IMU 组合导航软件设计文档

**文档状态：** 设计草案 v0.1（供算法与软件评审）
**设计依据：** GMIS 软件架构与开发指南、GMIS GPS/IMU Navigation 技术报告
**目标：** 形成可用 C++/Eigen 实现、支持传感器与观测模型插件扩展的 GMIS 组合导航软件架构。

---

## 1. 执行摘要

Generic Multi-sensor Integration Strategy（GMIS）的核心是用同一个全值状态估计框架处理不同传感器：载体的运动学状态由离散运动模型传播；IMU、GNSS 以及后续传感器以各自的观测模型进入滤波更新。GMIS 不把 IMU 作为唯一的状态传播输入，也不把 GNSS 更新写死在导航滤波器内。滤波器接收通用的预测模型和观测模型，执行状态估计；GMIS 专用模块负责提供运动模型、IMU/GNSS 模型、约束与诊断。

本设计把现有 INS/RTK 工程作为相邻算法实现而非直接改造成 GMIS。数据解码、时间系统、坐标变换、配置、日志和分析工具可以复用；传统 INS mechanization + error-state filter 与 GMIS 全值状态滤波分别实现、分别配置、分别回放比较。GMIS 的导航解算路径不调用传统 INS mechanization 推进状态。

首期目标为单 IMU + GNSS 的 Euler 姿态参数化 GMIS，并在配置中支持 `GMIS_LC` 与 `GMIS_LTC` 两种解算模式：

- 运动学状态时间传播；
- IMU 原始角速度与比力测量更新；
- `GMIS_LC`：RTK 位置/速度与 IMU 原始观测共同更新；
- `GMIS_LTC`：RTK 位置与 IMU 原始观测共同更新，并加入 L1 相邻历元时间差分载波相位构造的区间位移/速度观测；
- 可选 `GMIS_TC` 原始 GNSS 观测模型及动态双差模糊度管理，作为独立模式保留，不与 LTC 混淆；
- 可配置等式/伪观测约束；
- innovation、后验残差、冗余度、VCE 和故障检测诊断接口；
- JSON 多文件配置和异步多速率数据处理。

Quaternion、DCM、多 IMU、整周模糊度固定和实时 FDE 自动隔离纳入后续扩展，不应由首期接口假装已经完成。

## 2. 输入文档梳理与设计解读

### 2.1 架构指南表达的设计意图

架构指南将 GMIS 与传统 TMIS/ESKF 的主要差异归纳为：

1. **全值状态估计。** 状态保存位置、速度、加速度、姿态、角运动及传感器误差等物理量，而不是只估计 INS 标称解的误差增量。
2. **运动学时间传播。** `Predict` 由时间间隔与运动状态转移模型推进，不通过积分 IMU 原始测量完成捷联惯导 mechanization。
3. **IMU 直接作为观测。** 陀螺和加速度计原始数据与 GNSS 一样，构造 innovation 和观测残差，进入滤波测量更新。
4. **传感器模型可组合。** LC 以 GNSS 接收机位置/速度作观测；TC 以基站/流动站原始 GNSS 观测作观测。新增传感器应提供模型，而不是修改通用滤波器。
5. **统一统计诊断。** 测量残差、冗余度和方差分量估计应能按传感器组和通道输出。

### 2.2 技术报告给出的算法范围

技术报告进一步给出单 IMU + GPS 基础状态、三阶平动运动学模型、Euler 姿态运动学、IMU 陀螺/比力模型、GPS LC 杆臂观测、TC 双差伪距/载波相位模型、等式约束投影、VCE/FDE 和 C++ 接口草图。其名义基础状态为 27 维；紧组合增加与当前有效双差卫星对应的模糊度状态。

### 2.3 本设计采用的解释与边界

- GMIS 是一个新的估计方案，不是对现有 LTC/ESKF 程序的重命名。
- `GMIS_LC` 与 `GMIS_LTC` 是 GMIS 的算法配置，不是现有 TMIS 的 LC/LTC 配置别名；两套实现必须有独立状态、滤波历史与输出标识。
- 本设计的 `GMIS_LTC` 沿用已确认的项目级 LTC 观测定义：RTK 天线位置作为松组合位置观测，L1 相邻历元双差载波相位差形成天线区间位移/等效区间速度观测。RTK 速度默认不加入 LTC，以免与 TDCP 导出的速度重复约束；如需加入，必须显式配置并处理它与 TDCP 的相关性。
- 通用 `StateEstimator` 只负责通用的非线性滤波计算，不知道 GPS、IMU、LTC 或状态块的物理含义。
- 由 GMIS 专用 `GmisModelSet` 组合运动先验、IMU/GNSS 观测模型和约束模型，再由 `FilterManager` 调用 `StateEstimator`。
- 为避免把未完成的模型当成既定事实，首期明确只实现经数值核验的状态/模型集合；扩展参数化必须有自己完整的状态布局、传播 Jacobian 和观测 Jacobian。
- 本设计保留 LC 和 TC 两种 GNSS 接入模式。GMIS-LTC 是否作为特定观测组合加入，留给模型配置；不得把现有 LTC 的 TDCP 定义误称为报告中的 GMIS-TC 双差原始观测模型。

## 3. 设计目标与非目标

### 3.1 设计目标

- C++17 或更高版本，Eigen 矩阵运算，CMake 构建。
- 模块化、面向对象、低耦合；模块通过类型化消息和模型接口传递数据。
- 状态布局动态管理，支持固定状态块和卫星模糊度等动态状态块。
- 支持 IMU 高频、GNSS 低频、异步观测和时间戳排序。
- 观测模型可以按配置启用/停用，滤波器核心不依赖具体传感器。
- 日志能重建每次预测、观测更新和约束更新的输入、输出及诊断量。
- 可通过同一数据集并行运行传统 INS/RTK 与 GMIS，做一致的 performance 对比。

### 3.2 首期不承诺的能力

- 自动保证任意姿态参数化的等价性或数值稳定性。
- 仅凭接口草图即支持多 IMU 阵列或所有误差耦合项。
- 在观测建模、相关性建模和数据连续性未确认时宣称 TC 模糊度可可靠固定。
- 将滤波 covariance 的平方根对角线直接等同于实际定位误差或真值 RMS。
- 未验证的在线 VCE/FDE 自动调权/剔除策略。

## 4. 坐标、符号和单位规范

统一使用以下约定，具体数据转换在 adapter 层完成：

| 项目 | 规范 |
|---|---|
| ECEF | 地固坐标系，位置 m，速度 m/s |
| Local navigation frame | 首期统一选 NED 或 ENU，由配置指定；同一次运行不可混用 |
| Body frame | 按 IMU 安装定义；安装旋转矩阵明确为 body-from-IMU 或 IMU-from-body，禁止含糊命名 |
| 姿态 | 内部矩阵方向命名为 `C_body_to_nav`；Euler 顺序与旋转顺序在配置和测试中固定 |
| 角度/角速率 | 内部使用 rad、rad/s |
| 比力 | m/s^2；明确是 specific force，不是已经扣除重力的线加速度 |
| GNSS 伪距/相位 | m / cycle；相位转米时必须使用卫星信号对应波长 |
| GNSS 时间 | 标准化为连续秒数并保留原始时间系统和时间标签 |
| 协方差 | 状态单位的平方；配置中的标准差需显式转换为方差 |

所有数据类型使用带单位注释的强语义结构体；严禁把度和弧度、cycle 和 meter、ECEF 和 local 坐标放在无标签 `VectorXd` 中跨模块传递。

## 5. GMIS 数学模型

### 5.1 首期全值状态

单 IMU、Euler 姿态基础状态定义为：

\[
\mathbf{x}_b = [\mathbf{r}^n,\mathbf{v}^n,\mathbf{a}^n,
\boldsymbol\Theta,\boldsymbol\omega_{nb}^b,
\mathbf b_g,\mathbf b_a]^T
\]

GMIS 技术报告给出含陀螺/加计比例因子的 27 维候选状态。参考 `nav-engine-cursor-3` 的 LC 实现使用 21 维，当前工程首期也采用该 21 维布局；比例因子作为外部标定量，不在滤波器中估计。TC 若启用，再增加 M 个双差模糊度状态。

| 状态块 | 符号 | 维数 | 工程单位 | 首期策略 |
|---|---:|---:|---|---|
| 位置 | `position_n` | 3 | m | 必需 |
| 速度 | `velocity_n` | 3 | m/s | 必需 |
| 线加速度 | `acceleration_n` | 3 | m/s^2 | 必需 |
| 全值姿态 | `attitude_euler` | 3 | rad | 必需；角度顺序需锁定 |
| body 相对 nav 角速度 | `angular_rate_nb_b` | 3 | rad/s | 必需 |
| 陀螺零偏 | `gyro_bias` | 3 | rad/s | 可估计 |
| 加计零偏 | `accel_bias` | 3 | m/s^2 | 可估计 |
| 双差模糊度 | `dd_ambiguity[PRN]` | M | cycle 或 m，必须选一 | TC 动态状态 |

**固定杆臂：** IMU 到 GNSS 天线的 FRD body-frame 杆臂作为已知配置，不进入 21 维状态。位置和速度观测需分别补偿天线位置和旋转速度；若将来在线估计杆臂，必须重新评估姿态激励和可观测性。

Euler 参数化接近俯仰角 ±90° 时存在奇异性。Euler 是首期实现模型，不是最终参数化结论；Quaternion/DCM 需要独立状态 manifolds 和约束实现后才能启用。

### 5.2 运动学预测

以 jerk \(\mathbf w_j\) 驱动平动状态：

\[
\begin{aligned}
\mathbf r_{k+1} &= \mathbf r_k+\Delta t\mathbf v_k+\tfrac12\Delta t^2\mathbf a_k+\tfrac16\Delta t^3\mathbf w_{j,k},\\
\mathbf v_{k+1} &= \mathbf v_k+\Delta t\mathbf a_k+\tfrac12\Delta t^2\mathbf w_{j,k},\\
\mathbf a_{k+1} &= \mathbf a_k+\Delta t\mathbf w_{j,k}.
\end{aligned}
\]

首期固定使用 NED 导航系、FRD 载体系和 ZYX（heading-pitch-roll）旋转序列，状态排列为 \(\Theta=[roll,pitch,heading]^T\)，方向余弦矩阵 \(C_b^n=R_z(heading)R_y(pitch)R_x(roll)\)。Euler 角速度映射为：

\[
\dot{\boldsymbol\Theta}=M(roll,pitch)\boldsymbol\omega_{nb}^b,
\quad \dot{\boldsymbol\omega}_{nb}^b=\mathbf w_\alpha.
\]

\[
M=\begin{bmatrix}
1 & \sin roll\tan pitch & \cos roll\tan pitch\\
0 & \cos roll & -\sin roll\\
0 & \sin roll/\cos pitch & \cos roll/\cos pitch
\end{bmatrix}.
\]

该映射与 \([roll,pitch,heading]\) 状态顺序及 ZYX 定义配套。技术报告提供的 Euler 转移映射未明确匹配同一顺序，故实现不直接复制；矩阵和其 Jacobian 已用中心有限差分验证。俯仰角接近 ±90° 时该参数化失效，当前模型拒绝奇异附近的传播。

离散状态转移的一般形式：

\[
\hat{\mathbf x}_{k+1|k}=f(\hat{\mathbf x}_{k|k},\Delta t),\quad
P_{k+1|k}=F_kP_{k|k}F_k^T+G_kQ_kG_k^T,
\]

其中 \(F_k=\partial f/\partial x\)。Euler 模型的 Jacobian 必须包括 \(M\) 对姿态角的偏导，而不只是将 \(\Delta tM\) 放在角度-角速度块；离散化精度（Euler、midpoint 或更高阶）由配置固定并经验证。偏置/比例因子按随机游走或一阶 Gauss-Markov 模型单独配置。无周跳且卫星连续时，模糊度通常建模为常值；失锁/周跳时按卫星状态管理器重置或移除。

### 5.3 IMU 观测模型

原始 gyro 与 accel 构成传感器观测：

\[
\mathbf z_g=h_g(\mathbf x)+\mathbf v_g,\qquad
\mathbf z_a=h_a(\mathbf x)+\mathbf v_a.
\]

基线简化模型可以写成：

\[
h_g=(I+S_g)\boldsymbol\omega_{nb}^{b}+\mathbf b_g,
\quad h_a=(I+S_a)C_n^b(\mathbf a^n-\mathbf g^n)+\mathbf b_a.
\]

Earth-aware NED 模型中，raw gyro 与加速度计预测为：

\[
\begin{aligned}
\hat z_g &= (I+S_g)\left(\omega_{nb}^b+C_n^b(\omega_{ie}^n+\omega_{en}^n)\right)+b_g,\\
\hat z_a &= (I+S_a)C_n^b\left(a^n-g^n+(2\omega_{ie}^n+\omega_{en}^n)\times v^n\right)+b_a,\\
\omega_{ie}^n&=[\Omega_e\cos\varphi,0,-\Omega_e\sin\varphi]^T,\\
\omega_{en}^n&=[v_E/(R_N+h),-v_N/(R_M+h),-v_E\tan\varphi/(R_N+h)]^T.
\end{aligned}
\]

首个简化模型仍保留用于隔离数学验证，要求输入 \(\omega_{nb}^b\)，并拒绝未知 gyro convention。新增的 Earth-aware 模型接受 raw \(\omega_{ib}^b\)，按配置的 geodetic origin 和当前 NED 状态计算地球速率、运输速率、正常重力与 Coriolis 项。此版本按零杆臂实现；IMU 杆臂造成的角加速度与向心项、多 IMU 安装变换和更高阶曲率运动学仍未实现。模型通过 `GmisLcConfiguration` 选择，简化和 Earth-aware 模型使用不同的 model ID，避免配置/代码误用。

每个观测模型返回预测值 \(h(x)\)、Jacobian \(H\)、协方差 \(R\) 和有效通道掩码。Euler 旋转矩阵偏导 \(D_\alpha,D_\beta,D_\gamma\) 可由解析式实现，并通过有限差分检查。

### 5.4 GNSS 松组合观测

GNSS receiver 提供天线相位中心位置和速度。已知 body 杆臂 \(l_b\) 时：

\[
\hat{\mathbf z}_{p}=\mathbf r^n+C_b^nl_b,
\quad
\hat{\mathbf z}_{v}=\mathbf v^n+C_b^n(\boldsymbol\omega_{nb}^b\times l_b).
\]

残差采用统一约定 \(\nu=z-h(x_{pred})\)。位置/速度协方差及其交叉项由 GNSS adapter 提供；如果输入是 RTK solution，必须记录 fixed/float 状态、卫星数、质量指标和协方差来源。接收机输出与 IMU 观测时间不同步时，先按事件时间完成预测，再执行 GNSS update。

### 5.5 GNSS 紧组合观测

基站位置已知时，流动站天线位置为 \(r_{ant}=r^n+C_b^nl_b\)。双差几何距离为：

\[
\nabla\Delta\rho_{jk}= (\rho_u^k-\rho_r^k)-(\rho_u^j-\rho_r^j).
\]

首期 TC 观测可配置为双差伪距及双差载波相位：

\[
\begin{aligned}
z_{P,jk}&=\nabla\Delta\rho_{jk}+\nabla\Delta I_{jk}+\nabla\Delta T_{jk}+v_P,\\
z_{\Phi,jk}&=\nabla\Delta\rho_{jk}-\nabla\Delta I_{jk}+\nabla\Delta T_{jk}+\lambda\nabla\Delta N_{jk}+v_\Phi.
\end{aligned}
\]

若采用其他观测组合（如 TDCP），应使用单独命名的 measurement model，并定义历元连续性、周跳、波长、卫星筛选和时间间隔语义。共用参考星的 DD 观测存在相关性：以差分矩阵 \(D\) 表示时，\(R_{DD}=D R_{SD}D^T\)，不可简单将每行设为独立噪声。离子层、对流层、卫星钟差和天线改正由输入数据是否已改正决定；数据源和改正状态必须成为元数据。

### 5.6 约束更新

通用线性化等式约束表示为 \(D x=d_c\)。正交投影可实现为：

\[
K_c=P D^T(DPD^T+R_c)^{-1},\quad
x^+=x^-+K_c(d_c-Dx^-),
\]

\[
P^+=(I-K_cD)P^-(I-K_cD)^T+K_cR_cK_c^T.
\]

严格等式约束使用 \(R_c\rightarrow0\)，实际代码用稳定线性求解并检查约束秩；软约束用有限 \(R_c\)。静止时的 \(v=0\)、角速度约束等是候选模型，不能仅因名字相同便假定“零角速度”一定成立（地球自转和坐标定义可能使本体相对导航角速度非零）。ZUPT、ZARU 及其他约束由 detector 触发，constraint model 只构造观测，不负责自行判断状态。

### 5.7 解算模式定义：GMIS_LC 与 GMIS_LTC

解算模式由配置选择观测模型集合；模式本身不改变 `StateEstimator` 的数值算法，也不能通过模式字符串隐式更改状态定义或噪声。

| 模式 | IMU 角色 | GNSS 观测 | 默认使用 GNSS 速度 | GNSS 输入 |
|---|---|---|---|---|
| `TMIS_LC` | 驱动 INS mechanization | RTK 位置/速度 | 是 | RTK solution |
| `TMIS_TC` | 驱动 INS mechanization | 卫星级 TD-DDCP | 不适用 | rover/base 原始 GNSS |
| `TMIS_LTC` | 驱动 INS mechanization | RTK 位置 + L1 TD-DDCP 区间位移 | 否，TDCP 已提供区间速度信息 | RTK solution + rover/base 原始相位 |
| `GMIS_LC` | gyro/accel 直接作为观测；运动学模型传播状态 | RTK 位置/速度 | 是 | RTK solution |
| `GMIS_LTC` | gyro/accel 直接作为观测；运动学模型传播状态 | RTK 位置 + L1 TD-DDCP 区间位移/速度 | 否，默认避免重复约束 | RTK solution + rover/base 原始相位 |
| `GMIS_TC`（可选扩展） | 同 GMIS 直接观测 | 原始双差伪距/载波等模型 | 不适用 | rover/base 原始 GNSS |

**GMIS_LTC 的 TDCP 观测定义：** 对 L1 信号、卫星 $s$ 和 pivot $p$，先在历元 $k$ 形成 rover/base 接收机单差和卫星间双差载波相位，再作历元间差分：

\[
D_k^{s,p}=\lambda_1\{(L_{u,k}^{s}-L_{b,k}^{s})-(L_{u,k}^{p}-L_{b,k}^{p})\},\quad
z_{\Delta,k}^{s,p}=D_k^{s,p}-D_{k-1}^{s,p}.
\]

载波相位 $L$ 的输入单位为 cycles，乘信号波长转成 meter。只有信号、卫星弧段与 pivot 连续且无周跳时，常值双差模糊度项才抵消。预测观测是同一卫星几何双差距离的历元差：

\[
h_{\Delta,k}^{s,p}=\nabla\Delta\rho_k^{s,p}(r_{ant,k})-
\nabla\Delta\rho_{k-1}^{s,p}(r_{ant,k-1}),\qquad
r_{ant,i}=r_i^n+C_{b,i}^{n}l_b.
\]

可以将 $z_\Delta/\Delta t$ 表示为区间观测的位移率，但不能把每颗卫星的 range-rate innovation 未经几何组合直接当成三维位置差。模型需处理卫星位置/钟差变化、base/rover 历元配对、pivot 连续性、周跳掩码、波长与观测相关协方差。

**跨历元状态是 GMIS_LTC 的关键。** 上式依赖 $x_{k-1}$ 和 $x_k$。首选方案是 `GmisLtcFilterManager` 保留有限 lag 的状态/协方差，并将活动 lag 表示为通用估计器中的增广状态块；TDCP model 输出关于两个历元状态块的联合 Jacobian，`StateEstimator` 对增广状态执行普通线性更新，之后按配置边缘化过期状态。这样估计器仍不依赖 GNSS 或 TDCP 领域语义。若首期不做 lag-state estimation，只能采用经过推导的区间位移/速度伪观测与协方差传播近似，并在配置/诊断中标注近似和前历元不确定度处理；禁止使用仅关于当前状态的 Jacobian 而忽略前历元状态误差及其相关性。

RTK 位置和 TDCP 来自同一 rover/base GNSS 数据，通常相关。优先构造含交叉项的联合 (R_{RTK,TDCP})。若交叉协方差不可得，允许经过离线验证的保守协方差膨胀策略，并把 `correlation_handling` 写入结果元数据；不得默认两者是完全独立的高权重信息源。周跳、失锁、pivot 改变或时间间隔异常时，拒绝对应 TDCP 区间、结束旧弧段并重新初始化；RTK 位置可以依据自身质量继续更新。

#### GMIS_LTC 参考实现审查与首期建议（2026-10-28）

参考工程 `nav-engine-cursor-3` 中的 `GmisTightlyCoupledNavigator.cpp` 是 GMIS-TC 路径：它在同一 GNSS 更新中处理双差伪距、双差载波相位和模糊度，并附带一组 TDCP 残差。它与本设计的 GMIS_LTC（RTK 位置 + L1 TD-DDCP 区间观测）不是同一个模式，不能直接复制其状态布局或把两种模式合并。

该参考实现有可借鉴的弧段管理：按卫星/频率缓存前历元双差相位与几何项；通过连续参考星构造历元间残差；pivot 改变时清空缓存并重新起弧。它的 TDCP 更新只用当前历元 Jacobian，没有保留前历元状态协方差及跨历元交叉协方差；另一个 `tcUsePositionDeltaVelocity` 选项在 GNSS 更新后直接赋值 `v=(p_k-p_{k-1})/dt`，不是 Kalman measurement update。两种简化都不作为 GMIS_LTC 首期的默认实现。

当前工程的 `TdDdCarrierPhaseBuilder` 已能输出时间配对的 L1/L2 TD-DDCP（米）、前后历元卫星及 pivot 的 ECEF 坐标、共享 pivot 协方差和周跳/间隔状态。建议 GMIS_LTC 复用该**与滤波器无关的观测预处理结果**，但使用 GMIS 专属的 `GmisLtcTdcpObservationModel` 和 `GmisLtcFilterManager`；不得调用 TMIS ESKF 或 `EstimationModule` 更新。

建议的无歧义模式定义：保持 GMIS LC 的 21 维活动状态和坐标/IMU/固定杆臂约定；GNSS 更新包括 RTK 天线位置及 L1 TD-DDCP，不默认使用 RTK 速度；通过时间差分消去连续双差模糊度，不新增模糊度状态。L1 TD-DDCP 的残差应写成

\[
\nu_k^{s,p}=z_{\Delta,k}^{s,p}-\{\Delta\rho_{DD}^{s,p}(r_{ant,k})-\Delta\rho_{DD}^{s,p}(r_{ant,k-1})\},
\]

并由当前/前历元两个状态块共同线性化。优先使用 42 维临时 lag 增广状态及完整联合协方差；完成该区间更新后边缘化前历元块，继续输出 21 维 GMIS 状态。首历元、周跳、卫星缺测、pivot 改变、超时或 GNSS 间隔越界时只重置/拒绝相应 TDCP 区间，不阻断有效的 RTK 位置更新。

| 设计项 | 首期建议 | 当前状态 |
|---|---|---|
| 组合定义 | RTK position + L1 TD-DDCP；RTK velocity 默认关闭 | 与项目既有 LTC 定义一致，待用户确认 |
| 状态 | 21 维活动 GMIS 状态；TDCP 更新时暂时 lag-augment 为 42 维；不估模糊度 | 推荐方案，需确认复杂度可接受 |
| TDCP | 复用 `PreparedTdDdCarrierPhaseEpoch`；在 GMIS 模型中使用两端几何与联合 Jacobian | 预处理具备基础字段，lag 协方差传播尚未实现 |
| 弧段处理 | 连续 pivot；pivot 改变/周跳/中断后丢弃跨界 pair 并重新起弧 | 参考实现与当前预处理都支持类似策略 |
| RTK–TDCP 相关性 | 配置显式记录策略；若拿不到交叉协方差，先输出诊断并使用可配置的保守膨胀系数 | 膨胀系数需由数据验证，不能照搬无依据常数 |
| NIS/outlier gate | 首轮保存完整先验创新/NIS与接受原因；门限可配置且关闭时明确标记 report-only | 当前 GMIS_LC 全部执行更新，NIS 尾部很大；需要单独确认自动拒绝策略 |

当前 GMIS_LC 的回放报告显示 RTK P/V NIS 中位数/P95 很大，且当前配置不做 NIS gating；因此 GMIS_LTC 应沿用同一 RTK 输入作 LC/LTC 对照，同时把 RTK 和 TDCP 的创新、门限决策、源质量与 correlation policy 写入日志。不能把 FIX 状态或形式协方差单独当作 RTK 可信度证明。

## 6. 软件总体架构

### 6.1 分层结构

```text
Application / Replay Runner
  ├─ Configuration & Dependency Builder
  ├─ Dataset / Sensor Adapters
  ├─ Time Synchronizer + Event Queue
  ├─ Preprocessing & Quality Control
  ├─ Dynamic Status / Constraint Detectors
  ├─ GmisFilterManagerModule (GMIS orchestration)
  │    ├─ GmisMotionModel
  │    ├─ ImuObservationModel
  │    ├─ GnssLcObservationModel / GmisLtcTdcpModel / GnssTcObservationModel
  │    ├─ ConstraintModel plugins
  │    └─ GmisEstimationModule -> generic StateEstimator (EKF core)
  ├─ Ambiguity / satellite state manager
  ├─ Diagnostics: innovation, residual, VCE, FDE
  └─ Solution, covariance, diagnostics writers
```

### 6.2 核心模块职责

| 模块 | 职责 | 不负责 |
|---|---|---|
| `ConfigManager` | 加载、合并、校验多个 JSON；解析相对路径 | 模型运行 |
| `DatasetAdapter` | 解析现有 IMU/GNSS 格式并保留原始元数据 | 滤波、静默修补异常观测 |
| `SensorSynchronizer` | 时间系统标准化、事件排序、延迟/乱序策略 | 改变观测值 |
| `Preprocessor` | 单位转换、有效性标记、周跳/质量检查 | 修改滤波器状态 |
| `DynamicStatusModule` | 输出 stationary/transition/moving 等概率或状态 | 直接改状态/协方差 |
| `ConstraintManager` | 根据状态与门限生成候选约束观测 | 跳过 StateEstimator 更新 |
| `StateLayout` | 为固定/动态状态块分配索引，保存单位与缩放 | 物理观测建模 |
| `GmisMotionModel` | 提供状态转移 \(f,F,Q\) | 读取 IMU 文件、更新滤波器 |
| `ObservationModel` plugins | 提供 \(h,H,R\)、通道元数据和测量残差 | 保存滤波器状态 |
| `StateEstimator` | 通用 EKF predict/update/constraint numerical core | 识别 IMU/GNSS 或 GMIS 状态语义 |
| `FilterManager` | 排序模型调用、组装观测批次、调用估计器 | 重新实现矩阵运算 |
| `AmbiguityStateManager` | 卫星弧段、参考星变化、模糊度增删/重置 | 整周固定（首期之外） |
| `DiagnosticsManager` | 创新、残差、协方差、冗余度、门限统计 | 在没有审计信息时自行剔除数据 |
| `SolutionWriter` | 输出解、RMS/covariance、状态模式及来源 | 参与估计 |

### 6.3 通用 StateEstimator 接口

估计器只依赖维数一致的模型输入：

```cpp
struct LinearizedMeasurement {
    Timestamp time;
    VectorXd residual;       // z - h(x_pred)
    MatrixXd H;
    MatrixXd R;
    std::vector<ChannelId> channels;
    MeasurementGroup group;
};

class IStateTransitionModel {
public:
    virtual PredictionResult predict(const VectorXd& x,
                                     const MatrixXd& P,
                                     double dt) const = 0; // f, F, Q
};

class IObservationModel {
public:
    virtual LinearizedMeasurement linearize(const StateView& x,
                                             const SensorEvent& event) const = 0;
};

class StateEstimator {
public:
    void predict(const IStateTransitionModel&, double dt);
    UpdateResult update(const LinearizedMeasurement&);
    ConstraintResult constrain(const LinearizedConstraint&);
    const VectorXd& state() const;
    const MatrixXd& covariance() const;
};
```

`StateEstimator` 负责数值稳定性：LDLT/Cholesky 求解、不显式求逆、Joseph covariance update、对称化/PSD 诊断、维度检查、NaN/Inf 防护、状态 manifold reset hook 和失败码。大误差角度不得直接作线性加法；姿态状态更新需由参数化提供 `boxplus/normalize` 策略。

### 6.4 模块间信息传递

模块经有类型的不可变事件交换数据，例如 `ImuSampleEvent`、`GnssRawEpochEvent`、`GnssSolutionEvent`、`ConstraintCandidateEvent`、`FilterUpdateEvent`、`NavigationSolutionEvent`。事件携带时间戳、坐标系、单位、来源、质量标识和原始序号。事件总线在首期可为单线程队列，但接口不应依赖共享可变全局状态。估计器仅由 FilterManager 所在线程写入；观测模型只读状态快照。

### 6.5 与 TMIS 隔离及 FilterManager / EstimationModule 复用策略

**复用通用估计器，隔离算法编排。** `StateEstimator` 可供 TMIS 与 GMIS 共用，因为它只执行矩阵层的状态预测、线性更新、约束投影和数值检查，不含 INS/GMIS 物理语义。两种架构的状态含义、传播模型和反馈方式不同，不能共用一个可变 filter session；GMIS 结果也不能走 TMIS 的 `ErrorStateFeedback` / `InertialNavigationModule` 反馈路径。

首期建议：

- 保留现有 `FilterManagerModule` 和 `EstimationModule` 为 TMIS ESKF 实现，以免改动已验证的 LC/TC/LTC 结果。
- 新建 `GmisFilterManagerModule`，负责 GMIS 事件调度、模式选择、RTK/TDCP 缓存、历元连续性和有限 lag 窗口，并发布标准测量事件。
- 新建 `GmisEstimationModule`，管理全值状态与 covariance，组合 GMIS motion/observation/constraint models，并调用共享的通用 `StateEstimator`。
- 共用 JSON schema 基础设施、数据 adapter、typed event、通用估计数值内核和诊断序列化；GMIS/TMIS 分开管理状态、模型注册表、配置 profile、输出 topic 和运行上下文。
- 如果后续确认现有 FilterManager 已完全领域无关，可把两者抽象到 `GenericFilterOrchestrator`；仍须创建互相隔离的算法实例。首期不为少量代码复用把两种估计架构塞进一个带大量 TMIS/GMIS 分支的模块。

GMIS 不创建或复用传统 `InertialNavigationModule` 来持有全值状态。GMIS 解由专用 writer 从 GMIS 状态和 covariance 发布；如需统一输出接口，可增加薄的 `GmisNavigationOutputModule`。

**C++ namespace 与类型隔离约定：**

- 通用数值/基础设施放在 `nav::core`（例如 `nav::core::StateEstimator`、`nav::core::StateLayout`）。
- 传感器公共输入类型放在 `nav::sensors`（例如 `ImuSample`、`GnssRawEpoch`、`RtkSolution`），其坐标、单位和质量语义由类型字段明确表达。
- 传统误差状态导航类放在 `nav::tmis`（例如 `nav::tmis::FilterManagerModule`、`nav::tmis::EstimationModule`、`nav::tmis::InertialNavigationModule`、`nav::tmis::ErrorStateFeedback`）。
- GMIS 全值状态、模型和输出放在 `nav::gmis`（例如 `nav::gmis::GmisFilterManagerModule`、`nav::gmis::GmisEstimationModule`、`nav::gmis::GmisState`、`nav::gmis::GmisSolution`）。
- 算法专用 feedback、snapshot、measurement bundle 不得放入 `nav::sensors` 或 `nav::core` 伪装成通用类型；GMIS 不接受 TMIS 的 `ErrorStateFeedback`，TMIS 也不接受 `GmisState`。
- 禁止在生产头文件和实现中使用 `using namespace nav::tmis;` 或 `using namespace nav::gmis;`；跨命名空间调用须写全限定名或显式局部 `using`，代码评审可直接识别算法边界。
- namespace 提供名称隔离；真正的错误调用防护还依赖专用类型、构造器依赖注入和构建依赖约束。不得让 GMIS target 依赖 TMIS implementation target；允许二者共同依赖 `nav_core` 与 `nav_sensors`。

建议 CMake targets 分为 `nav_core`、`nav_sensors`、`nav_tmis`、`nav_gmis` 和对应的独立 runner。配置工厂按 `algorithm` 创建对应的 manager/estimation 实例，并校验 plugin 注册表属于该 algorithm namespace。错误组合（如 `algorithm=gmis` 却请求 TMIS feedback/model）必须在启动时失败并指出配置项。

## 7. 数据处理与调度流程

1. 配置构建器装配 adapter、预处理器、状态布局、GMIS 模型和 `StateEstimator`。
2. Adapter 逐条产生原始传感器事件；时间同步器标准化时间并检测过期/乱序。
3. 预处理器校验单位、范围、缺失值、卫星连续性和观测质量，输出有效通道掩码及诊断事件。
4. 每个新事件按时间间隔触发 `GmisMotionModel` 的预测；不得将同一时间段重复预测。
5. `GmisFilterManagerModule` 根据事件类型和配置选择 IMU、GMIS-LC、GMIS-LTC、可选 GMIS-TC 或约束 observation plugin，线性化后交给 `GmisEstimationModule` 调用 `StateEstimator::update/constrain`。
6. 记录 prior innovation、post-fit residual、S、R、H 维数/有效通道、状态增量、协方差与更新结果。
7. 以配置的输出频率生成导航解；输出状态和协方差所处历元、坐标系与是否滤波更新。

IMU 是高频观测事件，但不是 `Predict` 的积分输入。IMU 在 GNSS 两历元之间持续提供 measurement update；无 IMU 到达时，运动学过程模型仍可按事件间隔传播。若 GNSS 事件到达早于待处理 IMU，排序/缓冲策略必须按配置执行并记录延迟。

## 8. 状态维度扩展与动态观测

`StateLayout` 使用命名状态块，不能在业务代码中散布硬编码起始索引。TC 模糊度由稳定的卫星/信号/参考星身份键映射至状态索引。添加或删除状态时，要同步扩展/裁剪状态和协方差，正确设置新状态先验方差及交叉协方差，并记录变更事件。

参考星切换不是简单改 PRN 索引：双差模糊度基准发生变化时，必须执行明确的状态变换或重新初始化相关模糊度，并更新协方差。周跳检测失败、失锁和重新捕获都必须产生可审计的模糊度弧段事件。

## 9. JSON 配置设计

采用入口配置引用多个文件；路径相对入口 JSON 所在目录解析。加载时执行 schema/version 校验，禁止未知关键参数静默忽略；默认值写入 effective-config 输出以保证可复现。

建议文件：

```text
config/gmis_run.json
config/system.json
config/data.json
config/frames.json
config/state_layout.json
config/motion_model.json
config/imu_model.json
config/gnss_lc_model.json
config/gnss_tc_model.json
config/constraints.json
config/diagnostics.json
config/output.json
```

示例入口：

```json
{
  "schema_version": 1,
  "estimator": { "type": "ekf", "implementation": "generic_state_estimator" },
  "model_set": "gmis_gps_imu",
  "mode": "tc",
  "config_files": {
    "system": "system.json",
    "data": "data.json",
    "frames": "frames.json",
    "state_layout": "state_layout.json",
    "motion": "motion_model.json",
    "imu": "imu_model.json",
    "gnss_lc": "gnss_lc_model.json",
    "gnss_tc": "gnss_tc_model.json",
    "constraints": "constraints.json",
    "diagnostics": "diagnostics.json",
    "output": "output.json"
  }
}
```

每个模型单独配置 `enabled`、噪声、状态映射、通道筛选、更新频率、协方差来源、门限和缺失策略。禁止由一个 `mode` 字符串隐式推导所有模型参数。LC/TC/LTC 开关是模型编排选择；状态参数化、IMU 误差块和约束模块分别设置。模式使用算法命名空间：`tmis.lc/tc/ltc` 与 `gmis.lc/ltc`；可选 `gmis.tc` 单独注册，禁止只用无算法类型的 `lc` / `ltc` key。

GMIS 模式配置示例：

```json
{
  "algorithm": "gmis",
  "combination_mode": "gmis_ltc",
  "modules": {
    "filter_manager": "gmis_filter_manager_v1",
    "estimation": "gmis_estimation_module_v1",
    "state_estimator": "generic_ekf_v1"
  },
  "measurement_models": [
    "imu_raw_gyro_accel_v1",
    "rtk_position_loose_v1",
    "l1_td_ddcp_interval_displacement_v1"
  ],
  "gnss": {
    "use_rtk_velocity": false,
    "tdcp_signal": "L1",
    "tdcp_units": "meter",
    "pivot_policy": "continuous_or_reinitialize_arc",
    "cross_covariance_policy": "conservative_inflation"
  }
}
```

`gmis_lc` 默认配置 IMU raw gyro/accel、RTK position 和 RTK velocity models；`gmis_ltc` 默认配置 IMU raw gyro/accel、RTK position 与 L1 TD-DDCP interval displacement。是否附加 RTK velocity 必须由单独字段开启，并提供相关性处理。程序启动时验证模式、module type、measurement plugin、状态块和输入 topic 之间的契约。

## 10. 输出与性能分析

### 10.1 导航解

每行包含：时间、位置/速度/姿态全值、对应 covariance 对角线与相关摘要、传感器可用性、当前模式、GNSS 类型/质量、卫星数、模糊度数、静止状态、约束状态及质量标志。Covariance-derived sigma 与基于参考真值计算的误差/RMS 分开命名。

### 10.2 Filter 诊断

每次更新保存：观测组和通道、预测状态时间、innovation \(\nu=z-h(x^- )\)、innovation covariance \(S=HP^-H^T+R\)、NIS、post-fit residual、residual covariance、观测组冗余度、门限及接受/拒绝原因。若使用 VCE，记录窗口长度、有效样本数、估计因子、上下限、更新前后 R，且不能仅因窗口中的偶然残差自动无限增大/减小权重。

FDE 首期先实现只读 w-test/NIS 报告和显式配置的门限标志。自动通道隔离需在单故障假设、显著性水平、多重检验、相关 DD 测量处理和恢复逻辑明确之后实现。

### 10.3 性能评估

- 与参考解对齐时间后，分别计算 ENU/ECEF 位置误差、速度误差、roll/pitch/heading 误差及 3D RMS。
- 报告整体、运动状态分段、GNSS fixed/float/失锁分段和约束触发分段结果。
- 输出 RMS、均值、标准差、百分位数、最大误差、样本数及有效率；角度先 wrap 到 [-pi, pi)。
- 创新分析按传感器/轴向/卫星统计均值、标准差、NIS 和白噪声特征；先验创新与后验残差不可混淆。
- 传统 ESKF 与 GMIS 使用相同输入、坐标基准、参考值和有效时间范围，分开输出配置快照和版本号。

## 11. 建议代码目录

```text
nav/
  CMakeLists.txt                 # nav_core, nav_sensors, nav_tmis, nav_gmis targets
  include/nav/core/              # namespace nav::core
  include/nav/sensors/           # namespace nav::sensors
  include/nav/tmis/              # namespace nav::tmis (existing ESKF path)
  include/nav/gmis/              # namespace nav::gmis (full-state path)
  src/core/...
  src/sensors/...
  src/tmis/...
  src/gmis/...
  config/tmis/...
  config/gmis/...
  tests/{core,tmis,gmis,replay}/...
  docs/GMIS_GPS_IMU_Software_Design.md
```

传统 INS/RTK 模块继续位于现有目录，通过独立 runner 调用；公共数据读取和输出仅在单位/坐标/时间契约相同的情况下复用。

## 12. 数值实现与异常策略

- 禁止计算矩阵显式逆；使用 LDLT/LLT/SVD，并检查分解状态和条件数。
- EKF covariance 使用 Joseph 形式；每次更新做对称化并监控最小特征值/Cholesky 是否成功。
- 所有动态矩阵操作前验证维数、有限数值、正定/半正定条件和状态布局版本。
- 对 GNSS 异常、周跳、模式转换、约束解除和状态重置使用明确结果码，不以零向量或 NaN 表示缺失。
- 预测步长非正、过大或跨数据中断时执行配置的 fail/hold/reinitialize 策略，并写入状态事件日志。
- 对姿态角使用周期归一化；使用 manifolds 时由姿态状态策略执行 boxplus、boxminus 和 covariance reset。
- 在线自适应噪声必须有上下限、变化率限制、恢复策略和可关闭开关。

## 13. 公式与代码一致性审查清单

输入资料提供了详尽公式和代码蓝图，但若干部分在投入解算前必须统一并数值核验：

1. **Euler 传播 Jacobian。** 指南给出的姿态 transition 展示了角速度到角度的 \(M(\beta,\gamma)\) 块，但 EKF 的 \(F=\partial f/\partial x\) 还需包含 \(M\) 对 \(\beta,\gamma\) 的导数项；代码草图也出现了转换矩阵分量与文档矩阵不完全相同的风险。应以一个明确的坐标/欧拉约定推导并有限差分验证。
2. **IMU 物理观测。** 报告的完整物理模型含地球转动、运输角速率、科氏项、IMU 杆臂等，架构指南中的简化实现未全部涵盖。必须决定本数据工作区域需要哪些项，并让 `h(x)`、`H`、仿真数据和配置保持一致。
3. **观测时间与调用接口。** 报告支持 100 Hz IMU 与 1–10 Hz GPS，但同步调用式草图容易在 GPS 历元才更新 IMU，或重复消费同一 IMU。正式 API 必须按异步事件独立更新。
4. **TC 观测维数。** 资料中的矩阵有时将伪距和相位写作 \(2M\)，接口草图中又包含一个参考星加 M 个非参考星的观测字段；实现须明确 M 的定义（非参考星数）、每个频点/观测类型的通道数，以及参考星共同误差产生的完整协方差。
5. **载波/模糊度量纲和改正。** 模糊度可用 cycles 或 meter，但只能选一种状态定义；波长必须按星座与信号选择。电离层、对流层、卫星钟差、相位缠绕、天线相位中心等改正的输入状态需写明。
6. **状态转移噪声。** 报告给出 18 维噪声（jerk 3、角加速度 3、IMU 随机游走 12）；需核对每个噪声块到状态的 \(G\)、单位与离散谱密度定义。随机游走噪声的 \(\Delta t\) 缩放应与配置采用的连续/离散 PSD 定义一致。
7. **约束和协方差。** 静止时约束的物理含义取决于导航系及地球自转补偿；有限精度伪观测与严格等式投影应区分。受约束后 residual covariance、冗余度及 VCE 公式要使用同一 covariance 定义。
8. **VCE/FDE。** 要明确采用先验 innovation 还是后验残差、相关观测的分组方式、冗余度计算、自由度、滑窗有效样本和显著性门限。未经合成数据和回放验证不得自动剔除观测。
9. **Euler/Quaternion/DCM 状态布局。** 报告列出多种姿态参数化选项，但这些不是只切换一个枚举值即可复用的 21 维状态。首期只启用 Euler；其他参数化须提供完整 manifold、约束、Jacobian 和互相转换方案。

这份清单是设计阶段风险登记，不表示报告算法已被否定；目标是在实现前把坐标、物理模型和统计定义落实为单一、可测试规范。

## 14. 开发阶段与验收门槛

### 阶段 A：模型契约与纯数学验证

- 确定坐标、Euler 顺序、旋转方向、IMU 输出物理量、GNSS 改正项和噪声 PSD 单位。
- 对运动状态 transition、旋转矩阵及各观测 Jacobian 做解析/有限差分一致性检查。
- 在合成直线匀速、恒加速度、恒角速率、静止与杆臂运动数据上检查预测和更新。

**首期实现状态（2026-10-27）：** 已建立独立 `nav_gmis` library 与 `nav::gmis` 代码边界，固定 NED/FRD、ZYX、状态顺序 `[roll,pitch,heading]`；GMIS LC 状态调整为 21 维，移除 gyro/accelerometer scale-factor 状态，杆臂采用固定配置。generic `nav::core::StateEstimator` 提供 `f(x), F, Q` 非线性预测入口，原 TMIS 线性接口保持可用。21 维运动学 transition、Euler 旋转矩阵及其偏导、简化和 Earth-aware IMU observation、含非零固定杆臂的 RTK P/V observation 已完成合成有限差分核验。`GmisLcConfiguration` 能读取 `config/gmis/gmis_lc.json` 并装配独立 estimator session；独立 `gmis_lc_app` 输出 1 Hz 状态、协方差和创新统计，不接入 TMIS module registry。真实数据对比中，21 维版本相对 RTK 的位置后验残差中位数/P95 从 1.704/19.634 m 降至 1.395/13.563 m，但 formal position sigma 中位数从 0.438 m 增至 0.488 m，NIS 仍显著偏大；heading 与 27 维基线差异很大，当前不能判定总体导航性能提升。此回放使用此前配置的 `(0,0,0) m` 杆臂，而此前杆臂估计已关闭，因此该值没有独立标定。姿态仍由配置初值给定，RTK 速度弱分量会降权，尚无真值评估。Earth-aware 模型含地球自转、运输速率、正常重力和 Coriolis，但采用局部平面 NED 近似。

### 阶段 B：通用估计器与状态布局

- 实现 generic `StateEstimator`、可变状态 layout、Joseph update、约束投影和错误码。
- 用线性系统解析解、Monte Carlo 和 covariance consistency 检查数值正确性。

### 阶段 C：单 IMU + GMIS_LC 回放

- 接入现有 IMU 数据和 GNSS 解算位置速度。
- 检查异步更新、时间间隔、创新统计、可观测性和状态 covariance。
- 保存 effective JSON、代码版本和可复现输出。

### 阶段 D：GMIS_LTC 跨历元观测与 GNSS 弧段生命周期

- 先完成 TD-DDCP 区间观测、双差协方差、历元配对、连续性和周跳/参考星处理，再开放 GMIS_LTC。
- 用可控观测仿真验证两历元几何模型、联合 Jacobian、RTK position 与 TDCP 相关性，以及前历元状态不确定度传递。
- 可选 GMIS_TC 的原始伪距/相位及动态模糊度模型单独验收；整周固定仍是后续能力，不与 GMIS_LTC 浮点阶段混为一谈。

### 阶段 E：VCE/FDE、约束和性能分析

- 先离线输出诊断统计，再启用在线调权；每一阶段保留关闭开关和基线结果。
- 与传统 INS/RTK 及无新 feature GMIS 基线对比，按统一真值与时间区间出图、表格和报告。

## 15. 当前需要在实现前冻结的选择

依据本轮文档，已有一个合理的软件设计起点；以下算法约定应在编码相应模型前逐项确认/固化：

1. Local navigation frame 首期使用 ENU 还是 NED，以及 Earth-aware IMU 模型的精度范围。
2. 首期姿态只采用 Euler，并确定 Euler 序列、heading 定义和接近奇异点处理。
3. 陀螺输出是 \(\omega_{ib}^b\) 还是已补偿为 \(\omega_{nb}^b\)；加速度是否含重力响应比力。
4. 双差载波模糊度状态使用 cycles 还是 meters；每个卫星信号的波长/频率来源和参考星切换变换策略。
5. TC 使用伪距+相位、仅相位，或保留单独 TDCP 插件；各观测的改正项及协方差模型。
6. 动态状态检测和静止约束首期启用哪些约束，以及地球自转补偿后的零角速度定义。
7. GNSS LC covariance 的获取方式与 GNSS 更新时间插值/外推策略。
8. GMIS_LTC 首期采用 lag-state update，还是采用哪一种经推导的区间观测近似和协方差传播。
9. RTK position 与 TDCP 共享 GNSS 原始观测时，能否取得交叉协方差；否则采用何种经独立数据标定的保守膨胀系数。

这些选项不妨碍先完成通用状态估计器、配置、事件流和模型接口框架；在对应模型启用前，未冻结选项应当显式配置为 unsupported，而不是使用隐式默认。建议先搭建 `GMIS_LC`，同时设计好跨历元观测和相关性接口，再启用 `GMIS_LTC`。通过 `algorithm="gmis"` 与 `combination_mode="gmis_lc|gmis_ltc"` 切换；TMIS 使用独立 algorithm namespace 和 filter session。

## 16. 术语

| 缩写 | 含义 |
|---|---|
| GMIS | Generic Multi-sensor Integration Strategy |
| TMIS | Traditional Multi-sensor Integration Strategy |
| LC / TC | Loosely Coupled / Tightly Coupled |
| EKF | Extended Kalman Filter |
| VCE | Variance Component Estimation |
| FDE | Fault Detection and Exclusion |
| LOS | Line of Sight |
| DD | Double Difference |
| PSD | Power Spectral Density |

## 17. GMIS_LTC首版实现状态（2026-10-28）

依据本轮确认，GMIS_LTC 已采用 42 维跨历元联合状态 `[x(k−1), x(k)]`。IMU 预测及IMU观测仅作用于当前21维块；RTK天线位置作用于当前状态，L1 TD-DDCP 观测的Jacobian同时作用于前、当前状态，并保留二者的交叉协方差。每个GNSS历元完成联合更新后，将当前后验状态和协方差克隆为下一时间区间的两端初值。克隆协方差加入可配置的微小数值正则项以保持 generic `StateEstimator` 所需的正定性。

首期使用已有周跳预处理和 `TdDdCarrierPhaseBuilder`，只选取干净的L1 TD-DDCP行；L2暂不参与GMIS_LTC更新。RTK position与TD-DDCP在同一联合更新中按块对角R组合，暂按零交叉协方差处理，并通过 `tdcp.variance_inflation` 显式膨胀TDCP噪声。当前JSON值4.0仅是可调、未标定的起始值；NIS完整记录但不触发自动拒绝。RTK速度测量明确关闭，避免将LTC定义与LC混淆；没有模糊度状态，因为时间差分载波观测已消去连续弧段中的常数整周模糊度。

实现结构增加纯数据/预处理公共库 `nav_sensors`，供传统 `navcore` 和GMIS共用；该库不链接TMIS INS mechanization或RTKLIB。GMIS专属估计会话和runner分别位于 `nav::gmis` 与 `gmis_ltc_app`，由 `config/gmis/gmis_ltc.json` 创建，不通过TMIS FilterManager调用。

当前基线回放使用1564个1 Hz历元（GPS秒63390–64953），输入为恢复的RTK输出、IMU及base/rover GNSS观测。输出包括导航解、形式协方差RMS和逐行创新/NIS文件，Python分析页面使用既有回归分析工具生成。该次报告中的位置3D形式1σ RMS中位数约0.134 m、P95约0.165 m；这只是滤波器协方差自报，不是对独立真值的误差RMS，也不能据此认定实际达到分米或厘米精度。联合GNSS NIS中位数约2.75、P95约394，长尾仍需用统一真值和按观测自由度分组的统计进一步诊断。协方差膨胀系数4.0未经独立标定，因此该回放仅作为实现基线，不作为性能结论。

## 18. GMIS_TC设计（已确认：float首期）

GMIS_TC 使用原始GNSS观测做卫星级紧组合，不输入RTK位置/速度，也不使用GMIS_LTC的TD-DDCP。三种模式边界为：GMIS_LC使用RTK P/V，GMIS_LTC使用RTK位置+L1 TD-DDCP，GMIS_TC直接使用rover/base原始观测的双差伪距与载波相位。

### 18.1 观测模型

首期建议使用双频 `C1`、`P2` 伪距和 `L1`、`L2` 载波相位。载波相位先按cycles解码，乘各信号波长转成meters。`D1` 首期不生成速度伪观测，后续如需要单独添加Doppler模型。

对共同卫星，以base天线相位中心ECEF坐标和base记录的卫星ECEF坐标构造接收机单差、卫星双差。对卫星s相对pivot p，几何双差为：

\[
\rho_{DD}^{s,p}(r)=\bigl(\|r-r_s\|-\|r_b-r_s\|\bigr)-
                    \bigl(\|r-r_p\|-\|r_b-r_p\|\bigr),
\]

推荐双频观测模型：

\[
\begin{aligned}
P_{DD,f}^{s,p} &= \rho_{DD}^{s,p} + \alpha_f I_{DD}^{s,p} + T_{DD}^{s,p} + v_{P,f},\\
\Phi_{DD,f}^{s,p} &= \rho_{DD}^{s,p} - \alpha_f I_{DD}^{s,p} + T_{DD}^{s,p}
                       + \lambda_f N_{DD,f}^{s,p} + v_{\Phi,f}.
\end{aligned}
\]

`P_DD,1` 使用C1、`P_DD,2` 使用P2；相位分别使用L1/L2。`I_DD` 表示L1等效电离层双差，`α1=1`、`α2=(f1/f2)^2`；`N_DD,f` 是以cycles表示的双差模糊度。双差消去接收机钟差和卫星钟差，但电离层、对流层残差不会自动为零。首期显式估计每颗非pivot卫星的电离层双差状态；短基线对流层残差按配置噪声建模。导航位置Jacobian由双差几何距离对天线位置求导并转换到GMIS local NED；相位行还包含对应波长的模糊度偏导和电离层偏导。

### 18.2 状态与滤波器

保留GMIS 21维导航全状态 `[r,v,a,roll/pitch/heading,angular_rate,gyro_bias,accel_bias]`，并按连续卫星弧段动态增加每颗非pivot卫星的一个电离层状态（m）及L1/L2双差模糊度（cycles）。当前历元状态维数为 `21 + 3M`，M为共同非pivot卫星数。模糊度在连续无周跳弧段内近似常值；电离层差分按JSON随机游走模型传播。

继续由通用 `nav::core::StateEstimator` 执行直接全状态EKF：GMIS-TC调用方提供完整 `f(x), F, Q, z, H, R`。它不使用TMIS误差状态反馈，也不把GNSS位置直接赋值给状态。现有21维 `GmisLcModels::predict` 不能直接预测动态维数状态，因此GMIS-TC需有独立的 `GmisTcStateLayout`/转移适配器：对导航块使用GMIS运动模型，对ambiguity/ionosphere块使用各自动态模型，再将矩阵交给generic estimator。

### 18.3 状态生命周期与协方差

`GmisTcAmbiguityStateManager` 按(星座、SVID、频点、arc ID)管理ambiguity，接收周跳预处理状态并输出增删/重置事件。连续弧段优先保持pivot；pivot改变时，对仍共同的卫星执行明确的模糊度/电离层线性状态变换及 `P'=TPTᵀ`。无法可靠映射、卫星失锁或发生周跳时，只重置受影响信号弧段。共享pivot产生双差行间相关性，必须用 `R_DD=D R_SD Dᵀ` 构造完整观测协方差，不能把行默认视作独立。RTK P/V不进入TC更新，避免对同一原始GNSS信息重复计权。

### 18.4 模块及配置

在 `nav::gmis::tc` 中创建 `GmisTcFilterManager`（历元排序/状态布局）、`GmisTcObservationBuilder`（双差码/相位与H/R）、`GmisTcAmbiguityStateManager`（弧段/周跳/pivot/动态状态）和 `GmisTcEstimationModule`（向generic StateEstimator提交联合更新）。数据读取和周跳预处理可复用纯传感器库 `nav_sensors`；TC观测构造、状态和滤波会话不调用TMIS模块，也不复用GMIS_LTC lag更新。以 `algorithm="gmis"`、`combination_mode="gmis_tc"` 和独立 `config/gmis/gmis_tc.json` 装配。观测模型ID首期为IMU raw、DD C1、DD P2、float DD L1、float DD L2；路径、已知base ECEF、pivot、噪声、周跳、门限、状态噪声、AR开关和输出都由JSON配置。

### 18.5 浮点与整周固定分期

建议首期只实现float ambiguities，输出每频点模糊度均值/协方差、code/phase innovations、分组NIS、卫星数和弧段事件。浮点模型通过有限差分Jacobian核验及回放后，再单独增加联合整数搜索模块，包含完整模糊度协方差、ratio test、固定解残差验证和失败回退。不得把独立最近整数取整或滤波器形式RMS低于15 cm称为已fix或真实位置精度。

### 18.6 参考工程的采用范围

`nav-engine-cursor-3` 的 `GmisTightlyCoupledNavigator.cpp`、`GnssCodeDdModel` 和 `GnssPhaseDdModel` 提供双差几何、C1/P2代码组合、L1/L2相位及动态ambiguity状态的参考。其TDCP附加更新属于另一类观测，首期GMIS_TC不混用GMIS_LTC；`tcUsePositionDeltaVelocity` 直接覆盖速度，不采用；单模糊度最近整数简化fix和失败后直接施加位置增量/清空ambiguity的fallback也不作为TC默认算法。

### 18.7 精度评估边界

以相同IMU、rover/base原始GNSS、初值和时间区间分别运行GMIS_LC、GMIS_LTC、GMIS_TC，分开展示float/fixed状态、各观测组innovation/NIS、形式协方差、周跳/pivot事件和solution separation。当前数据只有起始rover ECEF坐标而没有完整独立轨迹真值；因此LC/LTC/TC模式差或形式RMS都不能证明实际误差达到15 cm。独立真值或经确认的高精度参考轨迹是做实际误差评价的前提。

**已确认的首期范围：** 双频C1/P2与L1/L2、显式双差电离层状态、float ambiguity；整数固定作为后续独立阶段。NIS首期记录但不自动拒绝，观测噪声与状态噪声由JSON配置。

### 18.8 首期回放实现检查（2026-10-28）

已加入独立 `gmis_tc_app` 回放入口、`nav::gmis::tc` 估计会话和 `config/gmis/gmis_tc.json`。GMIS_TC读取IMU及rover/base原始GNSS历元，不读取RTK solution；以动态 `21+3M` 全状态执行直接EKF，包含C1/P2双差码、L1/L2双差载波float模糊度和L1参考电离层状态。该实现仍属首期工程基线，尚未达到可宣称性能已验证的状态。

当前构建成功，使用同一数据完成1564个GNSS历元和158328个IMU样本回放；1564个GNSS批次均送入滤波器。GMIS_TC与现有GMIS_LTC在1564个共同历元上的3D位置解差中位数约0.385 m、P95约0.729 m、最大约1.407 m；与GMIS_LC的相应差值约1.422 m、13.44 m、26.30 m。这些是不同解算模式之间的solution separation，不是相对真值误差。轨迹覆盖数百米的运动范围，LC/LTC也呈现相近的主轨迹形态，因此不能只凭坐标跨度判定发散。

GMIS_TC位置协方差形式3D RMS中位数约0.149 m、P95约0.658 m，GNSS NIS中位数约2.34、P95约452，最大约9547；码创新绝对值P95约0.90 m，相位创新绝对值P95约0.33 m。形式RMS不是实测误差，NIS有明显长尾。由于没有独立完整轨迹真值，当前结果只能用于诊断和模式间差异检查，不能据此证明15 cm实际精度，也不能依据小协方差单独判定结果可信。

回归分析工具已支持读取GMIS_TC输出并生成含轨迹、速度、姿态、形式RMS与GNSS NIS的交互页面。后续性能结论前，应先审计原始双差观测/几何、pivot与动态状态变换、观测协方差，以及异常NIS历元；并与RTK/参考程序做同历元误差图。整数模糊度固定仍不在首期范围内。
