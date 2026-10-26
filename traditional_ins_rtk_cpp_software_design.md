# 传统 GNSS/IMU RTK 松紧组合导航软件设计说明（C++）

> 文档状态：设计与当前实现对照版（LC/TC/LTC 可切换；DynamicStatus、ZUPT、NHC、ZPC、ZHC 均有可配置的实验实现；heading-lock 已按不可变快照实现并完成三模式全量回放。实验参数仍需独立真值验证）
> 版本：2.3
> 日期：2026-10-25
> 输入材料：`Traditional INS_RTK Tightly-Coupled Design Report.pdf`（v1.0，6 页）、`traditional_ins_rtk_tightly_coupled_software_design_guide.md` 及 `Theoretical Foundations of ZUPTand NHC.pdf`（3 页）

## 1. 目的与范围

本文定义一套可落地为 C++ 工程的传统捷联 INS 与 GNSS RTK 软件，并严格区分三种组合模式：**LC（loosely coupled）**使用 RTK 输出的位置/速度作为 INS 滤波器量测；**TC（tightly coupled）**直接将卫星级 TD-DDCP 几何残差用于滤波更新；**LTC（loosely tightly coupled）**使用 RTK 位置做 LC 位置更新，并将 L1 相邻历元差分载波相位解算为天线位移 `Δposition`，再作为区间速度量测更新 INS 速度。三种模式由 JSON 配置切换，可分别启用量测模型和状态模型；LTC 的 RTK 速度可作为独立开关，但默认关闭，避免与相位推导速度重复提供速度约束。

本文所称“时间差分载波相位”按双站、双差、历元间差分定义：在每个历元先形成 rover/base 接收机单差，再对卫星与 pivot 卫星作双差，最后对相邻历元的双差载波相位作时间差分（TD-DDCP）。相位以 cycles 输入，乘波长转为米。在无周跳且卫星/pivot 连续时，整周模糊度在时间差分中抵消，因此融合 ESKF 不维护载波模糊度状态。首期不把差分伪距作为 INS 紧组合量测；伪距可由 RTK 引擎内部使用以生成 RTK 解。

本文是软件设计规格，不把输入材料中的示例代码视为正确实现。报告与指南中部分符号约定尚有冲突，且报告第 6 页的“基准星切换”和“时间同步”内容被截断；相关事项列为实现前必须确认的决策，详见第 12 节。

**首期范围**：NED 导航系、FRD 载体系、WGS-84 ECEF 几何；可配置 15/18 维 INS 误差状态；默认保持既有 15 维有效滤波行为，18 维杆臂状态通过显式 JSON 选项开启，经过雅可比、可观测性和真值验证后再用于生产回放；IMU 机械编排和协方差传播；可切换的 LC、TC、LTC 三种模式；LC 的 RTK 位置/速度更新；TC 的 TD-DDCP 卫星级几何更新；LTC 的 RTK 位置更新与 L1 TDCP `Δposition`/速度更新；各模式的量测相关性建模或经验证的去相关处理；历元连续性与周跳检测；动态状态检测（静止、匀速、加速/减速、转弯及未知）；由静止状态驱动的 ZUPT、Zero_Position_Change 和 Zero_Heading_Change，以及由运动状态驱动的 NHC 可选约束更新；进程内模块注册与强类型消息传递；多文件 JSON 配置；对 `data1` 的离线回放适配；时间戳检查、日志和结果输出。RTK 引擎内部的整数模糊度固定由 RTKLIB 等 RTK solution engine 负责，不属于融合 ESKF 状态。

**首期不默认开启**：IMU 到天线杆臂在线估计、比例因子状态、双频电离层/对流层残差参数化、完整 GNSS 接收机跟踪环、地图匹配。开启杆臂估计时初值为 `[0,0,0] m`，其先验协方差、随机游走噪声、姿态激励门限和估计开关由 JSON 配置。

**模式可配置性**：`combination_mode` 取 `lc`、`tc` 或 `ltc`。LC 默认使用 RTK P/V；TC 默认使用 L1/L2 TD-DDCP 几何观测；LTC 默认使用 RTK 位置和 L1 TDCP 推导的区间位移/速度观测，RTK 速度通过单独选项启用。每种模式均可独立打开/关闭观测模型并选择状态布局（例如基础 15 维 INS、加杆臂后的 18 维 INS）。不是所有观测与状态模型都可任意搭配：每个观测模型声明所需状态块、输入、维度和协方差契约；启动阶段必须校验满足要求，未注册或不兼容的组合明确报错，不静默忽略。

## 2. 术语和坐标约定

| 项目 | 约定 |
|---|---|
| 导航系 n | NED：北、东、地；向下为正 |
| 载体系 b | FRD：前、右、下；向下为正 |
| 地固系 e | WGS-84 ECEF，米 |
| 经纬高 | `[纬度(rad), 经度(rad), 椭球高(m)]` |
| 速度 | NED，m/s |
| 姿态 | `q_bn` 将 b 系向量旋转至 n 系；Eigen 四元数标量在前的构造和内存/序列化规则必须明确 |
| IMU 输入 | 优先使用增量 `dtheta(rad)`、`dv(m/s)`；单位、轴序、标定及时间标签由适配层保证 |
| RTK 位置/速度量测 | RTK 引擎输出；内部统一转换为 ECEF 位置（m）和 ECEF 速度（m/s），并携带状态质量及协方差 |
| TD-DDCP 量测 | 每历元 rover/base 单差及卫星/pivot 双差后，再对相邻历元差分；输入 cycles，进入观测模型前乘波长转为米 |
| 伪距 | RTK 引擎内部可用于其 RTK 解算；首期不作为独立的 INS 紧组合量测 |
| 模糊度 | RTK solution engine 内部按其模型管理；无周跳 TD-DDCP 在融合滤波器中不显式维护模糊度状态 |
| 位置误差 | 滤波误差在局部 NED 米制，不直接以经纬度弧度作为状态 |

所有公开接口须写明坐标系、单位、时间标签和误差符号。禁止仅以 `Vector3d` 表达未注明语义的量。

## 3. 分层系统架构与 module 通信

系统采用“分层职责 + 可插拔 module”的结构。数据按流水线流动；相邻 module 通过进程内强类型消息总线发布/订阅，不直接依赖其他 module 的具体类。`FilterManagerModule` 负责按事件时间整理输入并构造标准化量测包；`EstimationModule` 选择导航状态/过程/观测模型，将模型专属量测线性化后调用底层通用 `StateEstimator`；`StateEstimator` 只实现与领域无关的状态/协方差预测和数值滤波更新，不包含 INS、RTK、载波相位或车辆模型。每个导航算法实例有独立的滤波会话和状态上下文，不通过消息总线拆开发送内部可变状态，避免破坏滤波状态一致性。

```text
┌────────────────────────────────────────────────────────────────────┐
│ L0 应用/运行时层                                                    │
│ App + ConfigLoader + ModuleRegistry + Scheduler + Typed MessageBus │
└────────────────────────────────────────────────────────────────────┘
                       │配置、创建、连接、调度
┌────────────────────────────────────────────────────────────────────┐
│ L1 数据源层                                                         │
│ IMU 文件/设备 Source        GNSS Rover/Base 文件/设备 Source        │
└────────────────────────────────────────────────────────────────────┘
                       │ RawImuRecord / RawGnssRecord
┌────────────────────────────────────────────────────────────────────┐
│ L2 Decode 与标准化层                                                │
│ ImuDecoder / GnssDecoder / Unit & Frame Mapper / Timestamp Mapper  │
│ -> ImuSample (FRD, rad/s, m/s²) / GnssObservation (typed fields)    │
└────────────────────────────────────────────────────────────────────┘
                       │标准化传感器消息
┌────────────────────────────────────────────────────────────────────┐
│ L3 传感器处理与数据准备层                                           │
│ ImuProcessor       GnssPreprocessor       FilterManagerModule        │
│ DynamicStatusModule (滑窗状态识别)         ConstraintManager         │
│                    (质量/共视星)          (约束候选与门控)          │
└────────────────────────────────────────────────────────────────────┘
                       │ImuSample / PreparedGnssEpoch
┌────────────────────────────────────────────────────────────────────┐
│ L4 GNSS 解算与导航融合层                                             │
│ RTK Engine -> RtkSolution(position, velocity, covariance)             │
│               + PreparedTdDdCarrierPhaseEpoch                        │
│ 可配置 ESKF: LC / TC / LTC 三模式切换                                │
│ InertialNavigationModule: nominal state + INS mechanization           │
│ EstimationModule: model orchestration + INS ESKF session             │
│   └─ StateEstimator: generic state/covariance prediction and update  │
│ ErrorStateFeedback -> corrected nominal state -> NavigationSolution │
│ DynamicStatusModule -> ConstraintManager -> constraint measurements │
└────────────────────────────────────────────────────────────────────┘
                       │NavigationSolution / Diagnostics / SystemEvent
┌────────────────────────────────────────────────────────────────────┐
│ L5 输出层                                                           │
│ CSV/JSON/轨迹文件、日志、可视化、外部应用接口                       │
└────────────────────────────────────────────────────────────────────┘
```

### 3.1 各层职责

1. **L0 应用与运行时层**：读取多文件 JSON 配置并校验 schema；按配置实例化 modules；检查连接和消息类型；管理启动/停止、事件时间顺序、背压、日志及全局错误处理。
2. **L1 数据源层**：从 `data1` 回放文件或未来的传感器设备读入原始记录。只负责 I/O，不做导航计算。
3. **L2 Decode 与标准化层**：逐列解析文本/协议，按配置把数值映射到 typed records；转换单位、坐标轴和时间尺度；保留原始字段、缺失掩码、来源和时间戳。`0` 缺失规则在 GNSS decoder 中按字段实施。
4. **L3 传感器处理与数据准备层**：`ImuProcessor` 做标定、时间间隔质量检查和样本规范化；`GnssPreprocessor` 做 rover/base 配对、共视卫星整理、卫星状态有效性筛选和观测质量标记；`FilterManagerModule` 为每个算法实例排序/缓存事件；`DynamicStatusModule` 消费 IMU 与按配置选择的 `InertialStateSnapshot`/RTK/TDCP 运动信息，发布带置信度和证据标志的动态状态；`ConstraintManager` 按动态状态和约束模型条件准备 NHC/ZUPT 候选量测。该层不计算 Kalman 增益，不估计滤波状态。
5. **L4 GNSS 解算与导航融合层**：RTK solution engine（经 RTKLIB adapter 隔离）输出带状态质量和协方差的 RTK 天线位置/速度；相邻 rover/base 历元原始载波相位形成 TD-DDCP。`InertialNavigationModule` 独占名义导航状态并执行 IMU mechanization；`FilterManagerModule` 将时序匹配后的 INS 快照、GNSS/TDCP/约束原始观测构造成标准化 `NavigationMeasurementBundle`；`EstimationModule` 选择 INS 专用过程/量测模型并调用通用 `StateEstimator` 完成 `dx/P` 预测与更新。估计模块经带时间戳的 `ErrorStateFeedback` 将误差注入反馈给惯导模块；惯导模块校验并应用后发布修正后的 `NavigationSolution`。通用 `StateEstimator` 不含具体导航或传感器模型。当前 LC/TC/LTC 分别使用 RTK P/V、卫星级 TD-DDCP、RTK 位置加 L1 TDCP 推导区间平均速度；ZUPT、Zero_Position_Change、Zero_Heading_Change 与 NHC 为独立可配置约束模型，经 DynamicStatus 与 ConstraintManager 准入后进入同一量测更新/反馈事务。RTK 整数模糊度仅由 RTK engine 管理。
6. **L5 输出层**：订阅导航解和诊断消息并写入所选目标；输出层不反向修改算法状态，控制命令通过显式的 command message 传递。

### 3.2 module 之间的信息传递

- 总线消息为不可变、强类型值对象，至少携带 `timestamp`（事件时间）、可选接收时间、`sequence_id`、`source_id`、`frame_id`、单位/质量标记和缺失字段掩码。
- 标准消息包括 `RawImuRecord`、`RawGnssRecord`、`ImuSample`、`GnssObservation`、`PreparedGnssEpoch`、`RtkSolution`、`PreparedTdDdCarrierPhaseEpoch`、`ConstraintObservation`、`DynamicStatus`、`InertialStateSnapshot`、`ErrorStateFeedback`、`FeedbackApplied`、`NavigationSolution`、`ModuleDiagnostic` 和 `SystemEvent`。TD-DDCP 消息显式包含参考/当前历元及共同卫星与 pivot 身份。
- module 通过声明 `subscriptions` / `publications` 建立连接。启动时按 JSON pipeline 验证消息类型兼容；未连接的必需输入应在启动时报错。
- 回放可由 source module 并发读取，但进入单个导航估计实例后由 `FilterManagerModule` 按事件时间串行调度。RTK 迟到时，FilterManagerModule 使用 IMU 缓存请求配对的惯导/估计模块回滚重放；超出缓存的历元按配置拒绝。
- 同一标准化传感器流可广播给多个独立导航算法实例用于比较；每个实例有独立状态和输出 topic。跨算法融合必须配置一个显式融合 module，不把不同算法的状态向量直接平均。
- `InertialNavigationModule` 是名义位置/速度/姿态/bias/杆臂的唯一所有者；`EstimationModule` 是 `dx/P` 及滤波历史的唯一所有者。二者只交换不可变的 `InertialStateSnapshot`、带序号的 `ErrorStateFeedback`/`FeedbackApplied` 和诊断/协方差信息，不共享可变对象或让其他 module 直接改写状态。

### 3.3 DynamicStatusModule 与约束模块的边界

`DynamicStatusModule` 是独立的运动状态分析器，不是滤波器 measurement model。它只读取带时间戳的传感器/导航消息，维护有限时长滑窗，并发布不可变的 `DynamicStatus`；不得修改 `FilterContext`、状态向量或协方差。默认检测器使用 `ImuSample` 和 `InertialNavigationModule` 提供的 `InertialStateSnapshot`（未受当前待判定约束反馈污染）；有效的 `RtkSolution` 速度和 LTC 派生 TDCP 区间速度可作为可配置辅助证据。若订阅融合后的导航解，必须配置至少一个 GNSS/IMU 事件的延迟，禁止让同一历元的约束结果立即反过来改变用于决定该约束是否有效的分类，避免代数反馈。

`ConstraintManager` 订阅 `DynamicStatus`，并结合原始测量有效性、车辆模型、安装参数和约束自身质量门限，决定是否发布 `ConstraintObservation`。状态标签仅作为门控证据，不等同于约束已获准：ZUPT、Zero_Position_Change、Zero_Heading_Change 均需高置信 `stationary` 持续达到配置驻留时间；零位置/零航向变化模型还要求静止锚点有效。NHC 需车辆模型有效、坐标安装参数可用并满足运动质量条件。分类输入不足、时间戳过期或状态为 `unknown` 时，不因状态模块单独推断约束。约束消息包含观测时间、模型 ID、锚点 ID（若适用）、质量/证据、噪声或其配置引用；物理预测、残差与雅可比仍由 `EstimationModule` 中的专用 measurement model 构造。

### 3.4 FilterManagerModule 与算法模块的边界

`FilterManagerModule` 是运行时的数据准备/事件调度器，不是惯导、导航模型协调器或通用数值估计器本身。它负责：

- 按时间戳整理 IMU、rover、base 事件；使用已配置的 rover/base 时间容差配对历元。
- 按 SVID 将 rover/base 观测配成历元；从 base 对应行取得可用卫星位置/钟差/高度角。按用户规则，缺失卫星直接忽略，不尝试外部星历回退；同时为 RTK 解算器与相位差分支路提供一致的原始观测快照。
- 对字段缺失、时间不匹配、重复卫星、质量不合格生成诊断和 `PreparedGnssEpoch`；尽可能保留某卫星其他有效字段，不把整行无条件丢弃。
- 向 RTK solution engine 提供 GNSS 历元，并向融合滤波器提供时间一致的 `RtkSolution`、差分载波相位历元、`ImuSample` 和约束观测；维护必要的时间回滚缓存与事件统计。

`FilterManagerModule` 负责构造统一的观测数据包（观测值、时间、质量、协方差/相关性来源、卫星/pivot 身份等），但不拥有滤波状态、不计算 INS 名义状态、不做 mechanization、不选择/实现状态或观测数学模型、不线性化模型、不修改 `P/dx`、不执行误差注入，也不决定 RTK 整数固定。具体模型选择与观测预测、残差、雅可比构造由 `EstimationModule` 内的领域模型完成；通用矩阵预测/更新由其调用的 `StateEstimator` 完成。INS 名义传播/反馈属于 `InertialNavigationModule`；TD-DDCP 连续性由相位连续性组件维护，RTK 模糊度和整数固定留在 RTK engine。这样增加算法时复用数据层、调度层和通用滤波数值内核，同时允许各导航算法保留独立的物理模型与状态定义。

### 3.5 module 处理顺序

1. Source 发布 raw record；Decoder 解析并发布标准化消息。
2. `ImuProcessor` 输出 FRD body frame 的 `ImuSample`；`GnssPreprocessor` 与 `FilterManagerModule` 将 rover/base 观测配成时间一致的 GNSS 历元。
3. RTK solution engine 对 GNSS 历元执行 RTK 解算，发布 `RtkSolution`；连续性组件缓存前一有效载波相位历元，并构造相邻历元 TD-DDCP，保留卫星/信号/pivot 身份及周跳质量信息。
4. `DynamicStatusModule` 按事件时间更新滑窗并发布 `DynamicStatus`；`ConstraintManager` 根据状态、置信度和约束配置形成候选 `ConstraintObservation`。
5. `FilterManagerModule` 将 IMU 事件按时间调度给 `InertialNavigationModule` 和 `EstimationModule`：惯导模块传播名义状态并发布预测快照；FilterManager 将同一 IMU 区间及快照送入估计流水线。GNSS/TDCP/约束预处理结果由 FilterManager 按时间、质量和相关性元数据组装成 `NavigationMeasurementBundle`。
6. `EstimationModule` 接收 INS 快照、IMU 区间和量测包，调用配置的 INS 误差状态过程模型与 LC/TC/LTC 领域量测模型，以及已准入的 ZUPT/NHC 约束模型，计算/线性化 `F/Q` 及 innovation、`H/R`；然后将这些通用数值输入交给内部 `StateEstimator` 执行误差状态与协方差预测/更新。`StateEstimator` 不知道 RTK、INS、TDCP、车辆约束或量测含义。
7. 每次误差状态更新后，`EstimationModule` 从 StateEstimator 结果形成带 `epoch_id` 的 `ErrorStateFeedback`；惯导模块验证时间/序号，注入位置、速度、姿态、bias 和杆臂修正，并返回 `FeedbackApplied`。收到确认后，EstimationModule 请求 StateEstimator 将误差均值归零并按 reset Jacobian 变换 `P`；该反馈事务需串行、原子化并可回滚。
8. `InertialNavigationModule` 在反馈成功后发布唯一权威的已修正 `NavigationSolution`；不含新观测的 IMU 时刻则发布最新已校正名义状态的传播结果。估计模块发布协方差和滤波诊断快照，由结果汇总器按 `epoch_id` 附加到输出；它不另发一套竞争的名义轨迹。
9. DynamicStatusModule 默认读取未被其自身候选约束即时反馈污染的惯导传播快照；约束仅可在时间调度和反馈顺序明确后进入滤波更新。可配置多个完整的 `InertialNavigationModule + EstimationModule + StateEstimator session` 算法实例订阅同一输入流作对比，各实例状态隔离。
## 4. 状态、协方差与数据结构

### 4.1 状态定义

误差状态建议定义为：

三种融合模式可共享 `ins_error_18d_lever_arm_v1` 状态：`dx = [dr_N,dr_E,dr_D, dv_N,dv_E,dv_D, phi_N,phi_E,phi_D, dbg_x,dbg_y,dbg_z, dba_x,dba_y,dba_z, dl_bx,dl_by,dl_bz]^T`。索引定义为：位置 `[0,3)`、速度 `[3,6)`、姿态 `[6,9)`、陀螺 bias `[9,12)`、加计 bias `[12,15)`、IMU 到 GNSS 天线杆臂误差 `[15,18)`。该预设固定为 18 维；RTK engine 自己维护的模糊度不加入此状态向量。其他 JSON 选定的状态模型可使用不同的已注册布局。

前 15 维是惯导误差状态：局部 NED 位置误差（m）、速度误差（m/s）、姿态小角（rad）、本体系陀螺零偏误差（rad/s）和本体系加计零偏误差（m/s²）。索引 15–17 是 body/vehicle frame 下杆臂估计误差（m）。标称状态保存 `lever_arm_b`，首期初值 `[0,0,0] m`；杆臂为固定安装参数，误差状态按配置的小随机游走噪声传播，更新后反馈到标称杆臂并重置对应误差均值。比例因子若启用，需在状态布局版本中追加，不可暗中改变已有索引。

`InertialNavigationModule` 独占并维护 `NominalNavState`：时间戳、LLA/ECEF、NED 速度、`q_bn`、陀螺/加计 bias、当前估计 `lever_arm_b` 和初始化标志；其他 module 仅收到不可变快照。`EstimationModule` 拥有导航估计会话、领域模型注册表和历元相关历史；会话内的通用 `StateEstimator` 实例拥有数值滤波状态 `dx/P`。TD-DDCP 前一历元相位/卫星状态/pivot/时间戳/质量和一致更新所需的状态交叉协方差，以及 ZPC 位置锚点和 ZHC `HeadingLockState`，由 `EstimationModule` 的领域上下文维护，供模型构造有效创新协方差/受约束输出，再传给通用内核。heading-lock 输出状态不隐式改变默认 15/18 维 INS 状态布局；若将其并入增广状态实现，必须显式注册状态布局版本并验证完整协方差。杆臂初始值为零，但必须配置合理的初始方差；杆臂与初始位置/姿态误差可能相关，系统须保留交叉协方差并诊断可观测性。

### 4.2 TD-DDCP 历元配对与连续性

TD-DDCP 配对键至少包含 `{constellation, satellite PRN, signal/frequency, pivot PRN}`，并记录前后历元时间戳。只有同一卫星/信号在前后历元连续且无周跳、共视集与 pivot 兼容时才形成该行观测。首个有效历元只建立缓存，不执行相位更新；时间间隔超限、缺星、周跳或 pivot 不兼容时，丢弃受影响配对并以当前有效历元重新建立参考。每星周跳会使该星涉及的时间差分失效；pivot 周跳或切换会影响该组差分，应整组重置或重建配对。RTK 引擎内部的 ambiguity key 和 pivot 管理独立于融合状态。

### 4.3 初始化与输出

初始化输入应包含初始位置、速度、姿态、IMU bias、初始杆臂（默认零）、18×18 初始协方差，以及时间/来源元数据。必须拒绝非有限数、非正定协方差、无效四元数、纬度越界和重复/倒退时间戳。对外唯一导航轨迹由 `InertialNavigationModule` 发布：时间、已反馈修正的 LLA/ECEF、NED 速度、姿态、杆臂、解状态和反馈 epoch。协方差、卫星数/ratio、创新统计和告警由 `EstimationModule` 的同 epoch 结果快照提供，并由输出汇总器关联；不得让未反馈的误差状态解作为最终位置/速度/姿态输出。

## 5. 算法处理流程

### 5.1 IMU 传播

#### 5.1.1 InertialNavigationModule：名义状态传播

以下机械编排只由 `InertialNavigationModule` 执行并维护名义状态，不在估计模块内再维护第二套位置/速度/姿态：

1. IMU 时间戳代表采样时刻。按时间戳排序并检查间隔；对重复、倒序、过大间隔采用可配置策略（拒绝、重初始化或显式降级），不得静默继续。
2. 按配置对原始增量去 bias；若开启比例因子，按报告给出的对角模型补偿。
3. 运行二子样 coning/sculling 机械编排。首个样本、缺样、长间隔和重置后的历史增量规则必须显式处理；首个增量不可既用于初始化历史又再次按完整两样本公式重复计入。
4. 更新姿态（归一化并检查）、速度和 LLA；处理极区 `cos(lat)`、高度、地球曲率和重力模型边界。

完成名义传播后，惯导模块发布对应 `InertialStateSnapshot`；遇到无效 IMU 或时间异常时发布诊断，不发布伪造状态。

#### 5.1.2 EstimationModule：导航领域模型协调

`EstimationModule` 是导航/传感器模型与通用估计内核之间的适配和协调层。它接收同一 IMU 区间、匹配的 `InertialStateSnapshot` 和 FilterManager 构造的量测包；持有配置的状态布局、过程模型、量测模型及滤波会话。它把 INS 15 维误差扩展为含 body-frame 杆臂误差的固定 18 维布局（或载入其他注册布局），构造误差状态 `F/G`、离散 `Phi/Qd`，并通过测量模型构造观测预测、innovation、`H/R`。它不积分名义位置/速度/姿态；具体矩阵数值运算委托给 `StateEstimator`。

采用报告给出的二阶 `Phi` 和噪声梯形积分作为初始基线；实际 `Qc` 的单位及连续/离散噪声定义须在传感器参数中统一，避免重复乘除采样周期。

#### 5.1.3 StateEstimator：领域无关的通用数值估计内核

`StateEstimator` 是可复用的底层 C++ class，而不是运行时 module，也不是 INS/RTK/车辆模型容器。它不依赖 `ImuSample`、`RtkSolution`、GNSS 观测、导航坐标系、杆臂、LC/TC/LTC 或任何具体物理语义；不创建/选择状态变量、不构造量测、不计算几何距离/雅可比、不执行 mechanization，也不决定误差状态应如何反馈给名义状态。

它只接收由上层提供且维数一致的通用数值对象，例如状态向量 `x`（在本项目内为误差向量 `dx`）、协方差 `P`、状态转移矩阵 `Phi`、过程噪声协方差 `Qd`、innovation `nu`、观测雅可比 `H`、观测噪声 `R`，以及门控/稳健更新策略的通用参数；提供预测、线性 Kalman 更新、创新协方差与增益计算、NIS/标准化残差、数值健康检查、误差状态 reset 的通用矩阵变换等运算，并返回结构化数值结果与诊断。状态维数可动态配置，不将 18 维 INS 状态写死在内核中。

责任边界：`FilterManagerModule` 准备时间一致的输入观测包；`EstimationModule` 根据 INS 快照和所选模型构造 `Phi/Qd` 与 `nu/H/R` 并调用 `StateEstimator`；`StateEstimator` 只做通用数值估计；`InertialNavigationModule` 只传播/维护名义导航状态并应用估计模块发来的反馈。底层 class 可单独用合成矩阵测试，也可供未来其他估计问题复用；更换 LC/TC/LTC、加入新测量或新状态模型，不应要求修改 `StateEstimator`。

### 5.2 RTK 位置/速度松组合更新

1. RTK solution engine 使用准备好的 rover/base GNSS 历元执行 RTK 解算。已知基站坐标取配置的 `base_coordinates` 文件，作为基站天线相位中心的 ECEF 米坐标，传给 RTK 引擎的参考站位置；不得把 base 文件中的逐星 SatX/Y/Z 当作基站坐标。RTK 输出为天线相位中心位置、速度、解状态、有效卫星数、时间戳及可用协方差。
2. **FLOAT 与 FIX 均可用于松组合。** FIX 前产生的有效 FLOAT 解不需等待整数模糊度固定即可进入松组合更新；不得仅因 `status != FIX` 而丢弃 FLOAT。依据解状态和引擎协方差分别设置/映射观测噪声，并保留 `FIX/FLOAT/INVALID` 状态、卫星数、ratio（若可用）与位置/速度分量有效标记。INVALID、时间不匹配、非有限数或协方差无效的分量拒绝更新。若配置允许，其他状态（例如 DGPS）由单独开关决定，默认不纳入本项确认。
3. 将 INS 状态传播/回滚到 RTK 解的观测时间。用标称姿态和当前 `lever_arm_b` 计算 INS 预测的 rover 天线位置；天线速度还应包含杆臂旋转速度项，并按一致的 ECEF/NED 坐标定义实现。
4. 构造 `z_pv = [r_RTK^e, v_RTK^e]` 与预测函数 `h_pv(x)`，残差为 `z_pv - h_pv(x)`。雅可比包含位置、速度、姿态、IMU bias 和杆臂状态对天线位置/速度的影响。
5. 按 FIX/FLOAT 状态和质量配置观测协方差：优先使用 RTK 引擎提供的协方差；若只有精度指标，则按明示的模型构造并分别配置 FIX/FLOAT 噪声。FLOAT 可单独启用、降权或按质量门限拒绝，但系统默认允许合格 FLOAT 更新。保留位置-速度交叉协方差（若可用），拒绝无效分量并记录原因。
6. `EstimationModule` 将 RTK 观测交由配置的 LC 量测模型线性化，再调用通用 `StateEstimator` 执行 EKF 更新；估计模块从数值内核结果形成 `ErrorStateFeedback`，请求 `InertialNavigationModule` 注入 INS 名义位置、速度、姿态、陀螺/加计 bias 和杆臂状态。收到同 epoch 的 `FeedbackApplied` 后，EstimationModule 调用内核执行误差 reset 并用 reset Jacobian 更新 `P`。该更新不新增载波模糊度状态。

### 5.3 TC：相邻历元时间差分载波相位紧组合更新（TD-DDCP）

1. 每个 GNSS 历元先从 rover/base 原始载波相位形成接收机单差，再以共同可见卫星和选定 pivot 形成卫星间双差。相位 `L` 以 cycles 输入并乘频点波长 `λ` 转成米；逐字段检查缺失、卫星位置有效性和周跳质量。
2. 对卫星 `s`、pivot `p` 和当前历元 `k`，记单历元双差为 `D_k^{s,p}=λ{(L_{r,k}^s-L_{b,k}^s)-(L_{r,k}^p-L_{b,k}^p)}`。紧组合观测为相邻历元时间差分：`y_{TD}^{s,p}=D_k^{s,p}-D_{k-1}^{s,p}`。无周跳且信号/卫星/pivot 身份连续时，常值整周模糊度差分项抵消；融合 ESKF 不含模糊度状态。
3. 预测量为对应几何双差的时间变化：`h_{TD}^{s,p}=G_k^{s,p}(x_k)-G_{k-1}^{s,p}(x_{k-1})`，其中每个 `G` 使用该历元 rover INS/杆臂预测的天线 ECEF 位置、base 天线坐标及该历元卫星 ECEF 位置计算。大气延迟、卫星钟差和其他改正按差分模型一致处理；不能误把时间变化项当作跨历元公共项全部抵消。
4. 首个有效历元仅缓存，不做 TD-DDCP 更新。后续必须满足最大间隔、同频点、共视卫星和 pivot 连续、两端相位有效且无周跳。配对失败时丢弃该差分行并重新建立当前参考历元；不得用跨越未知周跳的相位差。
5. 创新为 `y_TD-h_TD`。雅可比同时依赖当前和参考历元 INS 状态、杆臂及其误差，不能把参考历元状态视为精确常量。实现需在滤波器内部保留参考历元状态与当前状态的交叉协方差，或使用等价的状态增广/一阶滞后因子更新，并将更新一致地回传至当前导航状态。噪声协方差应包括两历元载波噪声差分、卫星间共享 pivot 引起的相关性及时间相关噪声。

### 5.4 LTC：RTK 位置与 L1 TDCP 位移/速度更新

LTC 与 TC 的区别是：LTC 不把每颗卫星的 TD-DDCP 作为位置几何残差直接更新 INS 位置，而是先由 L1 TD-DDCP 解算历元间 rover 天线位移，再形成区间平均速度观测。

1. RTK 输出的位置作为松组合位置量测，按第 5.2 节的位置模型更新 INS；RTK 速度独立配置，默认在 LTC 关闭，以免与载波相位推导的速度重复约束。若显式启用 RTK 速度，必须纳入与相位推导速度之间的互相关/去相关策略。
2. 仅取 L1 TD-DDCP 行。对每行，以前一历元 INS 天线 ECEF 位置为线性化点，写成 `y_i = c_i + A_i Δr + ε_i`。`c_i` 包含基站坐标、前后卫星位置和参考位置处的已知几何变化；`A_i` 是当前几何双差对 rover 位移的 ECEF 雅可比。构造完整 L1 协方差 `R_L1`，保留共享 pivot 相关性，并加入参考历元位置不确定度传播项。
3. 当几何矩阵满秩且条件数满足配置门限时，使用加权最小二乘求 `Δr = (AᵀR_L1⁻¹A)⁻¹ AᵀR_L1⁻¹(y-c)`，并由信息矩阵逆得到 `R_Δr`。卫星数不足、几何退化、pivot/周跳质量不满足或矩阵分解失败时拒绝该速度更新，并记录诊断。
4. 转到 NED 后构造 `z_v = R_e^n Δr / Δt`，`R_v = R_e^n R_Δr (R_e^n)ᵀ / Δt²`。该量严格代表区间平均天线速度。当前实现将区间平均速度与前后 INS 端点速度均值 `0.5(vₖ₋₁+vₖ)` 比较，速度观测雅可比为当前速度块的 `0.5I`；前历元速度协方差按 `0.25P_v,k−1` 投影到量测协方差。此近似未包含前后速度互协方差及位置-速度交叉协方差，须在日志和性能报告中明确，并作为后续固定滞后滤波改进项。
5. 杆臂非零时，量测对应天线位移，必须建模天线与 IMU 参考点的旋转位移差；杆臂在线估计需保留对速度/位移雅可比的影响。L2 在该 LTC 量测模型中不参与位移解算，可继续用于 RTK 解算和周跳质量检测。

### 5.5 混合松紧组合的相关性处理

RTK 解通常使用与 TDCP 相同的原始 GNSS 观测。因此，`RtkSolution` 误差与载波相位派生量一般相关；LTC 的 L1 TDCP 位移/速度观测还跨越前后两个历元，前后导航状态也相关。**禁止将 RTK 位置与 L1 TDCP 派生速度默认视为独立，或把前历元位置/速度视为精确值**；否则会重复使用观测信息并低估协方差。当前实现仅投影前历元位置和速度边际协方差，未保留完整跨历元和 RTK/相位互协方差，故其协方差与 NIS 仍为近似量。

首选实现为在同一历元构造联合量测向量和完整协方差：

`z = [z_pv; z_TD]`, `R = [[R_pv, R_pv,TD], [R_TD,pv, R_TD]]`，并把前一历元状态不确定性及跨时状态协方差纳入创新协方差。

RTK adapter/measurement correlation component 必须给出 `R_pv,TD` 的来源，并处理 RTK 解与当前相位历元的相关性、TD-DDCP 对前后两历元原始相位的依赖，以及前后 INS 状态交叉协方差。若所用 RTKLIB 接口不能导出足够信息，则不能默认交叉协方差为零；需采用经验证的联合估计/去相关策略，或配置只启用一组量测的安全模式。单纯任意放大噪声不构成严格统计融合。

联合更新或经验证的去相关后，每个事件时间只执行一致的状态/协方差更新与闭环反馈；若采用分步实现，必须通过联合更新等价性验证。RTK 解无效时仍可在质量允许且相位连续性有效时执行 TD-DDCP 更新；相位质量不足时仍可单独使用有效 RTK P/V 量测。

### 5.6 周跳、基准星及历元连续性

周跳探测可组合接收机标志、双频 geometry-free、Doppler 积分、INS 预测相位变化残差等策略。检测门限、最大历元间隔和统计窗口配置化。周跳使受影响卫星/频点跨该历元的 TD-DDCP 配对无效；当前历元可作为下一次配对的参考，不在融合滤波器中初始化模糊度。

pivot 必须在构造一对相邻历元的双差时保持一致。pivot 切换时，优先结束旧配对并以当前历元重新选定 pivot、重建缓存；如果实现选择对旧双差行做变换，须验证观测、协方差和几何项的一致变换。无需对融合 ESKF 做模糊度状态变换；RTK engine 内部 pivot/ambiguity 生命周期由其 adapter 负责。

### 5.7 ZUPT 与静止段位置/航向约束

ZUPT、Zero_Position_Change（ZPC）和 Zero_Heading_Change（ZHC）基于同一静止判决，但表达不同约束：ZUPT 限制当前地面相对速度；ZPC 锁定静止段位置；ZHC 锁定静止段 heading。它们不改变 LC/TC/LTC 的定义，也不由 DynamicStatusModule 直接修改滤波器。ZPC、ZHC 各自独立做 NIS 准入；ZPC 被拒绝不得屏蔽 ZHC。ZHC 的目标是：锁定期间输出 heading 与锁定时的 heading 保持不变，heading-lock 1σ 保持锁定时的数值不变。该输出是受约束的锁定估计及其锁定不确定度，不应伪装成每历元重新计算的普通自由 ESKF 边缘协方差。经准入的有类型 `ConstraintObservation` 加入当前历元 `NavigationMeasurementBundle`；专用模型/约束管理器执行锁定状态投影，通用 `StateEstimator` 仍只执行通用数值运算。

#### 5.7.1 ZUPT：静止状态下的零速度更新

**适用条件**：ZUPT 仅在 DynamicStatusModule 输出高置信 `stationary`，状态持续超过 `zupt.minimum_stationary_duration_s`，且 ConstraintManager 的独立准入条件全部满足时才生成。运动、起步/停车转换、`unknown`、证据冲突、输入过期或 IMU 饱和/时间间隔异常时，拒绝 ZUPT。低速本身、单独的 RTK FIX 标志、单独的加速度模长接近重力均不足以判静止。

**测量模型**：在地面相对 NED 速度定义下，静止时 IMU 参考点的速度为零。设传播名义速度为 `v_hat_n`，则伪观测 `z_zupt = [0,0,0]^T m/s`，预测 `h_zupt(x) = v_hat_n`，创新为 `nu_zupt = z_zupt - h_zupt = -v_hat_n`。对于 `dx = truth - nominal` 的速度误差约定，候选雅可比为 `H_zupt = [0_(3x3), I_(3x3), 0_(3x3), 0_(3x3), 0_(3x3), 0_(3x3)]`，对应状态 `[dr,dv,phi,dbg,dba,dl_b]`；若全局误差定义取反，必须相应变换符号。当前设计仍要求在锁定全局误差符号时，用有限差分核对该模型与反馈方向，不能仅照抄 PDF 的矩阵符号。

该量测假设的是 IMU 参考点在地面相对导航系中的速度为零，不是 ECEF 坐标导数为零（地球自转下后者不成立）。若机械编排速度采用其他参考系，必须先将零速伪观测转换到相同速度定义。杆臂为零或整车静止且无旋转时 IMU/天线零速等价；存在旋转、非零杆臂或参考点不在车辆刚体上时，应按安装/杆臂模型修正参考点速度，或禁止该 ZUPT 更新。

**噪声与稳健性**：`R_zupt` 配置为正定、非零的三维速度协方差，并按 IMU 等级、静止检测质量、参考点定义与实验标定确定；不得设为零或无限小。更新前检查 innovation/NIS 和状态/协方差健康度，超过门限则拒绝并记录原因；必要时采用可配置的协方差膨胀。连续静止期间重复伪量测并非独立新信息，必须配置更新间隔/最大更新频率，或建模时间相关性/有效信息量，避免每个 100 Hz IMU 样本都被当成独立零速事实而导致协方差虚假收缩。成功更新后执行标准的误差反馈及 reset，不直接在 ConstraintManager 内清零速度或偏置。

#### 5.7.2 Zero_Position_Change（ZPC）：静止段零位置变化

**锚点生命周期**：Zero_Position_Change 只有在 ZUPT 的静止触发条件已成立且 ZPC 自身配置启用时才运行。首次通过静止驻留和约束准入的历元建立静止锚点 `a`，保存滤波器在该时刻（进入约束更新前）的 ECEF 位置、`q_bn`/anchor heading、对应导航/惯导状态时间、位置与姿态协方差、与当前状态的互协方差、`anchor_id` 和状态来源。默认不回溯到静止候选窗口起点；从确认静止并建立锚点后开始约束。静止持续期间可按独立配置频率更新。退出 `stationary`、ZUPT 静止触发撤销、状态变为 `unknown`、发生明显运动/时间断裂或锚点过期时关闭锚点；下一次确认静止时创建新锚点。

**观测模型**：采用地固 ECEF 米制位置，或等价的固定锚点局部切平面坐标。伪观测 `z_zpc = 0 m`，预测 `h_zpc = r_e,k - r_e,a`，创新为 `nu_zpc = -(r_hat_e,k - r_hat_e,a)`；它只表达“当前相对静止锚点没有位移”，不将绝对位置设为零。若使用当前历元 NED 位置状态，模型须先按明确的 ECEF/NED 变换将当前位置和锚点变换到同一坐标系。杆臂不为零、参考点不一致或静止判决期间存在旋转时，应把位置换算到同一个物理点，或拒绝该约束。

ZPC 的线性化同时依赖当前状态和锚点状态：`H_zpc = [H_p,k, -H_p,a]`（符号按全局误差定义统一）。锚点不是精确真值。必须传播/保存 `P_aa`、当前与锚点互协方差 `P_ka`，并在联合/固定滞后更新中正确更新状态；等价实现可使用增广状态或严格推导的有效量测协方差。不得把前历元位置直接以零方差常量塞入 EKF，否则会把其误差当作真值并造成绝对位置协方差虚假收缩。`R_zpc` 为非零噪声，覆盖静止模型容差、锚点时间/状态不确定度未显式建模部分及参考点误差；完整锚点不确定度已通过联合协方差处理时不得重复计入。

#### 5.7.3 Zero_Heading_Change（ZHC）：静止段 heading-lock

**用户可见契约**：本功能定义为静止期间锁定当前估计 heading，而非只限制相对锚点的残差。锁定生效时，输出 `heading_deg` 应与锁定值相同（仅容许数值精度误差），输出 `heading_rms_1sigma_deg` 应与锁定时的锁定不确定度相同。ZHC 不表示 heading 为零，也不提供磁北/真北等新的绝对航向信息；锁定值只是本次静止段开始时滤波器已有的 heading 估计。锁定 RMS 不能被表述为独立真值误差，也不能因反复施加同一约束而人为收缩。

**激活与锁定快照**：只有 DynamicStatus 持续满足高置信 `stationary`、ZUPT 本历元更新通过、heading/姿态状态有效且时间连续时，才创建 `heading_lock_id`。在第一次准入历元应用正常测量更新后，捕获 `heading_lock_rad`、`heading_lock_sigma_rad`、固定参考坐标系、时间戳和来源质量。该快照在整个锁定 episode 内不可被普通 RTK、TDCP、ZUPT/NHC、误差反馈或滤波重线性化改写。ZPC 有独立 NIS 门限；ZPC 拒绝不能抑制有效的 ZHC。

**估计状态处理**：当前实现的 4 维动态克隆锚点不满足该契约，因为每次 `FeedbackApplied` 都可能通过锚点误差态修改 `anchorHeadingRad_`，导致当前 heading 与锚点共同漂移、innovation 仍很小。新设计不再把锁定参考当作可被其他量测共同修正的普通克隆状态。`EstimationModule` 创建独立、不可变的 `HeadingLockState`，并将锁定 heading 作为当前名义姿态的受约束分量：每次传播/反馈后，在锚点参考坐标系中投影回 `heading_lock_rad`；反馈中的 heading 改正分量不得改变锁定值。roll/pitch、速度、位置、偏置等其余状态仍照常传播和更新。局部 NED 变化时须先用 ECEF/锚点参考姿态完成坐标变换，不能直接比较不同局部系 Euler yaw。

**heading RMS 语义与协方差**：锁定期间用于解输出和页面展示的 heading 1σ 明确取 `heading_lock_sigma_rad`，锁定值不随当前自由 ESKF 的 marginal covariance 改写。滤波器仍须保留完整协方差供其他状态更新，并另行输出 `free_heading_covariance_1sigma` 诊断值；二者分别表示“锁定 heading 的锁定时不确定度”和“若不施加 heading-lock 时当前 ESKF 的形式不确定度”，不能混为同一 RMS 字段。若后续实现需要将锁定变量并入联合 ESKF，则必须给出 PSD 保持的状态分解/协方差投影推导，并验证锁定方差严格不变；禁止简单冻结一个相关协方差矩阵对角元或把锚点设成零方差来伪造一致性。这样可保证产品输出 heading/RMS 稳定，同时不宣称其是普通无约束滤波器的当前后验协方差。

**创新与质量监控**：ZHC 仍记录每历元锁定预测残差、未约束候选 heading、反馈前后 heading 差、`heading_lock_id`、接受/拒绝状态及自由协方差。锁定采用确定性状态投影/约束执行，不得把每个历元对同一静止 heading 的重复事实误当成独立观测；若实现采用等价的量测更新，需显式建模跨历元完全相关的约束。残差过大、gyro/IMU 显示真实旋转、检测状态退出静止或输入时间断裂时，应拒绝/降级并记录原因，而不是持续把错误 heading 强拉回去。

**退出与重入**：DynamicStatus 离开 `stationary`、ZUPT 连续拒绝达到配置门限、确认真实转动、进入 `unknown` 超时或时间戳断裂时，结束当前 `heading_lock_id`。退出时以锁定 heading 作为释放初值，解除投影并按配置膨胀自由 ESKF heading 协方差，避免将锁定阶段“RMS 不变”的产品语义误解释为释放后仍无不确定度增长；新的静止 episode 重新捕获 heading 与 sigma。需配置状态迟滞/连续失败次数，避免临界状态下反复开关造成跳变。

**验收条件**：锁定区间内，`abs(wrap(heading_out - heading_lock)) <= heading_lock_tolerance_rad`，且 `abs(heading_rms_out - heading_lock_sigma) <= covariance_output_tolerance`；记录到的普通滤波 yaw feedback 不得改变锁定输出。验证姿态四元数归一化、roll/pitch 连续性、ECEF/NED 局部系变化、锁定进入/退出、其他量测在锁定期更新、NIS/innovation 诊断和完整协方差半正定性。若锁定期间出现明确真实旋转或动态误判，必须按退出策略释放，而不能为满足恒定曲线而掩盖真实运动。

#### 5.7.4 NHC：车辆非完整运动约束

**目标与触发**：Non-Holonomic Constraint（NHC）表示车辆在正常轮胎附着、无明显侧滑/跳跃且车辆模型适用时，车辆参考点在车体横向和垂向的速度近似为零。它不约束前向速度，也不是“车辆静止”判据。ConstraintManager 仅在 DynamicStatus 为有效运动状态（直行、匀速、加/减速或模型可补偿的转弯）、输入新鲜、车体/IMU 安装参数有效、并通过独立质量准入时生成；`stationary` 时由 ZUPT 处理，`unknown`、倒车方向未建模、High Dynamic/Slip、急转弯不可补偿、轮胎打滑、飞跃/颠簸或传感器饱和时暂停或拒绝。

**参考点和坐标系**：约定导航系为 NED、车体为 FRD，`C_b^n` 将 FRD 向量变换到 NED，`C_n^b=(C_b^n)^T`。模型约束车辆参考点（通常以后轴中心或车辆质心为配置参考点）的车体系速度，而不能无条件将 IMU 中心的侧向/垂向速度设为零。若 INS 名义速度位于 IMU 点，定义 `l_b` 为从车辆参考点指向 IMU 的 FRD 杆臂，先计算

`v_ref^b = C_n^b v_imu^n - ω_nb^b × l_b`

其中 `ω_nb^b` 是车体相对导航系的角速率，须由陀螺测量扣除零偏并扣除导航系转动率后得到；若 mechanization 输出的速度不是地面相对 NED 速度，应先按其明确定义转换，不能无条件重复扣除地球自转/运输率。若速度本来就在车辆参考点，则省略杆臂修正。杆臂方向须按上面的定义和叉乘次序实现。定义 `C_b^vehicle` 为 IMU FRD 到车辆 FRD 的静态安装旋转，静态安装角不为单位阵时用 `v_ref^vehicle = C_b^vehicle v_ref^b` 后取分量；不要把车辆坐标轴与 IMU FRD 轴默认视为重合。

**量测模型**：采用二维伪观测

`z_nhc = [0, 0]^T m/s`, `h_nhc(x) = S_v C_b^vehicle v_ref^b`, `ν_nhc = z_nhc - h_nhc(x) = -S_v v_ref^vehicle`,

其中 `S_v = [[0,1,0],[0,0,1]]`，按 `[forward, right, down]` FRD 取侧向和垂向分量。该模型假设车辆参考点无侧向滑移且道路/车体垂向速度可忽略；若其物理假设不成立，应拒绝更新，不能仅靠调小或调大噪声掩盖模型失效。横向和垂向噪声可不同，必要时支持二维相关项。

**Jacobian 与更新**：`H_nhc = ∂h_nhc/∂δx`，只对当前启用的 state layout 列块生成。速度块的基本项为 `S_v C_b^vehicle C_n^b`（具体符号按 `dx = truth - nominal` 及反馈定义推导）；姿态误差项来自坐标旋转对 `v^n` 的导数，陀螺零偏项来自 `-ω_nb^b×l_b`，杆臂状态启用时还包括 `∂h/∂l_b`。若状态速度定义、左右乘姿态误差或 lever-arm 状态不同，不得照搬固定矩阵；实现须以统一误差约定推导，并对每个非零状态块做中心有限差分验证。NHC 的观测只含两个分量，不能填充或更新未定义的状态列。通过创新/NIS 门限后由 EstimationModule 交给通用 StateEstimator 更新，随后仍走常规 ErrorStateFeedback 和姿态误差 reset。

**噪声、频率与相关性**：`R_nhc` 为正定、非零的 `2×2 (m/s)^2` 协方差，可由车辆类型、速度、角速率、路况和 DynamicStatus 质量分档/膨胀；急转弯但模型仍可信时可使用含角速率/杆臂项的模型并调大噪声。配置最大更新频率或相关时间，避免将高频重复伪观测当作独立证据。NHC 与 ZUPT 在静止时约束速度、与 GNSS/TDCP 速度更新也可能相关，因此不在 stationary 下与 ZUPT 重复启用，并须按第 5.7.5 节的相关性策略联合调度。NHC 单独不能提供绝对航向观测；其对航向/零偏的影响须通过协方差耦合和数据可观测性评估，不得将其描述为直接 heading measurement。

#### 5.7.5 约束调度、锚点相关性与拒绝路径

DynamicStatus 判决与约束准入使用独立门限及进入/退出迟滞；ConstraintManager 输出候选/准入/拒绝状态、证据来源、质量、锚点/lock ID、噪声模型及拒绝码。ZUPT/ZPC/ZHC 共享同一静止假设，彼此并非独立的传感器信息；联合启用时必须建模约束间相关性或使用经过验证的联合更新/保守去相关方案，不得简单把三项当成独立强观测重复压缩协方差。ZPC 的跨历元位置锚点及其与当前状态交叉协方差须保留；ZHC 的锁定 heading 与锁定 sigma 用专用 `HeadingLockState` 保持，不得通过反复独立量测把锁定 sigma 压小，也不得把锁定 sigma 冒充自由 ESKF 当前边缘协方差。禁止将零方差精确锚点直接混入联合滤波器；若需要把 ZHC lock 状态联合估计，必须推导并验证正确的先验方差、相关性和 PSD 保持。静止判决所用外部速度证据可能与当前 GNSS 量测共享原始观测，不得把 RTK 速度、TDCP 速度和 INS 派生状态误当作多份独立静止证明，也不得默认零互相关。NHC/ZUPT/ZPC/ZHC 与导航量测的联合更新应明确顺序和相关性假设，并在诊断中标记其为运动模型约束而非传感器真值。

### 5.8 DynamicStatusModule：运动状态估计

该模块在导航解算流水线中提供统一的运动状态标签，供性能分析、运行诊断和 ZUPT/ZPC/ZHC/NHC 准入使用。参考理论报告提出以滑动窗内比力与角速率统计构造静止指数，并要求 INS 速度近零；本设计将其作为可配置候选检测器，而非固定公式或已标定门限。静止比力检验使用由当前姿态和地球重力模型预测的静止 IMU 比力作为参考残差，符号按全局 IMU 约定实现并以静态数据验证；不以“比力模长接近 g”单独作判据。首版采用可解释的规则/统计检测器；接口与分类器实现分离，以便将来替换为 SHOE/GLRT、HMM、贝叶斯分类器或学习模型，但不把未校准的规则分数称为概率。

**输入与时间窗**：

- 必需输入为 `ImuSample`（FRD 角速度、比力、时间戳、有效性标志）。
- 默认辅助输入为 `InertialNavigationModule` 提供的 `InertialStateSnapshot`（NED 速度、姿态/四元数、时间戳、质量）；不得依赖已经融合当前候选约束的最终 `NavigationSolution` 来决定同一历元约束是否有效。GNSS 辅助证据可选用有效 RTK 速度、RTK 解状态/协方差及 L1 TDCP 派生的区间速度。每类证据独立配置启用，并按时间容差与最大陈旧时间对齐。
- 维护基于事件时间的有限滑窗；对每个窗口计算 median/MAD、均值/方差、速度变化率和持续时间。缺样、时间倒退、间隔过长或来源无效时清除/降级相关证据，发布 `unknown` 或低置信结果，不以前值无限延续。

**输出状态**：`DynamicStatus` 至少包含 `initializing`、`stationary`、`constant_velocity`、`accelerating`、`decelerating`、`turning`、`dynamic_other`、`unknown`。加速/减速由经姿态投影到车辆前向轴的纵向加速度符号与门限区分；转弯由角速度、航向变化和/或横向加速度识别，优先于“匀速”标签，避免把恒速转弯误判成直线匀速。若应用不需要方向类别，可在输出同时保留 `longitudinal_acceleration_mps2` 而将加速/减速合并为 `accelerating`。状态为单一主类别，另带连续运动指标和证据标志。

**主要判据**：

1. `stationary`：候选窗使用配置化的滑动窗长度/最小样本数，计算归一化角速率能量、相对静止比力预测的均值/方差（可用 median/MAD 抗离群），必要时计算报告中的 stationarity index `T = mean(||f_k - f_static||²/σ_a² + ||ω_k||²/σ_ω²)`。判定同时检查 IMU 静止特征和 INS NED 速度近零；RTK/TDCP/车轮速度或静止位置散布是可选的外部佐证，不作为 GNSS 失效时启用 ZUPT 的硬依赖。IMU 证据与由同一 IMU 传播的 INS 状态属于同一惯性证据组，不在置信评分中当作独立投票；RTK 与 TDCP 速度也不视为相互独立。若没有有效外部佐证，必须使用经专门标注数据验证的更严格 IMU 门限/驻留时间配置，否则输出 `unknown` 而不触发 ZUPT。比力静止参考的方向/符号必须按传感器轴约定处理并用静态数据确认；阈值须离线标定。仅使用加速度模长约等于重力或仅使用 GNSS 低速不得判定已静止。
2. `constant_velocity`：速度高于静止阈值，线性加速度/速度变化率低于门限，且航向变化与横向加速度不满足转弯判据；同时要求速度来源新鲜且不确定度合格。
3. `accelerating` / `decelerating`：纵向加速度超过正/负门限并持续最短时长；优先结合重力补偿后的 INS 线加速度与有效 RTK/TDCP 速度变化，单源冲突时降低置信度或输出 `dynamic_other`。
4. `turning`：角速度、航向变化或横向加速度超过门限；该类别可与 `accelerating`/`decelerating` 同时以辅助标志表示，主状态优先级由配置定义。
5. 所有阈值需考虑传感器噪声、安装误差、杆臂、IMU bias、姿态有效性和 RTK FLOAT/FIX 质量；姿态不可用时不应从 NED 加速度推断车体纵向加速度。不能将有偏低速/低角速度的传感器视为绝对真值。
6. 证据需声明相关组：`ImuSample` 与由同一 IMU 传播的 `InertialStateSnapshot` 属于同一惯性证据组；RTK 速度与 TDCP 派生速度通常共享 GNSS 观测，属于同一 GNSS 证据组。置信评分不得把同组信号当成独立投票重复累计；若要作为独立证据，应有各自噪声/互相关模型或单独可靠性标定。

**迟滞和置信度**：使用进入/退出两套门限，最短驻留时间与状态转换惩罚，避免在阈值附近逐历元抖动。输出 `confidence` 为 `[0,1]` 的规则置信评分，并附各证据有效性/分数；除非使用独立标定数据做概率校准，不解释为统计学后验概率。`unknown` 用于初始化、证据冲突、输入过期或质量不足。模块需输出状态切换事件、当前状态驻留时长、分量特征、门限命中情况及拒绝原因。

**动态状态消息契约**：

`DynamicStatus` 至少包括事件时间戳、窗口起止时间、主状态枚举、规则置信评分、状态持续时间、stationarity index/门限结果、FRD 角速率统计、相对静止比力模型的加速度残差统计、NED 速度/加速度、车辆前向/横向加速度（可用时）、航向角速度、RTK 状态/卫星数（可用时）、各输入新鲜度与有效性、证据 bitmask 和相关组 ID、候选约束提示、配置版本和诊断代码。`ConstraintObservation` 应包含约束类型（`zupt`/`zero_position_change`/`zero_heading_change`/`nhc`）、事件时间/epoch、静止驻留时间、锚点 ID/时间（若适用）、采用的 `R` 或噪声模型 ID、检测证据与来源相关组、准入状态及诊断码。消息均为不可变值对象。`zupt_candidate`、`zpc_candidate`、`zhc_candidate`/`nhc_candidate` 只表达候选条件，不代表滤波更新已经执行；最终更新由 ConstraintManager 和具体 measurement model 决定。

**配置与验收**：状态阈值、窗长、进入/退出驻留时长、证据源开关、时间容差、数据陈旧时限、RTK 质量门限和状态输出频率通过 `config/navigation/dynamic_status.json` 配置。ZUPT 观测噪声、准入置信度/驻留时长、NIS 门限、协方差膨胀、更新频率/相关性处理、参考速度定义及 GNSS 缺失策略通过 `config/kalman/zupt.json` 配置。文档不固化未经校准的数值；参数须按 IMU 噪声、车辆行为和标注数据标定。验收数据应覆盖静止、起步、匀速直线、正/负纵向加速度、制动、转弯、粗糙路面、GNSS 失锁/跳变、传感器时间间断和静止/运动转换；报告状态混淆矩阵、各类 precision/recall、状态延迟、ZUPT 误触发/漏触发、更新前后速度误差与协方差一致性、NIS 统计、静止期间重复更新的信息过计量，以及未知率。只有通过数据集评估的 detector 和约束准入策略才允许自动更新。

## 6. 数值与实现规范

- C++17（建议）与 Eigen 3；Eigen 只在内部数学层使用，公开数据类型尽量使用自有结构，方便 ABI 和序列化控制。
- 当前 LTC 状态模型使用 `Eigen::Matrix<double,18,18>` 表达含杆臂估计的固定维矩阵；完整 `P` 和 `H` 使用由已选 state model 决定的维数。RTK 模糊度不属于该融合状态。集中定义索引、维数和单位常量，禁止散落的魔法数字。
- EKF 不显式计算矩阵逆；求解失败、NaN/Inf、协方差非对称或显著非正定时返回结构化错误。
- 四元数保持单位化；所有坐标变换函数命名体现方向（如 `C_e_n`），以坐标基底测试验证正交性和往返转换。
- 核心算法不直接写文件、不访问设备、不依赖 ROS。外层适配器可独立接入 ROS/串口/回放工具。
- 所有阈值、噪声、频点与周跳参数通过配置对象传入；配置需校验、记录版本并支持完整复现。

## 7. 建议 C++ 目录和 API

```text
include/ins_rtk/
  types.hpp                 # 时间、观测、状态、错误和配置结构
  coordinate_frames.hpp     # ECEF/LLA/NED 变换、地球模型
  mechanization.hpp         # StrapdownMechanizer
  error_state_model.hpp     # F/G/Phi/Qd
  td_ddcp_model.hpp         # 相邻历元 TD-DDCP 几何、H、R
  carrier_phase_continuity.hpp # 相位缓存、周跳、pivot/历元配对
  temporal_correlation.hpp  # 跨历元状态与量测相关性
  filter.hpp                # 主控接口
src/                         # 对应实现
config/                      # 噪声与运行参数示例
```

建议接口（签名为设计草案）：

```cpp
struct Status { bool ok; ErrorCode code; std::string message; };
struct ImuSample { Time timestamp; Eigen::Vector3d gyro_radps; Eigen::Vector3d specific_force_mps2; };
struct RtkSolution {
  Time timestamp;
  Eigen::Vector3d antenna_position_ecef_m;
  Eigen::Vector3d antenna_velocity_ecef_mps;
  Eigen::Matrix<double, 6, 6> covariance;
  std::bitset<6> valid_components; // position/velocity availability mask
  RtkSolutionStatus status; // FIX / FLOAT / DGPS / INVALID
  std::size_t used_satellites;
};
struct PreparedTdDdCarrierPhaseEpoch {
  Time previous_timestamp;
  Time timestamp;
  std::vector<TimeDifferencedDoubleDifferenceCarrierPhase> observations; // metres, common sat/pivot identity
  Eigen::MatrixXd covariance;
  bool reference_epoch_valid;
};
using MeasurementObservation = std::variant<RtkPositionVelocityObservation,
                                            PreparedTdDdCarrierPhaseEpoch,
                                            ConstraintObservation>;
struct NavigationMeasurementBundle {
  Time timestamp;
  std::vector<MeasurementObservation> observations; // typed variants selected by configured model IDs
  CorrelationMetadata correlation; // cross-model and temporal covariance provenance
};
struct InertialStateSnapshot { Time timestamp; std::uint64_t epoch_id; NominalNavState state; };
struct ErrorStateCorrection { Eigen::Vector3d d_position_ned_m, d_velocity_ned_mps, d_attitude_ned_rad; Eigen::Vector3d d_gyro_bias_body_radps, d_accel_bias_body_mps2, d_lever_arm_body_m; };
struct ErrorStateFeedback { Time timestamp; std::uint64_t epoch_id; ErrorStateCorrection correction; };
struct FeedbackApplied { Time timestamp; std::uint64_t epoch_id; bool accepted; };
struct NavigationSolution { Time timestamp; std::uint64_t epoch_id; NominalNavState corrected_state; SolutionStatus status; EstimatorSummary estimator; };

class InertialNavigationModule : public IModule {
public:
  Status onImu(const ImuSample&, MessageContext&); // owns and propagates nominal state
  InertialStateSnapshot predictedState() const;
  FeedbackApplied applyFeedback(const ErrorStateFeedback&);
  NavigationSolution correctedSolution() const;
};

class EstimationModule : public IModule {
public:
  Status onImu(const ImuInterval&, const InertialStateSnapshot&); // process model builds Phi/Qd; delegates numeric prediction
  Status process(const NavigationMeasurementBundle&, const InertialStateSnapshot&); // measurement model builds innovation/H/R
  ErrorStateFeedback makeFeedback() const; // maps generic state correction to INS feedback
  void commitFeedback(const FeedbackApplied&); // delegates dx reset and covariance reset Jacobian
};

class StateEstimator {
public:
  void initialize(const Eigen::VectorXd& x, const Eigen::MatrixXd& P);
  PredictionResult predict(const Eigen::MatrixXd& Phi, const Eigen::MatrixXd& Qd);
  UpdateResult update(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H,
                      const Eigen::MatrixXd& R, const UpdateOptions&);
  void resetErrorState(const Eigen::MatrixXd& resetJacobian);
  const Eigen::VectorXd& state() const; // generic vector; no INS/GNSS semantics
  const Eigen::MatrixXd& covariance() const;
};
```

`EstimationModule` 内部组合通用 `StateEstimator`、`StateModelRegistry`、`ProcessModelRegistry` 和 `MeasurementModelRegistry`，按 JSON 装配状态布局、误差传播和 LC/TC/LTC 领域观测模型。量测模型接收 `NavigationMeasurementBundle` 与对应 INS 快照，执行物理预测、线性化及相关性处理，输出通用 innovation/`H/R`；过程模型输出通用 `Phi/Qd`。这些矩阵交由 `StateEstimator` 进行预测/更新。StateEstimator 只管理一般向量/矩阵及数值诊断，不含模型注册、消息解析或导航语义。名义 INS 状态由 `InertialNavigationModule` 持有。一个估计实例内的量测模型共享该实例的 StateEstimator 会话及领域历史，所有名义误差注入都通过有 epoch/time 校验的反馈事务完成。

调用约定：每个 `InertialNavigationModule + EstimationModule` 配对由一个事件调度线程串行调用；跨线程数据经队列传递。快照均为不可变拷贝。反馈拒绝、epoch 不匹配或超时必须使该历元更新回滚/失败，禁止只更新 nominal state 或只更新 `dx/P`。

## 8. 时间同步、延迟与回放

INS 状态必须可在观测时间戳处重建。首期实现可通过 IMU 环形缓存和状态快照支持有限时长回滚：若某个 `NavigationMeasurementBundle` 晚到且仍在缓存范围内，恢复该历元前状态、传播至观测时刻、按启用的模型联合/去相关更新，再重放缓存 IMU；超出缓存则拒绝该历元并记录原因。RTK solution 和原始差分相位须按 GNSS 观测时间戳关联，不能因解算输出延迟而错配历元，也不能直接将旧观测应用于当前时刻状态。IMU/GNSS 时间基准、天线观测历元定义、设备延迟及插值策略通过配置明确。

## 9. 诊断和可观测性

结构化日志至少记录输入时间与延迟、dt、IMU 更新次数、RTK 解状态/卫星数/位置速度协方差、P/V 与 TD-DDCP 各自创新和标准化残差、TD-DDCP 前后时间戳/间隔/共视星数/pivot/失效原因、DynamicStatus 状态/置信分数/状态驻留时长/各证据组有效性及状态切换原因、约束候选与门控拒绝原因、联合创新协方差及跨时交叉协方差来源、RTK 引擎 FIX/FLOAT/ratio 状态、周跳事件、固定 18 维状态、反馈量、协方差健康指标及错误码。日志可关闭高频逐样本输出，但需支持可复现实验的原始观测回放。

## 10. 配置项

| 配置组 | 主要参数 |
|---|---|
| IMU | 采样率、单位/轴序、ARW、VRW、bias 驱动噪声、相关时间、初始 bias 与协方差、比例因子开关 |
| 杆臂在线估计 | 初始杆臂（默认 0 m）、初始协方差、随机游走噪声、估计开关/状态布局版本 |
| 地球模型 | WGS-84 参数、正常重力模型 |
| RTK 解算 | 引擎选择、RTK P/V 状态映射、坐标框架、FIX/FLOAT 有效条件、位置速度协方差来源、输出延迟策略 |
| TD-DDCP 紧组合 | 频点/波长、最大相邻历元间隔、最小共视星数、pivot 连续策略、相位噪声/高度角模型、周跳检测与配对重置、基站坐标 |
| 混合量测相关性 | RTK P/V 与当前/前一历元原始相位交叉协方差、跨时状态协方差、联合/去相关方法、信息不可得时的安全降级策略 |
| RTK 引擎模糊度 | 仅 RTK 引擎使用：固定策略、ratio 门限、整数搜索选项、失锁超时；不定义融合 ESKF 状态 |
| 动态状态检测 | 滑窗时长/采样数、IMU/INS/RTK/TDCP 证据开关、源时间容差/陈旧时限、静止/匀速/加减速/转弯阈值、进入/退出迟滞、最短驻留时间、置信评分权重与冲突策略 |
| 运动约束 | NHC 噪声、车辆轴向/安装角、High Dynamic/Slip 抑制策略；ZUPT 噪声 `R_zupt`、静止置信度/驻留时间、NIS 门限、参考点速度定义；ZPC/ZHC 启用开关、锚点创建/复位/超时、`R_zpc`/`R_zhc`、锚点互协方差及更新频率/相关性处理 |
| 时间 | GNSS/IMU 时间偏移、最大可接受间隔、回滚缓存时长、延迟策略 |
| 数值 | 创新门限、协方差容差、矩阵分解失败策略 |

所有参数应有单位、默认值来源、允许范围与配置版本。报告中的周跳门限和 RTK 固定门限仅为候选值，不作为量产默认值，须依据设备与数据验证。

## 11. 验收与验证计划

此处定义后续验证要求，本次未执行测试。实现阶段至少建立：

1. 坐标变换单元验证：LLA↔ECEF 往返、NED/ECEF 旋转正交、FRD/NED 轴向与已知姿态用例。
2. 机械编排用例：静止水平、恒速直线、恒定转弯、地球自转、增量切分一致性、首样本和缺样处理。
3. 误差模型数值验证：有限差分核对 `F/H`，连续到离散协方差量纲检查，协方差对称半正定。
4. EKF 用例：无观测传播、RTK P/V 松组合更新、TD-DDCP 紧组合更新、联合量测及跨历元状态/量测交叉协方差、异常残差门限、闭环重置前后等价性。
5. TD-DDCP 用例：首历元只缓存、相邻历元时间差观测与几何预测一致、无周跳时模糊度抵消、周跳/缺星/间隔超限/pivot 改变时配对失效并重置参考、共享 pivot 的行间协方差、保留参考状态不确定度的更新一致性。RTK engine 自身另验证整数固定和 ratio 拒绝。
6. 时间用例：延迟历元回滚重放与按时历元结果一致；超出缓存的行为确定。
7. 数据集回放：与参考解对齐，评估位置/姿态误差、固定率、错误固定率、重捕获时间、创新统计和运行时延。
8. `StateEstimator` 通用数值用例：用与导航领域无关的已知线性系统验证状态/协方差预测、Kalman 更新、创新协方差、NIS、奇异/非正定输入处理及 reset Jacobian；通过不同状态维数确认内核不绑定 18 维 INS。该层使用合成矩阵，不加载 RTK/GNSS/IMU 模型。
9. DynamicStatus/ZUPT/ZPC/ZHC/NHC 用例：静止、起步、匀速直线、正负纵向加速、制动、恒速转弯、粗糙路面、IMU/GNSS 缺失与异常、时间戳过期、证据冲突、状态迟滞和约束门控；检查分类指标、误触发和延迟。静止真值下验证各约束 innovation/H 符号、velocity correction、ZPC 锚点误差、协方差 reset、R/NIS、跨历元相关性与更新频率；运动、unknown 或 ZUPT 静止触发撤销时必须关闭 ZPC/ZHC；非零杆臂/旋转参考点用例验证禁止错误零速/零位移/零航向变化。ZHC 验收须量化检查锁定期间输出 heading 与 `heading_lock_rad` 一致、heading-lock 1σ 与锁定快照一致、自由 ESKF covariance 单独记录、反馈和其他量测不能改变锁定输出、退出后按策略释放；同时验证 PSD 和 roll/pitch 连续性。
10. NHC 数值与物理验证：用已知 FRD 直行/加减速轨迹验证前向速度不被 NHC 拉向零、侧向/垂向残差和更新方向正确；对姿态、速度、陀螺零偏和可估计杆臂列逐列中心有限差分验证 H；验证静态安装角、正反向行驶、不同转弯率的杆臂补偿、非地面相对速度输入的坐标转换；在侧滑/急转/颠簸/unknown 时检查拒绝或协方差膨胀，验证 `R_nhc` 正定、NIS 门限和更新频率不造成重复信息过度累计，并与 GNSS/TDCP 速度联合时检查相关性处理。
11. 工程质量：编译警告、ASan/UBSan（平台支持时）、确定性回放、配置和日志可复现。

## 12. 实现前必须确认的事项

以下事项在输入材料中存在信息不足或数学约定需要核验。已完成的实现阶段不代表这些数学/数据约定已经验证：

1. **误差符号和量测雅可比符号**：报告将位置误差写为 `δr = r_hat - r_true`，同时将创新写为“观测减预测”，但给出的伪距雅可比为 `-u^T`。按通常 `u` 定义为接收机指向卫星的视线向量，几何距离导数与该组合可能符号不一致。必须统一 `dx` 定义、RTK P/V 与载波相位创新的 `z` 定义、`H` 的线性化点和反馈符号，并用有限差分确认。
2. **姿态误差定义**：明确 `C_hat = (I - [phi×]) C_true` 下四元数注入方向及滤波误差含义；报告和指南需统一，不能仅复制其中一段代码。
3. **杆臂雅可比**：验证 NED/ECEF 转换方向、`C_bn` 下标约定及 `H_rho [l^n×]` 的符号，以及新增 body-frame 杆臂误差列对天线位置/双差距离的雅可比；与几何距离数值差分对照。
4. **相位信号与频点映射**：多频/多星座各自波长、相位字段与信号映射、频间偏差及相位连续性判据需明确。TD-DDCP 融合模型不估计模糊度；RTK engine 的模糊度单位及 pivot 变换按其 adapter/引擎定义。
5. **时间差分配对规则**：定义相邻历元最大间隔、共视卫星门限、pivot 连续/切换时的重配对策略、周跳标记来源及当前历元重新作为参考的规则。
6. **GNSS 原始输入定义**：数据为非标准 RINEX 文本；RTK engine 用 rover/base 原始观测生成 RTK P/V 解，融合滤波器另由本软件按 rover/base 单差及卫星间双差构造载波相位量测。base 文件提供卫星位置，缺星忽略。SatX/Y/Z 已确认为 ECEF 米，SatElev 已确认为 degree；需要时使用 base 文件中的 SatClk/SatElev。SatClk 已确认单位为米；用户确认文本以空格分列，数值 `0` 表示该字段缺失。解析时按列标记缺失，不丢弃同一行其他有效字段。
7. **闭环重置协方差**：反馈后 18 维（含杆臂估计）误差均值归零，还须按误差坐标变换应用 reset Jacobian 更新 `P`；TD-DDCP 参考历元的联合状态/交叉协方差也须同步变换或重置。
8. **IMU 数据语义**：已确认表头单位、body frame 与 vehicle FRD 重合、加速度是含重力响应的比力、时间戳是采样时刻。文件值为按时间采样的角速度和比力，传播时需利用相邻采样时间积分为增量；首样本、异常采样间隔和设备内部补偿仍应由数据说明/配置处理。
9. **NHC/ZUPT/静止锚点参数与语义**：已将理论报告中的静止检测、零速伪观测、NHC 车体系侧向/垂向约束及动态状态门控纳入设计，并增加静止锚点相对位置/heading 变化约束。实现前仍需用具体 IMU 噪声/静态标注数据标定 stationarity index、进入/退出阈值、驻留时间、`R_zupt`、`R_zpc` 与 `R_zhc`；明确机械编排速度参考系、ZUPT 所约束的物理参考点、静止锚点创建/复位策略、锚点和当前状态的互协方差维护、杆臂/车辆安装角以及连续约束相关性/更新频率，并按第 11 节数值验证。理论报告不提供可直接移植的通用阈值或现场检测器验证结果。
10. **目标运行环境**：确认 C++ 标准、Eigen 版本、操作系统/CPU、ROS 需求、实时性预算、输入协议与输出接口。
11. **测试数据与指标**：提供 IMU/GNSS 型号、噪声标定、基站数据、真值或参考轨迹，以及固定率、误差、延迟等验收门限。

### 12.1 RTKLIB adapter 实现状态（2026-10-23）

- 已在 `ins_rtk_nav` 构建 RTKLIB 2.4.2 p13 相对定位核心；原始上游目录保持不变，通过 CMake 源文件引用并链接 Windows `winmm`。
- 当前 adapter 将 base 文件的逐历元 SatX/Y/Z 和 SatClk 转换为 RTKLIB 精密星历缓存。RTKLIB 的 `peph2pos` 需要连续 11 个采样点，因此在单颗卫星短暂缺行时对该卫星的临近状态作线性插值；该插值只用于星历插值输入，缺失观测仍按用户规则忽略。
- SatClk 米转换为秒，并同时填入临时广播钟差项，因为 RTKLIB `satposs()` 在读取精密星历前会先请求广播钟差。GPS L1/L2 波长显式填入 `nav.lam`。rover/base 使用 C1、L1、L2、D1，P2 不使用。
- 当前配置把 SVID 解释为 GPS PRN。该解释仍是待确认的数据假设；其他星座/编码不得直接复用当前 adapter。
- 已读取 rover 初始 ECEF 作为 RTKLIB 单点初始位置，并将已知 base ECEF 设置为静态参考站。
- RTK position covariance 和 velocity covariance 取 RTKLIB filter state covariance；仅当分量方差为正、有限且低于配置门限时标记为有效。有效 FLOAT 和 FIX 都发布为 `navigation.rtk`，由 CSV sink 输出；本次数据回放得到 1,564/1,564 历元的 FLOAT 解，未出现 FIX。
- 当前实现为离线批处理：RTK module 缓存全部 GNSS 历元，在 `DatasetReplayComplete` 后构建 11 历元插值上下文并顺序求解。因此现阶段 CSV 可用，但解算结果没有在数据源回放期间实时送入 FilterManager。后续 LTC 实现须采用整段预解算再时间合并，或实现有前视窗口的卫星状态缓存，不能将延迟到达的旧 RTK 量测直接用于当前 INS 状态。
- 该结果证明 adapter 端到端可运行，不等于准确度/模糊度固定率验收。当前精密星历由接收机数据文件给出的坐标构建，固定率、位置误差、速度质量仍需真值或独立参考轨迹验证。

### 12.2 LC/TC/LTC 分层融合实现与全数据评估（2026-10-25）

- 首版已按本设计拆分 `FilterManagerModule`、`InertialNavigationModule`、`EstimationModule` 和底层通用 `StateEstimator`。FilterManager 做时间匹配并产生标准量测包；INS 模块独占名义 mechanization 与误差反馈；EstimationModule 构造 INS 过程和 LC/TC/LTC 领域模型；StateEstimator 仅执行通用向量/协方差预测、更新、NIS 门控和 reset 运算。
- 三种模式由 `config/mode_lc.json`、`mode_tc.json`、`mode_ltc.json` 选择 measurement model ID。数值内核为通用 18 维 ESKF 布局：NED 位置/速度/姿态误差、陀螺/加速度计 bias 误差、body-frame 杆臂误差。为了让模块化重构不改变既有算法结果，当前正式配置 `estimate_lever_arm=false`，杆臂初值为零且杆臂 Jacobian 列被屏蔽；在线估计作为显式配置选项，暂不默认打开。
- 全数据离线回放：每种模式均输出 158,328 个 IMU 历元；输入 1,564 个 RTK 历元及 1,563 个相邻历元 TD-DDCP 集。LC 接受 1,564 个 RTK P/V 更新历元；TC 使用 1,563 个 TD 更新历元、26,399 条卫星量测行；LTC 接受 1,564 个 RTK 位置更新历元和 1,557 个 L1 TDCP 派生速度更新历元，6 个速度历元因 WLS 几何/有效行条件未接受。完整统计见 `output/LC_TC_LTC_Performance_Report.md` 和配套 PNG。
- 修复默认模型回归后，最终滤波协方差位置 3D 1σ 为 LC 0.03176 m、TC 0.05735 m、LTC 0.03075 m，与重构前已记录的 0.03174/0.05717/0.03061 m 接近；该量为内部不确定度，不是真实误差。逐历元 RTK-relative 的 3D RMS 为 LC 0.0219 m、TC 21.8938 m、LTC 0.0149 m。RTK-relative 不是独立真值检验。
- 回归原因：首版重构意外同时启用了杆臂状态更新和 NIS 拒绝门控，并把 `minimum_measurement_variance` 从方差下限改成对所有量测方差直接相加。在线杆臂更新还存在姿态和 gyro-bias Jacobian 符号错误。已改为门控阈值 0 表示关闭、恢复方差下限及原观测模型方差增量语义、默认关闭杆臂估计，并修正杆臂相关 Jacobian 符号。隔离回放验证了这些更改恢复 LC/LTC 更新数和位置协方差。
- TC 的 NIS/dof 目前为 0.050，旧版记录为 0.048；TC 对 RTK 的 21.89 m 差值在关闭杆臂后仍存在。TD-DDCP 主要约束历元间几何变化，不提供强的绝对位置基准；初始位置偏差可观测性不足是既有模型问题，不应归咎于模块化重构。LTC 也未保留完整跨历元状态交叉协方差，RTK 与原始载波互相关也未建模。
- 当前数据无独立 rover 位置、速度或姿态真值。不能据此报告真实导航 RMSE、厘米级精度或算法绝对性能排名。下一轮性能验收需要外部真值/测量基准；TC 还需补充绝对位置初始化/约束并处理创新相关性。

### 12.3 DynamicStatus、ZUPT 与 NHC 首版实现及对照（2026-10-25）

- 增加 `DynamicStatusModule`（滑窗 IMU 角速率/比力范数变化、有效 RTK 速度、驻留迟滞）、`ConstraintManagerModule`（静止时准入 ZUPT、正常运动时准入 NHC 并限制更新频率）及 `EstimationModule` 中对应的量测构造。ZUPT 在 NED 约束三轴速度；NHC 在 FRD 约束 right/down 两轴速度，前向速度不受约束。
- 新增 `ins_rtk_nav/config/mode_lc_constraints.json`、`mode_tc_constraints.json`、`mode_ltc_constraints.json`。旧 mode 配置不包含约束模型 ID，可用于无约束基线。动态检测门限、dwell、约束非零标准差、NIS 门限和最大更新间隔放在 `fusion_model.dynamic_status` 与 `fusion_model.constraints`。
- 同一数据全量回放结果保存在 `output/zupt_nhc_baseline/` 与 `ins_rtk_nav/build/mode_*_constraints/`；可用 `ins_rtk_nav/tools/analyze_zupt_nhc.py` 重建 `output/ZUPT_NHC_Performance_Report.md` 和 PNG。每模式生成 846–847 个 ZUPT 历元（接受 844–845），以及 538 个 NHC 历元（接受 536–538）。
- 约束配置的最终位置 3D 1σ 为 LC 0.03038 m、TC 0.04118 m、LTC 0.02959 m；速度 3D 1σ 为 0.05939、0.07520、0.04475 m/s；姿态 3D 1σ 为 5.176、5.185、5.173 deg。内部协方差大多收缩，不表示真实误差下降。TC 对 RTK 的 3D RMS 差从 21.89 m 降到 2.00 m，仍是共享 GNSS 来源的相对诊断，不是真值。
- ZUPT/NHC normalized innovation RMS 与 NIS/dof 明显小于 1（见报告），提示当前模型仍需校准并检查伪量测相关性。当前以 1 Hz 限频，但未建模约束与 GNSS/TDCP 及跨历元伪量测的交叉相关；检测门限和 `R` 是当前数据的初始调试值，不是通用车规参数。首版 NHC 假定车辆 FRD 与 IMU FRD 对齐且参考点与 INS 速度参考点重合；非零安装旋转/参考点杆臂尚未接入该更新。
- 当前数据无独立位置、速度或姿态真值，本次只报告滤波协方差、创新和 RTK-relative consistency；不能据此断言定位精度提高或达到厘米级。

### 12.4 ZPC / ZHC 锚点约束实现及全量回放（2026-10-25）

> 本节记录最初增广锚点实现及首轮评估；其 ZPC→ZHC 联动和“相对锚点”语义已由 12.5 节修订，不能当作当前目标设计。

- 新增 `zero_position_change_v1` 与 `zero_heading_change_v1` measurement models，以及独立的 `mode_*_anchor_constraints.json`。旧 baseline 和 ZUPT/NHC 结果保留在原目录；新回放输出到 `build/mode_*_anchor_constraints/`。
- 通过 DynamicStatus 确认 stationary 且 ZUPT 更新通过后建立静止锚点。滤波器在锚点有效期间从 18 维扩展为 22 维：额外 3 维为锚点 ECEF 位置误差，1 维为锚点航向误差；锚点克隆与当前状态联合传播/更新。离开 stationary 或进入 unknown 时将锚点边缘化并回到 18 维。ZPC 使用锚点固定 NED 投影的 ECEF 位置差；ZHC 使用 wrap 后的锚点相对 heading 差。
- 准入顺序为 ZUPT → ZPC → ZHC：ZUPT 被拒绝时不执行 ZPC/ZHC；ZPC 启用时，ZPC 被 NIS 门限拒绝也跳过同历元 ZHC。这一联动由回放发现的错误静止触发风险驱动，避免运动时只用 ZHC 锁定旧航向。NHC 仍单独按 moving 状态和自身 NIS gate 处理。
- 噪声初值为 `zero_position_change_sigma_m=0.05`、`zero_heading_change_sigma_deg=0.5`，沿用约束 NIS gate 16.27；它们是当前数据的实验初值。
- 三种模式各完成 158,328 个 IMU 样本全量回放，输出 1,569 个公共 1 Hz solution 历元。`output/ZPC_ZHC_Performance_Report.md` 和 PNG 汇总 baseline、ZUPT/NHC、ZUPT/NHC+ZPC/ZHC 及创新统计。新增交互页面可用 `python regression_analysis_tool/prototype/app.py --zupt-nhc-compare --port 8766` 查看。
- ZPC 历元：LC 接受 509/840、LTC 510/841、TC 840/840；LC/LTC 的大量 NIS 拒绝表明 detector 标成 stationary 时，当前位置有时与锚点不一致，应先核验动态误判和参考点。ZHC 只在 ZUPT 与 ZPC 准入后执行。TC 的 ZPC 虽均通过，TC 的 RTK-relative 位置 RMS 从 ZUPT/NHC 的约 2.00 m 升至约 2.87 m，仍提示其位置绝对基准和静止判决必须继续检查。
- +ZPC/ZHC 的航向协方差 median 明显下降，但这不是独立真值精度证明；TC NHC 的创新仍偏大且有门限拒绝。报告标记了这些限制。ZPC/ZHC 目前是可试验的模型首版，不应作为已验证车辆约束直接用于生产结论。

### 12.5 ZHC heading-lock 设计复核（2026-10-25，历史问题与方案）

- 用户明确的目标改为：ZHC 生效时，输出 heading 与锁定瞬间相同，输出 heading-lock 1σ 也维持锁定瞬间的值；该锁定不代表一个新的绝对航向传感器观测，不应反复压低 RMS。
- 对原始回放日志和实现的复核发现：GPS 63430–63750 s 内 LC/LTC 的 319 个 ZUPT 历元通过、319 个 ZPC 历元均被 NIS 拒绝，因此旧代码因 same-epoch ZPC gate 抑制了 ZHC。现代码已把 ZPC/ZHC NIS 判断解耦；本次构建及三模式全量回放后，该段 ZHC 在 LC/TC/LTC 均为 319/319 接受，innovation RMS 约 0.0044 deg。
- 解耦后仍有更深层问题：GPS 63940–64320 s 内 LC 的 372/372 个 ZHC 都被接受，ZHC innovation RMS 约 0.0207 deg、最大约 0.0982 deg，但输出 quaternion heading 变化约 53.6 deg。源码检查表明 `onFeedback` 会把 anchor-heading error correction 写回 `anchorHeadingRad_`，其它量测可令当前 heading 和锚点共同漂移；所以小 ZHC innovation 不表示输出 heading 恒定。
- 本版第 5.7.3 节据此将功能改为显式 `HeadingLockState`：静止确认后捕获不可变 heading 与 heading-lock sigma，反馈/传播后锁回该值；输出保持锁定 sigma，同时单独记录自由 ESKF heading covariance，明确两者语义。进入/退出须有迟滞、残差和真实转动检查，协方差处理不得破坏完整矩阵 PSD，也不得重复计入同一静止证据。
- 当时尚未实现的内容已在 12.6 完成；本节保留为历史设计复核。

### 12.6 ZHC heading-lock 实现与性能评估（2026-10-25）

- `InertialNavigationModule` 订阅 `navigation.heading_lock.control`。锁定激活时捕获固定参考 LLA、heading 与 sigma；每次 INS mechanization 和滤波 feedback 后，将 body-to-NED 姿态投影到固定参考系下的锁定 heading，同时保持 roll/pitch 分量连续。锁定释放后回到普通 ESKF 姿态输出。
- `EstimationModule` 在 ZHC 首次准入后创建专用 lock snapshot；随后记录 `zhc_lock_projection` 残差，不把每个历元对同一静止事实当成独立 Kalman 观测。输出包含锁定 heading、lock ID、投影修正量、锁定 1σ 和单独的 `free_heading_rms_1sigma_deg`。锁定 1σ 表示捕获时形式不确定度，不能与自由滤波器当前边缘协方差混淆。
- CSV 新增 `free_heading_rms_1sigma_deg`、`heading_lock_active`、`heading_lock_id`、`heading_lock_heading_deg`、`heading_lock_applied_correction_deg`。三模式 Release 构建通过，并完成相同 158,328 个 IMU 样本全量回放，每种模式产生 1,569 个 1 Hz solution 历元和 846 个 lock-active 历元/4 个 episodes。
- 锁定 heading 最大偏差 LC/TC/LTC 分别为 0.00000361°/0.00000424°/0.00000361°；每个 episode 内锁定 1σ 的最大跨度均为 0°。锁定 sigma median/P95 分别为 LC 1.4287/29.9182°、TC 1.4356/29.9182°、LTC 1.4297/29.9182°。首段锁定值约 29.9182°，使 P95 较大；保持 RMS 不变不意味着该估计准确。
- 全时段 heading 报告 1σ median/P95/final：LC 0.9340/29.9182/0.7402°，TC 0.9499/29.9182/0.7457°，LTC 0.9369/29.9182/0.7387°。自由 ESKF heading 1σ median/P95/final：LC 2.4017/66.6215/5.1597°、TC 2.4087/71.8679/5.1679°、LTC 2.3994/57.4955/5.1582°。这表明显示层的 lock sigma 与滤波器自由边缘不确定度被明确区分。
- 同次回放位置 3D 1σ median/final：LC 0.0309/0.0296 m、TC 0.0390/0.0363 m、LTC 0.0296/0.0279 m；速度 3D 1σ median/final：LC 0.0601/0.0591 m/s、TC 0.0776/0.0733 m/s、LTC 0.0474/0.0443 m/s。上述指标均为协方差 1σ，不是独立真值 RMSE；相对 RTK 仅为内部一致性诊断。
- 评估结论：输出 heading 和报告的 lock sigma 在锁定期间达到设计的不变量；自由 ESKF 协方差仍可增长，这是刻意保留的诊断量。当前最大风险是首个锁定 episode 捕获了约 29.9° 的 heading 不确定度，以及锁定是否错误覆盖真实转动尚待真值/人工状态标签确认。ZUPT/ZPC/ZHC/NHC 与 GNSS/TDCP 的统计互相关仍未完整建模，不能将协方差统计解读为已验证的真实定位精度。
- 报告与图：`output/ZPC_ZHC_Performance_Report.md`、`output/ZPC_ZHC_Performance.png`、`output/ZHC_HeadingLock_Performance.png`。重新运行 `python ins_rtk_nav/tools/analyze_zpc_zhc.py` 可重建评估产物。

## 13. 依据与限制

本设计的坐标系、18 维惯导/杆臂误差状态、闭环 ESKF、差分载波相位、杆臂、周跳、LAMBDA 和模块方向来自用户提供的传统 INS/RTK 设计报告与软件设计指南；ZUPT/NHC 理论、零速/非完整约束观测和动态状态门控参考用户提供的 `Theoretical Foundations of ZUPTand NHC.pdf`。Zero_Position_Change 和 Zero_Heading_Change 是用户本轮提出的静止锚点相对约束扩展，不声称由该理论报告原文给出。RTK P/V 松组合与差分载波相位紧组合的混合架构按用户决策更新。理论材料用于建立设计依据，不代表其中公式符号、候选检测统计量或参数已经针对当前传感器验证；输入代码段也不作为经过编译或数值验证的实现。量测相关性、误差约定和未提供的传感器参数仍须在实现和验证中锁定。

## 14. 工作目录现状与数据接入设计

本次只读检查到当前工作目录已有以下内容；未修改这些目录：

| 路径 | 观察结果 | 设计影响 |
|---|---|---|
| `eigen/` | Eigen 3.4.0 头文件源码树 | CMake 以本地源码目录作为 `Eigen3::Eigen` 依赖；避免把 Eigen 实现复制进业务模块 |
| `RTKLIB-master/` | RTKLIB 2.4.2，头文件标记 patch level p13；含 `src/`、`lib/`、`brd/`、`data/`、`test/` | 通过单独的 `rtklib_adapter` 封装 C API；对外提供 RTK 位置/速度、状态和协方差量测；核心融合算法不直接依赖 RTKLIB 类型 |
| `data1/imu/IMU_100Hz.txt` | 约 158,329 行；用户确认按文件 header 单位使用，IMU body frame 与 vehicle frame 重合；加速度为含重力响应的比力；表头示例单位为 m/s² 和 rad/s | 使用文本 IMU 回放适配器；车辆/IMU 轴向重合；header 的 x/y/z 列按 vehicle FRD 的前/右/下轴映射；IMU 时间戳按采样时刻解释 |
| `data1/GNSS/rover/rover.txt` | `gpssec svid C1 L1 D1 P2 L2 SatX SatY SatZ SatClk SatElev`，约 15,412 行 | 建立 rover 文本观测解析器；示例中部分卫星位置为零，不能无条件信任文件里的卫星位置 |
| `data1/GNSS/base/base.txt` | 同类列结构，约 16,491 行 | 建立 base 文本观测解析器；和 rover 按 GPS 时间（1e-6 s 容差）、SVID、信号做历元匹配；base 缺失卫星时忽略该星 |
| `data1/GNSS/base/coordinates.txt` | 已确认：基站天线相位中心 ECEF 坐标，单位 m | 配置为 base 天线相位中心 ECEF 坐标 |
| `data1/GNSS/rover/coordinates.txt` | 已确认：ECEF 坐标，单位 m；rover GNSS 天线相位中心初始位置 | 作为 GNSS 天线中心初始位置；经杆臂关系换算为 IMU 中心初值 |

用户确认当前 GNSS 文本不是标准 RINEX，文件已包含解算得到的卫星位置，且 rover 文件位置不完整。卫星位置按 GPS 时间历元以 1e-6 s 容差匹配，再按 SVID 从 base 文件读取，作为同历元 rover/base 共视观测的公共卫星状态；不得取 rover 中无效/缺失位置替代。若 base 对应历元没有该卫星或位置无效，则忽略该卫星，不执行星历回退。SatX/Y/Z 已确认是 ECEF 米；SatElev 已确认单位为 degree。需要时 SatClk/SatElev 从 base 文件读取；SatClk 已确认单位为米；GNSS 文本以空格分列，数值 `0` 表示对应字段缺失。

输入文件格式由独立 `IDataSource`/`IImuSource`/`IGnssSource` 适配器实现。解析层须处理空行、表头、连续空格分隔、数值 `0` 缺失标记、非法数值和时间乱序；提供行号、列名与文件名诊断。GNSS 缺失按字段处理：例如伪距缺失不自动判定载波也缺失，只有满足对应观测模型所需字段的卫星才进入该类更新。`data1` 是当前回放数据集，不把绝对路径写死在源码或算法模块中。

## 15. 可插拔模块和模块间信息传递

目标是让不同导航解算算法能在同一运行框架中并行比较、按配置串接或替换。模块不直接互相持有具体实现指针；依赖稳定的接口、消息类型和运行时服务。

### 15.1 模块分类

- **Source 模块**：文件回放、实时 IMU/GNSS 接收器。
- **Adapter/Preprocess 模块**：单位/轴向变换、时间对齐、传感器校准、GNSS 观测组帧。
- **Algorithm 模块**：纯 INS、INS/RTK 松组合、INS/RTK 紧组合、INS/RTK 松紧组合及其他滤波器/因子图解算器；每个算法可有自己的状态和参数。
- **DynamicStatus 模块**：由 IMU/导航/GNSS 证据维护状态窗、发布状态和置信/诊断信息；不更新导航滤波器。
- **Constraint 模块**：消费 DynamicStatus，经独立准入策略生成 NHC、ZUPT、轮速、气压计等量测。
- **Output 模块**：轨迹、诊断、CSV/JSON、可视化或应用接口。

本项目目标运行模块拆为 `FilterManagerModule`、`InertialNavigationModule`、`EstimationModule`、`DynamicStatusModule` 和 `ConstraintManager`；`EstimationModule` 内组合领域相关的 `StateModelRegistry`、`ProcessModelRegistry`、`MeasurementModelRegistry` 与底层通用 `StateEstimator`。其他可复用算法组件包括 `StrapdownMechanizer`、`RtkSolutionAdapter`、`RtkPositionVelocityMeasurementModel`、`TimeDifferencedDoubleDifferenceCarrierPhaseModel`、`CarrierPhaseContinuityManager`、`TemporalMeasurementCorrelationManager` 与 `ConstraintMeasurementModel`。`StateEstimator` 只接收通用状态/矩阵量，不含任何上述领域模型；`ConfigurableNavigationFilter` 若保留，只能作为 EstimationModule 内协调对象，不能复制 StateEstimator 的数值滤波职责，也不能成为另一状态所有者或公共名义解输出模块。JSON 通过稳定的模型 ID 选择并组合已注册实现；RTK 的 ambiguity manager / LAMBDA 留在 RTK solution engine adapter 内部。新算法可注册新的观测/状态模型，不需要改动数据解析和通用数值内核。

### 15.2 核心接口契约

```cpp
class IModule {
public:
  virtual ~IModule() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual Status configure(const ConfigView&) = 0;
  virtual Status initialize(const InitialContext&) = 0;
  virtual Status start(ModuleContext&) = 0;
  virtual void stop() noexcept = 0;
};

class IInertialNavigationModule : public IModule {
public:
  virtual Status onImu(const ImuSample&, MessageContext&) = 0;
  virtual InertialStateSnapshot predictedState() const = 0;
  virtual FeedbackApplied applyFeedback(const ErrorStateFeedback&) = 0;
  virtual std::optional<NavigationSolution> latestCorrectedSolution() const = 0;
};

class IEstimationModule : public IModule {
public:
  virtual Status onImu(const ImuSample&, const InertialStateSnapshot&) = 0;
  virtual Status onMeasurementBundle(const NavigationMeasurementBundle&, const InertialStateSnapshot&) = 0;
  virtual ErrorStateFeedback makeFeedback() const = 0;
  virtual void commitFeedback(const FeedbackApplied&) = 0;
  virtual EstimatorSummary summary() const = 0;
};
```

代码示例表达契约，不限定最终 ABI。模块声明其订阅和发布的消息类型、所需传感器、运行频率、是否支持延迟数据和是否可重置。`ModuleRegistry` 负责工厂注册和按 `type` 创建实例；内置模块静态注册，未来动态插件需另行规定 ABI/版本边界，首期不要求跨 DLL 热加载。

### 15.3 消息总线

采用进程内、强类型消息总线，首期单线程确定性调度；回放与实时接收器可独立生产消息，但进入算法队列后按事件时间戳排序。消息包含 `timestamp`（传感器事件时间）、`receive_timestamp`（接收时间，可选）、`sequence_id`、`source_id`、`frame_id`、单位约定和质量标记。

核心消息至少包括：`ImuSample`、`GnssObservationEpoch`、`BaseStationInfo`、`RtkSolution`、`PreparedTdDdCarrierPhaseEpoch`、`NavigationMeasurementBundle`、`ConstraintObservation`、`DynamicStatus`、`InertialStateSnapshot`、`ErrorStateFeedback`、`FeedbackApplied`、`NavigationSolution`、`ModuleDiagnostic`、`SystemEvent`。`ErrorStateFeedback` 只携带这一历元待注入的改正量，不暴露滤波器内部完整 `dx/P`。消息由配置的组合模式选择：LTC 消费 RTK 位置和 L1 TDCP 并发布派生位移/区间速度观测；TC 消费卫星级 TD-DDCP 行；LC 消费 RTK P/V。TD-DDCP 消息显式包含前后时间戳、卫星/pivot 坐标和共享 pivot 协方差。模块向总线发布不可变值对象；订阅者不得保留指向生产者临时内存的裸指针。大块数组可用共享只读缓冲区，但生命周期由消息对象持有。

总线提供订阅/退订、顺序处理、异常隔离、队列容量和背压策略、消息追踪 ID。滤波器的单实例状态更新保持串行，以免时序重排。消息总线只负责传递事件，不代替 ESKF 内部的状态交叉协方差；若两个算法模块需要融合其估计结果，必须使用明确的融合算法模块，而非简单平均状态。

### 15.4 运行管线

配置文件指定 `modules` 实例列表和 `pipeline` 连接关系。IMU propagation → estimator update → error feedback → corrected solution 是同一 epoch 的受控事务，不视为任意消息总线反馈环；FilterManagerModule 对其串行调度并用 epoch_id 配对。启动时校验其余依赖图无环、消息类型兼容及该事务契约完整，然后按拓扑顺序 configure/initialize/start。停止时逆序 stop。算法切换策略（单模块选择、并行对比或级联）由管线配置表达，数据源可复用，算法模块互不覆盖输出。

## 16. JSON 多文件配置系统

所有运行参数、输入/输出文件、模块选择和数据路径均通过 JSON 管理。JSON 采用 UTF-8；配置错误应在运行前给出文件、JSON Pointer、实际值和允许范围。配置读取分为解析、合并、变量/路径解析、模式校验、语义校验和模块配置六步。不得静默接受未知关键字段；支持 `schema_version`，兼容性升级须有显式迁移。

### 16.1 文件拆分建议

```text
config/
  system.json                 # 全局运行方式、模块管线、日志、输出目录
  paths.json                  # 输入数据、星历、输出和缓存路径
  sensors/
    imu.json                  # IMU 轴序、单位、标定和噪声参数
    gnss.json                 # GNSS 信号、天线、接收机和观测质量参数
  navigation/
    ins.json                  # 初始状态、地球模型、杆臂与机械编排选项
    rtk_solution.json         # RTK 引擎及 P/V 输出、解状态和协方差
    fusion_model.json         # 组合模式、观测模型和状态模型选择/参数引用
    dynamic_status.json       # 动态状态特征窗、判别阈值、证据源和迟滞策略
  kalman/
    ins_error_state.json      # 初始协方差、过程噪声、相关时间
    rtk_measurement.json      # RTK P/V 协方差及差分载波噪声/高度角模型
    measurement_correlation.json # P/V 解与原始差分相位的交叉协方差
    nhc.json                  # NHC 参数
    zupt.json                 # ZUPT 量测噪声、准入、NIS 与重复更新策略
    zero_position_change.json # 静止段位置锚点/相对位移模型与协方差
    zero_heading_change.json  # 静止段 heading 锚点/角差模型与协方差
  profiles/
    offline_dataset1.json     # 某数据集的组合入口/覆盖项
```

### 16.2 配置合并和路径规则

入口参数仅指定一个 profile（例如 `--config config/profiles/offline_dataset1.json`）。profile 以 `include` 明确引用其他 JSON；建议按 system → paths/sensors → navigation/kalman → profile override 的顺序合并。同一键的覆盖由后加载配置生效，并在启动摘要中显示最终值和来源文件。禁止依赖当前工作目录猜路径：相对路径默认相对于**声明该路径的 JSON 文件所在目录**；可选支持 `${ENV_VAR}`，但仅允许显式白名单环境变量，并在最终配置摘要中显示解析后的路径。

配置路径与数据路径分开：`paths.json` 给出原始文件、星历、输出、日志、回滚缓存目录；算法 JSON 不重复写文件系统路径。允许命令行只覆盖 profile、输出目录、日志级别等少量运行开关，并把覆盖记录进运行元数据。

### 16.3 JSON 示例

```json
{
  "schema_version": 1,
  "include": [
    "../system.json",
    "../paths.json",
    "../sensors/imu.json",
    "../sensors/gnss.json",
    "../navigation/ins.json",
    "../navigation/rtk_solution.json",
    "../navigation/fusion_model.json",
    "../navigation/dynamic_status.json",
    "../kalman/ins_error_state.json",
    "../kalman/rtk_measurement.json",
    "../kalman/measurement_correlation.json",
    "../kalman/zupt.json",
    "../kalman/nhc.json",
    "../kalman/zero_position_change.json",
    "../kalman/zero_heading_change.json"
  ],
  "profile_name": "offline_dataset1",
  "overrides": {
    "paths.data_root": "../../data1",
    "modules.navigation.primary": "configurable_navigation_filter"
  }
}
```

配置实现需要定义上述 `overrides` 是 JSON Pointer/点路径还是完整 JSON Merge Patch；建议使用标准 JSON Pointer 或明确的嵌套对象合并，不应在不同配置文件里使用两套规则。各字段的真实 schema 由 `config/schema/` 下 JSON Schema 文件维护，并做单位标注（字段名后缀或旁注文档）。

### 16.4 建议的配置分层

- `system.json`：`runtime.mode`（offline/realtime）、`runtime.scheduler`、消息队列容量、模块实例、pipeline、日志等级和输出订阅；选定的导航实例引用 fusion model 配置。
- `imu.json`：输入字段映射、时间单位、采样值/增量语义、轴向映射、标定矩阵、ARW/VRW、bias 模型和温度模型开关。
- `gnss.json`：rover/base 字段映射、星座与信号映射、波长、仰角/信噪比门限、观测协方差配置、天线信息。
- `ins.json`：坐标系、初始位置/速度/姿态及来源、杆臂初值（默认 `[0,0,0] m`）、在线估计开关/先验协方差/随机游走噪声、地球模型、机械编排选择。
- `ins_error_state.json`：状态维数/版本、初始协方差（含杆臂 3×3 块及交叉项）、连续噪声谱密度、bias 相关时间和杆臂随机游走谱密度、离散化方法。
- `rtk_solution.json`：RTK 引擎类型（首期 RTKLIB adapter）、输出坐标框架、FIX/FLOAT 状态映射、P/V 有效性及协方差来源。
- `dynamic_status.json`：分类器类型/version、状态枚举开关、滑窗和最小样本数、各证据订阅/topic、时间同步容差与陈旧时限、静止/匀速/加速/减速/转弯判据、进入/退出门限与 dwell time、规则置信评分权重、冲突及 `unknown` 策略、诊断输出；各阈值带 SI 单位并需数据标定。
- `zupt.json`：ZUPT 使能、静止触发准入、`R_zupt`、NIS/异常残差策略、更新频率及速度参考点定义。
- `nhc.json`：NHC 使能、DynamicStatus 允许/拒绝状态、车辆参考点与 FRD 轴约定、IMU 到车辆参考点杆臂及车辆-IMU 安装旋转、速度参考系/地球转动补偿约定、侧向/垂向 `R_nhc`（非零正定）、NIS 门限、最大更新频率/相关时间、随速度/角速率/路况膨胀噪声的策略，以及 high-dynamic/slip/unknown 拒绝策略。若杆臂从状态估计，配置须引用其 state block；否则明确使用标定常量及不确定度处理。
- `zero_position_change.json`：ZPC 使能、依赖 ZUPT 静止触发、锚点建立/销毁/过期策略、`R_zpc`、更新频率、ECEF/锚点局部坐标及锚点状态交叉协方差要求。
- `zero_heading_change.json`：ZHC 使能、依赖 ZUPT 静止触发及有效锚点、heading/yaw 定义、角度 wrap、`R_zhc`、更新频率、参考局部导航系和锚点姿态交叉协方差要求。ZHC 不能配置成绝对 heading=0 量测。
- `fusion_model.json`：组合模式标记（如 `lc`、`tc`、`ltc` 或 `custom`）、状态模型 ID、过程模型 ID、观测模型 ID 列表、更新策略、相邻历元配对/最大间隔、首历元缓存、周跳与 pivot 切换后的重配对、回退行为。`combination_mode` 用于表达所选形式，实际启用内容以显式模型 ID 列表为准。
- `rtk_measurement.json`：RTK P/V 协方差映射、TD-DDCP 相位噪声、星间相关模型、创新门限和异常值处理。
- `measurement_correlation.json`：RTK 解与当前/前一历元相位量测的联合协方差/去相关模型，以及跨历元状态相关性来源和有效性校验；禁止默认设成交叉协方差为零。
- 模型配置使用注册的稳定 ID，而非在 JSON 中编写可执行类名。state model 定义状态块、顺序、单位、初始协方差和过程噪声；measurement model 定义输入消息、残差/雅可比实现、噪声模型、所需状态块及相关性组。组合加载后生成并冻结状态布局，随后所有 `H` 列、协方差维数和序列化元数据均以该布局版本为准。
- `paths.json`：IMU 文件、rover/base 文件、rover 初始坐标文件、已知 base 天线相位中心坐标文件、可选星历和输出目录。`base_coordinates` 是 RTK 解算使用的已知基站 ECEF 坐标；路径支持独立指定，不将 `data1` 固化成算法默认。
- `rtk_solution.json`：RTK 引擎及输入映射、基站坐标来源、FLOAT/FIX 接受策略、解状态协方差映射、位置/速度有效性、最低卫星数、周跳/失锁和模糊度固定参数。FLOAT 默认接受；FIX 可用时同样接受并按其质量协方差更新。

## 17. 工程目录结构建议（与现有源码分离）

```text
ChatGPT/
  CMakeLists.txt
  cmake/                         # FindEigen3/依赖封装
  third_party/                   # 依赖说明或 CMake 引用，不复制已有源码
  config/                         # JSON 配置及 schema
  include/navcore/                # 公共类型、模块/消息接口、状态结果
  include/modules/                # 对外模块接口
  src/app/                        # CLI 和应用装配
  src/core/                       # 配置、注册表、总线、调度、日志
  src/io/                         # 文件 reader、时间/单位适配
  src/navigation/common/          # 坐标、地球、四元数、噪声模型
  src/modules/ins/                # INS 算法模块
  src/modules/rtk/                # RTK engine adapter 和 RTK 内部模糊度管理
  src/modules/fusion/             # RTK P/V 松组合 + TD-DDCP 紧组合主滤波模块
  src/modules/dynamic_status/     # 滑窗特征、状态机/分类器与 DynamicStatus 发布
  src/modules/constraints/        # 消费 DynamicStatus 的 NHC、ZUPT 准入与量测
  src/adapters/rtklib/            # RTKLIB C API 隔离适配层
  data1/                          # 已有原始数据，仅读取
  eigen/                          # 已有 Eigen 源码，只读依赖
  RTKLIB-master/                  # 已有 RTKLIB 源码，只读依赖
  output/                         # 回放结果、诊断和运行清单
```

实际第一步应在当前目录创建清晰的 `src/`、`include/`、`config/`、`tests/` 和 CMake 工程；保留 `data1/`、`eigen/`、`RTKLIB-master/` 原样。`third_party/` 不应复制或修改上游源码；通过 CMake 选项指向本地目录，未来可替换成包管理器。

RTKLIB 当前源码标记 2.4.2 p13，根目录 README 声明 BSD 2-Clause 加额外条款。集成时保留上游版权/许可证文本，并复核随附的额外条款及二进制库许可；通过 adapter 使用其 RTK P/V 解及质量/协方差信息作为松组合量测，同时保留其与原始差分载波相位的统计相关性；RTKLIB 输出不得冒充本项目的融合导航结果。Eigen 目录标记 3.4.0，按其许可文件要求保留声明。

## 18. 配置实例和数据集配置草案

`offline_dataset1.json` 只负责把现有数据接到模块上，字段含义应在数据格式确认后锁定：

```json
{
  "schema_version": 1,
  "include": ["../system.json", "../paths.json", "../sensors/imu.json", "../sensors/gnss.json", "../navigation/ins.json", "../navigation/rtk_solution.json", "../navigation/fusion_model.json", "../navigation/dynamic_status.json", "../kalman/ins_error_state.json", "../kalman/rtk_measurement.json", "../kalman/measurement_correlation.json", "../kalman/zupt.json", "../kalman/nhc.json", "../kalman/zero_position_change.json", "../kalman/zero_heading_change.json"],
  "profile_name": "offline_dataset1",
  "dataset": {
    "imu_format": "imu100hz_text_v1",
    "rover_format": "gnss_epoch_text_v1",
    "base_format": "gnss_epoch_text_v1",
    "delimiter": "whitespace",
    "missing_value": 0,
    "time_scale": "GPS",
    "coordinate_format": "ECEF_m"
  },
  "paths": {
    "imu": "../../data1/imu/IMU_100Hz.txt",
    "rover_observations": "../../data1/GNSS/rover/rover.txt",
    "rover_coordinates": "../../data1/GNSS/rover/coordinates.txt",
    "base_observations": "../../data1/GNSS/base/base.txt",
    "base_coordinates": "../../data1/GNSS/base/coordinates.txt",
    "navigation_data": "../../data1/GNSS/navigation"
  }
}
```

`fusion_model.json` 的 LTC 模式示例：

```json
{
  "schema_version": 1,
  "navigation_filter": {
    "type": "configurable_eskf",
    "combination_mode": "ltc",
    "state_model": "ins_error_18d_lever_arm_v1",
    "process_model": "strapdown_ins_error_v1",
    "measurement_models": [
      {
        "id": "rtk_position_velocity_loose_v1",
        "enabled": true,
        "config": "../kalman/rtk_measurement.json"
      },
      {
        "id": "td_ddcp_tight_v1",
        "enabled": true,
        "config": "../kalman/rtk_measurement.json"
      },
      {
        "id": "stationary_zupt_v1",
        "enabled": false,
        "config": "../kalman/zupt.json",
        "admission_topic": "dynamic_status.stationary_candidate"
      },
      {
        "id": "zero_position_change_anchor_v1",
        "enabled": false,
        "config": "../kalman/zero_position_change.json",
        "requires": ["dynamic_status.zupt_trigger_active", "static_anchor.valid"]
      },
      {
        "id": "zero_heading_change_anchor_v1",
        "enabled": false,
        "config": "../kalman/zero_heading_change.json",
        "requires": ["dynamic_status.zupt_trigger_active", "static_anchor.valid"]
      },
      {
        "id": "vehicle_nhc_v1",
        "enabled": false,
        "config": "../kalman/nhc.json",
        "admission_topic": "dynamic_status.nhc_candidate"
      }
    ],
    "correlation_model": "rtk_td_ddcp_temporal_joint_v1",
    "update_policy": "joint_or_validated_decorrelation"
  }
}
```

同一接口允许其他 profile 选择只含 RTK P/V 的松组合（`lc`）、只含具体已实现紧组合观测的配置（`tc`），或两类模型共同工作的 LTC（`ltc`）。更换 `state_model` 可选择不同状态块布局，但必须满足启用模型声明的依赖；例如 TD-DDCP 需要能预测天线位置的导航状态和相应杆臂状态（若模型配置要求估计杆臂）。启动校验还须检查重复使用原始观测造成的相关性处理、状态布局版本、各观测模型 Jacobian 列数、噪声矩阵维数及必需输入。模式名用于表达用户选择的组合形式，实际实现由模型 ID 决定；系统不允许数学上不兼容的任意拼接。

`coordinate_format` 已按用户确认设置为 ECEF 米。`rover_coordinates` 是 rover GNSS 天线相位中心初始位置；如 INS 状态定义在 IMU 中心，使用配置的杆臂和初始姿态反算 IMU 初值。卫星位置从 base 观测文件按历元/SVID 读取；按用户规则，base 历元缺少卫星时忽略该星，不启用星历回退。

## 19. 仍需数据所有者确认的内容

为了把数据回放接入并产生可信解算，还需要确认：

1. rover 初始位置和 base 坐标均已确认为 GNSS 天线相位中心 ECEF 米坐标。杆臂标称初值设为 `[0,0,0] m`；可显式启用新增 3 维 body-frame 杆臂误差状态估计实际安装偏移。启用前需明确 INS 初始点与 GNSS 天线中心的换算关系，并提供足够姿态激励及独立杆臂验证。
2. IMU 单位、body frame 与 vehicle frame 重合、x/y/z 对应 vehicle FRD 前/右/下、加速度为含重力响应的比力、时间戳为采样时刻均已确认。首个样本和采样间隔异常的处理仍由实现按配置定义并记录。
3. 已知 C1 为米制伪距，L1/L2 为对应频段载波相位且单位 cycles；P2 不参与首期解算；D1 为 L1 Doppler，保留原始 Hz 有符号值，负值有效，不做绝对值转换。SatX/Y/Z 已确认 ECEF 米，SatElev 已确认 degree；若需使用 SatClk/SatElev，则取 base 文件对应行。SatClk 已确认单位为米；GNSS 文件以空格分列，数值 `0` 表示对应字段缺失。
4. rover/base 历元配对容差已确认为 1e-6 s（1 微秒）；base 对应历元缺少卫星时忽略该星，不做星历回退。基站坐标使用 `base_coordinates` 中已确认的天线相位中心 ECEF 坐标。仍需确认文件包含的星座及 SVID 编码方式。
5. 已确认有效 RTK FLOAT 和 FIX 均可作为松组合量测，FLOAT 不必等待整周模糊度固定。实现需保留解状态并按解状态映射协方差；无效分量不得伪造。RTK 输出字段、位置/速度协方差和与 TD-DDCP 的互协方差仍需由实现评估，若无法导出 `R_pv,phi`，按第 5.5 节安全策略处理，不默认独立堆叠两类量测。

SatX/Y/Z 为 ECEF 米、SatClk 为米、SatElev 为 degree，空格分列且数值 `0` 表示字段缺失，均已纳入数据接口约定。仍需确认文件中的星座/SVID 编码，以及未列明的系统运行参数；这些通过配置项处理，不改变已确认的数据规则。
