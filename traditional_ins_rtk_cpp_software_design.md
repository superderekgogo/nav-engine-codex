# 传统 GNSS/IMU RTK 松紧组合导航软件设计说明（C++）

> 文档状态：软件实现基线草案（LTC 可配置组合方案；量测相关性处理为实现前置要求）
> 版本：1.4
> 日期：2026-10-21
> 输入材料：`Traditional INS_RTK Tightly-Coupled Design Report.pdf`（v1.0，6 页）及 `traditional_ins_rtk_tightly_coupled_software_design_guide.md`

## 1. 目的与范围

本文定义一套可落地为 C++ 工程的传统捷联 INS 与 GNSS RTK 软件，并严格区分三种组合模式：**LC（loosely coupled）**使用 RTK 输出的位置/速度作为 INS 滤波器量测；**TC（tightly coupled）**直接将卫星级 TD-DDCP 几何残差用于滤波更新；**LTC（loosely tightly coupled）**使用 RTK 位置做 LC 位置更新，并将 L1 相邻历元差分载波相位解算为天线位移 `Δposition`，再作为区间速度量测更新 INS 速度。三种模式由 JSON 配置切换，可分别启用量测模型和状态模型；LTC 的 RTK 速度可作为独立开关，但默认关闭，避免与相位推导速度重复提供速度约束。

本文所称“时间差分载波相位”按双站、双差、历元间差分定义：在每个历元先形成 rover/base 接收机单差，再对卫星与 pivot 卫星作双差，最后对相邻历元的双差载波相位作时间差分（TD-DDCP）。相位以 cycles 输入，乘波长转为米。在无周跳且卫星/pivot 连续时，整周模糊度在时间差分中抵消，因此融合 ESKF 不维护载波模糊度状态。首期不把差分伪距作为 INS 紧组合量测；伪距可由 RTK 引擎内部使用以生成 RTK 解。

本文是软件设计规格，不把输入材料中的示例代码视为正确实现。报告与指南中部分符号约定尚有冲突，且报告第 6 页的“基准星切换”和“时间同步”内容被截断；相关事项列为实现前必须确认的决策，详见第 12 节。

**首期范围**：NED 导航系、FRD 载体系、WGS-84 ECEF 几何；18 维 INS/杆臂误差状态（杆臂在线估计）；IMU 机械编排和协方差传播；可切换的 LC、TC、LTC 三种模式；LC 的 RTK 位置/速度更新；TC 的 TD-DDCP 卫星级几何更新；LTC 的 RTK 位置更新与 L1 TDCP `Δposition`/速度更新；各模式的量测相关性建模或经验证的去相关处理；历元连续性与周跳检测；NHC、ZUPT 可选更新；进程内模块注册与强类型消息传递；多文件 JSON 配置；对 `data1` 的离线回放适配；时间戳检查、日志和结果输出。RTK 引擎内部的整数模糊度固定由 RTKLIB 等 RTK solution engine 负责，不属于融合 ESKF 状态。

**首期不默认开启**：比例因子状态、双频电离层/对流层残差参数化、完整 GNSS 接收机跟踪环、地图匹配。IMU 到天线杆臂在线估计属于首期滤波状态，默认初值 `[0,0,0] m`，其先验协方差与随机游走噪声由 JSON 配置。

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

系统采用“分层职责 + 可插拔 module”的结构。数据按流水线流动；相邻 module 通过进程内强类型消息总线发布/订阅，不直接依赖其他 module 的具体类。滤波器算法由 JSON 选择的状态模型、过程模型和一个或多个观测模型组合而成；共享导航状态、误差状态和互协方差由该算法实例自己的 `FilterContext` 管理，不通过消息总线拆开发送，避免破坏滤波状态的一致性。

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
│ ImuProcessor       GnssPreprocessor       FilterManager             │
│                    (质量/共视星)          (配对、缓存、排序、路由)  │
└────────────────────────────────────────────────────────────────────┘
                       │ImuSample / PreparedGnssEpoch
┌────────────────────────────────────────────────────────────────────┐
│ L4 GNSS 解算与导航融合层                                             │
│ RTK Engine -> RtkSolution(position, velocity, covariance)             │
│               + PreparedTdDdCarrierPhaseEpoch                        │
│ 可配置 ESKF: LC / TC / LTC 三模式切换                                │
│ Mechanization / TD-DDCP / TemporalCorrelationManager                 │
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
4. **L3 传感器处理与数据准备层**：`ImuProcessor` 做标定、时间间隔质量检查和样本规范化；`GnssPreprocessor` 做 rover/base 配对、共视卫星整理、卫星状态有效性筛选和观测质量标记；`FilterManager` 为每个算法实例排序/缓存事件，将 IMU 样本送入 Navigator，将配对好的 GNSS 历元组装为 `PreparedGnssEpoch` 并交给配置的 filter module。它不计算 Kalman 增益，不估计滤波状态。
5. **L4 GNSS 解算与导航融合层**：RTK solution engine（经 RTKLIB adapter 隔离）输出带状态质量和协方差的 RTK 天线位置/速度；相邻 rover/base 历元原始载波相位形成 TD-DDCP。配置驱动的 `NavigationFilter` 根据 state model 和 measurement model 列表创建状态布局、传播模型及更新器。当前三种预设分别为：`lc` 使用 RTK P/V；`tc` 使用卫星级 TD-DDCP；`ltc` 使用 RTK 位置与 L1 TDCP 推导的区间平均速度。RTK 整数模糊度仅由 RTK engine 管理。新算法仍可实现公共导航模块接口进行替换或对比。
6. **L5 输出层**：订阅导航解和诊断消息并写入所选目标；输出层不反向修改算法状态，控制命令通过显式的 command message 传递。

### 3.2 module 之间的信息传递

- 总线消息为不可变、强类型值对象，至少携带 `timestamp`（事件时间）、可选接收时间、`sequence_id`、`source_id`、`frame_id`、单位/质量标记和缺失字段掩码。
- 标准消息包括 `RawImuRecord`、`RawGnssRecord`、`ImuSample`、`GnssObservation`、`PreparedGnssEpoch`、`RtkSolution`、`PreparedTdDdCarrierPhaseEpoch`、`ConstraintObservation`、`NavigationSolution`、`ModuleDiagnostic` 和 `SystemEvent`。TD-DDCP 消息显式包含参考/当前历元及共同卫星与 pivot 身份。
- module 通过声明 `subscriptions` / `publications` 建立连接。启动时按 JSON pipeline 验证消息类型兼容；未连接的必需输入应在启动时报错。
- 回放可由 source module 并发读取，但进入单个滤波实例后由 `FilterManager` 按事件时间串行调度。RTK 迟到时，FilterManager 使用 IMU 缓存请求该滤波实例回滚/重放；超出缓存的历元按配置拒绝。
- 同一标准化传感器流可广播给多个独立导航算法实例用于比较；每个实例有独立状态和输出 topic。跨算法融合必须配置一个显式融合 module，不把不同算法的状态向量直接平均。
- 算法实例内部 `Navigator` 与 `FilterAlgorithm` 通过明确的接口或共享 `FilterContext` 协作；这种内部紧耦合状态访问不跨 module bus。

### 3.3 FilterManager 与算法模块的边界

`FilterManager` 是运行时的数据准备/调度器，不是滤波器本身。它负责：

- 按时间戳整理 IMU、rover、base 事件；使用已配置的 rover/base 时间容差配对历元。
- 按 SVID 将 rover/base 观测配成历元；从 base 对应行取得可用卫星位置/钟差/高度角。按用户规则，缺失卫星直接忽略，不尝试外部星历回退；同时为 RTK 解算器与相位差分支路提供一致的原始观测快照。
- 对字段缺失、时间不匹配、重复卫星、质量不合格生成诊断和 `PreparedGnssEpoch`；尽可能保留某卫星其他有效字段，不把整行无条件丢弃。
- 向 RTK solution engine 提供 GNSS 历元，并向融合滤波器提供时间一致的 `RtkSolution`、差分载波相位历元、`ImuSample` 和约束观测；维护必要的时间回滚缓存与事件统计。

`FilterManager` 不负责：计算 TD-DDCP 几何雅可比、选 Kalman 量测模型、修改 `P/dx`、执行闭环反馈或决定 RTK 整数固定。这些职责属于具体算法 module；TD-DDCP 的历元配对/连续性由相位连续性组件维护，RTK 模糊度和整数固定则留在 RTK engine。这样增加新算法时复用数据层和调度层，而不同算法可保留各自的数学模型和状态定义。

### 3.4 module 处理顺序

1. Source 发布 raw record；Decoder 解析并发布标准化消息。
2. `ImuProcessor` 输出 FRD body frame 的 `ImuSample`；`GnssPreprocessor` 与 `FilterManager` 将 rover/base 观测配成时间一致的 GNSS 历元。
3. RTK solution engine 对 GNSS 历元执行 RTK 解算，发布 `RtkSolution`；连续性组件缓存前一有效载波相位历元，并构造相邻历元 TD-DDCP，保留卫星/信号/pivot 身份及周跳质量信息。
4. `FilterManager` 按事件时间将 IMU、RTK 解和 TD-DDCP 送入由 JSON 装配的 `ConfigurableNavigationFilter`。LC 运行 RTK P/V 更新；TC 运行卫星级 TD-DDCP 几何更新；LTC 运行 RTK 位置更新和 L1 TDCP 位移推导速度更新。关联量测需联合更新或经过去相关处理，不得默认独立。
5. 滤波器管理在线杆臂、前后历元状态/协方差关联及闭环反馈，发布统一的 `NavigationSolution` 和诊断消息；输出 modules 订阅并保存/显示。RTK 模糊度固定仍由 RTK engine 内部完成。
6. 可在 pipeline 配置多个导航算法实例订阅同一输入流，比较结果而不共享内部滤波状态。
## 4. 状态、协方差与数据结构

### 4.1 状态定义

误差状态建议定义为：

三种融合模式可共享 `ins_error_18d_lever_arm_v1` 状态：`dx = [dr_N,dr_E,dr_D, dv_N,dv_E,dv_D, phi_N,phi_E,phi_D, dbg_x,dbg_y,dbg_z, dba_x,dba_y,dba_z, dl_bx,dl_by,dl_bz]^T`。索引定义为：位置 `[0,3)`、速度 `[3,6)`、姿态 `[6,9)`、陀螺 bias `[9,12)`、加计 bias `[12,15)`、IMU 到 GNSS 天线杆臂误差 `[15,18)`。该预设固定为 18 维；RTK engine 自己维护的模糊度不加入此状态向量。其他 JSON 选定的状态模型可使用不同的已注册布局。

前 15 维是惯导误差状态：局部 NED 位置误差（m）、速度误差（m/s）、姿态小角（rad）、本体系陀螺零偏误差（rad/s）和本体系加计零偏误差（m/s²）。索引 15–17 是 body/vehicle frame 下杆臂估计误差（m）。标称状态保存 `lever_arm_b`，首期初值 `[0,0,0] m`；杆臂为固定安装参数，误差状态按配置的小随机游走噪声传播，更新后反馈到标称杆臂并重置对应误差均值。比例因子若启用，需在状态布局版本中追加，不可暗中改变已有索引。

`NominalNavState` 至少包含：时间戳、LLA、NED 速度、`q_bn`、陀螺/加计零偏、当前估计 `lever_arm_b`、初始化状态标志。`FilterState` 包含固定 18 维误差均值 `dx` 和协方差 `P`，以及 TD-DDCP 前一历元的相位、卫星状态、pivot、时间戳、质量标志和用于一致更新的前一历元状态/交叉协方差快照。杆臂初始值为零，但必须配置合理的初始方差；杆臂与初始位置/姿态误差可能相关，系统须保留交叉协方差并诊断可观测性。

### 4.2 TD-DDCP 历元配对与连续性

TD-DDCP 配对键至少包含 `{constellation, satellite PRN, signal/frequency, pivot PRN}`，并记录前后历元时间戳。只有同一卫星/信号在前后历元连续且无周跳、共视集与 pivot 兼容时才形成该行观测。首个有效历元只建立缓存，不执行相位更新；时间间隔超限、缺星、周跳或 pivot 不兼容时，丢弃受影响配对并以当前有效历元重新建立参考。每星周跳会使该星涉及的时间差分失效；pivot 周跳或切换会影响该组差分，应整组重置或重建配对。RTK 引擎内部的 ambiguity key 和 pivot 管理独立于融合状态。

### 4.3 初始化与输出

初始化输入应包含初始位置、速度、姿态、IMU bias、初始杆臂（默认零）、18×18 初始协方差，以及时间/来源元数据。必须拒绝非有限数、非正定协方差、无效四元数、纬度越界和重复/倒退时间戳。输出至少包含时间、LLA、ECEF、NED 速度、姿态、协方差所对应的导航不确定度、解状态（INS/FLOAT/FIX）、卫星数、ratio、创新统计和告警码。

## 5. 算法处理流程

### 5.1 IMU 传播

1. IMU 时间戳代表采样时刻。按时间戳排序并检查间隔；对重复、倒序、过大间隔采用可配置策略（拒绝、重初始化或显式降级），不得静默继续。
2. 按配置对原始增量去 bias；若开启比例因子，按报告给出的对角模型补偿。
3. 运行二子样 coning/sculling 机械编排。首个样本、缺样、长间隔和重置后的历史增量规则必须显式处理；首个增量不可既用于初始化历史又再次按完整两样本公式重复计入。
4. 更新姿态（归一化并检查）、速度和 LLA；处理极区 `cos(lat)`、高度、地球曲率和重力模型边界。
5. 由同一历元标称状态和比力构造 `F/G`，将 INS 15 维扩展为含 body-frame 杆臂误差的固定 18 维；杆臂误差按配置随机游走建模。离散化得到 `Phi/Qd`，传播 `P = Phi P Phi^T + Qd`。对称化 `P`，监控特征值/Cholesky 失败。

采用报告给出的二阶 `Phi` 和噪声梯形积分作为初始基线；实际 `Qc` 的单位及连续/离散噪声定义须在传感器参数中统一，避免重复乘除采样周期。

### 5.2 RTK 位置/速度松组合更新

1. RTK solution engine 使用准备好的 rover/base GNSS 历元执行 RTK 解算。已知基站坐标取配置的 `base_coordinates` 文件，作为基站天线相位中心的 ECEF 米坐标，传给 RTK 引擎的参考站位置；不得把 base 文件中的逐星 SatX/Y/Z 当作基站坐标。RTK 输出为天线相位中心位置、速度、解状态、有效卫星数、时间戳及可用协方差。
2. **FLOAT 与 FIX 均可用于松组合。** FIX 前产生的有效 FLOAT 解不需等待整数模糊度固定即可进入松组合更新；不得仅因 `status != FIX` 而丢弃 FLOAT。依据解状态和引擎协方差分别设置/映射观测噪声，并保留 `FIX/FLOAT/INVALID` 状态、卫星数、ratio（若可用）与位置/速度分量有效标记。INVALID、时间不匹配、非有限数或协方差无效的分量拒绝更新。若配置允许，其他状态（例如 DGPS）由单独开关决定，默认不纳入本项确认。
3. 将 INS 状态传播/回滚到 RTK 解的观测时间。用标称姿态和当前 `lever_arm_b` 计算 INS 预测的 rover 天线位置；天线速度还应包含杆臂旋转速度项，并按一致的 ECEF/NED 坐标定义实现。
4. 构造 `z_pv = [r_RTK^e, v_RTK^e]` 与预测函数 `h_pv(x)`，残差为 `z_pv - h_pv(x)`。雅可比包含位置、速度、姿态、IMU bias 和杆臂状态对天线位置/速度的影响。
5. 按 FIX/FLOAT 状态和质量配置观测协方差：优先使用 RTK 引擎提供的协方差；若只有精度指标，则按明示的模型构造并分别配置 FIX/FLOAT 噪声。FLOAT 可单独启用、降权或按质量门限拒绝，但系统默认允许合格 FLOAT 更新。保留位置-速度交叉协方差（若可用），拒绝无效分量并记录原因。
6. 进行松组合 EKF 更新，更新同一 `FilterState` 的 `dx/P`，再按统一误差约定闭环反馈 INS 和杆臂状态并执行协方差 reset。该更新不新增载波模糊度状态。

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

### 5.7 NHC / ZUPT

NHC 仅在车辆运动且车辆模型适用时约束车体系侧向/垂向速度；安装角、杆臂、转弯补偿和其对零偏的影响应通过观测模型明确。ZUPT 需由静止检测器触发，联合检查加速度/角速度稳定性并设置合理噪声。无检测器时不得自动把低速等同于静止。

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
struct NavSolution { Time timestamp; NominalNavState state; SolutionStatus status; Diagnostics diagnostics; };

class ConfigurableNavigationFilter {
public:
  Status initialize(const InitialConditions& init);
  Status propagateImu(const ImuSample& sample);
  Status process(const NavigationMeasurementBundle& bundle); // dispatches enabled model IDs
  NavSolution solution() const;
  void reset(ResetMode mode);
};
```

`ConfigurableNavigationFilter` 由 `StateModelRegistry`、`ProcessModelRegistry` 和 `MeasurementModelRegistry` 按 JSON 装配。每个观测模型实现统一的 prepare/linearize/update 契约，并声明所需状态块；state model 提供有版本的状态布局、注入/reset 规则及协方差初始化。状态模型可通过加入可选 state block 扩展，但不能在运行中改变状态维数或索引。单个 filter 实例内的所有观测模型更新同一 `FilterContext`，因此位置/速度和 TD-DDCP 仍共享同一状态及协方差。

调用约定：单个实例由一个处理线程串行调用；跨线程数据经队列传递。若产品需要并发查询，`solution()` 返回不可变快照或受锁保护的拷贝。

## 8. 时间同步、延迟与回放

INS 状态必须可在观测时间戳处重建。首期实现可通过 IMU 环形缓存和状态快照支持有限时长回滚：若某个 `NavigationMeasurementBundle` 晚到且仍在缓存范围内，恢复该历元前状态、传播至观测时刻、按启用的模型联合/去相关更新，再重放缓存 IMU；超出缓存则拒绝该历元并记录原因。RTK solution 和原始差分相位须按 GNSS 观测时间戳关联，不能因解算输出延迟而错配历元，也不能直接将旧观测应用于当前时刻状态。IMU/GNSS 时间基准、天线观测历元定义、设备延迟及插值策略通过配置明确。

## 9. 诊断和可观测性

结构化日志至少记录输入时间与延迟、dt、IMU 更新次数、RTK 解状态/卫星数/位置速度协方差、P/V 与 TD-DDCP 各自创新和标准化残差、TD-DDCP 前后时间戳/间隔/共视星数/pivot/失效原因、联合创新协方差及跨时交叉协方差来源、RTK 引擎 FIX/FLOAT/ratio 状态、周跳事件、固定 18 维状态、反馈量、协方差健康指标及错误码。日志可关闭高频逐样本输出，但需支持可复现实验的原始观测回放。

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
| 运动约束 | NHC 噪声、车辆轴向/安装角、静止检测阈值、ZUPT 噪声 |
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
8. 工程质量：编译警告、ASan/UBSan（平台支持时）、确定性回放、配置和日志可复现。

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
9. **NHC/ZUPT 模型**：确认车辆定义、安装角/杆臂、检测器、NHC 观测和零偏项；指南仅声明接口，尚不足以据此实现可靠模型。
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

## 13. 依据与限制

本设计的坐标系、18 维惯导/杆臂误差状态、闭环 ESKF、差分载波相位、杆臂、周跳、LAMBDA、NHC/ZUPT 和模块方向来自用户提供的 PDF 报告与 Markdown 指南；RTK P/V 松组合与差分载波相位紧组合的混合架构按用户本轮决策更新。输入代码段用于理解意图，不作为经过编译或数值验证的实现。量测相关性、误差约定和未提供的传感器参数仍须在实现和验证中锁定。

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
- **Constraint 模块**：NHC、ZUPT、轮速、气压计等按需启用。
- **Output 模块**：轨迹、诊断、CSV/JSON、可视化或应用接口。

本项目首期融合实现拆为 `StrapdownMechanizer`、可选 `ErrorStateModel`、`RtkSolutionAdapter`、`RtkPositionVelocityMeasurementModel`、`TimeDifferencedDoubleDifferenceCarrierPhaseModel`、`CarrierPhaseContinuityManager`、`TemporalMeasurementCorrelationManager`、`ConstraintMeasurementModel` 和 `ConfigurableNavigationFilter`。JSON 通过稳定的模型 ID 选择并组合已注册实现；RTK 的 ambiguity manager / LAMBDA 留在 RTK solution engine adapter 内部。新算法可注册新的观测/状态模型，不需要改动数据解析和输出代码。

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

class INavigationModule : public IModule {
public:
  virtual Status onImu(const ImuSample&, MessageContext&) = 0;
  virtual Status onMeasurementBundle(const NavigationMeasurementBundle&, MessageContext&) = 0;
  virtual Status onConstraint(const ConstraintObservation&, MessageContext&) = 0;
  virtual std::optional<NavigationSolution> latestSolution() const = 0;
};
```

代码示例表达契约，不限定最终 ABI。模块声明其订阅和发布的消息类型、所需传感器、运行频率、是否支持延迟数据和是否可重置。`ModuleRegistry` 负责工厂注册和按 `type` 创建实例；内置模块静态注册，未来动态插件需另行规定 ABI/版本边界，首期不要求跨 DLL 热加载。

### 15.3 消息总线

采用进程内、强类型消息总线，首期单线程确定性调度；回放与实时接收器可独立生产消息，但进入算法队列后按事件时间戳排序。消息包含 `timestamp`（传感器事件时间）、`receive_timestamp`（接收时间，可选）、`sequence_id`、`source_id`、`frame_id`、单位约定和质量标记。

核心消息至少包括：`ImuSample`、`GnssObservationEpoch`、`BaseStationInfo`、`RtkSolution`、`PreparedTdDdCarrierPhaseEpoch`、`NavigationMeasurementBundle`、`ConstraintObservation`、`NavigationSolution`、`ModuleDiagnostic`、`SystemEvent`。消息由配置的组合模式选择：LTC 消费 RTK 位置和 L1 TDCP 并发布派生位移/区间速度观测；TC 消费卫星级 TD-DDCP 行；LC 消费 RTK P/V。TD-DDCP 消息显式包含前后时间戳、卫星/pivot 坐标和共享 pivot 协方差。模块向总线发布不可变值对象；订阅者不得保留指向生产者临时内存的裸指针。大块数组可用共享只读缓冲区，但生命周期由消息对象持有。

总线提供订阅/退订、顺序处理、异常隔离、队列容量和背压策略、消息追踪 ID。滤波器的单实例状态更新保持串行，以免时序重排。消息总线只负责传递事件，不代替 ESKF 内部的状态交叉协方差；若两个算法模块需要融合其估计结果，必须使用明确的融合算法模块，而非简单平均状态。

### 15.4 运行管线

配置文件指定 `modules` 实例列表和 `pipeline` 连接关系。启动时先校验依赖图无环（若允许反馈环须显式声明并有调度规则），确认消息类型可连接，然后按拓扑顺序 configure/initialize/start。停止时逆序 stop。算法切换策略（单模块选择、并行对比或级联）由管线配置表达，数据源可复用，算法模块互不覆盖输出。

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
  kalman/
    ins_error_state.json      # 初始协方差、过程噪声、相关时间
    rtk_measurement.json      # RTK P/V 协方差及差分载波噪声/高度角模型
    measurement_correlation.json # P/V 解与原始差分相位的交叉协方差
    nhc.json                  # NHC 参数
    zupt.json                 # 静止检测与 ZUPT 参数
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
    "../kalman/ins_error_state.json",
    "../kalman/rtk_measurement.json",
    "../kalman/measurement_correlation.json"
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
  src/modules/constraints/        # NHC、ZUPT 等约束
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
  "include": ["../system.json", "../paths.json", "../sensors/imu.json", "../sensors/gnss.json", "../navigation/ins.json", "../navigation/rtk_solution.json", "../navigation/fusion_model.json", "../kalman/ins_error_state.json", "../kalman/rtk_measurement.json", "../kalman/measurement_correlation.json"],
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

1. rover 初始位置和 base 坐标均已确认为 GNSS 天线相位中心 ECEF 米坐标。杆臂标称初值设为 `[0,0,0] m`，通过新增 3 维 body-frame 杆臂误差状态在线估计实际安装偏移；GNSS 天线初始位置结合初始姿态和杆臂初值转换为 INS 初始位置。
2. IMU 单位、body frame 与 vehicle frame 重合、x/y/z 对应 vehicle FRD 前/右/下、加速度为含重力响应的比力、时间戳为采样时刻均已确认。首个样本和采样间隔异常的处理仍由实现按配置定义并记录。
3. 已知 C1 为米制伪距，L1/L2 为对应频段载波相位且单位 cycles；P2 不参与首期解算；D1 为 L1 Doppler，保留原始 Hz 有符号值，负值有效，不做绝对值转换。SatX/Y/Z 已确认 ECEF 米，SatElev 已确认 degree；若需使用 SatClk/SatElev，则取 base 文件对应行。SatClk 已确认单位为米；GNSS 文件以空格分列，数值 `0` 表示对应字段缺失。
4. rover/base 历元配对容差已确认为 1e-6 s（1 微秒）；base 对应历元缺少卫星时忽略该星，不做星历回退。基站坐标使用 `base_coordinates` 中已确认的天线相位中心 ECEF 坐标。仍需确认文件包含的星座及 SVID 编码方式。
5. 已确认有效 RTK FLOAT 和 FIX 均可作为松组合量测，FLOAT 不必等待整周模糊度固定。实现需保留解状态并按解状态映射协方差；无效分量不得伪造。RTK 输出字段、位置/速度协方差和与 TD-DDCP 的互协方差仍需由实现评估，若无法导出 `R_pv,phi`，按第 5.5 节安全策略处理，不默认独立堆叠两类量测。

SatX/Y/Z 为 ECEF 米、SatClk 为米、SatElev 为 degree，空格分列且数值 `0` 表示字段缺失，均已纳入数据接口约定。仍需确认文件中的星座/SVID 编码，以及未列明的系统运行参数；这些通过配置项处理，不改变已确认的数据规则。
