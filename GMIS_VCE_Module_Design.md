# GMIS-VCE 模块设计文档

**版本：** 0.1（理论与软件设计评审稿）

**状态：** 阶段 A 诊断型 GMIS-LC 原型已实现；AKF 调权尚未实现

**适用范围：** 当前 GMIS-LC、GMIS-LTC、GMIS-TC 及后续传感器模型
**目标：** 建立一个与导航状态和传感器种类解耦的方差分量估计模块，用于评估各传感器/观测组的噪声假设，并在验证后为滤波量测协方差提供可控的自适应缩放。

---

## 1. 执行摘要

Variance Component Estimation（VCE，方差分量估计）使用滤波器产生的残差及其冗余度，估计不同观测组的噪声方差因子。放在 GMIS 中，它能按观测模型或传感器组跟踪“实际残差与先验噪声模型是否相符”，并为后续 Adaptive Kalman Filter（AKF）提供统计依据。

建议将 VCE 设计成独立的 `nav::gmis::vce` 模块，置于 GMIS 的 `FilterManagerModule` 与通用 `StateEstimator` 之间：领域模型负责提供测量、Jacobian 和先验协方差；`StateEstimator` 仍只做通用预测/更新；VCE 管理残差组、滑窗方差因子和诊断；只有显式启用 AKF 时，协方差适配器才把估计因子应用到**后续历元**的量测协方差。

采用两阶段落地：

1. **阶段 A：只分析，不改变解算。** 采集正确的后验测量残差、残差协方差、冗余度和窗口方差因子；输出诊断报告与曲线。
2. **阶段 B：受控的 R 自适应。** 通过历史数据评估后，按模型组对后续 R 调权，设上下限、变化率和降级策略。先不自动修改 Q，也不把 VCE 与 FDE/硬剔除混为一谈。

GMIS 技术报告中的 Helmert VCE 主线可采用，但参考材料中的状态维度、残差接口和若干“直接从创新计算因子”的代码草图不能原样照搬。当前工程的 GMIS-LC 全值状态是 21 维，通用 `StateEstimator` 不依赖具体状态维度；因此 VCE 也必须由测量组及协方差矩阵驱动，不得硬编码 27 维状态。

## 2. 输入材料与项目现状

### 2.1 理论材料提炼

参考材料给出的主要思路是：

- 把观测协方差写成多个已知辅因子矩阵的线性组合；
- 滤波后计算各观测组的后验残差及其协方差；
- 通过残差二次型与冗余度之比估计每组方差分量；
- 用滑动窗口累积统计量，降低单历元冗余度不足和噪声波动带来的影响；
- 将估计出的方差因子用于噪声诊断，或在后续更新中自适应缩放 R/Q。

### 2.2 与现有代码的接口差距

当前 `navcore::StateEstimator::update` 返回创新、创新协方差、按边际标准差归一化的创新、NIS 和接受状态。它没有返回：测量后验残差、后验残差协方差、分组冗余度，也没有保留用于这些统计量的完整更新快照。GMIS 的 LC/LTC/TC session 会在领域层构造矩阵并调用这个通用估计器。

因此，当前日志中的 innovation/NIS 可用于一致性检查，但**不能直接代替 VCE 所需的后验残差分组统计**。在同一历元分批顺序更新多个测量组时，也不能把某一组的创新单独当成完整的 Helmert VCE 残差，因为它包含状态预测不确定度，而且会受其他组更新顺序影响。

### 2.3 设计需要纠正的参考假设

| 参考材料中的描述 | 本设计的处理 |
|---|---|
| 统一写作 `Σ = Σ σᵢ Dᵢ`，代码又估计 `σ̂²` | 统一使用方差因子 `λᵢ = σ₀ᵢ²`，写作 `R = Σ λᵢ Dᵢ`；标准差倍率为 `sqrt(λᵢ)`。 |
| 示例状态固定为 27 维 | VCE 不读取导航状态布局，只处理有效观测子空间上的矩阵。适配现有 21 维 GMIS 与今后不同状态布局。 |
| 示例代码在更新后从同一组 `d,H,P⁻,R` 回算全部分组残差 | 生产接口必须明确同一线性化点、全部联合观测和分组结构，或由估计器返回更新诊断快照；不可用顺序更新后混合时刻的矩阵伪造联合统计。 |
| 任意测量组可独立使用 `vᵢᵀDᵢ⁻¹vᵢ/rᵢ` | 该简式只适用于有正确协方差分组、且组间独立或相关性已纳入广义 VCE 的条件。共用参考星 DD、TDCP 和 RTK 共享原始 GNSS 数据时须保留相关性或合并成一个分量。 |
| 以 `|vᵢ|/sqrt(Dvvᵢᵢ)>3` 直接判野值并剔除 | VCE、NIS 门限与 FDE 是不同策略。此阈值只能作为诊断提示；自动剔除须有独立配置、校准过的门限和恢复逻辑。 |
| 同时估计并调节 R、Q | 先做 R 分量诊断/调权。Q 分量需要过程噪声后验估计及可观测性支持，单靠观测创新不能可靠区分过程模型失配与观测异常。 |

## 3. 理论模型

### 3.1 线性化滤波更新

在某个共同线性化点，定义创新：

\[
\mathbf d = \mathbf z - h(\hat{\mathbf x}^{-}),\qquad
\mathbf d = \mathbf H\,\delta\mathbf x + \boldsymbol\epsilon,
\]

\[
\mathbf S = \mathbf H\mathbf P^{-}\mathbf H^T + \mathbf R,\qquad
\mathbf K = \mathbf P^{-}\mathbf H^T\mathbf S^{-1}.
\]

其中，`P⁻` 是本次联合观测更新前的状态协方差，`R` 是本次联合观测的完整协方差。采用 Joseph 形式更新后得到 `P⁺`。数值实现应通过 LDLT/LLT 求解线性方程，避免显式求逆。

### 3.2 方差分量参数化

将观测按可识别的误差来源划分为 `m` 个分量：

\[
\mathbf R(\boldsymbol\lambda) = \sum_{i=1}^{m}\lambda_i\mathbf D_i,
\qquad \lambda_i=\sigma_{0,i}^2>0.
\]

`Dᵢ` 是第 `i` 组的先验协因数（cofactor）矩阵，包含该组内部的相关结构；`λᵢ=1` 表示先验噪声尺度与残差统计相符，`λᵢ>1` 表示该组实测离散度高于先验假设，`λᵢ<1` 表示低于先验假设。所有参与求解的 `Dᵢ` 必须与所属观测行、单位和时间相关性一致。

若每组预先给出绝对协方差 `Rᵢ`，也可以表示为 `Rᵢ=λᵢDᵢ`；实现中需要明确 `Dᵢ` 是方差还是标准差，避免重复平方。

### 3.3 后验观测残差与协方差

令状态校正量为 `δx̂=K d`，测量后验残差采用“预测减观测”的符号约定：

\[
\mathbf v = \mathbf H\,\delta\hat{\mathbf x}-\mathbf d
            = (\mathbf H\mathbf K-\mathbf I)\mathbf d,
\]

\[
\mathbf C_v = \mathbf R-\mathbf H\mathbf P^{+}\mathbf H^T.
\]

残差符号不影响方差因子的二次型，但日志和实现必须统一约定。对于观测分量 `i`，`vᵢ` 和 `Cᵥ,ᵢᵢ` 表示取出对应观测行后的子向量和子协方差。`Cᵥ` 必须使用更新后的协方差及相同线性化点；状态协方差应使用 Joseph 更新结果。

### 3.4 单历元 Helmert 因子估计

当各分量相互独立，或 `Dᵢ` 已按理论构造并正确处理交叉相关时，定义该组冗余度贡献：

\[
r_i=\operatorname{tr}(\mathbf D_i^{-1}\mathbf C_{v,ii}),
\]

\[
q_i=\mathbf v_i^T\mathbf D_i^{-1}\mathbf v_i,
\qquad
\hat\lambda_i=\frac{q_i}{r_i},\quad r_i>r_{\min}.
\]

`rᵢ` 是该组的有效自由度/冗余度贡献，不应与量测行数混为一谈。计算中通过矩阵分解求 `Dᵢ⁻¹Cᵥ,ᵢᵢ` 和 `Dᵢ⁻¹vᵢ`；不显式形成逆矩阵。若 `rᵢ` 小于门限、非有限、明显为负或组矩阵非正定，该历元不更新因子，只记录原因。

存在跨组相关时，以上分组简式不再自动成立。首版应把具有强相关的观测合并为同一个 variance component，或者明确实现适用于相关辅因子矩阵的 generalized/Helmert VCE 正规方程；不可把相关行假设为独立后套用简式。

### 3.5 多历元滑动窗口

对窗口 `W` 内有效历元 `k` 累积：

\[
\hat\lambda_{i,W}=
\frac{\sum_{k\in W} w_k\,\mathbf v_{i,k}^T\mathbf D_{i,k}^{-1}\mathbf v_{i,k}}
     {\sum_{k\in W} w_k\,r_{i,k}}.
\]

`wₖ` 是配置的历元权重，默认 1。窗口按秒数或有效更新历元数定义；每个分量独立计数有效样本和累计冗余度。窗口不足、运动激励不足或卫星几何导致低冗余时，因子状态为 `insufficient_redundancy`，不强行回退为“噪声正常”。

窗口统计应记录：`sum_q`、`sum_redundancy`、有效历元数、有效自由度、窗口起止时间、因子估计和质量标志。平滑因子可用 EWMA 或对数域平滑，但原始未平滑估计必须同时保存，防止平滑掩盖异常。

### 3.6 R 自适应和 Q 扩展

通过验证后，可将当前窗口估计用于**下一次**量测构造：

\[
\lambda_{i,k}^{use}=
\operatorname{clip}(\tilde\lambda_{i,k-1},\lambda_{min},\lambda_{max}),
\qquad
\mathbf R_{k,i}^{use}=\lambda_{i,k}^{use}\mathbf D_{i,k}.
\]

调权仅影响未来更新，不能在当前观测已被滤波器吸收后追溯修改它的权值。建议首期自适应仅允许保守上调（软降权），或将上下界与每秒变化率限制在专门的 JSON 配置中；下调噪声需要经过一致性验证，避免权值过大造成虚假的小协方差。

Q 的 VCE 后续单独设计。GMIS 预测噪声必须按来源分解为 `Q=Σ λq,j Qj`，并由滤波/平滑过程得到过程残差 `ŵ` 及其后验协方差。不能把状态估计误差或 GNSS innovation 直接解释为过程噪声残差。

## 4. 软件架构

### 4.1 模块位置

```text
GMIS FilterManagerModule
  ├─ 接收状态传播与各 MeasurementModel 的线性化结果
  ├─ 按时间戳、观测模型和分量配置组装 MeasurementBatch
  ├─ 查询 VceEngine 当前有效的 R 缩放因子（阶段 A 只返回 1）
  ├─ 调用通用 StateEstimator 执行一次联合更新
  └─ 将更新快照发给 VceResidualExtractor
          ├─ 提取/计算后验残差 v 与 C_v
          ├─ 按组形成冗余度与二次型统计
          └─ VceEngine 更新滑窗方差因子并发出 VceDiagnosticMessage

GmisEstimationModule / MeasurementModel 不调用 VCE 算法；StateEstimator 不识别 GNSS、IMU 或 VCE。
```

VCE 是 GMIS 侧的自适应与统计模块，不应放进通用 `StateEstimator`，也不应依赖 TMIS 的 INS mechanization/误差反馈类。`GMIS_LC/TC/LTC` 可在各自 FilterManager 中装配同一 VCE 接口，但每种模式使用独立模型组配置及独立 VCE 历史。

### 4.2 核心类型

建议首期接口（仅表达设计，不是当前已存在代码）：

```cpp
namespace nav::gmis::vce {

struct VarianceComponentSpec {
  std::string component_id;       // 例如 imu_gyro、rtk_position
  std::string measurement_model_id;
  Eigen::MatrixXd cofactor;       // D_i；保留本组通道相关性
  Eigen::VectorXi row_indices;    // 映射到联合 MeasurementBatch
};

struct MeasurementBatch {
  double time_seconds;
  Eigen::VectorXd innovation;     // d = z - h(x^-)
  Eigen::MatrixXd jacobian;       // 同一预测状态/线性化点
  Eigen::MatrixXd prior_covariance;
  Eigen::MatrixXd measurement_covariance;
  std::vector<VarianceComponentSpec> components;
  std::vector<std::string> row_labels;
};

struct VceComponentDiagnostic {
  std::string component_id;
  double quadratic_form = 0.0;   // q_i
  double redundancy = 0.0;        // r_i
  double raw_variance_factor = 1.0;
  double filtered_variance_factor = 1.0;
  std::size_t valid_epochs = 0;
  std::string status;
};

class VceResidualExtractor;
class VceEngine;
class MeasurementCovarianceAdaptor;
}
```

在不复制完整 `P⁻` 的前提下，也可让 `StateEstimator` 提供仅含线性化诊断的数据 `EstimatorUpdateDiagnostics`：创新、S、H、R、`P⁺` 或等价的后验残差协方差、更新接受状态。是否扩展通用结果类型，需结合内存成本评审；不得让通用 estimator 计算 `rᵢ` 或知道 component ID。

### 4.3 模块职责

| 模块 | 职责 | 禁止承担 |
|---|---|---|
| `VceConfiguration` | 解析启用模式、分组、窗口、门限、倍率边界 | 推断传感器物理模型或改写状态布局 |
| `VceResidualExtractor` | 使用同批次线性化矩阵/滤波结果求 `v`、`Cᵥ`、`qᵢ`、`rᵢ` | 直接改状态或自行剔除观测 |
| `VceEngine` | 有效性判断、窗口累计、因子估计/平滑、质量状态 | 直接调用 GNSS/IMU decoder 或 `StateEstimator` |
| `MeasurementCovarianceAdaptor` | 将已批准的 `λᵢ` 应用到后续批次协方差 | 反向修改历史更新、修改 Q 或触发 FDE |
| `VceDiagnosticLogger` | 输出逐历元诊断和汇总统计 | 影响滤波状态与调权决策 |
| `GmisFilterManager` | 协调批次、更新顺序、接受结果和 VCE 消息 | 把传感器物理公式复制到通用 EKF |

### 4.4 更新时序

每个滤波历元的顺序应固定：

1. 对状态进行 GMIS prediction，保存本历元 `x⁻, P⁻`。
2. 由可用模型在同一 `x⁻` 处构造 `MeasurementBatch`；包含有效观测掩码、完整 R、组 ID 和每行语义。
3. 阶段 A 使用配置先验 R；阶段 B 查询**上一窗口结束时**发布的倍率并构造本历元 R。
4. 对可形成同历元统计的观测，先联合 EKF 更新；不能在非共同线性化点拼接多个已经顺序更新的创新。
5. 更新接受后，由 `VceResidualExtractor` 得到后验残差和残差协方差，按 component 累计统计；被 NIS 门限拒绝、无有效数据或降维的观测行要按实际批次标记。
6. `VceEngine` 更新滑窗状态；新估计的倍率最早在下一历元应用，并发布诊断消息。
7. 日志记录先验倍率、实际使用倍率、原始/平滑估计、冗余度、二次型、有效 DOF、门限状态和是否用于调权。

## 5. 观测分组与 GMIS 模式策略

### 5.1 初始建议分组

| 分量 ID | 对象 | 初期处理 |
|---|---|---|
| `imu_gyro` | 单 IMU 陀螺三轴 | 与 accel 可在同一 IMU batch 更新；是否拆成两个分量取决于 R 的交叉项是否有可信模型。 |
| `imu_accel` | 单 IMU 加计三轴 | 同上。单历元冗余度低时只做较长窗口统计。 |
| `rtk_position` | RTK N/E/D 位置 | GMIS-LC/LTC 的位置观测组；保留输入 RTK covariance 来源，避免仅凭 status 推定权值。 |
| `gnss_dd_code` | 双差伪距组 | 组内共用 pivot 相关矩阵作为整体 D，不把 DD 行当独立通道。 |
| `gnss_dd_phase_L1/L2` | 双差载波相位组 | 模糊度模型、周跳和连续性通过后再估计；按频率/星座分组需有足够冗余度。 |
| `tdcp_L1` | L1 time-differenced DDCP | 初期在缺少 RTK-TDCP 跨协方差时仅记录联合 NIS/诊断；不与 RTK position 独立自适应调权。 |
| `zupt`, `nhc`, `zpc`, `zhc` | 运动学伪观测 | 单独记录残差与触发上下文；因误触发可能代表模型/分类错误，首期不自动估计其噪声倍率。 |
| `process_jerk`, `process_angular_acceleration` | GMIS 运动过程噪声 | 只做接口预留；待过程残差及状态可观测性方法评审后再开放 Q 调权。 |

### 5.2 相关性与可辨识性要求

- 共用 pivot 的 GNSS 双差噪声具有非对角相关项；`D_i` 必须继承由差分算子计算出的完整协方差。
- RTK position 与 TDCP 可能共享 GNSS 原始观测；在交叉协方差未知时，不可各自独立估一个 variance factor 再同时反馈到 R。可先将其作为一组诊断，或等待联合/经验交叉协方差建模。
- 连续 TDCP epoch 共享相位历元，窗口样本存在时间相关性。首期 window 估计需选择不重叠样本、采用有效样本数修正，或扩展为跨历元协方差；简单把每秒样本数当独立 DOF 会过度自信。
- IMU 残差也可能受温漂、振动与滤波状态误差产生的时间相关影响。VCE 输出的 `λ` 是当前模型条件下的经验方差尺度，不等价于完整的 Allan 噪声参数辨识。
- 分组设计须检查 `D_i` 线性独立性、窗口累计冗余度和各组因子的可辨识性；相似误差源可能只能作为一个合并分量估计。

## 6. 配置设计

VCE 使用单独 JSON 文件，由 GMIS 模式配置引用。阶段 A 设为诊断模式；不启用时，原 GMIS 处理路径和量测协方差必须保持不变。

```json
{
  "module": "gmis_vce_v1",
  "enabled": true,
  "mode": "diagnostic_only",
  "window": {
    "type": "duration_seconds",
    "duration_seconds": 30.0,
    "minimum_valid_epochs": 20,
    "minimum_total_redundancy": 5.0,
    "smoothing": "log_ewma",
    "smoothing_factor": 0.15
  },
  "factor_limits": {
    "minimum": 0.25,
    "maximum": 16.0,
    "maximum_relative_change_per_update": 0.25
  },
  "components": [
    {
      "id": "imu_gyro",
      "measurement_model_id": "imu_raw_gyro_accel_v1",
      "row_labels": ["gyro_x", "gyro_y", "gyro_z"],
      "prior_variance_factor": 1.0,
      "use_for_R_adaptation": false
    },
    {
      "id": "imu_accel",
      "measurement_model_id": "imu_raw_gyro_accel_v1",
      "row_labels": ["accel_x", "accel_y", "accel_z"],
      "prior_variance_factor": 1.0,
      "use_for_R_adaptation": false
    },
    {
      "id": "rtk_position",
      "measurement_model_id": "rtk_position_loose_v1",
      "row_labels": ["north", "east", "down"],
      "prior_variance_factor": 1.0,
      "use_for_R_adaptation": false
    }
  ],
  "diagnostics": {
    "write_epoch_csv": true,
    "write_window_csv": true,
    "nis_policy": "existing_filter_policy"
  }
}
```

说明：JSON 中 component 行标识映射必须由实际 MeasurementBatch 确认，不能只依赖固定行号；TC/LTC 变长卫星集需由 row label（measurement type、PRN、频率、pivot、epoch）动态映射。阶段 B 才允许把 `use_for_R_adaptation` 开启，并需要配置明确的倍率上下限、变化率、失效回退值和适用模式。

## 7. 日志与报告

建议每个历元输出 `vce_epoch.csv`，每个窗口输出 `vce_window.csv`。

**逐历元字段：**

`gps_seconds, component_id, measurement_model_id, row_count, valid, update_accepted, innovation_norm, residual_quadratic, redundancy, epoch_factor, window_factor, used_factor, prior_factor, status, status_reason`

**窗口汇总字段：**

`window_start, window_end, component_id, valid_epochs, rejected_epochs, sum_quadratic, sum_redundancy, raw_factor, smoothed_factor, applied_factor, factor_clipped, effective_dof, diagnostics`

报表至少包含：每组先验 σ、估计 λ 及等效 σ 比例 `sqrt(λ)`、时间序列、累计有效冗余度、创新 NIS、标准化后验残差分布、剔除/无效样本数、N/E/D 或 IMU 三轴分组统计。所有“RMS/STD”需注明是形式误差、后验残差统计或真值误差；VCE 本身不提供真值轨迹精度认证。

## 8. 错误处理与运行安全

- 对非有限输入、维数错误、非对称/非正定 D/R、LDLT 分解失败：该分量本窗口标记无效，不发布新倍率。
- `rᵢ≤r_min`、窗口总冗余度不足、观测组没有变化或条件数过差：保留前一个有效倍率（阶段 A 仍显示不可估计状态）。
- 观测被 NIS gating 拒绝或数据预处理判周跳/无效：默认不参与 VCE 累积，并分别记录被拒绝原因。
- 估计倍率超限：日志同时保留未裁剪因子和裁剪后候选因子；不得静默截断。
- 不允许 VCE 产生零方差、负方差或 NaN；适配前检查最终 R 对称正定。
- VCE 模块异常时 fail-safe 回退至配置的先验 R/Q，并保持导航解算继续运行；诊断状态须指明回退。
- 重放时固定输入、配置、代码版本和窗口定义，确保不启用 VCE 与现有 GMIS 基线结果一致。

## 9. 实施计划与验收条件

### 阶段 0：接口及统计定义评审

确定更新批次粒度、哪些模型能联合更新、Dᵢ 分组、时间相关处理以及是否扩展 `EstimatorUpdateResult`。通过线性小型仿真验证 `rᵢ` 和方差因子公式。

### 阶段 1：诊断型 VCE（首版已实现，仍需持续统计评审）

已实现 `VceConfig`、`VceEngine` 和在线 CSV 输出，并接入 GMIS-LC 的 IMU gyro/accel 与 RTK position/velocity 更新。VCE 只消费已接受更新的后验残差和残差协方差；任何方差因子都不反馈到滤波器。后续仍需用已知方差合成数据进一步验证 Helmert 因子估计和边界情况。

验收：合成数据中已知的 `λᵢ` 可在配置误差范围内恢复；低冗余窗口被标记不可估；有相关性的 DD 子块保留非对角项；开启 diagnostic-only 与关闭 VCE 的导航输出逐样本一致。

### 阶段 2：影子调权回放

在离线数据中计算“若应用当前因子”的候选 R，但仍不反馈到滤波器。对比 NIS、后验残差、协方差一致性、RTK/参考结果（如有）和各动态状态区间，并调好窗口、边界、平滑及延迟。

### 阶段 3：GMIS-LC 受控 R 调权

优先只开放经过诊断的单一独立分量，例如 `rtk_position`，默认允许上调噪声而不允许自动降到低于先验。固定数据集对比基线与自适应结果；若解发散、协方差不一致、因子频繁触顶或与 NIS 冲突，立即回退先验 R。

### 阶段 4：扩展 TC/LTC、IMU 轴向及 Q

逐类添加 TC code/phase、LTC RTK/TDCP 和 IMU gyro/accel 分量，先解决跨观测与跨历元相关性；最后评估过程噪声分量及在线 FDE。每种 mode 独立配置和验证，不跨 mode 共用累计历史。

**阶段 B 的调权验收：** 与诊断版对比，导航没有非有限值或明显跳变；VCE 因子能解释创新/残差变化且稳定；NIS/后验残差改善或保持在合理范围；RTK/独立真值评估不退化；关闭 VCE 可精确复现基线；所有倍率及状态都可从日志重放。

## 10. 首版实现状态

- 通用 `EstimatorUpdateResult` 增加线性化后验观测残差及其协方差字段；被拒绝的更新不提供后验残差统计。
- `nav::gmis::vce::VceEngine` 支持配置观测组、滑动时间窗口、最小有效样本/总冗余度和对数域 EWMA 平滑；VCE 仍不依赖 GMIS 21/42 维状态布局。
- `gmis_lc_app` 在每次接受的 IMU/RTK 更新后在线累计 `imu_gyro`、`imu_accel`、`rtk_position` 和有效的 `rtk_velocity` 统计，并写入 `runtime.vce_diagnostics_file` 指定的 CSV。
- 当前 JSON 将 VCE 设为 `diagnostic_only` 且 `apply_factors=false`。解析器拒绝任何启用协方差反馈的配置；CSV 的 `applied_to_R` 恒为 0。
- 同一配置下诊断开启/关闭两次回放产生完全相同的 GMIS-LC solution CSV（SHA-256 相同），证实首版 VCE 不改变导航输出。
- 目前只接入 GMIS-LC。TC/LTC、跨观测/历元相关分量、合成真值方差验证、R 调权和 Q 调权仍未实现。

## 11. 对当前方案的结论

VCE 与 GMIS 的架构匹配：GMIS 让 IMU、RTK、双差 GNSS、TDCP 和约束分别具有明确的观测模型，FilterManager 可以按模型把残差送入独立分量估计。但第一版应该先实现“**可靠的后验残差统计与噪声模型评估**”，而不是立即改变 Kalman 权值。

推荐的首个工程目标为：**GMIS-LC diagnostic-only VCE，按可辨识的联合观测组进行窗口估计，并保持导航输出不变**。等其模拟验证和残差统计通过，再开启对 RTK 位置 R 的受控自适应。GMIS-LTC 的 RTK 位置与 TDCP、GMIS-TC 的相关双差观测，需在交叉协方差问题解决后再独立定权。该顺序优先保证滤波器行为可回归、统计含义可解释。

## 12. 参考材料

1. `VCE in GMIS Framework.pdf`：VCE 基本背景、方差分量、冗余度和 GMIS 反馈架构。
2. `vce-gmis-theoretical-guide.pdf`：GMIS 全状态滤波中的测量残差、过程残差、Helmert VCE 和多历元窗口理论。
3. `vce-gmis-code-design-spec.md`：VCE C++ 模块划分、数值保护、滑窗及工程接口草案。
4. 本工程 `GMIS_GPS_IMU_Software_Design.md`：GMIS 状态、模型、模式边界和接口策略。
5. 本工程 `ins_rtk_nav/include/navcore/state_estimator.hpp` 与 `ins_rtk_nav/src/navigation/state_estimator.cpp`：当前通用状态估计器接口及更新诊断能力。

> 参考材料描述的功能是 VCE 设计输入，不代表 VCE 模块已进入当前代码或已经验证运行。
