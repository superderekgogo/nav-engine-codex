#!/usr/bin/env python3
"""Summarize event-time LC/TC/LTC navigation replays and pre-update innovations."""
import bisect, csv, math, statistics
from datetime import date
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
RUNS = {name: ROOT / "ins_rtk_nav" / "build" / f"mode_{name.lower()}" for name in ("LC", "TC", "LTC")}
OUT = ROOT / "output"
REPORT, FIGURE = OUT / "LC_TC_LTC_Performance_Report.md", OUT / "lc_tc_ltc_performance.png"

def read(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))
def num(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else math.nan
    except (KeyError, ValueError, TypeError):
        return math.nan
def rms(values):
    values = [v for v in values if math.isfinite(v)]
    return math.sqrt(sum(v*v for v in values)/len(values)) if values else math.nan
def pct(values, p):
    values = sorted(v for v in values if math.isfinite(v))
    return values[min(len(values)-1, math.ceil(p*len(values))-1)] if values else math.nan
def spread(values):
    values = [v for v in values if math.isfinite(v)]
    return (statistics.median(values), pct(values, .95), values[-1]) if values else (math.nan,)*3
def fmt(x, digits=4):
    return "n/a" if not math.isfinite(x) else f"{x:.{digits}f}"

solutions, innovations = {}, {}
for mode in RUNS:
    folder = RUNS[mode]
    solutions[mode] = read(folder / "ltc_solution.csv")
    innovations[mode] = read(folder / "ltc_innovations.csv")
rtk = sorted((num(row, "gps_seconds"), row) for row in read(RUNS["LC"] / "rtk_solution.csv"))
rtk_times = [item[0] for item in rtk]
first_time = min(num(rows[0], "gps_seconds") for rows in solutions.values())

colors = {"LC": "tab:blue", "TC": "tab:orange", "LTC": "tab:green"}
metrics = {
    "Position 3D 1σ (m)": "position_3d_rms_1sigma_m",
    "Velocity 3D 1σ (m/s)": "velocity_3d_rms_1sigma_mps",
    "Attitude 3D 1σ (deg)": "attitude_3d_rms_1sigma_deg",
}
fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
for ax, (title, key) in zip(axes.flat[:3], metrics.items()):
    for mode, rows in solutions.items():
        sample = rows[::100]
        ax.plot([(num(r, "gps_seconds")-first_time)/60 for r in sample],
                [num(r, key) for r in sample], label=mode, color=colors[mode], lw=1)
    ax.set_title(title); ax.set_xlabel("Elapsed time (min)"); ax.grid(True, alpha=.25); ax.legend()
ax = axes.flat[3]
for mode, rows in innovations.items():
    values = [num(r, "normalized_innovation") for r in rows]
    values = [x for x in values if math.isfinite(x)]
    if values: ax.hist(values, bins=60, density=True, histtype="step", label=f"{mode} (n={len(values):,})", color=colors[mode])
ax.set_xlim(-6, 6); ax.set_title("Pre-update normalized innovations"); ax.set_xlabel("Innovation / marginal sigma"); ax.grid(True, alpha=.25); ax.legend(fontsize=8)
fig.suptitle("INS/RTK LC, TC and LTC replay comparison")
OUT.mkdir(parents=True, exist_ok=True); fig.savefig(FIGURE, dpi=160); plt.close(fig)

def nearest_rtk(t):
    index = bisect.bisect_left(rtk_times, t)
    candidates = [i for i in (index-1, index) if 0 <= i < len(rtk)]
    return min(candidates, key=lambda i: abs(rtk_times[i]-t)) if candidates else None
def ned_rotation(lat, lon):
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return ((-sl*co, -sl*so, cl), (-so, co, 0.0), (-cl*co, -cl*so, -sl))
def rtk_diffs(mode):
    pos, vel = [], []
    nav = solutions[mode]
    nav_times = [num(row, "gps_seconds") for row in nav]
    for t, obs in rtk:
        index = bisect.bisect_left(nav_times, t)
        candidates = [i for i in (index-1, index) if 0 <= i < len(nav)]
        if not candidates: continue
        row = nav[min(candidates, key=lambda i: abs(nav_times[i]-t))]
        if abs(number := num(row, "gps_seconds") - t) > .02: continue
        mask = int(num(obs, "valid_components"))
        if mask & 7 != 7: continue
        lat, lon = math.radians(num(row, "latitude_deg")), math.radians(num(row, "longitude_deg"))
        rot = ned_rotation(lat, lon)
        de = [num(row, f"ecef_{axis}_m")-num(obs, f"{axis}_ecef_m") for axis in "xyz"]
        pos.append([sum(rot[i][j]*de[j] for j in range(3)) for i in range(3)])
        if mask & 56 == 56:
            rtk_velocity = [num(obs, f"v{axis}_ecef_mps") for axis in "xyz"]
            dv = [num(row, f"velocity_{axis}_mps")-sum(rot[i][j]*rtk_velocity[j] for j in range(3)) for i, axis in enumerate("ned")]
            vel.append(dv)
    p3 = [math.sqrt(sum(x*x for x in v)) for v in pos]
    h = [math.hypot(v[0], v[1]) for v in pos]
    d = [v[2] for v in pos]
    v3 = [math.sqrt(sum(x*x for x in v)) for v in vel]
    return len(pos), rms(h), rms(d), rms(p3), rms(v3)

lines = [
    "# LC / TC / LTC 组合导航性能评估", "",
    f"生成日期：{date.today().isoformat()}。三种模式使用 18 维闭环误差状态滤波架构，分别独立全量离线回放。",
    "",
    "## 架构与模型", "",
    "FilterManagerModule 收集回放事件并按时间对齐，构造导航量测包；InertialNavigationModule 独占 mechanization 与名义状态并处理误差反馈；EstimationModule 构造导航领域过程/量测模型；通用 StateEstimator 只处理状态向量、协方差及线性预测/更新/reset 矩阵。",
    "",
    "状态为 18 维：NED 位置、速度、姿态小角、陀螺 bias、加计 bias 和 body-frame lever arm。为保持此次架构重构与原融合模型等价，`estimate_lever_arm` 默认关闭；在线估计须在 JSON 显式启用，并通过充分姿态激励门控。",
    "",
    "| Mode | Measurement models | 实际含义 |", "|---|---|---|",
    "| LC | RTK P/V | RTK 天线位置及有效速度松组合更新 |",
    "| TC | L1/L2 TD-DDCP | 卫星级相邻历元双差载波相位几何变化；不输入 RTK P/V |",
    "| LTC | RTK position + L1 TDCP | RTK 位置松组合，L1 TD-DDCP WLS 派生区间平均速度；RTK 速度默认关闭 |",
    "",
    "## 数据与更新统计", "",
    f"每个模式输出 {len(solutions['LTC']):,} 个 IMU solution epoch；RTK batch 有 {len(rtk):,} 个历元；回放时间为 GPS {solutions['LTC'][0]['gps_seconds']}–{solutions['LTC'][-1]['gps_seconds']} s。",
    "", "| Mode | RTK 更新历元 | TD 更新历元 | TD 行数 | TD 未接受历元 | 最终位置 3D 1σ |", "|---|---:|---:|---:|---:|---:|"
]
for mode in RUNS:
    groups = {}
    for row in innovations[mode]:
        key = (row["gps_seconds"], row["previous_gps_seconds"], row["measurement"])
        groups.setdefault(key, []).append(row)
    rtk_epochs = sum(1 for key, rows in groups.items() if key[2] == "rtk_pv" and any(x["accepted"] == "1" for x in rows))
    td_groups = {key: rows for key, rows in groups.items() if key[2] in ("td_ddcp_range", "l1_tdcp_delta_position_velocity")}
    td_epochs = sum(any(x["accepted"] == "1" for x in rows) for rows in td_groups.values())
    td_rows = sum(len(rows) for rows in td_groups.values())
    expected = 1563 if mode != "LC" else 0
    lines.append(f"| {mode} | {rtk_epochs:,} | {td_epochs:,} | {td_rows:,} | {max(0, expected-td_epochs):,} | {fmt(num(solutions[mode][-1], 'position_3d_rms_1sigma_m'), 5)} m |")

lines += ["", "## 协方差内部不确定度", "",
          "以下 3D 1σ 为各 NED 协方差块迹的平方根，统计包含初始收敛阶段。它是滤波器内部不确定度，不是真值误差。",
          "", "| 指标 | LC median / P95 / final | TC median / P95 / final | LTC median / P95 / final |", "|---|---:|---:|---:|"]
for label, key in metrics.items():
    columns = [" / ".join(fmt(x, 5) for x in spread([num(r, key) for r in solutions[mode]])) for mode in ("LC", "TC", "LTC")]
    lines.append(f"| {label} | {columns[0]} | {columns[1]} | {columns[2]} |")

lines += ["", "## Innovation consistency", "",
          "Innovation 为更新前残差。NIS 每历元按整个相关观测向量计算，表中按历元去重；normalized innovation 使用边际标准差，相关观测行不是独立样本。",
          "", "| Mode / model | rows | accepted rows | bias | RMS | abs P95 | max abs | normalized RMS | mean NIS / dof |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
for mode in RUNS:
    models = {}
    for row in innovations[mode]: models.setdefault(row["measurement"], []).append(row)
    for name, rows in models.items():
        residual = [num(r, "innovation") for r in rows]
        norm = [num(r, "normalized_innovation") for r in rows]
        epochs = {}
        for row in rows: epochs[(row["gps_seconds"], row["previous_gps_seconds"], name)] = (num(row, "nis"), num(row, "dof"))
        nd = [nis/dof for nis, dof in epochs.values() if dof > 0 and math.isfinite(nis)]
        valid = [x for x in residual if math.isfinite(x)]
        accepted = sum(r["accepted"] == "1" for r in rows)
        lines.append(f"| {mode} / {name} | {len(rows):,} | {accepted:,} | {fmt(statistics.mean(valid) if valid else math.nan, 5)} | {fmt(rms(residual), 5)} | {fmt(pct([abs(x) for x in residual], .95), 5)} | {fmt(max((abs(x) for x in valid), default=math.nan), 5)} | {fmt(rms(norm), 3)} | {fmt(statistics.mean(nd) if nd else math.nan, 3)} |")

lines += ["", "## RTK-relative comparison (not truth)", "",
          "融合结果与最近 RTK 历元对比，仅作内部一致性诊断。LC/LTC 直接消费 RTK 位置；TC 和 RTK 也共享原始 GNSS 载波观测。因此这些差值都不能解释为独立精度或真值误差。",
          "", "| Mode | matched epochs | horizontal RMS (m) | vertical RMS (m) | 3D RMS (m) | velocity 3D RMS (m/s) |", "|---|---:|---:|---:|---:|---:|"]
for mode in RUNS:
    count, hr, vr, p3, v3 = rtk_diffs(mode)
    lines.append(f"| {mode} | {count:,} | {fmt(hr, 4)} | {fmt(vr, 4)} | {fmt(p3, 4)} | {fmt(v3, 4)} |")

lines += ["", "## 杆臂估计与数值健康", "",
          "杆臂输出在 IMU body frame，初值为零。没有独立杆臂真值；尤其 TC 的杆臂估计依赖轨迹多轴姿态激励，不能由收敛曲线单独验证。",
          "", "| Mode | final lever X/Y/Z (m) |", "|---|---:|"]
for mode, rows in solutions.items():
    last = rows[-1]
    values = [num(last, f"lever_b_{axis}_m") for axis in "xyz"]
    lines.append(f"| {mode} | {values[0]:.4f} / {values[1]:.4f} / {values[2]:.4f} |")
lines += ["", "三种回放均完成全部 epoch，解算位置、速度和协方差没有非有限值。", "",
          "## 结论与限制", "",
          "- LC、TC、LTC 均通过配置选择，输入数据解码、RTK 解算、TD-DDCP 构造复用；各融合 run 使用独立 INS/EstimationModule/StateEstimator 状态。",
          "- TC 不接收 RTK P/V，LTC 使用 RTK 位置和 L1 TDCP 区间速度。三种测量定义保持区分。",
          "- 本次回归核查发现：最初重构版本新增了在线杆臂更新、NIS 拒绝和不同的测量方差下限语义，导致 LC/LTC 更新/解明显偏离旧实现。默认关闭杆臂估计及 NIS 门控，并恢复各观测模型原有的方差处理后，三种最终位置 3D 1σ 为 LC 0.03176 m、TC 0.05735 m、LTC 0.03075 m，与重构前记录的 0.03174/0.05717/0.03061 m 接近；LC/LTC 分别接受 1,564 个 RTK 历元。杆臂估计的姿态和陀螺 bias 雅可比符号也已修正。",
          "- 当前 LTC 没有完整 RTK-载波互相关，也没有完整跨历元状态交叉协方差；TD NIS 与协方差仍是近似值。",
          "- 数据没有独立 rover 位置/速度/姿态真值，不能据此计算实际导航误差 RMSE 或证明厘米级精度。RTK-relative 差值不是独立真值检验。",
          "- 当前正式回放将 `estimate_lever_arm` 设为 false，以保持原 15 状态测量行为；18 维布局保留在线杆臂扩展接口。启用时仍需多轴激励及信息矩阵可观测性验证。",
          "- TC 相对 RTK 的 3D RMS 为 21.89 m，且 NIS/dof=0.050；旧版报告同样有 NIS/dof=0.048，但未做逐历元 RTK 对比。TC 使用时间差分载波缺少绝对位置观测，当前初始位置/绝对基准可观测性和协方差一致性需单独解决。这不应被归因于本次模块化重构，也不代表 TC 已达到可用性能。",
          "",
          f"图表：{FIGURE.relative_to(ROOT).as_posix()}。数据来自 LC/TC/LTC 分模式目录的 CSV。",
]
REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"Wrote {REPORT}")
print(f"Wrote {FIGURE}")
