from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1].parent
BASE = ROOT / "output" / "zupt_nhc_baseline"
NEW = ROOT / "ins_rtk_nav" / "build"
OUT = ROOT / "output"
MODES = ("lc", "tc", "ltc")
METRICS = {
    "position_3d_rms_1sigma_m": "位置 3D 1σ (m)",
    "velocity_3d_rms_1sigma_mps": "速度 3D 1σ (m/s)",
    "attitude_3d_rms_1sigma_deg": "姿态 3D 1σ (deg)",
}


def summarize_solution(path: Path) -> dict[str, dict[str, float]]:
    data = pd.read_csv(path)
    result = {}
    for column, label in METRICS.items():
        values = data[column].to_numpy(float)
        result[label] = {"median": float(np.nanmedian(values)), "p95": float(np.nanpercentile(values, 95)), "final": float(values[-1])}
    for column, label in (("roll_rms_1sigma_deg", "横滚 1σ (deg)"), ("pitch_rms_1sigma_deg", "俯仰 1σ (deg)"), ("heading_rms_1sigma_deg", "航向 1σ (deg)")):
        values = data[column].to_numpy(float)
        result[label] = {"median": float(np.nanmedian(values)), "p95": float(np.nanpercentile(values, 95)), "final": float(values[-1])}
    return result


def innovation_summary(path: Path, mode: str) -> dict[str, dict[str, float]]:
    data = pd.read_csv(path)
    data = data[data["mode"] == mode]
    result = {}
    for measurement, group in data.groupby("measurement"):
        if measurement not in {"rtk_pv", "td_ddcp_range", "l1_tdcp_delta_position_velocity", "zupt", "nhc"}:
            continue
        epochs = group.groupby("gps_seconds", sort=False)
        nis = epochs["nis"].first().to_numpy(float)
        dof = epochs["dof"].first().to_numpy(float)
        result[measurement] = {
            "rows": int(len(group)), "epochs": int(group["gps_seconds"].nunique()),
            "accepted_epochs": int(group[group["accepted"] == 1]["gps_seconds"].nunique()),
            "rejected_epochs": int(group[group["accepted"] == 0]["gps_seconds"].nunique()),
            "innovation_rms": float(np.sqrt(np.mean(np.square(group["innovation"].to_numpy(float))))),
            "normalized_rms": float(np.sqrt(np.mean(np.square(group["normalized_innovation"].to_numpy(float))))),
            "mean_nis_dof": float(np.mean(nis / dof)), "median_nis_dof": float(np.median(nis / dof)),
        }
    return result


def relative_to_rtk(solution_path: Path, rtk_path: Path) -> dict[str, float]:
    nav = pd.read_csv(solution_path)
    rtk = pd.read_csv(rtk_path)
    t = nav["gps_seconds"].to_numpy(float)
    rt = rtk["gps_seconds"].to_numpy(float)
    common = (t >= rt[0]) & (t <= rt[-1])
    delta = np.column_stack([
        nav[f"ecef_{axis}_m"].to_numpy(float)[common] - np.interp(t[common], rt, rtk[f"x_ecef_m" if axis == "x" else f"y_ecef_m" if axis == "y" else "z_ecef_m"].to_numpy(float))
        for axis in "xyz"
    ])
    lat = np.deg2rad(nav["latitude_deg"].to_numpy(float)[common])
    lon = np.deg2rad(nav["longitude_deg"].to_numpy(float)[common])
    # Local NED rotation at each solution epoch; ECEF delta is rotated into NED.
    north = -np.sin(lat) * np.cos(lon) * delta[:, 0] - np.sin(lat) * np.sin(lon) * delta[:, 1] + np.cos(lat) * delta[:, 2]
    east = -np.sin(lon) * delta[:, 0] + np.cos(lon) * delta[:, 1]
    down = -np.cos(lat) * np.cos(lon) * delta[:, 0] - np.cos(lat) * np.sin(lon) * delta[:, 1] - np.sin(lat) * delta[:, 2]
    horizontal = np.hypot(north, east)
    return {"epochs": int(common.sum()), "horizontal_rms": float(np.sqrt(np.mean(horizontal**2))),
            "vertical_rms": float(np.sqrt(np.mean(down**2))), "three_d_rms": float(np.sqrt(np.mean(horizontal**2 + down**2)))}


def fmt(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    summaries = {}
    innovations = {}
    baseline_innovations = {}
    rel = {}
    for mode in MODES:
        baseline = BASE / mode
        constrained = NEW / f"mode_{mode}_constraints"
        summaries[(mode, "baseline")] = summarize_solution(baseline / "ltc_solution.csv")
        summaries[(mode, "constraints")] = summarize_solution(constrained / "ltc_solution.csv")
        innovations[mode] = innovation_summary(constrained / "ltc_innovations.csv", mode)
        baseline_innovations[mode] = innovation_summary(baseline / "ltc_innovations.csv", mode)
        rel[(mode, "baseline")] = relative_to_rtk(baseline / "ltc_solution.csv", baseline / "rtk_solution.csv")
        rel[(mode, "constraints")] = relative_to_rtk(constrained / "ltc_solution.csv", constrained / "rtk_solution.csv")

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    for ax, (column, label) in zip(axes, METRICS.items()):
        plotted = []
        for mode in MODES:
            b = pd.read_csv(BASE / mode / "ltc_solution.csv")
            n = pd.read_csv(NEW / f"mode_{mode}_constraints" / "ltc_solution.csv")
            ax.plot(b["gps_seconds"], b[column], linestyle="--", alpha=0.65, label=f"{mode.upper()} baseline")
            ax.plot(n["gps_seconds"], n[column], linewidth=1.2, label=f"{mode.upper()} + ZUPT/NHC")
            plotted.extend([b[column].to_numpy(float), n[column].to_numpy(float)])
        plot_labels = {
            "位置 3D 1σ (m)": "Position 3D 1σ (m)",
            "速度 3D 1σ (m/s)": "Velocity 3D 1σ (m/s)",
            "姿态 3D 1σ (deg)": "Attitude 3D 1σ (deg)",
        }
        ax.set_ylabel(plot_labels[label])
        ax.set_ylim(0, float(np.nanpercentile(np.concatenate(plotted), 99.5)) * 1.08)
        ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel("GPS seconds")
    axes[0].legend(ncol=3, fontsize=8)
    fig.suptitle("LC / TC / LTC covariance uncertainty: baseline vs ZUPT/NHC")
    plot_path = OUT / "ZUPT_NHC_Performance.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    lines = [
        "# ZUPT / NHC 组合导航性能对比", "",
        "LC、TC、LTC 三种模式均使用同一批 158,328 个 IMU 样本和 GNSS 数据进行回放。Baseline 使用原有观测模型配置；约束组通过 JSON 增加 ZUPT 与 NHC。solution 输出频率为 1 Hz。", "",
        "> 下列位置、速度、姿态 RMS 为滤波器协方差计算的 1σ 不确定度，不是真值 RMSE。该 rover 数据没有独立的位置、速度或姿态真值。RTK-relative 指标仅用于内部一致性诊断，因为 LC/LTC 使用 RTK，TC 与 RTK 共享原始 GNSS 载波数据。", "",
        "## 滤波器协方差不确定度", "",
        "每个数值为 Baseline → 启用约束（相对变化）。", "",
        "| 指标 | 模式 | Median | P95 | Final |", "|---|---|---:|---:|---:|",
    ]
    for label in METRICS.values():
        for mode in MODES:
            b, n = summaries[(mode, "baseline")][label], summaries[(mode, "constraints")][label]
            values = []
            for key in ("median", "p95", "final"):
                pct = (n[key] / b[key] - 1.0) * 100 if b[key] else float("nan")
                values.append(f"{fmt(b[key])} → {fmt(n[key])} ({pct:+.1f}%)")
            lines.append(f"| {label} | {mode.upper()} | {values[0]} | {values[1]} | {values[2]} |")
    lines += ["", "### 横滚 / 俯仰 / 航向协方差 1σ", "", "| 模式 / 姿态角 | Median baseline → 约束组 | P95 baseline → 约束组 | Final baseline → 约束组 |", "|---|---:|---:|---:|"]
    for mode in MODES:
        for angle in ("横滚 1σ (deg)", "俯仰 1σ (deg)", "航向 1σ (deg)"):
            b, n = summaries[(mode, "baseline")][angle], summaries[(mode, "constraints")][angle]
            lines.append(f"| {mode.upper()} / {angle.split()[0]} | {fmt(b['median'])} → {fmt(n['median'])} | {fmt(b['p95'])} → {fmt(n['p95'])} | {fmt(b['final'])} → {fmt(n['final'])} |")
    lines += ["", "### GNSS / TDCP innovation 变化", "", "对比启用约束前后原有导航量测残差的统计量。", "", "| 模式 / 量测 | 行数 baseline → 约束组 | RMS baseline → 约束组 | 归一化 RMS baseline → 约束组 | Mean NIS/dof baseline → 约束组 |", "|---|---:|---:|---:|---:|"]
    for mode in MODES:
        for measurement in ("rtk_pv", "td_ddcp_range", "l1_tdcp_delta_position_velocity"):
            b = baseline_innovations[mode].get(measurement)
            n = innovations[mode].get(measurement)
            if b and n:
                lines.append(f"| {mode.upper()} / {measurement} | {b['rows']} → {n['rows']} | {fmt(b['innovation_rms'])} → {fmt(n['innovation_rms'])} | {fmt(b['normalized_rms'])} → {fmt(n['normalized_rms'])} | {fmt(b['mean_nis_dof'], 3)} → {fmt(n['mean_nis_dof'], 3)} |")
    lines += ["", "## 约束触发与 innovation", "", "RMS 和归一化 RMS 按观测行计算；不同分量及跨历元重复伪量测可能相关。NIS/dof 按每个更新历元仅计一次。", "", "| 模式 / 约束 | 历元数 | 接受 | 拒绝 | Innovation RMS | 归一化 RMS | Mean NIS/dof |", "|---|---:|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        for measurement in ("zupt", "nhc"):
            stat = innovations[mode].get(measurement)
            if not stat:
                lines.append(f"| {mode.upper()} / {measurement.upper()} | 0 | 0 | 0 | — | — | — |")
            else:
                lines.append(f"| {mode.upper()} / {measurement.upper()} | {stat['epochs']} | {stat['accepted_epochs']} | {stat['rejected_epochs']} | {fmt(stat['innovation_rms'])} m/s | {fmt(stat['normalized_rms'])} | {fmt(stat['mean_nis_dof'], 3)} |")
    lines += ["", "## 相对 RTK 解的差值（非真值）", "", "| 模式 | Baseline 3D RMS | 约束组 3D RMS | Baseline 水平 / 垂直 | 约束组水平 / 垂直 |", "|---|---:|---:|---:|---:|"]
    for mode in MODES:
        b, n = rel[(mode, "baseline")], rel[(mode, "constraints")]
        lines.append(f"| {mode.upper()} | {fmt(b['three_d_rms'])} m | {fmt(n['three_d_rms'])} m | {fmt(b['horizontal_rms'])} / {fmt(b['vertical_rms'])} m | {fmt(n['horizontal_rms'])} / {fmt(n['vertical_rms'])} m |")
    lines += ["", "## 结论与限制", "",
        "- 动态检测器使用 IMU 滑窗特征（角速率模长 RMS、比力模长波动）；RTK 速度协方差有效时将其作为速度证据，否则 INS 速度仅作诊断。检测器采用迟滞和驻留时间，并抑制高动态历元。当前阈值是针对本数据的初始调参，尚未通过标注数据独立验证。",
        "- ZUPT 是 NED 下三维零速度伪量测；NHC 是 FRD 下侧向/向下速度为零的二维伪量测，不约束前向速度。两者使用非零协方差、NIS 门限，最大更新频率配置为 1 Hz。",
        "- 协方差下降不能证明真实精度提升。约束 innovation 相对预测协方差较小（见归一化 RMS 与 NIS/dof）；约束与 RTK/TDCP、以及不同历元间的伪量测可能相关。当前实现尚未建模这些交叉相关。需用带静止/运动标签的数据和独立真值校准检测阈值、R 矩阵、参考点假设与 innovation 一致性，再把结果用于性能结论。",
        "- Zero_Position_Change 与 Zero_Heading_Change 本次未启用。估计器尚未表示静止锚点及其与当前状态的历元间交叉协方差；若把历史锚点当成精确真值会造成统计错误。",
        "- 本版 NHC 假定车辆 FRD 与 IMU FRD 重合，且 IMU 速度参考点就是车辆约束参考点（杆臂为零）。若实际平台存在安装旋转或参考点偏移，应先补齐标定参数及补偿模型。",
        "", f"协方差时间序列图：[ZUPT_NHC_Performance.png](ZUPT_NHC_Performance.png)。为显示初始收敛后的细节，图中纵轴上限取 99.5 分位数；启动阶段完整峰值仍计入表格统计。", ""]
    report_path = OUT / "ZUPT_NHC_Performance_Report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(report_path)
    print(plot_path)


if __name__ == "__main__":
    main()
