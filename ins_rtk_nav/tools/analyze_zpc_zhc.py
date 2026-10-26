from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output"
BASELINE = OUT / "zupt_nhc_baseline"
BUILD = ROOT / "ins_rtk_nav" / "build"
MODES = ("lc", "tc", "ltc")
SCENARIOS = ("baseline", "zupt_nhc", "zpc_zhc")
LABELS = {"baseline": "Baseline", "zupt_nhc": "ZUPT/NHC", "zpc_zhc": "ZUPT/NHC + ZPC/ZHC"}
METRICS = {
    "position_3d_rms_1sigma_m": ("Position 3D 1σ", "m"),
    "velocity_3d_rms_1sigma_mps": ("Velocity 3D 1σ", "m/s"),
    "attitude_3d_rms_1sigma_deg": ("Attitude 3D 1σ", "deg"),
    "heading_rms_1sigma_deg": ("Heading 1σ", "deg"),
}


def solution_path(mode: str, scenario: str) -> Path:
    if scenario == "baseline":
        return BASELINE / mode / "ltc_solution.csv"
    folder = "constraints" if scenario == "zupt_nhc" else "anchor_constraints"
    return BUILD / f"mode_{mode}_{folder}" / "ltc_solution.csv"


def innovation_path(mode: str, scenario: str) -> Path:
    return solution_path(mode, scenario).with_name("ltc_innovations.csv")


def summarize(data: pd.DataFrame) -> dict[str, dict[str, float]]:
    return {column: {"median": float(data[column].median()),
                     "p95": float(data[column].quantile(.95)),
                     "final": float(data[column].iloc[-1])}
            for column in METRICS}


def innovation_stats(path: Path, mode: str) -> dict[str, dict[str, float]]:
    data = pd.read_csv(path)
    data = data[(data["mode"] == mode) & data["measurement"].isin(["zupt", "nhc", "zpc", "zhc", "zhc_lock_projection"])]
    result = {}
    for name, group in data.groupby("measurement"):
        epochs = group.groupby("gps_seconds", sort=False).first()
        result[name] = {
            "epochs": int(len(epochs)),
            "accepted": int((epochs["accepted"] == 1).sum()),
            "rejected": int((epochs["accepted"] == 0).sum()),
            "rms": float(np.sqrt(np.mean(group["innovation"].to_numpy(float) ** 2))),
            "normalized_rms": float(np.sqrt(np.mean(group["normalized_innovation"].to_numpy(float) ** 2))),
            "mean_nis_dof": float(np.mean(epochs["nis"].to_numpy(float) / epochs["dof"].to_numpy(float))),
        }
    return result


def rtk_relative(nav_path: Path, rtk_path: Path) -> float:
    nav, rtk = pd.read_csv(nav_path), pd.read_csv(rtk_path)
    valid = nav["gps_seconds"].between(rtk["gps_seconds"].iloc[0], rtk["gps_seconds"].iloc[-1])
    nav = nav.loc[valid]
    reference = np.column_stack([np.interp(nav["gps_seconds"], rtk["gps_seconds"], rtk[f"{axis}_ecef_m"])
                                 for axis in "xyz"])
    delta = nav[["ecef_x_m", "ecef_y_m", "ecef_z_m"]].to_numpy(float) - reference
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    frames, stats = {}, {}
    for mode in MODES:
        common_times = None
        for scenario in SCENARIOS:
            frame = pd.read_csv(solution_path(mode, scenario))
            frames[(mode, scenario)] = frame
            times = set(frame["gps_seconds"].to_numpy(float))
            common_times = times if common_times is None else common_times & times
        if len(common_times or ()) != len(frames[(mode, "baseline")]):
            raise ValueError(f"{mode.upper()} outputs do not have fully aligned 1 Hz epochs")
        for scenario in SCENARIOS:
            stats[(mode, scenario)] = summarize(frames[(mode, scenario)])

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True, constrained_layout=True)
    palette = {"lc": "#3867d6", "tc": "#e39b29", "ltc": "#20a37a"}
    styles = {"baseline": "--", "zupt_nhc": "-", "zpc_zhc": ":"}
    for ax, (column, (label, unit)) in zip(axes, METRICS.items()):
        plotted = []
        for mode in MODES:
            for scenario in SCENARIOS:
                data = frames[(mode, scenario)]
                values = data[column].to_numpy(float)
                ax.plot(data["gps_seconds"], values, color=palette[mode], linestyle=styles[scenario],
                        linewidth=1.15 if scenario == "zpc_zhc" else 1.0,
                        alpha=.95 if scenario == "zpc_zhc" else .65,
                        label=f"{mode.upper()} · {LABELS[scenario]}")
                plotted.append(values)
        ax.set_ylabel(f"{label} ({unit})")
        ax.set_ylim(0, float(np.nanpercentile(np.concatenate(plotted), 99.5)) * 1.08)
        ax.grid(True, alpha=.25)
    axes[-1].set_xlabel("GPS seconds")
    axes[0].legend(ncol=3, fontsize=8)
    fig.suptitle("LC / TC / LTC covariance uncertainty: baseline, ZUPT/NHC, and ZPC/ZHC")
    plot_path = OUT / "ZPC_ZHC_Performance.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    lock_fig, lock_axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True, constrained_layout=True)
    lock_palette = {"lc": "#3867d6", "tc": "#e39b29", "ltc": "#20a37a"}
    for mode in MODES:
        data = frames[(mode, "zpc_zhc")]
        q = data[["q_w", "q_x", "q_y", "q_z"]].to_numpy(float)
        heading = np.unwrap(np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                       1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))) * 180.0 / np.pi
        lock_axes[0].plot(data["gps_seconds"], heading, color=lock_palette[mode], linewidth=1.1, label=f"{mode.upper()} output heading")
        lock_axes[1].plot(data["gps_seconds"], data["heading_rms_1sigma_deg"], color=lock_palette[mode], linewidth=1.2,
                          label=f"{mode.upper()} reported heading 1σ")
        if "free_heading_rms_1sigma_deg" in data:
            lock_axes[1].plot(data["gps_seconds"], data["free_heading_rms_1sigma_deg"], color=lock_palette[mode],
                              linewidth=.9, linestyle="--", alpha=.6, label=f"{mode.upper()} free ESKF 1σ")
    lock_axes[0].set_ylabel("Heading (deg)")
    lock_axes[0].grid(True, alpha=.25)
    lock_axes[0].legend(ncol=3, fontsize=8)
    lock_axes[1].set_ylabel("Heading 1σ (deg)")
    lock_axes[1].set_xlabel("GPS seconds")
    lock_axes[1].set_ylim(0, float(np.nanpercentile(np.concatenate([
        frames[(m, "zpc_zhc")]["heading_rms_1sigma_deg"].to_numpy(float) for m in MODES] + [
        frames[(m, "zpc_zhc")]["free_heading_rms_1sigma_deg"].to_numpy(float) for m in MODES]), 99.5)) * 1.08)
    lock_axes[1].grid(True, alpha=.25)
    lock_axes[1].legend(ncol=3, fontsize=8)
    lock_fig.suptitle("ZHC heading lock: output heading and locked/free uncertainty")
    lock_plot_path = OUT / "ZHC_HeadingLock_Performance.png"
    lock_fig.savefig(lock_plot_path, dpi=160)
    plt.close(lock_fig)

    lines = [
        "# ZPC / ZHC 组合导航性能评估", "",
        "LC、TC、LTC 分别对比原始 baseline、ZUPT/NHC、ZUPT/NHC+ZPC/ZHC 三组回放。新组通过独立 JSON 配置启用，三种模式均使用相同的 158,328 个 IMU 样本；solution 输出为 1 Hz，并按 GPS 时间对齐。", "",
        "> RMS 列为滤波协方差计算的形式 1σ，不是相对于独立真值的 RMSE。数据无独立 rover 真值；相对 RTK 仅作为内部一致性诊断。", "",
        "## 解算协方差统计", "",
        "每项为 median / P95 / final；可在不同场景间直接比较同一模式。", "",
        "| 指标 | 模式 | Baseline | ZUPT/NHC | + ZPC/ZHC |", "|---|---|---:|---:|---:|",
    ]
    for column, (label, unit) in METRICS.items():
        for mode in MODES:
            cells = []
            for scenario in SCENARIOS:
                s = stats[(mode, scenario)][column]
                cells.append(" / ".join(f"{s[k]:.4f}" for k in ("median", "p95", "final")))
            lines.append(f"| {label} ({unit}) | {mode.upper()} | " + " | ".join(cells) + " |")

    lines += ["", "## 静止约束 innovation 与门限", "",
              "接受/拒绝按更新历元统计；RMS 和归一化 RMS 按 innovation 分量统计，NIS/dof 每历元计一次。", "",
              "| 模式 | 量测 | 历元 | 接受 | 拒绝 | Innovation RMS | Normalized RMS | Mean NIS/dof |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        summary = innovation_stats(innovation_path(mode, "zpc_zhc"), mode)
        for measurement, unit in (("zupt", "m/s"), ("zpc", "m"), ("zhc", "rad"), ("zhc_lock_projection", "rad"), ("nhc", "m/s")):
            s = summary.get(measurement)
            if s:
                lines.append(f"| {mode.upper()} | {measurement.upper()} | {s['epochs']} | {s['accepted']} | {s['rejected']} | {s['rms']:.5f} {unit} | {s['normalized_rms']:.4f} | {s['mean_nis_dof']:.4f} |")

    lines += ["", "## Heading-lock 恒定性与 RMS 统计", "",
              "heading-lock 只对锁定期间生效；锁定 1σ 是捕获时的形式不确定度，并非独立真值 RMSE。自由 ESKF heading 1σ 作为另一条诊断序列保留。", "",
              "| 模式 | 锁定历元 | 锁定段数 | 输出 heading 最大偏差 (deg) | 段内锁定 1σ 最大跨度 (deg) | 锁定 1σ median / P95 (deg) | 全时段报告 1σ median / P95 / final (deg) | 自由 ESKF 1σ median / P95 / final (deg) |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        data = frames[(mode, "zpc_zhc")].copy()
        q = data[["q_w", "q_x", "q_y", "q_z"]].to_numpy(float)
        data["heading_from_q_deg"] = np.degrees(np.unwrap(np.arctan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                                                        1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))))
        locked = data[data["heading_lock_active"] == 1].copy()
        if len(locked):
            error = np.angle(np.exp(1j * np.radians(locked["heading_from_q_deg"] - locked["heading_lock_heading_deg"]))) * 180.0 / np.pi
            spans = locked.groupby("heading_lock_id")["heading_rms_1sigma_deg"].agg(lambda v: float(v.max() - v.min()))
            lock_stats = locked["heading_rms_1sigma_deg"]
            max_error = float(np.max(np.abs(error)))
            max_span = float(spans.max())
            episode_count = int(locked["heading_lock_id"].nunique())
        else:
            lock_stats = pd.Series(dtype=float)
            max_error = max_span = 0.0
            episode_count = 0
        reported = data["heading_rms_1sigma_deg"]
        free = data["free_heading_rms_1sigma_deg"]
        lock_text = f"{lock_stats.median():.4f} / {lock_stats.quantile(.95):.4f}" if len(lock_stats) else "n/a"
        lines.append(f"| {mode.upper()} | {len(locked)} | {episode_count} | {max_error:.8f} | {max_span:.8f} | {lock_text} | "
                     f"{reported.median():.4f} / {reported.quantile(.95):.4f} / {reported.iloc[-1]:.4f} | "
                     f"{free.median():.4f} / {free.quantile(.95):.4f} / {free.iloc[-1]:.4f} |")

    lines += ["", "## 相对 RTK 的位置差值（非真值）", "",
              "| 模式 | Baseline 3D RMS | ZUPT/NHC 3D RMS | ZUPT/NHC+ZPC/ZHC 3D RMS |", "|---|---:|---:|---:|"]
    for mode in MODES:
        values = [rtk_relative(solution_path(mode, scenario), BASELINE / mode / "rtk_solution.csv") for scenario in SCENARIOS]
        lines.append(f"| {mode.upper()} | " + " | ".join(f"{v:.4f} m" for v in values) + " |")

    lines += ["", "## 结果解读与限制", "",
              "- ZPC/ZHC 只有在 ZUPT measurement model 已启用、动态状态模块确认 stationary 且同历元 ZUPT 被接受后才生成。ZPC 与 ZHC 独立准入；ZPC 被拒绝不会跳过 ZHC。ZPC 位置锚点使用克隆误差状态，heading lock 使用独立的 HeadingLockState，不扩展通用滤波器状态。离开 stationary/进入 unknown 时释放两类锚点。",
              "- ZPC 采用锚点固定 ECEF 位置差并投影到锚点 NED；首次通过 ZHC 准入后，heading-lock 捕获当前 heading 和形式 1σ，并在机械编排/滤波反馈后投影回锁定 heading。重复历元记录锁定预测偏差（zhc_lock_projection），不把同一静止事实当成新的独立量测。ZPC 的观测噪声初值 0.05 m；ZHC 准入噪声初值 0.5 deg，尚未用独立标注数据校准。",
              "- LC/LTC 中 ZPC 有相当一部分历元被 NIS 门限拒绝，创新残差显示检测为 stationary 时的位置与锚点存在明显差异；这提示动态状态门控或真实静止参考点假设仍需核验。TC 中 ZPC 接受率更高。不得把被拒绝历元当作已约束。",
              "- 锁定期间输出 heading 和报告 heading 1σ 按设计保持为捕获值；自由 ESKF heading 1σ 单独输出，仍可随传播增大。锁定只是将已有估计保持不变，不会自动改善绝对航向知识。",
              "- +ZPC/ZHC 回放期间，若 NHC innovation 被门限拒绝，可能与车辆侧向滑移、转弯或动态状态误判有关。应检查高残差时段与车辆真值/人工状态标签，并验证 FRD 安装及无侧滑假设。",
              "- ZUPT、ZPC、ZHC 均来自同一静止假设，虽具有不同状态分量并按顺序更新，当前实现仍未建模这些伪量测之间的统计相关性；GNSS/TDCP 与静止检测证据的互相关也未建模。因此协方差下降可能偏乐观。",
              "", "## ZHC heading-drift 排查", "",
              "历史排查发现旧实现把 ZHC 与同历元 ZPC 的 NIS 接受状态串联；GPS 63430–63750 s 内 LC/LTC 的 319 个 ZUPT 历元通过而 ZPC 均被拒绝，导致 ZHC 被跳过。现已解耦，新的全量回放中该段的 ZHC lock projection 均有记录。旧克隆锚点还会吸收滤波反馈，导致小相对创新与输出 heading 漂移并存；因此现在使用独立 HeadingLockState 并在 INS 输出路径执行 heading 投影。",
              "新回放共 1,569 个 1 Hz solution 历元/模式，3 种模式各有 846 个 heading-lock 历元、4 个 lock episode。锁定期间输出 heading 相对快照的最大误差：LC 0.00000361°、TC 0.00000424°、LTC 0.00000361°；每个 episode 的锁定 1σ 跨度均为 0°。锁定 1σ median/P95：LC 1.4287/29.9182°、TC 1.4356/29.9182°、LTC 1.4297/29.9182°。P95 由首个 episode 捕获时约 29.9182° 的高初始不确定度主导；这说明锁定不会把不确定度凭空变小。全时段报告 heading 1σ median/P95/final：LC 0.9340/29.9182/0.7402°、TC 0.9499/29.9182/0.7457°、LTC 0.9369/29.9182/0.7387°。",
              "自由 ESKF heading 1σ median/P95/final 分别为 LC 2.4017/66.6215/5.1597°、TC 2.4087/71.8679/5.1679°、LTC 2.3994/57.4955/5.1582°；它未被 heading lock 冻结，专门反映滤波器自身边缘协方差。位置 3D 1σ median/final 为 LC 0.0309/0.0296 m、TC 0.0390/0.0363 m、LTC 0.0296/0.0279 m；速度 3D 1σ median/final 为 LC 0.0601/0.0591 m/s、TC 0.0776/0.0733 m/s、LTC 0.0474/0.0443 m/s。",
              "这些 RMS 都是协方差形式的 1σ，不是独立真值 RMSE；数据没有独立姿态/位置真值。Heading-lock 的不变性按实现契约成立，但锁定 heading 仍可能存在捕获前的初始偏差。ZPC/ZHC 与 ZUPT/NHC 伪量测相关性尚未完整建模，协方差可能偏乐观；后续应重点核验首次锁定的 29.9° 初始 heading 不确定度、DynamicStatus 锁定段分类，以及退出静止后的释放行为。",
              "- 轨迹相对 RTK 差值因 LC/LTC 使用 RTK 且 TC 共用原始 GNSS 载波而非独立真值，只能用于内部一致性诊断。", "",
              f"协方差图：[ZPC_ZHC_Performance.png](ZPC_ZHC_Performance.png)。heading-lock 专项图：[ZHC_HeadingLock_Performance.png](ZHC_HeadingLock_Performance.png)。99.5 分位数用于图轴可读性，完整范围仍计入表格统计。", ""]
    report_path = OUT / "ZPC_ZHC_Performance_Report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(report_path)
    print(plot_path)
    print(lock_plot_path)


if __name__ == "__main__":
    main()
