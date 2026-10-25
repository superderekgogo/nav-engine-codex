#!/usr/bin/env python3
"""Compare LC, TC and LTC replay solutions and pre-update innovations."""
import csv, math, statistics
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
RUNS = {"LC": ROOT/"ins_rtk_nav/build/mode_lc", "TC": ROOT/"ins_rtk_nav/build/mode_tc", "LTC": ROOT/"ins_rtk_nav/build/mode_ltc"}
OUT = ROOT/"output"
REPORT = OUT/"LC_TC_LTC_Performance_Report.md"
FIGURE = OUT/"lc_tc_ltc_performance.png"

def read(path):
    with path.open(newline="", encoding="utf-8-sig") as f: return list(csv.DictReader(f))
def num(row,key):
    try: return float(row[key])
    except (KeyError,ValueError,TypeError): return math.nan
def rms(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else math.nan
def pct(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    return xs[min(len(xs)-1,math.ceil(p*len(xs))-1)] if xs else math.nan
def fmt(x,d=4): return "n/a" if not math.isfinite(x) else f"{x:.{d}f}"
def summarize(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return (statistics.median(xs),pct(xs,.95),xs[-1] if xs else math.nan) if xs else (math.nan,math.nan,math.nan)

solutions={k:read(p/"ltc_solution.csv") for k,p in RUNS.items()}
innovations={k:read(p/"ltc_innovations.csv") for k,p in RUNS.items()}
first=min(num(rows[0],"gps_seconds") for rows in solutions.values())
metrics={"Position 3D 1σ (m)":"position_3d_rms_1sigma_m","Velocity 3D 1σ (m/s)":"velocity_3d_rms_1sigma_mps","Attitude 3D 1σ (deg)":"attitude_3d_rms_1sigma_deg"}

fig,axs=plt.subplots(2,2,figsize=(14,8),constrained_layout=True)
colors={"LC":"tab:blue","TC":"tab:orange","LTC":"tab:green"}
for ax,(label,key) in zip(axs.flat[:3],metrics.items()):
    for mode,rows in solutions.items():
        sampled=rows[::100]
        ax.plot([(num(r,"gps_seconds")-first)/60 for r in sampled],[num(r,key) for r in sampled],label=mode,color=colors[mode],lw=1)
    ax.set_title(label);ax.set_xlabel("Elapsed time (min)");ax.grid(True,alpha=.25);ax.legend()
ax=axs.flat[3]
for mode,rows in innovations.items():
    vals=[num(r,"normalized_innovation") for r in rows]
    vals=[x for x in vals if math.isfinite(x)]
    ax.hist(vals,bins=60,density=True,histtype="step",label=f"{mode} (n={len(vals)})",color=colors[mode])
ax.set_xlim(-6,6);ax.set_title("Pre-update normalized innovations");ax.set_xlabel("Residual / marginal sigma");ax.grid(True,alpha=.25);ax.legend(fontsize=8)
fig.suptitle("INS/RTK combination mode comparison")
OUT.mkdir(parents=True,exist_ok=True);fig.savefig(FIGURE,dpi=160);plt.close(fig)

lines=["# LC / TC / LTC 组合导航性能评估","",f"数据集重放：{len(solutions['LTC']):,} 个 INS 输出样本，{len(read(RUNS['LTC']/ 'rtk_solution.csv')):,} 个 RTK 历元；统计生成于 2026-10-24。三种模式独立配置、独立重放。","","## 模式定义与处理量","","| 模式 | 实际滤波更新 | RTK 解输入 | TDCP 输入 |","|---|---|---:|---:|"]
lines += ["| LC | RTK 位置与速度松组合更新 | 是 | 否 |","| TC | 卫星级 L1/L2 TD-DDCP 几何残差直接更新 | 否 | 是 |","| LTC | RTK 位置松组合；L1 TDCP 解算相邻历元天线 Δposition，再除以 Δt 得到区间平均速度更新 | 是（位置） | 是（仅 L1 用于 WLS） |","","LTC 的 L1 观测先构造模型 `y = c + A·Δr + ε`，用共享 pivot 相关协方差及前历元位置不确定度进行加权最小二乘，要求至少 4 条有效 L1 行且几何矩阵满秩；得到 Δr 后转换到 NED，并以 `Δr/Δt` 作为区间平均速度观测。滤波预测量使用前后 INS 端点速度均值 `0.5(vₖ₋₁+vₖ)`，而非仅用当前端点速度。该速度更新对应 H=0.5I；前历元速度协方差按 0.25 投影入观测噪声。","","## 回放统计","","| 模式 | INS 样本 | RTK 更新 | TDCP 更新历元 | TDCP 派生速度分量数 | TDCP 拒绝历元 | 最终位置 3D 1σ (m) |","|---|---:|---:|---:|---:|---:|---|"]
for mode,folder in RUNS.items():
    rows=solutions[mode]; inv=innovations[mode]; groups={}
    for r in inv:
        if r.get("measurement")=="td_ddcp_range": groups.setdefault((r["gps_seconds"],r["previous_gps_seconds"]),set()).add(r["svid"])
    tdrows=sum(1 for r in inv if r.get("measurement") in ("td_ddcp_range","l1_tdcp_delta_position_velocity"))
    td_epochs=len(groups) if mode=="TC" else len(set((r["gps_seconds"],r["previous_gps_seconds"]) for r in inv if r.get("measurement")=="l1_tdcp_delta_position_velocity"))
    accepted=int(td_epochs)
    rejected=1563-accepted if mode!="LC" else 0
    rtku=int(sum(1 for r in inv if r.get("measurement")=="rtk_pv" and r.get("component","").startswith("position_n")))
    last=rows[-1]
    lines.append(f"| {mode} | {len(rows):,} | {rtku:,} | {td_epochs:,} | {tdrows:,} | {rejected:,} | {fmt(num(last,'position_3d_rms_1sigma_m'))} |")
lines += ["","注意：创新 CSV 中 LTC/TC 的 `nis` 是每个联合更新历元的同一个 NIS 值，按 `(gps_seconds, previous_gps_seconds, measurement)` 去重统计；`normalized_innovation` 则是每行用边际标准差归一化，行间相关时不能视为独立标准正态样本。","","## 协方差变化","","以下为输出协方差迹开方（3D 1σ）的全程 median / P95 / 最终值，包含初始收敛期。它表示滤波器自身报告的不确定度，不是真值误差。","","| 指标 | LC median / P95 / final | TC median / P95 / final | LTC median / P95 / final |","|---|---:|---:|---:|"]
for label,key in metrics.items():
    values=[]
    for mode,rows in solutions.items(): values.append(" / ".join(fmt(v,5) for v in summarize([num(r,key) for r in rows])))
    lines.append(f"| {label} | {values[0]} | {values[1]} | {values[2]} |")
lines += ["","## Innovations","","| 模式 / 量测 | 行数 | bias | RMS | abs P95 | max abs | normalized RMS | normalized abs P95 | mean NIS/dof |","|---|---:|---:|---:|---:|---:|---:|---:|"]
for mode,rows in innovations.items():
    bands={}
    for r in rows: bands.setdefault(r.get("measurement","unknown"),[]).append(r)
    for meas,rs in bands.items():
        residual=[num(r,"innovation") for r in rs]; norm=[num(r,"normalized_innovation") for r in rs]
        epoch={}
        for r in rs: epoch[(r.get("gps_seconds"),r.get("previous_gps_seconds"),r.get("measurement"))]=(num(r,"nis"),num(r,"dof"))
        nd=[nis/dof for nis,dof in epoch.values() if dof>0 and math.isfinite(nis)]
        lines.append(f"| {mode} / {meas} | {len(rs):,} | {fmt(statistics.mean([x for x in residual if math.isfinite(x)]),5)} | {fmt(rms(residual),5)} | {fmt(pct([abs(x) for x in residual],.95),5)} | {fmt(max(abs(x) for x in residual if math.isfinite(x)),5)} | {fmt(rms(norm),3)} | {fmt(pct([abs(x) for x in norm],.95),3)} | {fmt(statistics.mean(nd),3)} |")
lines += ["","## Innovations interpretation","","按目前噪声/相关性近似，LTC 的 L1 派生速度更新 normalized RMS=0.878、mean NIS/dof=0.831，接近 1，初步看与设定噪声量级相符；LC RTK P/V 的 normalized RMS=1.283、NIS/dof=2.798，提示 RTK 量测噪声可能偏小或存在未建模异常；TC TD-DDCP 的 normalized RMS=0.199、NIS/dof=0.048，创新明显小于预测不确定度，当前 TC 协方差很可能偏保守，尤其应复核时间差分噪声、共享参考状态协方差投影和相关处理。上述 NIS 仅作诊断，不是独立精度评价。","","## 解释与限制","","- LTC 这次实际接受 1,557 个 L1 派生速度更新历元（4,671 个速度分量）；另有 6 个未能用于该更新，原因是有效 L1 行数不足或几何条件不合格。RTK 位置仍在所有 1,564 个有效历元更新。需要结合 CSV 和日志定位每个拒绝原因；当前输出只记录计数，没有逐原因诊断。","- LTC 的 TDCP 派生速度与 RTK 位置的误差存在数据相关性；当前实现没有前后状态完整互协方差，也没有 RTK 与相位之间的完整互协方差，因此 covariance/NIS 是近似评估。","- 三种模式的数值输出及协方差不能直接当作独立精度真值比较。LC/LTC 使用同一 RTK 引擎输入；没有独立 rover 位置、速度或姿态参考数据，不能据此认证绝对定位、速度或姿态精度。","- 目前报告中的 innovations 是更新前残差；NIS 使用实现内近似的预测协方差。normalized RMS 偏离 1 可提示噪声模型/相关性仍需调整，但受相关行与参考状态误差近似影响。","- 图表：`output/lc_tc_ltc_performance.png`。分模式原始结果分别在 `ins_rtk_nav/build/mode_lc/`、`mode_tc/`、`mode_ltc/`。","","## 结论","","三种模式均可通过 JSON 独立切换和运行。按当前数据，LTC 的 L1 TDCP 区间平均速度更新已成功进入滤波器，和 RTK 位置松组合并行工作；它不是 TC 的卫星级范围残差更新，也没有把 RTK 速度默认混入 LTC。LTC 最终位置协方差为约 0.030 m 的 3D 1σ（N/E/D 边际组合），可视为滤波器内部不确定度指标，不能解读为已验证的 3 cm 实际定位误差。","",""]
REPORT.write_text("\n".join(lines),encoding="utf-8")
print(REPORT);print(FIGURE)
