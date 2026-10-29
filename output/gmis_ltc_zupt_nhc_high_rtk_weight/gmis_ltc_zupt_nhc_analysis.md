# GMIS-LTC RTK position weight comparison

Common epochs: 1,564; GPS seconds 63390.000–64953.000.
The first run uses RTK position σ=[0.5, 0.5, 1.0] m; the second uses σ=[0.1, 0.1, 0.1] m. Both runs retain ZUPT/NHC/ZPC/ZHC. Values are covariance-derived formal 1σ statistics, not independent truth errors.

| Metric | Baseline RMS / median / P95 / max | Constraints RMS / median / P95 / max | Unit |
|---|---:|---:|---|
| Position 3D σ | 0.149844 / 0.132978 / 0.164318 / 1.22411 | 0.0479755 / 0.0453906 / 0.0553208 / 0.173204 | m |
| Velocity 3D σ | 0.0840801 / 0.0124727 / 0.0173695 / 3.27915 | 0.0838849 / 0.0123762 / 0.0167995 / 3.27244 | m/s |
| Attitude 3D σ | 2.63429 / 0.427202 / 5.64547 / 13.698 | 2.63402 / 0.427107 / 5.64511 / 13.698 | deg |
| GNSS update NIS | 146.603 / 2.5655 / 86.5289 / 3550.63 | 147.17 / 3.03311 / 88.1442 / 3561.33 | χ²; variable DOF |

| Pairwise solution difference | Median / P95 / max | Unit |
|---|---:|---|
| Baseline vs constrained position | 0.0815501 / 0.16587 / 0.251586 | m |

Constraint trigger/accept counts are available in the sibling `constraint_innovations.csv`; per-epoch NIS traces are selectable in the interactive dashboard.

| Constraint | Triggered | Accepted |
|---|---:|---:|
| NHC | 477 | 390 |
| ZHC | 840 | 838 |
| ZPC | 840 | 838 |
| ZUPT | 840 | 838 |

## Per-axis position formal uncertainty

Local NED position covariance diagonal, 1σ in metres:

| Axis | Standard-weight RMS / median / P95 | Higher-weight RMS / median / P95 | RMS reduction |
|---|---:|---:|---:|
| North | 0.06290 / 0.05682 / 0.06478 m | 0.02530 / 0.02423 / 0.02805 m | 59.8% |
| East | 0.05679 / 0.04922 / 0.06144 m | 0.02239 / 0.02090 / 0.02599 m | 60.6% |
| Down | 0.12358 / 0.10968 / 0.13776 m | 0.03407 / 0.03249 / 0.04021 m | 72.4% |

The formal Down 1σ RMS falls from 12.36 cm to 3.41 cm, so the filter covariance reaches a few-centimetre level under this weighting. This is a reported uncertainty, not an independently verified positioning error. GNSS NIS median/P95 changes from 2.57/86.53 to 3.03/88.14 (variable observation degrees of freedom). Constraint trigger/accept counts remain unchanged: ZUPT/ZPC/ZHC 840/838 each; NHC 477/390. Position solutions differ by median 8.16 cm, P95 16.59 cm, max 25.16 cm; this is inter-run separation, not truth error.
