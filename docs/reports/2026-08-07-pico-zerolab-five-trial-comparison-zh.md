# PICO 与 ZeroLab 五组人体动捕及 ELF3 Replay 综合对比报告

## 执行摘要

本报告汇集 Static、Single-joint、Dynamic、Combination 和 Body 五组既有录制及离线评估产物，比较 PICO 与 ZeroLab 的人体参考一致性、时序质量、动态平滑性和 ELF3 MuJoCo Replay 表现。本阶段建立可复现的章节、比较口径和证据索引；执行摘要的定量发现与设备选择结论在逐组证据完成分析后写入。

## 1. 评估目标与结论边界

评估回答三个问题：两套设备的 canonical 人体参考在各组相同动作条件下是否一致；各自 Sonic 目标在 ELF3 MuJoCo 闭环中的跟踪与稳定性如何；五种场景是否重复出现可用于工程选择的差异。

本报告只读取既有原始录制、canonical NPZ、alignment JSON、设备 metrics 和 Replay report；不重新录制，也不修改人体转换、标定、Sonic reference 构造或 MuJoCo 模型，不覆盖既有 canonical 或历史报告。

设备间 canonical agreement 衡量两条转换流的一致程度，而非绝对准确度：没有光学动捕等绝对真值，不将任一设备定义为 ground truth。Replay tracking 衡量机器人对各自 Sonic 目标的跟踪难度，不等价于真人动捕准确率。报告不建立无预定义权重或阈值的综合分数，也不跨组平均或按时长加权 RMSE。

## 2. 五组实验与动作内容

| 组别 | 动作内容与比较方式 | 标定帧数 | 解释边界 |
|---|---|---:|---|
| Static | 相同名义静态动作；PICO013 对 ZeroLab014，非同步跨 Trial | 100 | 仅配对共同且可识别的 T-pose、T-pose 后自然站立与 A-pose；末尾不完全对应动作不进入逐动作统计。 |
| Single-joint | 同次配对录制的单关节动作 | 100 | 使用本组公共 canonical 50 Hz 时间轴组内比较。 |
| Dynamic | 同次配对录制的动态动作 | 100 | 使用本组公共 canonical 50 Hz 时间轴组内比较。 |
| Combination | 同次配对录制的组合动作 | 100 | 使用本组公共 canonical 50 Hz 时间轴组内比较；磁盘来源保留历史拼写 `combation8`。 |
| Body | 同次配对录制的全身动作 | 60 | 临时诊断标定配置，不能评价 100 帧正式配置的最终上限。 |

Static 是相同名义动作的跨 Trial 对照，证据强度低于四组同步配对实验；不得报告跨设备接收延迟、互相关或逐时间点动态误差。Static 的全段 Replay 可并列展示，但同时列出样本数和时长，且不把接触总次数直接比较为优劣。

## 3. 指标定义与数据口径

四组同步配对实验（Single-joint、Dynamic、Combination、Body）各自在公共 canonical 50 Hz 时间轴上独立报告：原始 Timing（样本数、时长、平均接收率、P95/最大帧间隔、超过 30 ms 的 Gap）；Alignment（公共区间、估计延迟、相关系数、质量标志）；Canonical Agreement（SMPL pose、root orientation、SMPL FK position、ELF3 canonical `joint_pos`）；Dynamic Agreement（姿态角速度与位置速度 RMSE）；Smoothness（姿态速度、加速度、jerk、位置加速度 RMS）；Replay（tracking RMSE/MAE/P95/Max、速度跟踪 RMSE、限位、跌倒、最低基座高度、最大倾角）。

ELF3 canonical `joint_pos[29]` 中只有六个腕部通道出现设备差异，其余 23 个通道为零。因此 29 DoF 整体 RMSE 会被零通道稀释：报告同时保留整体值与六个 wrist DoF 单独结果，并优先以 wrist 指标解释设备差异。

Canonical stability 是人体参考低运动窗口统计，不是 MuJoCo 跌倒判定；Replay stability 才包含 `fell`、基座高度和最大倾角。`contact_count` 是任意接触的仿真子步数，受时长和 substeps 影响；`self_collision_count` 是接触事件累计值，非唯一自碰撞次数。两者跨时长只作风险提示，不作主排名指标。

## 4. 五组核心结果总览

| 组别 | 配对属性 | 结果归并规则 | 证据入口 |
|---|---|---|---|
| Static | 非同步跨 Trial | 按共同稳定阶段和全段 Replay 分列，不作逐帧动态归并 | Static canonical、replay、cross-trial analysis |
| Single-joint | 同次配对 | 组内 Timing、Alignment、Agreement、Dynamic、Smoothness、Replay 分列 | Single metrics 与双方 replay |
| Dynamic | 同次配对 | 组内 Timing、Alignment、Agreement、Dynamic、Smoothness、Replay 分列 | Dynamic metrics 与双方 replay |
| Combination | 同次配对 | 组内 Timing、Alignment、Agreement、Dynamic、Smoothness、Replay 分列 | Combination metrics 与双方 replay |
| Body | 同次配对，60 帧临时标定 | 组内指标分列，并保留临时配置限定 | Body metrics 与双方 replay |

Static 的三个共同稳定阶段中，T-pose 的 pose RMSE 为 13.731°，T 后自然站立为 30.582°，A-pose 为 23.754°；这是非同步跨 Trial 的名义姿态复现证据，不含时间延迟或逐帧差异。Single-joint 在 114.16 s 公共区间内估计延迟为 0 ms、相关系数 0.4795，pose/root/position RMSE 分别为 22.569°、6.694°、0.0831 m，动态姿态差异为 313.416°/s；双方 Replay 均为 5700 样本/113.98 s，PICO 有 23 个限位样本、ZeroLab 为 0。

本节只在五组分别完成核对后，以场景限定的数值和方向性发现归纳重复差异；时序、静态姿态、动态平滑性与 Replay 安全性相互独立判断。

## 5. Static：PICO013 与 ZeroLab014 跨 Trial 对照

PICO013 与 ZeroLab014 不是同步录制。故本节只把可识别的相同名义稳定动作配对：T-pose（T）、T-pose 后自然站立（N_after_T）和 A-pose（A）；PICO 的全段还包含同步开场和正式 T/N/A/N，ZeroLab 则是从 formal onset 开始的 formal T/N/A/direct-T/N。PICO 不含最后的 T/N，ZeroLab 也不含计划中的 A 后自然站立，因此这些末尾片段不进入统计。本组证据强度低于第 6--9 节同步配对实验，**不报告或解释跨设备接收延迟、互相关、时间对齐，或逐帧动态误差**。

### 5.1 名义姿态 agreement

下表每格为 `RMSE / P95 / 最大值`。这是各共同名义姿态在各自稳定阶段内的跨 Trial 汇总，不是同一时刻的动作误差。

| 阶段（每侧样本/时长） | SMPL pose（°） | root orientation（°） | SMPL FK local position（m） | ELF3 `joint_pos[29]`（rad） |
|---|---:|---:|---:|---:|
| T（375 / 7.48 s） | 13.731 / 22.753 / 32.461 | 4.375 / 5.219 / 5.346 | 0.0701 / 0.1234 / 0.1441 | 0.1039 / 0.3018 / 0.4119 |
| N_after_T（140 / 2.78 s） | 30.582 / 80.018 / 89.340 | 5.423 / 5.759 / 5.793 | 0.0915 / 0.1648 / 0.1996 | 0.3912 / 1.4299 / 1.5252 |
| A（375 / 7.48 s） | 23.754 / 50.010 / 66.552 | 6.181 / 8.004 / 8.163 | 0.1103 / 0.2268 / 0.3088 | 0.2253 / 0.7766 / 0.9819 |

T 的名义姿态最接近；N_after_T 的 pose 与 ELF3 差异最大，A 居中。ELF3 29 维整体值需要谨慎解读：只有 6 个 wrist 通道在设备间出现差异，另 23 个通道恒为零，整体 RMSE 会被零通道稀释。因此它用于确认腕部目标是否一致，而不能脱离 wrist 通道单独解释为 29 DoF 全身精度。

### 5.2 名义稳定段平滑性

| 阶段 | PICO pose 加速度 RMS（°/s²） | ZeroLab pose 加速度 RMS（°/s²） | PICO pose jerk RMS（°/s³） | ZeroLab pose jerk RMS（°/s³） | PICO / ZeroLab 位置加速度 RMS（m/s²） |
|---|---:|---:|---:|---:|---:|
| T | 68.378 | 469.993 | 5,650.118 | 43,631.120 | 0.315 / 1.263 |
| N_after_T | 124.906 | 1,315.325 | 10,395.922 | 119,992.664 | 0.525 / 3.127 |
| A | 167.895 | 574.866 | 13,676.349 | 54,125.992 | 1.137 / 1.872 |

在三个共同阶段中，ZeroLab 的姿态加速度均为 PICO 的约 3.4–10.5 倍，jerk 为约 4.0–11.5 倍；位置加速度也更高。这描述的是各自录制流在相同名义姿态中的平滑性差异，不能归因于同步时序或设备间延迟。

### 5.3 Sonic--MuJoCo Replay

两侧全段 Replay 的参考范围不同：PICO 是提供的完整 2222 样本/44.42 s 流，ZeroLab 是 formal 1653 样本/33.04 s 流。它们可并列显示各自目标的机器人跟踪难度，但不能将总 contact 或 self-collision 计数直接比较为优劣（时长与仿真子步数会影响累计值）。

| 全段 Replay | 样本 / 时长 | tracking RMSE（rad） | MAE / P95 / 最大误差（rad） | 速度 tracking RMSE（rad/s） | 限位样本 | 跌倒 | 最低基座高度（m） | 最大倾角（°） | contact / self-collision |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| PICO013 full | 2222 / 44.42 s | 0.0560 | 0.0308 / 0.1211 / 0.6586 | 0.2225 | 0 | 否 | 1.0051 | 6.617 | 22,220 / 20,989 |
| ZeroLab014 formal | 1653 / 33.04 s | 0.0530 | 0.0296 / 0.1267 / 0.6567 | 0.1749 | 0 | 否 | 1.0051 | 5.552 | 16,530 / 15,299 |

对可匹配的稳定阶段，Replay 指标如下；两侧每一阶段均无越限和跌倒。

| 阶段（每侧样本/时长） | PICO RMSE / P95（rad） | ZeroLab RMSE / P95（rad） | PICO / ZeroLab 速度 RMSE（rad/s） | PICO / ZeroLab 最大倾角（°） |
|---|---:|---:|---:|---:|
| T（375 / 7.48 s） | 0.0527 / 0.1256 | 0.0414 / 0.1262 | 0.0921 / 0.0782 | 3.219 / 2.025 |
| N_after_T（140 / 2.78 s） | 0.0462 / 0.0838 | 0.0557 / 0.1556 | 0.1101 / 0.1733 | 6.278 / 5.456 |
| A（375 / 7.48 s） | 0.0541 / 0.0999 | 0.0512 / 0.1221 | 0.1196 / 0.1428 | 4.543 / 3.280 |

Static 组只能说明两条转换流对 T、自然站立和 A 三种名义姿态的复现一致性及各自 Replay 的可执行性；它不提供绝对人体动捕准确率，也不把较低 Replay tracking 误差解释为更准确的人体参考。

## 6. Single-joint：同次配对录制

本组是同次配对录制，使用公共 canonical 50 Hz 时间轴；因而可报告 Timing、Alignment 和公共区间的逐时间序列 agreement。其结论仅适用于本单关节动作，不外推为其他动作的总体设备排名。

### 6.1 原始 Timing 与 Alignment

| 流 | 原始样本 | 原始时长（s） | 平均接收率（Hz） | P95 帧间隔（ms） | 最大帧间隔（ms） | 超过 30 ms |
|---|---:|---:|---:|---:|---:|---:|
| PICO | 5,795 | 119.246 | 48.589 | 34.535 | 312.584 | 694（11.98%） |
| ZeroLab | 5,948 | 120.916 | 49.183 | 30.430 | 298.622 | 416（7.00%） |

公共区间为 **114.16 s**（5709 个 50 Hz 栅格样本）；延迟搜索范围为 ±1.5 s，估计延迟为 **0 ms**（按定义是将 ZeroLab 时间戳提前该估计值），相关系数为 **0.4795**，且没有 quality flag。栅格化后两侧均为 50 Hz、P95 20 ms、无超过 30 ms 的间隔；这仅说明共同时间轴的重采样规则，不会抹去上表原始接收节奏的差异。

### 6.2 Canonical 与动态 agreement

| 指标（公共 114.16 s） | RMSE | P95 | 最大值 |
|---|---:|---:|---:|
| SMPL pose（°） | **22.569** | 43.365 | 176.340 |
| root orientation（°） | **6.694** | 17.930 | 27.362 |
| SMPL FK local position（m） | **0.0831** | 0.1521 | 0.6009 |
| ELF3 `joint_pos[29]`（rad） | 0.1814 | 0.5326 | 0.9837 |
| pose angular velocity（°/s） | **313.416** | — | — |
| position velocity（m/s） | 0.3158 | — | — |

右肩是 pose 逐关节 RMSE 最大的关节（63.426°），是本组 22.569°整体 pose RMSE 的主要局部误差来源。ELF3 的 29 维整体结果同样由 6 个 wrist 通道承载全部非零差异、其余 23 通道为零；六个 wrist 通道的 RMSE 依次为 0.6804、0.2880、0.2023、0.1498、0.5192、0.2746 rad，故不以被稀释的 29 维整体值替代腕部解释。上述 agreement 是两条转换流之间的一致性，仍非对真人的绝对精度。

### 6.3 平滑性与低运动窗口

| 流 | pose 速度 RMS（°/s） | pose 加速度 RMS（°/s²） | pose jerk RMS（°/s³） | 位置加速度 RMS（m/s²） |
|---|---:|---:|---:|---:|
| PICO | 22.299 | 642.300 | 53,270.909 | 3.173 |
| ZeroLab | 68.609 | 3,847.818 | 319,723.356 | 10.552 |

ZeroLab 的姿态速度、加速度和 jerk 分别约为 PICO 的 3.1、6.0 和 6.0 倍，位置加速度约为 3.3 倍。低运动窗口统计中，PICO 有 6 个窗口/607 样本（pose 标准差 20.955°、位置标准差 0.3293 m），ZeroLab 有 2 个窗口/135 样本（17.207°、0.3643 m）；这是参考流稳定窗口的描述，不是 Replay 是否跌倒的判据。

### 6.4 Sonic--MuJoCo Replay

| Replay | 样本 / 时长 | tracking RMSE（rad） | MAE / P95 / 最大误差（rad） | 速度 tracking RMSE（rad/s） | 限位样本 | 跌倒 | 最低基座高度（m） | 最大倾角（°） |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| PICO | **5700 / 113.98 s** | 0.0693 | 0.0331 / 0.1358 / 1.3889 | 0.6973 | **23 个限位样本** | 否 | 0.9919 | 16.095 |
| ZeroLab | **5700 / 113.98 s** | 0.0644 | 0.0308 / 0.1142 / 1.8996 | 0.5746 | **0 个限位样本** | 否 | 0.9980 | 17.618 |

两侧均以相同样本数、相同时长 Replay，因而本表可比较本单关节目标的机器人跟踪表现。ZeroLab 的 RMSE、MAE、P95 和速度 tracking RMSE 较低；PICO 的最大误差更低（1.3889 vs 1.8996 rad），但限位样本更多（23 vs 0），这些维度不合并为单一排名。两侧均未跌倒。Replay tracking 只反映机器人对各自 Sonic 目标的跟踪，不能据此把其中任一方称为更准确的人体动捕。

## 7. Dynamic：同次配对录制

本节按第 3 节口径呈现动态组证据，分别讨论快速动作的时序、姿态与位置速度、平滑性，以及 Sonic–MuJoCo 可执行性和安全性；仅单组出现的事件保持场景限定。

## 8. Combination：同次配对录制

本节按第 3 节口径呈现组合动作证据。正文采用规范名称“Combination”，路径和数据源保留历史目录拼写 `combation8` 以保证可追溯性。

## 9. Body：同次配对录制（60帧临时标定）

本节按第 3 节口径呈现全身动作证据。所有比较和结论均注明其使用 60 帧临时标定配置，不能据此评价 100 帧正式配置的最终能力上限。

## 10. PICO 的优点与缺点

本节仅从五组中重复出现且可由固定证据追溯的现象归纳 PICO 的优点、局限与适用条件；不以单一指标替代其他维度，也不作绝对准确度声明。

## 11. ZeroLab 的优点与缺点

本节仅从五组中重复出现且可由固定证据追溯的现象归纳 ZeroLab 的优点、局限与适用条件；较低 Replay tracking RMSE 只表示该段目标更易被机器人跟踪，不表示更接近真人动作。

## 12. 综合性能分析与适用场景

综合性能依据五种场景是否重复出现同方向差异形成：数据接收与时间稳定性、静态姿态与末端位置一致性、单关节映射可解释性、快速/组合动作平滑性、Sonic–MuJoCo 可执行性与安全性、部署便利性与传感器覆盖分别判断。不同动作难度的 RMSE 不直接平均；单组异常保留场景限定。

## 13. 局限与后续改进优先级

本报告没有绝对真值，故不输出设备绝对精度结论。Static 的非同步跨 Trial 性质降低证据强度；Body 的 60 帧临时标定限制其可比范围；接触计数受时长和仿真子步数影响。后续改进以补足可比较的同步证据、明确低置信度数据、保持单位一致和逐项追溯关键数值为优先。

## 14. 数据源与复现路径

下表列出本报告固定读取的证据源；报告不对多个重复 Replay 版本求平均。

| 组别 | 固定路径 | 用途 |
|---|---|---|
| Static | `/tmp/pair-static-013-eval-final-1785926456/pico_canonical.npz` | PICO013 canonical。 |
| Static | `/tmp/pico-sonic-mujoco-live-full-002/report.json` | PICO013 Sonic–MuJoCo Replay。 |
| Static | `/tmp/pair-static-014-zerolab-formal/zerolab_formal_canonical.npz` | ZeroLab014 canonical。 |
| Static | `/tmp/pair-static-014-zerolab-formal-mujoco-001/report.json` | ZeroLab014 Sonic–MuJoCo Replay。 |
| Static | `/tmp/pair-static-014-zerolab-formal/comparison_analysis.json` | 跨 Trial 共同稳定阶段与比较分析。 |
| Single-joint | `/tmp/pair-single-joint-metrics-verify-sl9CEz/report.json` | 配对 metrics、Timing、Alignment、Agreement 与 Smoothness。 |
| Single-joint | `/tmp/pico-single-joint-mujoco-Ahx7zb/report.json` | PICO Replay。 |
| Single-joint | `/tmp/zerolab-single-joint-mujoco-full-80q2PY/report.json` | ZeroLab Replay。 |
| Dynamic | `/tmp/pair-dynamic-003-metrics-HFfZ7D/report.json` | 配对 metrics、Timing、Alignment、Agreement 与 Smoothness。 |
| Dynamic | `/tmp/pair-dynamic-003-pico-mujoco-siZHzq/report.json` | PICO Replay。 |
| Dynamic | `/tmp/pair-dynamic-003-zerolab-mujoco-EOOe4f/report.json` | ZeroLab Replay。 |
| Combination | `/tmp/pair-combation8-metrics-RHbW1v/report.json` | 配对 metrics、Timing、Alignment、Agreement 与 Smoothness。 |
| Combination | `/tmp/pair-combation8-pico-full-agent/report.json` | PICO Replay。 |
| Combination | `/tmp/pair-combation8-zero-full-agent/report.json` | ZeroLab Replay。 |
| Body | `/tmp/pair-body-002-cal60-metrics-agent/report.json` | 60 帧临时标定下的配对 metrics、Timing、Alignment、Agreement 与 Smoothness。 |
| Body | `/tmp/pair-body-002-cal60-pico-full-agent/report.json` | 60 帧临时标定下的 PICO Replay。 |
| Body | `/tmp/pair-body-002-cal60-zero-full-agent/report.json` | 60 帧临时标定下的 ZeroLab Replay。 |

复现时逐个读取上述固定产物，核对每个 report 的 source、samples、duration、controller rate 与 reference source；完整 Replay 的样本数应等于 canonical 帧数减去 9 帧滑窗预热。同步配对组还应核对双方使用相同 canonical 长度和仿真参数；Static 仅按共同阶段核对样本数，避免把不对应动作配为同一阶段。
