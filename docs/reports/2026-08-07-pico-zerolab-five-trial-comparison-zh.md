# PICO 与 ZeroLab 五组人体动捕及 ELF3 Replay 综合对比报告

## 执行摘要

本报告汇集 Static、Single-joint、Dynamic、Combination 和 Body 五组既有录制及离线评估产物，比较 PICO 与 ZeroLab 的人体参考一致性、时序质量、动态平滑性和 ELF3 MuJoCo Replay 表现。结论按场景和维度给出，不跨动作平均，也不设置设备总分。

- **接收质量按场景分化。**Single-joint 中 ZeroLab 的平均接收率和原始 Gap 占比优于 PICO（49.183 Hz、7.00% 对 48.589 Hz、11.98%），但在 Combination/Body 中降至 37.337/35.700 Hz，Gap 占比升至 29.491%/37.55%。
- **Static 只支持名义姿态复现判断。**PICO013 与 ZeroLab014 是非同步跨 Trial；T、N_after_T、A 的 pose RMSE 分别为 13.731°、30.582°、23.754°，不能解释为逐帧误差或绝对精度。
- **肩腕映射是共同校核项。**Single-joint 的右肩 RMSE 达 63.426°；六个 wrist 通道承载全部 ELF3 设备间差异，29 DoF 整体值受 23 个零通道稀释，但现有证据尚不能确认差异来自哪一侧或哪一环节。
- **PICO 在已报告的多组 canonical 平滑性上更有优势。**Static 三阶段与 Single-joint 的 PICO 姿态加速度、jerk 均较低，Dynamic 也呈相同方向；该优势不等价于人体姿态更准确。
- **Replay 易跟踪性没有单一赢家。**ZeroLab 在 Single-joint 和 Combination 的 tracking RMSE 较低；PICO 在 60 帧临时诊断 Body 中较低（0.11847 对 0.13629 rad）。
- **Replay 安全事件也需分场景看。**ZeroLab 五组均未触发跌倒；PICO Dynamic 在 sample 6157 因 61.282° 倾角越阈值触发 `fell=true`，但最低基座高度仍为 0.813 m，不能表述为机器人倒地。
- **限位结果需逐场景验收。**Single-joint 中 PICO/ZeroLab 为 23/0 个限位样本，Body 中 ZeroLab 有 8 个，Static 双方均为 0；不能据此形成跨动作安全排名。
- **复杂动作暴露高动态差异。**Dynamic、Combination、Body 的 pose angular velocity RMSE 分别为 372.359、637.417、380.618°/s，且 Combination 存在接近 180° 的单点 pose 异常。
- **工程选择应先匹配任务。**平滑输入优先的任务可评估 PICO；连续输入任务需重点验证 ZeroLab 在 Combination/Body 暴露的风险，但现有 timing 证据不足以据此推荐 PICO；Single-joint/Combination 的机器人易跟踪性可评估 ZeroLab。正式选型仍需动作标签、完整哈希和外部 ground truth 补证。

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

Dynamic、Combination 与 Body 也均只在各自同步配对的公共 canonical 时间轴内比较：Dynamic 的 pose/root/position RMSE 为 26.068°、10.204°、0.0942 m，动态姿态差异为 372.359°/s；Combination 的 pose RMSE 为 21.956°、动态姿态差异为 637.417°/s；采用 **60 帧**临时诊断标定的 Body 公共区间为 103.02 s，pose/root/position RMSE 为 24.775°、21.069°、0.1395 m，动态姿态差异为 380.618°/s。它们均为设备间转换流 agreement，而非绝对人体动捕精度。

本节只在五组分别完成核对后，以场景限定的数值和方向性发现归纳重复差异；时序、静态姿态、动态平滑性与 Replay 安全性相互独立判断。

## 5. Static：PICO013 与 ZeroLab014 跨 Trial 对照

PICO013 与 ZeroLab014 不是同步录制。故本节只把可识别的相同名义稳定动作配对：T-pose（T）、T-pose 后自然站立（N_after_T）和 A-pose（A）；PICO 的全段还包含同步开场和正式 T/N/A/N，ZeroLab 则是从 formal onset 开始的 formal T/N/A/direct-T/N。ZeroLab 末尾的 direct-T/N 因 PICO 无对应动作而排除；ZeroLab 也不含计划中的 A 后自然站立，因此这些末尾片段不进入统计。本组证据强度低于第 6--9 节同步配对实验，**不报告或解释跨设备接收延迟、互相关、时间对齐，或逐帧动态误差**。

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

Dynamic 是同次同步配对录制，以下 Timing、Agreement、Smoothness 与 Replay 只在本动态动作组内部比较；不把它们与其他组归并为设备总体排名。两侧完整 Replay 均为 6,383 样本，使用 50 Hz 控制率、10 个 substeps，且 `reference.source=live`。

### 7.1 公共时间轴、canonical agreement 与动态差异

公共 canonical 时间轴重采样为 50 Hz，是双方逐时刻比较的共同栅格，不代表原始接收节奏没有 Gap，也不能用重采样后的固定 20 ms 间隔掩盖原始 Gap。该组的主要一致性结果如下；它们只衡量两条 canonical 转换流的 agreement，不是任一设备相对于真人的绝对精度。

| 指标（Dynamic 组内） | RMSE |
|---|---:|
| SMPL pose（°） | **26.068** |
| root orientation（°） | **10.204** |
| SMPL FK local position（m） | **0.0942** |
| pose angular velocity（°/s） | **372.359** |

动态姿态差异（pose angular velocity RMSE）达到 372.359°/s，说明两条人体参考在快速变化部分的差别不能由静态 pose RMSE 单独概括。ELF3 `joint_pos[29]` 的解释仍遵循第 3 节：仅六个 wrist 通道承载设备差异，29 DoF 整体值不能脱离该结构理解。

### 7.2 平滑性

在本 Dynamic 组内，ZeroLab 的姿态加速度与 jerk 均高于 PICO，表明其参考流在该快速动作中的高频变化更强。这是各自输入流的平滑性描述，不把它解释为跨组的总体结论或绝对动捕准确率。

### 7.3 Sonic--MuJoCo Replay 与阈值事件

双方 Replay 样本数、控制率、substeps 和 live reference 设置相同，故可并列检查本组各自 Sonic 目标的可执行性。PICO 的 `fell=true` 由 **sample 6157** 的最大倾角 **61.282°** 越过跌倒阈值触发；同一 Replay 的最低基座高度仍为 **0.813 m**。因此这是该判据下的阈值事件，**不能写成机器人倒地**。ZeroLab 未触发跌倒判定。Replay tracking 的表现只反映机器人跟踪各自 Sonic 目标的难度，不等价于真人动捕精度。

## 8. Combination：同次配对录制

Combination 是同次同步配对录制；正文采用规范名称“Combination”，路径和数据源保留历史目录拼写 `combation8` 以保证可追溯性。以下比较仅针对该组合动作组；两侧完整 Replay 均为 3,992 样本，使用 50 Hz 控制率、10 个 substeps，且 `reference.source=live`。

### 8.1 原始 Timing 与公共时间轴

ZeroLab 的原始平均接收率为 **37.337 Hz**，超过 30 ms 的原始 Gap 占 **29.491%**。公共 canonical 时间轴虽重采样为 50 Hz，以支持同步逐帧 agreement，但这不改变、更不能掩盖上述原始接收率和 Gap。时序现象只与本 Combination 配对内的 PICO 流比较，不外推为其他场景的结论。

### 8.2 Canonical 与动态 agreement

| 指标（Combination 组内） | RMSE / 事件 |
|---|---:|
| SMPL pose（°） | **21.956** |
| pose angular velocity（°/s） | **637.417** |
| 最大单点 pose 误差 | **接近 180°** |

637.417°/s 的动态姿态差异表明组合动作的瞬态差异显著；最大 pose 误差中存在接近 180° 的单点异常，故不能只以 RMSE 平均值淡化或删除该异常。以上 agreement 描述两条转换流彼此的一致性，并非相对于外部真值的绝对精度。

### 8.3 Sonic--MuJoCo Replay

两侧在本组 Replay 中均未触发跌倒判定。ZeroLab 的 tracking RMSE 为 **0.08833 rad**，略低于 PICO 的 **0.09144 rad**；这只说明机器人对这两条各自 Sonic 目标的跟踪结果不同，**不等于 ZeroLab 动捕更准确**。限位、接触等其它 Replay 维度也不合成为单一设备排名。

## 9. Body：同次配对录制（60 帧临时诊断标定）

**本节全部数值均来自 60 帧临时诊断标定配置。**Body 是同次同步配对录制，但这一临时配置不能用于评价 100 帧正式配置的最终能力上限，也不外推为任一设备的总体能力。两侧完整 Replay 均为 5,143 样本，使用 50 Hz 控制率、10 个 substeps，且 `reference.source=live`。

### 9.1 原始 Timing 与 Alignment（60 帧临时诊断配置）

在 **60 帧临时诊断配置** 下，公共区间为 **103.02 s**。ZeroLab 的原始平均接收率为 **35.700 Hz**，超过 30 ms 的原始 Gap 占 **37.55%**。为进行同步配对而重采样至 50 Hz 只定义公共比较栅格，不能抹去原始接收率和 Gap；这一时序结果不外推到 100 帧正式配置或其他动作组。

### 9.2 Canonical 与动态 agreement（60 帧临时诊断配置）

| 指标（Body 组内、60 帧临时诊断配置） | RMSE |
|---|---:|
| SMPL pose（°） | **24.775** |
| root orientation（°） | **21.069** |
| SMPL FK local position（m） | **0.1395** |
| pose angular velocity（°/s） | **380.618** |

这些结果仅是 **60 帧临时诊断配置** 下两条 canonical 转换流在本 Body 配对内的 agreement，不是绝对人体动捕精度，也不用于推断 100 帧正式标定的结果。380.618°/s 的动态姿态差异应与 pose、root 和位置 RMSE 分列阅读，不合成为跨组综合分数。

### 9.3 Sonic--MuJoCo Replay（60 帧临时诊断配置）

在 **60 帧临时诊断配置** 下，PICO 与 ZeroLab 的 Replay tracking RMSE 分别为 **0.11847 rad** 和 **0.13629 rad**；ZeroLab 有 **8 个限位样本**，两侧均未触发跌倒判定。这些 Replay 指标只描述机器人跟踪各自 Sonic 目标的难度和该临时配置下的安全诊断，不能解释为动捕绝对精度，更不能外推到 100 帧正式配置。

## 10. PICO 的优点与缺点

本节仅从五组中重复出现且可由固定证据追溯的现象归纳 PICO 的优点、局限与适用条件；不以单一指标替代其他维度，也不作绝对准确度声明。

- **数据接收。**现有展开数字不足以形成 PICO 接收优势：Combination/Body 只证明 ZeroLab 分别出现 37.337/35.700 Hz、原始 Gap 占 29.491%/37.55% 的连续输入风险，没有章节内 PICO 同组 timing 对照或预定义判负阈值，不能据此推荐 PICO。已具同组数字的 Single-joint 中，PICO 接收率为 48.589 Hz、Gap 占 11.98%，均弱于 ZeroLab 的 49.183 Hz、7.00%。
- **姿态。**Static 的 T 阶段设备间 pose RMSE 为 13.731°，低于 N_after_T 的 30.582°和 A 的 23.754°，说明两条流在标准 T 名义姿态上更接近。缺点是该指标为双方 agreement，不能将较小差异单独归功于 PICO；Single-joint 的右肩 63.426° RMSE 和六个非零 wrist 差异通道要求先校核两侧肩腕映射并拆分差异来源，确认后再修正。
- **动态平滑。**这是 PICO 最稳定的相对优势：Static 三阶段的 pose 加速度 RMS 为 ZeroLab 的约 1/3.4 至 1/10.5，jerk 为约 1/4.0 至 1/11.5；Single-joint 的姿态加速度/jerk 为 642.300/53,270.909，对方为 3,847.818/319,723.356，Dynamic 也呈 PICO 较低的方向。限制是平滑只描述高频变化，不证明动作幅值或真人姿态更正确。
- **Replay。**优点是 60 帧临时诊断 Body 的 tracking RMSE 较低（0.11847 对 0.13629 rad），Single-joint 的最大误差也较低（1.3889 对 1.8996 rad）。缺点是 Single-joint 的 RMSE、P95、速度 RMSE 均高于 ZeroLab，Combination tracking RMSE 也略高（0.09144 对 0.08833 rad）。
- **安全性。**Static 无限位、无跌倒，Body 未跌倒。缺点是 Single-joint 出现 23 个限位样本，而 ZeroLab 为 0；Dynamic 在 sample 6157 因最大倾角 61.282°触发 `fell=true`。该事件是阈值触发且最低基座高度仍为 0.813 m，不能扩大为真实倒地结论。
- **部署条件。**现有证据仅覆盖固定离线产物和 live reference Replay，未量化佩戴、遮挡、传感器数量、搭建时间或现场维护成本，因此不能从本报告给 PICO 部署便利性排序；工程选型需另行实测这些条件。

## 11. ZeroLab 的优点与缺点

本节仅从五组中重复出现且可由固定证据追溯的现象归纳 ZeroLab 的优点、局限与适用条件；较低 Replay tracking RMSE 只表示该段目标更易被机器人跟踪，不表示更接近真人动作。

- **数据接收。**优点是 Single-joint 的平均接收率为 49.183 Hz，原始 Gap 占 7.00%，优于同组 PICO 的 48.589 Hz 和 11.98%。缺点是在 Combination/Body 中分别降至 37.337/35.700 Hz，超过 30 ms 的原始 Gap 升至 29.491%/37.55%，对依赖连续实时输入的任务构成直接限制。
- **姿态。**Static 的 T/N_after_T/A pose agreement 为 13.731°/30.582°/23.754°，Single-joint 的位置 RMSE 为 0.0831 m；这些数值证明两条流仍有明显场景差异。缺点与 PICO 相同：右肩 63.426° RMSE、六个 wrist 通道非零差异和快速动作的高动态误差尚不能归因到单侧设备，必须通过外部真值或受控映射实验拆分来源。
- **动态平滑。**当前主要缺点是已报告多组的高频变化更强：Static 三阶段 pose 加速度为 PICO 的约 3.4–10.5 倍、jerk 为约 4.0–11.5 倍；Single-joint 的姿态加速度和 jerk 均约为 PICO 的 6.0 倍，Dynamic 亦高于 PICO。平滑性较弱不等价于绝对姿态误差更大。
- **Replay。**优点是 Single-joint 的 tracking RMSE/速度 RMSE 为 0.0644 rad/0.5746 rad/s，低于 PICO 的 0.0693/0.6973；Combination 的 0.08833 rad 也略低于 PICO 的 0.09144。缺点是 60 帧临时诊断 Body 的 RMSE 较高（0.13629 对 0.11847 rad），Static 的三个阶段也有胜有负，不能外推为普遍更易跟踪。
- **安全性。**五组均未触发跌倒，Single-joint 为 0 个限位样本，Static 也无限位。缺点是 60 帧临时诊断 Body 有 8 个限位样本；此外，`fell`、限位、最大倾角和 tracking 误差是不同维度，不能因“未跌倒”就忽略其它风险信号。
- **部署条件。**当前录制没有给出佩戴、遮挡、传感器数量、布置时间或环境适应性的量化对照，因此不对 ZeroLab 的部署门槛作数据外推；尤其不能用 Combination/Body 的接收抖动替代专门的遮挡或佩戴实验。

## 12. 综合性能分析与适用场景

综合性能依据五种场景中可重复或可明确限定的证据形成；不同动作难度的 RMSE 不直接平均，单组异常保留场景限定。

| 场景 / 维度 | 建议 | 数据依据 | 适用限制 |
|---|---|---|---|
| 稳定采集和动态平滑 | 连续实时输入任务需重点验证 ZeroLab 在 Combination/Body 暴露的接收风险，但不据此推荐 PICO；平滑目标优先时可基于已报告 smoothness 评估 PICO。 | ZeroLab 在 Combination/Body 为 37.337/35.700 Hz、Gap 占 29.491%/37.55%；PICO 在 Static 三阶段、Single-joint 和 Dynamic 的已报告平滑性均较好。Single-joint 的同组 timing 则是 ZeroLab 49.183 Hz、7.00% 优于 PICO 48.589 Hz、11.98%。 | Combination/Body 正文没有 PICO 同组 timing，且未定义单独判负阈值；平滑性不代表绝对精度，50 Hz 重采样结果也不能替代原始 Gap。 |
| 静态名义姿态 | T-pose 复现可作为双方标定检查点；需要更平滑的稳定姿态输入时优先试 PICO，但不据此选择“更准确”的设备。 | Static T/N_after_T/A 的 pose RMSE 为 13.731°/30.582°/23.754°；三阶段 PICO 的 pose 加速度和 jerk 均低于 ZeroLab。 | PICO013/ZeroLab014 为非同步跨 Trial；只比较共同名义阶段，不能报告时序、逐帧误差或单侧绝对精度。 |
| 单关节与末端 | 机器人跟踪和限位优先时，Single-joint 可评估 ZeroLab；无论设备，先校核肩腕映射并拆分差异来源，确认后再修正。 | ZeroLab/PICO tracking RMSE 为 0.0644/0.0693 rad、限位为 0/23；右肩 RMSE 63.426°，位置 RMSE 0.0831 m，六个 wrist 通道均有非零差异。 | tracking 是各自目标的可跟踪性；肩腕 agreement 差异不能定位到单侧硬件、算法或映射环节。 |
| 复杂动作 | 没有跨 Dynamic、Combination、Body 的统一赢家：输入平滑优先时评估 PICO；Dynamic 的跌倒阈值与 Combination 的易跟踪性优先时评估 ZeroLab；Body 临时配置下则先评估 PICO Replay。 | 三组动态姿态差异为 372.359/637.417/380.618°/s；ZeroLab Dynamic 未触发跌倒且 Combination RMSE 较低，PICO Body RMSE 较低，ZeroLab Body 有 8 个限位样本。 | Combination 有接近 180° 的单点异常；Body 仅为 60 帧临时诊断；各动作复杂度和时长不同，不能跨组平均。 |
| 实时 Sonic 可执行性 | 两套设备均已在五组产物中完成 Replay，但上线前应按目标动作分别验收 tracking、限位、倾角和基座高度，不以单一安全标签放行。 | Static 双方无跌倒/限位；Single-joint 双方未跌倒但限位 23/0；Dynamic 仅 PICO 触发倾角阈值；Combination 双方未跌倒；Body 双方未跌倒且 ZeroLab 有 8 个限位样本。 | Replay tracking 不是动捕精度；`fell` 是阈值判据，contact 与 self-collision 是受时长/substeps 影响的累计量，且现有产物缺少完整代码/模型哈希。 |
| 部署条件 | 暂不基于本五组数据推荐设备；应在目标现场另测佩戴、遮挡、传感器配置、布置时间和维护成本。 | 本报告数据源没有这些维度的量化对照。 | 不得用接收率、canonical 平滑性或 Replay 结果替代部署条件测试。 |

## 13. 局限与后续改进优先级

### 13.1 证据与解释局限

- **无绝对真值。**Agreement 只衡量两条转换流彼此的一致程度；没有光学动捕或其它外部 ground truth，无法把差异归因到某一设备，也不能输出绝对精度结论。Replay tracking 只衡量机器人跟踪各自 Sonic 目标的难度，不是动捕精度。
- **Static 非同步。**PICO013 与 ZeroLab014 是**非同步跨 Trial**，只允许比较 T、N_after_T、A 的名义稳定阶段；ZeroLab 末尾 direct-T/N 因 PICO 无对应动作而排除。样本重复性、起始姿态和录制差异均可能进入结果，因此不得推导延迟、互相关或逐帧动态误差。
- **Body 配置临时。**Body 使用 **60 帧**临时诊断标定，不能代表 100 帧正式标定的性能上限，亦不能把该组结果外推到其它动作。
- **缺少逐动作标签。**除 Static 可识别名义阶段外，动态录制没有逐动作标签；Dynamic、Combination、Body 的全段统计无法定位到具体动作、转场或异常来源。
- **重采样会隐藏接收问题。**同步配对数据重采样至 50 Hz 后呈固定 20 ms 间隔，但这不会消除原始 Gap；必须同时保留原始接收率、P95/最大间隔和超过 30 ms 的比例。
- **Replay 可追溯性有限。**固定 report 记录了来源、样本、控制率、substeps 与 live reference，但未记录完整代码提交、转换链、控制器、策略权重和 MuJoCo 模型哈希；未来软件或模型变化后不能仅凭路径保证逐位复现。
- **29 DoF 被零通道稀释。**ELF3 `joint_pos[29]` 只有六个 wrist 通道承载设备差异，其余 23 个通道为零；整体 RMSE 会压低腕部差异，必须与六通道结果并列解释。
- **四元数统计受实现定义约束。**root orientation 等统计依赖现有实现对四元数归一化、双覆盖符号与相对旋转角的处理；当前没有独立实现的交叉校验，复现时必须保持同一定义，不能直接与采用不同聚合方式的结果混用。
- **stability/contact 语义有限。**Canonical stability 是人体参考低运动窗口，不是跌倒判定；Replay 的 `fell` 才是阈值事件。`contact_count` 是任意接触的仿真子步累计，`self_collision_count` 是接触事件累计而非唯一碰撞数，且二者受时长和 substeps 影响，不能跨时长直接排名。
- **部署证据缺失。**五组数据没有佩戴、遮挡、传感器数量、搭建时间与环境适应性的量化对照，不能从现有数字推导部署优劣。

### 13.2 后续改进优先级

1. **ZeroLab 接收抖动：**先定位 Combination/Body 中 37.337/35.700 Hz 与 29.491%/37.55% Gap 的采集、传输和时间戳来源，并保留修复前后同动作原始 timing。
2. **肩腕映射：**用受控单关节动作校核两侧映射并拆分右肩 63.426° agreement 差异来源，逐一检查六个 wrist 通道的坐标系、符号、零位和限幅；确认异常侧或实现环节后再修正。
3. **动作标签：**为 Dynamic、Combination、Body 写入逐动作与转场标签，使异常、平滑性和 Replay 事件能回溯到动作片段。
4. **可追溯哈希：**在 canonical、metrics 与 Replay report 中固化录制源、代码提交、转换配置、控制器、策略权重和 MuJoCo 模型哈希。
5. **外部 ground truth：**使用同步光学动捕或等价测量建立绝对参考，并在相同 Trial、相同动作和相同标定配置下重新比较两套设备。

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
