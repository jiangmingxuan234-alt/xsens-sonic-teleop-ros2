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

本节只在五组分别完成核对后，以场景限定的数值和方向性发现归纳重复差异；时序、静态姿态、动态平滑性与 Replay 安全性相互独立判断。

## 5. Static：PICO013 与 ZeroLab014 跨 Trial 对照

本节只解释 PICO013 与 ZeroLab014 的相同名义动作跨 Trial 对照。比较范围为 T-pose、T-pose 后自然站立和 A-pose的 nominal pose agreement、root、FK position、ELF3 canonical wrist 通道、平滑性与 Replay 指标；结论明确标注跨 Trial 性质，不将其视为同步设备差异。

## 6. Single-joint：同次配对录制

本节按第 3 节口径呈现单关节组的 Timing、Alignment、canonical/动态一致性、平滑性和 Replay，重点解释单关节映射的可解释性，不将本组结果外推为其他动作的总体排名。

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
