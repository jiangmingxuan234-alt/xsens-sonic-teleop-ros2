# PICO 与 ZeroLab 五组实验综合对比报告设计

## 目标

基于五组既有录制与离线评估产物，生成一份中文 Markdown 报告，分别说明 PICO 与 ZeroLab 在人体动捕一致性、时序质量、动态平滑性和 ELF3 MuJoCo Replay 表现上的优势、局限及综合适用性。

报告回答三个问题：

1. 在每组相同动作条件下，两套设备的 canonical 人体参考有多一致？
2. 两套参考经过 Sonic policy 和 MuJoCo 闭环后，各自的跟踪与稳定性如何？
3. 五种动作场景中反复出现的差异是什么，工程上应如何选择和改进？

## 输出

最终报告写入：

```text
docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
```

报告使用中文，保留指标英文缩写和原始单位，提供汇总表、逐组分析、优缺点和最终综合结论。

## 实验范围

| 报告名称 | PICO 来源 | ZeroLab 来源 | 比较方式 | 标定帧数 |
|---|---|---|---|---:|
| Static | pair-static-013 | pair-static-014 | 相同动作、非同步跨 Trial | 100 |
| Single-joint | pair-single-joint-001 | pair-single-joint-001 | 同次配对录制 | 100 |
| Dynamic | pair-dynamic-003 | pair-dynamic-003 | 同次配对录制 | 100 |
| Combination | pair-combation8 | pair-combation8 | 同次配对录制 | 100 |
| Body | pair-body-002 | pair-body-002 | 同次配对录制 | 60，临时诊断配置 |

保留磁盘上的历史拼写 `combation8` 以便追溯；报告正文使用规范名称“Combination”。用户输入的 `pair-body-oo2` 按现有目录 `pair-body-002` 解释。

## 数据源与非破坏性约束

- 只读取现有原始录制、canonical NPZ、alignment JSON、设备 metrics 和 Replay report。
- 不重新录制，不修改人体转换算法、标定算法、Sonic reference 构造或 MuJoCo 模型。
- 不覆盖既有 canonical 和历史报告。
- 如某组缺少完整且可比的产物，只在新的临时目录生成派生评估文件。
- 报告引用固定的产物路径，不对多个重复 Replay 版本求平均。

## 比较口径

### 四组同步配对实验

Single-joint、Dynamic、Combination 和 Body 使用各自的公共 canonical 50 Hz 时间轴进行组内比较。每组动作不同，因此每组独立解释，不把五组样本拼接成一个总体 RMSE，也不按录制时长加权生成总分。

每组依次报告：

1. 原始 Timing：样本数、时长、平均接收率、P95/最大帧间隔、超过 30 ms 的 Gap。
2. Alignment：公共区间、估计延迟、相关系数和质量标志。
3. Canonical Agreement：SMPL pose、root orientation、SMPL FK position、ELF3 canonical `joint_pos`。
4. Dynamic Agreement：姿态角速度 RMSE 和位置速度 RMSE。
5. Smoothness：姿态速度、加速度、jerk 以及位置加速度 RMS。
6. Replay：tracking RMSE/MAE/P95/Max、速度跟踪 RMSE、限位、跌倒、最低基座高度和最大倾角。

### Static 跨 Trial 实验

Static 使用 `pair-static-013` 的 PICO 与 `pair-static-014` 的 ZeroLab。两者动作方案相近，但不是同一时间采集，因此：

- 不报告跨设备接收延迟、互相关或逐时间点动态误差。
- 只对双方共同且可识别的稳定阶段配对：T-pose、T-pose 后自然站立、A-pose。
- 逐阶段比较 nominal pose agreement、root、FK position、ELF3 canonical wrist 通道、平滑性和 Replay 指标。
- 不完全对应的末尾动作不进入逐动作一致性统计。
- 全段 Replay 可并列展示，但必须同时列出样本数和时长；不把接触总次数直接比较为优劣。
- 所有 Static 结论标注为“相同名义动作的跨 Trial 对照”，其证据强度低于同步配对实验。

## 指标解释边界

### Agreement 不是绝对精度

PICO–ZeroLab canonical agreement 只衡量两条转换流之间的一致程度。当前没有光学动捕或其他绝对真值，不能把其中任一设备直接定义为 ground truth，也不能从较小的设备间 RMSE 推导出绝对准确率更高。

### Replay Tracking 不是动捕准确率

Replay tracking 衡量 ELF3 MuJoCo 模型对各自 Sonic 目标关节的跟踪程度。ZeroLab 的 tracking RMSE 小于 PICO，只能说明该段 ZeroLab 目标更容易被机器人跟踪，不能说明 ZeroLab 更接近真人动作。

### ELF3 canonical 29 DoF 的限制

当前 canonical `joint_pos[29]` 只有六个腕部通道出现设备差异，其余 23 个通道为零。29 维整体 RMSE 会被零通道稀释，因此报告同时给出整体值和六个 wrist DoF 的单独结果，并优先使用 wrist 指标解释设备差异。

### Stability 与 Contact 的限制

- Canonical stability 是人体参考的低运动窗口统计，不是 MuJoCo 跌倒判定。
- Replay stability 才包含 `fell`、基座高度和最大倾角。
- `contact_count` 是发生任意接触的仿真子步数，随时长和 substeps 增长。
- `self_collision_count` 是接触事件累计值，不是唯一自碰撞次数；跨时长比较时只作风险提示，不作为主排名指标。

## 综合结论规则

报告不创建没有预定义权重或阈值的“综合得分”。综合性能通过以下证据形成：

1. 五种场景中是否重复出现同方向差异。
2. 时序、静态姿态、动态平滑性和 Replay 安全性分别判断，不相互替代。
3. 使用组内数值和场景胜负描述，不对不同动作难度的 RMSE 直接求平均。
4. 对只有单组出现的异常保留场景限定，例如 Dynamic PICO 的倾角阈值事件。
5. 对 Body 的结论注明 60 帧临时标定，不能据此评价 100 帧正式配置的最终上限。

最终结论按以下维度呈现：

- 数据接收与时间稳定性
- 静态姿态与末端位置一致性
- 单关节映射可解释性
- 快速/组合动作平滑性
- Sonic–MuJoCo 可执行性与安全性
- 部署便利性、传感器覆盖和使用限制

## 报告结构

1. 执行摘要
2. 实验、动作和数据口径
3. 五组核心结果总览
4. Static 跨 Trial 分析
5. Single-joint 配对分析
6. Dynamic 配对分析
7. Combination 配对分析
8. Body 配对分析
9. PICO 的优点与缺点
10. ZeroLab 的优点与缺点
11. 综合性能对比与适用场景
12. 局限、风险和后续改进优先级
13. 数据源与复现路径

## 验证与验收

生成报告前后执行以下检查：

- 核对每个 report 的 source、samples、duration、controller rate 和 reference source。
- 确认完整 Replay 的样本数等于 canonical 帧数减去 9 帧滑窗预热。
- 确认每个同步配对组的 PICO/ZeroLab Replay 使用相同 canonical 长度和仿真参数。
- Static 按共同阶段核对样本数，避免将不对应动作配成同一阶段。
- 所有表格单位一致；角度、弧度、米和速度单位不混用。
- 对缺失、不可比或低置信度数据明确写明，不用零值代替。
- 报告中的每个关键数字均能追溯到固定 JSON/NPZ 产物。
- 不出现“ZeroLab/PICO 绝对更准确”等超出证据的结论。

验收标准是：读者能够从一份 Markdown 中看清每组 PICO–ZeroLab 差异、理解两类指标的含义边界，并获得有数据依据且不过度外推的设备选择建议。
