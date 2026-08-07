# PICO 与 ZeroLab 五组实验综合对比报告 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成一份可追溯的中文 Markdown 报告，完成五组实验内的 PICO–ZeroLab 对比、设备优缺点归纳和综合性能分析。

**Architecture:** 报告以固定 JSON/NPZ 产物作为证据源。Static 使用 PICO013 与 ZeroLab014 的共同名义动作阶段进行跨 Trial 对照；其余四组使用同次配对录制的公共 canonical 时间轴。先锁定证据和口径，再完成逐组分析，最后仅根据跨场景重复趋势形成综合结论，不创建无依据的总分。

**Tech Stack:** Markdown、Git、Bash、`jq`、Python/NumPy（只读核验 NPZ）、现有 ZeroLab/PICO evaluation JSON。

## Global Constraints

- 最终文件固定为 `docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md`。
- 不修改算法、原始录制、canonical NPZ、alignment JSON、ONNX 或 MuJoCo 模型。
- 不覆盖 `/tmp` 中任何既有报告；本任务优先引用已验证的完整产物。
- 每组内部比较；不同动作组不拼接、不按时长加权、不计算无预定义权重的综合分数。
- Static 明确标注“相同名义动作、非同步跨 Trial”；不计算跨设备延迟、互相关或逐时间点动态误差。
- Body 明确标注 `calibration_frames=60` 临时诊断配置；其结论不外推为 100 帧正式配置上限。
- Agreement 不描述为绝对人体真值精度；Replay tracking 不描述为动捕准确率。
- 29 DoF canonical `joint_pos` 同时说明 23 个零通道的稀释效应，并优先解释六个 wrist 通道。
- Canonical stability、Replay stability、contact count 和 self-collision count 按设计文档中的语义分别解释。

---

### Task 1: 锁定证据源并建立报告骨架

**Files:**
- Create: `docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md`
- Read: `docs/superpowers/specs/2026-08-07-pico-zerolab-five-trial-comparison-report-design.md`

**Interfaces:**
- Consumes: 下列固定的 JSON/NPZ 评估产物。
- Produces: 带有完整结构、实验表、指标定义和证据路径表的 Markdown；后续任务只向既定章节补充数值与结论，执行摘要在综合分析完成后写入。

固定证据源：

```text
Static PICO canonical: /tmp/pair-static-013-eval-final-1785926456/pico_canonical.npz
Static PICO replay: /tmp/pico-sonic-mujoco-live-full-002/report.json
Static ZeroLab canonical: /tmp/pair-static-014-zerolab-formal/zerolab_formal_canonical.npz
Static ZeroLab replay: /tmp/pair-static-014-zerolab-formal-mujoco-001/report.json
Static cross-trial analysis: /tmp/pair-static-014-zerolab-formal/comparison_analysis.json

Single metrics: /tmp/pair-single-joint-metrics-verify-sl9CEz/report.json
Single PICO replay: /tmp/pico-single-joint-mujoco-Ahx7zb/report.json
Single ZeroLab replay: /tmp/zerolab-single-joint-mujoco-full-80q2PY/report.json

Dynamic metrics: /tmp/pair-dynamic-003-metrics-HFfZ7D/report.json
Dynamic PICO replay: /tmp/pair-dynamic-003-pico-mujoco-siZHzq/report.json
Dynamic ZeroLab replay: /tmp/pair-dynamic-003-zerolab-mujoco-EOOe4f/report.json

Combination metrics: /tmp/pair-combation8-metrics-RHbW1v/report.json
Combination PICO replay: /tmp/pair-combation8-pico-full-agent/report.json
Combination ZeroLab replay: /tmp/pair-combation8-zero-full-agent/report.json

Body metrics: /tmp/pair-body-002-cal60-metrics-agent/report.json
Body PICO replay: /tmp/pair-body-002-cal60-pico-full-agent/report.json
Body ZeroLab replay: /tmp/pair-body-002-cal60-zero-full-agent/report.json
```

- [ ] **Step 1: 验证所有固定证据源存在且 JSON 可解析**

Run:

```bash
cd /home/fazepurple/ros2_ws/bxi_rl_controller_ros2_example_dev
python3 -m json.tool /tmp/pair-static-014-zerolab-formal/comparison_analysis.json >/dev/null
python3 -m json.tool /tmp/pair-single-joint-metrics-verify-sl9CEz/report.json >/dev/null
python3 -m json.tool /tmp/pair-dynamic-003-metrics-HFfZ7D/report.json >/dev/null
python3 -m json.tool /tmp/pair-combation8-metrics-RHbW1v/report.json >/dev/null
python3 -m json.tool /tmp/pair-body-002-cal60-metrics-agent/report.json >/dev/null
for path in \
  /tmp/pair-static-013-eval-final-1785926456/pico_canonical.npz \
  /tmp/pico-sonic-mujoco-live-full-002/report.json \
  /tmp/pair-static-014-zerolab-formal/zerolab_formal_canonical.npz \
  /tmp/pair-static-014-zerolab-formal-mujoco-001/report.json \
  /tmp/pair-static-014-zerolab-formal/comparison_analysis.json \
  /tmp/pair-single-joint-metrics-verify-sl9CEz/report.json \
  /tmp/pico-single-joint-mujoco-Ahx7zb/report.json \
  /tmp/zerolab-single-joint-mujoco-full-80q2PY/report.json \
  /tmp/pair-dynamic-003-metrics-HFfZ7D/report.json \
  /tmp/pair-dynamic-003-pico-mujoco-siZHzq/report.json \
  /tmp/pair-dynamic-003-zerolab-mujoco-EOOe4f/report.json \
  /tmp/pair-combation8-metrics-RHbW1v/report.json \
  /tmp/pair-combation8-pico-full-agent/report.json \
  /tmp/pair-combation8-zero-full-agent/report.json \
  /tmp/pair-body-002-cal60-metrics-agent/report.json \
  /tmp/pair-body-002-cal60-pico-full-agent/report.json \
  /tmp/pair-body-002-cal60-zero-full-agent/report.json
do
  test -f "$path"
done
```

Expected: 五条命令均退出码 0。随后对上述清单中的 15 个 JSON 和 2 个 Static NPZ 执行 `test -f`，全部成功。

- [ ] **Step 2: 创建报告骨架和口径章节**

报告必须按此顺序创建章节：

```markdown
# PICO 与 ZeroLab 五组人体动捕及 ELF3 Replay 综合对比报告

## 执行摘要
## 1. 评估目标与结论边界
## 2. 五组实验与动作内容
## 3. 指标定义与数据口径
## 4. 五组核心结果总览
## 5. Static：PICO013 与 ZeroLab014 跨 Trial 对照
## 6. Single-joint：同次配对录制
## 7. Dynamic：同次配对录制
## 8. Combination：同次配对录制
## 9. Body：同次配对录制（60帧临时标定）
## 10. PICO 的优点与缺点
## 11. ZeroLab 的优点与缺点
## 12. 综合性能分析与适用场景
## 13. 局限与后续改进优先级
## 14. 数据源与复现路径
```

第 1–3、14 节直接写清 Global Constraints，不使用 `TBD`、`TODO` 或空表格。第 2 节列出五组动作和 Static 的跨 Trial 特例；第 14 节列出固定证据源及各自用途。

- [ ] **Step 3: 验证骨架、来源和禁止项**

Run:

```bash
rg -n '^## ' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
rg -n 'pair-static-013|pair-static-014|pair-single-joint|pair-dynamic-003|pair-combation8|pair-body-002' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
rg -n 'TBD|TODO|待补|占位' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
```

Expected: 14 个编号章节全部存在；六类源名称全部出现；最后一条命令无输出并返回 1。

- [ ] **Step 4: 提交证据与口径骨架**

```bash
git add docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git commit -m "docs: establish five-trial comparison evidence"
```

### Task 2: 完成 Static 与 Single-joint 详细分析

**Files:**
- Modify: `docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md`

**Interfaces:**
- Consumes: Task 1 的口径章节、Static cross-trial analysis、Static 两侧 Replay、Single metrics 和 Single 两侧 Replay。
- Produces: 第 5、6 节的完整表格、逐指标解释和组内结论；第 4 节增加 Static 与 Single 的一行摘要。

- [ ] **Step 1: 提取 Static 共同动作和 Replay 数值**

Run:

```bash
jq '{scope, formal_bounds, replay_direct, matched_stable_phases}' /tmp/pair-static-014-zerolab-formal/comparison_analysis.json
jq '{reference, metrics}' /tmp/pico-sonic-mujoco-live-full-002/report.json
jq '{reference, metrics}' /tmp/pair-static-014-zerolab-formal-mujoco-001/report.json
```

Expected: 共同稳定阶段包含 `T`、`N_after_T`、`A`；PICO Replay 为 2222 样本/44.42 s，ZeroLab formal Replay 为 1653 样本/33.04 s；两侧均无越限和跌倒。

- [ ] **Step 2: 写 Static 跨 Trial 分析**

第 5 节必须包含：

- PICO013 与 ZeroLab014 并非同步录制，禁止解释时间延迟。
- T、自然站立、A 三阶段的 pose/root/position/ELF3 29维 agreement 表；明确 ELF3 整体值主要由六个 wrist 通道贡献。
- 两侧分阶段 Replay RMSE、P95、速度跟踪和倾角表。
- 两侧 formal/full Replay 时长差异及不可直接比较 contact 总数的说明。
- ZeroLab 姿态加速度与 jerk 高于 PICO 的量化说明。
- 结论限定为名义姿态复现一致性，不宣称绝对准确率。

- [ ] **Step 3: 提取并写 Single-joint 配对分析**

Run:

```bash
jq '{timing, alignment, agreement, smoothness, stability}' /tmp/pair-single-joint-metrics-verify-sl9CEz/report.json
jq '{reference, metrics}' /tmp/pico-single-joint-mujoco-Ahx7zb/report.json
jq '{reference, metrics}' /tmp/zerolab-single-joint-mujoco-full-80q2PY/report.json
```

第 6 节必须报告 114.16 s 公共区间、0 ms 延迟、0.4795 相关系数、22.569° pose RMSE、6.694° root RMSE、0.0831 m position RMSE 和 313.416°/s 动态姿态差异；指出右肩为最大关节误差。Replay 表必须列出两侧均 5700 样本/113.98 s，以及 PICO 23 个限位样本、ZeroLab 0 个限位样本。

- [ ] **Step 4: 验证关键数字并提交**

Run:

```bash
rg -n '2222|1653|22\.569|313\.416|5700|23 个限位' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git diff --check
```

Expected: 所有关键数字至少出现一次；`git diff --check` 无输出。

```bash
git add docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git commit -m "docs: analyze static and single-joint trials"
```

### Task 3: 完成 Dynamic、Combination 与 Body 详细分析

**Files:**
- Modify: `docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md`

**Interfaces:**
- Consumes: 三组固定 metrics 与六份完整 Replay report。
- Produces: 第 7–9 节完整分析；第 4 节补齐三组摘要，使五组总览完整。

- [ ] **Step 1: 提取三组指标和 Replay 报告**

Run:

```bash
jq '{timing, alignment, agreement, smoothness, stability}' /tmp/pair-dynamic-003-metrics-HFfZ7D/report.json
jq '{timing, alignment, agreement, smoothness, stability}' /tmp/pair-combation8-metrics-RHbW1v/report.json
jq '{timing, alignment, agreement, smoothness, stability}' /tmp/pair-body-002-cal60-metrics-agent/report.json
jq '{reference, metrics}' /tmp/pair-dynamic-003-pico-mujoco-siZHzq/report.json
jq '{reference, metrics}' /tmp/pair-dynamic-003-zerolab-mujoco-EOOe4f/report.json
jq '{reference, metrics}' /tmp/pair-combation8-pico-full-agent/report.json
jq '{reference, metrics}' /tmp/pair-combation8-zero-full-agent/report.json
jq '{reference, metrics}' /tmp/pair-body-002-cal60-pico-full-agent/report.json
jq '{reference, metrics}' /tmp/pair-body-002-cal60-zero-full-agent/report.json
```

Expected: Dynamic Replay 各 6383 样本；Combination 各 3992；Body 各 5143。每对均为 50 Hz、10 substeps、`reference.source=live`。

- [ ] **Step 2: 写 Dynamic 分析**

第 7 节报告 26.068° pose RMSE、10.204° root RMSE、0.0942 m position RMSE、372.359°/s 动态姿态差异，以及 ZeroLab 的加速度/jerk 高于 PICO。Replay 必须说明 PICO `fell=true` 源于 sample 6157 最大倾角 61.282° 越过阈值，而最低高度 0.813 m，不能描述为倒地；ZeroLab 未触发跌倒判定。

- [ ] **Step 3: 写 Combination 分析**

第 8 节报告 ZeroLab 原始接收率 37.337 Hz、29.491% Gap、21.956° pose RMSE、637.417°/s 动态姿态差异和接近 180° 的单点最大姿态误差。Replay 说明两侧均未跌倒、ZeroLab tracking RMSE 0.08833 rad 略低于 PICO 0.09144 rad，但这不等于动捕更准确。

- [ ] **Step 4: 写 Body 分析并标记 60 帧配置**

第 9 节报告 103.02 s 公共区间、ZeroLab 原始接收率 35.700 Hz、37.55% Gap、24.775° pose RMSE、21.069° root RMSE、0.1395 m position RMSE和 380.618°/s 动态姿态差异。Replay 报告 PICO/ZeroLab RMSE 0.11847/0.13629 rad、ZeroLab 8 个限位样本和两侧均未跌倒；整节始终注明 60 帧临时诊断配置。

- [ ] **Step 5: 验证三组数据并提交**

Run:

```bash
rg -n '61\.282|37\.337|637\.417|35\.700|60 帧|8 个限位' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git diff --check
```

Expected: 六项证据均出现；`git diff --check` 无输出。

```bash
git add docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git commit -m "docs: analyze dynamic combination and body trials"
```

### Task 4: 形成设备优缺点、综合结论并完成证据审计

**Files:**
- Modify: `docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md`

**Interfaces:**
- Consumes: Task 2、3 的五组定量结论。
- Produces: 完整执行摘要、两套设备优缺点、场景选择建议、局限和最终可交付 Markdown。

- [ ] **Step 1: 写 PICO 与 ZeroLab 优缺点**

第 10、11 节分别按“数据接收、姿态、动态平滑、Replay、安全性、部署条件”组织。只陈述五组数据能够支持的结论：PICO 多数组中 canonical 更平滑；ZeroLab 在部分 Replay 中目标更易跟踪且部分组无越限；ZeroLab 接收质量在 Combination/Body 明显下降；两套设备都存在肩、腕和快速动作差异。对设备佩戴方式、遮挡和传感器数量只引用已知系统事实，不用当前数据虚构数值。

- [ ] **Step 2: 写综合性能与场景建议**

第 12 节使用维度式结论，不输出总分：

- 稳定采集和动态平滑：依据多组 timing/smoothness 趋势。
- 静态名义姿态：依据 Static 的 T/N/A 阶段结果。
- 单关节与末端：依据 Single-joint 肩腕误差。
- 复杂动作：依据 Dynamic/Combination/Body 的动态误差与 Replay 稳定性。
- 实时 Sonic 可执行性：依据五组 Replay 跟踪、限位和跌倒指标。

每个建议同时列出数据依据和适用限制。

- [ ] **Step 3: 写执行摘要、局限和改进优先级**

执行摘要用 6–10 条短结论概括结果。第 13 节必须包含：无绝对真值、Static 非同步、Body 60 帧、无逐动作标签、重采样掩盖原始 Gap、Replay 未记录完整代码/模型哈希、29 DoF 零通道稀释以及四元数统计实现限制。改进优先级按“ZeroLab 接收抖动、肩腕映射、动作标签、可追溯哈希、外部 ground truth”排序。

- [ ] **Step 4: 完成结构、数字和语义审计**

Run:

```bash
cd /home/fazepurple/ros2_ws/bxi_rl_controller_ros2_example_dev
rg -n '^## ' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
rg -n 'TBD|TODO|待补|占位|绝对更准确|综合得分' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
rg -n 'Agreement|Replay tracking|60 帧|非同步跨 Trial|wrist|跌倒' docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git diff --check
```

Expected: 14 个编号章节与执行摘要存在；禁止项搜索无输出；六个口径关键词均出现；`git diff --check` 无输出。

- [ ] **Step 5: 核对只修改目标报告并提交**

Run:

```bash
git status --short
git diff --name-only HEAD~3..HEAD
```

Expected: 本计划产生的提交只涉及目标报告；工作树中原有用户修改保持不变。

```bash
git add docs/reports/2026-08-07-pico-zerolab-five-trial-comparison-zh.md
git commit -m "docs: synthesize PICO and ZeroLab performance"
```
