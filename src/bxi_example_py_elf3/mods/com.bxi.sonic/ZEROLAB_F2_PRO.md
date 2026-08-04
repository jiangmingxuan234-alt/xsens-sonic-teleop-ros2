# ZeroLab F2 Pro 现场接入与采集

本指南用于首次联调、原始数据录制和 MuJoCo 实时验证。Windows 电脑运行
ZeroLab 与 MotionCaptureMaster，Ubuntu 电脑运行本 Mod 的适配器。Windows 端只需配置
持续 UDP Stream Output，不需要编写或运行自定义 Windows 接收代码。

## 1. 网络与 Stream Output

1. 确认 Windows 和 Ubuntu 位于可互通的局域网，并记录 Ubuntu 有线网卡的 IPv4 地址。
2. 在 ZeroLab/MotionCaptureMaster 的 Stream Output 配置中，将目标地址设为 Ubuntu IPv4，
   协议设为 UDP，端口设为 `18000`，发送频率设为 `50 Hz`，数据格式设为固定
   `992` 字节的 full-body packet。不要启用额外 header、prefix 或 suffix。
3. 点击 **Start/Enable Stream Output**。启用后数据会连续发送，直到点击
   **Stop/Disable Stream Output** 或退出应用程序。
4. 在 Windows 防火墙中允许 ZeroLab 和 MotionCaptureMaster 的网络访问，在 Ubuntu
   防火墙中允许入站 UDP `18000`。使用的防火墙不是 UFW 时，应配置等价规则；UFW 示例为：

   ```bash
   sudo ufw allow 18000/udp
   ```

5. 启动接收程序后，在 Ubuntu 确认端口只有一个 owner：

   ```bash
   ss -lunp | rg ':18000\b'
   ```

   若没有数据或包率异常，抓取到达 Ubuntu 的原始 UDP 包：

   ```bash
   sudo tcpdump -ni any 'udp port 18000'
   ```

`record-only` 与实时 `sonic_zerolab` 状态互斥，因为二者都需要独占 UDP `18000`。
不要在 `sonic_zerolab` 活跃时启动录制 CLI；切换用途前先停止当前 owner，并用 `ss`
确认端口已释放。

## 2. 原始数据录制

从仓库根目录执行以下命令，并把 `WINDOWS_IPV4` 替换为实际的 Windows 地址：

```bash
cd src/bxi_example_py_elf3/mods/com.bxi.sonic
python3 -m zerolab.record_cli \
  --output /tmp/zerolab-tpose-001 \
  --bind-host 0.0.0.0 \
  --port 18000 \
  --allowed-sender WINDOWS_IPV4
```

CLI 会持续录制到按 `--output` 指定的新目录。按 Ctrl-C 会停止接收，并在退出前统一
flush、fsync 和关闭记录文件。也可用 `--duration-seconds 5` 设置自动停止时间；值为
`0` 时持续运行到 Ctrl-C 或 SIGTERM。绑定、参数或写盘失败会打印简短错误并以状态码
`2` 退出，不要把失败运行当作有效 capture。

首次验收应生成以下五个具名录制，分别使用独立输出目录：

1. `zerolab-tpose-001`：稳定 T-pose，持续五秒。
2. `zerolab-rigid-yaw-001`：身体作为刚体向左、向右 yaw，不改变肢体相对姿态。
3. `zerolab-left-elbow-001`：只做左肘屈伸。
4. `zerolab-left-leg-001`：只抬左腿，再回到起始姿态。
5. `zerolab-shoulder-arm-001`：耸肩并抬臂，覆盖 shoulder shrug 和 arm elevation。

对这组录制逐项检查：有效 packet rate 是否接近 `50 Hz`；每包是否恰为 `992` 字节且
不存在隐藏 header；四元数轴、`xyzw` 顺序和正负号是否符合动作；手掌姿态语义是否
符合实际左右手；肩部传感器是否在耸肩与抬臂时产生预期响应。出现包长、包率或轴向
异常时，保留原始录制和 `tcpdump` 证据，不要通过猜测 offset 修正数据。

首个发布版本将解码后的 `root_translation` 和 17 个 body positions 仅用于诊断；它们
不生成 ELF3 关节目标。手指数据和左右手的 `uint16` hand values 同样不驱动 ELF3。
原始 recorder 仍完整保留每个 992 字节 packet，便于离线核查这些诊断字段。

## 3. MuJoCo 实时验证

确保 record-only CLI 已退出，然后启动现有仿真：

```bash
ros2 launch bxi_example_py_elf3 example_demo.launch.py
```

触发 `com.bxi.sonic/activate_zerolab`（`btn_10=4`）进入 ZeroLab SONIC。受试者应保持
ZeroLab 与 PICO 都兼容的共同 T-pose 至少两秒，随后继续保持姿态完成初始 10 帧 fill；
看到 ready 日志后再开始移动。在 live data 尚未 ready 时不要用动作试探校准状态。

结束 ZeroLab 验证时，先回到 `com.bxi.basic_actions/normal`，确认相关端口已释放，再进入
PICO SONIC。不要从 ZeroLab SONIC 直接切入 PICO 数据路径。

## 4. PICO 与 ZeroLab 配对采集

同一受试者可以同时穿戴 PICO 与 ZeroLab。先分别完成两台设备的校准，再共同保持双方
兼容的 T-pose。在同一台 Ubuntu 电脑上启动现有 PICO recorder 和本指南的 ZeroLab
record-only CLI，然后执行相同的同步动作并分别保存数据。

配对录制不等于双源实时控制：不得把 PICO 和 ZeroLab 同时送入 SONIC。两个录制器可
同时采集，但实时策略每次只能选择一个 source。跨设备延迟估计与公共 `50 Hz` 重采样
属于后续评估工作，不是首个发布版本的在线处理能力。
