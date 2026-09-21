# Xsens MVN + SONIC 遥操交接手册（含 ZeroLab）

[English](./README.en.md)

本文的第一部分是 **Xsens MVN → SONIC** 的接手流程；文末保留原有的 ZeroLab F2 Pro
部署说明。下一位接手者应先完成本页的 Xsens 部署、网络验收和仿真验收，再在空载、急停
可触及的条件下接入实体机器人。

本仓库是 `com.bxi.sonic` Mod。新机器人通常已经预装 BXI 基础控制工程；客户不需要重新安装基础 ROS/硬件控制器。本说明按照官方 SONIC 文档的目录约定，把 SONIC/PICO 与 ZeroLab F2 Pro 动捕接入现有工程。

> **先读安全说明**：这是会给实体机器人发送关节目标的位置控制链路。第一次接手必须
> 在仿真或空载条件下进行，现场必须有人守在物理急停旁。`Ctrl-C` 只表示软件收到停止
> 请求，不等于按下实体急停；动作异常时先按实体急停，再检查日志。不要用 `kill -9`
> 代替急停。代码、激活码、个人 venv、录制的 `.mvnx`/原始数据都不要提交到公开仓库。

## Xsens 接手流程总览

按下面顺序执行，不要跳过网络探针：

1. 确认角色、地址和代码版本；机器人 IP 留给现场填写。
2. 在运行 ROS2 控制程序的 Linux 主机部署 BXI 依赖、编译 Mod，并通过依赖检查。
3. 在机器人/控制主机确认 `robot_config.yaml`、手柄和 ROS2 环境；没有配置文件时不要启动实体硬件。
4. 在 MVN Analyze 配置 **UDP 9763、60 Hz、MXTP02/23 segments**，先只做 UDP 数据验收。
5. 先启动仿真验证 `pd_brake → normal → Xsens`，再按同样流程启动实体硬件。
6. 按 `WAITING_FOR_DATA → READY → LIVE` 三阶段操作，理解断流和新会话的处理方式。

相关入口：

- [仓库内 Xsens 详细约束](src/bxi_example_py_elf3/mods/com.bxi.sonic/XSENS_MVN.md)
- [仓库内 SONIC/ELF3 部署说明](src/bxi_example_py_elf3/mods/com.bxi.sonic/SONIC_ELF3.md)
- [仓库内 Xsens Mod 清单](src/bxi_example_py_elf3/mods/com.bxi.sonic/mod.yaml)
- [交付仓库 xsens-sonic-teleop-ros2](https://github.com/jiangmingxuan234-alt/xsens-sonic-teleop-ros2)

## Xsens 0. 角色、地址和已验证边界

本流程涉及三类设备。README 不写入现场机器人 IP，请在交接表或现场终端中填写：

```text
ROBOT_IP=                         # 机器人实际 IPv4；留给现场填写
ROBOT_USER=                       # 机器人登录用户名；留给现场填写
ROS2_HOST_IP=                     # 运行 xsens_source/ROS2 控制程序的 Linux 主机 IPv4
MVN_PC_IP=                        # 运行 MVN Analyze 的电脑 IPv4（跨电脑时才需要）
```

首版已经验证的路径是 **MVN Analyze 与 `xsens_source` 在同一台电脑**：

```text
MVN Analyze
  └─ UDP 127.0.0.1:9763 (MXTP02)
       └─ xsens_source（进入 sonic_xsens 后创建，50 Hz 状态/姿态）
            └─ ZMQ 127.0.0.1:5559
                 └─ xsens_bridge
                      └─ ZMQ 127.0.0.1:5557
                           └─ SONIC policy → BXI 控制器 → 机器人
```

| 项目 | 当前实现合同 |
| --- | --- |
| 传输协议 | UDP；代码使用 `AF_INET/SOCK_DGRAM`，没有 TCP 接收器 |
| MVN 目标（同机） | `127.0.0.1:9763` |
| MVN 输入流速率 | 60 Hz；软件若只有 40/60，选择 60 |
| 数据项 | Position + Orientation（Quaternion） |
| Actor | 一个 FullBody actor，23 个 body segments，无 props、无 fingers |
| 数据报 | `MXTP02`，760 bytes，大端字段，四元数按 `WXYZ` 解析 |
| source → bridge | `127.0.0.1:5559`，pose/status 使用 ZMQ |
| bridge → policy | `127.0.0.1:5557`，SMPL reference 使用 ZMQ |
| 默认来源白名单 | `allowed_sender: 127.0.0.1` |
| 状态/策略频率 | source 状态 50 Hz、pose 发布 50 Hz；MVN 输入 60 Hz |

跨电脑发送不是默认验收路径。MVN 在另一台电脑时，必须同时修改 `allowed_sender`、把
目标地址改成 `<ROS2_HOST_IP>`、放行 UDP 9763、重新构建并重新抓包验收；不能只修改
MVN 地址后直接启动实体机器人。

## Xsens 1. 获取并核对交付代码

推荐把交付仓库完整克隆到 ROS2 主机的普通用户目录。不要把它直接克隆到 `/opt/bxi`
或覆盖厂商已经部署的工作区：

```bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/jiangmingxuan234-alt/xsens-sonic-teleop-ros2.git
cd xsens-sonic-teleop-ros2
git checkout main
```

如果目录已经存在，先查看本地改动，再按现场流程执行 `git pull --ff-only`；不要用
`git reset --hard` 覆盖别人的现场修改。交付主分支必须至少包含 Xsens 代码基线
`abed84d`（该提交修复了 Xsens 启动状态合同），并且必须能找到以下文件：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
MOD="$REPO/src/bxi_example_py_elf3/mods/com.bxi.sonic"

cd "$REPO"
test -f "$MOD/mod.yaml"
test -f "$MOD/xsens/source_node.py"
test -f "$MOD/xsens/protocol.py"
grep -q 'sonic_xsens' "$MOD/mod.yaml"
git show --no-patch --oneline HEAD
git merge-base --is-ancestor abed84d HEAD
echo 'XSENS_CODE_LAYOUT=PASS'
```

若 `git merge-base` 因浅克隆提示找不到对象，先执行 `git fetch --unshallow`，再重试。
如果代码基线或文件检查失败，先停止部署并从交付仓库确认分支，不要拿旧版
`com.bxi.sonic` 单独目录拼接到新代码上。

已经有 BXI 工作区时，可以把 `REPO` 设置为该工作区根目录并跳过 clone；但要确认
`src/bxi_example_py_elf3/mods/com.bxi.sonic` 是本次交付版本，且不要用 `git add .`
把工作区其他实验文件一并提交。

本页后续代码块默认使用 `~/projects/xsens-sonic-teleop-ros2`。如果现场使用其他目录，
请把每个代码块第一行的 `REPO=...` 改成实际绝对路径；也可以在当前终端先执行
`export REPO=/absolute/path/to/xsens-sonic-teleop-ros2`，但新开的终端仍需重新设置。

## Xsens 2. ROS2 主机一次性部署

以下命令在真正运行 `bxi_example_py_elf3_demo` 的 Linux 主机执行。该主机可以是机器人
本机，也可以是外部控制电脑；如果是外部控制电脑，它必须能访问机器人硬件和手柄。

### 2.1 ROS2 与 BXI 二进制依赖

先检查系统已有的 ROS2 和 BXI setup：

```bash
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "找不到 /opt/ros/humble/setup.bash；请先安装或加载 ROS2 Humble" >&2
  exit 1
fi

BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
if [ ! -f "$BXI_SETUP" ]; then
  echo "找不到 bxi_ros2_pkg/setup.bash；请先安装 BXI ROS2 二进制包" >&2
  exit 1
fi
echo "BXI_SETUP=$BXI_SETUP"
```

如果 BXI 二进制包尚未安装，在有网络的 Ubuntu 22.04/ROS2 Humble 主机执行：

```bash
sudo install -d -o "$USER" -g "$(id -gn)" /opt/bxi
git clone https://github.com/bxirobotics/bxi_ros2_pkg.git /opt/bxi/bxi_ros2_pkg
```

如果 `/opt/bxi/bxi_ros2_pkg` 已存在，不要重复 clone；先确认其中有 `setup.bash`。官方
包说明见 [bxirobotics/bxi_ros2_pkg](https://github.com/bxirobotics/bxi_ros2_pkg)。

### 2.2 SONIC/PICO 依赖检查

先只检查，不修改系统：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
test -f "$REPO/build.sh"
cd "$REPO/src/bxi_example_py_elf3/mods/com.bxi.sonic"
chmod +x ./deploy_dependencies.sh
./deploy_dependencies.sh --check
```

看到 `SONIC dependencies are ready.` 才继续。若仅缺少通用 Python 包，可让脚本创建
Mod 内独立运行时；这不会修改 systemd，也不会安装 Torch/CUDA：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
cd "$REPO/src/bxi_example_py_elf3/mods/com.bxi.sonic"
./deploy_dependencies.sh --mod-runtime
./deploy_dependencies.sh --check
```

离线部署需准备 wheelhouse：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
cd "$REPO/src/bxi_example_py_elf3/mods/com.bxi.sonic"
./deploy_dependencies.sh --mod-runtime --offline \
  --wheelhouse /path/to/wheelhouse
```

脚本会检查 `numpy`、`scipy`、`zmq`、`msgpack`、`pinocchio`、`xrobotoolkit_sdk`、
`RoboticsServiceProcess` 和动态库闭包。激活码、个人虚拟环境和原始 MVN 录制文件不要
放到仓库；如果 MVN Analyze 需要登录/激活，单独在 MVN 电脑完成。

### 2.3 编译并确认 Mod 已安装

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi

test -f "$REPO/build.sh"
test -f "$BXI_SETUP"
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"

cd "$REPO"
bash ./build.sh
source install/setup.bash

ros2 pkg prefix bxi_example_py_elf3
ros2 pkg prefix remote_controller
INSTALL_MOD="$(ros2 pkg prefix bxi_example_py_elf3)/share/bxi_example_py_elf3/mods/com.bxi.sonic"
test -f "$INSTALL_MOD/mod.yaml"
test -f "$INSTALL_MOD/xsens/source_node.py"
echo "INSTALL_MOD=$INSTALL_MOD"
```

两条 `ros2 pkg prefix` 都应成功；`INSTALL_MOD` 必须指向本次工作区的 `install/`，不能
是旧工作区。每次修改 `mod.yaml` 或依赖后都要重新 `bash ./build.sh` 并重新
`source install/setup.bash`。

## Xsens 3. 机器人/控制端准备

如果 ROS2 主机就是机器人本机，可直接执行本节；如果控制程序在外部电脑，则在外部电脑
执行 ROS2 命令，在需要登录机器人时使用（把占位符替换后再执行）：

```bash
ROBOT_USER=""
ROBOT_IP=""
test -n "$ROBOT_USER" && test -n "$ROBOT_IP" && \
  ssh "$ROBOT_USER@$ROBOT_IP"
```

机器人 IP 有意不写入本文。先把上面两个空字符串填为现场值；若未填写，`ssh` 不会执行。
登录或打开控制主机后记录地址和路由：

```bash
hostname -s
ip -4 -br address
ip route
```

### 3.1 检查现场机器人配置（正式上机必须通过）

硬件 launch 的参数名是 `robot_config_file`，不是 `robot_ip`。默认读取
`/opt/bxi/robot_config.yaml`；该文件应由现场的 `bxi_robot_config_tool` 或厂商交付流程
生成。下面的检查在一个子 shell 中执行，失败会真正以非零状态停止，不会继续到硬件启动：

```bash
bash -eu <<'BASH'
CONFIG=/opt/bxi/robot_config.yaml
if [ ! -r "$CONFIG" ] || [ ! -s "$CONFIG" ]; then
  echo "缺少或不可读：$CONFIG" >&2
  echo "停止：不要继续启动实体机器人硬件 launch；请先用现场配置工具生成它。" >&2
  exit 1
fi
echo "ROBOT_CONFIG=PASS ($CONFIG)"
BASH
```

不要因为程序打印了 “using built-in defaults” 就继续上机；那只是代码的兼容回退，不是
现场验收。若配置文件在其他位置，可在确认内容后显式传入：

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py \
  robot_config_file:=/absolute/path/to/robot_config.yaml
```

### 3.2 手柄和输入映射

BattleDragon 手柄只需在运行 `remote_controller` 的主机配置一次：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
test -f "$REPO/script/bxi-battle-dragon-link"
test -f "$REPO/script/bxi-dev.rules"
cd "$REPO"
sudo install -m 0755 ./script/bxi-battle-dragon-link \
  /usr/local/bin/bxi-battle-dragon-link
sudo cp ./script/bxi-dev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔手柄后确认：

```bash
ls -l /dev/input/jsBattleDragon
```

当前默认 `xbox_default.yaml` 的 Xsens 组合键合同为：

| 输入 | 原始输入 | 门槛/输出 |
| --- | --- | --- |
| LT | `js.axis.5` | `>=0.85` |
| RT | `js.axis.4` | `>=0.85` |
| Y | `js.button.4` | 按下 |
| 保护条件 | `js.button.6/7`（LB/RB）必须松开 | 输出 `btn_10=11` |

因此操作说明中的 `LT + RT + Y` 是同一时刻按住三个输入，松开 LB/RB；CRSF 接收机的
对应合同是 CH7（LT）、CH3（RT）和 CH8 的 Y 编码。不要自行把轴号改成“看起来相近”的
编号，若现场硬件不同，先用输入诊断和 `remote_controller` 配置确认。

## Xsens 4. MVN Analyze 配置（动捕电脑）

先在 MVN Analyze 内确认设备、Actor 和追踪画面正常，再打开 Network Streamer/Network
Output。菜单名称会随 MVN 版本略有变化，官方入口：

- [Network Streamer in MVN](https://base.xsens.com/s/article/Network-Streamer-in-MVN-1611927767465?language=en_US)
- [MVN Analyze Software Overview](https://base.xsens.com/s/article/MVN-Analyze-Software-Overview?language=en_US)
- [Movella/Xsens 支持中心](https://base.xsens.com/)

### 4.0 授权、版本和动捕电脑准备

1. 在动捕电脑单独完成 MVN Analyze 登录/授权/激活；不要把激活码、账号令牌或授权文件
   复制到 ROS2 主机或提交到 Git。不同版本通常在启动提示、`Help/About` 或
   `License/Activation` 入口显示授权状态，若找不到入口，按上面的支持中心链接检索当前
   版本的授权说明。
2. 记录 MVN Analyze 版本、固件版本和授权到期日，交接时一并写入现场交接单；授权过期时
   先在 MVN 电脑更新，不要为了绕过授权修改网络或 parser。
3. 关闭会占用 UDP 9763 的旧探针/旧控制程序；同一台 ROS2 主机上，验收探针和
   `xsens_source` 不能同时监听该端口。

同机（推荐首验）逐项设置：

1. 连接动捕设备，加载一个 `FullBody` actor，确保人体骨架在 MVN 中稳定显示。
2. 打开 Network Streamer/Network Output，协议选 **UDP**（本代码不实现 TCP）。
3. 目标地址填 `127.0.0.1`，目标端口填 `9763`。
4. 流速率选 **60 Hz**；如果界面只有 40 和 60，选 60，不要选 40。
5. 数据项选择 **Position + Orientation (Quaternion)**。
6. 只发送一个 FullBody actor 的 **23 个 segments**；关闭 props 和 fingers。
7. 先保持 Live 或 Playback 正常播放，确认输出开关已启用。

本流程不要求额外的 T-pose heading 标定。使用已经确认的图示/中立姿势和机器人进入
SONIC 前的初始零位作为入场参考；Heading/Origin 对全局坐标的具体影响仍应以当前 MVN
版本和厂商答复为准。第一次动作始终从很小幅度开始。

### 4.1 跨电脑发送（首版未验收）

只有在同机路径已通过后才做跨电脑。MVN 电脑目标改为 `<ROS2_HOST_IP>:9763`；ROS2
主机的 Mod 配置必须把 `allowed_sender` 从 `127.0.0.1` 改为 MVN 电脑的固定地址：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
test -f "$REPO/build.sh"
cd "$REPO"
MOD="$REPO/src/bxi_example_py_elf3/mods/com.bxi.sonic"
grep -n 'allowed_sender' "$MOD/mod.yaml"
```

编辑 `mod.yaml` 中 `xsens_source.params.allowed_sender` 这一行（只改 IP，不改缩进），
例如最终应类似：

```yaml
      allowed_sender: <MVN_PC_IP>
```

然后重新构建并放行最小范围的 UDP：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
MVN_PC_IP=""  # 填写运行 MVN Analyze 的电脑 IPv4
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi

test -f "$REPO/build.sh"
test -f "$BXI_SETUP"
test -n "$MVN_PC_IP"
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"
cd "$REPO"
bash ./build.sh
source install/setup.bash

if command -v ufw >/dev/null 2>&1 && sudo ufw status | grep -q 'Status: active'; then
  sudo ufw allow from "$MVN_PC_IP" to any port 9763 proto udp
fi
```

先填写 `MVN_PC_IP` 再执行；变量为空时 `test -n` 会失败，此时不要继续构建或改防火墙。
跨电脑必须重新做下面的探针和抓包验收，且第一次仍在仿真或空载条件完成。

## Xsens 5. UDP 数据验收（启动 ROS2 状态前）

`xsens_source` 只有进入 `sonic_xsens` 状态后才会绑定 9763。数据探针必须在启动硬件
控制器或进入 Xsens 状态**之前**运行；探针结束后再启动控制器。9763 同一时刻只能有
一个监听者，不要把探针和 `xsens_source` 同时运行。

在 ROS2 主机执行下面整段代码。只复制代码块，不要复制终端提示符或上一次命令的输出：

```bash
python3 - <<'PY'
import socket
import sys

HOST = "0.0.0.0"
PORT = 9763
EXPECTED_BYTES = 760
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.settimeout(3.0)
bad = 0
received = 0
print(f"Listening on UDP {PORT}; waiting for 10 datagrams...")
try:
    while received < 10:
        data, sender = sock.recvfrom(2048)
        received += 1
        header = data[:6]
        ok = len(data) == EXPECTED_BYTES and header == b"MXTP02"
        if not ok:
            bad += 1
        print(
            received,
            "sender=", sender,
            "bytes=", len(data),
            "header=", repr(header),
            "hex=", data[:24].hex(" "),
        )
except socket.timeout:
    print("ERROR: 3 seconds passed without the next UDP datagram", file=sys.stderr)
    sys.exit(1)
finally:
    sock.close()

if bad:
    print("ERROR: packet size/header does not match MXTP02/760-byte contract", file=sys.stderr)
    sys.exit(1)
print("UDP_MXTP02_PROBE=PASS")
PY
```

通过标准是连续收到 10 个来自预期发送端的 `bytes=760`、`header=b'MXTP02'` 数据报。
你之前看到的 `('127.0.0.1', 动态端口)`、760 bytes、`MXTP02` 就是正确的同机现象；
`TimeoutError: 未找到命令` 或 `bash: unexpected token` 通常表示把程序输出、终端提示符或
网页转义字符一起粘贴回 shell，并不是 Xsens 协议错误。超时只应在探针代码中显示为
“没有收到 UDP”，不要把 `TimeoutError:` 单独当作命令执行。

探针失败时可另开终端抓包（抓包不占用 9763）：

```bash
sudo tcpdump -ni any -c 20 'udp port 9763'
```

同机应看到 `lo`/`127.0.0.1`；跨电脑应看到 `<MVN_PC_IP>`。若探针报
`Address already in use`，先退出其他探针或已经进入 Xsens 状态的控制程序，再重试。

## Xsens 6. 启动终端布局与完整操作流程

### 6.1 先做仿真验收

打开三个终端。每个终端都重新 source ROS2 和工作区；不要把一个终端的日志文本复制到
另一个终端执行。

终端 A（仿真控制器）：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
test -f "$BXI_SETUP"
test -f "$REPO/install/setup.bash"
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"
cd "$REPO"
source install/setup.bash
ros2 launch bxi_example_py_elf3 example_demo.launch.py
```

终端 B（遥控器）：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
test -f "$BXI_SETUP"
test -f "$REPO/install/setup.bash"
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"
cd "$REPO"
source install/setup.bash
ros2 launch remote_controller remote_controller.launch.py
```

终端 C（状态监控）：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
test -f "$BXI_SETUP"
test -f "$REPO/install/setup.bash"
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"
cd "$REPO"
source install/setup.bash
ros2 topic echo /simulation/state_machine_info std_msgs/msg/String
```

仿真稳定后再换成实体硬件 launch。硬件启动前先完成第 3.1 节的配置文件检查：

终端 A（实体机器人/控制主机）：

```bash
REPO="${REPO:-$HOME/projects/xsens-sonic-teleop-ros2}"
BXI_SETUP=/opt/bxi/bxi_ros2_pkg/setup.bash
if [ ! -f "$BXI_SETUP" ]; then
  BXI_SETUP=/opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
test -f "$BXI_SETUP"
test -f "$REPO/install/setup.bash"
test -s /opt/bxi/robot_config.yaml
source /opt/ros/humble/setup.bash
source "$BXI_SETUP"
cd "$REPO"
source install/setup.bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py \
  robot_config_file:=/opt/bxi/robot_config.yaml
```

终端 B 仍运行 `remote_controller.launch.py`；终端 C 把监控话题换成：

```bash
ros2 topic echo /hardware/state_machine_info std_msgs/msg/String
```

如果现场只允许键盘调试，可另开键盘驱动：

```bash
ros2 launch remote_controller remote_controller_keyboard.launch.py
```

但正式 Xsens 操作以已确认的 `LT + RT + Y` 手柄组合为准，不要依赖键盘自动重复。

### 6.2 `pd_brake → normal → READY → LIVE`

1. **启动和安全检查**：机器人可靠支撑、周围无人、手柄所有按键和 trigger 松开，物理
   急停旁有人。启动后按现场已经验证的控制方式先进入 `pd_brake`，再进入 `normal`；
   不要从 `zero_torque` 或故障态直接请求 Xsens。
2. **第一次 `LT + RT + Y`**：三键同时按下，输出 `btn_10=11`，从 `normal` 进入
   `sonic_xsens`。立即松开所有按键。这一次只启动 Xsens source/bridge 和三阶段状态机，
   **不是**让人体动作立刻接管。
3. **等待 READY**：保持图示/中立姿势，不要按键。source 先收集 10 帧连续窗口，再收集
   30 帧稳定性证据；代码的宽松安全检查是 pelvis 跨度不超过 0.15 m、segments 的
   95 分位角偏差不超过 20°。日志应出现：

   ```text
   Xsens READY — release controls, then press LT+RT+Y to request LIVE
   ```

4. **第二次 `LT + RT + Y`**：先确认 `btn_10` 已回到**精确的 0**（完全松开），看到
   READY 后再按一次组合键并立即松开。代码发送带 source epoch 的 arm 命令，等待状态回执
   和同一帧窗口的 exact join；不要在这一步连续快速连按。
5. **确认 LIVE**：等待日志出现 `phase=LIVE link=FRESH` 和
   `reference status: live_reference`，再从很小幅度动作开始。进入 LIVE 后有 0.4 秒
   smoothstep 平滑，不要用突然的大动作测试。
6. **结束**：先回到中立，小幅停止动作，再请求 `normal`；需要停机时按现场流程进入
   `pd_brake`、`recover` 或 `zero_torque`。异常时先按实体急停。

### 6.3 为什么要两次组合键

这是当前代码的单状态、三阶段合同：

```text
sonic_xsens / WAITING_FOR_DATA
        │  首次 LT+RT+Y（进入状态，仍使用 idle reference）
        ▼
READY（数据、窗口、稳定性通过；等待人工确认）
        │  松开到 btn_10=0，再次 LT+RT+Y（发送 arm proof）
        ▼
LIVE（授权 live reference；0.4 s 平滑切入）
```

第一次按键不能跳过数据检查；第二次按键也不能在旧的按键电平未释放时触发。日志中的
`enable rejected — observe exact btn_10=0` 就表示需要先完全松开手柄。

## Xsens 7. idle/live、平滑和断流语义

### 7.1 idle reference

进入 `sonic_xsens` 后，在 READY 前 policy 使用 Mod 内
`assets/stream_reference.npz` 的固定 idle window，`idle_frame_start=3509`。这让机器人
不会因为等待 MVN 或等待人工确认而直接使用未初始化的 live 姿态；READY 本身也不会把
未经授权的 live 数据送进 policy。

### 7.2 live reference 与 0.4 秒平滑

第二次组合键被 source 回执确认、且 pose/status 在同一 source epoch 和连续帧窗口内 join
成功后，live gate 才打开。policy 从当前 target 关节位置到 live reference 使用
`source_blend_seconds=0.4` 的 smoothstep 混合，完成后才清理 pending yaw。不要把 0.4 秒
理解成 UDP 缓冲时长；它是控制目标切换的平滑时长。

### 7.3 同 epoch 短暂断流

当前实现是“安全保持 + 自动恢复”：

- source/bridge 本地状态心跳超过 `status_timeout_s=0.2 s` 会使状态进入 HOLD；原始 MVN
  producer 数据超过 `stale_seconds=0.5 s` 也会进入 HOLD。`0.2 s` 是本地 heartbeat
  门槛，不是要求每个 UDP 包严格 0.2 秒到达。
- policy 关闭 live gate，并保留最后一个**已授权的 live reference** 作为 HOLD 输入；这
  不是把最后一帧 UDP 无限当作新鲜数据，也不是立即把机器人切到未授权的旧会话。
- 同一 source epoch 恢复、连续 join 窗口达到 10 帧后，代码自动回到 `link=FRESH`，不需要
  再按 `LT+RT+Y`；活动 yaw 保持不变。恢复期间不要按键抢占流程。

### 7.4 新 epoch/重启

当 source 识别到新的 epoch（例如 sample counter/time code 回退并通过候选验证）时，状态
进入 `HOLD_REARM_REQUIRED`，先完成 disarm barrier，再等待新的 30 帧 READY。看到：

```text
Xsens new session READY — release controls, then press LT+RT+Y to re-arm
```

此时必须确认 `btn_10=0`，再人工按**一次** `LT+RT+Y`；不会自动替新会话开 LIVE。若 MVN
重启后 sender、sample counter 和可用 time code 恰好都继续递增，MXTP02 本身无法可靠证明
“重启”，需要向厂商确认 session/take/restart 信号，不要擅自改 epoch 判定。

## Xsens 8. 停止、重启和端口检查

正常停止顺序：先松开手柄并回到 `normal`，再在遥控器终端按 `Ctrl-C`，最后在硬件控制器
终端按 `Ctrl-C`。确认进程和端口已释放后才能再次启动：

```bash
pgrep -af 'bxi_example_py_elf3_demo|remote_controller|xsens_source' || true
ss -lunp | grep ':9763' || true
```

如果 `9763` 仍被占用，先回到对应终端进行正常退出并重新检查；不要直接 `kill -9`。软件
停止不能替代实体急停。

## Xsens 9. 常见故障排查

| 现象 | 先检查 | 处理 |
| --- | --- | --- |
| `Package not found` | 是否 source 了 Humble、BXI setup、`install/setup.bash` | 按第 2.3 节顺序重新 source；确认 `ros2 pkg prefix` 指向当前工作区 |
| 探针 3 秒无包 | MVN Stream Output 是否开启、目标地址/9763、Windows 防火墙、网卡 | 先同机 `127.0.0.1`；再用 tcpdump 看是否有包 |
| 包不是 760 bytes 或不是 `MXTP02` | MVN 协议/数据项/props/fingers 设置 | 按第 4 节重选 UDP、Quaternion、FullBody 23 segments；不要修改 parser 猜格式 |
| `unexpected sender` / source summary 收到 0 个 | `allowed_sender` 与实际发送源 IP 不同 | 同机恢复 `127.0.0.1`；跨电脑按第 4.1 节改配置、重建、放行防火墙 |
| `Address already in use` | 探针或旧 `xsens_source` 仍占用 9763 | 退出旧进程，确认 `ss -lunp` 为空，再只启动一个监听者 |
| 一直 `COLLECTING_WINDOW` | 连续窗口不足或包计数不前进 | 保持 MVN Live/Playback 连续输出；确认没有多个发送端/seek |
| `PELVIS_UNSTABLE` / `SEGMENT_UNSTABLE` | 人体仍在走动、Actor/segments 配置不符 | 保持已确认中立姿势，确认单 FullBody/23 segments；不要为了“通过”而放宽代码阈值 |
| READY 后按键无效 | `btn_10` 没有先回到精确 0、按键连发或 LB/RB 同时按住 | 松开全部输入，看到 READY，再只按一次 LT+RT+Y |
| `phase=LIVE link=HOLD` | heartbeat 或 producer 超过门槛 | 同 epoch 不按键，等待 10 帧重新 join；检查网络间隔和 source summary |
| `HOLD_REARM_REQUIRED` | 检测到新 epoch/会话 | 等新的 READY 提示，精确 0 后手动按一次组合键 |
| 方向/朝向不对 | Heading/Origin、四元数顺序、坐标系或 yaw bias | 先停机并记录日志，使用允许的 alignment reset；向厂商确认，不要盲改轴映射 |
| 硬件启动打印 built-in defaults | `/opt/bxi/robot_config.yaml` 缺失或不可读 | 停止实体 launch，先由现场工具生成并通过第 3.1 节检查 |

## Xsens 10. 交付验收清单

在交接单中逐项勾选并保留日志：

- [ ] 交付仓库包含 `abed84d` 之后的 Xsens 代码，`mod.yaml`、`xsens/`、测试文件齐全。
- [ ] ROS2 Humble、BXI setup、Mod 依赖检查通过；`ros2 pkg prefix` 指向当前 install。
- [ ] MVN 已激活，单 FullBody actor、23 segments、Position + Quaternion、UDP 60 Hz。
- [ ] 同机 UDP 探针连续收到 10 个 `MXTP02`/760-byte 数据报；跨电脑额外完成来源白名单和抓包。
- [ ] 仿真完整走过 `pd_brake → normal → 首次组合键 → READY → 第二次组合键 → LIVE`。
- [ ] 实体硬件启动前通过 `robot_config.yaml` 检查，急停和安全员就位，先小幅动作。
- [ ] 日志能证明 `reference status: live_reference`，且切入平滑约 0.4 秒。
- [ ] 人为短暂断流时进入 HOLD，并在同 epoch 10 帧 join 后自动恢复；新 epoch 必须重新手动触发。
- [ ] 正常退出后 9763 端口释放，未遗留 `xsens_source`/控制器进程。
- [ ] 激活码、机器人真实 IP、个人路径、原始录制文件未写入公开仓库。

## Xsens 11. 需要向厂商/官网确认的关键问题

如果现场版本与本文合同冲突，先停止实体动作并记录版本/抓包，再向 Movella/Xsens 或
设备厂商确认：

1. 当前 MVN 版本的 MXTP02 sample counter、datagram counter、time code、wrap 和 playback seek 语义。
2. 是否有稳定的 session/take ID 或外部 restart 信号；仅靠 sender、counter、time code 可能无法识别所有重启。
3. 23 个 segment 的顺序、坐标系和 segment-frame 定义；用一帧 identity MVNX 对照验证。
4. 四元数是否可能出现 `q/-q` 代表切换；当前代码按 WXYZ、米制解释。
5. Heading/Origin 设置对全局坐标和初始朝向的影响。
6. BattleDragon 的 LT/RT/Y 轴号和阈值，以及 CRSF CH7/CH3/CH8-Y 的实际发送值。

不要在没有厂商证据时加入未文档化的轴交换、四元数重排或自动重启猜测。

---

## ZeroLab F2 Pro 原有部署文档

以下内容是仓库原有的 ZeroLab F2 Pro 文档，编号从 `0` 重新开始；它不属于上面的
Xsens MVN 交接步骤。需要接手 Xsens 时，完成 `Xsens 0`–`Xsens 11` 后即可按验收清单收尾。

官方 SONIC 遥操背景说明：

- [ELF3 SONIC 开发文档](https://wiki.bxirobotics.cn/elf3/developer/sonic/)
- [BXI controller ROS 2](https://github.com/bxirobotics/bxi_controller_ros2)
- [com.bxi.sonic 上游仓库](https://github.com/konodoki/com.bxi.sonic)

## 0. 先看清楚服务策略

安装后应保持：

```text
zerolab-network.service: active + enabled
zerolab-hardware.service: inactive + disabled
```

这表示网络接收配置可以随系统启动，但真正的硬件控制栈不会开机自动运行，必须由操作者在机器人可靠支撑、安全员守在物理急停旁时手动启动。

原有 `ros_elf_launch.service`、遥控器按键和基础控制逻辑不因本 Mod 改变。ZeroLab 与 PICO 也不能同时作为实时 SONIC 输入源；切换前必须先退出当前状态。

## 1. 现场需要填写的两个 IP

在整套流程开始前记录：

```text
ROBOT_IP=<机器人实际 IPv4 地址>
SENDER_IP=<运行 ZeroLab/MotionCaptureMaster 的电脑实际 IPv4 地址>
```

不要照抄示例地址。机器人可以使用 Wi-Fi 或网桥以太网地址；发送端地址也可以是客户现场电脑的地址。

机器人端检查：

```bash
hostname -s
ip -4 -br address
ip route
```

发送端电脑检查：

```bash
hostname -I
ip -4 -br address
```

## 2. 机器人端安装基础依赖

以下命令在机器人上执行。基础工程路径按官方 SONIC 文档约定使用 `/home/bxi/bxi_ws/bxi_rl_controller_ros2_example`。新机器人已经预装该基础工程时，不要再次 clone 基础工程。

```bash
sudo apt update
sudo apt install -y \
  git build-essential python3-pip python3-venv python3-yaml \
  network-manager iproute2 tcpdump openssh-server

test -f /opt/ros/humble/setup.bash
test -f /opt/bxi/bxi_ros2_pkg/setup.bash

BASE=/home/bxi/bxi_ws/bxi_rl_controller_ros2_example
test -d "$BASE"
```

只有在基础工程确实不存在时，才执行：

```bash
mkdir -p /home/bxi/bxi_ws
git clone https://github.com/bxirobotics/bxi_controller_ros2.git "$BASE"
```

确认 Mod 目录：

```bash
MODS="$BASE/src/bxi_example_py_elf3/mods"
test -d "$MODS"
echo "MODS=$MODS"
```

## 3. 下载 SONIC/ZeroLab Mod

按照官方 SONIC 文档，Mod 源码必须放在基础工程的以下位置：

```text
/home/bxi/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/mods/com.bxi.sonic
```

不要把仓库克隆到 `/opt/bxi/mods` 或工作区外的临时目录；编译和安装时，框架会从这个 `src/.../mods` 目录发现 Mod。

```bash
SONIC="$MODS/com.bxi.sonic"

if test -e "$SONIC"; then
  echo "已存在：$SONIC"
else
  git clone https://github.com/jiangmingxuan234-alt/com.bxi.sonic.git "$SONIC"
fi

test -f "$SONIC/mod.yaml"
test -f "$SONIC/requirements-pico.txt"
test -d "$SONIC/zerolab"
```

如果现场使用公司总仓库，把 clone 地址换成：

```bash
git clone https://github.com/konodoki/com.bxi.sonic.git "$SONIC"
```

## 4. 编译并安装 Mod

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash

cd "$BASE"
colcon build --packages-select bxi_example_py_elf3 --symlink-install
source "$BASE/install/setup.bash"

INSTALL_MODULE="$BASE/install/bxi_example_py_elf3/share/bxi_example_py_elf3/mods/com.bxi.sonic"
test -f "$INSTALL_MODULE/mod.yaml"
echo "INSTALL_MODULE=$INSTALL_MODULE"
```

编译完成后，运行时使用的是安装目录中的 Mod：

```text
/home/bxi/bxi_ws/bxi_rl_controller_ros2_example/install/bxi_example_py_elf3/share/bxi_example_py_elf3/mods/com.bxi.sonic
```

后续 `requirements-pico.txt`、`deploy/`、`vendor/` 和 `.runtime/` 均以这个安装目录为准。

## 5. 准备 PICO/SONIC Python 环境

不要只把依赖装进系统 Python；SONIC manager 应使用 Mod 内的隔离环境。

```bash
RUNTIME="$INSTALL_MODULE/.runtime/linux-x86_64/pico"
PYTHON="$RUNTIME/bin/python"

python3 -m venv "$RUNTIME"
"$PYTHON" -m pip install --upgrade pip
"$PYTHON" -m pip install -r "$INSTALL_MODULE/requirements-pico.txt"
```

验证：

```bash
"$PYTHON" - <<'PY'
import msgpack, numpy, scipy, zmq, pinocchio, xrobotoolkit_sdk
print("SONIC_PICO_IMPORTS=PASS")
PY
```

如果 `xrobotoolkit_sdk` 没有安装包，检查本 Mod 是否包含当前架构的 bundled binding：

```bash
find "$INSTALL_MODULE/vendor/python" -name 'xrobotoolkit_sdk*.so' -print
```

PICO 头显侧还必须运行与 XRoboToolkit PC Service 配套的数据发送程序，并启用 body tracking。机器人端的 `RoboticsServiceProcess` 由 SONIC manager 在进入 PICO 状态时自动启动和退出，不需要客户手动启动第二个 systemd 服务。

## 6. ZeroLab 发送端配置

在 ZeroLab/MotionCaptureMaster 所在电脑上：

1. 与机器人处于可互通的同一网络；
2. Stream Output 协议选择 UDP；
3. 目标地址填写 `ROBOT_IP`；
4. 目标端口填写 `18000`；
5. 频率填写 `50 Hz`；
6. 数据选择固定 `992-byte full-body packet`；
7. 不添加额外 header、prefix 或 suffix；
8. 开启 body tracking 并点击 Start/Enable Stream Output。

发送端防火墙允许出站 UDP `18000`。机器人端若启用 UFW：

```bash
sudo ufw allow 18000/udp
```

## 7. 三种现场网络模式

### 7.1 无网桥：Wi-Fi direct

适用于机器人和发送端都通过 Wi-Fi 接入同一局域网。机器人使用 Wi-Fi 地址作为 `ROBOT_IP`，不需要插网桥，也不需要配置 alias。

机器人端：

```bash
ip -4 -br address
ip route
```

发送端把 UDP 目标设置为机器人当前 Wi-Fi 地址。ZeroLab 配置使用：

```ini
ZEROLAB_NETWORK_MODE=direct
ZEROLAB_ALLOWED_SENDER=<SENDER_IP>
```

### 7.2 有网桥：普通以太网 direct

适用于网桥已上电、网线已连接，并且 `enp86s0` 通过 DHCP 或静态配置获得普通 IPv4 地址。

先确认网桥地址：

```bash
ip -4 -br address show enp86s0
nmcli -g GENERAL.STATE,GENERAL.CONNECTION device show enp86s0
```

只要网桥地址与发送端可互通，就继续使用 `direct`，把发送端 UDP 目标改成网桥地址：

```ini
ZEROLAB_NETWORK_MODE=direct
ZEROLAB_ALLOWED_SENDER=<SENDER_IP>
```

这种模式不要求额外 alias；服务不会改动普通以太网地址或 NetworkManager 连接。

### 7.3 有网桥：alias 模式

仅当现场网络或旧设备要求机器人在有线网卡上额外拥有一个固定 ZeroLab 地址时使用。必须先确认 `deploy/zerolab-network-config` 支持 `alias`，并填写普通以太网接口、Wi-Fi 接口和 `/32` alias 地址：

```ini
ZEROLAB_NETWORK_MODE=alias
ZEROLAB_ALIAS_IP=<固定的ZeroLab地址>
ZEROLAB_ETH=enp86s0
ZEROLAB_WIFI=wlo1
ZEROLAB_ALLOWED_SENDER=<SENDER_IP>
```

安装后验证服务配置：

```bash
sudo /usr/local/libexec/zerolab-network-config validate-sender /etc/default/zerolab-network
sudo systemctl restart zerolab-network.service
ip -4 -br address show enp86s0 wlo1
```

alias 模式只应在普通以太网已经 carrier + DHCP/静态地址正常时使用。它不应覆盖普通地址，也不应改变 NetworkManager 的连接。第一次部署必须抓包确认真实数据到达；没有抓到包不能标记为通过。

## 8. 安装 ZeroLab 网络服务

部署文件来自编译后的 Mod：

```bash
test -x "$INSTALL_MODULE/deploy/zerolab-network-config"
test -f "$INSTALL_MODULE/deploy/config/zerolab-network"
test -f "$INSTALL_MODULE/deploy/systemd/zerolab-network.service"
```

安装：

```bash
sudo install -m 0755 \
  "$INSTALL_MODULE/deploy/zerolab-network-config" \
  /usr/local/libexec/zerolab-network-config

sudo install -m 0644 \
  "$INSTALL_MODULE/deploy/config/zerolab-network" \
  /etc/default/zerolab-network

sudo install -m 0644 \
  "$INSTALL_MODULE/deploy/systemd/zerolab-network.service" \
  /etc/systemd/system/zerolab-network.service
```

写入现场配置（把占位符替换成真实值）：

```bash
sudo tee /etc/default/zerolab-network >/dev/null <<'EOF'
ZEROLAB_NETWORK_MODE=direct
ZEROLAB_ALLOWED_SENDER=<SENDER_IP>
EOF
```

然后启动网络服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now zerolab-network.service
systemctl is-active zerolab-network.service
systemctl is-enabled zerolab-network.service
```

应分别得到 `active` 和 `enabled`。

## 9. 安装硬件 unit，但保持手动启动

```bash
sudo install -m 0644 \
  "$INSTALL_MODULE/deploy/systemd/zerolab-hardware.service" \
  /etc/systemd/system/zerolab-hardware.service

sudo mkdir -p /etc/systemd/system/zerolab-hardware.service.d
sudo tee /etc/systemd/system/zerolab-hardware.service.d/20-sonic-pico-runtime.conf >/dev/null <<EOF
[Service]
Environment=SONIC_PICO_PYTHON=$PYTHON
Environment=SYSTEMD_PICO_PYTHON=$PYTHON
EOF

sudo systemctl daemon-reload
sudo systemctl disable zerolab-hardware.service
sudo systemctl stop zerolab-hardware.service
```

验收：

```bash
test "$(systemctl is-enabled zerolab-hardware.service)" = disabled
test "$(systemctl is-active zerolab-hardware.service)" = inactive
echo 'HARDWARE_MANUAL_ONLY=PASS'
```

## 10. 首次 UDP 验收

确认发送端正在向机器人发送后，在机器人端执行：

```bash
ip -4 -br address
ss -lunp | grep ':18000' || true
sudo tcpdump -ni any -c 20 'udp port 18000'
```

如果使用指定网卡，把 `any` 换成实际接口，例如 `enp86s0` 或 `wlo1`：

```bash
sudo tcpdump -ni enp86s0 -c 20 \
  'udp and src host <SENDER_IP> and dst port 18000'
```

看到 20 个来自 `<SENDER_IP>` 的 UDP 包后，才算网络接收验收通过。没有包时依次检查发送端 IP、目标机器人 IP、网卡、Windows 防火墙和机器人端口占用。

## 11. 启动 ZeroLab 实时控制

启动前必须：机器人可靠支撑、周围无人、安全员在物理急停旁、手柄完全松开；发现异常立即使用物理急停。

```bash
sudo systemctl start zerolab-hardware.service
systemctl status zerolab-hardware.service --no-pager
journalctl -u zerolab-hardware.service -f
```

在遥控器/框架中按原有映射进入状态：

```text
A（btn_10=11）：进入 ZeroLab WAIT_ARM 预 ARM 状态
Y（btn_10=12）：ARM，允许人体动作接管
再次按 Y：暂停并回到 WAIT_ARM/Normal 输出
```

首次进入后先保持中立姿势，等待 10 帧有效数据窗口和 `WAIT_ARM`；只做小幅、缓慢动作。结束时回到中立，再按 Y 暂停，随后切回 `normal`。不要直接从 ZeroLab 实时状态切进 PICO；必须先回到 `normal`，再进入原 SONIC/PICO 状态。

## 12. PICO 与 ZeroLab 切换

PICO 和 ZeroLab 不能同时占用同一个实时输入路径。安全切换顺序：

```text
ZeroLab ARMED -> Y 暂停 -> Normal -> 原 SONIC/PICO
原 SONIC/PICO -> 退出 SONIC -> Normal -> ZeroLab A/WAIT_ARM
```

进入 PICO 前确认 PICO 端已校准、body tracking 可用；进入 ZeroLab 前确认 UDP 持续到达且 `zerolab_source` 日志显示有效帧。

## 13. 停止流程

先回到 `pd_brake` 或由现场安全流程确认可停，再停止硬件服务：

```bash
sudo systemctl stop zerolab-hardware.service

for i in $(seq 1 30); do
  if ! pgrep -af \
    '[h]ardware_elf3|[b]xi_example_py_elf3_demo|[z]erolab_source|[R]oboticsServiceProcess' \
    >/dev/null; then
    echo 'ALL_CONTROLLERS_STOPPED=PASS'
    break
  fi
  sleep 1
done

systemctl is-active zerolab-hardware.service
```

网络服务可以继续运行；如果要完全卸载 ZeroLab 网络配置：

```bash
sudo systemctl disable --now zerolab-network.service
sudo rm -f /etc/systemd/system/zerolab-network.service
sudo rm -f /usr/local/libexec/zerolab-network-config
sudo rm -f /etc/default/zerolab-network
sudo systemctl daemon-reload
```

卸载前请先备份 `/etc/default/zerolab-network`，并确认没有实时控制进程。

## 14. 常见问题

### PICO manager 报 code 78

使用实际运行时验证依赖，不要只验证系统 Python：

```bash
"$PYTHON" -c 'import msgpack,numpy,scipy,zmq,pinocchio,xrobotoolkit_sdk; print("PASS")'
```

### `TCP failed`

先确认 PICO PC Service、头显端发送程序和 body tracking，再确认机器人端 PICO manager 选择的 Python 与 SDK。ZeroLab UDP 正常不代表 PICO TCP 已连接。

### `0 packets captured`

检查发送端是否真正开启 Stream Output，目标地址是否为机器人当前 IP，端口是否为 `18000`，以及是否抓错网卡。不要把 0 包结果标记为通过。

### 18000 端口被占用

```bash
ss -lunp | grep ':18000'
```

record-only 录制器和实时 `sonic_zerolab` 不能同时运行；先停止其中一个并确认端口释放。

## 15. 最终验收命令

```bash
echo '=== SONIC/ZEROLAB FINAL CHECK ==='
printf 'network active: '; systemctl is-active zerolab-network.service
printf 'network enabled: '; systemctl is-enabled zerolab-network.service
printf 'hardware active: '; systemctl is-active zerolab-hardware.service
printf 'hardware enabled: '; systemctl is-enabled zerolab-hardware.service
printf 'addresses: '; ip -4 -br address
printf 'sender policy: '; sudo cat /etc/default/zerolab-network
```

交付前应确认：

- PICO 依赖导入成功；
- ZeroLab 发送端能向机器人当前 IP 的 UDP `18000` 发送 `992-byte/50 Hz` 数据；
- 无网桥 direct、普通网桥 direct 均按现场 IP 验证；
- alias 模式只有在现场确有需要并完成严格抓包后才启用；
- A/Y 映射按 `WAIT_ARM -> ARM -> PAUSE` 验证；
- `zerolab-hardware.service` 为 `disabled + inactive`，不会开机自动启动。
