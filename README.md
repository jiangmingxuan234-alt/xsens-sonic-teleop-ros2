# SONIC + ZeroLab 动捕部署与使用

本仓库是 `com.bxi.sonic` Mod。新机器人通常已经预装 BXI 基础控制工程；客户不需要重新安装基础 ROS/硬件控制器。本说明按照官方 SONIC 文档的目录约定，把 SONIC/PICO 与 ZeroLab F2 Pro 动捕接入现有工程。

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
