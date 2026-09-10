# SmolVLA TCP 推理、采集与设备排错

本文适用于 Windows 11 + WSL2。Windows 独占 CH340 和 USB 摄像头，WSL 只通过 TCP 使用硬件，不需要 `/dev/ttyUSB*`、`/dev/video*` 或日常 `usbipd attach`。

```text
Windows COM -> TCP 8766 -> WSL serial_controller -> learm_driver
Windows camera -> TCP 8767 -> WSL network_camera_node -> ROS Image
```

## 1. 一次性退出 USBIP

在管理员 PowerShell 中执行。BUSID 必须以 `usbipd list` 当前结果为准：

```powershell
usbipd list
usbipd detach --busid <CH340-BUSID>
usbipd unbind --busid <CH340-BUSID>
usbipd detach --busid <CAMERA-BUSID>
usbipd unbind --busid <CAMERA-BUSID>
```

之后不要再对这两个设备执行 `usbipd bind/attach`。Windows 设备管理器中应能看到 COM 串口和摄像头。

## 2. 启动 Windows 硬件桥

在管理员 PowerShell 中进入项目的 WSL 路径。发行版名称可用 `wsl -l -q` 查看：

```powershell
cd \\wsl.localhost\Ubuntu\root\LeArm_Robot
.\tools\run_windows_hardware_bridge.ps1 -ListDevices
```

确认 CH340 的 COM 编号和可用摄像头索引后启动：

```powershell
.\tools\run_windows_hardware_bridge.ps1 `
  -SerialPort auto `
  -CameraIndex 0 `
  -CameraBackend auto `
  -SerialTcpPort 8766 `
  -CameraTcpPort 8767
```

脚本会创建 `%LOCALAPPDATA%\LeArmBridge\venv`，通过清华 PyPI 镜像安装依赖，自动确定 WSL 虚拟网卡地址，并创建只允许当前 WSL IP 访问两个端口的防火墙规则。多个 `1a86:7523` 设备存在时，必须把 `auto` 改为明确的 `COM3` 等端口。

若 PowerShell 首次显示脚本安全警告，可以选择 `R` 运行一次，或先执行：

```powershell
Unblock-File .\tools\run_windows_hardware_bridge.ps1
```

脚本会让清华镜像绕过 Windows 系统代理。如果日志仍显示连接 `127.0.0.1:7897`，检查 Windows“设置 -> 网络和 Internet -> 代理”，关闭已经失效的手动代理，或者重新启动对应的代理程序。

`CameraBackend=auto` 会先尝试 Media Foundation，取帧失败后自动切换 DirectShow。若已经确认 MSMF 持续出现 `can't grab frame`，可直接使用 `-CameraBackend dshow`。

正常日志应包含：

```text
serial ready: COM3 at 115200 baud (DTR=high, RTS=low)
camera ready: index=0, 640x480 at 15 FPS
serial TCP listening on <Windows地址>:8766
camera TCP listening on <Windows地址>:8767
```

保持该 PowerShell 窗口运行。串口或摄像头拔插后，桥会每 2 秒尝试重开；不需要重启 WSL。

## 3. WSL 检查 TCP 链路

```bash
WINDOWS_HOST=$(ip route show default | awk '/default/ {print $3; exit}')
echo "$WINDOWS_HOST"
nc -vz -w 2 "$WINDOWS_HOST" 8766
nc -vz -w 2 "$WINDOWS_HOST" 8767
```

若 `nc` 未安装：

```bash
sudo apt update
sudo apt install -y netcat-openbsd
```

`Connection refused` 表示 Windows 桥未监听或地址错误；`timed out` 通常表示防火墙未放行。重新以管理员身份运行 PowerShell 脚本会更新当前 WSL IP 的规则。

## 4. 启动 TCP 推理硬件栈

在第一个 WSL 终端运行：

```bash
cd /root/LeArm_Robot
tools/run_learm_tcp_stack.sh
```

该命令自动取得 Windows 主机地址，并启动 TCP 串口控制器、LeArm 驱动和网络相机节点。也可直接运行：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash
ros2 launch learm_vla_bridge tcp_inference.launch.py
```

## 5. 验证相机与 STM32

在第二个 WSL 终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash

ros2 topic hz /learm_network_camera/image_raw
ros2 param get /serial_controller transport
ros2 param get /serial_controller tcp_host
ros2 param get /learm_driver ack_timeout_ms
ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}'
```

预期 `transport` 为 `tcp`、驱动超时为 `30000`，状态响应包含 `success=True`。相机断开时节点不会重发旧帧；推理端检测到图像超过 2 秒会停止，不再发送动作。

连续检查 100 次状态：

```bash
for i in $(seq 1 100); do
  timeout 40 ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}' || break
done
```

## 6. 推理

默认图像来源已经是 `/learm_network_camera/image_raw`，不要再传 `/dev/video0`。

先做 20 步只读推理：

```bash
cd /root/LeArm_Robot
tools/run_smolvla_learm.sh --task '抓取白色锤子' --max-steps 20
```

再做 5 步受限运动：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' \
  --allow-motion \
  --hz 1 \
  --duration-ms 500 \
  --max-steps 5 \
  --service-timeout-s 35 \
  --retry-delay-s 1 \
  --max-communication-failures 5
```

最后连续推理，只需删除 `--max-steps 5`：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' \
  --allow-motion \
  --hz 1 \
  --duration-ms 500
```

`move_pose` 结果未知时不会重发原动作。程序会清空策略动作队列、查询真实状态并重新推理；连续通信失败 5 次才停止。默认退出不会自动急停。

## 7. 数据采集

停止已运行的推理硬件栈后，用一个命令启动 TCP 驱动、相机、观察与记录节点：

```bash
ros2 launch learm_vla_bridge tcp_collection.launch.py \
  task:='抓取白色锤子' \
  dataset_root:=/root/LeArm_Robot/datasets/learm_vla/raw
```

数据始终写入 WSL Linux 文件系统。Episode 服务保持不变：

```bash
ros2 service call /learm_vla_recorder/start_episode std_srvs/srv/Trigger '{}'
ros2 service call /learm_vla_recorder/stop_episode std_srvs/srv/Trigger '{}'
ros2 service call /learm_vla_recorder/abort_episode std_srvs/srv/Trigger '{}'
```

## 8. 浏览器控制

```bash
cd /root/LeArm_Robot
python3 tools/vla_web_gui.py
```

Windows 浏览器访问 `http://localhost:8765`。页面中的 Windows 主机地址会从 WSL 默认路由自动填写，串口和摄像头 TCP 端口默认为 8766、8767。浏览器直接调用 WSL 中的 ROS 节点；Windows 桥只负责原始硬件传输。

Web ROS 服务等待为 35 秒，HTTP 外层等待为 40 秒，与驱动的 30 秒 STM32 ACK 超时匹配。

## 9. 故障恢复

串口 TCP 正常但 STM32 无回包时，先看 Windows 桥日志是否仍显示 `serial ready`，再确认没有其他 Windows 程序占用 COM。重插 CH340 后等待桥自动重开，然后重新调用 `get_status`。

```bash
ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}'
ros2 service call /learm_driver/clear_estop std_srvs/srv/Trigger '{}'
```

网络相机无图像时检查：

```bash
ros2 node list | grep network_camera
ros2 topic info /learm_network_camera/image_raw -v
ros2 topic hz /learm_network_camera/image_raw
```

Windows 桥退出后，WSL 节点会进入重连状态。重新启动桥即可，不需要 USBIP，也不应依赖 WSL 中残留的设备节点。

## 10. 原生 Linux 回退

仅在真正的原生 Linux 或明确需要 USB 直通时使用：

```bash
ros2 launch learm_driver learm_bringup.launch.py \
  serial_transport:=serial serial_port:=/dev/ttyUSB0

ros2 launch learm_vla_bridge recording.launch.py \
  camera_source:=v4l2 camera_device:=/dev/video0

tools/run_smolvla_learm.sh \
  --camera-source v4l2 --camera /dev/video0 --task '抓取白色锤子'
```

这些是回退命令，不是 WSL 默认流程。
