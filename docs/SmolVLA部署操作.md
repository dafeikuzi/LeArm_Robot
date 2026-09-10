# SmolVLA TCP 部署操作

默认检查点：

```text
/root/LeArm_Robot/Smolvla_trianing/20260909_102253/outputs/smolvla_learm/checkpoints/last/pretrained_model
```

模型和 VLM 缓存均从 Linux 文件系统离线加载。Windows 原生持有 CH340 和摄像头，WSL 默认通过 TCP 8766、8767 使用硬件。

## 1. 启动硬件

管理员 PowerShell：

```powershell
cd \\wsl.localhost\Ubuntu-24.04\root\LeArm_Robot
.\tools\run_windows_hardware_bridge.ps1 -SerialPort auto -CameraIndex 0
```

第一个 WSL 终端：

```bash
cd /root/LeArm_Robot
tools/run_learm_tcp_stack.sh
```

第二个 WSL 终端验证：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash
ros2 topic hz /learm_network_camera/image_raw
ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}'
```

## 2. 安全推理顺序

先运行只读推理：

```bash
tools/run_smolvla_learm.sh --task '抓取白色锤子' --max-steps 20
```

确认输出和工作空间安全后运行 5 步：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' --allow-motion --hz 1 \
  --duration-ms 500 --max-steps 5
```

确认稳定后连续推理：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' --allow-motion --hz 1 --duration-ms 500
```

默认使用 `/learm_network_camera/image_raw`，图像超过 2 秒会停止推理。ROS 服务等待 35 秒，临时通信连续失败 5 次才退出；结果未知的动作不会自动重发。

完整设备解绑、Windows 防火墙、TCP 健康检查、故障恢复和原生 Linux 回退见 [SmolVLA TCP 推理、采集与设备排错](./SmolVLA推理与设备排错操作.md)。
