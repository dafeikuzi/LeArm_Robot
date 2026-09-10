# VLA TCP 数据采集

Windows 持有 CH340 和摄像头；WSL 通过 TCP 使用硬件，数据写入 Linux 文件系统：

```text
/root/LeArm_Robot/datasets/learm_vla/raw
```

## 1. 启动 Windows 桥

管理员 PowerShell：

```powershell
cd \\wsl.localhost\Ubuntu-24.04\root\LeArm_Robot
.\tools\run_windows_hardware_bridge.ps1 -SerialPort auto -CameraIndex 0
```

确认日志包含 `serial ready`、`camera ready`、8766 和 8767 两个监听端口。

## 2. 启动完整采集栈

不要同时运行推理硬件栈或另一套驱动。WSL 中执行：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash

ros2 launch learm_vla_bridge tcp_collection.launch.py \
  task:='抓取棕色小狗' \
  dataset_root:=/root/LeArm_Robot/datasets/learm_vla/raw
```

该入口同时启动 TCP 串口控制器、LeArm 驱动、网络相机、观察节点和记录器。

## 3. 检查输入

```bash
ros2 topic hz /learm_network_camera/image_raw
ros2 topic echo /learm_vla_observation/observation --once
ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}'
```

开始记录前应确认 `state_valid=true`、`estop_active=false` 且图像持续更新。机械臂状态来自 STM32 PWM 估计，不是物理编码器反馈。

## 4. 记录 Episode

```bash
ros2 service call /learm_vla_recorder/start_episode std_srvs/srv/Trigger '{}'
ros2 service call /learm_vla_recorder/stop_episode std_srvs/srv/Trigger '{}'
```

失败或中断的演示使用：

```bash
ros2 service call /learm_vla_recorder/abort_episode std_srvs/srv/Trigger '{}'
```

`stop_episode` 将结果标为成功；`abort_episode` 保留文件供检查，但不会标为可训练成功样本。

## 5. 浏览器采集与实时控制

```bash
cd /root/LeArm_Robot
python3 tools/vla_web_gui.py
```

在 Windows 浏览器打开 `http://localhost:8765`。页面默认使用 WSL 路由得到的 Windows 地址和 TCP 8766、8767。按“启动机械臂驱动”“启动观察/记录节点”的顺序操作，再用滑块控制并开始 Episode。

Web ROS 服务等待 35 秒，HTTP 请求等待 40 秒。滑块仍按 50 ms 节流发送；TCP 桥不缓存也不重发动作。

完整部署与排错见 [SmolVLA TCP 推理、采集与设备排错](./SmolVLA推理与设备排错操作.md)。
