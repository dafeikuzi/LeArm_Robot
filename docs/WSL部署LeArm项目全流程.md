# Windows + WSL2 部署 LeArm Robot 全流程

本文覆盖 Windows、WSL2 Ubuntu、RTX 5070、TCP 硬件桥、ROS 2 Jazzy、VLA 采集和 SmolVLA 训练。Windows 原生持有串口与摄像头，WSL2 运行 Linux、ROS 2、采集和推理程序。

## 1. 约定和要求

建议使用 Windows 11、WSL2、Ubuntu 24.04、ROS 2 Jazzy、NVIDIA RTX 5070。当前项目串口波特率为 115200；WSL 默认通过 Windows 主机的 TCP 8766 使用串口，通过 TCP 8767 使用摄像头。

~~~
Windows 共享盘: D:\APP_data\VMShare
WSL 共享盘:   /mnt/d/APP_data/VMShare
WSL 源码:     /home/USER/LeArm_Robot
LeRobot 数据: /mnt/d/APP_data/VMShare/lerobot_v3
训练输出:     /mnt/d/APP_data/VMShare/outputs
~~~

建议源码和 ROS 工作区放在 WSL Linux 文件系统，数据和训练输出放在共享盘。

## 2. 安装 WSL2

管理员 PowerShell：

~~~
wsl --install -d Ubuntu-24.04
~~~

重启 Windows，首次进入 Ubuntu 并创建 Linux 用户。检查并更新：

~~~
wsl --list --verbose
wsl --set-default Ubuntu-24.04
wsl --update
wsl --shutdown
wsl -d Ubuntu-24.04
~~~

Ubuntu 的 VERSION 必须为 2。

## 3. 配置 RTX 5070

在 Windows 安装 NVIDIA 官方 Windows 驱动，然后执行：

~~~
nvidia-smi
~~~

在 WSL 中执行：

~~~
nvidia-smi
ls -l /dev/dxg
~~~

两处都能看到 GPU 才继续。不要在 WSL 安装 nvidia-driver、cuda-drivers 或 Linux 显示驱动；WSL 使用 Windows 驱动映射的 CUDA 接口。[NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/)

## 4. 共享盘和源码

~~~
ls -la /mnt/d/APP_data/VMShare
df -h /mnt/d/APP_data/VMShare
~~~

如果 Windows 共享盘已有项目，复制到 WSL：

~~~
mkdir -p ~/LeArm_Robot
rsync -a --info=progress2 \
  --exclude='ros2_ws/build/' --exclude='ros2_ws/install/' --exclude='ros2_ws/log/' \
  /mnt/d/APP_data/VMShare/LeArm_Robot/ ~/LeArm_Robot/
~~~

不要复制旧的 build、install、log；它们包含原环境路径，必须在 WSL 重新编译。

## 5. Ubuntu 依赖和权限

~~~
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget unzip rsync \
  python3-pip python3-venv python3-dev usbutils v4l-utils ffmpeg \
  libopencv-dev python3-opencv libserial-dev
sudo usermod -aG dialout,video $USER
~~~

执行后退出 WSL，在 PowerShell 重新进入：

~~~
wsl --terminate Ubuntu-24.04
wsl -d Ubuntu-24.04
~~~

检查：

~~~
groups
~~~

输出应有 dialout 和 video。

## 6. 安装 ROS 2 Jazzy

按 [ROS 2 Jazzy Ubuntu 官方文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 配置软件源，然后：

~~~
sudo apt install -y ros-jazzy-desktop ros-jazzy-cv-bridge \
  ros-jazzy-image-transport ros-jazzy-camera-info-manager \
  ros-jazzy-rqt python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
~~~

## 7. 编译项目

~~~
source /opt/ros/jazzy/setup.bash
cd ~/LeArm_Robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg list | grep -E 'learm|arm_serial'
~~~

每个新终端执行：

~~~
source /opt/ros/jazzy/setup.bash
source ~/LeArm_Robot/ros2_ws/install/setup.bash
~~~

## 8. Windows TCP 硬件桥

不把 CH340 或摄像头附加到 WSL。管理员 PowerShell 进入项目目录并启动桥：

~~~powershell
cd \\wsl.localhost\Ubuntu-24.04\root\LeArm_Robot
.\tools\run_windows_hardware_bridge.ps1 -ListDevices
.\tools\run_windows_hardware_bridge.ps1 -SerialPort auto -CameraIndex 0
~~~

脚本使用国内镜像创建独立环境、自动配置仅当前 WSL 地址可访问的防火墙规则，并监听串口 8766、摄像头 8767。完整的一次性 USBIP 解绑、健康检查和排错命令见 [SmolVLA TCP 推理、采集与设备排错](./SmolVLA推理与设备排错操作.md)。

## 9. 启动 ROS 2 TCP 驱动、观察和 Web GUI

终端 A：

~~~
source /opt/ros/jazzy/setup.bash
source ~/LeArm_Robot/ros2_ws/install/setup.bash
tools/run_learm_tcp_stack.sh
~~~

项目配置必须保持 115200。检查节点：

~~~
ros2 node list
ros2 topic list | grep -E 'learm|serial'
ps -ef | grep -E 'serial_controller|learm_driver' | grep -v grep
~~~

终端 B：

~~~
source /opt/ros/jazzy/setup.bash
source ~/LeArm_Robot/ros2_ws/install/setup.bash
ros2 launch learm_vla_bridge observation.launch.py camera_source:=topic
ros2 topic echo /learm_vla_observation/observation --once
~~~

终端 C：

~~~
source /opt/ros/jazzy/setup.bash
source ~/LeArm_Robot/ros2_ws/install/setup.bash
python3 tools/vla_web_gui.py
~~~

Windows 浏览器访问 `http://localhost:8765`，填写 Windows 主机地址与 8766、8767 两个端口。

不使用 GUI 时：

~~~
ros2 launch learm_vla_bridge recording.launch.py \
  camera_source:=topic \
  dataset_root:=/mnt/d/APP_data/VMShare/learm_vla/raw \
  task='抓取棕色小狗'
~~~

采集前确认机械臂安全、任务名正确、摄像头正常、只有一个驱动实例且无持续状态超时。

## 10. 安装 LeRobot 和 SmolVLA

~~~
python3 -m venv ~/venvs/lerobot
source ~/venvs/lerobot/bin/activate
python -m pip install -U pip
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
python -m pip install -e '.[smolvla,training]'
~~~

检查 PyTorch GPU：

~~~
python - <<'PY'
import torch
print('torch:', torch.__version__)
print('cuda:', torch.cuda.is_available())
if torch.cuda.is_available(): print('gpu:', torch.cuda.get_device_name(0))
PY
~~~

## 11. 验证 LeRobot v3 数据

~~~
test -f /mnt/d/APP_data/VMShare/lerobot_v3/meta/info.json && echo dataset_ok
du -sh /mnt/d/APP_data/VMShare/lerobot_v3
python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
root='/mnt/d/APP_data/VMShare/lerobot_v3'
ds=LeRobotDataset('learm_vla', root=root, download_videos=False)
print('frames:', len(ds), 'episodes:', ds.meta.total_episodes)
x=ds[0]
print('keys:', sorted(x.keys()))
print('image:', x['observation.images.camera'].shape)
print('state:', x['observation.state'].shape, 'action:', x['action'].shape)
PY
~~~

根目录必须直接包含 data、meta、videos。

## 12. SmolVLA 冒烟和正式训练

先测试 10 步：

~~~
CUDA_VISIBLE_DEVICES=0 lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=learm_vla \
  --dataset.root=/mnt/d/APP_data/VMShare/lerobot_v3 \
  --dataset.video_backend=pyav \
  --policy.device=cuda --policy.use_amp=true \
  --policy.push_to_hub=false --batch_size=1 --steps=10 \
  --num_workers=0 \
  --output_dir=/mnt/d/APP_data/VMShare/outputs/smolvla_smoke \
  --job_name=smolvla_smoke --wandb.enable=false
~~~

成功标准：完成第 10 步、GPU 显存有占用，且没有 CUDA out of memory、视频解码或 image feature 错误。

正式训练：

~~~
CUDA_VISIBLE_DEVICES=0 lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=learm_vla \
  --dataset.root=/mnt/d/APP_data/VMShare/lerobot_v3 \
  --dataset.video_backend=pyav --policy.device=cuda --policy.use_amp=true \
  --policy.push_to_hub=false --batch_size=2 --steps=20000 --save_freq=2000 \
  --num_workers=0 \
  --output_dir=/mnt/d/APP_data/VMShare/outputs/smolvla_learm \
  --job_name=smolvla_learm --wandb.enable=false
~~~

显存不足时将 batch_size 改为 1。Windows 侧监控：

~~~
nvidia-smi -l 2
~~~

恢复 checkpoint：

~~~
lerobot-train --resume=true \
  --config_path=/mnt/d/APP_data/VMShare/outputs/smolvla_learm/checkpoints/last/pretrained_model/train_config.json \
  --output_dir=/mnt/d/APP_data/VMShare/outputs/smolvla_learm_resume \
  --policy.device=cuda
~~~

## 13. 常见问题

### GPU 不可用

在 Windows 检查 nvidia-smi，执行 wsl --update 和 wsl --shutdown。不要在 WSL 安装 Linux NVIDIA 驱动。

### 串口 TCP 不通或状态超时

检查 Windows 桥的 `serial ready` 日志，并从 WSL 对 Windows 默认网关的 8766 端口执行 `nc -vz`。关闭其他占用 COM 的 Windows 程序，确保只有一个 `serial_controller`。

### 网络摄像头没有图像

检查 Windows 桥的 `camera ready` 日志、TCP 8767，以及 `/learm_network_camera/image_raw` 的发布频率。

### ROS 话题不存在

每个终端都 source ROS 和工作区，并保持相同的 ROS_DOMAIN_ID：

~~~
echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID
~~~

### 训练显存不足

先使用 batch_size=1、num_workers=0、policy.use_amp=true，稳定后再增大 batch size。

## 14. 启动和关闭顺序

~~~
启动：Windows nvidia-smi -> Windows 硬件桥 -> WSL 检查 TCP
     -> TCP 驱动/网络相机 -> Web GUI/录制 -> 冒烟训练 -> 正式训练

关闭：停止 GUI/录制 -> 停止观察 -> 停止驱动
     -> 保存输出 -> 停止 Windows 硬件桥 -> 必要时 wsl --shutdown
~~~

不需要在每次启动或关闭时执行 USBIP 命令。

不要在数据写入或 checkpoint 保存过程中强制关闭 WSL、拔出共享盘或断电。
