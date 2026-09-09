# SmolVLA 部署操作

本项目的训练结果已经保存在 Linux 文件系统中，默认检查点为：

```text
/root/LeArm_Robot/Smolvla_trianing/20260909_102253/outputs/smolvla_learm/checkpoints/last/pretrained_model
```

SmolVLM 的本地配置和 tokenizer 缓存为：

```text
/root/LeArm_Robot/Smolvla_trianing/20260909_102253/hf-cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct/snapshots/7b375e1b73b11138ff12fe22c8f2822d8fe03467
```

部署程序会强制离线加载，不依赖 Hugging Face 网络。训练集字段已经核对：`joint_1..joint_5` 对应 ROS 的 `joint_2..joint_6`，`gripper` 的范围是 0（闭合）到 1（打开），动作单位为弧度。

## 1. 启动 ROS 驱动

先在一个终端绑定 USB 设备并启动驱动。当前设备如果显示为 `/dev/ttyUSB1`，使用：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash
ros2 launch learm_driver learm_bringup.launch.py serial_port:=/dev/ttyUSB1
```

确认状态服务能返回 STM32 数据：

```bash
source /opt/ros/jazzy/setup.bash
source /root/LeArm_Robot/ros2_ws/install/setup.bash
ros2 service call /learm_driver/get_status learm_driver/srv/GetArmStatus '{}'
```

## 2. 只读推理检查（推荐先运行）

在第二个终端执行。此模式不会发送任何舵机命令：

```bash
cd /root/LeArm_Robot
tools/run_smolvla_learm.sh --task '抓取白色锤子' --max-steps 1
```

可用任务文本必须与训练集一致：`抓取白色锤子`、`抓取紫色小马`、`抓取棕色小狗`。输出中应看到 `inference device: cuda`、当前 state 和模型 action。

推理时建议暂时停止观察节点，让推理程序独占 `/dev/video0`。此时不要使用 `--image-topic`：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' --max-steps 1
```

## 3. 允许机械臂动作

确认只读推理、关节方向和限位都正确后，才添加 `--allow-motion`。训练频率是 8 Hz，示例先只跑 10 步：

```bash
tools/run_smolvla_learm.sh \
  --task '抓取白色锤子' \
  --max-steps 10 \
  --allow-motion
```

如果必须保留观察节点，再复用 ROS 图像话题：

```bash
tools/run_smolvla_learm.sh \
  --image-topic /learm_vla_observation/image_raw \
  --task '抓取白色锤子' \
  --allow-motion
```

按 `Ctrl+C` 会停止推理循环；它不会默认触发急停。如需退出时请求驱动急停，加 `--estop-on-exit`。驱动报告急停有效时，程序会拒绝启动动作，需先执行：

```bash
ros2 service call /learm_driver/clear_estop std_srvs/srv/Trigger '{}'
```

## 4. GPU 和摄像头诊断

检查 CUDA：

```bash
source /root/venvs/lerobot/bin/activate
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

检查相机是否绑定且没有被其他进程抢占：

```bash
ls -l /dev/video*
v4l2-ctl --all -d /dev/video0
fuser -v /dev/video0
```

如果 `cannot open camera`，先停止占用摄像头的观察节点，或使用上面的 `--image-topic` 模式；若两个模式都无图像，需在 Windows 重新执行 `usbipd attach --wsl --busid 7-2` 后再检查 `/dev/video0`。
