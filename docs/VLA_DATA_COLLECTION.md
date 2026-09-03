# VLA 数据采集

本文档记录 LeArm 采集一条 VLA 抓取 demonstration 的命令顺序。记录器只采集数据，
不会主动控制机械臂；机械臂动作通过手动控制 GUI 完成。

默认设备和数据目录如下，实际设备名不一致时请按现场设备修改：

- 串口：`/dev/ttyUSB0`
- 摄像头：`/dev/video0`
- 数据目录：`/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw`

## 首次启动或源码更新后构建

在源码有改动、特别是新增采集 GUI 后，先重新构建工作区：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws

colcon build --packages-select learm_driver learm_motion_recorder learm_vla_bridge \
  --symlink-install --allow-overriding learm_driver
source install/setup.bash
```

## 图形化流程

完成工作区构建并 source 环境后，可使用单窗口流程控制器：

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash
ros2 run learm_vla_bridge vla_collection_gui
```

在窗口中填写串口、摄像头、任务和数据目录，按以下按钮顺序操作：

1. `构建工作区`
2. `启动机械臂驱动`
3. `打开控制 GUI`
4. `启动观察和记录节点`
5. `开始记录`
6. 完成任务后点击 `成功结束`，失败则点击 `放弃记录`

窗口只关闭由它启动的进程。正在记录时必须先成功结束或放弃记录。

## 前提

- 已完成上面的 ROS 2 工作区构建，并在每个新终端执行 `source install/setup.bash`。
- 串口设备和摄像头已连接。
- 不要同时运行 `color_tracker`，避免占用同一个摄像头。
- 机械臂处于安全位置，软件急停未激活。
- 启动前检查是否已有 `learm_vla_recorder`；不要重复启动记录器，必要时先在原终端按 `Ctrl-C` 结束旧进程。

## 终端一：启动机械臂驱动

如果串口设备不是 `/dev/ttyUSB0`，先用下面命令确认实际设备名：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

启动驱动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash

ros2 launch learm_driver learm_bringup.launch.py \
  serial_port:=/dev/ttyUSB0
```

保持该终端运行。

## 终端二：启动机械臂 GUI

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash

ros2 run learm_motion_recorder learm_motion_recorder
```

在 GUI 中将机械臂调整到安全初始位置，确认 STM32 状态正常且没有软件急停。

## 终端三：启动摄像头和 VLA 记录器

下面的配置最多记录约 15 帧/秒，并将任务文本写入每一条记录：

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash

ros2 launch learm_vla_bridge recording.launch.py \
  camera_device:=/dev/video0 \
  camera_fps:=30 \
  publish_rate_hz:=15.0 \
  status_poll_rate_hz:=15.0 \
  record_rate_hz:=15.0 \
  task:='抓取红色玩偶'
```

该命令只启动观察节点和记录器，还没有开始保存 episode。

## 终端四：检查图像和状态

确认两个 VLA 节点已启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash

ros2 node list | grep learm_vla
```

查看一次状态：

```bash
ros2 topic echo --once \
  /learm_vla_observation/observation
```

开始记录前应确认：

```text
state_valid: true
estop_active: false
```

可选：打开图像预览：

```bash
ros2 run rqt_image_view rqt_image_view
```

在窗口中选择 `/learm_vla_observation/image_raw`。

## 开始记录一条实例

确认图像和状态正常后，在终端四执行：

```bash
ros2 service call \
  /learm_vla_recorder/start_episode \
  std_srvs/srv/Trigger '{}'
```

看到 `recording episode_xxxxxx` 后，通过 GUI 完成一次完整抓取：

1. 张开夹爪。
2. 移动到物体上方。
3. 下降到物体位置。
4. 闭合夹爪并抬起物体。
5. 移动到放置区域。
6. 张开夹爪释放物体。

记录器会自动保存摄像头帧、机械臂状态、目标动作、任务文本和时间戳，不需要在 GUI 中点击保存。

## 成功结束

动作成功后等待约 1 秒，然后执行：

```bash
ros2 service call \
  /learm_vla_recorder/stop_episode \
  std_srvs/srv/Trigger '{}'
```

## 失败结束

抓取失败或动作中断时执行：

```bash
ros2 service call \
  /learm_vla_recorder/abort_episode \
  std_srvs/srv/Trigger '{}'
```

`aborted` episode 会保留用于检查，但不要用于训练。

## 数据位置和检查

数据保存在：

```text
/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw/episode_xxxxxx/
```

检查最新 episode：

```bash
LATEST_EPISODE=$(find /home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw \
  -maxdepth 1 -type d -name 'episode_*' | sort | tail -n 1)

cat "$LATEST_EPISODE/metadata.json"
wc -l "$LATEST_EPISODE/records.jsonl"
ls "$LATEST_EPISODE/frames" | head
```

成功数据应包含：

```json
"completed": true,
"outcome": "success"
```

结束程序时，先停止或放弃 episode，再依次关闭记录器、GUI 和机械臂驱动。

## 清空已有采集数据

删除前先停止正在运行的记录器，并确认下面命令列出的目录确实是要清理的 episode：

```bash
DATASET_ROOT=/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw
find "$DATASET_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'episode_*' -print
```

确认无误后将所有原始 episode 移入系统回收站（保留 `raw` 根目录）：

```bash
find "$DATASET_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'episode_*' \
  -exec gio trash {} +
```

需要永久删除时，请在系统回收站中再次确认后清空对应项目。
