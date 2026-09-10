# learm_vla_bridge

`learm_vla_bridge` is the local observation layer for a VLA policy hosted on a
separate GPU machine. It does not send motion commands.

`network_camera_node` pulls the newest JPEG from the Windows bridge. By default,
`observation_node` subscribes to that ROS image and polls `/learm_driver/get_status`.
It publishes:

- `/learm_vla_observation/image_raw` (`sensor_msgs/Image`): BGR camera frames.
- `/learm_vla_observation/observation` (`std_msgs/String`): compact JSON with
  task text, STM32 PWM estimates, calibrated joint positions, gripper opening,
  motion state, and emergency-stop state.

The state is derived from STM32 PWM estimates, not physical encoder feedback.
The calibration values in `config/observation.yaml` must remain identical to
`learm_driver/config/learm_driver.yaml`.

Build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
colcon build --packages-select learm_vla_bridge
source install/setup.bash
```

Start the arm driver first, then run the observation node:

```bash
ros2 launch learm_driver learm_bringup.launch.py tcp_host:=172.28.224.1
ros2 launch learm_vla_bridge tcp_collection.launch.py \
  windows_host:=172.28.224.1 \
  task:='Pick up the red block and place it in the bin.'
```

Inspect the local streams:

```bash
ros2 topic echo /learm_vla_observation/observation
rqt_image_view /learm_vla_observation/image_raw
```

The future remote policy client must consume these two topics and send proposed
actions to a separate local safety executor. It must never command the serial
controller directly.

## Record Demonstrations

The recorder writes a raw, append-only episode format. It never commands the
arm. During manual control, each sample contains a JPEG frame, the PWM-derived
current state, the STM32 target pose as the action label, a task description,
and timestamps. The target pose is only a command-derived label, not physical
feedback.

Each observation also carries STM32 status health fields. A static arm is valid
when the status values stay equal but successful status replies continue to
refresh `stm32_status_generation`. The recorder waits for a new generation
before an episode can start, reuses the last STM32 state when a status request
times out, and does not discard video frames because the arm is static, a
status value is unchanged, or a status request times out. Episode metadata records only
`quality.status_timeout_count`; a successful episode remains trainable even if
timeouts occurred. The converter itself does not perform any quality
selection.

## Collection GUI

The collection GUI starts and supervises the local driver, manual control GUI,
observation node, and episode recorder from one window. It also provides the
start, successful stop, and abort controls for each episode:

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash
ros2 run learm_vla_bridge vla_collection_gui
```

Set the serial port, camera device, task, dataset directory, and frequencies,
then use the buttons in order. The GUI only stops processes that it started.
Use `成功结束` before closing the recorder when a demonstration succeeds.

```text
datasets/learm_vla/raw/
  episode_000001/
    metadata.json
    records.jsonl
    frames/000000.jpg
```

Start the driver and manual motion recorder first. Then launch the observation
and data recorder together with a task that describes the demonstration:

```bash
ros2 launch learm_vla_bridge recording.launch.py \
  camera_source:=topic \
  windows_host:=172.28.224.1 \
  camera_fps:=15 \
  publish_rate_hz:=15.0 \
  status_poll_rate_hz:=15.0 \
  record_rate_hz:=15.0 \
  task:='Pick up the red block and place it in the bin.'
```

After the camera and observation topics are active, start and stop one episode
from another terminal:

```bash
ros2 service call /learm_vla_recorder/start_episode std_srvs/srv/Trigger '{}'
ros2 service call /learm_vla_recorder/stop_episode std_srvs/srv/Trigger '{}'
```

Use `abort_episode` after an unsuccessful or interrupted demonstration. It
retains the files with `outcome: aborted` for inspection; only episodes marked
`completed: true` and `outcome: success` should enter the training conversion.

```bash
ros2 service call /learm_vla_recorder/abort_episode std_srvs/srv/Trigger '{}'
```

Do not record while the software emergency stop is active. The recorder skips
samples that lack a valid status, image, or target pose. Review each episode
before training, then convert the accepted raw episodes to LeRobot format in a
separate step.
