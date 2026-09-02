# learm_vla_bridge

`learm_vla_bridge` is the local observation layer for a VLA policy hosted on a
separate GPU machine. It does not send motion commands.

`observation_node` reads one OpenCV camera and polls `/learm_driver/get_status`.
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
ros2 launch learm_driver learm_bringup.launch.py
ros2 launch learm_vla_bridge observation.launch.py \
  camera_device:=/dev/video0 \
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
  camera_device:=/dev/video0 \
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
