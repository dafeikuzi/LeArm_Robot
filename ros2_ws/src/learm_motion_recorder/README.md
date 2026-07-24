# learm_motion_recorder

This PyQt5 node is the normal LeArm control interface. Its sliders send the
latest target automatically: while a slider is moving, changes are coalesced
to at most one update every 100 ms, and releasing the slider flushes the final
target as soon as the active serial transaction completes.

Start the driver first:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
source install/setup.bash
ros2 launch learm_driver learm_bringup.launch.py
```

In a second terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash
ros2 run learm_motion_recorder learm_motion_recorder
```

Click `添加当前姿态` to append the current GUI targets and action duration as a
keyframe. Choose `同步` to send the full pose in one STM32 command, or use the
default `安全顺序` mode to execute `joint_6`, `joint_5 + joint_4`, and
`joint_3 + joint_2` in separate stages. The gripper can run before, after, or
outside the staged motion. The table supports replacement, deletion, reordering,
JSON save/load, and selected or full playback.

`复位到零位` moves `joint_6`, `joint_5 + joint_4`, and `joint_3 + joint_2` to
0 degrees in that order, then fully closes the gripper. Each stage uses the
current action-time setting.

The GUI shows target values and can read STM32 PWM estimates, but it does not
provide physical encoder feedback.
