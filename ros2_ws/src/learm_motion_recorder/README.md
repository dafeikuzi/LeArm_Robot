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
keyframe. The table supports replacement, deletion, reordering, JSON save/load,
and selected or full playback. Playback uses `/learm_driver/move_pose`, so the
gripper and all five rotary joints are sent in one STM32 command.

The GUI shows target values and can read STM32 PWM estimates, but it does not
provide physical encoder feedback.
