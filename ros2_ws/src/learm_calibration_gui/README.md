# learm_calibration_gui

`learm_calibration_gui` is a PyQt5 panel for checking one LeArm servo at a
time before formal angle calibration is configured. It communicates only with
the existing `/learm_driver` services and never opens the serial device.

Start the serial controller and driver in calibration mode first:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
source install/setup.bash
ros2 launch learm_driver learm_bringup.launch.py calibration_mode_enabled:=true
```

In a second terminal, start the panel:

```bash
source /opt/ros/jazzy/setup.bash
source /home/liuzhiwei/LeArm_Robot/ros2_ws/install/setup.bash
ros2 run learm_calibration_gui learm_calibration_gui
```

Choose a joint with its radio button, drag its PWM slider and set the motion
time. Press **Send Selected Joint** and confirm the dialog to command the
servo. The GUI can change PWM one microsecond at a time, but the driver still
enforces each joint's configured calibration range.

The current test configuration allows `joint_1` from `500` to `1400 us`, and
`joint_2` through `joint_6` from `500` to `2500 us`. Motion duration is from
`200` to `1000 ms`.

Use **Emergency Stop** immediately when motion is unexpected. The software
stop is not a replacement for removing servo power or a hardware emergency
stop.
