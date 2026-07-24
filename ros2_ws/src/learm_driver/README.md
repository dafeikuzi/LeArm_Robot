# learm_driver

`learm_driver` converts calibrated joint angles in radians into STM32 PWM
commands. It relies on `arm_serial_control` for raw serial transport.

Build and start the pair:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
colcon build
source install/setup.bash
ros2 launch learm_driver learm_bringup.launch.py
```

If the CH340 device was reconnected and its Linux name changed, pass the
current device explicitly, for example:

```bash
ros2 launch learm_driver learm_bringup.launch.py serial_port:=/dev/ttyUSB1
```

Services:

- `/learm_driver/move_joints`: sends one to five rotational joint targets,
  `joint_2` through `joint_6`, in radians and a duration in milliseconds.
- `/learm_driver/move_pose`: sends the gripper opening and fixed-order
  `joint_2` through `joint_6` targets in one STM32 command, for synchronized
  pose playback.
- `/learm_driver/set_gripper`: commands `joint_1` as a gripper, where
  `opening: 0.0` is fully closed and `opening: 1.0` is fully open.
- `/learm_driver/get_status`: returns the STM32 estimated current and target PWM values.
- `/learm_driver/emergency_stop`: freezes PWM interpolation and latches software stop.
- `/learm_driver/clear_estop`: permits later movement commands.
- `/learm_driver/calibration_move_pwm`: moves one servo by PWM for preliminary
  zero, direction and safe-range testing. It only works when
  `calibration_mode_enabled` is true and formal motion remains disabled.

Motion is enabled in `config/learm_driver.yaml` after confirming the gripper
and all five rotational joint ranges. Review calibration again after any
mechanical adjustment.

For interactive PWM calibration after formal motion has been enabled, first
temporarily set `calibration_enabled: false` in the YAML. Then start the
driver with `calibration_mode_enabled:=true` and run:

```bash
ros2 run learm_calibration_gui learm_calibration_gui
```
