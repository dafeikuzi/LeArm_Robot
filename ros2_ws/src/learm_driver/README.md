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

Services:

- `/learm_driver/move_joints`: sends one to six `joint_1` through `joint_6`
  targets in radians and a duration in milliseconds.
- `/learm_driver/get_status`: returns the STM32 estimated current and target PWM values.
- `/learm_driver/emergency_stop`: freezes PWM interpolation and latches software stop.
- `/learm_driver/clear_estop`: permits later movement commands.
- `/learm_driver/calibration_move_pwm`: moves one servo by PWM for preliminary
  zero, direction and safe-range testing. It only works when
  `calibration_mode_enabled` is true and formal motion remains disabled.

Motion is intentionally disabled in `config/learm_driver.yaml`. Set all six
joint calibration ranges and then set `calibration_enabled: true` only after
confirming each joint's safe direction and mechanical limits.

For interactive PWM calibration, start the driver with
`calibration_mode_enabled:=true`, then run:

```bash
ros2 run learm_calibration_gui learm_calibration_gui
```
