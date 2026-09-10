# arm_serial_control

This package provides a generic TCP or native serial transport node. It does not define arm
movement commands because those bytes are specific to the arm controller.

Build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Workspace
colcon build --packages-select arm_serial_control
source install/setup.bash
```

WSL defaults to the Windows TCP hardware bridge:

```bash
ros2 run arm_serial_control serial_controller --ros-args \
  -p transport:=tcp -p tcp_host:=172.28.224.1 -p tcp_port:=8766
```

Native Linux serial remains available as a fallback:

```bash
ros2 run arm_serial_control serial_controller --ros-args \
  -p transport:=serial -p port:=/dev/ttyUSB0 -p baud_rate:=115200
```

The node exposes `connect`, `disconnect`, and `send_hex` services. Received
bytes are published on `/serial_controller/received_hex` as uppercase
hexadecimal text. Printable serial lines, such as the STM32 angle report, are
also published on `/serial_controller/received_text`. Application nodes should
use the protocol-aware `learm_driver` package instead of sending device frames
directly.

```bash
ros2 topic echo /serial_controller/received_text
```
