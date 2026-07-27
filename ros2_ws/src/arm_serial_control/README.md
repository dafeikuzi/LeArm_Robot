# arm_serial_control

This package provides a generic serial transport node. It does not define arm
movement commands because those bytes are specific to the arm controller.

Build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Workspace
colcon build --packages-select arm_serial_control
source install/setup.bash
```

Start the node after setting the correct serial port and baud rate:

```bash
ros2 run arm_serial_control serial_controller --ros-args \
  -p port:=/dev/ttyUSB0 -p baud_rate:=9600 -p auto_connect:=true
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
