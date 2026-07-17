# LeArm Robot

This repository keeps the ROS 2 host software and STM32 firmware together.

- `ros2_ws`: ROS 2 Jazzy workspace. Build it with `colcon build` from this directory.
- `stm32/LeArm_Test`: STM32F103 firmware and Keil MDK project.

The original paths remain available as symbolic links:

- `/home/liuzhiwei/LeArm_Workspace`
- `/home/liuzhiwei/LeArm_Test`

## Windows firmware workflow

1. Install Git for Windows and Keil MDK-ARM.
2. Clone this private repository and run `git pull` before changing firmware.
3. Open `stm32/LeArm_Test/MDK-ARM/LeArm_Test.uvprojx` in Keil.
4. Build the project and flash it with ST-Link.
5. Commit only source and project files. The generated Keil outputs are ignored.

The controller uses USART1 at 9600 baud, 8 data bits, no parity, and one stop bit.
Use a 3.3 V TTL USB-to-serial adapter with TX/RX crossed and a common ground.
