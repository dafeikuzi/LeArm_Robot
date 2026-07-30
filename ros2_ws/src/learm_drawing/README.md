# learm_drawing

`learm_drawing` provides open-loop trajectory training tools for drawing with
the LeArm. It does not perform inverse kinematics, visual feedback, or machine
learning. It plays calibrated joint-space poses through `/learm_driver/move_pose`.

Build:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
colcon build --packages-select learm_drawing
source install/setup.bash
```

Dry-run the default pen tap sequence:

```bash
ros2 launch learm_drawing trajectory_player.launch.py dry_run:=true
```

Run a specific training trajectory after calibrating the pose file:

```bash
ros2 launch learm_drawing trajectory_player.launch.py \
  pose_file:=/path/to/your/drawing_poses.yaml \
  trajectory_file:=/home/liuzhiwei/LeArm_Robot/ros2_ws/install/learm_drawing/share/learm_drawing/trajectories/phase3_short_lines.yaml \
  dry_run:=false
```

The pose file must define five joint angles in degrees, ordered as
`joint_2` through `joint_6`. Keep `requires_user_calibration: true` until the
placeholder angles have been replaced with measured safe poses.

See `TRAINING.md` for the staged training checklist.
