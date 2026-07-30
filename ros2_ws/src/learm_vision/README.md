# learm_vision

`learm_vision` contains the LeArm vision algorithm prototypes:

- `color_tracker`: HSV color-object detection for quick camera experiments.
- `model_detector`: YOLO model inference that publishes object detections.
- dataset tools for frame capture, train/val splitting, validation, and YOLO
  training.

Build:

```bash
cd /home/liuzhiwei/LeArm_Robot/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select learm_vision
source install/setup.bash
```

Install the model dependency:

```bash
sudo apt install python3-pip
python3 -m pip install --user ultralytics
```

## YOLO Model Detection

Run inference on an image or video first. This avoids VMware webcam stream
issues while developing the algorithm:

```bash
ros2 run learm_vision model_detector --ros-args \
  -p model_path:=yolov8n.pt \
  -p input_source:=/path/to/test.jpg \
  -p show_image:=false
```

Or with the launch file:

```bash
ros2 launch learm_vision model_detector.launch.py \
  model_path:=yolov8n.pt \
  input_source:=/path/to/test.jpg \
  show_image:=false \
  log_detections:=true
```

Detection results are published as JSON:

```bash
ros2 topic echo /learm_model_detector/detections
```

Example output:

```json
{"frame_width": 640, "frame_height": 480, "detections": [{"class_id": 0, "name": "object_0", "confidence": 0.86, "bbox_xyxy": [10.0, 20.0, 120.0, 180.0], "center": [65.0, 100.0]}]}
```

After training a custom model, run:

```bash
ros2 run learm_vision model_detector --ros-args \
  -p model_path:=runs/detect/learm_objects/weights/best.pt \
  -p input_source:=/path/to/test_video.mp4
```

## Dataset Workflow

The default dataset skeleton is:

```text
/home/liuzhiwei/LeArm_Robot/datasets/learm_objects
```

Capture frames:

```bash
cd /home/liuzhiwei/LeArm_Robot
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run learm_vision capture_frames -- \
  --source /path/to/video.mp4 \
  --output datasets/learm_objects/images/raw \
  --every-n 10 \
  --max-frames 200
```

Label the images in `datasets/learm_objects/images/raw` with a YOLO annotation
tool, then put matching `.txt` labels in `datasets/learm_objects/labels/raw`.
Edit `datasets/learm_objects/data.yaml` when the real class names are known.

Split the raw data:

```bash
ros2 run learm_vision split_yolo_dataset -- \
  --dataset-root datasets/learm_objects \
  --val-ratio 0.2
```

Check the dataset:

```bash
ros2 run learm_vision check_yolo_dataset -- \
  --dataset-root datasets/learm_objects
```

Train YOLO nano on CPU:

```bash
ros2 run learm_vision train_yolo -- \
  --data datasets/learm_objects/data.yaml \
  --model yolov8n.pt \
  --epochs 50 \
  --device cpu
```

## HSV Color Tracker

Run with the default camera:

```bash
ros2 run learm_vision color_tracker
```

Run with a specific Linux camera device:

```bash
ros2 run learm_vision color_tracker --ros-args -p camera_device:=/dev/video0
```

The default camera format is `MJPG 320x240` because some VMware webcam devices
open but time out in their default format. You can override it:

```bash
ros2 run learm_vision color_tracker --ros-args \
  -p camera_device:=/dev/video0 \
  -p pixel_format:=MJPG \
  -p frame_width:=320 \
  -p frame_height:=240 \
  -p fps:=15
```

The detector defaults to red. Presets are `red`, `green`, `blue`, and
`yellow`:

```bash
ros2 run learm_vision color_tracker --ros-args -p target_color:=green
```

Detection results are published as JSON text:

```bash
ros2 topic echo /learm_color_tracker/object_detection
```

Use the HSV control window to tune the threshold for the current lighting.
Press `q` or `Esc` in the OpenCV image window to quit.
