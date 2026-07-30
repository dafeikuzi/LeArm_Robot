import json
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


IMAGE_SUFFIXES = {'.bmp', '.jpeg', '.jpg', '.png'}
VIDEO_SUFFIXES = {'.avi', '.m4v', '.mkv', '.mov', '.mp4'}


def parameter_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return True
        if normalized in ('0', 'false', 'no', 'off', ''):
            return False
    return bool(value)


class ModelDetector(Node):
    def __init__(self):
        super().__init__('learm_model_detector')

        self.model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        self.input_source = self.declare_parameter('input_source', '').value
        self.camera_backend = self.declare_parameter('camera_backend', 'v4l2').value
        self.pixel_format = self.declare_parameter('pixel_format', 'MJPG').value
        self.frame_width = int(self.declare_parameter('frame_width', 320).value)
        self.frame_height = int(self.declare_parameter('frame_height', 240).value)
        self.fps = int(self.declare_parameter('fps', 15).value)
        self.confidence_threshold = float(self.declare_parameter('confidence_threshold', 0.25).value)
        self.iou_threshold = float(self.declare_parameter('iou_threshold', 0.45).value)
        self.image_size = int(self.declare_parameter('image_size', 640).value)
        self.device = self.declare_parameter('device', 'cpu').value
        self.show_image = parameter_bool(self.declare_parameter('show_image', True).value)
        self.loop_video = parameter_bool(self.declare_parameter('loop_video', True).value)
        self.log_detections = parameter_bool(self.declare_parameter('log_detections', False).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 5.0).value)
        self.frame_index = 0

        self.publisher = self.create_publisher(String, '~/detections', 10)
        self.capture = None
        self.static_image = None
        self.input_kind = ''
        self.ready = False
        self.last_read_warning_time = 0.0
        self.window_name = 'LeArm Model Detector'

        self.model = self._load_model()
        if self.model is None:
            return

        if not self._open_input():
            return

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._process_next_frame)
        self.ready = True

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error(
                'Missing dependency: ultralytics. Install with: '
                'python3 -m pip install --user ultralytics')
            return None

        try:
            model = YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().error(f'Cannot load YOLO model {self.model_path}: {exc}')
            return None
        self.get_logger().info(f'Loaded YOLO model {self.model_path}')
        return model

    def _open_input(self):
        if not self.input_source:
            self.get_logger().error('input_source is empty. Set an image, video, or /dev/video device.')
            return False

        source_path = Path(str(self.input_source)).expanduser()
        if source_path.exists() and source_path.suffix.lower() in IMAGE_SUFFIXES:
            self.static_image = cv2.imread(str(source_path))
            if self.static_image is None:
                self.get_logger().error(f'Cannot read image {source_path}')
                return False
            self.input_kind = 'image'
            self.get_logger().info(f'Opened image {source_path}')
            return True

        if source_path.exists() and source_path.suffix.lower() in VIDEO_SUFFIXES:
            self.input_kind = 'video'
            return self._open_capture(str(source_path))

        self.input_kind = 'camera'
        return self._open_capture(self.input_source)

    def _open_capture(self, source):
        backend = cv2.CAP_V4L2 if self.camera_backend == 'v4l2' else cv2.CAP_ANY
        camera_source = int(source) if str(source).isdigit() else source
        self.capture = cv2.VideoCapture(camera_source, backend)
        if self.pixel_format:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format[:4]))
        if self.frame_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.capture.isOpened():
            self.get_logger().error(f'Cannot open input source {source}')
            return False
        self.get_logger().info(
            f'Opened {self.input_kind} {source} with {self.pixel_format} '
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
        return True

    def _process_next_frame(self):
        frame = self._read_frame()
        if frame is None:
            self._publish_empty()
            return

        detections = self._run_detection(frame)
        self.frame_index += 1
        self._publish_detections(detections, frame.shape[1], frame.shape[0])
        if self.log_detections:
            self._log_detections(detections)

        if self.show_image:
            annotated = self._draw_detections(frame.copy(), detections)
            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                rclpy.shutdown()

    def _read_frame(self):
        if self.input_kind == 'image':
            return self.static_image.copy()

        if self.capture is None or not self.capture.isOpened():
            return None

        ok, frame = self.capture.read()
        if ok:
            return frame

        if self.input_kind == 'video' and self.loop_video:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
            if ok:
                return frame

        now = time.monotonic()
        if now - self.last_read_warning_time > 2.0:
            self.last_read_warning_time = now
            self.get_logger().warn('Input frame read failed')
        return None

    def _run_detection(self, frame):
        results = self.model.predict(
            frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        names = getattr(result, 'names', {}) or {}
        detections = []
        if result.boxes is None:
            return detections

        for box in result.boxes:
            xyxy = [float(value) for value in box.xyxy[0].tolist()]
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            center_x = (xyxy[0] + xyxy[2]) / 2.0
            center_y = (xyxy[1] + xyxy[3]) / 2.0
            detections.append({
                'class_id': class_id,
                'name': str(names.get(class_id, class_id)),
                'confidence': round(confidence, 4),
                'bbox_xyxy': [round(value, 2) for value in xyxy],
                'center': [round(center_x, 2), round(center_y, 2)],
            })
        return detections

    def _publish_detections(self, detections, frame_width, frame_height):
        message = String()
        message.data = json.dumps({
            'frame_width': int(frame_width),
            'frame_height': int(frame_height),
            'detections': detections,
        })
        self.publisher.publish(message)

    def _publish_empty(self):
        self._publish_detections([], 0, 0)

    def _log_detections(self, detections):
        if not detections:
            self.get_logger().info(f'frame={self.frame_index} detections=0')
            return

        details = []
        for detection in detections[:5]:
            center_x, center_y = detection['center']
            details.append(
                f"{detection['name']} {detection['confidence']:.2f} "
                f"center=({center_x:.1f},{center_y:.1f})")
        self.get_logger().info(
            f"frame={self.frame_index} detections={len(detections)}; "
            + '; '.join(details))

    def _draw_detections(self, frame, detections):
        for detection in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in detection['bbox_xyxy']]
            center_x, center_y = [int(round(value)) for value in detection['center']]
            label = f"{detection['name']} {detection['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
            cv2.putText(
                frame, label, (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if not detections:
            cv2.putText(
                frame, 'no object detected', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return frame

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        if self.show_image:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ModelDetector()
    exit_code = 0
    try:
        if node.ready:
            rclpy.spin(node)
        else:
            exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
    return exit_code


if __name__ == '__main__':
    main()
