import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COLOR_PRESETS = {
    'red': (0, 120, 80, 10, 255, 255),
    'green': (35, 80, 60, 85, 255, 255),
    'blue': (90, 80, 60, 130, 255, 255),
    'yellow': (18, 80, 80, 38, 255, 255),
}


def _bool_parameter(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ColorTracker(Node):
    def __init__(self):
        super().__init__('learm_color_tracker')

        self.camera_index = int(self.declare_parameter('camera_index', 0).value)
        self.camera_device = str(self.declare_parameter('camera_device', '').value)
        self.camera_backend = str(self.declare_parameter('camera_backend', 'v4l2').value)
        self.pixel_format = str(self.declare_parameter('pixel_format', 'MJPG').value)
        self.frame_width = int(self.declare_parameter('frame_width', 1920).value)
        self.frame_height = int(self.declare_parameter('frame_height', 1080).value)
        self.fps = float(self.declare_parameter('fps', 30).value)
        self.processing_rate_hz = float(self.declare_parameter('processing_rate_hz', 0.0).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 5.0).value)
        self.stale_timeout_sec = float(self.declare_parameter('stale_timeout_sec', 1.0).value)
        self.reopen_interval_sec = float(self.declare_parameter('reopen_interval_sec', 2.0).value)
        self.read_timeout_ms = int(self.declare_parameter('read_timeout_ms', 1000).value)
        self.drop_corrupt_frames = _bool_parameter(self.declare_parameter('drop_corrupt_frames', True).value)
        self.target_color = str(self.declare_parameter('target_color', 'red').value)
        self.min_area = float(self.declare_parameter('min_area', 800.0).value)
        self.show_mask = _bool_parameter(self.declare_parameter('show_mask', False).value)
        self.preview_width = int(self.declare_parameter('preview_width', 960).value)
        self.preview_height = int(self.declare_parameter('preview_height', 540).value)

        self.publisher = self.create_publisher(String, '~/object_detection', 10)
        self.capture = None
        self.capture_thread = None
        self.capture_running = False
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_time = 0.0
        self.last_frame_time = 0.0
        self.capture_fps = 0.0
        self.last_read_warning_time = 0.0
        self.last_reopen_time = 0.0
        self.last_status_warning_time = 0.0
        self.bad_frame_count = 0
        self.last_publish_time = 0.0

        self.window_name = 'LeArm Vision'
        self.controls_name = 'HSV Controls'
        self.mask_name = 'LeArm Mask'
        self.timer_rate_hz = self.processing_rate_hz if self.processing_rate_hz > 0 else max(self.fps, 30.0)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.preview_width, self.preview_height)
        cv2.namedWindow(self.controls_name, cv2.WINDOW_NORMAL)
        self._create_trackbars()

        self._open_camera()
        self._start_capture_thread()
        self.timer = self.create_timer(1.0 / max(self.timer_rate_hz, 1.0), self._process_frame)

    def _create_trackbars(self):
        preset = COLOR_PRESETS.get(self.target_color, COLOR_PRESETS['red'])
        labels = ('H low', 'S low', 'V low', 'H high', 'S high', 'V high')
        maximums = (179, 255, 255, 179, 255, 255)
        for label, value, maximum in zip(labels, preset, maximums):
            cv2.createTrackbar(label, self.controls_name, int(value), maximum, lambda _value: None)

    def _open_camera(self):
        backend = cv2.CAP_V4L2 if self.camera_backend == 'v4l2' else cv2.CAP_ANY
        source_candidates = []
        if self.camera_device:
            source_candidates.append((self.camera_device, backend))
            source_candidates.append((self.camera_device, cv2.CAP_ANY))
            if self.camera_device.startswith('/dev/video'):
                suffix = self.camera_device.rsplit('video', 1)[-1]
                if suffix.isdigit():
                    index = int(suffix)
                    source_candidates.append((index, backend))
                    source_candidates.append((index, cv2.CAP_ANY))
        else:
            source_candidates.append((int(self.camera_index), backend))
            source_candidates.append((int(self.camera_index), cv2.CAP_ANY))

        self.capture = None
        used_source = None
        used_backend = None
        for source, candidate_backend in source_candidates:
            capture = cv2.VideoCapture(source, candidate_backend)
            if capture.isOpened():
                self.capture = capture
                used_source = source
                used_backend = candidate_backend
                break
            capture.release()

        if self.capture is None:
            self.get_logger().error(
                f'Cannot open camera {self.camera_device or self.camera_index}. '
                f'Check /dev/video* or VM USB camera passthrough.')
            return

        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC') and self.read_timeout_ms > 0:
            self.capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms)
        if self.pixel_format:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format[:4]))
        if self.frame_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.frame_width))
        if self.frame_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.frame_height))
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, int(self.fps))

        self.get_logger().info(
            f'Opened camera {used_source} with {self.pixel_format} '
            f'backend={int(used_backend)} '
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
            f'@ {self.capture.get(cv2.CAP_PROP_FPS):.1f} fps, '
            f'processing {self.timer_rate_hz:.1f} Hz')
        self.last_reopen_time = time.monotonic()

    def _reopen_camera(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self._open_camera()

    def _start_capture_thread(self):
        self.capture_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    def _capture_loop(self):
        while self.capture_running:
            if self.capture is None or not self.capture.isOpened():
                now = time.monotonic()
                self._maybe_reopen_camera(now)
                time.sleep(0.1)
                continue

            ok, frame = self.capture.read()
            now = time.monotonic()
            if ok and frame is not None and frame.size > 0:
                if self.drop_corrupt_frames and self._looks_corrupt(frame):
                    self.bad_frame_count += 1
                    if self.bad_frame_count == 1 or self.bad_frame_count % 30 == 0:
                        self.get_logger().warn('Dropped corrupt camera frame')
                    self._maybe_reopen_camera(now)
                    time.sleep(0.01)
                    continue

                self.bad_frame_count = 0
                if self.last_frame_time > 0.0:
                    instant_fps = 1.0 / max(now - self.last_frame_time, 1e-6)
                    self.capture_fps = instant_fps if self.capture_fps <= 0.0 else (
                        0.9 * self.capture_fps + 0.1 * instant_fps)
                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_frame_time = now
                self.last_frame_time = now
                continue

            if now - self.last_read_warning_time > 2.0:
                self.last_read_warning_time = now
                self.get_logger().warn('Camera frame read failed')
            self._maybe_reopen_camera(now)
            time.sleep(0.01)

    def _maybe_reopen_camera(self, now):
        if self.last_reopen_time <= 0.0:
            self.last_reopen_time = now
            return
        if now - self.last_reopen_time < self.reopen_interval_sec:
            return
        self.get_logger().warn('Reopening camera after stalled or corrupt frames')
        self.last_reopen_time = now
        self._reopen_camera()

    def _looks_corrupt(self, frame):
        if frame.ndim != 3 or frame.shape[2] != 3:
            return True
        height, width = frame.shape[:2]
        if height < 32 or width < 32:
            return True

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        split = int(height * 0.55)
        top = gray[:split]
        bottom = gray[split:]
        if top.size == 0 or bottom.size == 0:
            return False

        top_mean = float(top.mean())
        bottom_mean = float(bottom.mean())
        bottom_std = float(bottom.std())
        if bottom_std < 4.0 and (abs(bottom_mean - 128.0) < 10.0 or bottom_mean < 8.0) and abs(top_mean - bottom_mean) > 20.0:
            return True

        row_mean = gray.mean(axis=1)
        row_std = gray.std(axis=1)
        flat_rows = np.logical_and(
            row_std < 4.0,
            np.logical_or(np.abs(row_mean - 128.0) < 10.0, row_mean < 8.0),
        )
        return float(flat_rows.mean()) > 0.25

    def _get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None, 0.0
            return self.latest_frame.copy(), self.latest_frame_time

    def _threshold_values(self):
        return (
            cv2.getTrackbarPos('H low', self.controls_name),
            cv2.getTrackbarPos('S low', self.controls_name),
            cv2.getTrackbarPos('V low', self.controls_name),
            cv2.getTrackbarPos('H high', self.controls_name),
            cv2.getTrackbarPos('S high', self.controls_name),
            cv2.getTrackbarPos('V high', self.controls_name),
        )

    def _build_mask(self, hsv):
        h_low, s_low, v_low, h_high, s_high, v_high = self._threshold_values()
        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, s_high, v_high], dtype=np.uint8)

        if h_low <= h_high:
            mask = cv2.inRange(hsv, lower, upper)
        else:
            low_wrap = np.array([0, s_low, v_low], dtype=np.uint8)
            high_wrap = np.array([179, s_high, v_high], dtype=np.uint8)
            mask = cv2.bitwise_or(cv2.inRange(hsv, lower, high_wrap), cv2.inRange(hsv, low_wrap, upper))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _process_frame(self):
        frame, frame_time = self._get_latest_frame()
        if frame is None:
            now = time.monotonic()
            if now - self.last_status_warning_time > 2.0:
                self.last_status_warning_time = now
                self.get_logger().warn('Waiting for a valid camera frame')
            self._show_status_frame('waiting for valid camera frame')
            self._publish_detection(False)
            cv2.waitKey(1)
            return

        now = time.monotonic()
        stale_age = now - frame_time
        stale = frame_time <= 0.0 or stale_age > self.stale_timeout_sec
        if stale and now - self.last_status_warning_time > 2.0:
            self.last_status_warning_time = now
            self.get_logger().warn(f'Camera frame is stale by {stale_age:.1f}s')

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv)
        detection = self._detect_largest_object(mask)

        if detection is not None:
            x, y, width, height, area = detection
            center_x = x + width / 2.0
            center_y = y + height / 2.0
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.circle(frame, (round(center_x), round(center_y)), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f'object x={center_x:.0f} y={center_y:.0f} area={area:.0f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            self._publish_detection(True, center_x, center_y, area, width, height)
        else:
            cv2.putText(
                frame,
                'object not found',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            self._publish_detection(False)

        if stale:
            cv2.putText(
                frame,
                f'camera stale {stale_age:.1f}s',
                (10, frame.shape[0] - 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )
        elif self.capture_fps > 0.0:
            cv2.putText(
                frame,
                f'{self.capture_fps:.1f} fps',
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
        cv2.imshow(self.window_name, self._preview_frame(frame))
        if self.show_mask:
            cv2.imshow(self.mask_name, self._preview_frame(mask))
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            rclpy.shutdown()

    def _show_status_frame(self, text):
        height = max(self.frame_height, 240)
        width = max(self.frame_width, 320)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            text,
            (10, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )
        cv2.imshow(self.window_name, self._preview_frame(frame))
        if self.show_mask:
            blank = np.zeros((height, width), dtype=np.uint8)
            cv2.imshow(self.mask_name, self._preview_frame(blank))

    def _preview_frame(self, frame):
        if self.preview_width <= 0 or self.preview_height <= 0:
            return frame
        return cv2.resize(
            frame,
            (self.preview_width, self.preview_height),
            interpolation=cv2.INTER_LINEAR,
        )

    def _detect_largest_object(self, mask):
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_area:
            return None

        x, y, width, height = cv2.boundingRect(contour)
        return x, y, width, height, area

    def _publish_detection(
        self,
        detected,
        center_x=0.0,
        center_y=0.0,
        area=0.0,
        width=0,
        height=0,
    ):
        now = time.monotonic()
        publish_period = 1.0 / max(self.publish_rate_hz, 0.1)
        if now - self.last_publish_time < publish_period:
            return
        self.last_publish_time = now
        message = String()
        message.data = json.dumps({
            'detected': bool(detected),
            'x': round(float(center_x), 2),
            'y': round(float(center_y), 2),
            'area': round(float(area), 2),
            'width': int(width),
            'height': int(height),
        })
        self.publisher.publish(message)

    def destroy_node(self):
        self.capture_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=1.0)
        if self.capture is not None:
            self.capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ColorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
