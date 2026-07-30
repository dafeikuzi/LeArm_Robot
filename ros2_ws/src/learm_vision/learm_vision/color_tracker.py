import json
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


class ColorTracker(Node):
    def __init__(self):
        super().__init__('learm_color_tracker')

        self.camera_index = self.declare_parameter('camera_index', 0).value
        self.camera_device = self.declare_parameter('camera_device', '').value
        self.camera_backend = self.declare_parameter('camera_backend', 'v4l2').value
        self.pixel_format = self.declare_parameter('pixel_format', 'MJPG').value
        self.frame_width = self.declare_parameter('frame_width', 320).value
        self.frame_height = self.declare_parameter('frame_height', 240).value
        self.fps = self.declare_parameter('fps', 15).value
        self.target_color = self.declare_parameter('target_color', 'red').value
        self.min_area = float(self.declare_parameter('min_area', 800.0).value)
        self.show_mask = bool(self.declare_parameter('show_mask', True).value)

        self.publisher = self.create_publisher(String, '~/object_detection', 10)
        self.capture = None
        self.last_publish_time = 0.0

        self.window_name = 'LeArm Vision'
        self.controls_name = 'HSV Controls'
        self.mask_name = 'LeArm Mask'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.controls_name, cv2.WINDOW_NORMAL)
        self._create_trackbars()

        self._open_camera()
        self.timer = self.create_timer(1.0 / 30.0, self._process_frame)

    def _create_trackbars(self):
        preset = COLOR_PRESETS.get(self.target_color, COLOR_PRESETS['red'])
        labels = ('H low', 'S low', 'V low', 'H high', 'S high', 'V high')
        maximums = (179, 255, 255, 179, 255, 255)
        for label, value, maximum in zip(labels, preset, maximums):
            cv2.createTrackbar(label, self.controls_name, int(value), maximum, lambda _value: None)

    def _open_camera(self):
        source = self.camera_device if self.camera_device else int(self.camera_index)
        backend = cv2.CAP_V4L2 if self.camera_backend == 'v4l2' else cv2.CAP_ANY
        self.capture = cv2.VideoCapture(source, backend)
        if self.pixel_format:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format[:4]))
        if self.frame_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.frame_width))
        if self.frame_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.frame_height))
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, int(self.fps))

        if not self.capture.isOpened():
            self.get_logger().error(
                f'Cannot open camera {source}. Check /dev/video* or VM USB camera passthrough.')
            return
        self.get_logger().info(
            f'Opened camera {source} with {self.pixel_format} '
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

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
        if self.capture is None or not self.capture.isOpened():
            self._publish_detection(False)
            cv2.waitKey(1)
            return

        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warn('Camera frame read failed')
            self._publish_detection(False)
            cv2.waitKey(1)
            return

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

        cv2.imshow(self.window_name, frame)
        if self.show_mask:
            cv2.imshow(self.mask_name, mask)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            rclpy.shutdown()

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
        if now - self.last_publish_time < 0.1:
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
