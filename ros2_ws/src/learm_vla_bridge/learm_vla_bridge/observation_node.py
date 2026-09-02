import json
import math
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from learm_driver.srv import GetArmStatus


JOINT_NAMES = ('joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6')


class ObservationNode(Node):
    """Publishes camera frames and calibrated, command-derived arm observations."""

    def __init__(self):
        super().__init__('learm_vla_observation')
        self.camera_device = self.declare_parameter('camera_device', '/dev/video0').value
        self.camera_backend = self.declare_parameter('camera_backend', 'v4l2').value
        self.pixel_format = self.declare_parameter('pixel_format', 'MJPG').value
        self.frame_width = int(self.declare_parameter('frame_width', 640).value)
        self.frame_height = int(self.declare_parameter('frame_height', 480).value)
        self.camera_fps = int(self.declare_parameter('camera_fps', 15).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 5.0).value)
        self.status_poll_rate_hz = float(
            self.declare_parameter('status_poll_rate_hz', 2.0).value)
        self.status_service_name = self.declare_parameter(
            'status_service', '/learm_driver/get_status').value
        self.task = self.declare_parameter('task', '').value
        self.frame_id = self.declare_parameter('frame_id', 'learm_camera').value
        self.reopen_interval_s = float(self.declare_parameter('reopen_interval_s', 2.0).value)

        self.gripper_closed_pulse_us = int(
            self.declare_parameter('gripper.closed_pulse_us', 1400).value)
        self.gripper_open_pulse_us = int(
            self.declare_parameter('gripper.open_pulse_us', 500).value)
        self.joint_calibration = {
            name: {
                'min_position_rad': float(
                    self.declare_parameter(f'{name}.min_position_rad', -math.pi / 2.0).value),
                'max_position_rad': float(
                    self.declare_parameter(f'{name}.max_position_rad', math.pi / 2.0).value),
                'pulse_at_min_position_us': int(
                    self.declare_parameter(f'{name}.pulse_at_min_position_us', 500).value),
                'pulse_at_max_position_us': int(
                    self.declare_parameter(f'{name}.pulse_at_max_position_us', 2500).value),
            }
            for name in JOINT_NAMES
        }

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '~/image_raw', 10)
        self.observation_publisher = self.create_publisher(String, '~/observation', 10)
        self.status_client = self.create_client(GetArmStatus, self.status_service_name)
        self.status_future = None
        self.last_status = None
        self.capture = None
        self.last_open_attempt = 0.0
        self.last_camera_warning = 0.0

        image_period_s = 1.0 / max(self.publish_rate_hz, 0.1)
        status_period_s = 1.0 / max(self.status_poll_rate_hz, 0.1)
        self.image_timer = self.create_timer(image_period_s, self._publish_frame)
        self.status_timer = self.create_timer(status_period_s, self._request_status)

    def _open_capture(self):
        now = time.monotonic()
        if now - self.last_open_attempt < self.reopen_interval_s:
            return False
        self.last_open_attempt = now

        backend = cv2.CAP_V4L2 if self.camera_backend == 'v4l2' else cv2.CAP_ANY
        source = int(self.camera_device) if str(self.camera_device).isdigit() else self.camera_device
        capture = cv2.VideoCapture(source, backend)
        if self.pixel_format:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format[:4]))
        if self.frame_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if self.camera_fps > 0:
            capture.set(cv2.CAP_PROP_FPS, self.camera_fps)

        if not capture.isOpened():
            capture.release()
            self.get_logger().warn(f'Cannot open camera {self.camera_device}')
            return False

        self.capture = capture
        self.get_logger().info(
            f'Opened camera {self.camera_device} at '
            f'{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
        return True

    def _request_status(self):
        if self.status_future is not None and not self.status_future.done():
            return
        if not self.status_client.service_is_ready():
            return

        self.status_future = self.status_client.call_async(GetArmStatus.Request())
        self.status_future.add_done_callback(self._receive_status)

    def _receive_status(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Cannot read arm status: {exc}')
            return
        if response is None or not response.success:
            message = response.message if response is not None else 'empty response'
            self.get_logger().warn(f'Arm status request failed: {message}')
            return
        self.last_status = response

    def _publish_frame(self):
        if self.capture is None and not self._open_capture():
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            now = time.monotonic()
            if now - self.last_camera_warning >= self.reopen_interval_s:
                self.last_camera_warning = now
                self.get_logger().warn('Camera frame read failed; reopening camera')
            self.capture.release()
            self.capture = None
            return

        timestamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = timestamp
        image.header.frame_id = self.frame_id
        self.image_publisher.publish(image)
        self._publish_observation(timestamp)

    def _publish_observation(self, timestamp):
        status = self.last_status
        payload = {
            'timestamp': {
                'sec': timestamp.sec,
                'nanosec': timestamp.nanosec,
            },
            'task': self.task,
            'state_source': 'stm32_pwm_estimate',
            'state_valid': status is not None,
        }
        if status is not None:
            current_pulses = [int(value) for value in status.current_pulse_us]
            target_pulses = [int(value) for value in status.target_pulse_us]
            current_state = self._pulses_to_state(current_pulses)
            target_state = self._pulses_to_state(target_pulses)
            payload.update({
                'moving': bool(status.moving),
                'estop_active': bool(status.estop_active),
                'moving_mask': int(status.moving_mask),
                'current_pulse_us': current_pulses,
                'target_pulse_us': target_pulses,
                'state': current_state,
                'target_state': target_state,
                'state_valid': current_state is not None and target_state is not None,
            })

        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.observation_publisher.publish(message)

    def _pulses_to_state(self, pulses):
        joints_rad = []
        for index, name in enumerate(JOINT_NAMES, start=1):
            calibration = self.joint_calibration[name]
            pulse_at_min = calibration['pulse_at_min_position_us']
            pulse_at_max = calibration['pulse_at_max_position_us']
            if pulse_at_min == pulse_at_max:
                return None
            fraction = (pulses[index] - pulse_at_min) / (pulse_at_max - pulse_at_min)
            position = calibration['min_position_rad'] + fraction * (
                calibration['max_position_rad'] - calibration['min_position_rad'])
            joints_rad.append(round(position, 6))

        gripper_span = self.gripper_open_pulse_us - self.gripper_closed_pulse_us
        if gripper_span == 0:
            return None
        opening = (pulses[0] - self.gripper_closed_pulse_us) / gripper_span
        return {
            'joint_positions_rad': joints_rad,
            'gripper_opening': round(max(0.0, min(1.0, opening)), 6),
        }

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ObservationNode()
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
