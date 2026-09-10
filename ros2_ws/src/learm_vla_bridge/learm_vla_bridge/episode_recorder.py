import json
import re
from datetime import datetime, timezone
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


EPISODE_PATTERN = re.compile(r'^episode_(\d{6})$')


class EpisodeRecorder(Node):
    """Records manually demonstrated LeArm observations without issuing commands."""

    def __init__(self):
        super().__init__('learm_vla_recorder')
        self.image_topic = self.declare_parameter(
            'image_topic', '/learm_vla_observation/image_raw').value
        self.observation_topic = self.declare_parameter(
            'observation_topic', '/learm_vla_observation/observation').value
        self.dataset_root = Path(self.declare_parameter(
            'dataset_root', '/home/liuzhiwei/LeArm_Robot/datasets/learm_vla/raw').value).expanduser()
        self.record_rate_hz = float(self.declare_parameter('record_rate_hz', 5.0).value)
        self.max_frame_skew_s = float(self.declare_parameter('max_frame_skew_s', 0.3).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 95).value)
        self.action_duration_ms = int(self.declare_parameter('action_duration_ms', 1000).value)
        self.require_task = bool(self.declare_parameter('require_task', True).value)
        self.task_override = self.declare_parameter('task_override', '').value.strip()

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_observation = None
        self.recording = False
        self.episode_dir = None
        self.frames_dir = None
        self.records_path = None
        self.frame_index = 0
        self.last_record_timestamp_ns = None
        self.episode_task = ''
        self.episode_started_at = None
        self.episode_outcome = 'in_progress'
        self.episode_status_timeout_baseline = 0
        self.status_timeout_count = 0

        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self._on_image, 10)
        self.observation_subscription = self.create_subscription(
            String, self.observation_topic, self._on_observation, 10)
        self.start_service = self.create_service(Trigger, '~/start_episode', self._start_episode)
        self.stop_service = self.create_service(Trigger, '~/stop_episode', self._stop_episode)
        self.abort_service = self.create_service(Trigger, '~/abort_episode', self._abort_episode)

    def _on_image(self, message):
        self.latest_image = message

    def _on_observation(self, message):
        try:
            observation = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid observation JSON: {exc}')
            return

        self.latest_observation = observation
        if not self.recording:
            return
        self._update_status_health(observation)
        if not observation.get('stm32_status_ok', False):
            return
        if not observation.get('state_valid', False):
            return
        if observation.get('estop_active', False):
            self.get_logger().warn('Skipping sample while the software emergency stop is active')
            return
        if not self._has_valid_action(observation):
            self.get_logger().warn('Skipping observation without a valid target action')
            return
        if self.latest_image is None:
            return

        timestamp_ns = self._timestamp_to_ns(observation.get('timestamp'))
        image_timestamp_ns = self._image_timestamp_to_ns(self.latest_image)
        if timestamp_ns is None or image_timestamp_ns is None:
            return
        if abs(timestamp_ns - image_timestamp_ns) > int(self.max_frame_skew_s * 1_000_000_000):
            return
        if self.last_record_timestamp_ns is not None:
            minimum_delta_ns = int(1_000_000_000 / max(self.record_rate_hz, 0.1))
            if timestamp_ns - self.last_record_timestamp_ns < minimum_delta_ns:
                return

        self._write_sample(observation, timestamp_ns)

    def _start_episode(self, _request, response):
        if self.recording:
            response.success = False
            response.message = f'episode already active: {self.episode_dir.name}'
            return response

        if self.latest_image is None:
            response.success = False
            response.message = 'waiting for an image from the observation node'
            return response
        if self.latest_observation is None or not self.latest_observation.get('state_valid', False):
            response.success = False
            response.message = 'waiting for a valid arm observation and target pose'
            return response
        if not self.latest_observation.get('stm32_status_ok', False):
            response.success = False
            response.message = 'waiting for a successful STM32 status response'
            return response
        if not self._has_valid_action(self.latest_observation):
            response.success = False
            response.message = 'waiting for a valid target action from the arm status'
            return response
        self.dataset_root.mkdir(parents=True, exist_ok=True)
        self.episode_task = self.task_override
        if self.require_task and not self.episode_task:
            response.success = False
            response.message = 'task_override must be set before starting an episode'
            return response

        episode_index = self._next_episode_index()
        self.episode_dir = self.dataset_root / f'episode_{episode_index:06d}'
        self.frames_dir = self.episode_dir / 'frames'
        self.episode_dir.mkdir(parents=False, exist_ok=False)
        self.frames_dir.mkdir()
        self.records_path = self.episode_dir / 'records.jsonl'
        self.frame_index = 0
        self.last_record_timestamp_ns = None
        self.episode_started_at = datetime.now(timezone.utc).isoformat()
        self.episode_outcome = 'in_progress'
        baseline = self.latest_observation.get('status_timeout_count', 0)
        self.episode_status_timeout_baseline = baseline if (
            isinstance(baseline, int) and baseline >= 0) else 0
        self.status_timeout_count = 0
        self.recording = True
        self._write_metadata(completed=False)
        response.success = True
        response.message = f'recording {self.episode_dir.name} for task: {self.episode_task}'
        self.get_logger().info(response.message)
        return response

    def _stop_episode(self, _request, response):
        if not self.recording:
            response.success = False
            response.message = 'no active episode'
            return response

        episode_name, sample_count = self._finish_episode('success')
        response.success = True
        response.message = f'completed {episode_name} with {sample_count} samples'
        self.get_logger().info(response.message)
        return response

    def _abort_episode(self, _request, response):
        if not self.recording:
            response.success = False
            response.message = 'no active episode'
            return response

        episode_name, sample_count = self._finish_episode('aborted')
        response.success = True
        response.message = f'aborted {episode_name}; retained {sample_count} samples for review'
        self.get_logger().info(response.message)
        return response

    def _finish_episode(self, outcome):
        episode_name = self.episode_dir.name
        sample_count = self.frame_index
        self.episode_outcome = outcome
        self._write_metadata(completed=outcome == 'success')
        self.recording = False
        self.episode_dir = None
        self.frames_dir = None
        self.records_path = None
        self.last_record_timestamp_ns = None
        self.episode_started_at = None
        self.episode_status_timeout_baseline = 0
        return episode_name, sample_count

    def _write_sample(self, observation, timestamp_ns):
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            filename = f'{self.frame_index:06d}.jpg'
            image_path = self.frames_dir / filename
            written = cv2.imwrite(
                str(image_path), frame,
                [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, self.jpeg_quality))])
            if not written:
                raise OSError(f'cannot write {image_path}')

            record = {
                'schema_version': 1,
                'frame_index': self.frame_index,
                'timestamp_ns': timestamp_ns,
                'image': f'frames/{filename}',
                'task': self.episode_task,
                'observation': observation,
                'action': observation['target_state'],
                'action_duration_ms': self.action_duration_ms,
            }
            with self.records_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, separators=(',', ':')) + '\n')
            self.frame_index += 1
            self.last_record_timestamp_ns = timestamp_ns
        except (cv2.error, OSError, ValueError) as exc:
            self.get_logger().error(f'Cannot record sample: {exc}')

    def _write_metadata(self, completed):
        metadata = {
            'schema_version': 1,
            'created_at': self.episode_started_at,
            'updated_at': datetime.now(timezone.utc).isoformat(),
            'task': self.episode_task,
            'sample_count': self.frame_index,
            'completed': completed,
            'outcome': self.episode_outcome,
            'image_topic': self.image_topic,
            'observation_topic': self.observation_topic,
            'action_duration_ms': self.action_duration_ms,
            'state_source': 'stm32_pwm_estimate',
            'quality': {
                'trainable': completed,
                'status': 'ok' if completed else 'incomplete',
                'status_timeout_count': self.status_timeout_count,
            },
        }
        (self.episode_dir / 'metadata.json').write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8')

    def _next_episode_index(self):
        last_index = 0
        for path in self.dataset_root.iterdir():
            if not path.is_dir():
                continue
            match = EPISODE_PATTERN.match(path.name)
            if match:
                last_index = max(last_index, int(match.group(1)))
        return last_index + 1

    def _has_valid_action(self, observation):
        action = observation.get('target_state')
        if not isinstance(action, dict):
            return False
        positions = action.get('joint_positions_rad')
        opening = action.get('gripper_opening')
        return (
            isinstance(positions, list) and len(positions) == 5 and
            all(isinstance(value, (int, float)) for value in positions) and
            isinstance(opening, (int, float))
        )

    def _update_status_health(self, observation):
        timeout_count = observation.get('status_timeout_count', 0)
        if isinstance(timeout_count, int) and timeout_count >= 0:
            episode_count = max(timeout_count - self.episode_status_timeout_baseline, 0)
            self.status_timeout_count = max(self.status_timeout_count, episode_count)

    @staticmethod
    def _timestamp_to_ns(timestamp):
        if not isinstance(timestamp, dict):
            return None
        sec = timestamp.get('sec')
        nanosec = timestamp.get('nanosec')
        if not isinstance(sec, int) or not isinstance(nanosec, int):
            return None
        return sec * 1_000_000_000 + nanosec

    @staticmethod
    def _image_timestamp_to_ns(image):
        timestamp = image.header.stamp
        if timestamp.sec == 0 and timestamp.nanosec == 0:
            return None
        return timestamp.sec * 1_000_000_000 + timestamp.nanosec

    def destroy_node(self):
        if self.recording:
            self.episode_outcome = 'interrupted'
            self._write_metadata(completed=False)
        super().destroy_node()


def main():
    rclpy.init()
    node = EpisodeRecorder()
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
