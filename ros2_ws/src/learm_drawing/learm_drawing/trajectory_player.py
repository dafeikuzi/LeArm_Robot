import math
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from learm_driver.srv import MovePose
from rclpy.node import Node


JOINT_COUNT = 5
MIN_DURATION_MS = 20
MAX_DURATION_MS = 30000
JOINT_LIMIT_DEG = 90.0


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


def default_share_file(*parts):
    return str(Path(get_package_share_directory('learm_drawing'), *parts))


def load_yaml_file(path):
    try:
        with Path(path).expanduser().open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except OSError as exc:
        raise ValueError(f'cannot read {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ValueError(f'cannot parse {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return data


def require_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a number')
    return float(value)


def parse_joints_deg(value, label):
    if not isinstance(value, list) or len(value) != JOINT_COUNT:
        raise ValueError(f'{label}.joints_deg must contain {JOINT_COUNT} values')
    joints = [require_number(item, f'{label}.joints_deg[{index}]') for index, item in enumerate(value)]
    for index, angle in enumerate(joints):
        if abs(angle) > JOINT_LIMIT_DEG:
            raise ValueError(
                f'{label}.joints_deg[{index}]={angle} exceeds +/-{JOINT_LIMIT_DEG} deg')
    return joints


def parse_opening(value, label):
    opening = require_number(value, label)
    if not 0.0 <= opening <= 1.0:
        raise ValueError(f'{label} must be between 0.0 and 1.0')
    return opening


def parse_duration_ms(value, label):
    duration = int(require_number(value, label))
    if duration < MIN_DURATION_MS or duration > MAX_DURATION_MS:
        raise ValueError(f'{label} must be between {MIN_DURATION_MS} and {MAX_DURATION_MS} ms')
    return duration


def parse_wait_after_ms(value, label):
    wait_after = int(require_number(value, label))
    if wait_after < 0 or wait_after > 60000:
        raise ValueError(f'{label} must be between 0 and 60000 ms')
    return wait_after


@dataclass(frozen=True)
class Pose:
    name: str
    joints_deg: list[float]
    opening: float


@dataclass(frozen=True)
class Step:
    label: str
    joints_deg: list[float]
    opening: float
    duration_ms: int
    wait_after_ms: int


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('learm_trajectory_player')

        self.pose_file = self.declare_parameter(
            'pose_file',
            default_share_file('config', 'drawing_poses.template.yaml')).value
        self.trajectory_file = self.declare_parameter(
            'trajectory_file',
            default_share_file('trajectories', 'phase2_pen_tap.yaml')).value
        self.pose_service_name = self.declare_parameter(
            'pose_service', '/learm_driver/move_pose').value
        self.dry_run = parameter_bool(self.declare_parameter('dry_run', True).value)
        self.repeat = int(self.declare_parameter('repeat', 1).value)
        self.service_wait_timeout_s = float(
            self.declare_parameter('service_wait_timeout_s', 10.0).value)
        self.service_call_timeout_s = float(
            self.declare_parameter('service_call_timeout_s', 5.0).value)

        self.pose_client = self.create_client(MovePose, self.pose_service_name)

    def run(self):
        try:
            plan_name, calibration_required, steps = self._load_plan()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False

        if calibration_required and not self.dry_run:
            self.get_logger().error(
                'This pose/trajectory template still has requires_user_calibration=true. '
                'Edit the pose values and set it to false before running dry_run:=false.')
            return False

        if self.repeat < 1:
            self.get_logger().error('repeat must be at least 1')
            return False

        self._log_plan(plan_name, steps)
        if self.dry_run:
            self.get_logger().info('dry_run=true, no commands were sent to the arm.')
            return True

        if not self.pose_client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error(f'service not available: {self.pose_service_name}')
            return False

        for cycle_index in range(self.repeat):
            self.get_logger().info(f'starting cycle {cycle_index + 1}/{self.repeat}: {plan_name}')
            for step_index, step in enumerate(steps):
                if not self._send_step(step_index, step):
                    return False
        self.get_logger().info(f'completed {plan_name}')
        return True

    def _load_plan(self):
        pose_document = load_yaml_file(self.pose_file)
        trajectory_document = load_yaml_file(self.trajectory_file)
        poses = self._parse_poses(pose_document)
        steps = self._parse_steps(trajectory_document, poses)
        plan_name = str(trajectory_document.get('name') or Path(self.trajectory_file).stem)
        calibration_required = bool(pose_document.get('requires_user_calibration', False)) or bool(
            trajectory_document.get('requires_user_calibration', False))
        return plan_name, calibration_required, steps

    def _parse_poses(self, document):
        default_opening = parse_opening(document.get('default_opening', 0.0), 'default_opening')
        raw_poses = document.get('poses')
        if not isinstance(raw_poses, dict) or not raw_poses:
            raise ValueError('pose file must define a non-empty poses mapping')

        poses = {}
        for name, raw_pose in raw_poses.items():
            label = f'poses.{name}'
            if not isinstance(raw_pose, dict):
                raise ValueError(f'{label} must be a mapping')
            joints_deg = parse_joints_deg(raw_pose.get('joints_deg'), label)
            opening = parse_opening(raw_pose.get('opening', default_opening), f'{label}.opening')
            poses[str(name)] = Pose(str(name), joints_deg, opening)
        return poses

    def _parse_steps(self, document, poses):
        defaults = document.get('defaults') or {}
        if not isinstance(defaults, dict):
            raise ValueError('trajectory defaults must be a mapping')
        default_duration = parse_duration_ms(defaults.get('duration_ms', 1000), 'defaults.duration_ms')
        default_wait_after = parse_wait_after_ms(
            defaults.get('wait_after_ms', 200), 'defaults.wait_after_ms')
        default_opening = parse_opening(defaults.get('opening', 0.0), 'defaults.opening')

        raw_steps = document.get('steps')
        if not isinstance(raw_steps, list) or not raw_steps:
            raise ValueError('trajectory file must define a non-empty steps list')

        steps = []
        for index, raw_step in enumerate(raw_steps):
            label = f'steps[{index}]'
            if not isinstance(raw_step, dict):
                raise ValueError(f'{label} must be a mapping')

            pose_name = raw_step.get('pose')
            pose = None
            if pose_name is not None:
                pose = poses.get(str(pose_name))
                if pose is None:
                    raise ValueError(f'{label}.pose references unknown pose: {pose_name}')

            if 'joints_deg' in raw_step:
                joints_deg = parse_joints_deg(raw_step['joints_deg'], label)
            elif pose is not None:
                joints_deg = list(pose.joints_deg)
            else:
                raise ValueError(f'{label} must define pose or joints_deg')

            opening_source = pose.opening if pose is not None else default_opening
            opening = parse_opening(raw_step.get('opening', opening_source), f'{label}.opening')
            duration = parse_duration_ms(
                raw_step.get('duration_ms', default_duration), f'{label}.duration_ms')
            wait_after = parse_wait_after_ms(
                raw_step.get('wait_after_ms', default_wait_after), f'{label}.wait_after_ms')
            step_label = str(raw_step.get('label') or pose_name or f'step_{index + 1}')
            steps.append(Step(step_label, joints_deg, opening, duration, wait_after))
        return steps

    def _log_plan(self, plan_name, steps):
        self.get_logger().info(
            f'loaded {plan_name}: steps={len(steps)}, repeat={self.repeat}, dry_run={self.dry_run}')
        for index, step in enumerate(steps):
            joints = ', '.join(f'{value:.1f}' for value in step.joints_deg)
            self.get_logger().info(
                f'{index + 1:02d}. {step.label}: joints_deg=[{joints}], '
                f'opening={step.opening:.2f}, duration_ms={step.duration_ms}, '
                f'wait_after_ms={step.wait_after_ms}')

    def _send_step(self, step_index, step):
        request = MovePose.Request()
        request.opening = step.opening
        request.positions_rad = [math.radians(value) for value in step.joints_deg]
        request.duration_ms = step.duration_ms

        self.get_logger().info(f'sending step {step_index + 1}: {step.label}')
        future = self.pose_client.call_async(request)
        if not self._wait_future(future):
            self.get_logger().error(f'step {step_index + 1} timed out: {step.label}')
            return False

        response = future.result()
        if response is None or not response.success:
            error_code = getattr(response, 'error_code', 0)
            message = getattr(response, 'message', 'no response')
            self.get_logger().error(
                f'step {step_index + 1} rejected ({error_code}): {message}')
            return False

        self._sleep_ms(step.duration_ms + step.wait_after_ms)
        return True

    def _wait_future(self, future):
        deadline = time.monotonic() + self.service_call_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _sleep_ms(self, duration_ms):
        deadline = time.monotonic() + duration_ms / 1000.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))


def main():
    rclpy.init()
    node = TrajectoryPlayer()
    success = False
    try:
        success = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    main()
