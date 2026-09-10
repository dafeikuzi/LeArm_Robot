#!/usr/bin/env python3
"""Run a locally fine-tuned SmolVLA policy against the LeArm ROS driver.

The default mode is a read-only dry run: it reads the camera and arm state,
performs policy inference, and prints the proposed action without moving the
robot. Add ``--allow-motion`` only after the dry run and a small workspace
test have been checked.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from learm_driver.srv import GetArmStatus, MovePose
from lerobot.configs import PreTrainedConfig
from lerobot.policies import make_pre_post_processors
from lerobot.policies.smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame, make_robot_action


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CHECKPOINT = ROOT / "Smolvla_trianing/20260909_102253/outputs/smolvla_learm/checkpoints/last/pretrained_model"
DEFAULT_VLM = ROOT / "Smolvla_trianing/20260909_102253/hf-cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct/snapshots/7b375e1b73b11138ff12fe22c8f2822d8fe03467"
DATASET_JOINT_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5")
ROS_JOINT_NAMES = ("joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
ACTION_NAMES = DATASET_JOINT_NAMES + ("gripper",)
DEFAULT_SERVICE_TIMEOUT_S = 35.0
DEFAULT_RETRY_DELAY_S = 1.0
DEFAULT_MAX_COMMUNICATION_FAILURES = 5
TRANSIENT_DRIVER_ERROR_CODES = frozenset((100, 101))
FEATURES = {
    "observation.images.camera": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channel"],
    },
    "observation.state": {
        "dtype": "float32",
        "shape": (6,),
        "names": list(ACTION_NAMES),
    },
    "action": {
        "dtype": "float32",
        "shape": (6,),
        "names": list(ACTION_NAMES),
    },
}


class CommunicationError(RuntimeError):
    """A temporary ROS/serial failure that may recover without restarting rollout."""

    def __init__(self, operation: str, detail: str, *, result_unknown: bool = False) -> None:
        super().__init__(detail)
        self.operation = operation
        self.result_unknown = result_unknown


def pulse_to_state(pulses: list[int]) -> np.ndarray:
    """Convert driver pulses [gripper, joint_2..joint_6] to training units."""
    if len(pulses) != 6:
        raise ValueError(f"driver returned {len(pulses)} pulses, expected 6")
    joints = [((float(pulses[i]) - 500.0) / 2000.0) * math.pi - math.pi / 2.0 for i in range(1, 6)]
    opening = (float(pulses[0]) - 1400.0) / (500.0 - 1400.0)
    return np.asarray(joints + [max(0.0, min(1.0, opening))], dtype=np.float32)


def wait_for_ros_future(node: Node, future, timeout_s: float, operation: str):
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    except Exception as exc:
        raise CommunicationError(operation, f"ROS service call failed: {exc}") from exc
    if not future.done():
        future.cancel()
        raise CommunicationError(operation, f"timed out waiting for /learm_driver/{operation}")
    try:
        response = future.result()
    except Exception as exc:
        raise CommunicationError(operation, f"ROS service call failed: {exc}") from exc
    if response is None:
        raise CommunicationError(operation, f"empty response from /learm_driver/{operation}")
    return response


def require_successful_response(response, operation: str, *, result_unknown: bool = False):
    if response.success:
        return response
    detail = f"{operation} failed ({response.error_code}): {response.message}"
    if response.error_code in TRANSIENT_DRIVER_ERROR_CODES:
        raise CommunicationError(operation, detail, result_unknown=result_unknown)
    raise RuntimeError(detail)


class ArmClient(Node):
    def __init__(self, service_timeout_s: float) -> None:
        super().__init__("smolvla_learm_rollout")
        self.service_timeout_s = service_timeout_s
        self.status_client = self.create_client(GetArmStatus, "/learm_driver/get_status")
        self.pose_client = self.create_client(MovePose, "/learm_driver/move_pose")
        self.estop_client = self.create_client(Trigger, "/learm_driver/emergency_stop")

    def wait_for_services(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        for client, name in (
            (self.status_client, "/learm_driver/get_status"),
            (self.pose_client, "/learm_driver/move_pose"),
        ):
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(f"ROS service is unavailable: {name}")

    def get_status(self):
        try:
            future = self.status_client.call_async(GetArmStatus.Request())
        except Exception as exc:
            raise CommunicationError("get_status", f"could not call /learm_driver/get_status: {exc}") from exc
        response = wait_for_ros_future(self, future, self.service_timeout_s, "get_status")
        return require_successful_response(response, "get_status")

    def move_pose(self, action: dict[str, float], duration_ms: int) -> str:
        request = MovePose.Request()
        request.opening = max(0.0, min(1.0, float(action["gripper"])))
        request.positions_rad = [
            max(-math.pi / 2.0, min(math.pi / 2.0, float(action[name])))
            for name in DATASET_JOINT_NAMES
        ]
        request.duration_ms = max(100, min(5000, int(duration_ms)))
        try:
            future = self.pose_client.call_async(request)
        except Exception as exc:
            raise CommunicationError(
                "move_pose", f"could not call /learm_driver/move_pose: {exc}", result_unknown=True
            ) from exc
        try:
            response = wait_for_ros_future(self, future, self.service_timeout_s, "move_pose")
        except CommunicationError as exc:
            raise CommunicationError("move_pose", str(exc), result_unknown=True) from exc
        return require_successful_response(response, "move_pose", result_unknown=True).message

    def emergency_stop(self) -> None:
        if not self.estop_client.wait_for_service(timeout_sec=0.5):
            return
        future = self.estop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


class ImageBuffer:
    """Keep the newest ROS image so rollout never queues stale camera frames."""

    def __init__(self, node: Node, topic: str, max_age_s: float) -> None:
        self.latest: np.ndarray | None = None
        self.received_at: float | None = None
        self.max_age_s = max_age_s
        self.subscription = node.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, message: Image) -> None:
        try:
            channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(message.encoding)
            if channels is None:
                raise ValueError(f"unsupported ROS image encoding: {message.encoding}")
            array = np.frombuffer(message.data, dtype=np.uint8)
            array = array.reshape((message.height, message.step // channels, channels))[:, : message.width]
            if message.encoding == "rgb8":
                self.latest = array.copy()
            elif message.encoding == "bgr8":
                self.latest = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
            elif message.encoding == "rgba8":
                self.latest = cv2.cvtColor(array, cv2.COLOR_RGBA2RGB)
            elif message.encoding == "bgra8":
                self.latest = cv2.cvtColor(array, cv2.COLOR_BGRA2RGB)
            else:
                self.latest = cv2.cvtColor(array, cv2.COLOR_GRAY2RGB)
            self.received_at = time.monotonic()
        except Exception:
            self.latest = None
            self.received_at = None

    def read(self, node: Node, timeout_s: float = 2.0) -> np.ndarray:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if (self.latest is not None and self.received_at is not None and
                    time.monotonic() - self.received_at <= self.max_age_s):
                return self.latest.copy()
            rclpy.spin_once(node, timeout_sec=0.1)
        if self.latest is None or self.received_at is None:
            raise RuntimeError("no image received from ROS image topic")
        age_s = time.monotonic() - self.received_at
        if age_s > self.max_age_s:
            raise RuntimeError(
                f"latest ROS image is stale ({age_s:.2f}s > {self.max_age_s:.2f}s); motion stopped"
            )
        return self.latest.copy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--vlm-model", type=Path, default=DEFAULT_VLM)
    parser.add_argument("--task", required=True, help="task text used during training")
    parser.add_argument(
        "--camera-source", choices=("topic", "v4l2"), default="topic",
        help="use the Windows TCP camera ROS topic or a native Linux V4L2 device",
    )
    parser.add_argument("--camera", default="/dev/video0", help="V4L2 fallback device")
    parser.add_argument(
        "--image-topic", default="/learm_network_camera/image_raw",
        help="ROS Image topic used when --camera-source=topic",
    )
    parser.add_argument(
        "--image-max-age-s", type=float, default=2.0,
        help="stop before inference when the newest ROS image is older than this",
    )
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--hz", type=float, default=8.0, help="control rate; training data was recorded at 8 Hz")
    parser.add_argument("--duration-ms", type=int, default=125, help="duration of each MovePose command")
    parser.add_argument("--max-steps", type=int, default=0, help="stop after N steps; 0 means run until Ctrl-C")
    parser.add_argument(
        "--service-timeout-s",
        type=float,
        default=DEFAULT_SERVICE_TIMEOUT_S,
        help="seconds to wait for each ROS driver service response",
    )
    parser.add_argument(
        "--retry-delay-s",
        type=float,
        default=DEFAULT_RETRY_DELAY_S,
        help="seconds to wait after a temporary communication failure",
    )
    parser.add_argument(
        "--max-communication-failures",
        type=int,
        default=DEFAULT_MAX_COMMUNICATION_FAILURES,
        help="stop after this many consecutive temporary communication failures",
    )
    parser.add_argument("--allow-motion", action="store_true", help="enable MovePose commands")
    parser.add_argument("--estop-on-exit", action="store_true", help="request emergency stop when Ctrl-C exits")
    args = parser.parse_args()
    if args.service_timeout_s <= 0:
        parser.error("--service-timeout-s must be greater than 0")
    if args.retry_delay_s < 0:
        parser.error("--retry-delay-s must be at least 0")
    if args.max_communication_failures < 1:
        parser.error("--max-communication-failures must be at least 1")
    if args.image_max_age_s <= 0:
        parser.error("--image-max-age-s must be greater than 0")
    return args


def open_camera(device: str) -> cv2.VideoCapture:
    source = int(device) if device.isdigit() else device
    capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv2.CAP_PROP_FPS, 30)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"cannot open camera: {device}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (width, height) != (640, 480):
        print(f"warning: camera negotiated {width}x{height}; model expects 640x480")
    return capture


def load_policy(checkpoint: Path, vlm_model: Path, device: torch.device):
    if not checkpoint.is_dir():
        raise FileNotFoundError(f"checkpoint directory not found: {checkpoint}")
    if not vlm_model.is_dir():
        raise FileNotFoundError(f"local VLM directory not found: {vlm_model}")
    config = PreTrainedConfig.from_pretrained(str(checkpoint))
    config.vlm_model_name = str(vlm_model)
    config.device = str(device)
    policy = SmolVLAPolicy.from_pretrained(str(checkpoint), config=config)
    preprocess, postprocess = make_pre_post_processors(
        policy.config,
        str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": str(device)}},
    )
    policy.reset()
    return policy, preprocess, postprocess


def format_state(state: np.ndarray) -> str:
    joints = ", ".join(
        f"{ros_name}={math.degrees(float(state[index])):+.1f}deg"
        for index, ros_name in enumerate(ROS_JOINT_NAMES)
    )
    return f"{joints}, gripper={state[5] * 100:.1f}%"


def format_action(action: dict[str, float]) -> str:
    joints = ", ".join(
        f"{ros_name}={math.degrees(float(action[dataset_name])):+.1f}deg"
        for dataset_name, ros_name in zip(DATASET_JOINT_NAMES, ROS_JOINT_NAMES)
    )
    return f"{joints}, gripper={action['gripper'] * 100:.1f}%"


def infer_action(policy, preprocess, postprocess, device: torch.device, rgb: np.ndarray, state: np.ndarray,
                 task: str) -> dict[str, float]:
    raw_observation = {"camera": rgb}
    raw_observation.update(dict(zip(ACTION_NAMES, state.tolist())))
    frame = build_inference_frame(raw_observation, device, FEATURES, task=task, robot_type="learm")
    action = postprocess(policy.select_action(preprocess(frame)))
    robot_action = make_robot_action(action, FEATURES)
    return {
        **{
            name: max(-math.pi / 2.0, min(math.pi / 2.0, float(robot_action[name])))
            for name in DATASET_JOINT_NAMES
        },
        "gripper": max(0.0, min(1.0, float(robot_action["gripper"]))),
    }


def read_camera_frame(capture) -> np.ndarray:
    ok, bgr = capture.read()
    if not ok or bgr is None:
        raise RuntimeError("camera frame read failed")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def run_rollout_loop(args: argparse.Namespace, arm: ArmClient, policy, preprocess, postprocess,
                     device: torch.device, capture, image_buffer: ImageBuffer | None) -> int:
    period = 1.0 / max(0.1, args.hz)
    next_tick = time.monotonic()
    steps = 0
    communication_failures = 0

    def record_communication_failure(error: CommunicationError) -> bool:
        nonlocal communication_failures, next_tick
        communication_failures += 1
        policy.reset()
        result_note = "; command result is unknown, original action will not be retried" \
            if error.result_unknown else ""
        print(
            f"warning: {error}{result_note} "
            f"(failure {communication_failures}/{args.max_communication_failures})",
            flush=True,
        )
        if communication_failures >= args.max_communication_failures:
            print(
                f"fatal: communication failed {communication_failures} consecutive times; rollout stopped\n"
                "check the Windows TCP bridge, restart learm_driver, then verify get_status",
                flush=True,
            )
            return False
        time.sleep(args.retry_delay_s)
        next_tick = time.monotonic()
        return True

    while args.max_steps <= 0 or steps < args.max_steps:
        try:
            status = arm.get_status()
        except CommunicationError as error:
            if not record_communication_failure(error):
                return 1
            continue

        if bool(status.estop_active):
            raise RuntimeError("driver reports emergency stop active; clear it before rollout")

        if image_buffer is not None:
            rgb = image_buffer.read(arm)
        else:
            rgb = read_camera_frame(capture)
        state = pulse_to_state([int(v) for v in status.current_pulse_us])
        action_dict = infer_action(policy, preprocess, postprocess, device, rgb, state, args.task)

        if args.allow_motion:
            try:
                message = arm.move_pose(action_dict, args.duration_ms)
            except CommunicationError as error:
                if not record_communication_failure(error):
                    return 1
                try:
                    recovered = arm.get_status()
                except CommunicationError as recovery_error:
                    print(f"recovery: status unavailable: {recovery_error}", flush=True)
                    continue
                if bool(recovered.estop_active):
                    raise RuntimeError("driver reports emergency stop active during communication recovery")
                current = pulse_to_state([int(v) for v in recovered.current_pulse_us])
                target = pulse_to_state([int(v) for v in recovered.target_pulse_us])
                print(
                    f"recovery: status received, moving={str(bool(recovered.moving)).lower()}, "
                    f"estop=false, current=[{format_state(current)}], target=[{format_state(target)}]; "
                    "policy action queue reset",
                    flush=True,
                )
                next_tick = time.monotonic()
                continue
        else:
            message = "dry-run"

        steps += 1
        communication_failures = 0
        print(
            f"step={steps} current=[{format_state(state)}] "
            f"target=[{format_action(action_dict)}] {message}",
            flush=True,
        )
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))
    return 0


def main() -> int:
    args = parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device=cuda requested, but torch.cuda.is_available() is false")
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device if args.device != "auto" else "cpu")
    print(f"loading checkpoint: {args.checkpoint}")
    print(f"inference device: {device}")
    policy, preprocess, postprocess = load_policy(args.checkpoint, args.vlm_model, device)
    rclpy.init()
    arm = ArmClient(args.service_timeout_s)
    capture = None
    image_buffer = (
        ImageBuffer(arm, args.image_topic, args.image_max_age_s)
        if args.camera_source == "topic" else None
    )
    try:
        arm.wait_for_services(args.service_timeout_s)
        if image_buffer is None:
            capture = open_camera(args.camera)
        print("ROS services and camera are ready")
        if not args.allow_motion:
            print("DRY RUN: no servo commands will be sent (use --allow-motion to enable movement)")
        return run_rollout_loop(args, arm, policy, preprocess, postprocess, device, capture, image_buffer)
    except KeyboardInterrupt:
        print("\nrollout stopped")
    finally:
        if args.estop_on_exit and args.allow_motion:
            arm.emergency_stop()
        arm.destroy_node()
        rclpy.shutdown()
        if capture is not None:
            capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
