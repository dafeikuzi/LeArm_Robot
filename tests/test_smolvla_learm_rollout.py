import argparse
import contextlib
import io
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import smolvla_learm_rollout as rollout


ACTION = {
    "joint_1": 0.0,
    "joint_2": -0.1,
    "joint_3": -0.2,
    "joint_4": 0.1,
    "joint_5": -0.3,
    "gripper": 0.8,
}


def make_status(*, estop=False, moving=False):
    return SimpleNamespace(
        estop_active=estop,
        moving=moving,
        current_pulse_us=[1000, 1500, 1400, 1300, 1200, 1100],
        target_pulse_us=[1000, 1500, 1400, 1300, 1200, 1100],
    )


def make_args(**overrides):
    values = {
        "hz": 1.0,
        "max_steps": 1,
        "allow_motion": True,
        "duration_ms": 500,
        "retry_delay_s": 0.0,
        "max_communication_failures": 5,
        "task": "test task",
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class FakeCapture:
    def read(self):
        return True, np.zeros((480, 640, 3), dtype=np.uint8)


class FakePolicy:
    def __init__(self):
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


class FakeArm:
    def __init__(self, status_results, move_results):
        self.status_results = iter(status_results)
        self.move_results = iter(move_results)
        self.events = []

    def get_status(self):
        self.events.append("status")
        result = next(self.status_results)
        if isinstance(result, Exception):
            raise result
        return result

    def move_pose(self, _action, _duration_ms):
        self.events.append("move")
        result = next(self.move_results)
        if isinstance(result, Exception):
            raise result
        return result


class RolloutRecoveryTests(unittest.TestCase):
    def run_loop(self, arm, policy, args=None):
        with mock.patch.object(rollout, "infer_action", return_value=ACTION), \
                mock.patch.object(rollout.time, "sleep"):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                result = rollout.run_rollout_loop(
                    args or make_args(), arm, policy, None, None, "cpu", FakeCapture(), None
                )
        return result, output.getvalue()

    def test_move_timeout_checks_status_before_next_action(self):
        timeout = rollout.CommunicationError(
            "move_pose", "timed out waiting for /learm_driver/move_pose", result_unknown=True
        )
        arm = FakeArm(
            [make_status(), make_status(moving=True), make_status()],
            [timeout, "accepted"],
        )
        policy = FakePolicy()

        result, output = self.run_loop(arm, policy)

        self.assertEqual(result, 0)
        self.assertEqual(arm.events, ["status", "move", "status", "status", "move"])
        self.assertEqual(policy.reset_count, 1)
        self.assertIn("original action will not be retried", output)
        self.assertIn("recovery: status received", output)

    def test_fifth_consecutive_status_failure_stops_rollout(self):
        failures = [
            rollout.CommunicationError("get_status", "STM32 response timed out")
            for _ in range(5)
        ]
        arm = FakeArm(failures, [])
        policy = FakePolicy()

        result, output = self.run_loop(arm, policy)

        self.assertEqual(result, 1)
        self.assertEqual(arm.events, ["status"] * 5)
        self.assertEqual(policy.reset_count, 5)
        self.assertIn("communication failed 5 consecutive times", output)

    def test_four_move_failures_then_ack_recovers(self):
        failures = [
            rollout.CommunicationError("move_pose", "STM32 response timed out", result_unknown=True)
            for _ in range(4)
        ]
        arm = FakeArm([make_status()] * 9, failures + ["accepted"])
        policy = FakePolicy()

        result, output = self.run_loop(arm, policy)

        self.assertEqual(result, 0)
        self.assertEqual(arm.events.count("move"), 5)
        self.assertEqual(policy.reset_count, 4)
        self.assertIn("failure 4/5", output)
        self.assertNotIn("fatal:", output)

    def test_noncommunication_error_is_not_retried(self):
        arm = FakeArm([RuntimeError("invalid calibration")], [])
        policy = FakePolicy()

        with self.assertRaisesRegex(RuntimeError, "invalid calibration"):
            self.run_loop(arm, policy)
        self.assertEqual(policy.reset_count, 0)


class ServiceTimeoutTests(unittest.TestCase):
    def test_timeout_cancels_future_and_uses_requested_duration(self):
        future = mock.Mock()
        future.done.return_value = False

        with mock.patch.object(rollout.rclpy, "spin_until_future_complete") as spin:
            with self.assertRaises(rollout.CommunicationError):
                rollout.wait_for_ros_future(object(), future, 35.0, "move_pose")

        spin.assert_called_once_with(mock.ANY, future, timeout_sec=35.0)
        future.cancel.assert_called_once_with()

    def test_driver_transport_and_timeout_errors_are_recoverable(self):
        for error_code in (100, 101):
            response = SimpleNamespace(success=False, error_code=error_code, message="temporary failure")
            with self.assertRaises(rollout.CommunicationError):
                rollout.require_successful_response(response, "get_status")

    def test_driver_configuration_error_is_fatal(self):
        response = SimpleNamespace(success=False, error_code=103, message="invalid calibration")
        with self.assertRaisesRegex(RuntimeError, "invalid calibration"):
            rollout.require_successful_response(response, "move_pose", result_unknown=True)


class ImageFreshnessTests(unittest.TestCase):
    def test_stale_topic_frame_stops_inference(self):
        buffer = rollout.ImageBuffer.__new__(rollout.ImageBuffer)
        buffer.latest = np.zeros((480, 640, 3), dtype=np.uint8)
        buffer.received_at = time.monotonic() - 2.1
        buffer.max_age_s = 2.0
        with self.assertRaisesRegex(RuntimeError, "stale"):
            buffer.read(None, timeout_s=0.0)


if __name__ == "__main__":
    unittest.main()
