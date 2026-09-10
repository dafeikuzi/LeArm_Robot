import socket
import sys
import threading
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/learm_vla_bridge'))

import windows_hardware_bridge as windows_bridge
from learm_vla_bridge.network_camera_node import NetworkCameraClient


class CameraProtocolTests(unittest.TestCase):
    def setUp(self):
        self.client_socket, self.server_socket = socket.socketpair()
        self.client_socket.settimeout(1.0)
        self.server_socket.settimeout(1.0)
        self.client = NetworkCameraClient('unused', 0, 1.0, 2 * 1024 * 1024)
        self.client.socket = self.client_socket

    def tearDown(self):
        self.client.close()
        self.server_socket.close()

    def test_fragmented_header_and_payload_are_reassembled(self):
        frame = np.zeros((24, 32, 3), dtype=np.uint8)
        frame[:, :, 1] = 200
        ok, encoded = cv2.imencode('.jpg', frame)
        self.assertTrue(ok)
        jpeg = encoded.tobytes()
        packet = windows_bridge.CAMERA_HEADER.pack(
            windows_bridge.CAMERA_MAGIC, windows_bridge.CAMERA_VERSION, 0,
            32, 24, 7, 123456, len(jpeg),
        ) + jpeg

        def serve():
            self.assertEqual(self.server_socket.recv(4), windows_bridge.CAMERA_REQUEST)
            for offset in range(0, len(packet), 13):
                self.server_socket.sendall(packet[offset:offset + 13])

        thread = threading.Thread(target=serve)
        thread.start()
        sequence, captured_ns, decoded = self.client.read()
        thread.join(timeout=1.0)

        self.assertEqual(sequence, 7)
        self.assertEqual(captured_ns, 123456)
        self.assertEqual(decoded.shape, (24, 32, 3))

    def test_oversized_payload_is_rejected_before_body_read(self):
        header = windows_bridge.CAMERA_HEADER.pack(
            windows_bridge.CAMERA_MAGIC, windows_bridge.CAMERA_VERSION, 0,
            640, 480, 1, 1, 2 * 1024 * 1024 + 1,
        )

        def serve():
            self.server_socket.recv(4)
            self.server_socket.sendall(header)

        thread = threading.Thread(target=serve)
        thread.start()
        with self.assertRaisesRegex(ValueError, 'too large'):
            self.client.read()
        thread.join(timeout=1.0)

    def test_windows_latest_frame_replaces_old_frame(self):
        latest = windows_bridge.LatestCameraFrame()
        first = np.zeros((10, 12, 3), dtype=np.uint8)
        second = np.full((8, 9, 3), 255, dtype=np.uint8)
        latest.update(first, 80)
        first_sequence = latest.snapshot()[0]
        latest.update(second, 80)
        sequence, captured_ns, width, height, jpeg = latest.snapshot()
        self.assertEqual(sequence, first_sequence + 1)
        self.assertGreater(captured_ns, 0)
        self.assertEqual((width, height), (9, 8))
        self.assertLessEqual(len(jpeg), windows_bridge.MAX_JPEG_BYTES)

if __name__ == '__main__':
    unittest.main()
